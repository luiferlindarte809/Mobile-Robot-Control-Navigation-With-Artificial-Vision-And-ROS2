#!/usr/bin/env python3
import rclpy
import numpy as np
import math
import csv
import os
from datetime import datetime
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Int32
from std_msgs.msg import Float32MultiArray, String, Int16MultiArray
from geometry_msgs.msg import Vector3

class ControlNode(Node):
    def __init__(self):
        super().__init__('Control')

        # Parámetros
        self.declare_parameter('rate_hz', 10)
        rate_hz = float(self.get_parameter('rate_hz').get_parameter_value().double_value or 10.0)
        self.period = 1.0 / rate_hz

        # QoS para sensores (best-effort suele ir bien)
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # ---- Subs a sensores ----

        # /selector
        self.selec_control = 0
        self.create_subscription(Int32, '/selector', self.on_selec, sensor_qos)

        # /Distancia: Float32MultiArray -> [Min_proy_I, Min_proy_D]
        self.dist_left = 0.0
        self.dist_right = 0.0
        self.create_subscription(Float32MultiArray, '/Distancia', self.on_dist, sensor_qos)
        
        # /Senal
        self.Senal = "Stop"
        self.create_subscription(String, '/Senal', self.on_Senal, sensor_qos)

        # /BNO_pub: Vector3 -> x=pitch, y=roll, z=yaw (grados)
        self.yaw_rad = 0.0
        self.create_subscription(Vector3, '/BNO_pub', self.on_bno, sensor_qos)

        # /Sens_Car_ESP/m{1..4}: Float32MultiArray -> [ref, vel, u]
        self.mot = [[0.0, 0.0, 0.0] for _ in range(4)]
        for i in range(4):
            topic = f'/Sens_Car_ESP/m{i+1}'
            self.create_subscription(Float32MultiArray, topic, self._make_on_motor(i), sensor_qos)

        # ---- Pubs a la ESP y BNO ----
        self.pub_cmd_R = self.create_publisher(Float32MultiArray, '/Sens_Car_ESP/cmd_R', 10)
        self.pub_bno_reset = self.create_publisher(String, '/BNO_reset', 10)

        # Timer principal
        self.timer = self.create_timer(self.period, self.on_timer)
        self.get_logger().info(f'Control iniciado @ {rate_hz:.1f} Hz')

        # Variables de control
        self.x = [0.0, 0.0]
        self.u = [0.0, 0.0]
        self.xhat = [0.0, 0.0]

        # Maniobra
        r = 0.0225  # radio de rueda (m)
        b = 0.0651  # semi-distancia entre ruedas (m)
        c = r / (2.0 * b)  # relación cinemática
        self.M = np.array([[r/2.0, r/2.0], [c, -c]], dtype=float)
        self.inv_M = np.array([[1.0/r, 1.0/(2.0*c)], [1.0/r, -1.0/(2.0*c)]], dtype=float)
        
        self.Map = np.array([0.0, 0.0, 0.0], dtype=float) # [x y theta]
        self.Map_Graf = np.array([0.0, 0.0, 0.0], dtype=float) # [x y theta]
        self.ref_dot = np.array([[0.0], [0.0]], dtype=float) # derivadas de xref e yref
        self.ref_MAP = np.array([[0.0], [0.0]], dtype=float)
        self.err = np.array([[0.0], [0.0]], dtype=float)
        self.count = 0
        self.yaw_Graf = 0.0
        self.sentido = 0.0
        self.ref_completa = np.array([[0.0], [0.0]], dtype=float)
        self.sum_ref_lin = 0.0
        
        ref_dir = "/home/pi/logs_ros2/Referencias"
        try:
            self.ref_Der = np.loadtxt(os.path.join(ref_dir, "ref_Der.csv"), delimiter=",")
            self.ref_Izq = np.loadtxt(os.path.join(ref_dir, "ref_Izq.csv"), delimiter=",")
            self.get_logger().info(f"Referencias cargadas: Der shape {self.ref_Der.shape}")
        except Exception as e:
            self.get_logger().error(f"Error cargando referencias: {e}")
            # Crear referencias dummy para evitar crash
            self.ref_Der = np.zeros((2, 200))
            self.ref_Der = np.zeros((2, 180))
        
        self.xh = 0.0
        self.yh = 0.0
        self.selec_control_anterior = 0
        self.senal_anterior = "Stop"
        
        # ---- Logging (CSV) ----
        # Parámetros configurables
        self.declare_parameter('log_enabled', True)
        self.declare_parameter('log_dir', os.path.expanduser('~/logs_ros2/control'))
        self.declare_parameter('log_prefix', 'control')
        self.declare_parameter('max_rows', 300000)     # ~8.3 horas a 10 Hz
        self.declare_parameter('flush_every', 20)      # fsync cada 20 filas (~2 s)
        self.declare_parameter('decimate_n', 1)        # 1 = loguear cada ciclo

        self.log_enabled  = bool(self.get_parameter('log_enabled').value)
        self.log_dir      = str(self.get_parameter('log_dir').value)
        self.log_prefix   = str(self.get_parameter('log_prefix').value)
        self.max_rows     = int(self.get_parameter('max_rows').value)
        self.flush_every  = int(self.get_parameter('flush_every').value)
        self.decimate_n   = int(self.get_parameter('decimate_n').value)

        self._row_count   = 0
        self._since_flush = 0
        self._dec_i       = 0
        self.log_fh = None
        self.log_writer = None
        self.log_path = None

        # Reloj para 't_s' relativo (x de tu gráfica)
        self.t0 = self.get_clock().now()
        # Guardar Dist como atributo para el logger
        self.Dist = 0.0

        self._init_logger()

    # --------- Callbacks ----------
    def on_dist(self, msg: Float32MultiArray):
        if len(msg.data) >= 2:
            self.dist_left = float(msg.data[0])/1000
            self.dist_right = float(msg.data[1])/1000

    def on_bno(self, msg: Vector3):
        # Yaw es el más importante para ti
        self.yaw_rad = float(msg.z)*math.pi/180
    
    def on_selec(self, msg: Int32):
        # Acepta cualquier entero; normaliza a {0,1} si lo quieres binario
        try:
            self.selec_control = int(msg.data)
            if self.selec_control in [2, 4, 6] or self.selec_control != self.selec_control_anterior:
                self.xhat = [0.0, 0.0]
            if self.selec_control != self.selec_control_anterior:
                if self.selec_control in [7, 8]:
                    self.origen_x = self.Map[0]
                    self.origen_y = self.Map[1]
                    self.origen_x_Graf = self.Map_Graf[0]
                    self.count = 0
        except Exception:
            self.get_logger().warn(f'/selector inválido: {msg}')
            return

    def on_Senal(self, msg: String):
        if str(msg.data) != "none" and str(msg.data) != "Red Light" and str(msg.data) != self.Senal:
            self.Senal = str(msg.data)
            self.xhat = [0.0, 0.0]
            try:
                if self.Senal in ["Go straight", "Speed Limit 60", "Speed Limit 80"] or self.Senal != self.senal_anterior:
                    self.xhat = [0.0, 0.0]
                if self.Senal != self.senal_anterior:
                    if self.Senal in ["Turn left", "Turn right"]:
                        self.origen_x = self.Map[0]
                        self.origen_y = self.Map[1]
                        self.origen_x_Graf = self.Map_Graf[0]
                        self.count = 0
            except Exception:
                self.get_logger().warn(f'/selector inválido: {msg}')
                return

    def _make_on_motor(self, idx):
        def _cb(msg: Float32MultiArray):
            if len(msg.data) >= 3:
                self.mot[idx][0] = float(msg.data[0])  # ref (m/s)
                self.mot[idx][1] = float(msg.data[1])  # vel (m/s)
                self.mot[idx][2] = float(msg.data[2])  # u   (-100..100)
        return _cb
        
    def _init_logger(self):
        if not self.log_enabled:
            self.get_logger().info("💾 Logging deshabilitado (log_enabled=false).")
            return
        os.makedirs(self.log_dir, exist_ok=True)
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.log_path = os.path.join(self.log_dir, f"{self.log_prefix}_{ts}.csv")
        self.log_fh = open(self.log_path, 'w', newline='')
        self.log_writer = csv.writer(self.log_fh)
        # Cabecera
        self.log_writer.writerow([
            'stamp_sec', 't_s', 'x_Graf','y_Graf', 'yaw_Graf','x_ref_Graf','y_ref_Graf', 'x','y','psi','x_ref','y_ref', 'xh', 'yh', 'err_x', 'err_y', 'Dist_LIDAR', 'yaw_rad',
            'u0', 'u1', 'vel_left', 'vel_right',
            'xhat0', 'xhat1', 'selec_control', 'Señal'
        ])
        self.get_logger().info(f"💾 Log en: {self.log_path}")

    def _log_row(self):
        if not self.log_writer:
            return
        # Decimación (si quieres loguear cada N ciclos)
        self._dec_i += 1
        if self._dec_i % self.decimate_n != 0:
            return

        now = self.get_clock().now()
        t_s = (now - self.t0).nanoseconds / 1e9
        stamp_sec = now.nanoseconds / 1e9

        # Asegura xhat como 2 floats
        try:
            xh = np.array(self.xhat, dtype=float).reshape(-1).tolist()
        except Exception:
            xh = [float('nan'), float('nan')]
        if len(xh) < 2:
            xh = (xh + [float('nan'), float('nan')])[:2]

        row = [
            f"{stamp_sec:.6f}",
            f"{t_s:.3f}",
            float(self.Map_Graf[0]),
            float(self.Map_Graf[1]),
            float(self.Map_Graf[2]),
            float(self.ref_completa[0]),
            float(self.ref_completa[1]),
            float(self.Map[0]),
            float(self.Map[1]),
            float(self.Map[2]),
            float(self.ref_MAP[0]),
            float(self.ref_MAP[1]),
            float(self.xh),
            float(self.yh),
            float(self.err[0]),
            float(self.err[1]),
            float(self.x[0]),
            float(self.x[1]),
            float(self.u[0]),
            float(self.u[1]),
            float(self.mot[0][1]),
            float(self.mot[2][1]),
            float(xh[0]),
            float(xh[1]),
            int(self.selec_control),
            str(self.Senal),
        ]
        self.log_writer.writerow(row)
        self._row_count   += 1
        self._since_flush += 1

        # fsync periódico para minimizar pérdida en apagón, pero no castigar la SD
        if self._since_flush >= self.flush_every:
            self.log_fh.flush()
            os.fsync(self.log_fh.fileno())
            self._since_flush = 0

        # Rotura simple al alcanzar límite
        if self._row_count >= self.max_rows:
            self.get_logger().warn("💾 max_rows alcanzado; cerrando archivo de log.")
            try:
                self.log_fh.flush()
                os.fsync(self.log_fh.fileno())
            finally:
                self.log_fh.close()
                self.log_fh = None
                self.log_writer = None

    # Cierre limpio del logger al destruir el nodo
    def destroy_node(self):
        try:
            if self.log_fh:
                self.log_fh.flush()
                os.fsync(self.log_fh.fileno())
                self.log_fh.close()
                self.get_logger().info(f"💾 Log cerrado: {self.log_path}")
        finally:
            super().destroy_node()
    
    # --------- Lazo periódico ----------
    def on_timer(self):
        # Referencias 
        wi_ref = [4.444, 8.889, 13.333] # 2.222rad/s = 0.05m/s
        wd_ref = [4.444, 8.889, 13.333]

        ref = 0.3
	
        # referencia simple
#	if self.dist_med == 0:
        if self.dist_left == 0.0:
            self.Dist = ref
        else:
            self.Dist = self.dist_left - ref

        # Odometria
        V = (self.mot[0][1] + self.mot[2][1])/2
        omega = (self.mot[0][1] - self.mot[2][1])/0.1302
        
        # Variables de estado
        self.x = [self.Dist, self.yaw_rad]
        
        dt = self.period
        # Cinematica [x y theta]
        self.Map[0] = dt*V*math.cos(self.x[1]) + self.Map[0]
        self.Map[1] = dt*V*math.sin(self.x[1]) + self.Map[1]
        self.Map[2] = dt*omega + self.Map[2]

        # Cinematica para graficar
        self.Map_Graf[0] = dt*V*math.cos(self.x[1] + self.sentido) + self.Map_Graf[0]
        self.Map_Graf[1] = dt*V*math.sin(self.x[1] + self.sentido) + self.Map_Graf[1]
        self.Map_Graf[2] = dt*omega + self.Map[2]
        
        self.selec_control_anterior = self.selec_control
        self.senal_anterior = self.Senal

        #if self.selec_control == 0: #or self.Senal == "Stop":
        if self.Senal == "Stop":
            self.u = [0.0, 0.0]  # ← importante para el log
            ref_motor = Float32MultiArray(data=[0.0, 0.0, 0.0, 0.0])
            self.pub_cmd_R.publish(ref_motor)
            #self.xhat = [0, 0]
            self.ref_completa[0] = self.Map_Graf[0]
            self.ref_completa[1] = self.sum_ref_lin
            
        elif self.selec_control == 1: # LQR 0.1 m/s
            #K = [55.8682, 6.6356, -55.8682, -6.6356]
            K= [254.9009, 18.6389, -254.9009, -18.6389]

            # acciones de control lqr
            self.u[0] = -K[0]*self.x[0] - K[1]*self.x[1] + wi_ref[0]
            self.u[1] = -K[2]*self.x[0] - K[3]*self.x[1] + wd_ref[0]

            self.get_logger().info(
                f"v={(self.mot[0][1]+self.mot[2][1])/2:.3f} m/s | yaw={self.yaw_rad:.1f}rad | dIzq={self.dist_left:.1f} m dDer={self.dist_right:.1f} m"
            )

            # referencias de los motores
            ref_motor = Float32MultiArray(data=[self.u[0]*0.0225, self.u[0]*0.0225, self.u[1]*0.0225, self.u[1]*0.0225])
            self.pub_cmd_R.publish(ref_motor)

        #elif self.selec_control == 2: # LQG 0.1 m/s
        elif self.Senal == "Go straight":
            #K = [55.8682, 6.6356, -55.8682, -6.6356]
            K= [254.9009, 18.6389, -254.9009, -18.6389]
            # Modelo
            #L = np.array([[0.9906],[33.7660]], dtype=float)          # 2x1 (columna)
            L = np.array([[0.2613],[3.0308]], dtype=float)          # 2x1 (columna)
            F = np.array([[1.0, 0.01], [0.0, 1.0]], dtype=float)
            C2 = np.array([[1.0, 0.0]], dtype=float)        # 1x2 (fila)  <-- importante
            G = np.array([[ 0.0001, -0.0001],[ 0.0173, -0.0173]], dtype=float) # 2x2

            # acciones de control lqg
            self.u[0] = -K[0]*self.xhat[0] - K[1]*self.xhat[1] + wi_ref[0]
            self.u[1] = -K[2]*self.xhat[0] - K[3]*self.xhat[1] + wd_ref[0]

                # --- Empaquetar vectores como columnas (2x1) ---
            xhat = np.array(self.xhat, dtype=float).reshape(2, 1)   # 2x1
            uvec = np.array(self.u,   dtype=float).reshape(2, 1)    # 2x1
            xvec = np.array(self.x,   dtype=float).reshape(2, 1)    # 2x1

            self.xhat = (F - L @ C2) @ xhat + G @ uvec + L @ C2 @ xvec

            self.get_logger().info(
                f"v={(self.mot[0][1]+self.mot[2][1])/2:.3f} m/s | yaw={self.yaw_rad:.1f}rad | dIzq={self.dist_left:.1f} m dDer={self.dist_right:.1f} m"
            )

            # referencias de los motores
            ref_motor = Float32MultiArray(data=[self.u[0]*0.0225, self.u[0]*0.0225, self.u[1]*0.0225, self.u[1]*0.0225])
            self.pub_cmd_R.publish(ref_motor)
            self.ref_completa[0] = self.Map_Graf[0]
            self.ref_completa[1] = self.sum_ref_lin

        elif self.selec_control == 3: # LQR 0.2 m/s
            #K = [27.9340, 6.6358, -27.9340, -6.6358]
            K = [122.8037, 18.5257, -122.8037, -18.5257]

            # acciones de control lqr
            self.u[0] = -K[0]*self.x[0] - K[1]*self.x[1] + wi_ref[1]
            self.u[1] = -K[2]*self.x[0] - K[3]*self.x[1] + wd_ref[1]

            self.get_logger().info(
                f"v={(self.mot[0][1]+self.mot[2][1])/2:.3f} m/s | yaw={self.yaw_rad:.1f}rad | dIzq={self.dist_left:.1f} m dDer={self.dist_right:.1f} m"
            )

            # referencias de los motores
            ref_motor = Float32MultiArray(data=[self.u[0]*0.0225, self.u[0]*0.0225, self.u[1]*0.0225, self.u[1]*0.0225])
            self.pub_cmd_R.publish(ref_motor)

        #elif self.selec_control == 4: # LQG 0.2 m/s
        elif self.Senal == "Speed Limit 60":
            #K = [27.9340, 6.6358, -27.9340, -6.6358]
            K = [122.8037, 18.5257, -122.8037, -18.5257]
            # Modelo
            #L = np.array([[1.3193],[27.3439]], dtype=float)          # 2x1 (columna)
            L = np.array([[0.3676],[2.8706]], dtype=float)          # 2x1 (columna)
            F = np.array([[1.0, 0.02], [0.0, 1.0]], dtype=float)
            C2 = np.array([[1.0, 0.0]], dtype=float)        # 1x2 (fila)  <-- importante
            G = np.array([[ 0.0002, -0.0002],[ 0.0173, -0.0173]], dtype=float) # 2x2

            # acciones de control lqg
            self.u[0] = -K[0]*self.xhat[0] - K[1]*self.xhat[1] + wi_ref[1]
            self.u[1] = -K[2]*self.xhat[0] - K[3]*self.xhat[1] + wd_ref[1]

                # --- Empaquetar vectores como columnas (2x1) ---
            xhat = np.array(self.xhat, dtype=float).reshape(2, 1)   # 2x1
            uvec = np.array(self.u,   dtype=float).reshape(2, 1)    # 2x1
            xvec = np.array(self.x,   dtype=float).reshape(2, 1)    # 2x1

            self.xhat = (F - L @ C2) @ xhat + G @ uvec + L @ C2 @ xvec

            self.get_logger().info(
                f"v={(self.mot[0][1]+self.mot[2][1])/2:.3f} m/s | yaw={self.yaw_rad:.1f}rad | dIzq={self.dist_left:.1f} m dDer={self.dist_right:.1f} m"
            )

            # referencias de los motores
            ref_motor = Float32MultiArray(data=[self.u[0]*0.0225, self.u[0]*0.0225, self.u[1]*0.0225, self.u[1]*0.0225])
            self.pub_cmd_R.publish(ref_motor)
            self.ref_completa[0] = self.Map_Graf[0]
            self.ref_completa[1] = self.sum_ref_lin

        elif self.selec_control == 5: # LQR 0.3 m/s
            #K = [18.7336, 6.6497, -18.7336, -6.6497]
            K = [80.9118, 18.4905, -80.9118, -18.4905]

            # acciones de control lqr
            self.u[0] = -K[0]*self.x[0] - K[1]*self.x[1] + wi_ref[2]
            self.u[1] = -K[2]*self.x[0] - K[3]*self.x[1] + wd_ref[2]

            self.get_logger().info(
                f"v={(self.mot[0][1]+self.mot[2][1])/2:.3f} m/s | yaw={self.yaw_rad:.1f}rad | dIzq={self.dist_left:.1f} m dDer={self.dist_right:.1f} m"
            )

            # referencias de los motores
            ref_motor = Float32MultiArray(data=[self.u[0]*0.0225, self.u[0]*0.0225, self.u[1]*0.0225, self.u[1]*0.0225])
            self.pub_cmd_R.publish(ref_motor)

        #elif self.selec_control == 6: # LQG 0.3 m/s
        elif self.Senal == "Speed Limit 80":
            #K = [18.7336, 6.6497, -18.7336, -6.6497]
            K = [80.9118, 18.4905, -80.9118, -18.4905]
            # Modelo
            #L = np.array([[1.5349],[23.3762]], dtype=float)          # 2x1 (columna)
            L = np.array([[0.4478],[2.7538]], dtype=float)          # 2x1 (columna)
            F = np.array([[1.0, 0.03], [0.0, 1.0]], dtype=float)
            C2 = np.array([[1.0, 0.0]], dtype=float)        # 1x2 (fila)  <-- importante
            G = np.array([[ 0.0003, -0.0003],[ 0.0173, -0.0173]], dtype=float) # 2x2

            # acciones de control lqg
            self.u[0] = -K[0]*self.xhat[0] - K[1]*self.xhat[1] + wi_ref[2]
            self.u[1] = -K[2]*self.xhat[0] - K[3]*self.xhat[1] + wd_ref[2]

                # --- Empaquetar vectores como columnas (2x1) ---
            xhat = np.array(self.xhat, dtype=float).reshape(2, 1)   # 2x1
            uvec = np.array(self.u,   dtype=float).reshape(2, 1)    # 2x1
            xvec = np.array(self.x,   dtype=float).reshape(2, 1)    # 2x1

            self.xhat = (F - L @ C2) @ xhat + G @ uvec + L @ C2 @ xvec

            self.get_logger().info(
                f"v={(self.mot[0][1]+self.mot[2][1])/2:.3f} m/s | yaw={self.yaw_rad:.1f}rad | dIzq={self.dist_left:.1f} m dDer={self.dist_right:.1f} m"
            )

            # referencias de los motores
            ref_motor = Float32MultiArray(data=[self.u[0]*0.0225, self.u[0]*0.0225, self.u[1]*0.0225, self.u[1]*0.0225])
            self.pub_cmd_R.publish(ref_motor)
            self.ref_completa[0] = self.Map_Graf[0]
            self.ref_completa[1] = self.sum_ref_lin

        #elif self.selec_control == 7: # Giro Izquierda
        elif self.Senal == "Turn left":

            K_slam = np.array([[0.1, 0], [0, 0.1]])

            theta = self.yaw_rad
            #theta = self.Map[2]
            
            h = 0.02
            self.xh = self.Map[0] + h*dt*math.cos(theta)
            self.yh = self.Map[1] + h*dt*math.sin(theta)
            #self.xh = dt*V*math.cos(theta) - dt*h*omega*math.sin(theta) + self.xh
            #self.yh = dt*V*math.sin(theta) + dt*h*omega*math.cos(theta) + self.yh

            xy_h = np.array([[self.xh],[self.yh]], dtype=float)

            self.ref_MAP = np.array([[self.ref_Izq[0][self.count] + self.origen_x], [self.ref_Izq[1][self.count] + self.origen_y]], dtype=float)
            ref_next = np.array([[self.ref_Izq[0][self.count + 1] + self.origen_x], [self.ref_Izq[1][self.count + 1] + self.origen_y]], dtype=float)
            
            self.ref_dot = (ref_next - self.ref_MAP) / dt

            self.err = self.ref_MAP - xy_h

            J_inv = np.array([[math.cos(theta), math.sin(theta)],[-math.sin(theta)/h, math.cos(theta)/h]], dtype=float)

            U = J_inv @ (self.ref_dot + K_slam @ self.err)

            # acciones de control lqr
            #self.u = self.inv_M @ U
            U_vec = U.reshape(2, 1)
            w_wheels = self.inv_M @ U_vec
            self.u = [float(w_wheels[0, 0]), float(w_wheels[1, 0])]

            #self.get_logger().info(
            #    f"v={V} m/s | yaw={self.x[1]:.1f}rad | y={self.x[0]:.1f} m x={self.Map[0]:.1f} m"
            #)

            # referencias de los motores
            ref_motor = Float32MultiArray(data=[self.u[0]*0.0225, self.u[0]*0.0225, self.u[1]*0.0225, self.u[1]*0.0225])
            self.pub_cmd_R.publish(ref_motor)

            #if self.xh >= 0.95*self.ref_MAP[0] and self.xh <= 1.05*self.ref_MAP[0] and self.yh >= 0.95*self.ref_MAP[1] and self.yh <= 1.05*self.ref_MAP[1]:
            #self.count = self.count + 1

            self.get_logger().info(f"contador = {self.count}, ref = [{self.ref_MAP[0]},{self.ref_MAP[1]}], posicion = [{xy_h[0]},{xy_h[1]}]")
            self.ref_completa[0] = -self.ref_Izq[0][self.count] + self.origen_x_Graf
            self.ref_completa[1] = -self.ref_MAP[1]+self.sum_ref_lin
            self.count = self.count + 1

            if self.count >= self.ref_Izq.shape[1]-1:
                self.get_logger().warn("Fin de referencia alcanzado")
                self.selec_control = 0
                self.Senal = "Stop"
                self.count = 0
                
                if self.sentido == 0.0:
                    self.sentido = math.pi
                else:
                    self.sentido = 0.0
                
                self.sum_ref_lin = self.sum_ref_lin + 0.6

                reset_bno = String(data='reset')
                self.pub_bno_reset.publish(reset_bno)
                return


        #elif self.selec_control == 8: # Giro Derecha
        elif self.Senal == "Turn right":
            K_slam = np.array([[0.1, 0], [0, 0.1]])

            theta = self.yaw_rad
            #theta = self.Map[2]
            
            h = 0.02
            self.xh = self.Map[0] + h*dt*math.cos(theta)
            self.yh = self.Map[1] + h*dt*math.sin(theta)
            #self.xh = dt*V*math.cos(theta) - dt*h*omega*math.sin(theta) + self.xh
            #self.yh = dt*V*math.sin(theta) + dt*h*omega*math.cos(theta) + self.yh

            xy_h = np.array([[self.xh],[self.yh]], dtype=float)

            self.ref_MAP = np.array([[self.ref_Der[0][self.count] + self.origen_x], [self.ref_Der[1][self.count] + self.origen_y]], dtype=float)
            ref_next = np.array([[self.ref_Der[0][self.count + 1] + self.origen_x], [self.ref_Der[1][self.count + 1] + self.origen_y]], dtype=float)
            
            self.ref_dot = (ref_next - self.ref_MAP) / dt

            self.err = self.ref_MAP - xy_h

            J_inv = np.array([[math.cos(theta), math.sin(theta)],[-math.sin(theta)/h, math.cos(theta)/h]], dtype=float)

            U = J_inv @ (self.ref_dot + K_slam @ self.err)

            # acciones de control lqr
            #self.u = self.inv_M @ U
            U_vec = U.reshape(2, 1)
            w_wheels = self.inv_M @ U_vec
            self.u = [float(w_wheels[0, 0]), float(w_wheels[1, 0])]

            #self.get_logger().info(
            #    f"v={V} m/s | yaw={self.x[1]:.1f}rad | y={self.x[0]:.1f} m x={self.Map[0]:.1f} m"
            #)

            # referencias de los motores
            ref_motor = Float32MultiArray(data=[self.u[0]*0.0225, self.u[0]*0.0225, self.u[1]*0.0225, self.u[1]*0.0225])
            self.pub_cmd_R.publish(ref_motor)

            #if self.xh >= 0.95*self.ref_MAP[0] and self.xh <= 1.05*self.ref_MAP[0] and self.yh >= 0.95*self.ref_MAP[1] and self.yh <= 1.05*self.ref_MAP[1]:
            #self.count = self.count + 1

            self.get_logger().info(f"contador = {self.count}, ref = [{self.ref_MAP[0]},{self.ref_MAP[1]}], posicion = [{xy_h[0]},{xy_h[1]}]")
            self.count = self.count + 1
            self.ref_completa[0] = self.ref_MAP[0]
            self.ref_completa[1] = self.ref_MAP[1]+self.sum_ref_lin

            if self.count >= self.ref_Der.shape[1]-1:
                self.get_logger().warn("Fin de referencia alcanzado")
                self.selec_control = 0
                self.Senal = "Stop"
                self.count = 0

                if self.sentido == 0.0:
                    self.sentido = math.pi
                else:
                    self.sentido = 0.0

                self.sum_ref_lin = self.sum_ref_lin + 0.6

                reset_bno = String(data='reset')
                self.pub_bno_reset.publish(reset_bno)
                return
            
        # ---- LOG DE UNA FILA (cada periodo) ----
        self._log_row()
		
        # Log compacto para verificar vida del nodo
        

        # (Opcional) Publicar referencias actuales (aquí envía 4 ceros)
        # ref = Float32MultiArray(data=[0.0, 0.0, 0.0, 0.0])
        # self.pub_cmd_R.publish(ref)

def main():
    rclpy.init()
    node = ControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

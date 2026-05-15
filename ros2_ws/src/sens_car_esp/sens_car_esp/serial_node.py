# sens_car_esp/serial_node.py
import threading
import time
import json
import serial
import serial.tools.list_ports

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray, Int16MultiArray


def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


class SerialBridge(Node):
    """
    Nodo ROS2 que:
      - Publica:
          /Sens_Car_ESP/raw     (String)             -> línea cruda del puerto serie
          /Sens_Car_ESP/m1..m4  (Float32MultiArray)  -> [ref, vel, u]
          /Sens_Car_ESP/mins    (Float32MultiArray)  -> [dDer, dIzq]
      - Suscribe:
          /Sens_Car_ESP/tx_raw  (String)             -> reenvía tal cual por serie
          /Sens_Car_ESP/cmd_R   (Float32MultiArray)  -> "R v1 v2 v3 v4"
          /Sens_Car_ESP/cmd_S   (Int16MultiArray)    -> "S a1 a2"
    Parámetros:
      port:     /dev/ttyUSB0 (por defecto)
      baudrate: 115200
    """
    def __init__(self):
        super().__init__('Sens_Car_ESP')

        # --- Parámetros ---
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.baud = self.get_parameter('baudrate').get_parameter_value().integer_value

        # --- Publishers ---
        self.pub_raw = self.create_publisher(String, '/Sens_Car_ESP/raw', 50)
        self.pub_m = [
            self.create_publisher(Float32MultiArray, '/Sens_Car_ESP/m1', 10),
            self.create_publisher(Float32MultiArray, '/Sens_Car_ESP/m2', 10),
            self.create_publisher(Float32MultiArray, '/Sens_Car_ESP/m3', 10),
            self.create_publisher(Float32MultiArray, '/Sens_Car_ESP/m4', 10),
        ]
        # Descomentar publisher "pub_mins" para lectura de lidar con esp
#        self.pub_mins = self.create_publisher(Float32MultiArray, '/Sens_Car_ESP/mins', 10)

        # --- Subscribers ---
        self.sub_tx_raw = self.create_subscription(String, '/Sens_Car_ESP/tx_raw', self.on_tx_raw, 10)
        self.sub_cmd_R  = self.create_subscription(Float32MultiArray, '/Sens_Car_ESP/cmd_R', self.on_cmd_R, 10)
        self.sub_cmd_S  = self.create_subscription(Int16MultiArray, '/Sens_Car_ESP/cmd_S', self.on_cmd_S, 10)

        self.get_logger().info(f"Abriendo puerto {self.port} @ {self.baud} ...")

        self._ser = None
        self._ser_lock = threading.Lock()
        self._stop = False

        # Hilo lector
        self._th = threading.Thread(target=self.reader_loop, daemon=True)
        self._th.start()

        # Timer de reconexión (por si se desconecta)
        self._recon_timer = self.create_timer(2.0, self.ensure_port_open)

    # ---------- IO Serie ----------
    def ensure_port_open(self):
        if self._ser is not None and self._ser.is_open:
            return
        try:
            self._ser = serial.Serial(self.port, self.baud, timeout=0.2)
            self.get_logger().info(f"Conectado a {self.port}")
        except Exception as e:
            self._ser = None
            self.get_logger().warn(f"No pude abrir {self.port}: {e}")

    def reader_loop(self):
        # Bucle lector (publica cada línea recibida)
        while not self._stop:
            if self._ser is None or not self._ser.is_open:
                time.sleep(0.5)
                continue
            try:
                line = self._ser.readline()  # bytes hasta \n o timeout
                if not line:
                    continue

                try:
                    s = line.decode('utf-8', errors='replace').rstrip('\r\n')
                except Exception:
                    s = str(line)

                # Publica siempre la línea cruda (útil para depurar)
                self.pub_raw.publish(String(data=s))

                # Busca el marcador @JSON en cualquier parte de la línea
                idx = s.find('@JSON')
                if idx >= 0:
                    tail = s[idx + len('@JSON'):].strip()

                    # Extraer SOLO el JSON: desde la primera '{' hasta el ÚLTIMO '}'
                    lb = tail.find('{')
                    rb = tail.rfind('}')
                    if lb == -1 or rb == -1 or rb <= lb:
                        self.get_logger().warn(f"No encontré llaves en: {tail}")
                        continue
                    payload = tail[lb:rb+1]

                    # Parseo robusto
                    try:
                        data = json.loads(payload)
                    except Exception as e:
                        self.get_logger().warn(f"JSON inválido: {e} | {payload}")
                        continue

                    # Esperamos m1..m4 = [ref,vel,u], mins = [dDer,dIzq]
                    ok = True
                    motors = []
                    for key in ("m1", "m2", "m3", "m4"):
                        vec = data.get(key)
                        if not isinstance(vec, list) or len(vec) != 3:
                            ok = False
                            break
                        try:
                            motors.append([float(vec[0]), float(vec[1]), float(vec[2])])
                        except Exception:
                            ok = False
                            break
                            
                    # Descomentar para si se lee lidar con esp
#                    mins = data.get("mins")
#                    if not isinstance(mins, list) or len(mins) != 2:
#                        ok = False
#                    else:
#                        try:
#                            mins = [float(mins[0]), float(mins[1])]
#                        except Exception:
#                            ok = False

                    if not ok:
                        self.get_logger().warn(f"Estructura JSON inesperada: {payload}")
                        continue

                    # Publicar 4 motores
                    for i in range(4):
                        self.pub_m[i].publish(Float32MultiArray(data=motors[i]))

                    # Descomentar para si se lee lidar con esp
                    # Publicar mínimos LIDAR
 #                   self.pub_mins.publish(Float32MultiArray(data=mins))

            except Exception as e:
                self.get_logger().warn(f"Error leyendo serie: {e}")
                try:
                    if self._ser:
                        self._ser.close()
                except Exception:
                    pass
                self._ser = None
                time.sleep(0.5)

    def write_line(self, text: str):
        with self._ser_lock:
            if self._ser is None or not self._ser.is_open:
                self.get_logger().warn("Serie no abierto; descarto TX")
                return
            data = (text.strip() + '\n').encode('utf-8')
            self._ser.write(data)

    # ---------- Callbacks ----------
    def on_tx_raw(self, msg: String):
        self.get_logger().info(f"TX raw: {msg.data}")
        self.write_line(msg.data)

    def on_cmd_R(self, msg: Float32MultiArray):
        if not msg.data or len(msg.data) != 4:
            self.get_logger().warn("cmd_R: se esperan 4 floats (m/s)")
            return
        vals = [max(-1.41, min(1.41, float(v))) for v in msg.data]
        cmd = f"R {vals[0]:.3f} {vals[1]:.3f} {vals[2]:.3f} {vals[3]:.3f}"
        self.get_logger().info(f"TX: {cmd}")
        self.write_line(cmd)

    def on_cmd_S(self, msg: Int16MultiArray):
        if not msg.data or len(msg.data) != 2:
            self.get_logger().warn("cmd_S: se esperan 2 enteros (grados)")
            return
        a1 = int(msg.data[0]); a2 = int(msg.data[1])
        a1 = clamp(a1, -90, 90)
        a2 = clamp(a2, -90, 90)  # cambia a 20 si tu S2 realmente va a [-90,20]
        cmd = f"S {a1:d} {a2:d}"
        self.get_logger().info(f"TX: {cmd}")
        self.write_line(cmd)

    # ---------- Shutdown ----------
    def destroy_node(self):
        self._stop = True
        try:
            if self._ser and self._ser.is_open:
                self._ser.close()
        except Exception:
            pass
        return super().destroy_node()


def main():
    rclpy.init()
    node = SerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


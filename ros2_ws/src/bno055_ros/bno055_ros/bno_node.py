#!/usr/bin/env python3
import os
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Vector3

# Raspberry Pi 5: usa gpiozero con backend lgpio
os.environ.setdefault('GPIOZERO_PIN_FACTORY', 'lgpio')
from gpiozero import DigitalOutputDevice  # on()=HIGH, off()=LOW

import board
import busio
import adafruit_bno055

# ===================== Configuración =====================
I2C_ADDR = 0x28          # Cambia a 0x29 si el pin ADR está a 3V3
RST_PIN  = 17            # GPIO17 (BCM) conectado a nRESET del BNO055 (activo-bajo)
PUBLISH_RATE_HZ = 10.0   # Frecuencia de publicación ROS 10Hz=100ms o 5Hz=200ms
USE_EXT_CRYSTAL = False  # Según tu prueba, NDOF sin cristal fue el que funcionó

# Modos del BNO055 (datasheet)
MODE_CONFIG  = 0x00
MODE_IMUPLUS = 0x07      # sin magnetómetro (yaw relativo)
MODE_NDOF    = 0x0C      # con magnetómetro (yaw absoluto)

# ===================== Utilidades =====================
def wrap_angle_deg(a):
    return None if a is None else (a + 180.0) % 360.0 - 180.0

def _valid_tuple(e):
    return (e is not None) and all(v is not None for v in e)

def _is_all_zero(e, eps=1e-6):
    h, r, p = e
    return abs(h or 0.0) < eps and abs(r or 0.0) < eps and abs(p or 0.0) < eps

# ===================== Nodo ROS =====================
class BNONode(Node):
    def __init__(self):
        super().__init__('BNO')

        # Publisher/Subscriber (existen aunque el sensor aún no esté listo)
        self.pub = self.create_publisher(Vector3, 'BNO_pub', 10)
        self.sub = self.create_subscription(String, 'BNO_reset', self.on_reset_cmd, 10)

        # Estado IMU / offsets
        self.i2c = busio.I2C(board.SCL, board.SDA)  # una sola instancia del bus
        self.rst = DigitalOutputDevice(RST_PIN, active_high=True, initial_value=True)
        self.bno = None
        self.yaw0 = self.pitch0 = self.roll0 = 0.0

        # Auto-zero diferido
        self.auto_zero_done = False
        self.auto_zero_t0 = 0.0

        # Gestión de fallos I2C
        self.err_count = 0
        self.max_err_before_reinit = 3
        self.last_reinit = 0.0
        self.reinit_backoff = 1.5  # s

        # Timers
        self.timer_pub  = self.create_timer(1.0 / PUBLISH_RATE_HZ, self.publish_euler)
        self.timer_init = self.create_timer(1.0, self.try_init_once)

        self.get_logger().info("Nodo iniciado. Intentando inicializar BNO055...")

    # -------- Reset físico --------
    def pulse_reset(self):
        self.get_logger().warn("Reset HW: LOW 15 ms, luego HIGH...")
        self.rst.off()       # LOW -> entra en reset
        time.sleep(0.015)
        self.rst.on()        # HIGH -> sale de reset
        time.sleep(0.9)      # tiempo de arranque

    # -------- Configurar modo y cristal; verifica que entregue lecturas --------
    def configure_mode(self, mode=MODE_NDOF, use_xtal=USE_EXT_CRYSTAL):
        self.bno.mode = MODE_CONFIG
        time.sleep(0.03)
        try:
            self.bno.use_external_crystal = use_xtal
        except Exception:
            pass
        self.bno.mode = mode
        time.sleep(0.06)
        for _ in range(10):
            e = self.bno.euler
            if _valid_tuple(e):
                return True
            time.sleep(0.1)
        return False

    # -------- Fija offsets ahora mismo (soft-zero inmediato) --------
    def soft_zero(self):
        if not self.bno:
            return
        e = self.bno.euler
        if not _valid_tuple(e):
            return
        self.yaw0, self.roll0, self.pitch0 = (e[0] or 0.0), (e[1] or 0.0), (e[2] or 0.0)
        self.auto_zero_done = True
        self.get_logger().info("Soft-zero aplicado (0°).")

    # -------- Inicializa con backoff; programa auto-zero diferido --------
    def init_now(self):
        now = time.time()
        if now - self.last_reinit < self.reinit_backoff:
            return False
        self.last_reinit = now

        try:
            self.pulse_reset()
            self.bno = adafruit_bno055.BNO055_I2C(self.i2c, address=I2C_ADDR)

            # Primero intenta NDOF (sin cristal). Si no, cae a IMUPLUS.
            if self.configure_mode(MODE_NDOF, USE_EXT_CRYSTAL) or \
               self.configure_mode(MODE_IMUPLUS, USE_EXT_CRYSTAL):
                # Programa auto-zero ~0.5 s después con la primera lectura válida no-nula
                self.auto_zero_done = False
                self.auto_zero_t0 = time.time() + 0.5
                self.err_count = 0
                self.get_logger().info("BNO055 inicializado. Auto-zero en ~0.5 s con la primera lectura estable.")
                return True
            else:
                self.get_logger().warn("Sin Euler válidos tras configurar modo; reintentaré.")
                self.bno = None
                return False

        except Exception as e:
            self.get_logger().warn(f"init_now() fallo: {e}")
            self.bno = None
            return False

    # -------- Timer: intenta inicializar si no hay IMU activa --------
    def try_init_once(self):
        if self.bno is None:
            self.init_now()

    # -------- Comandos por topic --------
    def on_reset_cmd(self, msg: String):
        cmd = (msg.data or "").strip().lower()
        if cmd in ("hard", "reset_hard"):
            self.get_logger().warn("Reset HARD solicitado por topic...")
            self.init_now()  # hará reset + reconfig + auto-zero diferido
        else:
            self.soft_zero() # pone 0° desde la orientación actual

    # -------- Publicación periódica --------
    def publish_euler(self):
        if not self.bno:
            return
        try:
            e = self.bno.euler  # (heading=yaw, roll, pitch)
            if not _valid_tuple(e):
                return

            # Auto-zero diferido: toma la primera lectura válida no-nula tras warm-up
            if not self.auto_zero_done:
                if time.time() >= self.auto_zero_t0:
                    if not _is_all_zero(e):  # evita el frame 0,0,0 de arranque
                        self.yaw0, self.roll0, self.pitch0 = (e[0] or 0.0), (e[1] or 0.0), (e[2] or 0.0)
                        self.auto_zero_done = True
                        self.get_logger().info("Auto-zero aplicado: referencia fijada con lectura estable.")
                    else:
                        return  # espera siguiente ciclo si aún es 0,0,0
                else:
                    return      # aún no toca auto-zero

            h, r, p = e
            yaw = wrap_angle_deg(h - self.yaw0)
            rr  = wrap_angle_deg(r - self.roll0)
            pp  = wrap_angle_deg(p - self.pitch0)

            self.pub.publish(Vector3(x=float(pp or 0.0),
                                     y=float(rr or 0.0),
                                     z=float(yaw or 0.0)))
            self.err_count = 0  # lectura OK: limpia errores

        except OSError as ex:
            # Errno 121: Remote I/O error típico del bus I2C en Linux
            self.err_count += 1
            if self.err_count == 1:
                self.get_logger().warn(f"I2C error (posible glitch): {ex}. Intentaré recuperar...")
            if self.err_count >= self.max_err_before_reinit:
                self.get_logger().error(f"I2C error x{self.err_count}. Re-inicializando sensor...")
                self.bno = None
                self.init_now()  # intenta ya (con backoff)
        except Exception as ex:
            self.err_count += 1
            self.get_logger().error(f"Excepción leyendo BNO: {ex}")
            if self.err_count >= self.max_err_before_reinit:
                self.bno = None
                self.init_now()

def main():
    rclpy.init()
    node = BNONode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


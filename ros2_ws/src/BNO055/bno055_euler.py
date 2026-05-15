#!/usr/bin/env python3
import time, threading, errno
import board, busio
from gpiozero import OutputDevice
from smbus2 import SMBus
from adafruit_bno055 import BNO055_I2C

# ---- GPIO reset (nRST activo en bajo) ----
RST = OutputDevice(17, active_high=True, initial_value=True)
def hw_reset():
    RST.off(); time.sleep(0.010)  # 10 ms bajo
    RST.on();  time.sleep(0.80)   # ~800 ms boot

# ---- Detecta dirección leyendo CHIP_ID ----
def detect_addr(busnum=1):
    with SMBus(busnum) as b:
        for addr in (0x28, 0x29):
            try:
                chip = b.read_byte_data(addr, 0x00)
                if chip == 0xA0:
                    return addr
            except Exception:
                pass
    return 0x28  # fallback

# ---- Entrada por consola: '1' para reset ----
class ResetInput:
    def __init__(self):
        self.req = False
        self.t = threading.Thread(target=self._loop, daemon=True)
    def start(self): self.t.start()
    def _loop(self):
        print("Entrada: '1'+Enter para RESET; '0' continúa.")
        while True:
            try:
                if input().strip() == "1":
                    self.req = True
            except EOFError:
                break

def calib_str(bno):
    try:
        sys, g, a, m = bno.calibration_status
        return f"cal(sys={sys}, g={g}, a={a}, m={m})"
    except Exception:
        return "cal(? ? ? ?)"

def make_bno(i2c, addr):
    # Pequeña espera por si el chip está inicializando
    for _ in range(3):
        try:
            bno = BNO055_I2C(i2c, address=addr)
            time.sleep(0.05)
            # primer toque para forzar acceso I2C
            _ = bno.temperature
            return bno
        except OSError as e:
            if getattr(e, "errno", None) == errno.EREMOTEIO or "Remote I/O" in str(e):
                time.sleep(0.1)
            else:
                raise
    return BNO055_I2C(i2c, address=addr)

def main():
    # I2C de la Pi
    i2c = busio.I2C(board.SCL, board.SDA)
    addr = detect_addr()
    print(f"Dirección BNO055 detectada: 0x{addr:02X}")
    bno = make_bno(i2c, addr)

    ri = ResetInput(); ri.start()
    print("Leyendo Euler del BNO055 (grados). Ctrl+C para salir.")

    while True:
        # Reset manual solicitado
        if ri.req:
            print("[RESET] por consola…")
            hw_reset()
            addr = detect_addr()
            bno  = make_bno(i2c, addr)
            print("[RESET] Listo.")
            ri.req = False

        # Lectura robusta con reintento si hay [Errno 121]
        try:
            e = bno.euler  # (heading, roll, pitch) en grados
            if e is None:
                print("Esperando datos…")
            else:
                heading, roll, pitch = e
                yaw = (heading if heading is not None else float("nan")) % 360.0
                print(f"yaw={yaw:7.2f}°, roll={roll:7.2f}°, pitch={pitch:7.2f}° | {calib_str(bno)}")
        except OSError as e:
            if getattr(e, "errno", None) == errno.EREMOTEIO or "Remote I/O" in str(e):
                print("[I2C 121] Sin respuesta. Reset automático…")
                hw_reset()
                addr = detect_addr()
                bno  = make_bno(i2c, addr)
                print("[I2C 121] Reintentando tras reset.")
            else:
                raise
        time.sleep(0.02)  # ~50 Hz

if __name__ == "__main__":
    main()

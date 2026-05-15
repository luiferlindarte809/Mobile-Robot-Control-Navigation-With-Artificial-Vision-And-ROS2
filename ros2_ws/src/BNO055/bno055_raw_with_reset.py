#!/usr/bin/env python3
import time, threading
from smbus2 import SMBus
import RPi.GPIO as GPIO

# ---- Config HW ----
I2C_BUS = 1
ADDRS = (0x28, 0x29)
RESET_GPIO_BCM = 17        # nRST (activo en bajo)
RESET_LOW_MS = 10
BOOT_TIME_S = 0.8

# ---- Registros (Página 0) ----
REG_PAGE_ID     = 0x07
REG_CHIP_ID     = 0x00    # 0xA0
REG_OPR_MODE    = 0x3D
REG_PWR_MODE    = 0x3E
REG_UNIT_SEL    = 0x3B
REG_SYS_TRIGGER = 0x3F
REG_CALIB_STAT  = 0x35

# Euler (LSB primero), 1 LSB = 1/16 grado
EUL_HEADING_LSB = 0x1A  # yaw/heading
EUL_ROLL_LSB    = 0x1C
EUL_PITCH_LSB   = 0x1E

# Modos
OPR_MODE_CONFIG = 0x00
OPR_MODE_NDOF   = 0x0C
PWR_MODE_NORMAL = 0x00

def twos(lo, hi):
    v = (hi << 8) | lo
    return v - 0x10000 if v & 0x8000 else v

def gpio_setup():
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    GPIO.setup(RESET_GPIO_BCM, GPIO.OUT, initial=GPIO.HIGH)  # nRST inactivo (alto)

def hw_reset():
    GPIO.output(RESET_GPIO_BCM, GPIO.LOW)
    time.sleep(RESET_LOW_MS/1000.0)
    GPIO.output(RESET_GPIO_BCM, GPIO.HIGH)
    time.sleep(BOOT_TIME_S)

def detect_addr(bus):
    for a in ADDRS:
        try:
            if bus.read_byte_data(a, REG_CHIP_ID) == 0xA0:
                return a
        except OSError:
            pass
    return None

def wait_ready(bus, addr, tmax=2.0):
    t0 = time.time()
    while time.time()-t0 < tmax:
        try:
            if bus.read_byte_data(addr, REG_CHIP_ID) == 0xA0:
                return True
        except OSError:
            pass
        time.sleep(0.02)
    return False

def configure(bus, addr):
    bus.write_byte_data(addr, REG_PAGE_ID, 0x00); time.sleep(0.01)
    bus.write_byte_data(addr, REG_OPR_MODE, OPR_MODE_CONFIG); time.sleep(0.025)
    # Unidades: 0x00 => Euler en grados (1/16°), gyro dps, acc m/s^2, temp °C
    bus.write_byte_data(addr, REG_UNIT_SEL, 0x00); time.sleep(0.01)
    bus.write_byte_data(addr, REG_PWR_MODE, PWR_MODE_NORMAL); time.sleep(0.01)
    # (Opcional) cristal externo si tu placa lo tiene:
    # bus.write_byte_data(addr, REG_SYS_TRIGGER, 0x80); time.sleep(0.01)
    bus.write_byte_data(addr, REG_OPR_MODE, OPR_MODE_NDOF); time.sleep(0.02)

def read_calib(bus, addr):
    s = bus.read_byte_data(addr, REG_CALIB_STAT)
    return (s>>6)&3, (s>>4)&3, (s>>2)&3, s&3  # sys, gyro, accel, mag

def read_euler(bus, addr):
    raw = bus.read_i2c_block_data(addr, EUL_HEADING_LSB, 6)
    h = twos(raw[0], raw[1]) / 16.0
    r = twos(raw[2], raw[3]) / 16.0
    p = twos(raw[4], raw[5]) / 16.0
    return (h % 360.0), r, p

class ResetInput:
    def __init__(self):
        self.req = False
        self._stop = False
        import threading
        self.t = threading.Thread(target=self._loop, daemon=True)
    def start(self): self.t.start()
    def stop(self): self._stop = True
    def _loop(self):
        print("Entrada: '1'+Enter para RESET; '0' continúa.")
        while not self._stop:
            try:
                if input().strip() == "1":
                    self.req = True
            except EOFError:
                break

def main():
    gpio_setup()
    ri = ResetInput(); ri.start()
    with SMBus(I2C_BUS) as bus:
        addr = detect_addr(bus)
        if addr is None:
            print("No se detecta BNO055 (0x28/0x29). Revisa cableado/I2C.")
            return
        print(f"Dirección detectada: 0x{addr:02X}")

        if not wait_ready(bus, addr):  # por si está arrancando
            print("BNO no listo; continuo.")
        configure(bus, addr)

        print("Leyendo Euler (grados). Ctrl+C para salir.")
        try:
            while True:
                # Reset manual
                if ri.req:
                    print("[RESET] solicitado…")
                    hw_reset()
                    addr = detect_addr(bus)
                    if addr is None:
                        print("[RESET] No detecto el BNO tras reset.")
                        continue
                    if not wait_ready(bus, addr):
                        print("[RESET] BNO no listo aún.")
                    configure(bus, addr)
                    print("[RESET] OK.")
                    ri.req = False

                try:
                    yaw, roll, pitch = read_euler(bus, addr)
                    sys_, g, a, m = read_calib(bus, addr)
                    print(f"yaw={yaw:7.2f}°, roll={roll:7.2f}°, pitch={pitch:7.2f}° | cal(sys={sys_}, g={g}, a={a}, m={m})")
                except OSError as e:
                    print(f"[I2C] {e}. Reintento tras reset…")
                    hw_reset()
                    addr = detect_addr(bus)
                    if addr:
                        configure(bus, addr)
                time.sleep(0.02)  # ~50 Hz
        except KeyboardInterrupt:
            pass
        finally:
            ri.stop()
            GPIO.cleanup()
            print("\nSalida limpia.")

if __name__ == "__main__":
    main()

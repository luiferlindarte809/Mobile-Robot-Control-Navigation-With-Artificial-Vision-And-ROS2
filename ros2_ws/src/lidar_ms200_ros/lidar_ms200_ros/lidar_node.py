#!/usr/bin/env python3
import threading, time, serial, math
from collections import namedtuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

# ------------------- Constantes del protocolo MS200 -------------------
MS200_HEAD_1 = 0xAA
MS200_HEAD_2 = 0x55
MS200_TAIL_1 = 0x31
MS200_TAIL_2 = 0xF2
MS200_FLAG_SN = 0x01
MS200_FLAG_VERSION = 0x02
MS200_DATA_START = 0x54

MS200_POINT_MAX = 360
MS200_BUF_MAX   = 100
MS200_POINT_PER_PACK = 12

CRC_TABLE = [
    0x00,0x4d,0x9a,0xd7,0x79,0x34,0xe3,0xae,0xf2,0xbf,0x68,0x25,0x8b,0xc6,0x11,0x5c,0xa9,0xe4,0x33,0x7e,0xd0,0x9d,0x4a,0x07,0x5b,0x16,0xc1,0x8c,0x22,0x6f,0xb8,0xf5,
    0x1f,0x52,0x85,0xc8,0x66,0x2b,0xfc,0xb1,0xed,0xa0,0x77,0x3a,0x94,0xd9,0x0e,0x43,0xb6,0xfb,0x2c,0x61,0xcf,0x82,0x55,0x18,0x44,0x09,0xde,0x93,0x3d,0x70,0xa7,
    0xea,0x3e,0x73,0xa4,0xe9,0x47,0x0a,0xdd,0x90,0xcc,0x81,0x56,0x1b,0xb5,0xf8,0x2f,0x62,0x97,0xda,0x0d,0x40,0xee,0xa3,0x74,0x39,0x65,0x28,0xff,0xb2,0x1c,0x51,
    0x86,0xcb,0x21,0x6c,0xbb,0xf6,0x58,0x15,0xc2,0x8f,0xd3,0x9e,0x49,0x04,0xaa,0xe7,0x30,0x7d,0x88,0xc5,0x12,0x5f,0xf1,0xbc,0x6b,0x26,0x7a,0x37,0xe0,0xad,0x03,
    0x4e,0x99,0xd4,0x7c,0x31,0xe6,0xab,0x05,0x48,0x9f,0xd2,0x8e,0xc3,0x14,0x59,0xf7,0xba,0x6d,0x20,0xd5,0x98,0x4f,0x02,0xac,0xe1,0x36,0x7b,0x27,0x6a,0xbd,0xf0,
    0x5e,0x13,0xc4,0x89,0x63,0x2e,0xf9,0xb4,0x1a,0x57,0x80,0xcd,0x91,0xdc,0x0b,0x46,0xe8,0xa5,0x72,0x3f,0xca,0x87,0x50,0x1d,0xb3,0xfe,0x29,0x64,0x38,0x75,0xa2,
    0xef,0x41,0x0c,0xdb,0x96,0x42,0x0f,0xd8,0x95,0x3b,0x76,0xa1,0xec,0xb0,0xfd,0x2a,0x67,0xc9,0x84,0x53,0x1e,0xeb,0xa6,0x71,0x3c,0x92,0xdf,0x08,0x45,0x19,0x54,
    0x83,0xce,0x60,0x2d,0xfa,0xb7,0x5d,0x10,0xc7,0x8a,0x24,0x69,0xbe,0xf3,0xaf,0xe2,0x35,0x78,0xd6,0x9b,0x4c,0x01,0xf4,0xb9,0x6e,0x23,0x8d,0xc0,0x17,0x5a,0x06,
    0x4b,0x9c,0xd1,0x7f,0x32,0xe5,0xa8
]

def crc8(buf):
    c = 0x00
    for b in buf:
        c = CRC_TABLE[(c ^ b) & 0xFF]
    return c & 0xFF

Point = namedtuple('Point', 'distance intensity')  # distance en mm

class MS200Parser:
    """Parser estado-a-estado del MS200 (traducción directa de tu C)."""
    def __init__(self):
        self.rx_flag = 0
        self.rx_buf_len = 0
        self.rx_idx = 0
        self.rx_buf = bytearray(MS200_BUF_MAX)
        self.points = [Point(0,0) for _ in range(MS200_POINT_MAX)]
        self._lock = threading.Lock()
        self._new = False

    def _update_data(self, frame):
        count = frame[1] & 0x1F
        if count < 2:
            return
        start_angle = (frame[5] << 8) | frame[4]
        end_angle   = (frame[-4] << 8) | frame[-5]
        if end_angle > start_angle:
            step = (end_angle - start_angle) // (count - 1)
        else:
            step = (36000 + end_angle - start_angle) // (count - 1)

        with self._lock:
            for i in range(count):
                angle = ((start_angle + i*step) // 100) % 360
                dist = (frame[3*i+7] << 8) | frame[3*i+6]
                inten = frame[3*i+8]
                self.points[angle] = Point(dist, inten)
            self._new = True

    def feed_byte(self, b):
        if self.rx_flag == 0:
            if b == MS200_HEAD_1:
                self.rx_flag = 1
                self.rx_buf[0] = b
            elif b == MS200_DATA_START:
                self.rx_flag = 5
                self.rx_buf[0] = b

        elif self.rx_flag == 1:
            if b == MS200_HEAD_2:
                self.rx_flag = 2
                self.rx_buf[1] = b
            else:
                self.rx_flag = 0

        elif self.rx_flag == 2:
            self.rx_buf[2] = b
            self.rx_flag = 3

        elif self.rx_flag == 3:
            self.rx_buf[3] = b
            self.rx_flag = 4
            self.rx_buf_len = b + 3
            self.rx_idx = 0

        elif self.rx_flag == 4:
            if 4 + self.rx_idx < MS200_BUF_MAX:
                self.rx_buf[4 + self.rx_idx] = b
            self.rx_idx += 1
            if self.rx_idx >= self.rx_buf_len:
                # system report completo (no actualizamos puntos aquí)
                self.rx_flag = 0
                self.rx_buf_len = 0
                self.rx_idx = 0
            if 4 + self.rx_idx >= MS200_BUF_MAX:
                self.rx_flag = 0
                self.rx_buf_len = 0
                self.rx_idx = 0

        elif self.rx_flag == 5:
            self.rx_buf[1] = b
            self.rx_flag = 6
            self.rx_idx = 2
            self.rx_buf_len = (self.rx_buf[1] & 0x1F) * 3 + 11

        elif self.rx_flag == 6:
            if self.rx_idx < MS200_BUF_MAX:
                self.rx_buf[self.rx_idx] = b
            self.rx_idx += 1

            if self.rx_idx >= self.rx_buf_len:
                frame = bytes(self.rx_buf[:self.rx_buf_len])
                if crc8(frame[:-1]) == frame[-1]:
                    self._update_data(frame)
                self.rx_flag = 0
                self.rx_buf_len = 0
                self.rx_idx = 0

            if self.rx_idx >= MS200_BUF_MAX:
                self.rx_flag = 0
                self.rx_buf_len = 0
                self.rx_idx = 0

        else:
            self.rx_flag = 0
            self.rx_buf_len = 0
            self.rx_idx = 0

    def pop_new_points(self):
        with self._lock:
            if not self._new:
                return None
            self._new = False
            return list(self.points)

# ------------------- Nodo /LIDAR -> /Distancia -------------------
class LidarNode(Node):
    """
    Publica /Distancia (Float32MultiArray) = [Min_proy_I, Min_proy_D] en mm,
    cada 100 ms.
    """
    def __init__(self):
        super().__init__('LIDAR')

        # Parámetros (ajusta port según tu USB-TTL)
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 230400)
        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.baud = int(self.get_parameter('baudrate').get_parameter_value().integer_value)

        self.pub_dist = self.create_publisher(Float32MultiArray, '/Distancia', 10)

        self.parser = MS200Parser()
        self.ser = None
        self.stop_flag = False

        # Hilo de lectura UART
        self.th = threading.Thread(target=self._reader_loop, daemon=True)
        self.th.start()

        # Timer de publicación cada 100 ms
        self.create_timer(0.1, self._publish_100ms)

        self.get_logger().info(f"LIDAR: leyendo {self.port} @ {self.baud} baudios")

    def _open_serial(self):
        if self.ser and self.ser.is_open:
            return
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.05)
        except Exception as e:
            self.get_logger().warn(f"No puedo abrir {self.port}: {e}")
            self.ser = None

    def _reader_loop(self):
        while not self.stop_flag:
            if not self.ser or not self.ser.is_open:
                self._open_serial()
                time.sleep(0.3)
                continue
            try:
                data = self.ser.read(512)
                if data:
                    for b in data:
                        self.parser.feed_byte(b)
            except Exception as e:
                self.get_logger().warn(f"Error serie: {e}")
                try:
                    self.ser.close()
                except Exception:
                    pass
                self.ser = None
                time.sleep(0.3)

    @staticmethod
    def _window_min_proj(points, a0, a1):
        """mínimo de |d_mm * sin(ang)| para ángulos [a0..a1-1] grados."""
        minp = 1e6
        for ang in range(a0, a1):
            d_mm = points[ang % 360].distance
            proj = abs(d_mm * math.sin(math.radians(ang)))
            if proj < minp and proj > 0.0:
                minp = proj
        return minp if minp < 1e6 else 1e6

    def _publish_100ms(self):
        pts = self.parser.pop_new_points()
        if pts is None:
            return

        # Tu lógica exacta:
        # Derecha: 80..99°, Izquierda: 260..279°
        Min_proy_D = self._window_min_proj(pts, 50, 95)
        Min_proy_I = self._window_min_proj(pts, 265, 310)

        msg = Float32MultiArray()
        # Orden pedido: [Min_proy_I, Min_proy_D]
        msg.data = [float(Min_proy_I), float(Min_proy_D)]
        self.pub_dist.publish(msg)

    def destroy_node(self):
        self.stop_flag = True
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
        except Exception:
            pass
        return super().destroy_node()

def main():
    rclpy.init()
    node = LidarNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

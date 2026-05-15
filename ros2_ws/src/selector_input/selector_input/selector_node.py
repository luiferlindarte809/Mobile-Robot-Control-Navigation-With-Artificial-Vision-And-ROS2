#!/usr/bin/env python3
import threading
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import Int32


class SelectorNode(Node):
    """
    Lee enteros desde consola y publica en un tópico (por defecto /selector).
    Soporta también on/true/yes=1 y off/false/no=0.
    Parámetros:
      - topic (str): nombre del tópico. Default: /selector
      - latched (bool): si True, usa TRANSIENT_LOCAL (latch). Default: True
      - depth (int): profundidad de cola. Default: 1
    Comandos:
      - Escribe un entero y Enter, publica ese valor.
      - 'q'/'quit'/'exit' para salir.
    """

    def __init__(self):
        super().__init__('Camara')

        # Parámetros
        self.declare_parameter('topic', '/selector')
        self.declare_parameter('latched', True)
        self.declare_parameter('depth', 1)

        self.topic = self.get_parameter('topic').get_parameter_value().string_value
        latched = self.get_parameter('latched').get_parameter_value().bool_value
        depth = int(self.get_parameter('depth').get_parameter_value().integer_value or 1)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=depth,
            durability=DurabilityPolicy.TRANSIENT_LOCAL if latched else DurabilityPolicy.VOLATILE
        )

        self.pub = self.create_publisher(Int32, self.topic, qos)
        self._stop = False

        self.get_logger().info(
            f"Iniciado. Publicando Int32 en '{self.topic}' "
            f"(latched={latched}, depth={depth}). "
            "Escribe un entero y presiona Enter. 'q' para salir."
        )

        # Hilo de entrada por consola
        self._th = threading.Thread(target=self._input_loop, daemon=True)
        self._th.start()

    def _input_loop(self):
        while rclpy.ok() and not self._stop:
            try:
                s = input('selector> ').strip()
            except EOFError:
                break
            if not s:
                continue
            low = s.lower()
            if low in ('q', 'quit', 'exit'):
                self.get_logger().info('Saliendo por comando de usuario.')
                rclpy.shutdown()
                break

            # Parsear a entero (acepta on/off true/false)
            try:
                val = int(s)
            except ValueError:
                if low in ('on', 'true', 't', 'yes', 'y'):
                    val = 1
                elif low in ('off', 'false', 'f', 'no', 'n'):
                    val = 0
                else:
                    self.get_logger().warn("Entrada inválida. Escribe un entero (o on/off).")
                    continue

            msg = Int32()
            msg.data = val
            self.pub.publish(msg)
            self.get_logger().info(f'Publicado {val} en {self.topic}')

    def destroy_node(self):
        self._stop = True
        try:
            self._th.join(timeout=0.5)
        except Exception:
            pass
        return super().destroy_node()


def main():
    rclpy.init()
    node = SelectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

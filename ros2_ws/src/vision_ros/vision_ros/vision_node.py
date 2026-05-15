#!/usr/bin/env python3
import os
os.environ.setdefault("QT_QPA_PLATFORM", "xcb")  # evita líos en Wayland

import sys
# Ajusta la versión si tu Python no es 3.12:
sys.path.insert(0, "/home/pi/yoloenv/lib/python3.12/site-packages")

import cv2
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ultralytics import YOLO

class VisionNode(Node):
    def __init__(self):
        super().__init__('Vision_ROS')

        # -------- Parámetros ROS --------
        self.declare_parameter('model_path', '/home/pi/Vision/MejoresResultados25epoch/trasito4_YOLOv8s_noflip_allclass_9602/weights/best.pt')
        self.declare_parameter('camera_index', 0)
        self.declare_parameter('imgsz', 640)
        self.declare_parameter('conf', 0.5)
        self.declare_parameter('rate_hz', 15.0)
        self.declare_parameter('show_window', False)  # True si quieres previsualizar

        model_path   = self.get_parameter('model_path').get_parameter_value().string_value
        camera_index = int(self.get_parameter('camera_index').value)
        self.imgsz   = int(self.get_parameter('imgsz').value)
        self.conf    = float(self.get_parameter('conf').value)
        rate_hz      = float(self.get_parameter('rate_hz').value)
        self.show    = bool(self.get_parameter('show_window').value)

        # -------- Modelo YOLO --------
        self.get_logger().info(f'Cargando modelo: {model_path}')
        self.model = YOLO(model_path)
        self.device = 'cpu'  # en Pi, CPU

        # -------- Cámara --------
        self.cap = cv2.VideoCapture(camera_index, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if not self.cap.isOpened():
            raise RuntimeError('No se pudo abrir la cámara')

        if self.show:
            self.win = "YOLO"
            cv2.namedWindow(self.win, cv2.WINDOW_NORMAL)

        # -------- Publisher --------
        # (ASCII: /Senal)
        self.pub_label = self.create_publisher(String, '/Senal', 10)
        self.get_logger().info("pub creado")
        # -------- Timer --------
        self.timer = self.create_timer(1.0 / rate_hz, self.on_timer)
        self.t0 = time.time(); self.frames = 0
        self.get_logger().info(f'/Vision_ROS listo. Publicando en /Senal @ {rate_hz:.1f} Hz')

    def on_timer(self):
        ok, frame = self.cap.read()
        if not ok:
            self.get_logger().warn('Sin frame de cámara')
            return

        # --- Inferencia ---
        # Soporta tanto modelos "detect" como "classify"
        results = self.model.predict(
            source=frame,
            conf=self.conf,
            imgsz=self.imgsz,
            verbose=False,
            device=self.device
        )
        res = results[0]

        label_text = 'none'
        try:
            # Caso modelo de CLASIFICACIÓN
            if hasattr(res, 'probs') and res.probs is not None:
                top1 = int(res.probs.top1)
                conf = float(res.probs.top1conf)
                name = res.names[top1]
                label_text = f'{name}' #:{conf:.2f}'
            # Caso modelo de DETECCIÓN: toma el bbox con mayor confianza
            elif res.boxes is not None and len(res.boxes) > 0:
                confs = res.boxes.conf.cpu().numpy()
                i = confs.argmax()
                cls_id = int(res.boxes.cls[i].item())
                conf = float(confs[i])
                name = res.names[cls_id]
                label_text = f'{name}' #:{conf:.2f}'
        except Exception as e:
            self.get_logger().warn(f'Error al extraer clase: {e}')

        # --- Publica resultado ---
        self.pub_label.publish(String(data=label_text))

        # --- Ventana opcional ---
        if self.show:
            annotated = res.plot()
            self.frames += 1
            if self.frames % 10 == 0:
                dt = time.time() - self.t0
                fps = self.frames / dt if dt > 0 else 0.0
                cv2.putText(annotated, f'FPS:{fps:.1f}', (10, 25),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
            cv2.imshow(self.win, annotated)
            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord('q')):
                rclpy.shutdown()

    def destroy_node(self):
        try:
            if hasattr(self, 'cap') and self.cap:
                self.cap.rele_ase()
            if self.show:
                cv2.destroyAllWindows()
        finally:
            super().destroy_node()

def main():
    rclpy.init()
    node = VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

import cv2
from ultralytics import YOLO

# --- Cargar tu modelo entrenado ---
model_path = r"/home/pi/Vision-20251104T234956Z-1-001/Vision/MejoresResultados25epoch/trasito4_YOLOv8s_noflip_allclass_9602/weights"
model = YOLO(model_path)

# --- Iniciar la cámara (0 = cámara principal) ---
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("❌ No se pudo acceder a la cámara.")
    exit()

print("✅ Cámara iniciada. Presiona 'Q' para salir.")

while True:
    ret, frame = cap.read()
    if not ret:
        print("⚠️ No se pudo leer el frame de la cámara.")
        break

    # --- Hacer detección en tiempo real ---
    results = model.predict(source=frame, conf=0.5, verbose=False)

    # --- Dibujar las detecciones ---
    annotated_frame = results[0].plot()

    # --- Mostrar el resultado ---
    cv2.imshow("Detección en tiempo real - YOLOv8", annotated_frame)

    # --- Salir con la tecla Q ---
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# --- Liberar cámara y cerrar ventanas ---
cap.release()
cv2.destroyAllWindows()

print("👋 Detección finalizada.")
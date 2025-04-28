from ultralytics import YOLO

# Carga el modelo original
model = YOLO("yolov8n.pt")

# Exporta directamente. El archivo se guardará como yolov8n.onnx en el mismo directorio
model.export(
    format="onnx",
    imgsz=448,
    batch=4,
    dynamic=False,
    simplify=True,
    opset=12
)

# Luego lo mueves/renombras
import shutil
shutil.move("yolov8n.onnx", "/home/rcasal/ros2_ws/src/tracker_360/models/yolov8n_b4.onnx")

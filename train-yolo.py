from ultralytics import YOLO
import os

# Configure memory allocator
os.environ["PYTORCH_CUDA_ALLOC_CONF"] = "backend:native,garbage_collection_threshold:0.8"

# Load a model
model = YOLO('yolov8l-pose.pt')  # load a pretrained model (recommended for training)

# Train the model
results = model.train(data='/usr/src/ultralytics/pick-place/not-remove/Bottles.v1i.yolov5pytorch/data.yaml', epochs=100, imgsz=640, batch=8, device=0)

from ultralytics import YOLO

# load yolov8n (nano) model
model = YOLO("yolov8n.yaml")

# train the model
results = model.train(data="config.yaml", epochs=3)
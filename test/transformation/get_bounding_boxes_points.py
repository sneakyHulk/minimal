from ultralytics import YOLO

# Load a model
model = YOLO("yolov9c.pt")  # pretrained YOLOv8n model

# Run batched inference on a list of images
results = model(["1690366193546_dis.jpg", "1690366193546_undis.jpg"])  # return a list of Results objects

# Process results list
for result in results:
    boxes = result.boxes.xyxy  # Boxes object for bounding box outputs
    print(boxes[0])
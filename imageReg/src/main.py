from ultralytics import YOLO
import cv2

# Load a pretrained YOLO model (recommended for training)
# model = YOLO("yolo11n.pt")
# model = YOLO("../../runs/detect/train3/weights/best.pt") # Best model from training
# model = YOLO("model_v7c.pt")
model = YOLO("task2_v4a.pt")
# model = YOLO("best_task2_v4.pt")
# model = YOLO("best_v12.pt")

# Use real-time camera
# results = model.track(source="0", show=True, stream=True)
# for result in results:
#     boxes = result.boxes
#     for box in boxes:
#         print(box)

# Path to your input image
image_path = "/Users/zack/Desktop/TestS.png"

# Run inference
results = model(image_path)

# Get the first result
result = results[0]

# Print detected boxes
for box in result.boxes:
    cls_id = int(box.cls[0])
    conf = float(box.conf[0])
    x1, y1, x2, y2 = box.xyxy[0].tolist()
    class_name = model.names[cls_id]

    print(f"{class_name}: {conf:.2f} | box=({x1:.1f}, {y1:.1f}, {x2:.1f}, {y2:.1f})")

# Draw detections on the image
annotated = result.plot()

# Display the annotated image
cv2.imshow("Annotated Image", annotated)
cv2.waitKey(0)
cv2.destroyAllWindows()

def main():
    print("Hello from sc2079-mdp-group2-2026!")


if __name__ == "__main__":
    main()

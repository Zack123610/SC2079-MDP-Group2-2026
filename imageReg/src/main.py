from ultralytics import YOLO

# Load a pretrained YOLO model (recommended for training)
# model = YOLO("yolo26n.pt")
# model = YOLO("../../runs/detect/train2/weights/best.pt") # Best model from training
model = YOLO("model_v5.pt")
# model = YOLO("best_v12.pt")

# Use real-time camera
results = model.track(source="0", show=True, stream=True)
for result in results:
    boxes = result.boxes
    for box in boxes:
        print(box)

def main():
    print("Hello from sc2079-mdp-group2-2026!")


if __name__ == "__main__":
    main()

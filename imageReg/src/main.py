from ultralytics import YOLO

# Load a pretrained YOLO model (recommended for training)
model = YOLO("yolo26n.pt")

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

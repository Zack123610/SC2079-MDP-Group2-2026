"""
PC YOLO Inference Receiver
Receives video stream from Raspberry Pi and runs YOLO inference.

Run on PC:
    python pc_receiver.py --host <PI_IP_ADDRESS> --port 5555

Dependencies (install on PC):
    pip install pyzmq opencv-python ultralytics
"""

import argparse
import time
import cv2
import numpy as np
import zmq
from ultralytics import YOLO


def main():
    parser = argparse.ArgumentParser(description="PC YOLO Inference Receiver")
    parser.add_argument("--host", required=True, help="Raspberry Pi IP address")
    parser.add_argument("--port", type=int, default=5555, help="Port number")
    parser.add_argument("--model", default="yolo11n.pt", help="YOLO model path")
    parser.add_argument("--conf", type=float, default=0.5, help="Confidence threshold")
    parser.add_argument("--show", action="store_true", default=True, help="Show video window")
    args = parser.parse_args()

    # Setup ZeroMQ subscriber
    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    socket.setsockopt(zmq.RCVHWM, 1)  # Only keep latest frame
    socket.setsockopt(zmq.CONFLATE, 1)  # Discard old frames
    socket.setsockopt_string(zmq.SUBSCRIBE, "")  # Subscribe to all messages
    socket.connect(f"tcp://{args.host}:{args.port}")
    print(f"[INFO] Connected to tcp://{args.host}:{args.port}")

    # Load YOLO model
    print(f"[INFO] Loading YOLO model: {args.model}")
    model = YOLO(args.model)
    print("[INFO] Model loaded successfully")

    frame_count = 0
    start_time = time.time()
    last_fps_time = start_time
    fps_display = 0.0

    try:
        while True:
            try:
                # Receive frame with timeout
                data = socket.recv(zmq.NOBLOCK)
            except zmq.Again:
                # No frame available, wait a bit
                time.sleep(0.001)
                continue

            # Decode JPEG
            frame = cv2.imdecode(
                np.frombuffer(data, dtype=np.uint8),
                cv2.IMREAD_COLOR
            )
            
            if frame is None:
                continue

            # Run YOLO inference
            results = model(frame, conf=args.conf, verbose=False)
            
            # Get annotated frame
            annotated_frame = results[0].plot()
            
            # Calculate FPS
            frame_count += 1
            current_time = time.time()
            if current_time - last_fps_time >= 1.0:
                fps_display = frame_count / (current_time - start_time)
                last_fps_time = current_time

            # Add FPS overlay
            cv2.putText(
                annotated_frame,
                f"FPS: {fps_display:.1f}",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (0, 255, 0),
                2
            )

            # Display detected objects info
            for result in results:
                boxes = result.boxes
                for i, box in enumerate(boxes):
                    cls_id = int(box.cls[0])
                    cls_name = model.names[cls_id]
                    conf = float(box.conf[0])
                    print(f"[DETECT] {cls_name}: {conf:.2f}")

            # Show frame
            if args.show:
                cv2.imshow("YOLO Inference", annotated_frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

    except KeyboardInterrupt:
        print("\n[INFO] Stopping receiver...")
    finally:
        socket.close()
        context.term()
        cv2.destroyAllWindows()
        print("[INFO] Cleanup complete")


if __name__ == "__main__":
    main()

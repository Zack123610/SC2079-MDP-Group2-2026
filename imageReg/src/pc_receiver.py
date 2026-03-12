"""
PC YOLO Inference Receiver

Receives video stream from Raspberry Pi and runs YOLO inference.
Publishes detection results back to the Pi on a second ZMQ channel
so the Pi can relay them to Android / STM32.

Run on PC:
    python pc_receiver.py --host <PI_IP_ADDRESS> --port 5555

Ports:
    5555  (SUB) - Subscribe to video frames from Pi's camera streamer
    5556  (PUB) - Publish detection results back to Pi

Detection result format (JSON per message):
    {
        "cls_id":   5,
        "cls_name": "obstacle_5",
        "conf":     0.92,
        "bbox":     [x1, y1, x2, y2]
    }

Dependencies (install on PC):
    pip install pyzmq opencv-python ultralytics
"""

import argparse
import json
import time

import cv2
import numpy as np
import zmq
from ultralytics import YOLO


def main():
    parser = argparse.ArgumentParser(description="PC YOLO Inference Receiver")
    parser.add_argument("--host", required=True, help="Raspberry Pi IP address")
    parser.add_argument("--port", type=int, default=5555, help="Video stream port (SUB)")
    parser.add_argument("--result-port", type=int, default=5556,
                        help="Detection results port (PUB)")
    parser.add_argument("--model", default="yolo26n.pt", help="YOLO model path")
    parser.add_argument("--conf", type=float, default=0.5, help="Confidence threshold")
    parser.add_argument("--show", action="store_true", default=True,
                        help="Show video window")
    args = parser.parse_args()

    context = zmq.Context()

    # --- Video frame subscriber (Pi -> PC) --------------------------------
    sub_socket = context.socket(zmq.SUB)
    sub_socket.setsockopt(zmq.RCVHWM, 1)
    sub_socket.setsockopt(zmq.CONFLATE, 1)
    sub_socket.setsockopt_string(zmq.SUBSCRIBE, "")
    sub_socket.connect(f"tcp://{args.host}:{args.port}")
    print(f"[INFO] Subscribed to video on tcp://{args.host}:{args.port}")

    # --- Detection result publisher (PC -> Pi) ----------------------------
    pub_socket = context.socket(zmq.PUB)
    pub_socket.setsockopt(zmq.SNDHWM, 256)
    pub_socket.bind(f"tcp://0.0.0.0:{args.result_port}")
    print(f"[INFO] Publishing detections on tcp://0.0.0.0:{args.result_port}")

    # --- YOLO model -------------------------------------------------------
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
                data = sub_socket.recv(zmq.NOBLOCK)
            except zmq.Again:
                time.sleep(0.001)
                continue

            frame = cv2.imdecode(
                np.frombuffer(data, dtype=np.uint8),
                cv2.IMREAD_COLOR,
            )
            if frame is None:
                continue

            results = model(frame, conf=args.conf, verbose=False)

            annotated_frame = results[0].plot()

            # FPS
            frame_count += 1
            current_time = time.time()
            if current_time - last_fps_time >= 1.0:
                fps_display = frame_count / (current_time - start_time)
                last_fps_time = current_time

            cv2.putText(
                annotated_frame,
                f"FPS: {fps_display:.1f}",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (0, 255, 0),
                2,
            )

            # Process and publish detections
            for result in results:
                for box in result.boxes:
                    cls_id = int(box.cls[0])
                    cls_name = model.names[cls_id]
                    conf = float(box.conf[0])
                    bbox = box.xyxy[0].tolist()

                    print(f"[DETECT] {cls_name}: {conf:.2f}")

                    detection = {
                        "cls_id": cls_id,
                        "cls_name": cls_name,
                        "conf": round(conf, 4),
                        "bbox": [round(v, 1) for v in bbox],
                    }
                    pub_socket.send_string(json.dumps(detection), zmq.NOBLOCK)

            if args.show:
                cv2.imshow("YOLO Inference", annotated_frame)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break

    except KeyboardInterrupt:
        print("\n[INFO] Stopping receiver...")
    finally:
        sub_socket.close()
        pub_socket.close()
        context.term()
        cv2.destroyAllWindows()
        print("[INFO] Cleanup complete")


if __name__ == "__main__":
    main()

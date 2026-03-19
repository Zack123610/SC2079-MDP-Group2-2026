"""
PC YOLO Inference Receiver

Receives video stream from Raspberry Pi and runs YOLO inference.
Publishes detection results back to the Pi on a second ZMQ channel
so the Pi can relay them to Android / STM32.
Also supports one-shot image inference via REQ/REP.

Run on PC:
    python pc_receiver.py --host <PI_IP_ADDRESS> --port 5555

Ports:
    5555  (SUB) - Subscribe to video frames from Pi's camera streamer
    5556  (PUB) - Publish detection results back to Pi
    5557  (REP) - Receive one-shot JPEG requests and reply with box list

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


def extract_boxes(results, model) -> list[dict]:
    boxes_payload: list[dict] = []
    for result in results:
        for box in result.boxes:
            cls_id = int(box.cls[0])
            cls_name = model.names[cls_id]
            conf = float(box.conf[0])
            bbox = box.xyxy[0].tolist()
            boxes_payload.append(
                {
                    "cls_id": cls_id,
                    "cls_name": cls_name,
                    "conf": round(conf, 4),
                    "bbox": [round(v, 1) for v in bbox],
                }
            )
    return boxes_payload


def handle_request_inference(rep_socket, model, conf: float) -> bool:
    """
    Process at most one pending one-shot inference request.

    Returns True if a request was processed, else False.
    """
    try:
        parts = rep_socket.recv_multipart(zmq.NOBLOCK)
    except zmq.Again:
        return False

    request_id = None
    start_ts = time.time()
    try:
        if len(parts) == 2:
            metadata_raw, image_raw = parts
            metadata = json.loads(metadata_raw.decode("utf-8"))
            request_id = metadata.get("request_id")
        elif len(parts) == 1:
            image_raw = parts[0]
        else:
            rep_socket.send_string(
                json.dumps(
                    {
                        "request_id": request_id,
                        "ok": False,
                        "boxes": [],
                        "error": f"invalid multipart size: {len(parts)}",
                    }
                )
            )
            return True

        frame = cv2.imdecode(
            np.frombuffer(image_raw, dtype=np.uint8),
            cv2.IMREAD_COLOR,
        )
        if frame is None:
            rep_socket.send_string(
                json.dumps(
                    {
                        "request_id": request_id,
                        "ok": False,
                        "boxes": [],
                        "error": "failed to decode image",
                    }
                )
            )
            return True

        results = model(frame, conf=conf, verbose=False)
        boxes = extract_boxes(results, model)
        inference_ms = (time.time() - start_ts) * 1000.0
        rep_socket.send_string(
            json.dumps(
                {
                    "request_id": request_id,
                    "ok": True,
                    "boxes": boxes,
                    "inference_ms": round(inference_ms, 2),
                }
            )
        )
        print(f"[REQ] request_id={request_id}, boxes={len(boxes)}")
        return True

    except Exception as e:
        rep_socket.send_string(
            json.dumps(
                {
                    "request_id": request_id,
                    "ok": False,
                    "boxes": [],
                    "error": str(e),
                }
            )
        )
        return True


def main():
    parser = argparse.ArgumentParser(description="PC YOLO Inference Receiver")
    parser.add_argument("--host", default=None,
                        help="Raspberry Pi IP address (for stream mode)")
    parser.add_argument("--port", type=int, default=5555, help="Video stream port (SUB)")
    parser.add_argument("--result-port", type=int, default=5556,
                        help="Detection results port (PUB)")
    parser.add_argument("--request-port", type=int, default=5557,
                        help="One-shot inference port (REP)")
    parser.add_argument("--model", default="yolo26n.pt", help="YOLO model path")
    parser.add_argument("--conf", type=float, default=0.5, help="Confidence threshold")
    parser.add_argument("--show", action="store_true", default=True,
                        help="Show video window")
    args = parser.parse_args()

    context = zmq.Context()

    sub_socket = None
    pub_socket = None
    if args.host:
        # --- Video frame subscriber (Pi -> PC) ----------------------------
        sub_socket = context.socket(zmq.SUB)
        sub_socket.setsockopt(zmq.RCVHWM, 1)
        sub_socket.setsockopt(zmq.CONFLATE, 1)
        sub_socket.setsockopt_string(zmq.SUBSCRIBE, "")
        sub_socket.connect(f"tcp://{args.host}:{args.port}")
        print(f"[INFO] Subscribed to video on tcp://{args.host}:{args.port}")

        # --- Detection result publisher (PC -> Pi) ------------------------
        pub_socket = context.socket(zmq.PUB)
        pub_socket.setsockopt(zmq.SNDHWM, 1)
        pub_socket.bind(f"tcp://0.0.0.0:{args.result_port}")
        print(f"[INFO] Publishing detections on tcp://0.0.0.0:{args.result_port}")
    else:
        print("[INFO] Stream mode disabled (no --host provided)")

    # --- One-shot inference server (Pi -> PC -> Pi) -----------------------
    rep_socket = context.socket(zmq.REP)
    rep_socket.setsockopt(zmq.LINGER, 0)
    rep_socket.bind(f"tcp://0.0.0.0:{args.request_port}")
    print(f"[INFO] Listening for one-shot inference on tcp://0.0.0.0:{args.request_port}")

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
            handled_request = handle_request_inference(rep_socket, model, args.conf)

            if sub_socket is None:
                if not handled_request:
                    time.sleep(0.001)
                continue

            try:
                data = sub_socket.recv(zmq.NOBLOCK)
            except zmq.Again:
                if not handled_request:
                    time.sleep(0.001)
                continue

            frame = cv2.imdecode(
                np.frombuffer(data, dtype=np.uint8),
                cv2.IMREAD_COLOR,
            )
            if frame is None:
                time.sleep(0.001)
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

            # Process and publish detections (legacy stream mode)
            boxes = extract_boxes(results, model)
            for detection in boxes:
                print(f"[DETECT] {detection['cls_name']}: {detection['conf']:.2f}")
                pub_socket.send_string(json.dumps(detection), zmq.NOBLOCK)

            if args.show:
                cv2.imshow("YOLO Inference", annotated_frame)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break

    except KeyboardInterrupt:
        print("\n[INFO] Stopping receiver...")
    finally:
        if sub_socket:
            sub_socket.close()
        if pub_socket:
            pub_socket.close()
        rep_socket.close()
        context.term()
        cv2.destroyAllWindows()
        print("[INFO] Cleanup complete")


if __name__ == "__main__":
    main()

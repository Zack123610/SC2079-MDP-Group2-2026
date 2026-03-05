"""
Raspberry Pi Camera Streamer
Streams Picamera2 video over ZeroMQ to PC for YOLO inference.

Run on Raspberry Pi:
    python pi_streamer.py --host 0.0.0.0 --port 5555

Dependencies (install on Pi):
    pip install pyzmq opencv-python-headless
"""

import argparse
import time
import cv2
import zmq
from picamera2 import Picamera2


def create_camera(width: int = 1280, height: int = 720, fps: int = 60) -> Picamera2:
    """Initialize and configure Picamera2."""
    picam2 = Picamera2()
    
    # Configure for video streaming
    config = picam2.create_video_configuration(
        main={"size": (width, height), "format": "RGB888"},
        controls={"FrameRate": fps}
    )
    picam2.configure(config)
    return picam2


def main():
    parser = argparse.ArgumentParser(description="Pi Camera ZeroMQ Streamer")
    parser.add_argument("--host", default="0.0.0.0", help="Bind address")
    parser.add_argument("--port", type=int, default=5555, help="Port number")
    parser.add_argument("--width", type=int, default=1280, help="Frame width")
    parser.add_argument("--height", type=int, default=720, help="Frame height")
    parser.add_argument("--fps", type=int, default=60, help="Target FPS")
    parser.add_argument("--quality", type=int, default=80, help="JPEG quality (1-100)")
    args = parser.parse_args()

    # Setup ZeroMQ publisher
    context = zmq.Context()
    socket = context.socket(zmq.PUB)
    socket.setsockopt(zmq.SNDHWM, 1)  # Only keep latest frame
    socket.bind(f"tcp://{args.host}:{args.port}")
    print(f"[INFO] Streaming on tcp://{args.host}:{args.port}")

    # Initialize camera
    picam2 = create_camera(args.width, args.height, args.fps)
    picam2.start()
    print(f"[INFO] Camera started: {args.width}x{args.height} @ {args.fps}fps")

    # JPEG encode parameters
    encode_params = [cv2.IMWRITE_JPEG_QUALITY, args.quality]

    frame_count = 0
    start_time = time.time()

    try:
        while True:
            # Capture frame
            frame = picam2.capture_array()
            
            # Convert RGB to BGR for OpenCV encoding
            frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            
            # Encode to JPEG
            success, encoded = cv2.imencode(".jpg", frame_bgr, encode_params)
            if not success:
                continue
            
            # Send frame
            socket.send(encoded.tobytes(), zmq.NOBLOCK)
            
            # FPS tracking
            frame_count += 1
            if frame_count % 100 == 0:
                elapsed = time.time() - start_time
                fps = frame_count / elapsed
                print(f"[INFO] Streaming at {fps:.1f} FPS, frame size: {len(encoded.tobytes()) / 1024:.1f} KB")

    except KeyboardInterrupt:
        print("\n[INFO] Stopping stream...")
    finally:
        picam2.stop()
        socket.close()
        context.term()
        print("[INFO] Cleanup complete")


if __name__ == "__main__":
    main()

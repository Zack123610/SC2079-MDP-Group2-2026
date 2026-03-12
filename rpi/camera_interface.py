"""
Camera Interface Module

Streams Picamera2 video over ZeroMQ (PUB) to the PC for YOLO inference.
Runs the capture loop in a background daemon thread so it can be started
alongside the rest of main.py without blocking.

Usage:
    cam = CameraInterface(port=5555)
    cam.start()          # starts streaming in background thread
    ...
    cam.stop()           # stops camera and ZMQ socket

Context manager is supported:
    with CameraInterface() as cam:
        ...

Standalone test (run on Raspberry Pi):
    python camera_interface.py --port 5555
"""

import argparse
import sys
import threading
import time
from typing import Optional

import cv2
import zmq

# picamera2 is only available on the Raspberry Pi.
try:
    from picamera2 import Picamera2  # type: ignore
    _PICAMERA_AVAILABLE = True
except ImportError:
    Picamera2 = None  # type: ignore
    _PICAMERA_AVAILABLE = False


class CameraInterface:
    """
    ZeroMQ PUB streamer for the Raspberry Pi camera.

    Captures JPEG-encoded frames from Picamera2 and publishes them on
    a ZMQ PUB socket.  The PC's ``pc_receiver.py`` subscribes to this
    stream for YOLO inference.

    Attributes
    ----------
    DEFAULT_PORT : int
        Default ZMQ PUB port (5555).
    DEFAULT_WIDTH / DEFAULT_HEIGHT / DEFAULT_FPS : int
        Default capture resolution and frame rate.
    DEFAULT_QUALITY : int
        Default JPEG encode quality (1–100).
    """

    DEFAULT_PORT = 5555
    DEFAULT_WIDTH = 1280
    DEFAULT_HEIGHT = 720
    DEFAULT_FPS = 60
    DEFAULT_QUALITY = 80

    def __init__(
        self,
        host: str = "0.0.0.0",
        port: int = DEFAULT_PORT,
        width: int = DEFAULT_WIDTH,
        height: int = DEFAULT_HEIGHT,
        fps: int = DEFAULT_FPS,
        quality: int = DEFAULT_QUALITY,
    ) -> None:
        self._host = host
        self._port = port
        self._width = width
        self._height = height
        self._fps = fps
        self._quality = quality

        self._zmq_context: Optional[zmq.Context] = None
        self._zmq_socket = None
        self._camera = None

        self._thread: Optional[threading.Thread] = None
        self._running = False
        self._frame_count = 0

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def start(self) -> bool:
        """
        Initialise camera + ZMQ and begin streaming in a background thread.

        Returns True if started successfully, False on error.
        """
        if self._running:
            print("[CAM] Already running")
            return True

        if not _PICAMERA_AVAILABLE:
            print("[CAM] picamera2 is not installed. Cannot start camera.")
            return False

        # ZMQ publisher
        self._zmq_context = zmq.Context()
        self._zmq_socket = self._zmq_context.socket(zmq.PUB)
        self._zmq_socket.setsockopt(zmq.SNDHWM, 1)
        self._zmq_socket.bind(f"tcp://{self._host}:{self._port}")
        print(f"[CAM] ZMQ PUB bound on tcp://{self._host}:{self._port}")

        # Picamera2
        try:
            self._camera = Picamera2()
            config = self._camera.create_video_configuration(
                main={
                    "size": (self._width, self._height),
                    "format": "RGB888",
                },
                controls={"FrameRate": self._fps},
            )
            self._camera.configure(config)
            self._camera.start()
            print(f"[CAM] Camera started: {self._width}x{self._height} "
                  f"@ {self._fps} fps, JPEG quality={self._quality}")
        except Exception as e:
            print(f"[CAM] Failed to start camera: {e}")
            self._cleanup_zmq()
            return False

        # Background thread
        self._running = True
        self._frame_count = 0
        self._thread = threading.Thread(target=self._stream_loop, daemon=True)
        self._thread.start()
        return True

    def stop(self) -> None:
        """Stop the streaming thread, camera, and ZMQ socket."""
        self._running = False
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=3.0)
            self._thread = None

        if self._camera:
            try:
                self._camera.stop()
            except Exception:
                pass
            self._camera = None
            print("[CAM] Camera stopped")

        self._cleanup_zmq()
        print(f"[CAM] Streamed {self._frame_count} frames total")

    @property
    def is_running(self) -> bool:
        return self._running

    @property
    def frame_count(self) -> int:
        return self._frame_count

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------

    def _stream_loop(self) -> None:
        """Capture → JPEG encode → ZMQ publish loop (runs in thread)."""
        encode_params = [cv2.IMWRITE_JPEG_QUALITY, self._quality]
        start_time = time.time()

        while self._running:
            try:
                frame = self._camera.capture_array()
                frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

                success, encoded = cv2.imencode(".jpg", frame_bgr, encode_params)
                if not success:
                    continue

                self._zmq_socket.send(encoded.tobytes(), zmq.NOBLOCK)
                self._frame_count += 1

                if self._frame_count % 200 == 0:
                    elapsed = time.time() - start_time
                    fps = self._frame_count / elapsed if elapsed > 0 else 0
                    size_kb = len(encoded.tobytes()) / 1024
                    #print(f"[CAM] {fps:.1f} FPS, frame ~{size_kb:.1f} KB")

            except Exception as e:
                if self._running:
                    print(f"[CAM] Stream error: {e}")
                break

    def _cleanup_zmq(self) -> None:
        if self._zmq_socket:
            try:
                self._zmq_socket.close()
            except Exception:
                pass
            self._zmq_socket = None
        if self._zmq_context:
            try:
                self._zmq_context.term()
            except Exception:
                pass
            self._zmq_context = None

    # ------------------------------------------------------------------
    # Context manager
    # ------------------------------------------------------------------

    def __enter__(self) -> "CameraInterface":
        self.start()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> bool:
        self.stop()
        return False


# ---------------------------------------------------------------------------
# Standalone test
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Camera Interface (standalone test)")
    parser.add_argument("--host", default="0.0.0.0", help="Bind address")
    parser.add_argument("--port", type=int, default=5555, help="ZMQ PUB port")
    parser.add_argument("--width", type=int, default=CameraInterface.DEFAULT_WIDTH)
    parser.add_argument("--height", type=int, default=CameraInterface.DEFAULT_HEIGHT)
    parser.add_argument("--fps", type=int, default=60)
    parser.add_argument("--quality", type=int, default=80)
    args = parser.parse_args()

    cam = CameraInterface(
        host=args.host,
        port=args.port,
        width=args.width,
        height=args.height,
        fps=args.fps,
        quality=args.quality,
    )

    if not cam.start():
        print("Failed to start camera. Exiting.")
        sys.exit(1)

    print("Streaming... Press Ctrl+C to stop.")
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        cam.stop()

"""
PC Interface Module

Subscribes to detection results published by the PC's ``pc_receiver.py``
over a ZeroMQ PUB/SUB channel.

The PC publishes one JSON-encoded string per detection event.  This module
runs a background listener thread and stores incoming detections in a
thread-safe queue so the main loop can consume them without blocking.

Detection message format (JSON string sent by PC):
    {
        "cls_id":   5,
        "cls_name": "obstacle_5",
        "conf":     0.92,
        "bbox":     [x1, y1, x2, y2]
    }

Usage:
    pc = PCInterface(host="<PC_IP>", port=5556)
    pc.start()

    detection = pc.get_detection()   # non-blocking, returns dict or None
    detections = pc.get_all()        # drain everything currently queued

    pc.stop()

Context manager is supported:
    with PCInterface(host="<PC_IP>") as pc:
        ...

Standalone test:
    python pc_interface.py --host <PC_IP> --port 5556
"""

import argparse
import json
import queue
import sys
import threading
import time
from typing import Any, Optional

import zmq


class PCInterface:
    """
    ZeroMQ SUB listener for YOLO detection results from the PC.

    The PC side (``pc_receiver.py``) publishes detection results on a
    ZMQ PUB socket (default port 5556).  This class subscribes to that
    stream in a background thread and makes detections available through
    :meth:`get_detection` (non-blocking) or :meth:`get_all`.
    """

    DEFAULT_PORT = 5556

    def __init__(
        self,
        host: str = "127.0.0.1",
        port: int = DEFAULT_PORT,
        max_queue: int = 64,
    ) -> None:
        """
        Args:
            host:      IP address of the PC running ``pc_receiver.py``.
            port:      ZMQ PUB port on the PC for detection results.
            max_queue: Maximum number of queued detections before oldest
                       are silently dropped.
        """
        self._host = host
        self._port = port
        self._max_queue = max_queue

        self._zmq_context: Optional[zmq.Context] = None
        self._zmq_socket = None

        self._queue: queue.Queue[dict[str, Any]] = queue.Queue(maxsize=max_queue)
        self._thread: Optional[threading.Thread] = None
        self._running = False
        self._recv_count = 0

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def start(self) -> bool:
        """
        Connect to the PC's detection PUB socket and start listening.

        Returns True if the ZMQ socket was created (the actual TCP
        connection is lazy and will succeed once the PC is reachable).
        """
        if self._running:
            print("[PC] Already running")
            return True

        self._zmq_context = zmq.Context()
        self._zmq_socket = self._zmq_context.socket(zmq.SUB)
        self._zmq_socket.setsockopt(zmq.RCVHWM, 1)
        self._zmq_socket.setsockopt(zmq.CONFLATE, 1)
        self._zmq_socket.setsockopt_string(zmq.SUBSCRIBE, "")
        self._zmq_socket.connect(f"tcp://{self._host}:{self._port}")
        print(f"[PC] Subscribed to detections on tcp://{self._host}:{self._port}")

        self._running = True
        self._recv_count = 0
        self._thread = threading.Thread(target=self._listen_loop, daemon=True)
        self._thread.start()
        return True

    def stop(self) -> None:
        """Stop the listener thread and close the ZMQ socket."""
        self._running = False
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=3.0)
            self._thread = None

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

        print(f"[PC] Stopped. Received {self._recv_count} detection(s) total.")

    @property
    def is_running(self) -> bool:
        return self._running

    # ------------------------------------------------------------------
    # Read detections
    # ------------------------------------------------------------------

    def get_detection(self) -> Optional[dict[str, Any]]:
        """
        Return one detection dict from the queue, or ``None`` if empty.
        Non-blocking.
        """
        try:
            return self._queue.get_nowait()
        except queue.Empty:
            return None

    def get_all(self) -> list[dict[str, Any]]:
        """Drain and return all queued detections (may be empty)."""
        items: list[dict[str, Any]] = []
        while True:
            try:
                items.append(self._queue.get_nowait())
            except queue.Empty:
                break
        return items

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------

    def _listen_loop(self) -> None:
        """Background loop: receive JSON strings from ZMQ and enqueue."""
        while self._running:
            try:
                raw = self._zmq_socket.recv(zmq.NOBLOCK)
            except zmq.Again:
                time.sleep(0.01)
                continue
            except Exception as e:
                if self._running:
                    print(f"[PC] Recv error: {e}")
                break

            try:
                detection = json.loads(raw.decode("utf-8"))
            except (json.JSONDecodeError, UnicodeDecodeError) as e:
                print(f"[PC] Invalid detection payload: {e}")
                continue

            self._recv_count += 1

            # Drop oldest if queue is full
            if self._queue.full():
                try:
                    self._queue.get_nowait()
                except queue.Empty:
                    pass

            self._queue.put(detection)

            cls_name = detection.get("cls_name", "?")
            conf = detection.get("conf", 0)
            # print(f"[PC] Detection #{self._recv_count}: {cls_name} ({conf:.2f})")

    # ------------------------------------------------------------------
    # Context manager
    # ------------------------------------------------------------------

    def __enter__(self) -> "PCInterface":
        self.start()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> bool:
        self.stop()
        return False


# ---------------------------------------------------------------------------
# Standalone test
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="PC Interface (standalone test)")
    parser.add_argument("--host", required=True, help="PC IP address")
    parser.add_argument("--port", type=int, default=5556, help="Detection PUB port")
    args = parser.parse_args()

    pc = PCInterface(host=args.host, port=args.port)
    if not pc.start():
        print("Failed to start. Exiting.")
        sys.exit(1)

    print("Listening for detections... Press Ctrl+C to stop.")
    try:
        while True:
            det = pc.get_detection()
            if det:
                print(f"  -> {det}")
            else:
                time.sleep(0.05)
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        pc.stop()

"""
PC Interface Module

Subscribes to detection results published by the PC's ``pc_receiver.py``
over a ZeroMQ PUB/SUB channel, and supports one-shot inference requests
over a ZeroMQ REQ/REP channel.

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

One-shot inference request/response format:
  Pi -> PC (REQ multipart):
    [metadata_json_bytes, jpeg_image_bytes]

  PC -> Pi (REP JSON):
    {
        "request_id": 12,
        "ok": true,
        "boxes": [...],
        "inference_ms": 27.4
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
    DEFAULT_REQUEST_PORT = 5557

    def __init__(
        self,
        host: str = "127.0.0.1",
        port: int = DEFAULT_PORT,
        request_port: int = DEFAULT_REQUEST_PORT,
        max_queue: int = 64,
    ) -> None:
        """
        Args:
            host:      IP address of the PC running ``pc_receiver.py``.
            port:      ZMQ PUB port on the PC for detection results.
            request_port: ZMQ REP port on the PC for one-shot inference.
            max_queue: Maximum number of queued detections before oldest
                       are silently dropped.
        """
        self._host = host
        self._port = port
        self._request_port = request_port
        self._max_queue = max_queue

        self._zmq_context: Optional[zmq.Context] = None
        self._zmq_socket = None
        self._req_socket = None

        self._queue: queue.Queue[dict[str, Any]] = queue.Queue(maxsize=max_queue)
        self._thread: Optional[threading.Thread] = None
        self._running = False
        self._recv_count = 0
        self._request_seq = 0
        self._req_lock = threading.Lock()

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

        self._ensure_context()
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

    def start_request_client(self) -> bool:
        """Connect REQ socket for one-shot inference requests."""
        if self._req_socket is not None:
            return True

        self._ensure_context()
        try:
            self._req_socket = self._zmq_context.socket(zmq.REQ)
            self._req_socket.setsockopt(zmq.LINGER, 0)
            self._req_socket.connect(f"tcp://{self._host}:{self._request_port}")
            print(
                "[PC] Inference request client connected to "
                f"tcp://{self._host}:{self._request_port}"
            )
            return True
        except Exception as e:
            print(f"[PC] Failed to start request client: {e}")
            if self._req_socket:
                try:
                    self._req_socket.close()
                except Exception:
                    pass
                self._req_socket = None
            return False

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

        if self._req_socket:
            try:
                self._req_socket.close()
            except Exception:
                pass
            self._req_socket = None

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

    def request_inference(
        self,
        image_jpeg: bytes,
        timeout_s: float = 5.0,
    ) -> Optional[dict[str, Any]]:
        """
        Send one captured JPEG image to PC and wait for one inference result.

        Returns parsed response dict on success, else ``None``.
        """
        if not image_jpeg:
            return None

        if self._req_socket is None and not self.start_request_client():
            return None

        self._request_seq += 1
        request_id = self._request_seq
        metadata = {
            "request_id": request_id,
            "image_format": "jpg",
            "timestamp": time.time(),
        }

        with self._req_lock:
            try:
                self._req_socket.send_multipart(
                    [json.dumps(metadata).encode("utf-8"), image_jpeg]
                )
            except Exception as e:
                print(f"[PC] Failed to send inference request: {e}")
                self._reset_request_client()
                return None

            poller = zmq.Poller()
            poller.register(self._req_socket, zmq.POLLIN)
            timeout_ms = max(1, int(timeout_s * 1000))
            events = dict(poller.poll(timeout_ms))
            if self._req_socket not in events:
                print(f"[PC] Inference request timed out ({timeout_s:.1f}s)")
                # REQ socket state is out-of-sync on timeout, recreate it.
                self._reset_request_client()
                return None

            try:
                raw = self._req_socket.recv()
            except Exception as e:
                print(f"[PC] Failed to receive inference response: {e}")
                self._reset_request_client()
                return None

        try:
            response = json.loads(raw.decode("utf-8"))
        except (json.JSONDecodeError, UnicodeDecodeError) as e:
            print(f"[PC] Invalid inference response payload: {e}")
            return None

        if response.get("request_id") != request_id:
            print(
                "[PC] Warning: request_id mismatch "
                f"(sent={request_id}, received={response.get('request_id')})"
            )
        return response

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------

    def _ensure_context(self) -> None:
        if self._zmq_context is None:
            self._zmq_context = zmq.Context()

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

    def _reset_request_client(self) -> None:
        if self._req_socket:
            try:
                self._req_socket.close()
            except Exception:
                pass
            self._req_socket = None
        self.start_request_client()

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
    parser.add_argument("--request-port", type=int, default=5557,
                        help="Inference request REP port")
    args = parser.parse_args()

    pc = PCInterface(host=args.host, port=args.port, request_port=args.request_port)
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

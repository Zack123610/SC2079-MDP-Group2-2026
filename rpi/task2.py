"""
Task 2 – Two-obstacle left/right decision flow.

Flow:
  1. Wait for Android to send BEGIN.
  2. Send STM command 1000, wait for DONE.
  3. Run 1-second detection window, count LEFT vs RIGHT detections only.
     - "39" or "Left" => LEFT
     - "38" or "Right" => RIGHT
     - Obstacle/others are ignored
     If LEFT > RIGHT -> send 3000, else if RIGHT > LEFT -> send 4000.
  4. Wait for DONE.
  5. Run another 1-second detection window.
     If LEFT > RIGHT -> send 3030, else if RIGHT > LEFT -> send 4040.
  6. Wait for DONE.

STM command format:
  XXXX
  (4 ASCII characters, no < > framing, no checksum bytes)
"""

from __future__ import annotations

import argparse
import re
import sys
import threading
import time
from typing import Any, Optional, Union

from bluetooth_interface import BluetoothInterface as BluetoothSerial
from bluetooth_interface_socket import BluetoothInterface as BluetoothSocket
from camera_interface import CameraInterface
from pc_interface import PCInterface
from stm32_interface import STM32Interface

BluetoothIface = Union[BluetoothSerial, BluetoothSocket]

# ---------------------------------------------------------------------------
# Task-2 constants
# ---------------------------------------------------------------------------

CMD_FORWARD_TO_OBS = "1000"
CMD_FIRST_LEFT = "3000"
CMD_FIRST_RIGHT = "4000"
CMD_SECOND_LEFT = "3030"
CMD_SECOND_RIGHT = "4040"

STM_WAIT_TIMEOUT = 45.0
DETECTION_TIME = 1.0
MAX_DETECTION_RETRIES = 4
MIN_DISTINCT_MARGIN = 2


def send_and_wait_done(
    stm: STM32Interface,
    cmd: str,
    timeout: float = STM_WAIT_TIMEOUT,
) -> bool:
    """Send one 4-char STM command and wait for a response containing DONE."""
    payload = cmd.strip()
    if len(payload) != 4:
        print(f"[TASK2] Invalid STM command length: '{payload}'")
        return False

    print(f"[TASK2] -> STM32: {payload}")

    if not stm.send(payload, add_newline=False):
        print("[TASK2] Failed to send to STM32")
        return False

    start = time.time()
    while time.time() - start < timeout:
        msg = stm.receive(timeout=0.1)
        if not msg:
            continue

        normalized = msg.strip().upper()
        print(f"[TASK2] <- STM32: {normalized}")
        if "DONE" in normalized:
            return True

    print(f"[TASK2] Timeout waiting for response containing DONE ({timeout}s)")
    return False


# ---------------------------------------------------------------------------
# Detection tracker
# ---------------------------------------------------------------------------

_ID_SUFFIX_RE = re.compile(r"-id-(\d+)$", re.IGNORECASE)


def _parse_int(value: Any) -> Optional[int]:
    if isinstance(value, int):
        return value
    if value is None:
        return None
    s = str(value).strip()
    if not s:
        return None
    if s.isdigit():
        return int(s)
    return None


def classify_arrow(det: dict[str, Any]) -> Optional[str]:
    """
    Return LEFT / RIGHT / OBSTACLE / None.

    LEFT mapping:  cls_id 39, "39", "Left", "...-id-39"
    RIGHT mapping: cls_id 38, "38", "Right", "...-id-38"
    """
    cls_name = str(det.get("cls_name", "")).strip()
    cls_name_l = cls_name.lower()
    cls_id = _parse_int(det.get("cls_id"))

    name_id: Optional[int] = None
    suffix = _ID_SUFFIX_RE.search(cls_name)
    if suffix:
        name_id = int(suffix.group(1))
    elif cls_name.isdigit():
        name_id = int(cls_name)

    ids = {v for v in (cls_id, name_id) if v is not None}

    if 39 in ids or "left" in cls_name_l:
        return "LEFT"
    if 38 in ids or "right" in cls_name_l:
        return "RIGHT"
    if "obstacle" in cls_name_l:
        return "OBSTACLE"
    return None


class ArrowDetectionTracker:
    """Thread-safe running counts of detections in the current window."""

    def __init__(self) -> None:
        self._left = 0
        self._right = 0
        self._ignored = 0
        self._total = 0
        self._lock = threading.Lock()

    def clear(self) -> None:
        with self._lock:
            self._left = 0
            self._right = 0
            self._ignored = 0
            self._total = 0

    def push(self, det: dict[str, Any]) -> None:
        label = classify_arrow(det)
        with self._lock:
            self._total += 1
            if label == "LEFT":
                self._left += 1
            elif label == "RIGHT":
                self._right += 1
            else:
                self._ignored += 1

    def snapshot(self) -> tuple[int, int, int, int]:
        with self._lock:
            return (self._left, self._right, self._ignored, self._total)


def detection_collector(
    pc: PCInterface,
    tracker: ArrowDetectionTracker,
    stop_event: threading.Event,
) -> None:
    """Drain PC detections and feed the tracker continuously."""
    while not stop_event.is_set():
        det = pc.get_detection()
        if det:
            tracker.push(det)
        else:
            time.sleep(0.01)


def prepare_for_detection(pc: PCInterface, tracker: ArrowDetectionTracker) -> None:
    """
    Clear tracker and drain PC queue so the next detection window starts fresh.
    Call this right after receiving DONE from STM32, before each detection window.
    """
    tracker.clear()
    # Drain any stale detections from the PC queue (from before/during the move)
    discarded = pc.get_all()
    if discarded:
        print(f"[TASK2] Cleared {len(discarded)} stale detection(s) from queue")


def detect_direction(
    tracker: ArrowDetectionTracker,
    stage_name: str,
    detection_time: float,
    retries: int,
    min_distinct_margin: int,
) -> Optional[str]:
    """
    Run detection windows until a non-tie decision is obtained.

    Returns:
        "LEFT" / "RIGHT" / None
    """
    for attempt in range(1, retries + 1):
        tracker.clear()
        print(f"[TASK2] {stage_name}: collecting detections "
              f"({detection_time:.1f}s), attempt {attempt}/{retries}")
        time.sleep(detection_time)

        left, right, ignored, total = tracker.snapshot()
        diff = abs(left - right)
        print(f"[TASK2] {stage_name}: LEFT={left}, RIGHT={right}, "
              f"ignored={ignored}, total={total}, diff={diff}")

        if left == right:
            print(f"[TASK2] {stage_name}: tie, retrying...")
            continue

        decision = "LEFT" if left > right else "RIGHT"
        if diff < min_distinct_margin:
            print(f"[TASK2] {stage_name}: warning - not very distinct "
                  f"(min={min_distinct_margin}), using {decision}")
        return decision

    print(f"[TASK2] {stage_name}: failed to get a clear LEFT/RIGHT decision")
    return None


# ---------------------------------------------------------------------------
# Task2 flow
# ---------------------------------------------------------------------------

def run(
    bt: BluetoothIface,
    stm: STM32Interface,
    pc: PCInterface,
    tracker: ArrowDetectionTracker,
    detection_time: float,
    retries: int,
    min_distinct_margin: int,
) -> None:
    print("\n" + "=" * 60)
    print("  Task 2 – Arrow-based Obstacle Decisions")
    print("=" * 60)
    print("Waiting for Android message: BEGIN")
    print()

    while True:
        msg = bt.readline()
        if not msg:
            time.sleep(0.05)
            continue

        token = msg.strip().upper()
        print(f"[TASK2] <- Android: '{msg.strip()}'")

        if token != "BEGIN":
            print("[TASK2] Ignoring message (expecting BEGIN)")
            continue

        print("[TASK2] BEGIN received. Starting Task 2 run.")

        # 1) Move forward to first obstacle
        if not send_and_wait_done(stm, CMD_FORWARD_TO_OBS):
            print("[TASK2] Aborting run (step 1 failed)")
            continue

        # 2) Detect first arrow and execute turn
        prepare_for_detection(pc, tracker)
        first_direction = detect_direction(
            tracker=tracker,
            stage_name="First obstacle",
            detection_time=detection_time,
            retries=retries,
            min_distinct_margin=min_distinct_margin,
        )
        if first_direction is None:
            print("[TASK2] Aborting run (first obstacle decision failed)")
            continue

        first_cmd = CMD_FIRST_LEFT if first_direction == "LEFT" else CMD_FIRST_RIGHT
        print(f"[TASK2] First obstacle decision: {first_direction} -> {first_cmd}")
        if not send_and_wait_done(stm, first_cmd):
            print("[TASK2] Aborting run (step 3 failed)")
            continue

        # 3) Detect second arrow and execute turn
        prepare_for_detection(pc, tracker)
        second_direction = detect_direction(
            tracker=tracker,
            stage_name="Second obstacle",
            detection_time=detection_time,
            retries=retries,
            min_distinct_margin=min_distinct_margin,
        )
        if second_direction is None:
            print("[TASK2] Aborting run (second obstacle decision failed)")
            continue

        second_cmd = CMD_SECOND_LEFT if second_direction == "LEFT" else CMD_SECOND_RIGHT
        print(f"[TASK2] Second obstacle decision: {second_direction} -> {second_cmd}")
        if not send_and_wait_done(stm, second_cmd):
            print("[TASK2] Aborting run (step 5 failed)")
            continue

        print("[TASK2] Run complete.")
        bt.send("TASK2,DONE")


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def _cleanup(
    stm: Optional[STM32Interface] = None,
    cam: Optional[CameraInterface] = None,
    pc: Optional[PCInterface] = None,
    bt: Optional[BluetoothIface] = None,
) -> None:
    if bt:
        bt.close()
    if pc:
        pc.stop()
    if cam:
        cam.stop()
    if stm:
        stm.close()


def main() -> None:
    parser = argparse.ArgumentParser(description="Task 2 – Arrow decision flow")
    parser.add_argument("--pc-host", required=True, help="PC IP address")
    parser.add_argument("--pc-port", type=int, default=5556,
                        help="PC detection ZMQ PUB port")
    parser.add_argument("--cam-port", type=int, default=5555,
                        help="Camera stream ZMQ PUB port")
    parser.add_argument("--cam-width", type=int, default=CameraInterface.DEFAULT_WIDTH,
                        help="Camera capture width")
    parser.add_argument("--cam-height", type=int, default=CameraInterface.DEFAULT_HEIGHT,
                        help="Camera capture height")
    parser.add_argument("--no-camera", action="store_true",
                        help="Skip camera (if external streamer already running)")
    parser.add_argument("--bt-mode", choices=["socket", "serial"], default="serial",
                        help="Bluetooth backend: socket or serial")
    parser.add_argument("--detection-time", type=float, default=DETECTION_TIME,
                        help="Detection window duration (seconds)")
    parser.add_argument("--retries", type=int, default=MAX_DETECTION_RETRIES,
                        help="Max detection retries per obstacle")
    parser.add_argument("--min-distinct-margin", type=int, default=MIN_DISTINCT_MARGIN,
                        help="Warn if |LEFT-RIGHT| is below this margin")
    args = parser.parse_args()

    # STM32
    print("[INIT] Connecting to STM32 ...")
    stm = STM32Interface()
    if not stm.is_connected:
        print("[INIT] STM32 not found. Exiting.")
        sys.exit(1)
    print("[INIT] STM32 connected")

    # Camera
    cam: Optional[CameraInterface] = None
    if not args.no_camera:
        cam = CameraInterface(
            port=args.cam_port,
            width=args.cam_width,
            height=args.cam_height,
        )
        if cam.start():
            print("[INIT] Camera streaming")
        else:
            print("[INIT] Camera failed - continuing without camera")
            cam = None

    # PC detection listener
    pc = PCInterface(host=args.pc_host, port=args.pc_port)
    if not pc.start():
        print("[INIT] PC detection listener failed. Exiting.")
        _cleanup(stm, cam=cam)
        sys.exit(1)
    print("[INIT] PC detection listener running")

    # Bluetooth
    if args.bt_mode == "socket":
        print("[INIT] Using socket-based Bluetooth")
        bt: BluetoothIface = BluetoothSocket()
    else:
        print("[INIT] Using serial-based Bluetooth (/dev/rfcomm0)")
        bt = BluetoothSerial()
    if not bt.start():
        print("[INIT] Bluetooth failed. Exiting.")
        _cleanup(stm, cam=cam, pc=pc)
        sys.exit(1)
    print("[INIT] Bluetooth connected")

    # Detection tracker + collector thread
    tracker = ArrowDetectionTracker()
    stop_event = threading.Event()
    collector = threading.Thread(
        target=detection_collector,
        args=(pc, tracker, stop_event),
        daemon=True,
    )
    collector.start()

    # Main loop
    try:
        run(
            bt=bt,
            stm=stm,
            pc=pc,
            tracker=tracker,
            detection_time=max(0.1, args.detection_time),
            retries=max(1, args.retries),
            min_distinct_margin=max(0, args.min_distinct_margin),
        )
    except KeyboardInterrupt:
        print("\n[TASK2] Interrupted")
    finally:
        stop_event.set()
        collector.join(timeout=2.0)
        _cleanup(stm, cam=cam, pc=pc, bt=bt)
        print("[TASK2] Shutdown complete")


if __name__ == "__main__":
    main()

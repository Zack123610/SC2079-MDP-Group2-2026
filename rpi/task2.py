"""
Task 2 – Two-obstacle left/right decision flow (photo-based inference).

Flow:
  1. Wait for Android to send BEGIN.
  2. Send STM command 1000, wait for DONE.
  3. Capture one photo with Picamera2 and send to PC for inference.
     - Ignore obstacle detections.
     - LEFT = class id 39 or class name containing "left".
     - RIGHT = class id 38 or class name containing "right".
     If LEFT -> send 3000, if RIGHT -> send 4000.
  4. Wait for DONE.
  5. Capture another photo and infer again.
     If LEFT -> send 3030, if RIGHT -> send 4040.
  6. Wait for DONE, then notify Android TASK2,DONE.

  python task2.py --pc-host <PC_IP> --pc-request-port 5557
  python imageReg/src/pc_receiver.py --request-port 5557 --model yolo26n.pt
"""

from __future__ import annotations

import argparse
import re
import sys
import time
from typing import Any, Optional, Union

import cv2

from bluetooth_interface import BluetoothInterface as BluetoothSerial
from bluetooth_interface_socket import BluetoothInterface as BluetoothSocket
from pc_interface import PCInterface
from stm32_interface import STM32Interface

try:
    from picamera2 import Picamera2  # type: ignore
    _PICAMERA_AVAILABLE = True
except ImportError:
    Picamera2 = None  # type: ignore
    _PICAMERA_AVAILABLE = False

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
INFERENCE_TIMEOUT = 5.0
MAX_CAPTURE_RETRIES = 4
CAMERA_WARMUP = 0.25


class SnapshotCamera:
    """Picamera2 one-shot capture helper."""

    DEFAULT_WIDTH = 3280
    DEFAULT_HEIGHT = 2464
    DEFAULT_QUALITY = 80

    def __init__(
        self,
        width: int = DEFAULT_WIDTH,
        height: int = DEFAULT_HEIGHT,
        quality: int = DEFAULT_QUALITY,
        warmup_s: float = CAMERA_WARMUP,
    ) -> None:
        self._width = width
        self._height = height
        self._quality = quality
        self._warmup_s = warmup_s
        self._camera = None

    def start(self) -> bool:
        if not _PICAMERA_AVAILABLE:
            print("[CAM] picamera2 is not installed. Cannot capture images.")
            return False

        try:
            self._camera = Picamera2()
            config = self._camera.create_still_configuration(
                main={
                    "size": (self._width, self._height),
                    "format": "RGB888",
                }
            )
            self._camera.configure(config)
            self._camera.start()
            time.sleep(max(0.0, self._warmup_s))
            print(
                "[CAM] Ready for snapshots: "
                f"{self._width}x{self._height}, JPEG quality={self._quality}"
            )
            return True
        except Exception as e:
            print(f"[CAM] Failed to start snapshot camera: {e}")
            self.stop()
            return False

    def capture_jpeg(self) -> Optional[bytes]:
        if not self._camera:
            return None

        try:
            frame = self._camera.capture_array()
            frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            success, encoded = cv2.imencode(
                ".jpg",
                frame_bgr,
                [cv2.IMWRITE_JPEG_QUALITY, self._quality],
            )
            if not success:
                return None
            return encoded.tobytes()
        except Exception as e:
            print(f"[CAM] Snapshot capture failed: {e}")
            return None

    def stop(self) -> None:
        if self._camera:
            try:
                self._camera.stop()
            except Exception:
                pass
            self._camera = None
            print("[CAM] Snapshot camera stopped")


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


_ID_SUFFIX_RE = re.compile(r"-id-(\d+)$", re.IGNORECASE)


def _parse_int(value: Any) -> Optional[int]:
    if isinstance(value, int):
        return value
    if value is None:
        return None
    s = str(value).strip()
    if s.isdigit():
        return int(s)
    return None


def _parse_float(value: Any, default: float = 0.0) -> float:
    try:
        return float(value)
    except Exception:
        return default


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

    if "obstacle" in cls_name_l:
        return "OBSTACLE"
    if 39 in ids or "left" in cls_name_l:
        return "LEFT"
    if 38 in ids or "right" in cls_name_l:
        return "RIGHT"
    return None


def decide_direction_from_boxes(boxes: list[dict[str, Any]]) -> Optional[str]:
    """Pick LEFT/RIGHT from one inference result's boxes, ignoring obstacles."""
    left_count = 0
    right_count = 0
    left_best = -1.0
    right_best = -1.0

    for box in boxes:
        label = classify_arrow(box)
        conf = _parse_float(box.get("conf"), default=0.0)
        if label == "LEFT":
            left_count += 1
            left_best = max(left_best, conf)
        elif label == "RIGHT":
            right_count += 1
            right_best = max(right_best, conf)

    if left_count == 0 and right_count == 0:
        return None

    if left_count > right_count:
        return "LEFT"
    if right_count > left_count:
        return "RIGHT"

    if left_best > right_best:
        return "LEFT"
    if right_best > left_best:
        return "RIGHT"
    return None


def detect_direction_from_photo(
    camera: SnapshotCamera,
    pc: PCInterface,
    stage_name: str,
    retries: int,
    inference_timeout: float,
) -> Optional[str]:
    """Capture photo(s), request PC inference, and derive LEFT/RIGHT decision."""
    for attempt in range(1, retries + 1):
        print(f"[TASK2] {stage_name}: capturing photo (attempt {attempt}/{retries})")
        image_jpeg = camera.capture_jpeg()
        if not image_jpeg:
            print(f"[TASK2] {stage_name}: capture failed, retrying...")
            continue

        response = pc.request_inference(image_jpeg, timeout_s=inference_timeout)
        if not response:
            print(f"[TASK2] {stage_name}: no inference response, retrying...")
            continue

        if not response.get("ok", True):
            print(f"[TASK2] {stage_name}: PC returned error: {response.get('error')}")
            continue

        boxes_raw = response.get("boxes", [])
        if not isinstance(boxes_raw, list):
            print(f"[TASK2] {stage_name}: invalid response boxes type")
            continue

        boxes = [box for box in boxes_raw if isinstance(box, dict)]
        decision = decide_direction_from_boxes(boxes)
        print(f"[TASK2] {stage_name}: received {len(boxes)} box(es), decision={decision}")
        if decision:
            return decision

        print(f"[TASK2] {stage_name}: no LEFT/RIGHT found, retrying...")

    print(f"[TASK2] {stage_name}: failed to get LEFT/RIGHT after {retries} attempt(s)")
    return None


def run(
    bt: BluetoothIface,
    stm: STM32Interface,
    pc: PCInterface,
    camera: SnapshotCamera,
    retries: int,
    inference_timeout: float,
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

        # 2) First obstacle photo inference and turn
        first_direction = detect_direction_from_photo(
            camera=camera,
            pc=pc,
            stage_name="First obstacle",
            retries=retries,
            inference_timeout=inference_timeout,
        )
        if first_direction is None:
            print("[TASK2] Aborting run (first obstacle decision failed)")
            continue

        first_cmd = CMD_FIRST_LEFT if first_direction == "LEFT" else CMD_FIRST_RIGHT
        print(f"[TASK2] First obstacle decision: {first_direction} -> {first_cmd}")
        if not send_and_wait_done(stm, first_cmd):
            print("[TASK2] Aborting run (step 3 failed)")
            continue

        # 3) Second obstacle photo inference and turn
        second_direction = detect_direction_from_photo(
            camera=camera,
            pc=pc,
            stage_name="Second obstacle",
            retries=retries,
            inference_timeout=inference_timeout,
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


def _cleanup(
    stm: Optional[STM32Interface] = None,
    cam: Optional[SnapshotCamera] = None,
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
    parser.add_argument("--pc-request-port", type=int, default=PCInterface.DEFAULT_REQUEST_PORT,
                        help="PC inference REQ/REP port")
    parser.add_argument("--cam-width", type=int, default=SnapshotCamera.DEFAULT_WIDTH,
                        help="Camera capture width")
    parser.add_argument("--cam-height", type=int, default=SnapshotCamera.DEFAULT_HEIGHT,
                        help="Camera capture height")
    parser.add_argument("--cam-quality", type=int, default=SnapshotCamera.DEFAULT_QUALITY,
                        help="JPEG quality for snapshots")
    parser.add_argument("--cam-warmup", type=float, default=CAMERA_WARMUP,
                        help="Camera warmup time after start (seconds)")
    parser.add_argument("--bt-mode", choices=["socket", "serial"], default="serial",
                        help="Bluetooth backend: socket or serial")
    parser.add_argument("--retries", type=int, default=MAX_CAPTURE_RETRIES,
                        help="Max capture/inference retries per obstacle")
    parser.add_argument("--inference-timeout", type=float, default=INFERENCE_TIMEOUT,
                        help="Timeout waiting for PC inference response (seconds)")
    args = parser.parse_args()

    # STM32
    print("[INIT] Connecting to STM32 ...")
    stm = STM32Interface()
    if not stm.is_connected:
        print("[INIT] STM32 not found. Exiting.")
        sys.exit(1)
    print("[INIT] STM32 connected")

    # Camera (snapshot mode)
    cam = SnapshotCamera(
        width=args.cam_width,
        height=args.cam_height,
        quality=max(1, min(100, args.cam_quality)),
        warmup_s=max(0.0, args.cam_warmup),
    )
    if not cam.start():
        print("[INIT] Snapshot camera failed. Exiting.")
        _cleanup(stm)
        sys.exit(1)

    # PC inference request client
    pc = PCInterface(
        host=args.pc_host,
        request_port=args.pc_request_port,
    )
    if not pc.start_request_client():
        print("[INIT] PC inference client failed. Exiting.")
        _cleanup(stm, cam=cam)
        sys.exit(1)
    print("[INIT] PC inference client running")

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

    try:
        run(
            bt=bt,
            stm=stm,
            pc=pc,
            camera=cam,
            retries=max(1, args.retries),
            inference_timeout=max(0.1, args.inference_timeout),
        )
    except KeyboardInterrupt:
        print("\n[TASK2] Interrupted")
    finally:
        _cleanup(stm, cam=cam, pc=pc, bt=bt)
        print("[TASK2] Shutdown complete")


if __name__ == "__main__":
    main()

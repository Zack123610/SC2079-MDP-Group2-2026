"""
Obstacle A5 – Image identification around an obstacle.

Standalone program for the "identify the image on an obstacle" task.

Flow:
  1. Start camera streaming (-> PC) and PC detection listener.
  2. Connect to STM32.
  3. User types "1000" to move the robot forward towards an obstacle.
  4. STM32 responds "DONE" (arrived) or "OBST" (obstacle detected).
  5. While the robot is moving (and at all times), image recognition runs
     continuously.  The program keeps a rolling window of the last 10
     detected class names.
  6. After STM32 responds, evaluate the window:
       - If 8/10 agree AND the majority class is NOT "Obstacle" AND
         cls_id is 0–30 → image identified.  Report and wait for user.
       - If the majority is "Obstacle" (blank/unknown face) →
         send "8000" (rotate 90°) to look at the next face.
  7. After "8000" + STM "DONE", re-evaluate.  Repeat up to 3 times
     (4 faces total: the original + 3 rotations).
  8. After exhausting all faces, report failure and wait for user.

Run on Raspberry Pi:
    python obstacle_a5.py --pc-host <PC_IP>

Dependencies:
    Same as main.py plus pyzmq (for camera / PC link).
"""

import argparse
import collections
import sys
import threading
import time
from typing import Any, Optional

from camera_interface import CameraInterface
from pc_interface import PCInterface
from stm32_interface import STM32Interface


# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

CMD_FORWARD  = "1000"   # move forward
CMD_ROTATE   = "8000"   # rotate 90° and search next face
MAX_ROTATES  = 3        # max "8000" commands (3 rotations = 4 faces total)
WINDOW_SIZE  = 10       # rolling detection window
CONFIDENCE_THRESHOLD = 8  # need 8/10 agreeing detections to be "certain"
VALID_CLS_RANGE = range(10, 42)  # image IDs 10–41 are valid

STM_RECV_TIMEOUT = 30.0  # seconds to wait for STM response
DETECTION_WAIT   = 3.0   # seconds to wait for detections on each face


# ---------------------------------------------------------------------------
# Image ID extraction
# ---------------------------------------------------------------------------


def extract_image_id(cls_name: Optional[str]) -> Optional[int]:
    """
    Extract the numeric image ID from a class name in ``"Name-id-XX"`` format.

    Example: ``"One-id-11"`` → ``11``

    Returns None if the name doesn't match the expected pattern.
    """
    if not cls_name or "-id-" not in cls_name:
        return None
    try:
        return int(cls_name.rsplit("-id-", 1)[1])
    except (ValueError, IndexError):
        return None


# ---------------------------------------------------------------------------
# Detection tracker
# ---------------------------------------------------------------------------

class DetectionTracker:
    """
    Thread-safe rolling window of the last N detected class names,
    continuously fed by the PCInterface background listener.
    """

    def __init__(self, window_size: int = WINDOW_SIZE) -> None:
        self._window: collections.deque[dict[str, Any]] = collections.deque(
            maxlen=window_size,
        )
        self._lock = threading.Lock()
        self._window_size = window_size

    def push(self, detection: dict[str, Any]) -> None:
        with self._lock:
            self._window.append(detection)

    def clear(self) -> None:
        with self._lock:
            self._window.clear()

    @property
    def count(self) -> int:
        with self._lock:
            return len(self._window)

    def evaluate(self) -> tuple[Optional[str], Optional[int], int, int]:
        """
        Analyse the current window.

        Returns:
            (majority_cls_name, majority_cls_id, majority_count, window_len)

            majority_cls_name / _id may be None if the window is empty.
        """
        with self._lock:
            if not self._window:
                return (None, None, 0, 0)

            counter: dict[str, list[dict[str, Any]]] = {}
            for det in self._window:
                name = det.get("cls_name", "unknown")
                counter.setdefault(name, []).append(det)

            best_name = max(counter, key=lambda k: len(counter[k]))
            best_dets = counter[best_name]
            best_id = extract_image_id(best_name)

            return (best_name, best_id, len(best_dets), len(self._window))


# ---------------------------------------------------------------------------
# Background: drain PCInterface detections into the tracker
# ---------------------------------------------------------------------------

def _detection_collector(
    pc: PCInterface,
    tracker: DetectionTracker,
    stop_event: threading.Event,
) -> None:
    """Drain detections from PCInterface and push into the tracker."""
    while not stop_event.is_set():
        det = pc.get_detection()
        if det:
            tracker.push(det)
        else:
            time.sleep(0.02)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def send_and_wait(stm: STM32Interface, cmd: str) -> Optional[str]:
    """
    Send *cmd* to STM32 and poll for a response.

    Polls with a short interval (0.1 s) so we react immediately when
    data arrives, rather than blocking for the full timeout.
    """
    print(f"\n[A5] Sending '{cmd}' to STM32...")
    if not stm.send(cmd, add_newline=False):
        print("[A5] Failed to send command to STM32!")
        return None

    print(f"[A5] Waiting for STM32 response (timeout={STM_RECV_TIMEOUT}s)...")
    start = time.time()
    while time.time() - start < STM_RECV_TIMEOUT:
        response = stm.receive(timeout=0.1)
        if response:
            elapsed = time.time() - start
            print(f"[A5] STM32 responded: '{response}' ({elapsed:.1f}s)")
            return response
    print("[A5] No response from STM32 (timeout)")
    return None


def print_window(tracker: DetectionTracker) -> None:
    """Pretty-print the current detection window evaluation."""
    name, cls_id, count, total = tracker.evaluate()
    if total == 0:
        print("[A5] Detection window: empty (no detections yet)")
        return
    bar = "#" * count + "." * (total - count)
    print(f"[A5] Detection window ({total}): "
          f"majority='{name}' (id={cls_id}), "
          f"count={count}/{total} [{bar}]")


def is_valid_image(cls_name: Optional[str], image_id: Optional[int]) -> bool:
    """
    True if the detection is a valid target image.

    Invalid when the name has no ``-id-`` suffix (e.g. plain ``"Obstacle"``),
    or the extracted ID is outside the 0–30 range.
    """
    if cls_name is None or image_id is None:
        return False
    if "obstacle" in cls_name.lower():
        return False
    if "-id-" not in cls_name:
        return False
    if image_id not in VALID_CLS_RANGE:
        return False
    return True


# ---------------------------------------------------------------------------
# Main routine
# ---------------------------------------------------------------------------

def run(stm: STM32Interface, tracker: DetectionTracker) -> None:
    """Interactive loop: wait for user commands, execute the A5 flow."""
    print("\n" + "=" * 60)
    print("  Obstacle A5 – Image Identification")
    print("=" * 60)
    print("Commands:")
    print(f"  {CMD_FORWARD}  – Move robot forward towards obstacle")
    print("  quit  – Exit the program")
    print()

    while True:
        try:
            user_input = input("[A5] Enter command> ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            break

        if user_input.lower() in ("quit", "exit", "q"):
            break

        if user_input != CMD_FORWARD:
            print(f"[A5] Unknown command '{user_input}'. "
                  f"Use '{CMD_FORWARD}' to start or 'quit' to exit.")
            continue

        # --- Phase 1: move forward ----------------------------------------
        tracker.clear()
        response = send_and_wait(stm, CMD_FORWARD)
        if response is None:
            print("[A5] STM32 did not respond. Try again.")
            continue

        if response not in ("0000","DONE", "OBST"):
            print(f"[A5] Unexpected STM32 response: '{response}'. Continuing anyway.")

        # --- Phase 2: evaluate faces, rotate if needed --------------------
        identified = False
        rotate_count = 0

        while rotate_count <= MAX_ROTATES:
            # Wait a fixed period for detections to accumulate on this face
            print(f"[A5] Collecting detections for {DETECTION_WAIT}s...")
            time.sleep(DETECTION_WAIT)

            name, cls_id, count, total = tracker.evaluate()
            print_window(tracker)

            if total == 0:
                # Blank face – YOLO detected nothing at all
                print("[A5] No detections on this face (blank).")
            elif total >= WINDOW_SIZE and count >= CONFIDENCE_THRESHOLD:
                if is_valid_image(name, cls_id):
                    print(f"\n[A5] *** IMAGE IDENTIFIED: '{name}' "
                          f"(cls_id={cls_id}, {count}/{total} confidence) ***\n")
                    identified = True
                    break
                else:
                    print(f"[A5] Majority is '{name}' – not a valid image.")
            elif total < WINDOW_SIZE:
                print(f"[A5] Too few detections ({total}/{WINDOW_SIZE}). "
                      "Treating as inconclusive.")
            else:
                print(f"[A5] No consensus ({count}/{total}). "
                      "Treating as inconclusive.")

            # Can we still rotate?
            if rotate_count >= MAX_ROTATES:
                print(f"[A5] Already rotated {MAX_ROTATES} times "
                      "(all 4 faces checked). Giving up.")
                break

            # --- Rotate 90° ----------------------------------------------
            rotate_count += 1
            print(f"\n[A5] Rotating to next face ({rotate_count}/{MAX_ROTATES})...")
            tracker.clear()
            resp = send_and_wait(stm, CMD_ROTATE)
            if resp is None:
                print("[A5] STM32 did not respond to rotate. Aborting.")
                break

        if not identified:
            print("[A5] Could not identify a valid image on any face.")

        print("[A5] Ready for next command.\n")

    print("[A5] Exiting obstacle_a5.")


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(description="Obstacle A5 – Image Identification")
    parser.add_argument("--pc-host", required=True, help="PC IP address (for detection results)")
    parser.add_argument("--pc-port", type=int, default=5556, help="PC detection PUB port")
    parser.add_argument("--cam-port", type=int, default=5555, help="Camera stream PUB port")
    parser.add_argument("--no-camera", action="store_true",
                        help="Skip camera streaming (useful if pi_streamer is already running)")
    args = parser.parse_args()

    # --- STM32 ------------------------------------------------------------
    print("[STM32] Connecting...")
    stm = STM32Interface()
    if not stm.is_connected:
        print("[STM32] Failed to connect. Exiting.")
        sys.exit(1)
    print("[STM32] Connected")

    # --- Camera (optional) ------------------------------------------------
    cam: Optional[CameraInterface] = None
    if not args.no_camera:
        cam = CameraInterface(port=args.cam_port)
        if not cam.start():
            print("[CAM] Failed to start camera. Continuing without streaming.")
            cam = None
        else:
            print("[CAM] Streaming to PC")

    # --- PC detection listener --------------------------------------------
    pc = PCInterface(host=args.pc_host, port=args.pc_port)
    if not pc.start():
        print("[PC] Failed to start detection listener. Exiting.")
        if cam:
            cam.stop()
        stm.close()
        sys.exit(1)
    print("[PC] Listening for detections")

    # --- Detection tracker ------------------------------------------------
    tracker = DetectionTracker()
    stop_event = threading.Event()
    collector_thread = threading.Thread(
        target=_detection_collector,
        args=(pc, tracker, stop_event),
        daemon=True,
    )
    collector_thread.start()

    # --- Run the interactive loop -----------------------------------------
    try:
        run(stm, tracker)
    except KeyboardInterrupt:
        print("\n[A5] Interrupted.")
    finally:
        stop_event.set()
        collector_thread.join(timeout=2.0)
        pc.stop()
        if cam:
            cam.stop()
        stm.close()
        print("[A5] Cleanup complete.")


if __name__ == "__main__":
    main()

"""
Task 1 – Obstacle Navigation and Image Recognition

Full autonomous flow:
  1. Android sends ROBOT position, OBSTACLE list, then BEGIN.
  2. RPi parses everything and POSTs to the algo service on PC:5001.
  3. For each segment the algo returns, RPi walks the instructions:
       a. Translate each instruction into BOTH an STM command and an
          Android UI command, send them, and wait for STM to acknowledge
          ("0000" or "DONE") before moving on.
       b. On CAPTURE_IMAGE, run DetectionTracker majority-vote window
          and send  TARGET,<ObstacleNumber>,<TargetID>  to Android.
  4. After all segments (or a 6-minute timeout), send "0000" (stop) to
     STM and loop back, waiting for the next batch of obstacles.

Android → RPi message examples:
    ROBOT,0,0,NORTH
    OBSTACLE,1,120,120,NORTH       (x_raw=120 → grid x=12)
    OBSTACLE,2,90,80,SOUTH
    BEGIN

Run on Raspberry Pi:
    python task1.py --pc-host <PC_IP>
"""

import argparse
import sys
import threading
import time
from typing import Any, Optional

from algo_interface import AlgoInterface
from bluetooth_interface import BluetoothInterface
from camera_interface import CameraInterface
from obstacle_a5 import (
    CONFIDENCE_THRESHOLD,
    DETECTION_WAIT,
    WINDOW_SIZE,
    DetectionTracker,
    _detection_collector,
    is_valid_image,
)
from pc_interface import PCInterface
from stm32_interface import STM32Interface

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

STM_RECV_TIMEOUT = 30.0   # max seconds to wait for one STM response
RUN_TIMEOUT      = 360.0  # 6-minute cap per run

# Algo string instructions → STM32 4-char commands
STM_STRING_CMD: dict[str, str] = {
    "LEFT":           "3000",
    "RIGHT":          "4000",
    "FORWARD_LEFT":   "6000",
    "FORWARD_RIGHT":  "7000",
    "BACKWARD_LEFT":  "8000",
    "BACKWARD_RIGHT": "9000",
}

# Algo string instructions → Android UI commands
ANDROID_TURN_CMD: dict[str, str] = {
    "LEFT":           "STAT_TURN,LEFT",
    "RIGHT":          "STAT_TURN,RIGHT",
    "FORWARD_LEFT":   "TURN,FORWARD_LEFT",
    "FORWARD_RIGHT":  "TURN,FORWARD_RIGHT",
    "BACKWARD_LEFT":  "TURN,BACKWARD_LEFT",
    "BACKWARD_RIGHT": "TURN,BACKWARD_RIGHT",
}

# Direction-code prefix for dict-based move instructions
MOVE_DIR_CODE: dict[str, str] = {
    "FORWARD":  "1",
    "BACKWARD": "2",
}

# ---------------------------------------------------------------------------
# Parsing
# ---------------------------------------------------------------------------


def parse_robot(msg: str) -> Optional[dict[str, Any]]:
    """
    Parse ``ROBOT,<x>,<y>,<direction>`` from Android.

    Returns an algo-service-compatible robot dict.
    """
    parts = [p.strip() for p in msg.split(",")]
    if len(parts) != 4 or parts[0].upper() != "ROBOT":
        return None
    try:
        direction = parts[3].upper()
    except IndexError:
        return None
    return {
        "direction": direction,
        "south_west": {"x": 0, "y": 0},
        "north_east": {"x": 1, "y": 1},
    }


def parse_obstacle(msg: str) -> Optional[dict[str, Any]]:
    """
    Parse ``OBSTACLE,<image_id>,<x_raw>,<y_raw>,<direction>`` from Android.

    Raw coordinates are divided by 10 to convert to grid units
    (e.g. 120 → 12).
    """
    parts = [p.strip() for p in msg.split(",")]
    if len(parts) != 5 or parts[0].upper() != "OBSTACLE":
        return None
    try:
        image_id  = int(parts[1])
        x         = int(parts[2]) // 10
        y         = int(parts[3]) // 10
        direction = parts[4].upper()
    except (ValueError, IndexError):
        return None
    return {
        "image_id": image_id,
        "direction": direction,
        "south_west": {"x": x, "y": y},
        "north_east": {"x": x + 1, "y": y + 1},
    }


# ---------------------------------------------------------------------------
# Instruction → (STM command, Android command)
# ---------------------------------------------------------------------------


def instruction_to_commands(
    instruction: Any,
) -> tuple[Optional[str], Optional[str]]:
    """
    Convert one algo instruction into ``(stm_cmd, android_cmd)``.

    Returns ``(None, None)`` for ``CAPTURE_IMAGE`` (handled separately).
    """
    if isinstance(instruction, dict):
        move   = instruction.get("move", "").upper()
        amount = instruction.get("amount", 0)
        code   = MOVE_DIR_CODE.get(move)
        if code is None:
            print(f"[TASK1] Unknown move type: {move}")
            return (None, None)
        stm_cmd     = f"{code}{amount:03d}"
        android_cmd = f"MOVE,{amount},{move}"
        return (stm_cmd, android_cmd)

    if isinstance(instruction, str):
        upper = instruction.upper()
        if upper == "CAPTURE_IMAGE":
            return (None, None)
        stm_cmd     = STM_STRING_CMD.get(upper)
        android_cmd = ANDROID_TURN_CMD.get(upper)
        if stm_cmd is None:
            print(f"[TASK1] Unknown string instruction: {instruction}")
        return (stm_cmd, android_cmd)

    return (None, None)


# ---------------------------------------------------------------------------
# STM32 send + wait
# ---------------------------------------------------------------------------


def send_and_wait_stm(stm: STM32Interface, cmd: str) -> Optional[str]:
    """Send *cmd* to STM32 and poll until it replies ``0000`` or ``DONE``."""
    print(f"[TASK1] → STM32: '{cmd}'")
    if not stm.send(cmd, add_newline=False):
        print("[TASK1] Failed to send to STM32")
        return None

    start = time.time()
    while time.time() - start < STM_RECV_TIMEOUT:
        response = stm.receive(timeout=0.1)
        if response:
            elapsed = time.time() - start
            print(f"[TASK1] ← STM32: '{response}' ({elapsed:.1f}s)")
            return response

    print(f"[TASK1] STM32 response timed out ({STM_RECV_TIMEOUT}s)")
    return None


# ---------------------------------------------------------------------------
# Image capture (DetectionTracker majority vote)
# ---------------------------------------------------------------------------


def capture_image(tracker: DetectionTracker, obstacle_id: int) -> Optional[int]:
    """
    Clear the detection window, wait for fresh detections, then evaluate.

    Returns the target ``cls_id`` if valid (not "Obstacle", id 0–30), else None.
    """
    tracker.clear()

    print(f"[TASK1] CAPTURE_IMAGE – obstacle {obstacle_id}, "
          f"collecting for {DETECTION_WAIT}s …")
    time.sleep(DETECTION_WAIT)

    name, cls_id, count, total = tracker.evaluate()

    if total == 0:
        print(f"[TASK1] No detections for obstacle {obstacle_id}")
        return None

    bar = "#" * count + "." * (total - count)
    print(f"[TASK1] Window ({total}): majority='{name}' id={cls_id} "
          f"count={count}/{total} [{bar}]")

    if count >= CONFIDENCE_THRESHOLD and is_valid_image(name, cls_id):
        print(f"[TASK1] *** IDENTIFIED: obstacle {obstacle_id} "
              f"→ target {cls_id} ***")
        return cls_id

    print(f"[TASK1] Inconclusive for obstacle {obstacle_id}")
    return None


# ---------------------------------------------------------------------------
# Execute one segment (one obstacle)
# ---------------------------------------------------------------------------


def execute_segment(
    segment: dict[str, Any],
    stm: STM32Interface,
    bt: BluetoothInterface,
    tracker: DetectionTracker,
    run_start: float,
) -> bool:
    """
    Execute every instruction in one algo segment.

    For each movement/turn:
      1. Send the Android UI command to Android.
      2. Send the STM command to STM32 and wait for its acknowledgement.

    For CAPTURE_IMAGE:
      Run DetectionTracker evaluation and report TARGET to Android.

    Returns False if the 6-minute run timeout was hit or STM became
    unresponsive, True otherwise.
    """
    obstacle_id  = segment.get("image_id", "?")
    instructions = segment.get("instructions", [])

    print(f"\n[TASK1] ══ Segment: obstacle {obstacle_id} "
          f"({len(instructions)} instructions) ══")

    for instruction in instructions:
        # --- Check 6-minute timeout ---
        elapsed = time.time() - run_start
        if elapsed > RUN_TIMEOUT:
            print(f"[TASK1] 6-minute timeout reached ({elapsed:.0f}s)")
            return False

        # --- CAPTURE_IMAGE ---
        if isinstance(instruction, str) and instruction.upper() == "CAPTURE_IMAGE":
            target_id = capture_image(tracker, obstacle_id)
            if target_id is not None:
                target_msg = f"TARGET,{obstacle_id},{target_id}"
                print(f"[TASK1] → Android: {target_msg}")
                bt.send(target_msg)
            else:
                print(f"[TASK1] Obstacle {obstacle_id}: identification failed")
            continue

        # --- Movement / Turn ---
        stm_cmd, android_cmd = instruction_to_commands(instruction)
        if stm_cmd is None:
            print(f"[TASK1] Skipping unrecognised instruction: {instruction}")
            continue

        # 1) Notify Android (UI update)
        if android_cmd:
            print(f"[TASK1] → Android: {android_cmd}")
            bt.send(android_cmd)

        # 2) Execute on STM and wait for ack
        response = send_and_wait_stm(stm, stm_cmd)
        if response is None:
            print("[TASK1] STM32 unresponsive – aborting segment")
            return False

    print(f"[TASK1] ══ Segment for obstacle {obstacle_id} complete ══")
    return True


# ---------------------------------------------------------------------------
# Main loop
# ---------------------------------------------------------------------------


def run(
    bt: BluetoothInterface,
    stm: STM32Interface,
    algo: AlgoInterface,
    tracker: DetectionTracker,
) -> None:
    """
    Top-level event loop.

    Collects ROBOT + OBSTACLE messages from Android, then on BEGIN
    requests a path from the algo service and executes all segments.
    After completion (or 6-min timeout), sends ``0000`` to STM and
    waits for the next batch.
    """
    print("\n" + "=" * 60)
    print("  Task 1 – Obstacle Navigation & Image Recognition")
    print("=" * 60)
    print("Waiting for commands from Android …")
    print("  ROBOT,<x>,<y>,<direction>")
    print("  OBSTACLE,<id>,<x>,<y>,<direction>")
    print("  BEGIN\n")

    robot_cfg: dict[str, Any] = {
        "direction": "NORTH",
        "south_west": {"x": 0, "y": 0},
        "north_east": {"x": 1, "y": 1},
    }
    obstacles: list[dict[str, Any]] = []

    while True:
        msg = bt.readline()
        if not msg:
            time.sleep(0.05)
            continue

        msg = msg.strip()
        print(f"[TASK1] ← Android: '{msg}'")

        token = msg.upper().replace(" ", "")

        # ---- ROBOT position / direction ----------------------------------
        if token.startswith("ROBOT"):
            parsed = parse_robot(msg)
            if parsed:
                robot_cfg = parsed
                print(f"[TASK1] Robot direction set to {robot_cfg['direction']}")
            else:
                print(f"[TASK1] Bad ROBOT format: '{msg}'")
            continue

        # ---- OBSTACLE registration ---------------------------------------
        if token.startswith("OBSTACLE"):
            obs = parse_obstacle(msg)
            if obs:
                obstacles.append(obs)
                print(f"[TASK1] Obstacle {obs['image_id']} → "
                      f"grid ({obs['south_west']['x']},{obs['south_west']['y']}) "
                      f"facing {obs['direction']}  [{len(obstacles)} total]")
            else:
                print(f"[TASK1] Bad OBSTACLE format: '{msg}'")
            continue

        # ---- BEGIN navigation --------------------------------------------
        if token == "BEGIN":
            if not obstacles:
                print("[TASK1] No obstacles registered – ignoring BEGIN")
                continue

            print(f"\n[TASK1] ▶ BEGIN with {len(obstacles)} obstacle(s), "
                  f"robot facing {robot_cfg['direction']}")

            # Request path from algo service
            result = algo.request_path(obstacles, robot=robot_cfg)
            if not result or not result.get("segments"):
                print("[TASK1] Algo returned no path")
                continue

            segments = result["segments"]
            print(f"[TASK1] Algo returned {len(segments)} segment(s)\n")

            # Execute all segments
            run_start = time.time()
            for seg in segments:
                if not execute_segment(seg, stm, bt, tracker, run_start):
                    break

            # Stop the robot
            elapsed = time.time() - run_start
            print(f"\n[TASK1] Run finished ({elapsed:.1f}s) – sending STOP")
            stm.send("0000", add_newline=False)

            # Reset for next run
            obstacles.clear()
            robot_cfg = {
                "direction": "NORTH",
                "south_west": {"x": 0, "y": 0},
                "north_east": {"x": 1, "y": 1},
            }
            print("[TASK1] Waiting for next set of obstacles …\n")
            continue

        print(f"[TASK1] Unrecognised message: '{msg}'")


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------


def main() -> None:
    parser = argparse.ArgumentParser(description="Task 1 – Obstacle Navigation")
    parser.add_argument("--pc-host", required=True,
                        help="PC IP address")
    parser.add_argument("--algo-port", type=int, default=5001,
                        help="Algo service port on PC")
    parser.add_argument("--pc-port", type=int, default=5556,
                        help="PC detection ZMQ PUB port")
    parser.add_argument("--cam-port", type=int, default=5555,
                        help="Camera stream ZMQ PUB port")
    parser.add_argument("--no-camera", action="store_true",
                        help="Skip camera (if pi_streamer already running)")
    parser.add_argument("--pybluez", action="store_true",
                        help="Use legacy PyBluez RFCOMM server for Bluetooth")
    args = parser.parse_args()

    # --- STM32 ----------------------------------------------------------------
    print("[INIT] Connecting to STM32 …")
    stm = STM32Interface()
    if not stm.is_connected:
        print("[INIT] STM32 not found. Exiting.")
        sys.exit(1)
    print("[INIT] STM32 connected")

    # --- Camera ---------------------------------------------------------------
    cam: Optional[CameraInterface] = None
    if not args.no_camera:
        cam = CameraInterface(port=args.cam_port)
        if not cam.start():
            print("[INIT] Camera failed – continuing without streaming")
            cam = None
        else:
            print("[INIT] Camera streaming")

    # --- PC detection listener ------------------------------------------------
    pc = PCInterface(host=args.pc_host, port=args.pc_port)
    if not pc.start():
        print("[INIT] PC detection listener failed. Exiting.")
        _cleanup(stm, cam=cam)
        sys.exit(1)
    print("[INIT] PC detection listener running")

    # --- Algo service ---------------------------------------------------------
    algo = AlgoInterface(host=args.pc_host, port=args.algo_port)
    algo.start()

    # --- Bluetooth ------------------------------------------------------------
    bt = BluetoothInterface(use_pybluez_server=args.pybluez)
    if not bt.start():
        print("[INIT] Bluetooth failed. Exiting.")
        _cleanup(stm, cam=cam, pc=pc, algo=algo)
        sys.exit(1)
    print("[INIT] Bluetooth connected")

    # --- Detection tracker (background drain) ---------------------------------
    tracker = DetectionTracker(window_size=WINDOW_SIZE)
    stop_event = threading.Event()
    collector = threading.Thread(
        target=_detection_collector,
        args=(pc, tracker, stop_event),
        daemon=True,
    )
    collector.start()

    # --- Main loop ------------------------------------------------------------
    try:
        run(bt, stm, algo, tracker)
    except KeyboardInterrupt:
        print("\n[TASK1] Interrupted")
    finally:
        stop_event.set()
        collector.join(timeout=2.0)
        _cleanup(stm, cam=cam, pc=pc, algo=algo, bt=bt)
        print("[TASK1] Shutdown complete")


def _cleanup(
    stm: Optional[STM32Interface] = None,
    cam: Optional[CameraInterface] = None,
    pc: Optional[PCInterface] = None,
    algo: Optional[AlgoInterface] = None,
    bt: Optional[BluetoothInterface] = None,
) -> None:
    if bt:
        bt.close()
    if algo:
        algo.stop()
    if pc:
        pc.stop()
    if cam:
        cam.stop()
    if stm:
        stm.close()


if __name__ == "__main__":
    main()

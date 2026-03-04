"""
Task 1 – Obstacle Navigation and Image Recognition

Execution model (experimental – batch send):
  1. Android sends ROBOT position, OBSTACLE list, then BEGIN.
  2. RPi parses everything and POSTs to the algo service on PC:5001.
  3. RPi flattens ALL instructions from ALL segments into one list,
     translating each into a 4-char STM command (CAPTURE_IMAGE → "5000")
     and a parallel Android UI command.
  4. The full STM command string is concatenated and sent to STM in one
     shot.  STM slices every 4 bytes and executes sequentially.
  5. RPi enters a response loop:
       - "DONE" → send the indexed Android command to update the UI.
       - "HALT" (from "5000") → run DetectionTracker, send TARGET to
         Android, then send "RESM" to STM to resume.
  6. After all instructions (or 6-minute timeout), send "0000" to STM
     and loop back for the next batch.

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

    CAPTURE_IMAGE → ``("5000", None)`` — the Android message is
    determined at runtime after image recognition.
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
            return ("5000", None)
        stm_cmd     = STM_STRING_CMD.get(upper)
        android_cmd = ANDROID_TURN_CMD.get(upper)
        if stm_cmd is None:
            print(f"[TASK1] Unknown string instruction: {instruction}")
        return (stm_cmd, android_cmd)

    return (None, None)


# ---------------------------------------------------------------------------
# Build flat command lists from algo segments
# ---------------------------------------------------------------------------


def build_command_lists(
    segments: list[dict[str, Any]],
) -> tuple[list[str], list[Optional[str]], dict[int, int]]:
    """
    Flatten all algo segments into parallel command lists.

    Returns:
        stm_commands:    List of 4-char STM commands.
        android_commands: Parallel list of Android UI strings (None for
                         CAPTURE_IMAGE entries — filled at runtime).
        capture_map:     ``{ index: obstacle_id }`` for every CAPTURE_IMAGE.
    """
    stm_commands: list[str] = []
    android_commands: list[Optional[str]] = []
    capture_map: dict[int, int] = {}

    for segment in segments:
        obstacle_id = segment.get("image_id", 0)
        for instruction in segment.get("instructions", []):
            stm_cmd, android_cmd = instruction_to_commands(instruction)
            if stm_cmd is None:
                print(f"[TASK1] Skipping unrecognised instruction: {instruction}")
                continue
            idx = len(stm_commands)
            stm_commands.append(stm_cmd)
            android_commands.append(android_cmd)
            if stm_cmd == "5000":
                capture_map[idx] = obstacle_id

    return stm_commands, android_commands, capture_map


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
# Wait for one STM response
# ---------------------------------------------------------------------------


def wait_for_stm(stm: STM32Interface) -> Optional[str]:
    """Poll STM32 for a single response (``DONE`` or ``HALT``)."""
    start = time.time()
    while time.time() - start < STM_RECV_TIMEOUT:
        response = stm.receive(timeout=0.1)
        if response:
            elapsed = time.time() - start
            print(f"[TASK1] ← STM32: '{response}' ({elapsed:.1f}s)")
            return response.strip()
    print(f"[TASK1] STM32 response timed out ({STM_RECV_TIMEOUT}s)")
    return None


# ---------------------------------------------------------------------------
# Execute all commands (batch-send model)
# ---------------------------------------------------------------------------


def _build_stm_payload(stm_commands: list[str]) -> str:
    """
    Frame the concatenated STM commands for transmission.

    Format:  ``<cmd1cmd2...cmdN>CC``

    where CC is a checksum: ``(sum of all digit chars in the commands) % 100``.

    Example:
        commands = ["6030", "2020", "6033"]
        body     = "603020206033"
        checksum = (6+0+3+0+2+0+2+0+6+0+3+3) % 100 = 25
        payload  = "<603020206033>25"
    """
    body = "".join(stm_commands)
    checksum = sum(int(ch) for ch in body if ch.isdigit()) % 100
    return f"<{body}>{checksum}"


def execute_all(
    stm: STM32Interface,
    bt: BluetoothInterface,
    tracker: DetectionTracker,
    stm_commands: list[str],
    android_commands: list[Optional[str]],
    capture_map: dict[int, int],
    run_start: float,
) -> bool:
    """
    Send the full framed STM command string, then process
    responses one-by-one.

    Returns True if all commands completed, False on timeout / error.
    """
    # --- Batch send ---
    stm_payload = _build_stm_payload(stm_commands)
    print(f"[TASK1] → STM32 (batch): '{stm_payload}' "
          f"({len(stm_commands)} commands)")
    if not stm.send(stm_payload, add_newline=False):
        print("[TASK1] Failed to send batch to STM32")
        return False

    # --- Response loop ---
    idx = 0
    while idx < len(stm_commands):
        # 6-minute timeout check
        elapsed = time.time() - run_start
        if elapsed > RUN_TIMEOUT:
            print(f"[TASK1] 6-minute timeout reached ({elapsed:.0f}s)")
            return False

        response = wait_for_stm(stm)
        if response is None:
            print("[TASK1] STM32 unresponsive – aborting")
            return False

        if response == "HALT":
            # --- CAPTURE_IMAGE: STM is paused, waiting for RESM ---
            obstacle_id = capture_map.get(idx, 0)
            print(f"[TASK1] HALT received – running image capture "
                  f"for obstacle {obstacle_id}")

            target_id = capture_image(tracker, obstacle_id)
            if target_id is not None:
                target_msg = f"TARGET,{obstacle_id},{target_id}"
                print(f"[TASK1] → Android: {target_msg}")
                bt.send(target_msg)
            else:
                print(f"[TASK1] Obstacle {obstacle_id}: identification failed")

            # Resume STM
            print("[TASK1] → STM32: 'RESM'")
            stm.send("RESM", add_newline=False)
            idx += 1

        elif response == "DONE":
            # --- Normal instruction completed ---
            android_cmd = android_commands[idx]
            if android_cmd:
                print(f"[TASK1] → Android: {android_cmd}")
                bt.send(android_cmd)
            idx += 1

        else:
            print(f"[TASK1] Unexpected STM response: '{response}' "
                  f"(treating as DONE for idx {idx})")
            android_cmd = android_commands[idx]
            if android_cmd:
                print(f"[TASK1] → Android: {android_cmd}")
                bt.send(android_cmd)
            idx += 1

    print(f"[TASK1] All {len(stm_commands)} commands executed")
    return True


# ---------------------------------------------------------------------------
# Manual STM command mode
# ---------------------------------------------------------------------------


# def _manual_stm_mode(stm: STM32Interface) -> None:
#     """
#     Interactive prompt that lets the operator send raw commands to STM32.

#     Type a command (e.g. ``1030``, ``3000``) and press Enter.
#     Type ``quit`` or ``q`` to exit back to the Android listener loop.
#     """
#     print("\n" + "-" * 50)
#     print("  Manual STM command mode")
#     print("  Type a command to send to STM32 (e.g. 1030)")
#     print("  Type 'quit' or 'q' to return to Android listener")
#     print("-" * 50)

#     while True:
#         try:
#             cmd = input("[STM manual] > ").strip()
#         except (EOFError, KeyboardInterrupt):
#             print()
#             break

#         if cmd.lower() in ("quit", "exit", "q", ""):
#             break

#         print(f"[STM manual] → STM32: '{cmd}'")
#         stm.send(cmd, add_newline=False)

#         response = wait_for_stm(stm)
#         if response:
#             print(f"[STM manual] ← STM32: '{response}'")
#         else:
#             print("[STM manual] No response (timeout)")

#     print("[STM manual] Exiting manual mode\n")


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
    requests a path from the algo service, batch-sends all instructions
    to STM, and processes responses.  After completion (or 6-min timeout),
    sends ``0000`` to STM and waits for the next batch.
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

            # 1) Request path from algo service
            result = algo.request_path(obstacles, robot=robot_cfg)
            if not result or not result.get("segments"):
                print("[TASK1] Algo returned no path")
                continue

            segments = result["segments"]
            print(f"[TASK1] Algo returned {len(segments)} segment(s)")

            # 2) Build flat command lists
            stm_cmds, android_cmds, capture_map = build_command_lists(segments)
            print(f"[TASK1] Built {len(stm_cmds)} commands "
                  f"({len(capture_map)} captures)\n")

            # 3) Execute
            run_start = time.time()
            execute_all(
                stm, bt, tracker,
                stm_cmds, android_cmds, capture_map,
                run_start,
            )

            # 4) Stop the robot
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

            # Manual STM command mode until Android sends new obstacles
            # _manual_stm_mode(stm)
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
    parser.add_argument("--algo-port", type=int, default=15001,
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

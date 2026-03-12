"""
Task 1 – Obstacle Navigation and Image Recognition

Execution model (experimental – batch send):
  1. Android sends ROBOT position, OBSTACLE list, then BEGIN.
  2. RPi parses everything and POSTs to the algo service on PC
     (POST /path with new flat request format).
  3. Algo returns a flat list of commands (FW50, FR00, SNAP4_C, …).
     RPi translates each into a 4-char STM command and a parallel
     Android UI command, then batch-sends all STM commands in one
     framed payload.
  4. RPi enters a response loop:
       - "DONE" → send the indexed Android command to update the UI.
       - "HALT" (from "5000" / SNAP) → run DetectionTracker, send
         TARGET to Android, then send "RESM" to STM to resume.
  5. After all instructions (or 6-minute timeout), send "0000" to STM
     and loop back for the next batch.

Android → RPi message examples:
    ROBOT,0,0,NORTH
    OBSTACLE,1,120,120,NORTH       (x_raw=120 → grid x=12)
    OBSTACLE,2,90,80,SOUTH
    OBSTACLE,2,90,80,EAST          (upserts: updates obstacle 2 direction)
    CLEAR                          (resets all obstacles + robot state)
    BEGIN

Run on Raspberry Pi:
    python task1.py --pc-host <PC_IP>
    python task1.py --pc-host <PC_IP> --bt-mode serial   # use /dev/rfcomm0
    python task1.py --pc-host <PC_IP> --bt-mode socket   # native AF_BLUETOOTH (default)
"""

import argparse
import re
import sys
import threading
import time
from typing import Any, Optional, Union

from algo_interface import AlgoInterface
from bluetooth_interface import BluetoothInterface as BluetoothSerial
from bluetooth_interface_socket import BluetoothInterface as BluetoothSocket
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

BluetoothIface = Union[BluetoothSerial, BluetoothSocket]

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

STM_RECV_TIMEOUT = 30.0   # max seconds to wait for one STM response
RUN_TIMEOUT      = 360.0  # 6-minute cap per run

# Android direction string → algo service direction number
DIRECTION_MAP: dict[str, int] = {
    "NORTH": 0,
    "EAST":  2,
    "SOUTH": 4,
    "WEST":  6,
}

# ---------------------------------------------------------------------------
# Parsing
# ---------------------------------------------------------------------------


def parse_robot(msg: str) -> Optional[int]:
    """
    Parse ``ROBOT,<x>,<y>,<direction>`` from Android.

    Returns the robot direction as an algo-service int (0/2/4/6),
    or None on parse error.
    """
    parts = [p.strip() for p in msg.split(",")]
    if len(parts) != 4 or parts[0].upper() != "ROBOT":
        return None
    direction = DIRECTION_MAP.get(parts[3].upper())
    if direction is None:
        print(f"[TASK1] Unknown direction: {parts[3]}")
    return direction


def parse_obstacle(msg: str) -> Optional[dict[str, Any]]:
    """
    Parse ``OBSTACLE,<id>,<x_raw>,<y_raw>,<direction>`` from Android.

    Raw coordinates are divided by 10 to convert to grid units
    (e.g. 120 → 12).  Returns an algo-service-compatible obstacle dict.
    """
    parts = [p.strip() for p in msg.split(",")]
    if len(parts) != 5 or parts[0].upper() != "OBSTACLE":
        return None
    try:
        obstacle_number = int(parts[1])
        x = int(parts[2]) // 10
        y = int(parts[3]) // 10
        d = DIRECTION_MAP.get(parts[4].upper())
        if d is None:
            print(f"[TASK1] Unknown direction: {parts[4]}")
            return None
    except (ValueError, IndexError):
        return None
    return {
        "x": x,
        "y": y,
        "d": d,
        "obstacleNumber": obstacle_number,
    }


# ---------------------------------------------------------------------------
# Algo instruction → (STM command, Android command)
# ---------------------------------------------------------------------------

# Regex to split a movement command into prefix + numeric suffix
_CMD_RE = re.compile(r"^([A-Z]+)(\d+)$", re.IGNORECASE)
# SNAP commands: SNAP4_C, SNAP5_R, or legacy SNAP1
_SNAP_RE = re.compile(r"^SNAP(\d+)(?:_([A-Z]))?$", re.IGNORECASE)


def instruction_to_commands(
    instruction: str,
) -> tuple[Optional[str], Optional[str]]:
    """
    Convert one algo instruction string into ``(stm_cmd, android_cmd)``.

    Algo format   →  STM       →  Android
    FW50          →  1050      →  MOVE,50,FORWARD
    BW30          →  2030      →  MOVE,30,BACKWARD
    FL00          →  6000      →  TURN,FORWARD_LEFT
    FR00          →  7000      →  TURN,FORWARD_RIGHT
    BL00          →  8000      →  TURN,BACKWARD_LEFT
    BR00          →  9000      →  TURN,BACKWARD_RIGHT
    SNAP4_C       →  5000      →  (None – determined at runtime)
    FN / FIN      →  (None)    →  (None – end marker, skipped)
    """
    upper = instruction.upper()

    if upper in ("FIN", "FN"):
        return (None, None)

    # SNAP commands (e.g. SNAP4_C, SNAP5_R, SNAP1)
    if _SNAP_RE.match(upper):
        return ("5000", None)

    m = _CMD_RE.match(upper)
    if not m:
        print(f"[TASK1] Cannot parse algo instruction: {instruction}")
        return (None, None)

    prefix = m.group(1)
    num    = int(m.group(2))

    if prefix == "FW":
        return (f"1{num:03d}", f"MOVE,{num},FORWARD")
    if prefix == "BW":
        return (f"2{num:03d}", f"MOVE,{num},BACKWARD")
    if prefix == "FL":
        return ("6000", "TURN,FORWARD_LEFT")
    if prefix == "FR":
        return ("7000", "TURN,FORWARD_RIGHT")
    if prefix == "BL":
        return ("8000", "TURN,BACKWARD_LEFT")
    if prefix == "BR":
        return ("9000", "TURN,BACKWARD_RIGHT")

    print(f"[TASK1] Unknown algo instruction: {instruction}")
    return (None, None)


# ---------------------------------------------------------------------------
# Build flat command lists from algo commands
# ---------------------------------------------------------------------------


def build_command_lists(
    commands: list[str],
) -> tuple[list[str], list[Optional[str]], dict[int, int]]:
    """
    Convert the flat algo command list into parallel STM / Android lists.

    Returns:
        stm_commands:     List of 4-char STM commands.
        android_commands: Parallel list of Android UI strings (None for
                          SNAP entries — filled at runtime).
        capture_map:      ``{ index: obstacle_number }`` for every SNAP.
    """
    stm_commands: list[str] = []
    android_commands: list[Optional[str]] = []
    capture_map: dict[int, int] = {}

    for cmd_str in commands:
        stm_cmd, android_cmd = instruction_to_commands(cmd_str)
        if stm_cmd is None:
            continue
        idx = len(stm_commands)
        stm_commands.append(stm_cmd)
        android_commands.append(android_cmd)
        if stm_cmd == "5000":
            snap_m = _SNAP_RE.match(cmd_str)
            if snap_m:
                capture_map[idx] = int(snap_m.group(1))
            else:
                print(f"[TASK1] WARNING: could not extract obstacle number "
                      f"from '{cmd_str}'")
                capture_map[idx] = -1

    return stm_commands, android_commands, capture_map


# ---------------------------------------------------------------------------
# Image capture (DetectionTracker majority vote)
# ---------------------------------------------------------------------------


def capture_image(tracker: DetectionTracker, obstacle_id: int) -> Optional[int]:
    """
    Wait for the rolling detection window to be filled and cycled with
    fresh detections, then evaluate.

    The deque (maxlen=WINDOW_SIZE) is continuously fed by the background
    collector — new detections push in and old ones drop off.  We sleep
    DETECTION_WAIT seconds so the window has time to cycle through with
    readings from the current face before we evaluate.

    Returns the numeric image ID if valid (10–41), else None.
    """
    print(f"[TASK1] CAPTURE_IMAGE – obstacle {obstacle_id}, "
          f"collecting for {DETECTION_WAIT}s …")
    time.sleep(DETECTION_WAIT)

    name, image_id, count, total = tracker.evaluate()

    if total == 0:
        print(f"[TASK1] No detections for obstacle {obstacle_id}")
        return None

    bar = "#" * count + "." * (total - count)
    print(f"[TASK1] Window ({total}): majority='{name}' image_id={image_id} "
          f"count={count}/{total} [{bar}]")

    if count >= CONFIDENCE_THRESHOLD and is_valid_image(name, image_id):
        print(f"[TASK1] *** IDENTIFIED: obstacle {obstacle_id} "
              f"→ target {image_id} ***")
        return image_id

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
    """
    body = "".join(stm_commands)
    checksum = sum(int(ch) for ch in body if ch.isdigit()) % 100
    return f"<{body}>{checksum:02d}"


def execute_all(
    stm: STM32Interface,
    bt: BluetoothIface,
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
    stm_payload = _build_stm_payload(stm_commands)
    print(f"[TASK1] → STM32 (batch): '{stm_payload}' "
          f"({len(stm_commands)} commands)")
    if not stm.send(stm_payload, add_newline=False):
        print("[TASK1] Failed to send batch to STM32")
        return False

    idx = 0
    while idx < len(stm_commands):
        elapsed = time.time() - run_start
        if elapsed > RUN_TIMEOUT:
            print(f"[TASK1] 6-minute timeout reached ({elapsed:.0f}s)")
            return False

        response = wait_for_stm(stm)
        if response is None:
            print("[TASK1] STM32 unresponsive – aborting")
            return False

        # STM echoes "RESM" back after we send it — ignore the echo
        if response == "RESM":
            print("[TASK1] Ignoring RESM echo from STM32")
            continue

        if response == "HALT":
            obstacle_id = capture_map.get(idx, -1)
            if obstacle_id == -1:
                print(f"[TASK1] WARNING: HALT at idx {idx} has no matching "
                      f"obstacle in capture_map – skipping capture")
                stm.send("RESM", add_newline=False)
                idx += 1
                continue

            print(f"[TASK1] HALT received – running image capture "
                  f"for obstacle {obstacle_id}")

            target_id = capture_image(tracker, obstacle_id)
            if target_id is not None:
                target_msg = f"TARGET,{obstacle_id},{target_id}"
                print(f"[TASK1] → Android: {target_msg}")
                bt.send(target_msg)
            else:
                print(f"[TASK1] Obstacle {obstacle_id}: identification failed")

            print("[TASK1] → STM32: 'RESM'")
            stm.send("RESM", add_newline=False)
            idx += 1

        elif response == "DONE":
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
# Main loop
# ---------------------------------------------------------------------------


def run(
    bt: BluetoothIface,
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
    print("  OBSTACLE,<id>,<x>,<y>,<direction>  (upserts by id)")
    print("  CLEAR                               (reset all state)")
    print("  BEGIN\n")

    robot_dir: int = 0  # NORTH by default
    obstacles: dict[int, dict[str, Any]] = {}  # keyed by obstacleNumber

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
            if parsed is not None:
                robot_dir = parsed
                print(f"[TASK1] Robot direction set to {robot_dir}")
            else:
                print(f"[TASK1] Bad ROBOT format: '{msg}'")
            continue

        # ---- CLEAR state -------------------------------------------------
        if token == "CLEAR":
            obstacles.clear()
            robot_dir = 0
            print("[TASK1] State cleared (obstacles + robot direction)")
            continue

        # ---- OBSTACLE registration (upsert by obstacleNumber) ------------
        if token.startswith("OBSTACLE"):
            obs = parse_obstacle(msg)
            if obs:
                obs_num = obs["obstacleNumber"]
                action = "Updated" if obs_num in obstacles else "Added"
                obstacles[obs_num] = obs
                print(f"[TASK1] {action} obstacle {obs_num} → "
                      f"grid ({obs['x']},{obs['y']}) "
                      f"d={obs['d']}  [{len(obstacles)} total]")
            else:
                print(f"[TASK1] Bad OBSTACLE format: '{msg}'")
            continue

        # ---- BEGIN navigation --------------------------------------------
        if token == "BEGIN":
            if not obstacles:
                print("[TASK1] No obstacles registered – ignoring BEGIN")
                continue

            obstacle_list = list(obstacles.values())

            print(f"\n[TASK1] ▶ BEGIN with {len(obstacle_list)} obstacle(s), "
                  f"robot_dir={robot_dir}")

            # 1) Request path from algo service
            result = algo.request_path(
                obstacle_list,
                robot_dir=robot_dir,
            )
            if not result or not result.get("data"):
                print("[TASK1] Algo returned no data")
                continue

            algo_commands = result["data"].get("commands", [])
            if not algo_commands:
                print("[TASK1] Algo returned empty commands")
                continue

            print(f"[TASK1] Algo commands: {algo_commands}")

            # 2) Build flat command lists
            stm_cmds, android_cmds, capture_map = build_command_lists(algo_commands)
            print(f"[TASK1] Built {len(stm_cmds)} STM commands "
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
            stm.send("<0000>00", add_newline=False)

            # Reset for next run
            obstacles.clear()
            robot_dir = 0
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
    parser.add_argument("--cam-width", type=int,
                        default=CameraInterface.DEFAULT_WIDTH,
                        help="Camera capture width")
    parser.add_argument("--cam-height", type=int,
                        default=CameraInterface.DEFAULT_HEIGHT,
                        help="Camera capture height")
    parser.add_argument("--no-camera", action="store_true",
                        help="Skip camera (if pi_streamer already running)")
    parser.add_argument("--bt-mode", choices=["socket", "serial"], default="serial",
                        help="Bluetooth backend: 'socket' (native AF_BLUETOOTH, default) "
                             "or 'serial' (/dev/rfcomm0 via pyserial)")
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
        cam = CameraInterface(
            port=args.cam_port,
            width=args.cam_width,
            height=args.cam_height,
        )
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
    if args.bt_mode == "socket":
        print("[INIT] Using socket-based Bluetooth (AF_BLUETOOTH)")
        bt = BluetoothSocket()
    else:
        print("[INIT] Using serial-based Bluetooth (/dev/rfcomm0)")
        bt = BluetoothSerial()
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
    bt: Optional[BluetoothIface] = None,
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

"""
Algo Instruction Converter

Converts algo service JSON responses into STM32 commands and Android UI
commands.  Can be used as an importable module or run standalone to
interactively convert a JSON response.

Algo command format:
    FW50  → Forward 50       → STM 1050, Android MOVE,50,FORWARD
    BW30  → Backward 30      → STM 2030, Android MOVE,30,BACKWARD
    FL00  → Forward Left      → STM 6000, Android TURN,FORWARD_LEFT
    FR00  → Forward Right     → STM 7000, Android TURN,FORWARD_RIGHT
    BL00  → Backward Left     → STM 8000, Android TURN,BACKWARD_LEFT
    BR00  → Backward Right    → STM 9000, Android TURN,BACKWARD_RIGHT
    SNAP4_C → Capture (obs 4)  → STM 5000, Android TARGET at runtime
    FN / FIN → End marker      → skipped

Standalone usage:
    # From a file
    python convert_instructions.py --file response.json

    # Interactive (paste JSON then Ctrl-D)
    python convert_instructions.py
"""

import argparse
import json
import re
import sys
from typing import Any, Optional

# Regex to split a movement command into prefix + numeric suffix
_CMD_RE = re.compile(r"^([A-Z]+)(\d+)$", re.IGNORECASE)
# SNAP commands: SNAP4_C, SNAP5_R, or legacy SNAP1
_SNAP_RE = re.compile(r"^SNAP(\d+)(?:_([A-Z]))?$", re.IGNORECASE)


# ---------------------------------------------------------------------------
# Core conversion
# ---------------------------------------------------------------------------


def instruction_to_commands(
    instruction: str,
) -> tuple[Optional[str], Optional[str]]:
    """
    Convert one algo instruction string into ``(stm_cmd, android_cmd)``.
    """
    upper = instruction.upper()

    if upper in ("FIN", "FN"):
        return (None, None)

    # SNAP commands (e.g. SNAP4_C, SNAP5_R, SNAP1)
    if _SNAP_RE.match(upper):
        return ("5000", None)

    m = _CMD_RE.match(upper)
    if not m:
        print(f"[CONVERT] Cannot parse: {instruction}")
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

    print(f"[CONVERT] Unknown instruction: {instruction}")
    return (None, None)


def build_stm_payload(stm_commands: list[str]) -> str:
    """
    Frame concatenated STM commands for transmission.

    Format:  ``<cmd1cmd2...cmdN>CC``

    where CC is a checksum: ``(sum of all digit chars) % 100``.
    """
    body = "".join(stm_commands)
    checksum = sum(int(ch) for ch in body if ch.isdigit()) % 100
    return f"<{body}>{checksum:02d}"


# ---------------------------------------------------------------------------
# Bulk conversion from algo JSON
# ---------------------------------------------------------------------------


def convert_response(data: dict[str, Any]) -> None:
    """
    Parse new algo response format and print STM / Android commands.
    """
    commands = data.get("data", {}).get("commands", [])
    if not commands:
        print("No commands found in response.")
        return

    distance = data.get("data", {}).get("distance", "?")
    print(f"\nAlgo response: {len(commands)} commands, distance={distance}\n")

    all_stm: list[str] = []

    print(f"  {'#':<4} {'Algo':<10} {'STM':>6}  {'Android'}")
    print(f"  {'─'*4} {'─'*10} {'─'*6}  {'─'*30}")

    for i, cmd_str in enumerate(commands):
        stm_cmd, android_cmd = instruction_to_commands(cmd_str)
        stm_str = stm_cmd or "—"
        android_str = android_cmd or ("(SNAP → TARGET at runtime)"
                                      if cmd_str.upper().startswith("SNAP")
                                      else "(skip)")
        print(f"  {i:<4} {cmd_str:<10} {stm_str:>6}  {android_str}")
        if stm_cmd:
            all_stm.append(stm_cmd)

    if all_stm:
        payload = build_stm_payload(all_stm)
        print(f"\n── STM payload ({len(all_stm)} commands) ──")
        print(f"  {payload}")


# ---------------------------------------------------------------------------
# Standalone entry point
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Convert algo JSON response to STM / Android commands",
    )
    parser.add_argument(
        "--file", "-f",
        help="Path to a JSON file containing the algo response. "
             "If omitted, reads from stdin (paste JSON then Ctrl-D).",
    )
    args = parser.parse_args()

    if args.file:
        with open(args.file) as fh:
            raw = fh.read()
    else:
        print("Paste the algo JSON response, then press Ctrl-D:\n")
        raw = sys.stdin.read()

    try:
        data = json.loads(raw)
    except json.JSONDecodeError as e:
        print(f"Invalid JSON: {e}")
        sys.exit(1)

    convert_response(data)

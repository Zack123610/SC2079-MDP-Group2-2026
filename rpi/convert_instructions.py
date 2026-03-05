"""
Algo Instruction Converter

Converts algo service JSON responses into STM32 commands and Android UI
commands.  Can be used as an importable module or run standalone to
interactively convert a JSON response.

Standalone usage:
    # From a file
    python convert_instructions.py --file response.json

    # Interactive (paste JSON then Ctrl-D)
    python convert_instructions.py
"""

import argparse
import json
import sys
from typing import Any, Optional

# ---------------------------------------------------------------------------
# Command mapping tables
# ---------------------------------------------------------------------------

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
# Core conversion
# ---------------------------------------------------------------------------


def instruction_to_commands(
    instruction: Any,
) -> tuple[Optional[str], Optional[str]]:
    """
    Convert one algo instruction into ``(stm_cmd, android_cmd)``.

    Algo instructions are either:
      - dict  ``{"amount": 50, "move": "FORWARD"}``  → ``("1050", "MOVE,50,FORWARD")``
      - str   ``"FORWARD_LEFT"``                     → ``("6000", "TURN,FORWARD_LEFT")``
      - str   ``"CAPTURE_IMAGE"``                    → ``("5000", None)``
    """
    if isinstance(instruction, dict):
        move   = instruction.get("move", "").upper()
        amount = instruction.get("amount", 0)
        code   = MOVE_DIR_CODE.get(move)
        if code is None:
            print(f"[CONVERT] Unknown move type: {move}")
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
            print(f"[CONVERT] Unknown string instruction: {instruction}")
        return (stm_cmd, android_cmd)

    return (None, None)


def build_stm_payload(stm_commands: list[str]) -> str:
    """
    Frame concatenated STM commands for transmission.

    Format:  ``<cmd1cmd2...cmdN>CC``

    where CC is a checksum: ``(sum of all digit chars) % 100``.
    """
    body = "".join(stm_commands)
    checksum = sum(int(ch) for ch in body if ch.isdigit()) % 100
    return f"<{body}>{checksum}"


# ---------------------------------------------------------------------------
# Bulk conversion from algo JSON
# ---------------------------------------------------------------------------


def convert_response(data: dict[str, Any]) -> None:
    """
    Parse a full algo service response and print the converted commands.
    """
    segments = data.get("segments", [])
    if not segments:
        print("No segments found in response.")
        return

    all_stm: list[str] = []

    for seg in segments:
        obstacle_id = seg.get("image_id", "?")
        instructions = seg.get("instructions", [])
        print(f"\n── Segment: obstacle {obstacle_id} "
              f"({len(instructions)} instructions) ──")
        print(f"  {'#':<4} {'Algo instruction':<30} {'STM':>6}  {'Android'}")
        print(f"  {'─'*4} {'─'*30} {'─'*6}  {'─'*28}")

        for i, instr in enumerate(instructions):
            stm_cmd, android_cmd = instruction_to_commands(instr)
            instr_str = json.dumps(instr) if isinstance(instr, dict) else str(instr)
            stm_str = stm_cmd or "—"
            android_str = android_cmd or "(CAPTURE → TARGET at runtime)"
            print(f"  {i:<4} {instr_str:<30} {stm_str:>6}  {android_str}")
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

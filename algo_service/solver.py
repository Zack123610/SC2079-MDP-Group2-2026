"""
Build pathfinding segments for Task 1: visit each obstacle in order,
navigate to a valid capture position, then CAPTURE_IMAGE.

Input: robot config (start position + heading), list of obstacles.
Output: list of segments, each with image_id and instructions (including CAPTURE_IMAGE).
"""

from __future__ import annotations

from typing import Any

from .config import ROBOT_SIZE
from .grid import is_valid_robot_placement
from .pathfinding import plan_to_obstacle


def _obstacle_cells(obstacles: list[dict[str, Any]]) -> set[tuple[int, int]]:
    """Set of grid cells occupied by obstacles."""
    out: set[tuple[int, int]] = set()
    for o in obstacles:
        sw = o.get("south_west", {})
        ne = o.get("north_east", {})
        x0 = sw.get("x", 0)
        y0 = sw.get("y", 0)
        x1 = ne.get("x", x0 + 1)
        y1 = ne.get("y", y0 + 1)
        for x in range(x0, x1):
            for y in range(y0, y1):
                out.add((x, y))
    return out


def _robot_start(robot: dict[str, Any]) -> tuple[int, int, str]:
    """(sx, sy, heading) from robot config. Use south_west as position."""
    sw = robot.get("south_west", {"x": 0, "y": 0})
    sx = sw.get("x", 0)
    sy = sw.get("y", 0)
    heading = (robot.get("direction") or "NORTH").upper()
    return (sx, sy, heading)


def build_segments(
    robot: dict[str, Any],
    obstacles: list[dict[str, Any]],
) -> list[dict[str, Any]]:
    """
    Build one segment per obstacle: path to a capture position + CAPTURE_IMAGE.

    Robot config: direction, south_west, north_east (north_east used only to
    infer size; we assume robot is ROBOT_SIZE x ROBOT_SIZE for collision).
    Each obstacle: image_id, direction (face with image), south_west, north_east.

    Returns list of segments, each:
      { "image_id": int, "instructions": [...], "cost": int, "path": [...] }
    """
    if not obstacles:
        return []

    obstacle_cells = _obstacle_cells(obstacles)
    sx, sy, heading = _robot_start(robot)

    # Validate start
    if not is_valid_robot_placement(sx, sy, obstacle_cells):
        return []

    segments: list[dict[str, Any]] = []

    for obs in obstacles:
        image_id = obs.get("image_id", 0)
        face = (obs.get("direction") or "NORTH").upper()
        sw = obs.get("south_west", {})
        ox = sw.get("x", 0)
        oy = sw.get("y", 0)

        result = plan_to_obstacle(
            sx, sy, heading,
            ox, oy, face,
            obstacle_cells,
        )

        if result is None:
            segments.append({
                "image_id": image_id,
                "instructions": [],
                "cost": 0,
                "path": [],
            })
            continue

        instructions, end_sx, end_sy = result
        instructions_with_capture = instructions + ["CAPTURE_IMAGE"]
        cost = len(instructions_with_capture)

        segments.append({
            "image_id": image_id,
            "instructions": instructions_with_capture,
            "cost": cost,
            "path": [],  # optional: could encode path cells
        })

        # Next segment starts from capture position (robot doesn't move during capture)
        sx, sy = end_sx, end_sy

    return segments

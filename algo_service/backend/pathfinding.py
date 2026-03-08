"""
Pathfinding from robot start to a goal region, avoiding obstacles.

Uses A* with state (sx, sy, heading). Actions:
- FORWARD / BACKWARD by 1 unit (step); we later coalesce into single FORWARD/BACKWARD amount.
- FORWARD_LEFT, FORWARD_RIGHT, BACKWARD_LEFT, BACKWARD_RIGHT (fixed 2 forward, 3 side).

All movements are validated so the 3x3 robot never overlaps an obstacle or leaves the arena.
"""

from __future__ import annotations

import heapq
from typing import Any

from .config import (
    ARENA_SIZE,
    ROBOT_SIZE,
    UNIT_CM,
    NORTH,
    SOUTH,
    EAST,
    WEST,
    ARC_FORWARD_UNITS,
    ARC_SIDE_UNITS,
)
from .grid import (
    is_valid_robot_placement,
    capture_positions_for_obstacle,
)


# Unit deltas: (dx, dy) for forward in each heading (forward = +1 in heading direction)
FORWARD_DELTA = {
    NORTH: (0, 1),
    SOUTH: (0, -1),
    EAST: (1, 0),
    WEST: (-1, 0),
}
BACKWARD_DELTA = {h: (-d[0], -d[1]) for h, d in FORWARD_DELTA.items()}
LEFT_DELTA = {
    NORTH: (-1, 0),   # left of north = west
    SOUTH: (1, 0),
    EAST: (0, 1),
    WEST: (0, -1),
}
RIGHT_DELTA = {
    NORTH: (1, 0),
    SOUTH: (-1, 0),
    EAST: (0, -1),
    WEST: (0, 1),
}

# After an arc move, the robot's heading changes by 90°:
# FORWARD_LEFT / BACKWARD_LEFT: turn left (ccw)  — NORTH→WEST, WEST→SOUTH, etc.
# FORWARD_RIGHT / BACKWARD_RIGHT: turn right (cw) — NORTH→EAST, EAST→SOUTH, etc.
HEADING_AFTER_LEFT = {
    NORTH: WEST,
    WEST: SOUTH,
    SOUTH: EAST,
    EAST: NORTH,
}
HEADING_AFTER_RIGHT = {
    NORTH: EAST,
    EAST: SOUTH,
    SOUTH: WEST,
    WEST: NORTH,
}


def arc_delta(heading: str, forward: bool, left: bool) -> tuple[int, int]:
    """
    Delta (dx, dy) for arc move: 2 forward, 3 left or 3 right.
    forward=True, left=True  -> FORWARD_LEFT
    forward=True, left=False -> FORWARD_RIGHT
    forward=False, left=True -> BACKWARD_LEFT
    forward=False, left=False-> BACKWARD_RIGHT
    """
    fd = FORWARD_DELTA[heading]
    ld = LEFT_DELTA[heading] if left else RIGHT_DELTA[heading]
    sign = 1 if forward else -1
    dx = sign * ARC_FORWARD_UNITS * fd[0] + ARC_SIDE_UNITS * ld[0]
    dy = sign * ARC_FORWARD_UNITS * fd[1] + ARC_SIDE_UNITS * ld[1]
    return (dx, dy)


def next_states(
    sx: int,
    sy: int,
    heading: str,
    obstacle_cells: set[tuple[int, int]],
) -> list[tuple[int, int, str, Any]]:
    """
    List of (new_sx, new_sy, new_heading, action) reachable in one step.
    action is either ("FORWARD", 1), ("BACKWARD", 1), "FORWARD_LEFT", etc.
    """
    out: list[tuple[int, int, str, Any]] = []

    # FORWARD 1 unit
    dx, dy = FORWARD_DELTA[heading]
    nx, ny = sx + dx, sy + dy
    if is_valid_robot_placement(nx, ny, obstacle_cells):
        out.append((nx, ny, heading, ("FORWARD", 1)))

    # BACKWARD 1 unit
    dx, dy = BACKWARD_DELTA[heading]
    nx, ny = sx + dx, sy + dy
    if is_valid_robot_placement(nx, ny, obstacle_cells):
        out.append((nx, ny, heading, ("BACKWARD", 1)))

    # FORWARD_LEFT
    dx, dy = arc_delta(heading, forward=True, left=True)
    nx, ny = sx + dx, sy + dy
    if is_valid_robot_placement(nx, ny, obstacle_cells):
        out.append((nx, ny, HEADING_AFTER_LEFT[heading], "FORWARD_LEFT"))

    # FORWARD_RIGHT
    dx, dy = arc_delta(heading, forward=True, left=False)
    nx, ny = sx + dx, sy + dy
    if is_valid_robot_placement(nx, ny, obstacle_cells):
        out.append((nx, ny, HEADING_AFTER_RIGHT[heading], "FORWARD_RIGHT"))

    # BACKWARD_LEFT
    dx, dy = arc_delta(heading, forward=False, left=True)
    nx, ny = sx + dx, sy + dy
    if is_valid_robot_placement(nx, ny, obstacle_cells):
        out.append((nx, ny, HEADING_AFTER_LEFT[heading], "BACKWARD_LEFT"))

    # BACKWARD_RIGHT
    dx, dy = arc_delta(heading, forward=False, left=False)
    nx, ny = sx + dx, sy + dy
    if is_valid_robot_placement(nx, ny, obstacle_cells):
        out.append((nx, ny, HEADING_AFTER_RIGHT[heading], "BACKWARD_RIGHT"))

    return out


def heuristic(sx: int, sy: int, heading: str, goal_states: set[tuple[int, int, str]]) -> int:
    """Min Manhattan distance to any goal (sx, sy, goal_heading); +0 if heading already matches."""
    if not goal_states:
        return 0
    return min(
        abs(sx - gx) + abs(sy - gy)
        for (gx, gy, _gh) in goal_states
    )


def astar(
    start_sx: int,
    start_sy: int,
    start_heading: str,
    goal_states: set[tuple[int, int, str]],
    obstacle_cells: set[tuple[int, int]],
) -> list[Any] | None:
    """
    A* from (start_sx, start_sy, start_heading) to any (gx, gy, goal_heading) in goal_states.
    The robot must end at (gx, gy) facing goal_heading (so it faces the obstacle for CAPTURE_IMAGE).
    Returns list of actions or None if no path.
    """
    if (start_sx, start_sy, start_heading) in goal_states:
        return []

    initial_path: list[Any] = []
    h0 = heuristic(start_sx, start_sy, start_heading, goal_states)
    tie = 0
    open_heap: list[tuple[int, int, int, int, int, str, list[Any]]] = [
        (h0, tie, 0, start_sx, start_sy, start_heading, initial_path)
    ]
    closed: set[tuple[int, int, str]] = set()

    while open_heap:
        _f, _tie, g, sx, sy, heading, path = heapq.heappop(open_heap)
        state = (sx, sy, heading)
        if state in closed:
            continue
        closed.add(state)

        if (sx, sy, heading) in goal_states:
            return path

        for nx, ny, nh, action in next_states(sx, sy, heading, obstacle_cells):
            if (nx, ny, nh) in closed:
                continue
            tie += 1
            new_path = path + [action]
            new_g = g + 1
            h = heuristic(nx, ny, nh, goal_states)
            heapq.heappush(
                open_heap,
                (new_g + h, tie, new_g, nx, ny, nh, new_path),
            )

    return None


def coalesce_moves(actions: list[Any]) -> list[Any]:
    """
    Merge consecutive FORWARD n and BACKWARD n into single FORWARD/BACKWARD with total amount.
    Other actions (arc turns) stay as strings.
    """
    if not actions:
        return []
    result: list[Any] = []
    i = 0
    while i < len(actions):
        a = actions[i]
        if a == "FORWARD_LEFT" or a == "FORWARD_RIGHT" or a == "BACKWARD_LEFT" or a == "BACKWARD_RIGHT":
            result.append(a)
            i += 1
            continue
        if isinstance(a, tuple) and (a[0] == "FORWARD" or a[0] == "BACKWARD"):
            direction = a[0]
            total = a[1]
            i += 1
            while i < len(actions) and actions[i] == (direction, 1):
                total += 1
                i += 1
            result.append((direction, total))
            continue
        i += 1
    return result


def path_to_instructions(
    actions: list[Any],
) -> list[Any]:
    """
    Convert path actions to algo service instructions.

    - ("FORWARD", n) -> {"move": "FORWARD", "amount": n * UNIT_CM}
    - ("BACKWARD", n) -> {"move": "BACKWARD", "amount": n * UNIT_CM}
    - "FORWARD_LEFT" etc. -> "FORWARD_LEFT" string
    """
    coalesced = coalesce_moves(actions)
    instructions: list[Any] = []
    for a in coalesced:
        if isinstance(a, tuple):
            direction, units = a
            instructions.append({
                "move": direction,
                "amount": units * UNIT_CM,
            })
        else:
            instructions.append(a)
    return instructions


def apply_actions(
    sx: int,
    sy: int,
    heading: str,
    actions: list[Any],
) -> tuple[int, int, str]:
    """
    Apply path actions (before coalescing) and return final (sx, sy, heading).
    Arc moves update the robot's heading (90° turn left or right).
    """
    for a in actions:
        if a == "FORWARD_LEFT":
            dx, dy = arc_delta(heading, forward=True, left=True)
            heading = HEADING_AFTER_LEFT[heading]
        elif a == "FORWARD_RIGHT":
            dx, dy = arc_delta(heading, forward=True, left=False)
            heading = HEADING_AFTER_RIGHT[heading]
        elif a == "BACKWARD_LEFT":
            dx, dy = arc_delta(heading, forward=False, left=True)
            heading = HEADING_AFTER_LEFT[heading]
        elif a == "BACKWARD_RIGHT":
            dx, dy = arc_delta(heading, forward=False, left=False)
            heading = HEADING_AFTER_RIGHT[heading]
        elif isinstance(a, tuple) and a[0] == "FORWARD":
            dx, dy = FORWARD_DELTA[heading]
            dx, dy = dx * a[1], dy * a[1]
        elif isinstance(a, tuple) and a[0] == "BACKWARD":
            dx, dy = BACKWARD_DELTA[heading]
            dx, dy = dx * a[1], dy * a[1]
        else:
            continue
        sx += dx
        sy += dy
    return (sx, sy, heading)


def plan_to_obstacle(
    start_sx: int,
    start_sy: int,
    start_heading: str,
    obstacle_x: int,
    obstacle_y: int,
    obstacle_face: str,
    obstacle_cells: set[tuple[int, int]],
) -> tuple[list[Any], int, int, str] | None:
    """
    Plan from (start_sx, start_sy, start_heading) to a capture position for obstacle
    at (obstacle_x, obstacle_y) with image on obstacle_face. The robot must end
    facing obstacle_face (toward the obstacle) to identify the image.
    Returns (instructions, end_sx, end_sy, end_heading) or None if no path.
    """
    position_goals = capture_positions_for_obstacle(
        obstacle_x, obstacle_y, obstacle_face
    )
    position_goals = {
        (gx, gy)
        for (gx, gy) in position_goals
        if is_valid_robot_placement(gx, gy, obstacle_cells)
    }
    if not position_goals:
        return None
    # Goal state: at any capture cell AND facing the obstacle (face = direction robot looks to see image)
    goal_states = {(gx, gy, obstacle_face) for (gx, gy) in position_goals}
    path = astar(
        start_sx, start_sy, start_heading,
        goal_states, obstacle_cells,
    )
    if path is None:
        return None
    instructions = path_to_instructions(path)
    end_sx, end_sy, end_heading = apply_actions(start_sx, start_sy, start_heading, path)
    return (instructions, end_sx, end_sy, end_heading)

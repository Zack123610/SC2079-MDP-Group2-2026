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
        out.append((nx, ny, heading, "FORWARD_LEFT"))

    # FORWARD_RIGHT
    dx, dy = arc_delta(heading, forward=True, left=False)
    nx, ny = sx + dx, sy + dy
    if is_valid_robot_placement(nx, ny, obstacle_cells):
        out.append((nx, ny, heading, "FORWARD_RIGHT"))

    # BACKWARD_LEFT
    dx, dy = arc_delta(heading, forward=False, left=True)
    nx, ny = sx + dx, sy + dy
    if is_valid_robot_placement(nx, ny, obstacle_cells):
        out.append((nx, ny, heading, "BACKWARD_LEFT"))

    # BACKWARD_RIGHT
    dx, dy = arc_delta(heading, forward=False, left=False)
    nx, ny = sx + dx, sy + dy
    if is_valid_robot_placement(nx, ny, obstacle_cells):
        out.append((nx, ny, heading, "BACKWARD_RIGHT"))

    return out


def heuristic(sx: int, sy: int, goal_cells: set[tuple[int, int]]) -> int:
    """Min Manhattan distance from (sx, sy) to any goal cell (using robot corner)."""
    if not goal_cells:
        return 0
    return min(
        abs(sx - gx) + abs(sy - gy)
        for (gx, gy) in goal_cells
    )


def astar(
    start_sx: int,
    start_sy: int,
    start_heading: str,
    goal_cells: set[tuple[int, int]],
    obstacle_cells: set[tuple[int, int]],
) -> list[Any] | None:
    """
    A* from (start_sx, start_sy, start_heading) to any cell in goal_cells.
    Returns list of actions (each ("FORWARD", n), ("BACKWARD", n), or "FORWARD_LEFT" etc.)
    or None if no path.
    """
    if (start_sx, start_sy) in goal_cells:
        return []

    # state = (sx, sy, heading)
    # priority, steps, state, path
    initial_path: list[Any] = []
    h0 = heuristic(start_sx, start_sy, goal_cells)
    open_heap: list[tuple[int, int, int, int, str, list[Any]]] = [
        (h0, 0, start_sx, start_sy, start_heading, initial_path)
    ]
    closed: set[tuple[int, int, str]] = set()
    # Tie-break by (sx, sy, heading) for determinism
    tie = 0

    while open_heap:
        _f, g, sx, sy, heading, path = heapq.heappop(open_heap)
        state = (sx, sy, heading)
        if state in closed:
            continue
        closed.add(state)

        if (sx, sy) in goal_cells:
            return path

        for nx, ny, nh, action in next_states(sx, sy, heading, obstacle_cells):
            if (nx, ny, nh) in closed:
                continue
            new_path = path + [action]
            new_g = g + 1
            h = heuristic(nx, ny, goal_cells)
            heapq.heappush(
                open_heap,
                (new_g + h, new_g, nx, ny, nh, new_path),
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
) -> tuple[int, int]:
    """
    Apply path actions (before coalescing) and return final (sx, sy).
    Heading is unchanged (arc moves do not change heading in our model).
    """
    for a in actions:
        if a == "FORWARD_LEFT":
            dx, dy = arc_delta(heading, forward=True, left=True)
        elif a == "FORWARD_RIGHT":
            dx, dy = arc_delta(heading, forward=True, left=False)
        elif a == "BACKWARD_LEFT":
            dx, dy = arc_delta(heading, forward=False, left=True)
        elif a == "BACKWARD_RIGHT":
            dx, dy = arc_delta(heading, forward=False, left=False)
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
    return (sx, sy)


def plan_to_obstacle(
    start_sx: int,
    start_sy: int,
    start_heading: str,
    obstacle_x: int,
    obstacle_y: int,
    obstacle_face: str,
    obstacle_cells: set[tuple[int, int]],
) -> tuple[list[Any], int, int] | None:
    """
    Plan from (start_sx, start_sy, start_heading) to a capture position
    for obstacle at (obstacle_x, obstacle_y) with image on obstacle_face.
    Returns (instructions, end_sx, end_sy) or None if no path.
    """
    goals = capture_positions_for_obstacle(
        obstacle_x, obstacle_y, obstacle_face
    )
    goals = {
        (gx, gy)
        for (gx, gy) in goals
        if is_valid_robot_placement(gx, gy, obstacle_cells)
    }
    if not goals:
        return None
    path = astar(
        start_sx, start_sy, start_heading,
        goals, obstacle_cells,
    )
    if path is None:
        return None
    instructions = path_to_instructions(path)
    end_sx, end_sy = apply_actions(start_sx, start_sy, start_heading, path)
    return (instructions, end_sx, end_sy)

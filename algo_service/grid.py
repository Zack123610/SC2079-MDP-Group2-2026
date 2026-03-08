"""
Grid representation and collision checks.

- Arena [0, ARENA_SIZE) x [0, ARENA_SIZE).
- Robot at (sx, sy) occupies [sx, sx+ROBOT_SIZE) x [sy, sy+ROBOT_SIZE).
- Obstacles are 1x1 cells; obstacle at (ox, oy) blocks cell (ox, oy).
- View cell: the cell at CAPTURE_DISTANCE_UNITS from the obstacle (on the side
  opposite the image face), so the robot gets optimal image recognition distance.
"""

from __future__ import annotations

from .config import (
    ARENA_SIZE,
    ROBOT_SIZE,
    CAPTURE_DISTANCE_UNITS,
    NORTH,
    SOUTH,
    EAST,
    WEST,
)


def robot_cells(sx: int, sy: int) -> set[tuple[int, int]]:
    """Set of grid cells occupied by robot with south-west corner (sx, sy)."""
    return {
        (x, y)
        for x in range(sx, sx + ROBOT_SIZE)
        for y in range(sy, sy + ROBOT_SIZE)
    }


def obstacle_cell(ox: int, oy: int) -> set[tuple[int, int]]:
    """Single 1x1 obstacle at (ox, oy)."""
    return {(ox, oy)}


def in_arena(sx: int, sy: int) -> bool:
    """Robot (south-west at sx, sy) is fully inside the arena."""
    return (
        0 <= sx and sx + ROBOT_SIZE <= ARENA_SIZE
        and 0 <= sy and sy + ROBOT_SIZE <= ARENA_SIZE
    )


def robot_collides_obstacles(
    sx: int,
    sy: int,
    obstacle_cells: set[tuple[int, int]],
) -> bool:
    """True if robot at (sx, sy) overlaps any obstacle cell."""
    return bool(robot_cells(sx, sy) & obstacle_cells)


def is_valid_robot_placement(
    sx: int,
    sy: int,
    obstacle_cells: set[tuple[int, int]],
) -> bool:
    """Robot at (sx, sy) is in arena and does not hit any obstacle."""
    return in_arena(sx, sy) and not robot_collides_obstacles(
        sx, sy, obstacle_cells
    )


def view_cell_for_obstacle(ox: int, oy: int, face: str) -> tuple[int, int]:
    """
    Cell at CAPTURE_DISTANCE_UNITS from the obstacle (opposite the image face).

    Image recognition works best at 2 units away. We need to be on the opposite
    side of the face to look at it, at distance CAPTURE_DISTANCE_UNITS:
    - Face NORTH (image on north side) → we stand south → (ox, oy - CAPTURE_DISTANCE_UNITS)
    - Face SOUTH → (ox, oy + CAPTURE_DISTANCE_UNITS)
    - Face EAST  → (ox - CAPTURE_DISTANCE_UNITS, oy)
    - Face WEST  → (ox + CAPTURE_DISTANCE_UNITS, oy)
    """
    if face == NORTH:
        return (ox, oy - CAPTURE_DISTANCE_UNITS)
    if face == SOUTH:
        return (ox, oy + CAPTURE_DISTANCE_UNITS)
    if face == EAST:
        return (ox - CAPTURE_DISTANCE_UNITS, oy)
    if face == WEST:
        return (ox + CAPTURE_DISTANCE_UNITS, oy)
    return (ox, oy - CAPTURE_DISTANCE_UNITS)  # default


def capture_positions_for_obstacle(
    ox: int,
    oy: int,
    face: str,
) -> set[tuple[int, int]]:
    """
    South-west positions (sx, sy) such that the 3x3 robot overlaps
    the view cell (at CAPTURE_DISTANCE_UNITS from the obstacle) for
    optimal image recognition.
    """
    vx, vy = view_cell_for_obstacle(ox, oy, face)
    # Robot must contain (vx, vy). So sx <= vx < sx+ROBOT_SIZE, sy <= vy < sy+ROBOT_SIZE.
    return {
        (sx, sy)
        for sx in range(vx - ROBOT_SIZE + 1, vx + 1)
        for sy in range(vy - ROBOT_SIZE + 1, vy + 1)
        if 0 <= sx and sx + ROBOT_SIZE <= ARENA_SIZE
        and 0 <= sy and sy + ROBOT_SIZE <= ARENA_SIZE
    }

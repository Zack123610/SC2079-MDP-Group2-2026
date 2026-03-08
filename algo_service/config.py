"""
Arena and robot constants for Task 1 pathfinding.

- Arena: 20 x 20 units.
- Robot: 3 x 3 units (occupies a 3x3 block; position = south-west corner).
- Obstacle: 1 x 1 unit, one face has the valid image.
- 1 unit = 10 cm for STM32 movement amounts.
"""

ARENA_SIZE = 20
ROBOT_SIZE = 3
OBSTACLE_SIZE = 1
UNIT_CM = 10

# Image recognition works best at this distance (units) from the obstacle face.
CAPTURE_DISTANCE_UNITS = 2

# Arc turn displacement in grid units: (forward, left) = (2, 3)
# FORWARD_LEFT: 2 forward, 3 left
# Same magnitude for FORWARD_RIGHT (2 forward, 3 right), etc.
ARC_FORWARD_UNITS = 2
ARC_SIDE_UNITS = 3

# Directions (grid: x right, y up; NORTH = +y)
NORTH = "NORTH"
SOUTH = "SOUTH"
EAST = "EAST"
WEST = "WEST"
DIRECTIONS = (NORTH, EAST, SOUTH, WEST)

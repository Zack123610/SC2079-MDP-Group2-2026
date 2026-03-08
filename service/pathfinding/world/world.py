from __future__ import annotations

from abc import ABC
from dataclasses import dataclass

import numpy as np
import math

from pathfinding.world.primitives import Direction, Point, Vector


class World:
    """
    The actual size of the world in centimetres.
    """
    __actual_size = 200

    """
    A world. Clearance computation is optimized for rectangular/square obstacles.
    """

    def __init__(self, size: int, robot: Robot, obstacles: list[Obstacle]):
        self.size = size
        self.grid = np.full((size, size), True)
        self.obstacles = obstacles
        self.robot = robot

        print(
            "WORLD size=%s cell_size=%s",
            self.size,
            self.cell_size,
        )

        print(
            "ROBOT sw=%s ne=%s centre=%s N=%s E=%s S=%s W=%s",
            self.robot.south_west,
            self.robot.north_east,
            self.robot.centre,
            self.robot.north_length,
            self.robot.east_length,
            self.robot.south_length,
            self.robot.west_length,
        )

        assert all(map(lambda obstacle: self.__inside(obstacle), self.obstacles))
        self.__annotate_grid()

        assert self.__inside(robot)

    def __inside(self, entity: Entity) -> bool:
        return (0 <= entity.south_west.x < self.size and
                0 <= entity.south_west.y < self.size and
                0 <= entity.north_east.x < self.size and
                0 <= entity.north_east.y < self.size)

    def __annotate_grid(self: World) -> None:
        # Clear grid first
        self.grid[:, :] = True
        
        # Block edges where robot would go out
        robot_half_width = self.robot.east_length  # = 1 for 3-wide robot
        robot_half_height = self.robot.north_length  # = 1 for 3-tall robot
        
        # Block left/right edges
        self.grid[:, :robot_half_width] = False  # Left edge
        self.grid[:, -robot_half_width:] = False  # Right edge
        
        # Block top/bottom edges  
        self.grid[:robot_half_height, :] = False  # Bottom edge
        self.grid[-robot_half_height:, :] = False  # Top edge
        
        # Block obstacle areas with robot clearance
        for obstacle in self.obstacles:
            # Block obstacle cell itself
            ox1, oy1 = obstacle.south_west.x, obstacle.south_west.y
            ox2, oy2 = obstacle.north_east.x, obstacle.north_east.y
            
            # Extend by robot half-size in all directions
            block_x1 = max(0, ox1 - robot_half_width)
            block_x2 = min(self.size, ox2 + robot_half_width + 1)
            block_y1 = max(0, oy1 - robot_half_height)
            block_y2 = min(self.size, oy2 + robot_half_height + 1)
            
            self.grid[block_y1:block_y2, block_x1:block_x2] = False
        
        print(f"Grid blocked: {np.count_nonzero(~self.grid)} cells")


    def contains(self, vector: Vector) -> bool:
        """Check if the robot at this vector position is within world bounds"""
        # Check robot center is within grid
        if not (0 <= vector.x < self.size and 0 <= vector.y < self.size):
            return False
        
        if not self.grid[vector.y, vector.x]:
            # print(f"  ❌ Cell ({vector.x},{vector.y}) is blocked in grid")
            return False

        # Also check robot footprint doesn't exceed bounds
        robot_width = self.robot.north_east.x - self.robot.south_west.x
        robot_height = self.robot.north_east.y - self.robot.south_west.y
        
        half_width = robot_width // 2
        half_height = robot_height // 2
        
        min_x = vector.x - half_width
        max_x = vector.x + half_width
        min_y = vector.y - half_height
        max_y = vector.y + half_height
        
        return (0 <= min_x < self.size and 0 <= max_x < self.size and
                0 <= min_y < self.size and 0 <= max_y < self.size)

    
    def is_safe(self, pos: Vector) -> bool:
        '''
        # 1. Calculate padding (3 cells for a 30cm robot on 5cm grid)
        half_robot_cm = 15
        padding = math.ceil(half_robot_cm / self.cell_size)

        # 2. Get actual grid boundaries from the array itself
        # shape[0] is height (y), shape[1] is width (x)
        max_y, max_x = self.grid.shape

        # 3. Boundary Check: Ensure the entire footprint is inside the map
        if (pos.x - padding < 0 or pos.x + padding >= max_x or
            pos.y - padding < 0 or pos.y + padding >= max_y):
            return False

        # 4. Footprint Check: Slice the pre-annotated grid
        # We use int() to ensure indices are valid for slicing
        y_start, y_end = int(pos.y - padding), int(pos.y + padding + 1)
        x_start, x_end = int(pos.x - padding), int(pos.x + padding + 1)
        
        footprint = self.grid[y_start:y_end, x_start:x_end]
        
        # If the slice is empty (shouldn't happen with our boundary check) 
        # or if any cell is False (blocked), return False.
        if footprint.size == 0 or not np.all(footprint):
            return False
        '''
        return self.contains(pos)
        
    @property
    def cell_size(self) -> int:
        return self.__actual_size // self.size


@dataclass
class Entity(ABC):
    direction: Direction
    south_west: Point
    north_east: Point

    def __post_init__(self):
        assert 0 <= self.south_west.x <= self.north_east.x
        assert 0 <= self.south_west.y <= self.north_east.y
        assert (self.north_east.y - self.south_west.y) == (self.north_east.x - self.south_west.x)
        self.centre = Point((self.north_east.x + self.south_west.x) // 2, (self.north_east.y + self.south_west.y) // 2)
        self.north_length = self.north_east.y - self.centre.y
        self.east_length = self.north_east.x - self.centre.x
        self.south_length = self.centre.y - self.south_west.y
        self.west_length = self.centre.x - self.south_west.x

    @property
    def clearance(self):
        # Assumes that height & width are the same
        return self.north_east.y - self.south_west.y + 1

    @property
    def vector(self) -> Vector:
        return Vector(self.direction, self.centre.x, self.centre.y)


@dataclass(unsafe_hash=True)
class Obstacle(Entity):
    image_id: int

    def __post_init__(self):
        super().__post_init__()
        assert 1 <= self.image_id < 36


@dataclass
class Robot(Entity):
    def __post_init__(self):
        super().__post_init__()

    @property
    def camera_position(self) -> Point:
        """Camera is at front center of robot"""
        centre_x = (self.north_east.x + self.south_west.x) // 2
        if self.direction == Direction.NORTH:
            return Point(centre_x, self.north_east.y)
        elif self.direction == Direction.SOUTH:
            return Point(centre_x, self.south_west.y)
        elif self.direction == Direction.EAST:
            return Point(self.north_east.x, centre_x)
        else:  # WEST
            return Point(self.south_west.x, centre_x)

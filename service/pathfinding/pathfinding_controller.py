from __future__ import annotations

import os
from datetime import datetime
from http import HTTPStatus

import time
import numpy as np
from flask import make_response, request
from flask_openapi3 import APIBlueprint, Tag
from pydantic import BaseModel, Field

from pathfinding.search.instructions import (
    MiscInstruction,
    TurnInstruction, MoveInstruction,
)
from pathfinding.search.search import Segment, search
from pathfinding.world.objective import generate_objectives
from pathfinding.world.primitives import Direction, Point, Vector
from pathfinding.world.world import Obstacle, Robot, World

api = APIBlueprint(
    "/pathfinding",
    __name__,
    url_prefix="/pathfinding",
    abp_tags=[Tag(name="Pathfinding")],
)


import heapq
import math
from typing import List
import numpy as np
from python_tsp.exact import solve_tsp_dynamic_programming



from typing import List

from typing import List
from enum import Enum

def command_generator(states, obstacles):
    """
    This function takes in a list of states and generates a list of commands for the robot to follow
    
    Inputs
    ------
    states: list of State objects
    obstacles: list of obstacles, each obstacle is a dictionary with keys "x", "y", "d", and "id"

    Returns
    -------
    commands: list of commands for the robot to follow
    """

    # Convert the list of obstacles into a dictionary with key as the obstacle id and value as the obstacle
    obstacles_dict = {ob['obstacleNumber']: ob for ob in obstacles}
    
    # Initialize commands list
    commands = []

    # Iterate through each state in the list of states
    for i in range(1, len(states)):
        steps = "00"

        # If previous state and current state are the same direction,
        if states[i].direction == states[i - 1].direction:
            # Forward - Must be (east facing AND x value increased) OR (north facing AND y value increased)
            if (states[i].x > states[i - 1].x and states[i].direction == Direction.EAST) or (states[i].y > states[i - 1].y and states[i].direction == Direction.NORTH):
                commands.append("FW10")
            # Forward - Must be (west facing AND x value decreased) OR (south facing AND y value decreased)
            elif (states[i].x < states[i-1].x and states[i].direction == Direction.WEST) or (
                    states[i].y < states[i-1].y and states[i].direction == Direction.SOUTH):
                commands.append("FW10")
            # Backward - All other cases where the previous and current state is the same direction
            else:
                commands.append("BW10")

            # If any of these states has a valid screenshot ID, then add a SNAP command as well to take a picture
            if states[i].screenshot_id != -1:
                # NORTH = 0
                # EAST = 2
                # SOUTH = 4
                # WEST = 6

                current_ob_dict = obstacles_dict[states[i].screenshot_id] # {'x': 9, 'y': 10, 'd': 6, 'id': 9}
                current_robot_position = states[i] # {'x': 1, 'y': 8, 'd': <Direction.NORTH: 0>, 's': -1}

                # Obstacle facing WEST, robot facing EAST
                if current_ob_dict['d'] == 6 and current_robot_position.direction == 2:
                    if current_ob_dict['y'] > current_robot_position.y:
                        commands.append(f"SNAP{states[i].screenshot_id}_L")
                    elif current_ob_dict['y'] == current_robot_position.y:
                        commands.append(f"SNAP{states[i].screenshot_id}_C")
                    elif current_ob_dict['y'] < current_robot_position.y:
                        commands.append(f"SNAP{states[i].screenshot_id}_R")
                    else:
                        commands.append(f"SNAP{states[i].screenshot_id}")
                
                # Obstacle facing EAST, robot facing WEST
                elif current_ob_dict['d'] == 2 and current_robot_position.direction == 6:
                    if current_ob_dict['y'] > current_robot_position.y:
                        commands.append(f"SNAP{states[i].screenshot_id}_R")
                    elif current_ob_dict['y'] == current_robot_position.y:
                        commands.append(f"SNAP{states[i].screenshot_id}_C")
                    elif current_ob_dict['y'] < current_robot_position.y:
                        commands.append(f"SNAP{states[i].screenshot_id}_L")
                    else:
                        commands.append(f"SNAP{states[i].screenshot_id}")

                # Obstacle facing NORTH, robot facing SOUTH
                elif current_ob_dict['d'] == 0 and current_robot_position.direction == 4:
                    if current_ob_dict['x'] > current_robot_position.x:
                        commands.append(f"SNAP{states[i].screenshot_id}_L")
                    elif current_ob_dict['x'] == current_robot_position.x:
                        commands.append(f"SNAP{states[i].screenshot_id}_C")
                    elif current_ob_dict['x'] < current_robot_position.x:
                        commands.append(f"SNAP{states[i].screenshot_id}_R")
                    else:
                        commands.append(f"SNAP{states[i].screenshot_id}")

                # Obstacle facing SOUTH, robot facing NORTH
                elif current_ob_dict['d'] == 4 and current_robot_position.direction == 0:
                    if current_ob_dict['x'] > current_robot_position.x:
                        commands.append(f"SNAP{states[i].screenshot_id}_R")
                    elif current_ob_dict['x'] == current_robot_position.x:
                        commands.append(f"SNAP{states[i].screenshot_id}_C")
                    elif current_ob_dict['x'] < current_robot_position.x:
                        commands.append(f"SNAP{states[i].screenshot_id}_L")
                    else:
                        commands.append(f"SNAP{states[i].screenshot_id}")
            continue

        # If previous state and current state are not the same direction, it means that there will be a turn command involved
        # Assume there are 4 turning command: FR, FL, BL, BR (the turn command will turn the robot 90 degrees)
        # FR00 | FR30: Forward Right;
        # FL00 | FL30: Forward Left;
        # BR00 | BR30: Backward Right;
        # BL00 | BL30: Backward Left;

        # Facing north previously
        if states[i - 1].direction == Direction.NORTH:
            # Facing east afterwards
            if states[i].direction == Direction.EAST:
                # y value increased -> Forward Right
                if states[i].y > states[i - 1].y:
                    commands.append("FR{}".format(steps))
                # y value decreased -> Backward Left
                else:
                    commands.append("BL{}".format(steps))
            # Facing west afterwards
            elif states[i].direction == Direction.WEST:
                # y value increased -> Forward Left
                if states[i].y > states[i - 1].y:
                    commands.append("FL{}".format(steps))
                # y value decreased -> Backward Right
                else:
                    commands.append("BR{}".format(steps))
            else:
                raise Exception("Invalid turing direction")

        elif states[i - 1].direction == Direction.EAST:
            if states[i].direction == Direction.NORTH:
                if states[i].y > states[i - 1].y:
                    commands.append("FL{}".format(steps))
                else:
                    commands.append("BR{}".format(steps))

            elif states[i].direction == Direction.SOUTH:
                if states[i].y > states[i - 1].y:
                    commands.append("BL{}".format(steps))
                else:
                    commands.append("FR{}".format(steps))
            else:
                raise Exception("Invalid turing direction")

        elif states[i - 1].direction == Direction.SOUTH:
            if states[i].direction == Direction.EAST:
                if states[i].y > states[i - 1].y:
                    commands.append("BR{}".format(steps))
                else:
                    commands.append("FL{}".format(steps))
            elif states[i].direction == Direction.WEST:
                if states[i].y > states[i - 1].y:
                    commands.append("BL{}".format(steps))
                else:
                    commands.append("FR{}".format(steps))
            else:
                raise Exception("Invalid turing direction")

        elif states[i - 1].direction == Direction.WEST:
            if states[i].direction == Direction.NORTH:
                if states[i].y > states[i - 1].y:
                    commands.append("FR{}".format(steps))
                else:
                    commands.append("BL{}".format(steps))
            elif states[i].direction == Direction.SOUTH:
                if states[i].y > states[i - 1].y:
                    commands.append("BR{}".format(steps))
                else:
                    commands.append("FL{}".format(steps))
            else:
                raise Exception("Invalid turing direction")
        else:
            raise Exception("Invalid position")

        # If any of these states has a valid screenshot ID, then add a SNAP command as well to take a picture
        if states[i].screenshot_id != -1:  
            # NORTH = 0
            # EAST = 2
            # SOUTH = 4
            # WEST = 6

            current_ob_dict = obstacles_dict[states[i].screenshot_id] # {'x': 9, 'y': 10, 'd': 6, 'id': 9}
            current_robot_position = states[i] # {'x': 1, 'y': 8, 'd': <Direction.NORTH: 0>, 's': -1}

            # Obstacle facing WEST, robot facing EAST
            if current_ob_dict['d'] == 6 and current_robot_position.direction == 2:
                if current_ob_dict['y'] > current_robot_position.y:
                    commands.append(f"SNAP{states[i].screenshot_id}_L")
                elif current_ob_dict['y'] == current_robot_position.y:
                    commands.append(f"SNAP{states[i].screenshot_id}_C")
                elif current_ob_dict['y'] < current_robot_position.y:
                    commands.append(f"SNAP{states[i].screenshot_id}_R")
                else:
                    commands.append(f"SNAP{states[i].screenshot_id}")
            
            # Obstacle facing EAST, robot facing WEST
            elif current_ob_dict['d'] == 2 and current_robot_position.direction == 6:
                if current_ob_dict['y'] > current_robot_position.y:
                    commands.append(f"SNAP{states[i].screenshot_id}_R")
                elif current_ob_dict['y'] == current_robot_position.y:
                    commands.append(f"SNAP{states[i].screenshot_id}_C")
                elif current_ob_dict['y'] < current_robot_position.y:
                    commands.append(f"SNAP{states[i].screenshot_id}_L")
                else:
                    commands.append(f"SNAP{states[i].screenshot_id}")

            # Obstacle facing NORTH, robot facing SOUTH
            elif current_ob_dict['d'] == 0 and current_robot_position.direction == 4:
                if current_ob_dict['x'] > current_robot_position.x:
                    commands.append(f"SNAP{states[i].screenshot_id}_L")
                elif current_ob_dict['x'] == current_robot_position.x:
                    commands.append(f"SNAP{states[i].screenshot_id}_C")
                elif current_ob_dict['x'] < current_robot_position.x:
                    commands.append(f"SNAP{states[i].screenshot_id}_R")
                else:
                    commands.append(f"SNAP{states[i].screenshot_id}")

            # Obstacle facing SOUTH, robot facing NORTH
            elif current_ob_dict['d'] == 4 and current_robot_position.direction == 0:
                if current_ob_dict['x'] > current_robot_position.x:
                    commands.append(f"SNAP{states[i].screenshot_id}_R")
                elif current_ob_dict['x'] == current_robot_position.x:
                    commands.append(f"SNAP{states[i].screenshot_id}_C")
                elif current_ob_dict['x'] < current_robot_position.x:
                    commands.append(f"SNAP{states[i].screenshot_id}_L")
                else:
                    commands.append(f"SNAP{states[i].screenshot_id}")

    # Final command is the stop command (FN)
    commands.append("FN")

    # Compress commands if there are consecutive forward or backward commands
    compressed_commands = [commands[0]]

    for i in range(1, len(commands)):
        # If both commands are BW
        if commands[i].startswith("BW") and compressed_commands[-1].startswith("BW"):
            # Get the number of steps of previous command
            steps = int(compressed_commands[-1][2:])
            # If steps are not 90, add 10 to the steps
            if steps != 90:
                compressed_commands[-1] = "BW{}".format(steps + 10)
                continue

        # If both commands are FW
        elif commands[i].startswith("FW") and compressed_commands[-1].startswith("FW"):
            # Get the number of steps of previous command
            steps = int(compressed_commands[-1][2:])
            # If steps are not 90, add 10 to the steps
            if steps != 90:
                compressed_commands[-1] = "FW{}".format(steps + 10)
                continue
        
        # Otherwise, just add as usual
        compressed_commands.append(commands[i])

    return compressed_commands


class Direction(int, Enum):
    NORTH = 0
    EAST = 2
    SOUTH = 4
    WEST = 6
    SKIP = 8

    def __int__(self):
        return self.value

    @staticmethod
    def rotation_cost(d1, d2):
        diff = abs(d1 - d2)
        return min(diff, 8 - diff)

MOVE_DIRECTION = [
    (1, 0, Direction.EAST),
    (-1, 0, Direction.WEST),
    (0, 1, Direction.NORTH),
    (0, -1, Direction.SOUTH),
]

TURN_FACTOR = 1

EXPANDED_CELL = 1 # for both agent and obstacles

WIDTH = 20
HEIGHT = 20

ITERATIONS = 2000
TURN_RADIUS = 1
turn_wrt_big_turns = [[3 * TURN_RADIUS, TURN_RADIUS],
                  [4 * TURN_RADIUS, 2 * TURN_RADIUS]]

SAFE_COST = 1000 # the cost for the turn in case there is a chance that the robot is touch some obstacle
SCREENSHOT_COST = 50 # the cost for the place where the picture is taken

def is_valid(center_x: int, center_y: int):
    """Checks if given position is within bounds

    Inputs
    ------
    center_x (int): x-coordinate
    center_y (int): y-coordinate

    Returns
    -------
    bool: True if valid, False otherwise
    """
    return center_x > 0 and center_y > 0 and center_x < WIDTH - 1 and center_y < HEIGHT - 1





class CellState:
    """Base class for all objects on the arena, such as cells, obstacles, etc"""

    def __init__(self, x, y, direction: Direction = Direction.NORTH, screenshot_id=-1, penalty=0):
        self.x = x
        self.y = y
        self.direction = direction
        # If screenshot_od != -1, the snapshot is taken at that position is for the obstacle with id = screenshot_id
        self.screenshot_id = screenshot_id
        self.penalty = penalty  # Penalty for the view point of taking picture

    def cmp_position(self, x, y) -> bool:
        """Compare given (x,y) position with cell state's position

        Args:
            x (int): x coordinate
            y (int): y coordinate

        Returns:
            bool: True if same, False otherwise
        """
        return self.x == x and self.y == y

    def is_eq(self, x, y, direction):
        """Compare given x, y, direction with cell state's position and direction

        Args:
            x (int): x coordinate
            y (int): y coordinate
            direction (Direction): direction of cell

        Returns:
            bool: True if same, False otherwise
        """
        return self.x == x and self.y == y and self.direction == direction

    def __repr__(self):
        return "x: {}, y: {}, d: {}, screenshot: {}".format(self.x, self.y, self.direction, self.screenshot_id)

    def set_screenshot(self, screenshot_id):
        """Set screenshot id for cell

        Args:
            screenshot_id (int): screenshot id of cell
        """
        self.screenshot_id = screenshot_id

    def get_dict(self):
        """Returns a dictionary representation of the cell

        Returns:
            dict: {x,y,direction,screeshot_id}
        """
        return {'x': self.x, 'y': self.y, 'd': self.direction, 's': self.screenshot_id}


class Obstacle(CellState):
    """Obstacle class, inherited from CellState"""

    def __init__(self, x: int, y: int, direction: Direction, obstacle_id: int):
        super().__init__(x, y, direction)
        self.obstacle_id = obstacle_id

    def __eq__(self, other):
        """Checks if this obstacle is the same as input in terms of x, y, and direction

        Args:
            other (Obstacle): input obstacle to compare to

        Returns:
            bool: True if same, False otherwise
        """
        return self.x == other.x and self.y == other.y and self.direction == other.direction

    def get_view_state(self, retrying) -> List[CellState]:
        """Constructs the list of CellStates from which the robot can view the symbol on the obstacle

        Returns:
            List[CellState]: Valid cell states where robot can be positioned to view the symbol on the obstacle
        """
        cells = []

        # If the obstacle is facing north, then robot's cell state must be facing south
        if self.direction == Direction.NORTH:
            if retrying == False:
                # Or (x, y + 3)
                if is_valid(self.x, self.y + 1 + EXPANDED_CELL * 2):
                    cells.append(CellState(
                        self.x, self.y + 1 + EXPANDED_CELL * 2, Direction.SOUTH, self.obstacle_id, 5))
                # Or (x, y + 4)
                if is_valid(self.x, self.y + 2 + EXPANDED_CELL * 2):
                    cells.append(CellState(
                        self.x, self.y + 2 + EXPANDED_CELL * 2, Direction.SOUTH, self.obstacle_id, 0))

                # Or (x + 1, y + 3)
                # if is_valid(self.x + 1, self.y + 1 + EXPANDED_CELL * 2):
                #     cells.append(CellState(self.x + 1, self.y + 1 + EXPANDED_CELL * 2, Direction.SOUTH, self.obstacle_id, SCREENSHOT_COST*10))
                # # Or (x - 1, y + 3)
                # if is_valid(self.x - 1, self.y + 1 + EXPANDED_CELL * 2):
                #     cells.append(CellState(self.x - 1, self.y + 1 + EXPANDED_CELL * 2, Direction.SOUTH, self.obstacle_id, SCREENSHOT_COST*10))

                # Or (x + 1, y + 4)
                if is_valid(self.x + 1, self.y + 2 + EXPANDED_CELL * 2):
                    cells.append(CellState(self.x + 1, self.y + 2 + EXPANDED_CELL *
                                 2, Direction.SOUTH, self.obstacle_id, SCREENSHOT_COST))
                # Or (x - 1, y + 4)
                if is_valid(self.x - 1, self.y + 2 + EXPANDED_CELL * 2):
                    cells.append(CellState(self.x - 1, self.y + 2 + EXPANDED_CELL *
                                 2, Direction.SOUTH, self.obstacle_id, SCREENSHOT_COST))

            elif retrying == True:
                # Or (x, y + 4)
                if is_valid(self.x, self.y + 2 + EXPANDED_CELL * 2):
                    cells.append(CellState(
                        self.x, self.y + 2 + EXPANDED_CELL * 2, Direction.SOUTH, self.obstacle_id, 0))
                # Or (x, y + 5)
                if is_valid(self.x, self.y + 3 + EXPANDED_CELL * 2):
                    cells.append(CellState(
                        self.x, self.y + 3 + EXPANDED_CELL * 2, Direction.SOUTH, self.obstacle_id, 0))
                # Or (x + 1, y + 4)
                if is_valid(self.x + 1, self.y + 2 + EXPANDED_CELL * 2):
                    cells.append(CellState(self.x + 1, self.y + 2 + EXPANDED_CELL *
                                 2, Direction.SOUTH, self.obstacle_id, SCREENSHOT_COST))
                # Or (x - 1, y + 4)
                if is_valid(self.x - 1, self.y + 2 + EXPANDED_CELL * 2):
                    cells.append(CellState(self.x - 1, self.y + 2 + EXPANDED_CELL *
                                 2, Direction.SOUTH, self.obstacle_id, SCREENSHOT_COST))

        # If obstacle is facing south, then robot's cell state must be facing north
        elif self.direction == Direction.SOUTH:

            if retrying == False:
                # Or (x, y - 3)
                if is_valid(self.x, self.y - 1 - EXPANDED_CELL * 2):
                    cells.append(CellState(
                        self.x, self.y - 1 - EXPANDED_CELL * 2, Direction.NORTH, self.obstacle_id, 5))
                # Or (x, y - 4)
                if is_valid(self.x, self.y - 2 - EXPANDED_CELL * 2):
                    cells.append(CellState(
                        self.x, self.y - 2 - EXPANDED_CELL * 2, Direction.NORTH, self.obstacle_id, 0))

                # Or (x + 1, y - 3)
                # if is_valid(self.x + 1, self.y - 1 - EXPANDED_CELL * 2):
                #     cells.append(CellState(self.x + 1, self.y - 1 - EXPANDED_CELL * 2, Direction.NORTH, self.obstacle_id, SCREENSHOT_COST*10))
                # # Or (x - 1, y - 3)
                # if is_valid(self.x - 1, self.y - 1 - EXPANDED_CELL * 2):
                #     cells.append(CellState(self.x - 1, self.y - 1 - EXPANDED_CELL * 2, Direction.NORTH, self.obstacle_id, SCREENSHOT_COST*10))

                # Or (x + 1, y - 4)
                if is_valid(self.x + 1, self.y - 2 - EXPANDED_CELL * 2):
                    cells.append(CellState(self.x + 1, self.y - 2 - EXPANDED_CELL *
                                 2, Direction.NORTH, self.obstacle_id, SCREENSHOT_COST))
                # Or (x - 1, y - 4)
                if is_valid(self.x - 1, self.y - 2 - EXPANDED_CELL * 2):
                    cells.append(CellState(self.x - 1, self.y - 2 - EXPANDED_CELL *
                                 2, Direction.NORTH, self.obstacle_id, SCREENSHOT_COST))

            elif retrying == True:
                # Or (x, y - 4)
                if is_valid(self.x, self.y - 2 - EXPANDED_CELL * 2):
                    cells.append(CellState(
                        self.x, self.y - 2 - EXPANDED_CELL * 2, Direction.NORTH, self.obstacle_id, 0))
                # Or (x, y - 5)
                if is_valid(self.x, self.y - 3 - EXPANDED_CELL * 2):
                    cells.append(CellState(
                        self.x, self.y - 3 - EXPANDED_CELL * 2, Direction.NORTH, self.obstacle_id, 0))
                # Or (x + 1, y - 4)
                if is_valid(self.x + 1, self.y - 2 - EXPANDED_CELL * 2):
                    cells.append(CellState(self.x + 1, self.y - 2 - EXPANDED_CELL *
                                 2, Direction.NORTH, self.obstacle_id, SCREENSHOT_COST))
                # Or (x - 1, y - 4)
                if is_valid(self.x - 1, self.y - 2 - EXPANDED_CELL * 2):
                    cells.append(CellState(self.x - 1, self.y - 2 - EXPANDED_CELL *
                                 2, Direction.NORTH, self.obstacle_id, SCREENSHOT_COST))

        # If obstacle is facing east, then robot's cell state must be facing west
        elif self.direction == Direction.EAST:

            if retrying == False:
                # Or (x + 3,y)
                if is_valid(self.x + 1 + EXPANDED_CELL * 2, self.y):
                    cells.append(CellState(self.x + 1 + EXPANDED_CELL * 2,
                                 self.y, Direction.WEST, self.obstacle_id, 5))
                # Or (x + 4,y)
                if is_valid(self.x + 2 + EXPANDED_CELL * 2, self.y):
                    # print(f"Obstacle facing east, Adding {self.x + 2 + EXPANDED_CELL * 2}, {self.y}")
                    cells.append(CellState(self.x + 2 + EXPANDED_CELL * 2,
                                 self.y, Direction.WEST, self.obstacle_id, 0))

                # Or (x + 3,y + 1)
                # if is_valid(self.x + 1 + EXPANDED_CELL * 2, self.y + 1):
                #     #print(f"Obstacle facing east, Adding {self.x + 2 + EXPANDED_CELL * 2}, {self.y + 1}")
                #     cells.append(CellState(self.x + 1 + EXPANDED_CELL * 2, self.y + 1, Direction.WEST, self.obstacle_id, SCREENSHOT_COST*10))
                # # Or (x + 3,y - 1)
                # if is_valid(self.x + 1 + EXPANDED_CELL * 2, self.y - 1):
                #     #print(f"Obstacle facing east, Adding {self.x + 2 + EXPANDED_CELL * 2}, {self.y - 1}")
                #     cells.append(CellState(self.x + 1 + EXPANDED_CELL * 2, self.y - 1, Direction.WEST, self.obstacle_id, SCREENSHOT_COST*10))

                # Or (x + 4, y + 1)
                if is_valid(self.x + 2 + EXPANDED_CELL * 2, self.y + 1):
                    cells.append(CellState(self.x + 2 + EXPANDED_CELL * 2, self.y +
                                 1, Direction.WEST, self.obstacle_id, SCREENSHOT_COST))
                # Or (x + 4, y - 1)
                if is_valid(self.x + 2 + EXPANDED_CELL * 2, self.y - 1):
                    cells.append(CellState(self.x + 2 + EXPANDED_CELL * 2, self.y -
                                 1, Direction.WEST, self.obstacle_id, SCREENSHOT_COST))

            elif retrying == True:
                # Or (x + 4, y)
                if is_valid(self.x + 2 + EXPANDED_CELL * 2, self.y):
                    cells.append(CellState(self.x + 2 + EXPANDED_CELL * 2,
                                 self.y, Direction.WEST, self.obstacle_id, 0))
                # Or (x + 5, y)
                if is_valid(self.x + 3 + EXPANDED_CELL * 2, self.y):
                    cells.append(CellState(self.x + 3 + EXPANDED_CELL * 2,
                                 self.y, Direction.WEST, self.obstacle_id, 0))
                # Or (x + 4,y + 1)
                if is_valid(self.x + 2 + EXPANDED_CELL * 2, self.y + 1):
                    cells.append(CellState(self.x + 2 + EXPANDED_CELL * 2, self.y +
                                 1, Direction.WEST, self.obstacle_id, SCREENSHOT_COST))
                # Or (x + 4,y - 1)
                if is_valid(self.x + 2 + EXPANDED_CELL * 2, self.y - 1):
                    cells.append(CellState(self.x + 2 + EXPANDED_CELL * 2, self.y -
                                 1, Direction.WEST, self.obstacle_id, SCREENSHOT_COST))

        # If obstacle is facing west, then robot's cell state must be facing east
        elif self.direction == Direction.WEST:
            # It can be (x - 2,y)
            # if is_valid(self.x - EXPANDED_CELL * 2, self.y):
            #     cells.append(CellState(self.x - EXPANDED_CELL * 2, self.y, Direction.EAST, self.obstacle_id, 0))

            if retrying == False:
                # Or (x - 3, y)
                if is_valid(self.x - 1 - EXPANDED_CELL * 2, self.y):
                    cells.append(CellState(self.x - 1 - EXPANDED_CELL * 2,
                                 self.y, Direction.EAST, self.obstacle_id, 5))
                # Or (x - 4, y)
                if is_valid(self.x - 2 - EXPANDED_CELL * 2, self.y):
                    cells.append(CellState(self.x - 2 - EXPANDED_CELL * 2,
                                 self.y, Direction.EAST, self.obstacle_id, 0))

                # Or (x - 3,y + 1)
                # if is_valid(self.x - 1 - EXPANDED_CELL * 2, self.y + 1):
                #     cells.append(CellState(self.x - 1 - EXPANDED_CELL * 2, self.y + 1, Direction.EAST, self.obstacle_id, SCREENSHOT_COST*10))
                # # Or (x - 3,y - 1)
                # if is_valid(self.x - 1 - EXPANDED_CELL * 2, self.y - 1):
                #     cells.append(CellState(self.x - 1 - EXPANDED_CELL * 2, self.y - 1, Direction.EAST, self.obstacle_id, SCREENSHOT_COST*10))

                # Or (x - 4, y + 1)
                if is_valid(self.x - 2 - EXPANDED_CELL * 2, self.y + 1):
                    cells.append(CellState(self.x - 2 - EXPANDED_CELL * 2, self.y +
                                 1, Direction.EAST, self.obstacle_id, SCREENSHOT_COST))
                # Or (x - 4, y - 1)
                if is_valid(self.x - 2 - EXPANDED_CELL * 2, self.y - 1):
                    cells.append(CellState(self.x - 2 - EXPANDED_CELL * 2, self.y -
                                 1, Direction.EAST, self.obstacle_id, SCREENSHOT_COST))

            elif retrying == True:
                # Or (x - 4, y)
                if is_valid(self.x - 2 - EXPANDED_CELL * 2, self.y):
                    cells.append(CellState(self.x - 2 - EXPANDED_CELL * 2,
                                 self.y, Direction.EAST, self.obstacle_id, 0))
                # Or (x - 5, y)
                if is_valid(self.x - 3 - EXPANDED_CELL * 2, self.y):
                    cells.append(CellState(self.x - 3 - EXPANDED_CELL * 2,
                                 self.y, Direction.EAST, self.obstacle_id, 0))
                # Or (x - 4, y + 1)
                if is_valid(self.x - 2 - EXPANDED_CELL * 2, self.y + 1):
                    cells.append(CellState(self.x - 2 - EXPANDED_CELL * 2, self.y +
                                 1, Direction.EAST, self.obstacle_id, SCREENSHOT_COST))
                # Or (x - 4, y - 1)
                if is_valid(self.x - 2 - EXPANDED_CELL * 2, self.y - 1):
                    cells.append(CellState(self.x - 2 - EXPANDED_CELL * 2, self.y -
                                 1, Direction.EAST, self.obstacle_id, SCREENSHOT_COST))

        return cells


class Grid:
    """
    Grid object that contains the size of the grid and a list of obstacles
    """
    def __init__(self, size_x: int, size_y: int):
        """
        Args:
            size_x (int): Size of the grid in the x direction
            size_y (int): Size of the grid in the y direction
        """
        self.size_x = size_x
        self.size_y = size_y
        self.obstacles: List[Obstacle] = []

    def add_obstacle(self, obstacle: Obstacle):
        """Add a new obstacle to the Grid object, ignores if duplicate obstacle

        Args:
            obstacle (Obstacle): Obstacle to be added
        """
        # Loop through the existing obstacles to check for duplicates
        to_add = True
        for ob in self.obstacles:
            if ob == obstacle:
                to_add = False
                break

        if to_add:
            self.obstacles.append(obstacle)

    def reset_obstacles(self):
        """
        Resets the obstacles in the grid
        """
        self.obstacles = []

    def get_obstacles(self):
        """
        Returns the list of obstacles in the grid
        """
        return self.obstacles

    def reachable(self, x: int, y: int, turn=False, preTurn=False) -> bool:
        """Checks whether the given x,y coordinate is reachable/safe. Criterion is as such:
        - Must be at least 4 units away in total (x+y) from the obstacle
        - Greater distance (x or y distance) must be at least 3 units away from obstacle

        Args:
            x (int): _description_
            y (int): _description_

        Returns:
            bool: _description_
        """
        
        if not self.is_valid_coord(x, y):
            return False

        for ob in self.obstacles:
            # print(f"Looking at position x:{x} y:{y} against ob: {ob.x} {ob.y}")
            if ob.x == 4 and ob.y <= 4 and x < 4 and y < 4:
                # print(f"ob.x: {ob.x} ob.y: {ob.y} x: {x} y:{y} Triggered four bypass")
                continue

            # if x <= 3 and y <= 4:
            #     continue

            # Must be at least 4 units away in total (x+y)
            if abs(ob.x - x) + abs(ob.y - y) >= 4:
                # print(f"ob.x: {ob.x} ob.y: {ob.y} x: {x} y:{y} Triggered more than 3 units bypass")
                continue
            # If max(x,y) is less than 3 units away, consider not reachable
            # if max(abs(ob.x - x), abs(ob.y - y)) < EXPANDED_CELL * 2 + 1:
            if turn:
                if max(abs(ob.x - x), abs(ob.y - y)) < EXPANDED_CELL * 3 + 1:
                    # if ob.x == 0 and ob.y == 10 and x == 1 and y == 12:
                    #     print(f"ob.x: {ob.x} ob.y: {ob.y} x: {x} y:{y} Triggered less than 3 max units trap")
                    return False
            if preTurn:
                if max(abs(ob.x - x), abs(ob.y - y)) < EXPANDED_CELL * 3 + 1:
                    # if ob.x == 0 and ob.y == 10 and x == 1 and y == 12:
                    #     print(f"ob.x: {ob.x} ob.y: {ob.y} x: {x} y:{y} Triggered less than 3 max units trap")
                    return False
            else:
                if max(abs(ob.x - x), abs(ob.y - y)) < 3:
                    # print(f"ob.x: {ob.x} ob.y: {ob.y} x: {x} y:{y} Triggered less than 3 max units trap")
                    return False

        return True

    def is_valid_coord(self, x: int, y: int) -> bool:
        """Checks if given position is within bounds

        Args:
            x (int): x-coordinate
            y (int): y-coordinate

        Returns:
            bool: True if valid, False otherwise
        """
        if x < 1 or x >= self.size_x - 1 or y < 1 or y >= self.size_y - 1:
            return False

        return True

    def is_valid_cell_state(self, state: CellState) -> bool:
        """Checks if given state is within bounds

        Args:
            state (CellState)

        Returns:
            bool: True if valid, False otherwise
        """
        return self.is_valid_coord(state.x, state.y)

    def get_view_obstacle_positions(self, retrying) -> List[List[CellState]]:
        """
        This function return a list of desired states for the robot to achieve based on the obstacle position and direction.
        The state is the position that the robot can see the image of the obstacle and is safe to reach without collision
        :return: [[CellState]]
        """
        # print(f"Inside get_view_obstacle_positions: retrying = {retrying}")
        optimal_positions = []
        for obstacle in self.obstacles:
            if obstacle.direction == 8:
                continue
            else:
                view_states = [view_state for view_state in obstacle.get_view_state(
                    retrying) if self.reachable(view_state.x, view_state.y)]
            optimal_positions.append(view_states)

        return optimal_positions
        
class Robot:
    def __init__(self, center_x: int, center_y: int, start_direction: Direction):
        """Robot object class

        Args:
            center_x (int): x coordinate of center of robot
            center_y (int): y coordinate of center of robot
            start_direction (Direction): Direction robot is facing at the start

        Internals:
            states: List of cell states of the robot's historical path
        """
        self.states: List[CellState] = [
            CellState(center_x, center_y, start_direction)]

    def get_start_state(self):
        """Returns the starting cell state of the robot

        Returns:
            CellState: starting cell state of robot (x,y,d)
        """
        return self.states[0]



class MazeSolver:
    def __init__(
            self,
            size_x: int,
            size_y: int,
            robot_x: int,
            robot_y: int,
            robot_direction: Direction,
            big_turn=None # the big_turn here is to allow 3-1 turn(0 - by default) | 4-2 turn(1)
    ):
        # Initialize a Grid object for the arena representation
        self.grid = Grid(size_x, size_y)
        # Initialize a Robot object for robot representation
        self.robot = Robot(robot_x, robot_y, robot_direction)
        # Create tables for paths and costs
        self.path_table = dict()
        self.cost_table = dict()
        if big_turn is None:
            self.big_turn = 0
        else:
            self.big_turn = int(big_turn)

    def add_obstacle(self, x: int, y: int, direction: Direction, obstacle_id: int):
        """Add obstacle to MazeSolver object

        Args:
            x (int): x coordinate of obstacle
            y (int): y coordinate of obstacle
            direction (Direction): Direction of obstacle
            obstacle_id (int): ID of obstacle
        """
        # Create an obstacle object
        obstacle = Obstacle(x, y, direction, obstacle_id)
        # Add created obstacle to grid object
        self.grid.add_obstacle(obstacle)

    def reset_obstacles(self):
        self.grid.reset_obstacles()

    @staticmethod
    def compute_coord_distance(x1: int, y1: int, x2: int, y2: int, level=1):
        """Compute the L-n distance between two coordinates

        Args:
            x1 (int)
            y1 (int)
            x2 (int)
            y2 (int)
            level (int, optional): L-n distance to compute. Defaults to 1.

        Returns:
            float: L-n distance between the two given points
        """
        horizontal_distance = x1 - x2
        vertical_distance = y1 - y2

        # Euclidean distance
        if level == 2:
            return math.sqrt(horizontal_distance ** 2 + vertical_distance ** 2)

        return abs(horizontal_distance) + abs(vertical_distance)

    @staticmethod
    def compute_state_distance(start_state: CellState, end_state: CellState, level=1):
        """Compute the L-n distance between two cell states

        Args:
            start_state (CellState): Start cell state
            end_state (CellState): End cell state
            level (int, optional): L-n distance to compute. Defaults to 1.

        Returns:
            float: L-n distance between the two given cell states
        """
        return MazeSolver.compute_coord_distance(start_state.x, start_state.y, end_state.x, end_state.y, level)

    @staticmethod
    def get_visit_options(n):
        """Generate all possible n-digit binary strings

        Args:
            n (int): number of digits in binary string to generate

        Returns:
            List: list of all possible n-digit binary strings
        """
        s = []
        l = bin(2 ** n - 1).count('1')

        for i in range(2 ** n):
            s.append(bin(i)[2:].zfill(l))

        s.sort(key=lambda x: x.count('1'), reverse=True)
        return s

    def get_optimal_order_dp(self, retrying) -> List[CellState]:
        distance = 1e9
        optimal_path = []

        #print(f"Inside get_optimal_order_dp: retrying = {retrying}")
        # Get all possible positions that can view the obstacles
        all_view_positions = self.grid.get_view_obstacle_positions(retrying)
        #print(f"all_view_positions: {all_view_positions}")
        #print(f"All view position: {all_view_positions}")

        for op in self.get_visit_options(len(all_view_positions)):
            # op is binary string of length len(all_view_positions) == len(obstacles)
            # If index == 1 means the view_positions[index] is selected to visit, otherwise drop

            # Calculate optimal_cost table

            # Initialize `items` to be a list containing the robot's start state as the first item
            items = [self.robot.get_start_state()]
            # Initialize `cur_view_positions` to be an empty list
            cur_view_positions = []
            
            # print(f"===================\nop = {op}")
            # print("List of obstacle visited: \n")

            # For each obstacle
            for idx in range(len(all_view_positions)):
                # If robot is visiting
                if op[idx] == '1':
                    # Add possible cells to `items`
                    items = items + all_view_positions[idx]
                    # Add possible cells to `cur_view_positions`
                    cur_view_positions.append(all_view_positions[idx])
                    #print("obstacle: {}\n".format(self.grid.obstacles[idx]))

            # Generate the path cost for the items
            self.path_cost_generator(items)
            combination = []
            self.generate_combination(cur_view_positions, 0, [], combination, [ITERATIONS])

            for c in combination: # run the algo some times ->
                visited_candidates = [0] # add the start state of the robot

                cur_index = 1
                fixed_cost = 0 # the cost applying for the position taking obstacle pictures
                for index, view_position in enumerate(cur_view_positions):
                    visited_candidates.append(cur_index + c[index])
                    fixed_cost += view_position[c[index]].penalty
                    cur_index += len(view_position)
                
                cost_np = np.zeros((len(visited_candidates), len(visited_candidates)))

                for s in range(len(visited_candidates) - 1):
                    for e in range(s + 1, len(visited_candidates)):
                        u = items[visited_candidates[s]]
                        v = items[visited_candidates[e]]
                        if (u, v) in self.cost_table.keys():
                            cost_np[s][e] = self.cost_table[(u, v)]
                        else:
                            cost_np[s][e] = 1e9
                        cost_np[e][s] = cost_np[s][e]
                cost_np[:, 0] = 0
                _permutation, _distance = solve_tsp_dynamic_programming(cost_np)
                # print(f"fixed_cost = {fixed_cost}")
                # print(f"distance = {_distance}")
                if _distance + fixed_cost >= distance:
                    continue

                optimal_path = [items[0]]
                distance = _distance + fixed_cost

                for i in range(len(_permutation) - 1):
                    from_item = items[visited_candidates[_permutation[i]]]
                    to_item = items[visited_candidates[_permutation[i + 1]]]

                    cur_path = self.path_table[(from_item, to_item)]
                    for j in range(1, len(cur_path)):
                        optimal_path.append(CellState(cur_path[j][0], cur_path[j][1], cur_path[j][2]))

                    optimal_path[-1].set_screenshot(to_item.screenshot_id)

            if optimal_path:
                # if found optimal path, return
                break

        return optimal_path, distance

    @staticmethod
    def generate_combination(view_positions, index, current, result, iteration_left):
        if index == len(view_positions):
            result.append(current[:])
            return

        if iteration_left[0] == 0:
            return

        iteration_left[0] -= 1
        for j in range(len(view_positions[index])):
            current.append(j)
            MazeSolver.generate_combination(view_positions, index + 1, current, result, iteration_left)
            current.pop()

    def get_safe_cost(self, x, y):
        """Get the safe cost of a particular x,y coordinate wrt obstacles that are exactly 2 units away from it in both x and y directions

        Args:
            x (int): x-coordinate
            y (int): y-coordinate

        Returns:
            int: safe cost
        """
        for ob in self.grid.obstacles:
            if abs(ob.x-x) == 2 and abs(ob.y-y) == 2:
                return SAFE_COST
            
            if abs(ob.x-x) == 1 and abs(ob.y-y) == 2:
                return SAFE_COST
            
            if abs(ob.x-x) == 2 and abs(ob.y-y) == 1:
                return SAFE_COST

        return 0

    def get_neighbors(self, x, y, direction):  # TODO: see the behavior of the robot and adjust...
        """
        Return a list of tuples with format:
        newX, newY, new_direction
        """
        # Neighbors have the following format: {newX, newY, movement direction, safe cost}
        # Neighbors are coordinates that fulfill the following criteria:
        # If moving in the same direction:
        #   - Valid position within bounds
        #   - Must be at least 4 units away in total (x+y) 
        #   - Furthest distance must be at least 3 units away (x or y)
        # If it is exactly 2 units away in both x and y directions, safe cost = SAFECOST. Else, safe cost = 0

        neighbors = []
        # Assume that after following this direction, the car direction is EXACTLY md
        for dx, dy, md in MOVE_DIRECTION:
            if md == direction:  # if the new direction == md
                # Check for valid position
                if self.grid.reachable(x + dx, y + dy):  # go forward;
                    # Get safe cost of destination
                    safe_cost = self.get_safe_cost(x + dx, y + dy)
                    neighbors.append((x + dx, y + dy, md, safe_cost))
                # Check for valid position
                if self.grid.reachable(x - dx, y - dy):  # go back;
                    # Get safe cost of destination
                    safe_cost = self.get_safe_cost(x - dx, y - dy)
                    neighbors.append((x - dx, y - dy, md, safe_cost))

            else:  # consider 8 cases
                
                # Turning displacement is either 4-2 or 3-1
                bigger_change = turn_wrt_big_turns[self.big_turn][0]
                smaller_change = turn_wrt_big_turns[self.big_turn][1]

                # north <-> east
                if direction == Direction.NORTH and md == Direction.EAST:

                    # Check for valid position
                    if self.grid.reachable(x + bigger_change, y + smaller_change, turn = True) and self.grid.reachable(x, y, preTurn = True):
                        # Get safe cost of destination
                        safe_cost = self.get_safe_cost(x + bigger_change, y + smaller_change)
                        neighbors.append((x + bigger_change, y + smaller_change, md, safe_cost + 10))

                    # Check for valid position
                    if self.grid.reachable(x - smaller_change, y - bigger_change, turn = True) and self.grid.reachable(x, y, preTurn = True):
                        # Get safe cost of destination
                        safe_cost = self.get_safe_cost(x - smaller_change, y - bigger_change)
                        neighbors.append((x - smaller_change, y - bigger_change, md, safe_cost + 10))

                if direction == Direction.EAST and md == Direction.NORTH:
                    if self.grid.reachable(x + smaller_change, y + bigger_change, turn = True) and self.grid.reachable(x, y, preTurn = True):
                        safe_cost = self.get_safe_cost(x + smaller_change, y + bigger_change)
                        neighbors.append((x + smaller_change, y + bigger_change, md, safe_cost + 10))

                    if self.grid.reachable(x - bigger_change, y - smaller_change, turn = True) and self.grid.reachable(x, y, preTurn = True):
                        safe_cost = self.get_safe_cost(x - bigger_change, y - smaller_change)
                        neighbors.append((x - bigger_change, y - smaller_change, md, safe_cost + 10))

                # east <-> south
                if direction == Direction.EAST and md == Direction.SOUTH:
                    
                    if self.grid.reachable(x + smaller_change, y - bigger_change, turn = True) and self.grid.reachable(x, y, preTurn = True):
                        safe_cost = self.get_safe_cost(x + smaller_change, y - bigger_change)
                        neighbors.append((x + smaller_change, y - bigger_change, md, safe_cost + 10))

                    if self.grid.reachable(x - bigger_change, y + smaller_change, turn = True) and self.grid.reachable(x, y, preTurn = True):
                        safe_cost = self.get_safe_cost(x - bigger_change, y + smaller_change)
                        neighbors.append((x - bigger_change, y + smaller_change, md, safe_cost + 10))

                if direction == Direction.SOUTH and md == Direction.EAST:
                    if self.grid.reachable(x + bigger_change, y - smaller_change, turn = True) and self.grid.reachable(x, y, preTurn = True):
                        safe_cost = self.get_safe_cost(x + bigger_change, y - smaller_change)
                        neighbors.append((x + bigger_change, y - smaller_change, md, safe_cost + 10))

                    if self.grid.reachable(x - smaller_change, y + bigger_change, turn = True) and self.grid.reachable(x, y, preTurn = True):
                        safe_cost = self.get_safe_cost(x - smaller_change, y + bigger_change)
                        neighbors.append((x - smaller_change, y + bigger_change, md, safe_cost + 10))

                # south <-> west
                if direction == Direction.SOUTH and md == Direction.WEST:
                    if self.grid.reachable(x - bigger_change, y - smaller_change, turn = True) and self.grid.reachable(x, y, preTurn = True):
                        safe_cost = self.get_safe_cost(x - bigger_change, y - smaller_change)
                        neighbors.append((x - bigger_change, y - smaller_change, md, safe_cost + 10))

                    if self.grid.reachable(x + smaller_change, y + bigger_change, turn = True) and self.grid.reachable(x, y, preTurn = True):
                        safe_cost = self.get_safe_cost(x + smaller_change, y + bigger_change)
                        neighbors.append((x + smaller_change, y + bigger_change, md, safe_cost + 10))

                if direction == Direction.WEST and md == Direction.SOUTH:
                    if self.grid.reachable(x - smaller_change, y - bigger_change, turn = True) and self.grid.reachable(x, y, preTurn = True):
                        safe_cost = self.get_safe_cost(x - smaller_change, y - bigger_change)
                        neighbors.append((x - smaller_change, y - bigger_change, md, safe_cost + 10))

                    if self.grid.reachable(x + bigger_change, y + smaller_change, turn = True) and self.grid.reachable(x, y, preTurn = True):
                        safe_cost = self.get_safe_cost(x + bigger_change, y + smaller_change)
                        neighbors.append((x + bigger_change, y + smaller_change, md, safe_cost + 10))

                # west <-> north
                if direction == Direction.WEST and md == Direction.NORTH:
                    if self.grid.reachable(x - smaller_change, y + bigger_change, turn = True) and self.grid.reachable(x, y, preTurn = True):
                        safe_cost = self.get_safe_cost(x - smaller_change, y + bigger_change)
                        neighbors.append((x - smaller_change, y + bigger_change, md, safe_cost + 10))

                    if self.grid.reachable(x + bigger_change, y - smaller_change, turn = True) and self.grid.reachable(x, y, preTurn = True):
                        safe_cost = self.get_safe_cost(x + bigger_change, y - smaller_change)
                        neighbors.append((x + bigger_change, y - smaller_change, md, safe_cost + 10))

                if direction == Direction.NORTH and md == Direction.WEST:
                    if self.grid.reachable(x + smaller_change, y - bigger_change, turn = True) and self.grid.reachable(x, y, preTurn = True):
                        safe_cost = self.get_safe_cost(x + smaller_change, y - bigger_change)
                        neighbors.append((x + smaller_change, y - bigger_change, md, safe_cost + 10))

                    if self.grid.reachable(x - bigger_change, y + smaller_change, turn = True) and self.grid.reachable(x, y, preTurn = True):
                        safe_cost = self.get_safe_cost(x - bigger_change, y + smaller_change)
                        neighbors.append((x - bigger_change, y + smaller_change, md, safe_cost + 10))

        return neighbors

    def path_cost_generator(self, states: List[CellState]):
        """Generate the path cost between the input states and update the tables accordingly

        Args:
            states (List[CellState]): cell states to visit
        """
        def record_path(start, end, parent: dict, cost: int):

            # Update cost table for the (start,end) and (end,start) edges
            self.cost_table[(start, end)] = cost
            self.cost_table[(end, start)] = cost

            path = []
            cursor = (end.x, end.y, end.direction)

            while cursor in parent:
                path.append(cursor)
                cursor = parent[cursor]

            path.append(cursor)

            # Update path table for the (start,end) and (end,start) edges, with the (start,end) edge being the reversed path
            self.path_table[(start, end)] = path[::-1]
            self.path_table[(end, start)] = path

        def astar_search(start: CellState, end: CellState):
            # astar search algo with three states: x, y, direction

            # If it is already done before, return
            if (start, end) in self.path_table:
                return

            # Heuristic to guide the search: 'distance' is calculated by f = g + h
            # g is the actual distance moved so far from the start node to current node
            # h is the heuristic distance from current node to end node
            g_distance = {(start.x, start.y, start.direction): 0}

            # format of each item in heap: (f_distance of node, x coord of node, y coord of node)
            # heap in Python is a min-heap
            heap = [(self.compute_state_distance(start, end), start.x, start.y, start.direction)]
            parent = dict()
            visited = set()

            while heap:
                # Pop the node with the smallest distance
                _, cur_x, cur_y, cur_direction = heapq.heappop(heap)
                
                if (cur_x, cur_y, cur_direction) in visited:
                    continue

                if end.is_eq(cur_x, cur_y, cur_direction):
                    record_path(start, end, parent, g_distance[(cur_x, cur_y, cur_direction)])
                    return

                visited.add((cur_x, cur_y, cur_direction))
                cur_distance = g_distance[(cur_x, cur_y, cur_direction)]

                for next_x, next_y, new_direction, safe_cost in self.get_neighbors(cur_x, cur_y, cur_direction):
                    if (next_x, next_y, new_direction) in visited:
                        continue

                    move_cost = Direction.rotation_cost(new_direction, cur_direction) * TURN_FACTOR + 1 + safe_cost

                    # the cost to check if any obstacles that considered too near the robot; if it
                    # safe_cost =

                    # new cost is calculated by the cost to reach current state + cost to move from
                    # current state to new state + heuristic cost from new state to end state
                    next_cost = cur_distance + move_cost + \
                                self.compute_coord_distance(next_x, next_y, end.x, end.y)

                    if (next_x, next_y, new_direction) not in g_distance or \
                            g_distance[(next_x, next_y, new_direction)] > cur_distance + move_cost:
                        g_distance[(next_x, next_y, new_direction)] = cur_distance + move_cost
                        parent[(next_x, next_y, new_direction)] = (cur_x, cur_y, cur_direction)

                        heapq.heappush(heap, (next_cost, next_x, next_y, new_direction))

        # Nested loop through all the state pairings
        for i in range(len(states) - 1):
            for j in range(i + 1, len(states)):
                astar_search(states[i], states[j])



@api.post("/")
def pathfinding():
    print(datetime.now())

    content = request.get_json()

    # Get parameters from JSON
    obstacles = content['obstacles']
    retrying = content['retrying']
    robot_x = content['robot_x']
    robot_y = content['robot_y']
    robot_direction = int(content['robot_dir'])

    # Initialize solver
    maze_solver = MazeSolver(20, 20, robot_x, robot_y, robot_direction, big_turn=None)

    # Add obstacles
    for ob in obstacles:
        maze_solver.add_obstacle(ob['x'], ob['y'], ob['d'], ob['obstacleNumber'])

    start = time.time()

    # Compute optimal path
    optimal_path, distance = maze_solver.get_optimal_order_dp(retrying=retrying)

    print(f"Time taken to find shortest path: {time.time() - start}s")
    print(f"Distance to travel: {distance}")

    # Generate commands
    commands = command_generator(optimal_path, obstacles)

    # Generate path results
    path_results = [optimal_path[0].get_dict()]
    i = 0

    for command in commands:
        if command.startswith("SNAP"):
            continue
        if command.startswith("FN"):
            continue
        elif command.startswith("FW") or command.startswith("FS"):
            i += int(command[2:]) // 10
        elif command.startswith("BW") or command.startswith("BS"):
            i += int(command[2:]) // 10
        else:
            i += 1

        path_results.append(optimal_path[i].get_dict())

    if len(commands) == 1 and commands[0] == "FN":
        commands = ["BW10", f"SNAP{obstacles[0]['obstacleNumber']}", "FN"]

    print(commands)

    response = make_response(
        {
            "data": {
                "distance": distance,
                "path": path_results,
                "commands": commands
            },
            "error": None
        },
        HTTPStatus.OK
    )

    response.mimetype = "application/json"

    print("response", response.get_data(as_text=True))

    return response

def capture():
    if not os.path.exists('.replay'):
        os.makedirs('.replay')

    with open(f'.replay/{datetime.now().strftime('%Y-%m-%d--%H-%M-%S')}.json', 'w') as file:
        file.write(request.get_data(as_text=True))


def dump(world: World, segments: list[Segment]):
    """
    Dumps the path to a txt file for visualization.

    :param world:
    :param segments:
    :return:
    """
    map = np.array(world.grid, dtype=int)
    for obstacle in world.obstacles:
        west_x = max(obstacle.south_west.x, 0)
        east_x = min(obstacle.north_east.x + 1, world.size)
        south_y = max(obstacle.south_west.y, 0)
        north_y = min(obstacle.north_east.y + 1, world.size)

        map[west_x:east_x, south_y:north_y] = 9

    for i, s in enumerate(segments):
        for v in s.vectors:
            map[v.x, v.y] = i + 2

    a = np.rot90(map)
    np.savetxt("dump.txt", a, fmt="%d")

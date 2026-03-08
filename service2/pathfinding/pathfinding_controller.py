from http import HTTPStatus
from datetime import datetime
import json


from flask import make_response
from flask_openapi3 import APIBlueprint, Tag

from pydantic import BaseModel, Field




from flask import request, jsonify
from http import HTTPStatus
from datetime import datetime

api = APIBlueprint(
    "/pathfinding",
    __name__,
    url_prefix="/pathfinding",
    abp_tags=[Tag(name="Pathfinding")],
)

# --------------------------------------------------
# Enums
# --------------------------------------------------



# --------------------------------------------------
# Request Models
# --------------------------------------------------



# --------------------------------------------------
# Solution class (replace with your real planner)
# --------------------------------------------------


from enum import Enum
from typing import List, Dict, Tuple, Optional
import json
from dataclasses import dataclass
from itertools import permutations

class Direction(Enum):
    NORTH = "NORTH"
    SOUTH = "SOUTH"
    EAST = "EAST"
    WEST = "WEST"

class MoveType(Enum):
    FORWARD = "FORWARD"
    BACKWARD = "BACKWARD"
    FORWARD_LEFT = "FORWARD_LEFT"
    FORWARD_RIGHT = "FORWARD_RIGHT"
    BACKWARD_LEFT = "BACKWARD_LEFT"
    BACKWARD_RIGHT = "BACKWARD_RIGHT"

@dataclass
class Point:
    x: float
    y: float

@dataclass
class Rectangle:
    min_x: float
    min_y: float
    max_x: float
    max_y: float

    def intersects(self, other: 'Rectangle') -> bool:
        # Allow touching (edges equal) – no overlap
        return not (self.max_x <= other.min_x or
                    self.min_x >= other.max_x or
                    self.max_y <= other.min_y or
                    self.min_y >= other.max_y)

class RobotState:
    def __init__(self, x: float, y: float, direction: Direction):
        self.x = x
        self.y = y
        self.direction = direction
        self.width = 30
        self.height = 30

    def get_bounding_box(self) -> Rectangle:
        return Rectangle(self.x, self.y, self.x + self.width, self.y + self.height)

    def copy(self):
        return RobotState(self.x, self.y, self.direction)

class CurveParams:
    @staticmethod
    def get(move: MoveType, d: Direction) -> Tuple[float, float, Direction]:
        if move == MoveType.FORWARD_LEFT:
            if d == Direction.NORTH: return (-40, 5, Direction.WEST)
            if d == Direction.EAST:  return (5, 40, Direction.NORTH)
            if d == Direction.SOUTH: return (40, -5, Direction.EAST)
            if d == Direction.WEST:  return (-5, -40, Direction.SOUTH)
        elif move == MoveType.FORWARD_RIGHT:
            if d == Direction.NORTH: return (48, 8, Direction.EAST)
            if d == Direction.EAST:  return (8, -48, Direction.SOUTH)
            if d == Direction.SOUTH: return (-48, -8, Direction.WEST)
            if d == Direction.WEST:  return (-8, 48, Direction.NORTH)
        elif move == MoveType.BACKWARD_LEFT:
            if d == Direction.NORTH: return (-27, -22, Direction.EAST)
            if d == Direction.EAST:  return (-22, 27, Direction.SOUTH)
            if d == Direction.SOUTH: return (27, 22, Direction.WEST)
            if d == Direction.WEST:  return (22, -27, Direction.NORTH)
        elif move == MoveType.BACKWARD_RIGHT:
            if d == Direction.NORTH: return (29, -22, Direction.WEST)
            if d == Direction.EAST:  return (-22, -29, Direction.NORTH)
            if d == Direction.SOUTH: return (-29, 22, Direction.EAST)
            if d == Direction.WEST:  return (22, 29, Direction.SOUTH)
        raise ValueError(f"Invalid move {move} for direction {d}")

class Obstacle:
    def __init__(self, data: Dict, grid_size: int):
        self.grid_size = grid_size
        self.image_id = data["image_id"]
        self.direction = Direction(data["direction"])
        self.sw = Point(data["south_west"]["x"], data["south_west"]["y"])
        self.ne = Point(data["north_east"]["x"], data["north_east"]["y"])
        self.width = self.ne.x - self.sw.x
        self.height = self.ne.y - self.sw.y

    def get_bounding_box(self) -> Rectangle:
        return Rectangle(self.sw.x, self.sw.y, self.ne.x, self.ne.y)

    def required_facing(self) -> Direction:
        if self.direction == Direction.NORTH:
            return Direction.SOUTH
        elif self.direction == Direction.SOUTH:
            return Direction.NORTH
        elif self.direction == Direction.EAST:
            return Direction.WEST
        else:  # WEST
            return Direction.EAST

    def get_target_positions(self) -> List[Tuple[Point, Direction]]:
        w, h = 30, 30
        required_dir = self.required_facing()
        ox, oy = self.sw.x, self.sw.y

        if self.direction == Direction.NORTH:
            x_min = ox - 20
            x_max = ox
            y_min = oy
            y_max = oy + 20
        elif self.direction == Direction.SOUTH:
            x_min = ox - 20
            x_max = ox
            y_min = oy - 50
            y_max = oy - 30
        elif self.direction == Direction.WEST:
            x_min = ox - 50
            x_max = ox - 30
            y_min = oy - 20
            y_max = oy
        else:  # EAST
            x_min = ox
            x_max = ox + 20
            y_min = oy - 20
            y_max = oy

        positions = []
        for rx in range(int(round(x_min)), int(round(x_max)) + 1):
            if rx < 0 or rx > self.grid_size - w:
                continue
            for ry in range(int(round(y_min)), int(round(y_max)) + 1):
                if ry < 0 or ry > self.grid_size - h:
                    continue
                positions.append((Point(float(rx), float(ry)), required_dir))
        return positions

class Solution:
    def __init__(self):
        self.grid_size = 200
        self.tolerance = 15.0
        self.max_straight = 200
        self.obstacles = []

    def solve_path(self, robot: Dict, obstacles: List[Dict]) -> Dict:
        self.obstacles = [Obstacle(obs, self.grid_size) for obs in obstacles]
        start = RobotState(
            robot["south_west"]["x"],
            robot["south_west"]["y"],
            Direction(robot["direction"])
        )

        best_segments = None
        best_count = -1
        best_cost = float('inf')

        # Try every permutation of obstacles
        for perm in permutations(self.obstacles):
            current = start.copy()
            segments = []
            total_cost = 0

            for obs in perm:
                target_list = obs.get_target_positions()
                best_local_cost = float('inf')
                best_local_instr = None

                for target_pos, target_dir in target_list:
                    path = self.find_path(current, target_pos, target_dir)
                    if path:
                        instr, cost = path
                        if cost < best_local_cost:
                            best_local_cost = cost
                            best_local_instr = instr

                if best_local_instr is None:
                    # Skip this obstacle – cannot be visited from current state
                    continue

                best_local_instr.append("CAPTURE_IMAGE")
                segments.append({
                    "cost": best_local_cost,
                    "image_id": obs.image_id,
                    "instructions": best_local_instr
                })
                total_cost += best_local_cost
                current = self.simulate(current, best_local_instr[:-1])

            # Compare with best so far: more segments is better, then lower cost
            if len(segments) > best_count or (len(segments) == best_count and total_cost < best_cost):
                best_count = len(segments)
                best_cost = total_cost
                best_segments = segments

        return {"segments": best_segments if best_segments else []}

    def find_path(self, start: RobotState, target_pos: Point, target_dir: Direction):
        best_instr = None
        best_cost = float('inf')

        straight = self.try_straight(start, target_pos, target_dir)
        if straight:
            instr, cost = straight
            if self.is_path_clear(start, instr):
                if cost < best_cost:
                    best_cost = cost
                    best_instr = instr

        single = self.try_single_curve(start, target_pos, target_dir)
        if single:
            instr, cost = single
            if self.is_path_clear(start, instr):
                if cost < best_cost:
                    best_cost = cost
                    best_instr = instr

        two = self.try_two_curves_with_mid_straight(start, target_pos, target_dir)
        if two:
            instr, cost = two
            if self.is_path_clear(start, instr):
                if cost < best_cost:
                    best_cost = cost
                    best_instr = instr

        return (best_instr, best_cost) if best_instr else None

    def try_straight(self, start: RobotState, target_pos: Point, target_dir: Direction):
        if start.direction != target_dir:
            return None
        dx = target_pos.x - start.x
        dy = target_pos.y - start.y
        if start.direction == Direction.NORTH:
            if abs(dx) > 1e-3: return None
            dist = dy
        elif start.direction == Direction.SOUTH:
            if abs(dx) > 1e-3: return None
            dist = -dy
        elif start.direction == Direction.EAST:
            if abs(dy) > 1e-3: return None
            dist = dx
        else:
            if abs(dy) > 1e-3: return None
            dist = -dx
        dist_int = int(round(dist))
        if abs(dist - dist_int) > 1e-3:
            return None
        dist_10 = int(round(dist_int / 10.0)) * 10
        if start.direction == Direction.NORTH:
            x_end = start.x
            y_end = start.y + dist_10
        elif start.direction == Direction.SOUTH:
            x_end = start.x
            y_end = start.y - dist_10
        elif start.direction == Direction.EAST:
            x_end = start.x + dist_10
            y_end = start.y
        else:
            x_end = start.x - dist_10
            y_end = start.y
        if abs(x_end - target_pos.x) > self.tolerance or abs(y_end - target_pos.y) > self.tolerance:
            return None
        instr = []
        if dist_10 > 0:
            instr.append({"amount": dist_10, "move": "FORWARD"})
        elif dist_10 < 0:
            instr.append({"amount": -dist_10, "move": "BACKWARD"})
        else:
            return None
        cost = abs(dist_10)
        return instr, cost

    def try_single_curve(self, start: RobotState, target_pos: Point, target_dir: Direction):
        if start.direction == Direction.NORTH:
            dx0, dy0 = 0, 1
        elif start.direction == Direction.SOUTH:
            dx0, dy0 = 0, -1
        elif start.direction == Direction.EAST:
            dx0, dy0 = 1, 0
        else:
            dx0, dy0 = -1, 0

        for move in [MoveType.FORWARD_LEFT, MoveType.FORWARD_RIGHT,
                     MoveType.BACKWARD_LEFT, MoveType.BACKWARD_RIGHT]:
            cx, cy, new_dir = CurveParams.get(move, start.direction)
            if new_dir != target_dir:
                continue
            if new_dir == Direction.NORTH:
                dx1, dy1 = 0, 1
            elif new_dir == Direction.SOUTH:
                dx1, dy1 = 0, -1
            elif new_dir == Direction.EAST:
                dx1, dy1 = 1, 0
            else:
                dx1, dy1 = -1, 0
            Cx = target_pos.x - start.x - cx
            Cy = target_pos.y - start.y - cy
            d1 = d2 = None
            if dx0 != 0:
                if abs(dy1) > 1e-6:
                    d2 = Cy / dy1
                    d1 = (Cx - dx1 * d2) / dx0
                else:
                    if abs(Cy) < 1e-6:
                        d2 = 0
                        d1 = (Cx - dx1 * d2) / dx0
                    else:
                        continue
            else:
                if abs(dx1) > 1e-6:
                    d2 = Cx / dx1
                    d1 = (Cy - dy1 * d2) / dy0
                else:
                    if abs(Cx) < 1e-6:
                        d2 = 0
                        d1 = (Cy - dy1 * d2) / dy0
                    else:
                        continue
            if d1 is None or d2 is None:
                continue
            d1_int = int(round(d1))
            d2_int = int(round(d2))
            if abs(d1 - d1_int) > 1e-3 or abs(d2 - d2_int) > 1e-3:
                continue
            def round10(v): return int(round(v / 10.0)) * 10
            d1_10 = round10(d1_int)
            d2_10 = round10(d2_int)
            x_temp = start.x + d1_10 * dx0 + cx + d2_10 * dx1
            y_temp = start.y + d1_10 * dy0 + cy + d2_10 * dy1
            if abs(x_temp - target_pos.x) > self.tolerance or abs(y_temp - target_pos.y) > self.tolerance:
                continue
            instr = []
            if d1_10 > 0:
                instr.append({"amount": d1_10, "move": "FORWARD"})
            elif d1_10 < 0:
                instr.append({"amount": -d1_10, "move": "BACKWARD"})
            instr.append(move.value)
            if d2_10 > 0:
                instr.append({"amount": d2_10, "move": "FORWARD"})
            elif d2_10 < 0:
                instr.append({"amount": -d2_10, "move": "BACKWARD"})
            cost = abs(d1_10) + abs(d2_10) + 30
            return instr, cost
        return None

    def try_two_curves_with_mid_straight(self, start: RobotState, target_pos: Point, target_dir: Direction):
        if start.direction == Direction.NORTH:
            dx0, dy0 = 0, 1
        elif start.direction == Direction.SOUTH:
            dx0, dy0 = 0, -1
        elif start.direction == Direction.EAST:
            dx0, dy0 = 1, 0
        else:
            dx0, dy0 = -1, 0

        curve_moves = [MoveType.FORWARD_LEFT, MoveType.FORWARD_RIGHT,
                       MoveType.BACKWARD_LEFT, MoveType.BACKWARD_RIGHT]
        best_local_instr = None
        best_local_cost = float('inf')

        for move1 in curve_moves:
            cx1, cy1, dir1 = CurveParams.get(move1, start.direction)
            for move2 in curve_moves:
                cx2, cy2, dir2 = CurveParams.get(move2, dir1)
                if dir2 != target_dir:
                    continue
                if dir1 == Direction.NORTH:
                    dx1, dy1 = 0, 1
                elif dir1 == Direction.SOUTH:
                    dx1, dy1 = 0, -1
                elif dir1 == Direction.EAST:
                    dx1, dy1 = 1, 0
                else:
                    dx1, dy1 = -1, 0
                if target_dir == Direction.NORTH:
                    dxf, dyf = 0, 1
                elif target_dir == Direction.SOUTH:
                    dxf, dyf = 0, -1
                elif target_dir == Direction.EAST:
                    dxf, dyf = 1, 0
                else:
                    dxf, dyf = -1, 0
                cx = cx1 + cx2
                cy = cy1 + cy2
                Cx = target_pos.x - start.x - cx
                Cy = target_pos.y - start.y - cy

                # Loop over mid-straight multiples of 10
                for d2_candidate in range(-self.max_straight, self.max_straight + 1, 10):
                    # Check if the system is singular (start and final directions same vertical or horizontal)
                    if abs(dx0) < 1e-6 and abs(dxf) < 1e-6:
                        # Start and final both vertical (north/south)
                        if abs(dx1 * d2_candidate - Cx) > 1e-3:
                            continue
                        RHS = Cy - dy1 * d2_candidate
                        # Search over d1 multiples of 10
                        for d1_candidate in range(-self.max_straight, self.max_straight + 1, 10):
                            if abs(dyf) < 1e-6:
                                # dyf=0 (should not happen here because both vertical)
                                if abs(RHS - dy0 * d1_candidate) > 1e-3:
                                    continue
                                d3_candidate = 0
                            else:
                                d3 = (RHS - dy0 * d1_candidate) / dyf
                                d3_int = int(round(d3))
                                if abs(d3 - d3_int) > 1e-3:
                                    continue
                                d3_candidate = int(round(d3_int / 10.0)) * 10
                            # Compute final position with these rounded values
                            x_temp = start.x + d1_candidate * dx0 + cx1 + d2_candidate * dx1 + cx2 + d3_candidate * dxf
                            y_temp = start.y + d1_candidate * dy0 + cy1 + d2_candidate * dy1 + cy2 + d3_candidate * dyf
                            if abs(x_temp - target_pos.x) <= self.tolerance and abs(y_temp - target_pos.y) <= self.tolerance:
                                # Build instructions
                                instr = []
                                if d1_candidate > 0:
                                    instr.append({"amount": d1_candidate, "move": "FORWARD"})
                                elif d1_candidate < 0:
                                    instr.append({"amount": -d1_candidate, "move": "BACKWARD"})
                                instr.append(move1.value)
                                if d2_candidate > 0:
                                    instr.append({"amount": d2_candidate, "move": "FORWARD"})
                                elif d2_candidate < 0:
                                    instr.append({"amount": -d2_candidate, "move": "BACKWARD"})
                                instr.append(move2.value)
                                if d3_candidate > 0:
                                    instr.append({"amount": d3_candidate, "move": "FORWARD"})
                                elif d3_candidate < 0:
                                    instr.append({"amount": -d3_candidate, "move": "BACKWARD"})
                                cost = abs(d1_candidate) + abs(d2_candidate) + abs(d3_candidate) + 60
                                if cost < best_local_cost:
                                    best_local_cost = cost
                                    best_local_instr = instr
                    elif abs(dy0) < 1e-6 and abs(dyf) < 1e-6:
                        # Start and final both horizontal (east/west)
                        if abs(dy1 * d2_candidate - Cy) > 1e-3:
                            continue
                        RHS = Cx - dx1 * d2_candidate
                        for d1_candidate in range(-self.max_straight, self.max_straight + 1, 10):
                            if abs(dxf) < 1e-6:
                                if abs(RHS - dx0 * d1_candidate) > 1e-3:
                                    continue
                                d3_candidate = 0
                            else:
                                d3 = (RHS - dx0 * d1_candidate) / dxf
                                d3_int = int(round(d3))
                                if abs(d3 - d3_int) > 1e-3:
                                    continue
                                d3_candidate = int(round(d3_int / 10.0)) * 10
                            x_temp = start.x + d1_candidate * dx0 + cx1 + d2_candidate * dx1 + cx2 + d3_candidate * dxf
                            y_temp = start.y + d1_candidate * dy0 + cy1 + d2_candidate * dy1 + cy2 + d3_candidate * dyf
                            if abs(x_temp - target_pos.x) <= self.tolerance and abs(y_temp - target_pos.y) <= self.tolerance:
                                instr = []
                                if d1_candidate > 0:
                                    instr.append({"amount": d1_candidate, "move": "FORWARD"})
                                elif d1_candidate < 0:
                                    instr.append({"amount": -d1_candidate, "move": "BACKWARD"})
                                instr.append(move1.value)
                                if d2_candidate > 0:
                                    instr.append({"amount": d2_candidate, "move": "FORWARD"})
                                elif d2_candidate < 0:
                                    instr.append({"amount": -d2_candidate, "move": "BACKWARD"})
                                instr.append(move2.value)
                                if d3_candidate > 0:
                                    instr.append({"amount": d3_candidate, "move": "FORWARD"})
                                elif d3_candidate < 0:
                                    instr.append({"amount": -d3_candidate, "move": "BACKWARD"})
                                cost = abs(d1_candidate) + abs(d2_candidate) + abs(d3_candidate) + 60
                                if cost < best_local_cost:
                                    best_local_cost = cost
                                    best_local_instr = instr
                    else:
                        # Non-singular case
                        det = dx0 * dyf - dy0 * dxf
                        if abs(det) < 1e-6:
                            continue
                        RHSx = Cx - d2_candidate * dx1
                        RHSy = Cy - d2_candidate * dy1
                        d1 = (RHSx * dyf - RHSy * dxf) / det
                        d3 = (dx0 * RHSy - dy0 * RHSx) / det
                        d1_int = int(round(d1))
                        d3_int = int(round(d3))
                        if abs(d1 - d1_int) > 1e-3 or abs(d3 - d3_int) > 1e-3:
                            continue
                        def round10(v): return int(round(v / 10.0)) * 10
                        d1_10 = round10(d1_int)
                        d3_10 = round10(d3_int)
                        x_temp = start.x + d1_10 * dx0 + cx1 + d2_candidate * dx1 + cx2 + d3_10 * dxf
                        y_temp = start.y + d1_10 * dy0 + cy1 + d2_candidate * dy1 + cy2 + d3_10 * dyf
                        if abs(x_temp - target_pos.x) > self.tolerance or abs(y_temp - target_pos.y) > self.tolerance:
                            continue
                        instr = []
                        if d1_10 > 0:
                            instr.append({"amount": d1_10, "move": "FORWARD"})
                        elif d1_10 < 0:
                            instr.append({"amount": -d1_10, "move": "BACKWARD"})
                        instr.append(move1.value)
                        if d2_candidate > 0:
                            instr.append({"amount": d2_candidate, "move": "FORWARD"})
                        elif d2_candidate < 0:
                            instr.append({"amount": -d2_candidate, "move": "BACKWARD"})
                        instr.append(move2.value)
                        if d3_10 > 0:
                            instr.append({"amount": d3_10, "move": "FORWARD"})
                        elif d3_10 < 0:
                            instr.append({"amount": -d3_10, "move": "BACKWARD"})
                        cost = abs(d1_10) + abs(d2_candidate) + abs(d3_10) + 60
                        if cost < best_local_cost:
                            best_local_cost = cost
                            best_local_instr = instr
        return (best_local_instr, best_local_cost) if best_local_instr else None

    def is_path_clear(self, start: RobotState, instructions: List) -> bool:
        state = start.copy()
        for inst in instructions:
            if isinstance(inst, dict):
                move = inst["move"]
                amount = inst["amount"]
                step = 1
                steps = amount // step
                for i in range(1, steps + 1):
                    temp = state.copy()
                    if move == "FORWARD":
                        if temp.direction == Direction.NORTH:
                            temp.y += i * step
                        elif temp.direction == Direction.SOUTH:
                            temp.y -= i * step
                        elif temp.direction == Direction.EAST:
                            temp.x += i * step
                        else:
                            temp.x -= i * step
                    else:  # BACKWARD
                        if temp.direction == Direction.NORTH:
                            temp.y -= i * step
                        elif temp.direction == Direction.SOUTH:
                            temp.y += i * step
                        elif temp.direction == Direction.EAST:
                            temp.x -= i * step
                        else:
                            temp.x += i * step
                    if self.check_collision(temp):
                        return False
                if move == "FORWARD":
                    if state.direction == Direction.NORTH:
                        state.y += amount
                    elif state.direction == Direction.SOUTH:
                        state.y -= amount
                    elif state.direction == Direction.EAST:
                        state.x += amount
                    else:
                        state.x -= amount
                else:
                    if state.direction == Direction.NORTH:
                        state.y -= amount
                    elif state.direction == Direction.SOUTH:
                        state.y += amount
                    elif state.direction == Direction.EAST:
                        state.x -= amount
                    else:
                        state.x += amount
            else:
                move_type = MoveType(inst)
                dx, dy, new_dir = CurveParams.get(move_type, state.direction)
                fractions = [0.0, 0.25, 0.5, 0.75, 1.0]
                for f in fractions:
                    temp = state.copy()
                    temp.x += f * dx
                    temp.y += f * dy
                    if self.check_collision(temp):
                        return False
                state.x += dx
                state.y += dy
                state.direction = new_dir
        return True

    def check_collision(self, robot: RobotState) -> bool:
        if robot.x < 0 or robot.y < 0 or robot.x + robot.width > self.grid_size or robot.y + robot.height > self.grid_size:
            return True
        robot_box = robot.get_bounding_box()
        for obs in self.obstacles:
            if robot_box.intersects(obs.get_bounding_box()):
                return True
        return False

    def simulate(self, start: RobotState, instructions: List) -> RobotState:
        state = start.copy()
        for inst in instructions:
            if isinstance(inst, dict):
                move = inst["move"]
                amount = inst["amount"]
                if move == "FORWARD":
                    if state.direction == Direction.NORTH:
                        state.y += amount
                    elif state.direction == Direction.SOUTH:
                        state.y -= amount
                    elif state.direction == Direction.EAST:
                        state.x += amount
                    else:
                        state.x -= amount
                elif move == "BACKWARD":
                    if state.direction == Direction.NORTH:
                        state.y -= amount
                    elif state.direction == Direction.SOUTH:
                        state.y += amount
                    elif state.direction == Direction.EAST:
                        state.x -= amount
                    else:
                        state.x += amount
            else:
                move_type = MoveType(inst)
                dx, dy, new_dir = CurveParams.get(move_type, state.direction)
                state.x += dx
                state.y += dy
                state.direction = new_dir
        return state



# --------------------------------------------------
# API Endpoint
# --------------------------------------------------


@api.post("/")
def pathfinding():

    print("Request received:", datetime.now())

    input_data = request.get_json()

    robot = input_data.get("robot", {})
    obstacles = input_data.get("obstacles", [])

    print("robot:", robot)
    print("obstacles:", obstacles)

    sol = Solution()
    result = sol.solve_path(robot, obstacles)

    print("Path computed:", result)

    return jsonify(result), HTTPStatus.OK
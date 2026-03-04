from pathfinding.search.instructions import TurnInstruction
from pathfinding.world.primitives import Direction, Vector
from pathfinding.world.world import World

def turn(world: World, start: Vector, instruction: TurnInstruction) -> list[Vector] | None:
    """
    Performs a turn.

    :param world: The world.
    :param start: The initial vector.
    :param instruction: The turn instruction.
    :return: The path of the turn if it is legal, otherwise returns None.
    """

    # The turning radius (in grid cells). The turning radius is assumed to be 25cm.
    turning_radius = instruction.radius(world.cell_size)
    offset = 3 // world.cell_size

    curve: list[Vector] | None
    match (start.direction, instruction):
        # y facing north
        case (Direction.NORTH, TurnInstruction.FORWARD_LEFT):
            x = start.x
            y = start.y - world.robot.south_length + offset
            return __curve(
                world,
                turning_radius,
                Vector(
                    Direction.WEST,
                    x - turning_radius - world.robot.east_length + offset,
                    y + turning_radius,
                ),
                x - turning_radius,
                y,
                1,
            )

        case (Direction.NORTH, TurnInstruction.FORWARD_RIGHT):
            x = start.x
            y = start.y - world.robot.south_length + offset
            return __curve(
                world,
                turning_radius,
                Vector(
                    Direction.EAST,
                    x + turning_radius + world.robot.west_length - offset,
                    y + turning_radius,
                ),
                x + turning_radius,
                y,
                2,
            )

        case (Direction.NORTH, TurnInstruction.BACKWARD_LEFT):
            x = start.x
            y = start.y - world.robot.south_length + offset
            return __curve(
                world,
                turning_radius,
                Vector(
                    Direction.EAST,
                    x - turning_radius + world.robot.west_length - offset,
                    y - turning_radius,
                ),
                x - turning_radius,
                y,
                4,
            )

        case (Direction.NORTH, TurnInstruction.BACKWARD_RIGHT):
            x = start.x
            y = start.y - world.robot.south_length + offset
            return __curve(
                world,
                turning_radius,
                Vector(
                    Direction.WEST,
                    x + turning_radius - world.robot.west_length + offset,
                    y - turning_radius,
                ),
                x + turning_radius,
                y,
                3,
            )

        # y facing east
        case (Direction.EAST, TurnInstruction.FORWARD_LEFT):
            x = start.x - world.robot.west_length + offset
            y = start.y
            return __curve(
                world,
                turning_radius,
                Vector(
                    Direction.NORTH,
                    x + turning_radius,
                    y + turning_radius + world.robot.south_length - offset,
                ),
                x,
                y + turning_radius,
                4,
            )

        case (Direction.EAST, TurnInstruction.FORWARD_RIGHT):
            x = start.x - world.robot.west_length + offset
            y = start.y
            return __curve(
                world,
                turning_radius,
                Vector(
                    Direction.SOUTH,
                    x + turning_radius,
                    y - turning_radius - world.robot.north_length + offset,
                ),
                x,
                y - turning_radius,
                1,
            )

        case (Direction.EAST, TurnInstruction.BACKWARD_LEFT):
            x = start.x - world.robot.west_length + offset
            y = start.y
            return __curve(
                world,
                turning_radius,
                Vector(
                    Direction.SOUTH,
                    x - turning_radius,
                    y + turning_radius - world.robot.north_length + offset,
                ),
                x,
                y + turning_radius,
                3,
            )

        case (Direction.EAST, TurnInstruction.BACKWARD_RIGHT):
            x = start.x - world.robot.west_length + offset
            y = start.y
            return __curve(
                world,
                turning_radius,
                Vector(Direction.NORTH, x - turning_radius, y - turning_radius),
                x,
                y - turning_radius + world.robot.south_length - offset,
                2,
            )

        # y facing south
        case (Direction.SOUTH, TurnInstruction.FORWARD_LEFT):
            x = start.x
            y = start.y + world.robot.north_length - offset
            return __curve(
                world,
                turning_radius,
                Vector(
                    Direction.EAST,
                    x + turning_radius + world.robot.west_length - offset,
                    y - turning_radius,
                ),
                x + turning_radius,
                y,
                3,
            )

        case (Direction.SOUTH, TurnInstruction.FORWARD_RIGHT):
            x = start.x
            y = start.y + world.robot.north_length - offset
            return __curve(
                world,
                turning_radius,
                Vector(
                    Direction.WEST,
                    x - turning_radius - world.robot.east_length + offset,
                    y - turning_radius,
                ),
                x - turning_radius,
                y,
                4,
            )

        case (Direction.SOUTH, TurnInstruction.BACKWARD_LEFT):
            x = start.x
            y = start.y + world.robot.north_length - offset
            return __curve(
                world,
                turning_radius,
                Vector(
                    Direction.WEST,
                    x + turning_radius - world.robot.east_length + offset,
                    y + turning_radius,
                ),
                x + turning_radius,
                y,
                2,
            )

        case (Direction.SOUTH, TurnInstruction.BACKWARD_RIGHT):
            x = start.x
            y = start.y + world.robot.north_length - offset
            return __curve(
                world,
                turning_radius,
                Vector(
                    Direction.EAST,
                    x - turning_radius + world.robot.west_length - offset,
                    y + turning_radius,
                ),
                x - turning_radius,
                y,
                1,
            )

        # y facing west
        case (Direction.WEST, TurnInstruction.FORWARD_LEFT):
            x = start.x + world.robot.east_length - offset
            y = start.y
            return __curve(
                world,
                turning_radius,
                Vector(
                    Direction.SOUTH,
                    x - turning_radius,
                    y - turning_radius - world.robot.north_length + offset,
                ),
                x,
                y - turning_radius,
                2,
            )

        case (Direction.WEST, TurnInstruction.FORWARD_RIGHT):
            x = start.x + world.robot.east_length - offset
            y = start.y
            return __curve(
                world,
                turning_radius,
                Vector(
                    Direction.NORTH,
                    x - turning_radius,
                    y + turning_radius + world.robot.south_length - offset,
                ),
                x,
                y + turning_radius,
                3,
            )

        case (Direction.WEST, TurnInstruction.BACKWARD_LEFT):
            x = start.x + world.robot.east_length - offset
            y = start.y
            return __curve(
                world,
                turning_radius,
                Vector(
                    Direction.NORTH,
                    x + turning_radius,
                    y - turning_radius + world.robot.south_length - offset,
                ),
                x,
                y - turning_radius,
                1,
            )

        case (Direction.WEST, TurnInstruction.BACKWARD_RIGHT):
            x = start.x + world.robot.east_length - offset
            y = start.y
            return __curve(
                world,
                turning_radius,
                Vector(
                    Direction.SOUTH,
                    x + turning_radius,
                    y + turning_radius - world.robot.north_length + offset,
                ),
                x,
                y + turning_radius,
                4,
            )


def __curve(
    world: World,
    turning_radius: int,
    end: Vector,
    centre_x: int,
    centre_y: int,
    quadrant: int,
) -> list[Vector] | None:
    """
    Uses a modified Midpoint circle algorithm to determine a continuous, 
    chronological path for a robot turn.
    """
    assert 1 <= quadrant <= 4

    x = turning_radius
    y = 0
    err = 0

    # We use two lists to collect points from the two ends of the 90-degree arc
    # path_start builds from the axis towards the 45-degree diagonal
    # path_end builds from the other axis towards the 45-degree diagonal
    path_start = []
    path_end = []

    # Map the circle coordinates to the specific world quadrant
    match quadrant:
        case 1: # North to East (or East to North)
            a_map = lambda _x, _y: Vector(end.direction, centre_x + _x, centre_y + _y)
            b_map = lambda _x, _y: Vector(end.direction, centre_x + _y, centre_y + _x)
        case 2: # North to West
            a_map = lambda _x, _y: Vector(end.direction, centre_x - _y, centre_y + _x)
            b_map = lambda _x, _y: Vector(end.direction, centre_x - _x, centre_y + _y)
        case 3: # South to West
            a_map = lambda _x, _y: Vector(end.direction, centre_x - _x, centre_y - _y)
            b_map = lambda _x, _y: Vector(end.direction, centre_x - _y, centre_y - _x)
        case 4: # South to East
            a_map = lambda _x, _y: Vector(end.direction, centre_x + _y, centre_y - _x)
            b_map = lambda _x, _y: Vector(end.direction, centre_x + _x, centre_y - _y)

    while x >= y:
        a = a_map(x, y)
        b = b_map(x, y)

        # Immediate collision check: if any part of the arc is blocked, the turn is invalid
        if not world.contains(a) or not world.contains(b):
            return None

        path_start.append(a)
        path_end.append(b)

        y += 1
        err += 1 + 2 * y
        if 2 * (err - x) + 1 > 0:
            x -= 1
            err += 1 - 2 * x

    # IMPORTANT: The algorithm produces path_end in reverse order relative to the start.
    # We reverse path_end so that the points flow logically from Start -> Middle -> End.
    path_end.reverse()
    
    # Combine the two halves of the arc
    full_path = path_start + path_end
    
    # Ensure the final target vector is included and valid
    if world.contains(end):
        full_path.append(end)
    else:
        return None

    return full_path
    
'''
def turn(world: World, current: Vector, move: TurnInstruction) -> list[Vector] | None:
    """
    Generate turning arc that starts from current position.
    Returns ALL points along the arc.
    """
    is_forward = move in [TurnInstruction.FORWARD_LEFT, TurnInstruction.FORWARD_RIGHT]
    is_left = move in [TurnInstruction.FORWARD_LEFT, TurnInstruction.BACKWARD_LEFT]
    
    # Get final direction after turn
    new_direction = _get_new_direction(current.direction, is_left, is_forward)
    
    # For forward turns, we move while turning
    return _generate_turn_arc(current, new_direction, is_left, is_forward, world)

def _generate_turn_arc(
    start: Vector,
    end_dir: Direction,
    is_left: bool,
    is_forward: bool,
    world: World
) -> list[Vector] | None:

    template = _get_turn_template(start.direction, is_left, is_forward)

    x, y = start.x, start.y
    all_points: list[Vector] = []

    for dx, dy, progress in template:
        x += dx
        y += dy

        # Logic Fix: Only change direction at the end of the template (progress == 1)
        direction = start.direction if progress < 1 else end_dir
        vec = Vector(direction, x, y)

        if not world.contains(vec):
            return None # Collision check fails
            
        all_points.append(vec)

    # To avoid the "1 by 1" JSON output, the segment.py trace logic 
    # should only pick the LAST point of this arc.
    return all_points
    
# Helper functions
def _get_new_direction(current_dir: Direction, is_left: bool, is_forward: bool) -> Direction:
    """Calculate final direction after turn."""
    
    # Standard turn mapping (looking from above)
    if is_forward:
        if is_left:
            # Forward-left: turn 90° left while moving forward
            return {
                Direction.NORTH: Direction.WEST,
                Direction.EAST: Direction.NORTH,
                Direction.SOUTH: Direction.EAST,
                Direction.WEST: Direction.SOUTH
            }[current_dir]
        else:
            # Forward-right: turn 90° right while moving forward
            return {
                Direction.NORTH: Direction.EAST,
                Direction.EAST: Direction.SOUTH,
                Direction.SOUTH: Direction.WEST,
                Direction.WEST: Direction.NORTH
            }[current_dir]
    else:  # BACKWARD
        if is_left:
            # Backward-left: reverse + turn left
            # Equivalent to: move backward, turn vehicle left
            # Actually, when moving backward and turning left,
            # the rear becomes the front, so left becomes right!
            return {
                Direction.NORTH: Direction.EAST,   # Was WEST, now EAST
                Direction.EAST: Direction.SOUTH,   # Was NORTH, now SOUTH
                Direction.SOUTH: Direction.WEST,   # Was EAST, now WEST
                Direction.WEST: Direction.NORTH    # Was SOUTH, now NORTH
            }[current_dir]
        else:
            # Backward-right
            return {
                Direction.NORTH: Direction.WEST,   # Was EAST, now WEST
                Direction.EAST: Direction.NORTH,   # Was SOUTH, now NORTH
                Direction.SOUTH: Direction.EAST,   # Was WEST, now EAST
                Direction.WEST: Direction.SOUTH    # Was NORTH, now SOUTH
            }[current_dir]
  
'''
'''
def _get_turn_template(direction: Direction, is_left: bool, is_forward: bool):
    """
    Returns a list of (dx, dy, progress) tuples.
    If this function returns (0, 0), the robot will turn in place (1x1).
    """
    # 1. Define movement offsets based on direction
    if direction == Direction.NORTH:
        if is_forward:
            # Move North 3 cells, then move West/East while finishing turn
            return [(0, 1, 0), (0, 1, 0), (0, 1, 0), (-1, 1, 0.5), (-1, 0, 1)] if is_left else \
                   [(0, 1, 0), (0, 1, 0), (0, 1, 0), (1, 1, 0.5), (1, 0, 1)]
        else: # BACKWARD
            # Move South 3 cells, then swing tail West/East
            return [(0, -1, 0), (0, -1, 0), (0, -1, 0), (1, -1, 0.5), (1, 0, 1)] if is_left else \
                   [(0, -1, 0), (0, -1, 0), (0, -1, 0), (-1, -1, 0.5), (-1, 0, 1)]

    elif direction == Direction.EAST:
        if is_forward:
            return [(1, 0, 0), (1, 0, 0), (1, 0, 0), (1, 1, 0.5), (0, 1, 1)] if is_left else \
                   [(1, 0, 0), (1, 0, 0), (1, 0, 0), (1, -1, 0.5), (0, -1, 1)]
        else: # BACKWARD
            return [(-1, 0, 0), (-1, 0, 0), (-1, 0, 0), (-1, -1, 0.5), (0, -1, 1)] if is_left else \
                   [(-1, 0, 0), (-1, 0, 0), (-1, 0, 0), (-1, 1, 0.5), (0, 1, 1)]

    elif direction == Direction.SOUTH:
        if is_forward:
            return [(0, -1, 0), (0, -1, 0), (0, -1, 0), (1, -1, 0.5), (1, 0, 1)] if is_left else \
                   [(0, -1, 0), (0, -1, 0), (0, -1, 0), (-1, -1, 0.5), (-1, 0, 1)]
        else: # BACKWARD
            return [(0, 1, 0), (0, 1, 0), (0, 1, 0), (-1, 1, 0.5), (-1, 0, 1)] if is_left else \
                   [(0, 1, 0), (0, 1, 0), (0, 1, 0), (1, 1, 0.5), (1, 0, 1)]

    elif direction == Direction.WEST:
        if is_forward:
            return [(-1, 0, 0), (-1, 0, 0), (-1, 0, 0), (-1, -1, 0.5), (0, -1, 1)] if is_left else \
                   [(-1, 0, 0), (-1, 0, 0), (-1, 0, 0), (-1, 1, 0.5), (0, 1, 1)]
        else: # BACKWARD
            return [(1, 0, 0), (1, 0, 0), (1, 0, 0), (1, 1, 0.5), (0, 1, 1)] if is_left else \
                   [(1, 0, 0), (1, 0, 0), (1, 0, 0), (1, -1, 0.5), (0, -1, 1)]

    # NEVER return a default (0,0) template here. 
    # If the code reaches here, it should crash so you know which direction is missing.
    raise ValueError(f"No turn template defined for {direction} forward={is_forward}")
'''
'''
def _get_turn_template(direction: Direction, is_left: bool, is_forward: bool):
    """
    Returns (dx, dy, progress). 
    Displacements are increased to force a large arc and prevent 1x1 transitions.
    """
    # Define a 'Large Arc' constant - total displacement in both axes
    # Based on your 40cm radius / 5cm cell = 8 cells, 
    # we want the sum of dx/dy to be around 4-6 to feel like an arc.
    
    if direction == Direction.NORTH:
        if is_forward:
            # North -> West: Needs to move +y and -x
            return [(0, 1, 0), (0, 1, 0), (0, 1, 0), (-1, 1, 0.5), (-1, 1, 0.5), (-1, 0, 1), (-1, 0, 1)] if is_left else \
                   [(0, 1, 0), (0, 1, 0), (0, 1, 0), (1, 1, 0.5), (1, 1, 0.5), (1, 0, 1), (1, 0, 1)]
        else: # BACKWARD
            # North -> East/West: Needs to move -y and +/- x
            return [(0, -1, 0), (0, -1, 0), (0, -1, 0), (1, -1, 0.5), (1, -1, 0.5), (1, 0, 1), (1, 0, 1)] if is_left else \
                   [(0, -1, 0), (0, -1, 0), (0, -1, 0), (-1, -1, 0.5), (-1, -1, 0.5), (-1, 0, 1), (-1, 0, 1)]

    elif direction == Direction.EAST:
        if is_forward:
            return [(1, 0, 0), (1, 0, 0), (1, 0, 0), (1, 1, 0.5), (1, 1, 0.5), (0, 1, 1), (0, 1, 1)] if is_left else \
                   [(1, 0, 0), (1, 0, 0), (1, 0, 0), (1, -1, 0.5), (1, -1, 0.5), (0, -1, 1), (0, -1, 1)]
        else: # BACKWARD
            return [(-1, 0, 0), (-1, 0, 0), (-1, 0, 0), (-1, -1, 0.5), (-1, -1, 0.5), (0, -1, 1), (0, -1, 1)] if is_left else \
                   [(-1, 0, 0), (-1, 0, 0), (-1, 0, 0), (-1, 1, 0.5), (-1, 1, 0.5), (0, 1, 1), (0, 1, 1)]

    elif direction == Direction.SOUTH:
        if is_forward:
            return [(0, -1, 0), (0, -1, 0), (0, -1, 0), (1, -1, 0.5), (1, -1, 0.5), (1, 0, 1), (1, 0, 1)] if is_left else \
                   [(0, -1, 0), (0, -1, 0), (0, -1, 0), (-1, -1, 0.5), (-1, -1, 0.5), (-1, 0, 1), (-1, 0, 1)]
        else: # BACKWARD
            return [(0, 1, 0), (0, 1, 0), (0, 1, 0), (-1, 1, 0.5), (-1, 1, 0.5), (-1, 0, 1), (-1, 0, 1)] if is_left else \
                   [(0, 1, 0), (0, 1, 0), (0, 1, 0), (1, 1, 0.5), (1, 1, 0.5), (1, 0, 1), (1, 0, 1)]

    elif direction == Direction.WEST:
        if is_forward:
            return [(-1, 0, 0), (-1, 0, 0), (-1, 0, 0), (-1, -1, 0.5), (-1, -1, 0.5), (0, -1, 1), (0, -1, 1)] if is_left else \
                   [(-1, 0, 0), (-1, 0, 0), (-1, 0, 0), (-1, 1, 0.5), (-1, 1, 0.5), (0, 1, 1), (0, 1, 1)]
        else: # BACKWARD
            return [(1, 0, 0), (1, 0, 0), (1, 0, 0), (1, 1, 0.5), (1, 1, 0.5), (0, 1, 1), (0, 1, 1)] if is_left else \
                   [(1, 0, 0), (1, 0, 0), (1, 0, 0), (1, -1, 0.5), (1, -1, 0.5), (0, -1, 1), (0, -1, 1)]

    raise ValueError(f"No turn template defined for {direction} forward={is_forward}")

def __curve(
    world: World,
    turning_radius: int,
    end: Vector,
    centre_x: int,
    centre_y,
    quadrant: int,
) -> list[Vector] | None:
    assert 1 <= quadrant <= 4

    x = turning_radius
    y = 0
    err = 0

    # The original Midpoint circle algorithm fills in quadrants from two extremes. We store them in separate lists to
    # ensure an ordered list of vectors starting from the starting vector is returned.
    path = []
    a_map = None
    b_map = None

    match quadrant:
        case 1:
            a_map = lambda _x, _y: Vector(end.direction, centre_x + _x, centre_y + _y)
            b_map = lambda _x, _y: Vector(end.direction, centre_x + _y, centre_y + _x)
        case 2:
            a_map = lambda _x, _y: Vector(end.direction, centre_x - _y, centre_y + _x)
            b_map = lambda _x, _y: Vector(end.direction, centre_x - _x, centre_y + _y)
        case 3:
            a_map = lambda _x, _y: Vector(end.direction, centre_x - _x, centre_y - _y)
            b_map = lambda _x, _y: Vector(end.direction, centre_x - _y, centre_y - _x)
        case 4:
            a_map = lambda _x, _y: Vector(end.direction, centre_x + _y, centre_y - _x)
            b_map = lambda _x, _y: Vector(end.direction, centre_x + _x, centre_y - _y)

    while x >= y:
        a = a_map(x, y)
        if world.contains(a):
            path.append(a)
        else:
            return None

        b = b_map(x, y)
        if world.contains(b):
            path.append(b)
        else:
            return None

        y += 1
        err += 1 + 2 * y
        if 2 * (err - x) + 1 > 0:
            x -= 1
            err += 1 - 2 * x

    path.append(end)
    return path
'''
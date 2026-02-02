from pathfinding.search.instructions import TurnInstruction
from pathfinding.world.primitives import Direction, Vector
from pathfinding.world.world import World


def turn(world: World, current: Vector, move: TurnInstruction) -> list[Vector] | None:
    """
    Simplified turn for grid navigation.
    Generates points that approximate a turning arc within grid constraints.
    """
    
    is_forward = move in [TurnInstruction.FORWARD_LEFT, TurnInstruction.FORWARD_RIGHT]
    is_left = move in [TurnInstruction.FORWARD_LEFT, TurnInstruction.BACKWARD_LEFT]
    
    # Get turn template based on direction and turn type
    turn_template = _get_turn_template(current.direction, is_left, is_forward)
    
    # Apply template to current position
    arc_points = []
    for dx, dy, dir_change in turn_template:
        new_x = current.x + dx
        new_y = current.y + dy
        
        # Create intermediate direction (simplified)
        if dir_change == 0:
            new_dir = current.direction
        elif dir_change == 1:
            new_dir = _get_new_direction(current.direction, is_left, is_forward)
        else:
            # Gradual direction change (simplified)
            new_dir = current.direction if dx == 0 else _get_new_direction(current.direction, is_left, is_forward)
        
        point = Vector(new_dir, new_x, new_y)
        arc_points.append(point)
    
    # Validate
    for point in arc_points:
        if not world.contains(point):
            return None
    
    return arc_points

# Helper functions
def _get_new_direction(current_dir: Direction, is_left: bool, is_forward: bool) -> Direction:
    """Calculate final direction after turn."""
    if is_forward:
        if is_left:  # Forward-left
            directions = {
                Direction.NORTH: Direction.WEST,
                Direction.EAST: Direction.NORTH,
                Direction.SOUTH: Direction.EAST,
                Direction.WEST: Direction.SOUTH
            }
        else:  # Forward-right
            directions = {
                Direction.NORTH: Direction.EAST,
                Direction.EAST: Direction.SOUTH,
                Direction.SOUTH: Direction.WEST,
                Direction.WEST: Direction.NORTH
            }
    else:  # Backward
        if is_left:  # Backward-left
            directions = {
                Direction.NORTH: Direction.EAST,
                Direction.EAST: Direction.SOUTH,
                Direction.SOUTH: Direction.WEST,
                Direction.WEST: Direction.NORTH
            }
        else:  # Backward-right
            directions = {
                Direction.NORTH: Direction.WEST,
                Direction.EAST: Direction.NORTH,
                Direction.SOUTH: Direction.EAST,
                Direction.WEST: Direction.SOUTH
            }
    return directions[current_dir]
    
def _get_turn_template(direction: Direction, is_left: bool, is_forward: bool):
    """Pre-defined turn patterns for different directions."""
    
    # Template: list of (dx, dy, direction_progress)
    # direction_progress: 0=start dir, 1=end dir, 0.5=halfway
    
    if direction == Direction.NORTH:
        if is_forward and is_left:  # North → West (forward-left)
            return [
                (0, 1, 0),    # Move forward
                (-1, 1, 0.5), # Diag left-forward
                (-1, 0, 1),   # Move left
                (-1, 0, 1)    # Final position
            ]
        elif is_forward and not is_left:  # North → East (forward-right)
            return [
                (0, 1, 0),
                (1, 1, 0.5),
                (1, 0, 1),
                (1, 0, 1)
            ]
    elif direction == Direction.EAST:
        if is_forward and is_left:  # East → North
            return [
                (1, 0, 0),
                (1, 1, 0.5),
                (0, 1, 1),
                (0, 1, 1)
            ]
        elif is_forward and not is_left:  # East → South
            return [
                (1, 0, 0),
                (1, -1, 0.5),
                (0, -1, 1),
                (0, -1, 1)
            ]
    # Add templates for other directions...
    
    # Default: simple 2-point turn (not realistic but works)
    return [
        (0, 0, 0),  # Start
        (0, 0, 1)   # End (turned in place)
    ]

def __curve(
    world: World,
    turning_radius: int,
    end: Vector,
    centre_x: int,
    centre_y,
    quadrant: int,
) -> list[Vector] | None:
    """
    Uses a modified Midpoint circle algorithm to determine the curved path of a robot when turning.

    :param centre_x: The centre of the turning radius's x value.
    :param centre_y: The centre of the turning radius's y value.
    :param quadrant: The quadrant of the circle.
        Quadrants:
              2 | 1
            ----+----
              3 | 4
    :return: the vectors in the curve, may contain duplicates
    """
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

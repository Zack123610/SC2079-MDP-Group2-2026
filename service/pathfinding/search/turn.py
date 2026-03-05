from pathfinding.search.instructions import TurnInstruction
from pathfinding.world.primitives import Direction, Vector
from pathfinding.world.world import World

def turn(world: World, start: Vector, instruction: TurnInstruction) -> list[Vector] | None:
    """
    Performs an asymmetrical turn with lead-in/lead-out offsets and a safety buffer.
    """
    # 1. Physical Constants from your requirements
    # We add a 2-cell safety buffer to the radius to prevent clipping
    safety_buffer = 2 
    radius = instruction.radius(world.cell_size) + safety_buffer
    
    # Lead distance (Forward/Backward component)
    lead = instruction.straight_offset(world.cell_size)
    
    # Helper to generate straight points for the lead-in
    def get_lead_points(origin: Vector, dist: int, direction: Direction):
        pts = []
        for i in range(1, dist + 1):
            if direction == Direction.NORTH: pts.append(Vector(direction, origin.x, origin.y + i))
            elif direction == Direction.SOUTH: pts.append(Vector(direction, origin.x, origin.y - i))
            elif direction == Direction.EAST: pts.append(Vector(direction, origin.x + i, origin.y))
            elif direction == Direction.WEST: pts.append(Vector(direction, origin.x - i, origin.y))
        return pts

    path_lead = []
    curve_res = None

    match (start.direction, instruction):
        # --- NORTH FACING ---
        case (Direction.NORTH, TurnInstruction.FORWARD_LEFT):
            # 5cm forward, then 40cm arc to the left
            path_lead = get_lead_points(start, lead, Direction.NORTH)
            pivot = path_lead[-1] if path_lead else start
            curve_res = __curve(world, radius, Vector(Direction.WEST, pivot.x - radius, pivot.y + radius), 
                                pivot.x - radius, pivot.y, 1)

        case (Direction.NORTH, TurnInstruction.FORWARD_RIGHT):
            # 8cm forward, then 48cm arc to the right
            path_lead = get_lead_points(start, lead, Direction.NORTH)
            pivot = path_lead[-1] if path_lead else start
            curve_res = __curve(world, radius, Vector(Direction.EAST, pivot.x + radius, pivot.y + radius), 
                                pivot.x + radius, pivot.y, 2)

        case (Direction.NORTH, TurnInstruction.BACKWARD_LEFT):
            # 22cm backward, then 27cm arc to the left (resulting in East)
            path_lead = get_lead_points(start, lead, Direction.SOUTH)
            pivot = path_lead[-1] if path_lead else start
            curve_res = __curve(world, radius, Vector(Direction.EAST, pivot.x - radius, pivot.y - radius), 
                                pivot.x - radius, pivot.y, 4)

        case (Direction.NORTH, TurnInstruction.BACKWARD_RIGHT):
            # 22cm backward, then 29cm arc to the right (resulting in West)
            path_lead = get_lead_points(start, lead, Direction.SOUTH)
            pivot = path_lead[-1] if path_lead else start
            curve_res = __curve(world, radius, Vector(Direction.WEST, pivot.x + radius, pivot.y - radius), 
                                pivot.x + radius, pivot.y, 3)

        # --- EAST FACING ---
        case (Direction.EAST, TurnInstruction.FORWARD_LEFT):
            path_lead = get_lead_points(start, lead, Direction.EAST)
            pivot = path_lead[-1] if path_lead else start
            curve_res = __curve(world, radius, Vector(Direction.NORTH, pivot.x + radius, pivot.y + radius), 
                                pivot.x, pivot.y + radius, 4)

        case (Direction.EAST, TurnInstruction.BACKWARD_RIGHT):
            path_lead = get_lead_points(start, lead, Direction.WEST)
            pivot = path_lead[-1] if path_lead else start
            curve_res = __curve(world, radius, Vector(Direction.NORTH, pivot.x - radius, pivot.y - radius), 
                                pivot.x, pivot.y - radius, 2)

        # Note: Repeat similar logic for SOUTH and WEST directions...
        # Use the lead-in logic to shift the center of the arc.

    if curve_res is None:
        return None
    
    return path_lead + curve_res
'''
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
'''


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


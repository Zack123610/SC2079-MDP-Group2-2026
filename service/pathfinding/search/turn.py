from pathfinding.search.instructions import TurnInstruction
from pathfinding.world.primitives import Direction, Vector
from pathfinding.world.world import World

def turn(world: World, start: Vector, instruction: TurnInstruction) -> list[Vector] | None:
    """
    Performs an asymmetrical turn with lead-in/lead-out offsets and a safety buffer.
    """
    # 1. Physical Constants from your requirements
    # We add a 10cm safety buffer to the radius to prevent clipping
    safety_buffer = 10 // world.cell_size 
    radius = instruction.radius(world.cell_size) + safety_buffer
    
    # Lead distance (Forward/Backward component)
    lead = instruction.straight_offset(world.cell_size)
    
    print(f"DEBUG: Instruction {instruction.name}, Lead Cells: {lead}, Radius Cells: {radius}")
    
    # Helper to generate straight points for the lead-in
    def get_lead_points(origin: Vector, dist: int, direction: Direction):
        pts = []
        for i in range(1, dist + 1):
            dx, dy = {Direction.NORTH: (0, 1), Direction.SOUTH: (0, -1), 
                      Direction.EAST: (1, 0), Direction.WEST: (-1, 0)}[direction]
            pts.append(Vector(direction, origin.x + dx*i, origin.y + dy*i))
        return pts

    path_lead = []
    curve_res = None

    match (start.direction, instruction):
        # --- NORTH FACING ---
        case (Direction.NORTH, TurnInstruction.FORWARD_LEFT):
            path_lead = get_lead_points(start, lead, Direction.NORTH)
            pivot = path_lead[-1] if path_lead else start
            curve_res = __curve(world, radius, Vector(Direction.WEST, pivot.x - radius, pivot.y + radius), 
                                pivot.x - radius, pivot.y, quadrant="TOP_RIGHT")

        case (Direction.NORTH, TurnInstruction.FORWARD_RIGHT):
            path_lead = get_lead_points(start, lead, Direction.NORTH)
            pivot = path_lead[-1] if path_lead else start
            curve_res = __curve(world, radius, Vector(Direction.EAST, pivot.x + radius, pivot.y + radius), 
                                pivot.x + radius, pivot.y, quadrant="TOP_LEFT")

        case (Direction.NORTH, TurnInstruction.BACKWARD_LEFT):
            path_lead = get_lead_points(start, lead, Direction.SOUTH)
            pivot = path_lead[-1] if path_lead else start
            curve_res = __curve(world, radius, Vector(Direction.EAST, pivot.x - radius, pivot.y - radius), 
                                pivot.x - radius, pivot.y, quadrant="BOTTOM_RIGHT")

        case (Direction.NORTH, TurnInstruction.BACKWARD_RIGHT):
            path_lead = get_lead_points(start, lead, Direction.SOUTH)
            pivot = path_lead[-1] if path_lead else start
            curve_res = __curve(world, radius, Vector(Direction.WEST, pivot.x + radius, pivot.y - radius), 
                                pivot.x + radius, pivot.y, quadrant="BOTTOM_LEFT")

        # --- EAST FACING ---
        case (Direction.EAST, TurnInstruction.FORWARD_LEFT):
            path_lead = get_lead_points(start, lead, Direction.EAST)
            pivot = path_lead[-1] if path_lead else start
            curve_res = __curve(world, radius, Vector(Direction.NORTH, pivot.x + radius, pivot.y + radius), 
                                pivot.x, pivot.y + radius, quadrant="BOTTOM_LEFT")

        case (Direction.EAST, TurnInstruction.FORWARD_RIGHT):
            path_lead = get_lead_points(start, lead, Direction.EAST)
            pivot = path_lead[-1] if path_lead else start
            curve_res = __curve(world, radius, Vector(Direction.SOUTH, pivot.x + radius, pivot.y - radius), 
                                pivot.x, pivot.y - radius, quadrant="TOP_LEFT")

        case (Direction.EAST, TurnInstruction.BACKWARD_LEFT):
            path_lead = get_lead_points(start, lead, Direction.WEST)
            pivot = path_lead[-1] if path_lead else start
            curve_res = __curve(world, radius, Vector(Direction.SOUTH, pivot.x - radius, pivot.y - radius), 
                                pivot.x, pivot.y - radius, quadrant="TOP_RIGHT")

        case (Direction.EAST, TurnInstruction.BACKWARD_RIGHT):
            path_lead = get_lead_points(start, lead, Direction.WEST)
            pivot = path_lead[-1] if path_lead else start
            curve_res = __curve(world, radius, Vector(Direction.NORTH, pivot.x - radius, pivot.y + radius), 
                                pivot.x, pivot.y + radius, quadrant="BOTTOM_RIGHT")

        # --- SOUTH FACING ---
        case (Direction.SOUTH, TurnInstruction.FORWARD_LEFT):
            path_lead = get_lead_points(start, lead, Direction.SOUTH)
            pivot = path_lead[-1] if path_lead else start
            curve_res = __curve(world, radius, Vector(Direction.EAST, pivot.x + radius, pivot.y - radius),
                                pivot.x + radius, pivot.y, quadrant="BOTTOM_LEFT")

        case (Direction.SOUTH, TurnInstruction.FORWARD_RIGHT):
            # FIXED: Now uses BOTTOM_RIGHT to ensure the Y-coordinate decreases (Forward for South)
            path_lead = get_lead_points(start, lead, Direction.SOUTH)
            pivot = path_lead[-1] if path_lead else start
            curve_res = __curve(world, radius, Vector(Direction.WEST, pivot.x - radius, pivot.y - radius),
                                pivot.x - radius, pivot.y, quadrant="BOTTOM_RIGHT")

        case (Direction.SOUTH, TurnInstruction.BACKWARD_LEFT):
            path_lead = get_lead_points(start, lead, Direction.NORTH)
            pivot = path_lead[-1] if path_lead else start
            curve_res = __curve(world, radius, Vector(Direction.WEST, pivot.x - radius, pivot.y + radius),
                                pivot.x - radius, pivot.y, quadrant="TOP_RIGHT")

        case (Direction.SOUTH, TurnInstruction.BACKWARD_RIGHT):
            path_lead = get_lead_points(start, lead, Direction.NORTH)
            pivot = path_lead[-1] if path_lead else start
            curve_res = __curve(world, radius, Vector(Direction.EAST, pivot.x + radius, pivot.y + radius),
                                pivot.x + radius, pivot.y, quadrant="TOP_LEFT")

        # --- WEST FACING ---
        case (Direction.WEST, TurnInstruction.FORWARD_LEFT):
            path_lead = get_lead_points(start, lead, Direction.WEST)
            pivot = path_lead[-1] if path_lead else start
            curve_res = __curve(world, radius, Vector(Direction.SOUTH, pivot.x - radius, pivot.y - radius),
                                pivot.x, pivot.y - radius, quadrant="TOP_RIGHT")

        case (Direction.WEST, TurnInstruction.FORWARD_RIGHT):
            path_lead = get_lead_points(start, lead, Direction.WEST)
            pivot = path_lead[-1] if path_lead else start
            curve_res = __curve(world, radius, Vector(Direction.NORTH, pivot.x - radius, pivot.y + radius),
                                pivot.x, pivot.y + radius, quadrant="BOTTOM_RIGHT")

        case (Direction.WEST, TurnInstruction.BACKWARD_LEFT):
            path_lead = get_lead_points(start, lead, Direction.EAST)
            pivot = path_lead[-1] if path_lead else start
            curve_res = __curve(world, radius, Vector(Direction.NORTH, pivot.x + radius, pivot.y + radius),
                                pivot.x, pivot.y + radius, quadrant="BOTTOM_LEFT")

        case (Direction.WEST, TurnInstruction.BACKWARD_RIGHT):
            path_lead = get_lead_points(start, lead, Direction.EAST)
            pivot = path_lead[-1] if path_lead else start
            curve_res = __curve(world, radius, Vector(Direction.SOUTH, pivot.x + radius, pivot.y - radius),
                                pivot.x, pivot.y - radius, quadrant="TOP_LEFT")

                                
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

def __curve(world, radius, end, centre_x, centre_y, quadrant) -> list[Vector] | None:
    path_start = []
    path_end = []
    
    # 1. Initialize Midpoint Variables
    x = radius
    y = 0
    err = 1 - x
    
    # Direction is determined by the target 'end' vector's direction
    target_dir = end.direction

    while x >= y:
        # 2. Map coordinates based on Quadrant
        # We need to map (x,y) from the unit circle to our world coordinates
        coords = []
        if quadrant == "TOP_LEFT":    # Start (-R, 0) -> End (0, R)
            coords = [(centre_x - x, centre_y + y), (centre_x - y, centre_y + x)]
        elif quadrant == "TOP_RIGHT":  # Start (R, 0) -> End (0, R)
            coords = [(centre_x + x, centre_y + y), (centre_x + y, centre_y + x)]
        elif quadrant == "BOTTOM_LEFT": # Start (-R, 0) -> End (0, -R)
            coords = [(centre_x - x, centre_y - y), (centre_x - y, centre_y - x)]
        elif quadrant == "BOTTOM_RIGHT": # Start (R, 0) -> End (0, -R)
            coords = [(centre_x + x, centre_y - y), (centre_x + y, centre_y - x)]

        # 3. Create Vectors and Check Safety
        # a is the point closer to the axis, b is closer to the 45-degree diagonal
        v_a = Vector(target_dir, coords[0][0], coords[0][1])
        v_b = Vector(target_dir, coords[1][0], coords[1][1])

        # We must check if the robot's body fits at these points
        if not world.is_safe(v_a) or not world.is_safe(v_b):
            return None

        path_start.append(v_a)
        path_end.append(v_b)

        # 4. Standard Midpoint Error Update
        y += 1
        if err < 0:
            err += 2 * y + 1
        else:
            x -= 1
            err += 2 * (y - x) + 1

    # 5. Reverse path_end to maintain sequence and join
    path_end.reverse()
    full_path = path_start + path_end
    
    return full_path


    """
    old code
    match (start.direction, instruction):
        # --- NORTH FACING ---
        case (Direction.NORTH, TurnInstruction.FORWARD_LEFT):
            # 5cm forward, then 40cm arc to the left
            path_lead = get_lead_points(start, lead, Direction.NORTH)
            pivot = path_lead[-1] if path_lead else start
            curve_res = __curve(world, radius, Vector(Direction.WEST, pivot.x - radius, pivot.y + radius), 
                                pivot.x - radius, pivot.y, quadrant="TOP_RIGHT")

        case (Direction.NORTH, TurnInstruction.FORWARD_RIGHT):
            # 8cm forward, then 48cm arc to the right
            path_lead = get_lead_points(start, lead, Direction.NORTH)
            pivot = path_lead[-1] if path_lead else start
            curve_res = __curve(world, radius, Vector(Direction.EAST, pivot.x + radius, pivot.y + radius), 
                                pivot.x + radius, pivot.y, quadrant="TOP_LEFT")

        case (Direction.NORTH, TurnInstruction.BACKWARD_LEFT):
            # 22cm backward, then 27cm arc to the left (resulting in East)
            path_lead = get_lead_points(start, lead, Direction.SOUTH)
            pivot = path_lead[-1] if path_lead else start
            curve_res = __curve(world, radius, Vector(Direction.EAST, pivot.x - radius, pivot.y - radius), 
                                pivot.x - radius, pivot.y, quadrant="BOTTOM_RIGHT")

        case (Direction.NORTH, TurnInstruction.BACKWARD_RIGHT):
            # 22cm backward, then 29cm arc to the right (resulting in West)
            path_lead = get_lead_points(start, lead, Direction.SOUTH)
            pivot = path_lead[-1] if path_lead else start
            curve_res = __curve(world, radius, Vector(Direction.WEST, pivot.x + radius, pivot.y - radius), 
                                pivot.x + radius, pivot.y, quadrant="BOTTOM_LEFT")

        # --- EAST FACING ---
        case (Direction.EAST, TurnInstruction.FORWARD_LEFT):
            path_lead = get_lead_points(start, lead, Direction.EAST)
            pivot = path_lead[-1] if path_lead else start
            curve_res = __curve(world, radius, 
                                Vector(Direction.NORTH, pivot.x + radius, pivot.y + radius), 
                                pivot.x, pivot.y + radius, quadrant="BOTTOM_LEFT")

        case (Direction.EAST, TurnInstruction.FORWARD_RIGHT):
            path_lead = get_lead_points(start, lead, Direction.EAST)
            pivot = path_lead[-1] if path_lead else start
            curve_res = __curve(world, radius, 
                                Vector(Direction.SOUTH, pivot.x + radius, pivot.y - radius), 
                                pivot.x, pivot.y - radius, quadrant="TOP_LEFT")

        case (Direction.EAST, TurnInstruction.BACKWARD_LEFT):
            path_lead = get_lead_points(start, lead, Direction.WEST)
            pivot = path_lead[-1] if path_lead else start
            curve_res = __curve(world, radius, 
                                Vector(Direction.SOUTH, pivot.x - radius, pivot.y - radius), 
                                pivot.x, pivot.y - radius, quadrant="TOP_RIGHT")

        case (Direction.EAST, TurnInstruction.BACKWARD_RIGHT):
            path_lead = get_lead_points(start, lead, Direction.WEST)
            pivot = path_lead[-1] if path_lead else start
            curve_res = __curve(world, radius, 
                                Vector(Direction.NORTH, pivot.x - radius, pivot.y + radius), 
                                pivot.x, pivot.y + radius, quadrant="BOTTOM_RIGHT")
        
                # --- SOUTH FACING ---
        case (Direction.SOUTH, TurnInstruction.FORWARD_LEFT):
            path_lead = get_lead_points(start, lead, Direction.SOUTH)
            pivot = path_lead[-1] if path_lead else start
            curve_res = __curve(world, radius,
                                Vector(Direction.EAST, pivot.x + radius, pivot.y - radius),
                                pivot.x + radius, pivot.y, quadrant="TOP_LEFT")

        case (Direction.SOUTH, TurnInstruction.FORWARD_RIGHT):
            path_lead = get_lead_points(start, lead, Direction.SOUTH)
            pivot = path_lead[-1] if path_lead else start
            curve_res = __curve(world, radius,
                                Vector(Direction.WEST, pivot.x - radius, pivot.y - radius),
                                pivot.x - radius, pivot.y, quadrant="TOP_RIGHT")

        case (Direction.SOUTH, TurnInstruction.BACKWARD_LEFT):
            path_lead = get_lead_points(start, lead, Direction.NORTH)
            pivot = path_lead[-1] if path_lead else start
            curve_res = __curve(world, radius,
                                Vector(Direction.WEST, pivot.x - radius, pivot.y + radius),
                                pivot.x - radius, pivot.y, quadrant="BOTTOM_RIGHT")

        case (Direction.SOUTH, TurnInstruction.BACKWARD_RIGHT):
            path_lead = get_lead_points(start, lead, Direction.NORTH)
            pivot = path_lead[-1] if path_lead else start
            curve_res = __curve(world, radius,
                                Vector(Direction.EAST, pivot.x + radius, pivot.y + radius),
                                pivot.x + radius, pivot.y, quadrant="BOTTOM_LEFT")

        # --- WEST FACING ---
        case (Direction.WEST, TurnInstruction.FORWARD_LEFT):
            path_lead = get_lead_points(start, lead, Direction.WEST)
            pivot = path_lead[-1] if path_lead else start
            curve_res = __curve(world, radius,
                                Vector(Direction.SOUTH, pivot.x - radius, pivot.y - radius),
                                pivot.x, pivot.y - radius, quadrant="TOP_RIGHT")

        case (Direction.WEST, TurnInstruction.FORWARD_RIGHT):
            path_lead = get_lead_points(start, lead, Direction.WEST)
            pivot = path_lead[-1] if path_lead else start
            curve_res = __curve(world, radius,
                                Vector(Direction.NORTH, pivot.x - radius, pivot.y + radius),
                                pivot.x, pivot.y + radius, quadrant="BOTTOM_RIGHT")

        case (Direction.WEST, TurnInstruction.BACKWARD_LEFT):
            path_lead = get_lead_points(start, lead, Direction.EAST)
            pivot = path_lead[-1] if path_lead else start
            curve_res = __curve(world, radius,
                                Vector(Direction.NORTH, pivot.x + radius, pivot.y + radius),
                                pivot.x, pivot.y + radius, quadrant="BOTTOM_LEFT")

        case (Direction.WEST, TurnInstruction.BACKWARD_RIGHT):
            path_lead = get_lead_points(start, lead, Direction.EAST)
            pivot = path_lead[-1] if path_lead else start
            curve_res = __curve(world, radius,
                                Vector(Direction.SOUTH, pivot.x + radius, pivot.y - radius),
                                pivot.x, pivot.y - radius, quadrant="TOP_LEFT")

"""
from pathfinding.world.primitives import Vector, Direction


# In straight() function, ensure it doesn't go beyond bounds
def straight(current: Vector, modifier: int, length: int) -> list[Vector]:
    """Generate straight path of given length"""
    path = []
    for i in range(1, length + 1):
        distance = i * modifier
        if current.direction == Direction.NORTH:
            new_point = Vector(current.direction, current.x, current.y + distance)
        elif current.direction == Direction.SOUTH:
            new_point = Vector(current.direction, current.x, current.y - distance)
        elif current.direction == Direction.EAST:
            new_point = Vector(current.direction, current.x + distance, current.y)
        elif current.direction == Direction.WEST:
            new_point = Vector(current.direction, current.x - distance, current.y)
        
        # STOP if this point would be invalid
        # Don't add it to path
        path.append(new_point)
    return path

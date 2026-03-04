from __future__ import annotations

from dataclasses import dataclass

from pathfinding.search.instructions import Turn, TurnInstruction, Move, MoveInstruction, MiscInstruction
from pathfinding.search.segment import segment
from pathfinding.world.primitives import Vector
from pathfinding.world.world import World, Obstacle

def search(world: World, objectives: dict[Obstacle, tuple[Vector, set[Vector]]]) -> list[Segment]:
    segments = []
    current = world.robot.vector
    for _ in world.obstacles:
        seg = segment(world, current, objectives)
        if seg is None:
            for objective in objectives.keys():
                print(f'WARNING: Unable to find path to {objective}. Skipping.')
            return segments

        obstacle, _, path = seg
        segments.append(Segment.compress(world, seg))
        current, _ = path[-1]
        objectives.pop(obstacle)

    return segments

""" 
import math

def search(world: World, objectives: dict[Obstacle, tuple[Vector, set[Vector]]]) -> list[Segment]:
    if not objectives:
        return []
    
    segments = []
    current = world.robot.vector
    
    # Find optimal visiting order
    ordered_obstacles = find_optimal_order(world, objectives)
    
    for obstacle in ordered_obstacles:
        # Get objectives for just this obstacle
        single_objective = {obstacle: objectives[obstacle]}
        
        # Find path to this obstacle
        result = segment(world, current, single_objective)
        
        if result is None:
            print(f"WARNING: Cannot find path to obstacle {obstacle.image_id}. Skipping.")
            continue
        
        obstacle_found, cost, path = result
        
        # Compress and add segment
        seg = Segment.compress(world, (obstacle_found, cost, path))
        segments.append(seg)
        
        # Update current position to end of path
        if path:
            current, _ = path[-1]
        
        # Remove from objectives (optional)
        objectives.pop(obstacle, None)
    
    return segments
"""
def find_optimal_order(world: World, objectives: dict[Obstacle, tuple[Vector, set[Vector]]]) -> list[Obstacle]:
    # Find optimal order to visit all obstacles using greedy TSP approach.
    
    if not objectives:
        return []
    
    # Start from robot position
    start_pos = world.robot.vector
    remaining = list(objectives.keys())
    ordered = []
    current_pos = start_pos
    
    while remaining:
        # Find nearest obstacle from current position
        nearest = None
        min_distance = float('inf')
        
        for obstacle in remaining:
            # Get the best objective for this obstacle
            best_objective, _ = objectives[obstacle]
            
            # Estimate distance (Euclidean as heuristic)
            distance = math.sqrt(
                (best_objective.x - current_pos.x) ** 2 + 
                (best_objective.y - current_pos.y) ** 2
            )
            
            if distance < min_distance:
                min_distance = distance
                nearest = obstacle
        
        if nearest:
            ordered.append(nearest)
            # Update current position to this obstacle's objective
            best_objective, _ = objectives[nearest]
            current_pos = best_objective
            remaining.remove(nearest)
    
    print(f"Optimal order: {[obs.image_id for obs in ordered]}")
    return ordered

@dataclass
class Segment:
    image_id: int
    cost: int
    instructions: list[TurnInstruction | MoveInstruction | MiscInstruction]
    vectors: list[Vector]

    @classmethod
    def compress(cls, world: World, information: tuple[Obstacle, int, list[tuple[Vector, Turn | Move | None]]]) -> Segment:
        obstacle, cost, parts = information
        instructions: list[TurnInstruction | MoveInstruction | MiscInstruction] = []
        vectors: list[Vector] = []

        for vector, move in parts:
            match move:
                case Turn():
                    instructions.append(move.turn)
                    vectors.extend(move.vectors)

                case Move() if instructions and isinstance(instructions[-1], MoveInstruction) and instructions[-1].move == move.move:
                    instructions[-1].amount += len(move.vectors) * world.cell_size
                    vectors.extend(move.vectors)

                case Move():
                    instructions.append(MoveInstruction(move=move.move, amount=len(move.vectors) * world.cell_size))
                    vectors.extend(move.vectors)

        instructions.append(MiscInstruction.CAPTURE_IMAGE)

        return cls(obstacle.image_id, cost, instructions, vectors)

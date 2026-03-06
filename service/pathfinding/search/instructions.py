from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from math import pi

from pydantic import BaseModel, Field

from pathfinding.world.primitives import Vector


class MiscInstruction(str, Enum):
    CAPTURE_IMAGE = 'CAPTURE_IMAGE'


class MoveInstruction(BaseModel):
    move: Straight
    amount: int = Field(
        ge=1, description="The amount to move the robot in centimetres."
    )


@dataclass
class Move:
    move: Straight
    vectors: list[Vector]


class Straight(str, Enum):
    FORWARD = 'FORWARD'
    BACKWARD = 'BACKWARD'


class TurnInstruction(str, Enum):
    FORWARD_LEFT = 'FORWARD_LEFT'
    FORWARD_RIGHT = 'FORWARD_RIGHT'
    BACKWARD_LEFT = 'BACKWARD_LEFT'
    BACKWARD_RIGHT = 'BACKWARD_RIGHT'

    def radius(self, cell_size: int) -> int:
        """
        The turning radius (in grid cells). The turning radius is different for each direction.

        :param cell_size: the cell size
        :return: the turning radius in grid cells.
        """
        match self:
            case TurnInstruction.FORWARD_LEFT:
                return round(40 / cell_size)
            case TurnInstruction.FORWARD_RIGHT:
                return round(48 / cell_size)
            case TurnInstruction.BACKWARD_LEFT:
                return round(27 / cell_size)
            case TurnInstruction.BACKWARD_RIGHT:
                return round(29 / cell_size)

    def straight_offset(self, cell_size: int) -> int:
        """The 'forward/backward' component of the asymmetrical turn."""
        match self:
            case TurnInstruction.FORWARD_LEFT: return round(5 / cell_size)
            case TurnInstruction.FORWARD_RIGHT: return round(8 / cell_size)
            case TurnInstruction.BACKWARD_LEFT: return round(22 / cell_size)
            case TurnInstruction.BACKWARD_RIGHT: return round(22 / cell_size)

    def arc_length(self, cell_size: int) -> int:
        return round(self.radius(cell_size) * (pi / 2))


@dataclass
class Turn:
    turn: TurnInstruction
    vectors: list[Vector]

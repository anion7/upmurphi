from __future__ import annotations

"""Reference planning problem used by the Monte Carlo planner demo."""

from dataclasses import dataclass
from typing import Iterable, List, Sequence, Set, Tuple

from planner import PlanningProblem

Coord = Tuple[int, int]
Action = str


@dataclass(frozen=True)
class Move:
    name: Action
    delta: Coord


MOVES: Sequence[Move] = (
    Move("UP", (0, -1)),
    Move("DOWN", (0, 1)),
    Move("LEFT", (-1, 0)),
    Move("RIGHT", (1, 0)),
)


class GridWorldProblem(PlanningProblem[Coord, Action]):
    """Simple deterministic path planning domain on a 2D grid."""

    def __init__(
        self,
        width: int,
        height: int,
        start: Coord,
        goal: Coord,
        obstacles: Iterable[Coord] | None = None,
    ) -> None:
        self.width = width
        self.height = height
        self._start = start
        self._goal = goal
        self._obstacles: Set[Coord] = set(obstacles or [])

        if not self._in_bounds(start) or not self._in_bounds(goal):
            raise ValueError("Start/goal coordinates must be inside the grid.")

    # ------------------------------------------------------------------
    # PlanningProblem interface
    # ------------------------------------------------------------------
    def initial_state(self) -> Coord:
        return self._start

    def is_goal(self, state: Coord) -> bool:
        return state == self._goal

    def actions(self, state: Coord) -> Iterable[Action]:
        for move in MOVES:
            nx = state[0] + move.delta[0]
            ny = state[1] + move.delta[1]
            if self._is_free((nx, ny)):
                yield move.name

    def transition(self, state: Coord, action: Action) -> Coord:
        move = next((m for m in MOVES if m.name == action), None)
        if move is None:
            raise ValueError(f"Unknown action: {action}")
        next_state = (state[0] + move.delta[0], state[1] + move.delta[1])
        if not self._is_free(next_state):
            raise ValueError("Transition requested into a blocked cell.")
        return next_state

    def heuristic(self, state: Coord) -> float:
        # Manhattan distance acts as an admissible heuristic for grid motion.
        return abs(state[0] - self._goal[0]) + abs(state[1] - self._goal[1])

    # Uniform action costs keep the example compact.

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    def _is_free(self, coord: Coord) -> bool:
        return self._in_bounds(coord) and coord not in self._obstacles

    def _in_bounds(self, coord: Coord) -> bool:
        x, y = coord
        return 0 <= x < self.width and 0 <= y < self.height

    def render(self, path: Sequence[Coord] | None = None) -> str:
        """Return an ASCII representation of the current map."""

        path_set = set(path or [])
        rows: List[str] = []
        for y in range(self.height):
            row: List[str] = []
            for x in range(self.width):
                coord = (x, y)
                if coord == self._start:
                    row.append("S")
                elif coord == self._goal:
                    row.append("G")
                elif coord in path_set:
                    row.append("*")
                elif coord in self._obstacles:
                    row.append("#")
                else:
                    row.append(".")
            rows.append("".join(row))
        return "\n".join(rows)


def demo_problem() -> GridWorldProblem:
    """Return a small canned planning instance."""

    obstacles = {
        (2, 0),
        (2, 1),
        (2, 2),
        (2, 3),
        (2, 4),
        (4, 2),
        (4, 3),
    }
    return GridWorldProblem(
        width=6,
        height=6,
        start=(0, 0),
        goal=(5, 5),
        obstacles=obstacles,
    )


__all__ = ["GridWorldProblem", "demo_problem"]

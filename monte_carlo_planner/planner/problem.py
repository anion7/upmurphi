from __future__ import annotations

"""Core abstractions for Monte Carlo based planning."""

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Generic, Iterable, List, Sequence, Tuple, TypeVar

State = TypeVar("State")
Action = TypeVar("Action")


class PlanningProblem(ABC, Generic[State, Action]):
    """Abstract base class that describes deterministic planning problems.

    Subclasses must describe a search space by providing an initial state,
    action enumeration, state transition model, and goal predicate.
    Optional cost and heuristic hooks allow the planner to bias its search.
    """

    @abstractmethod
    def initial_state(self) -> State:
        """Return the initial state of the planning problem."""

    @abstractmethod
    def is_goal(self, state: State) -> bool:
        """Return True when the provided state satisfies the goal criteria."""

    @abstractmethod
    def actions(self, state: State) -> Iterable[Action]:
        """Return the set of feasible actions for the given state."""

    @abstractmethod
    def transition(self, state: State, action: Action) -> State:
        """Return the successor state obtained by applying ``action``."""

    def action_cost(self, state: State, action: Action, next_state: State) -> float:
        """Return the edge cost for applying ``action``.

        By default every action has unit cost. Override this method when your
        domain needs non-uniform costs.
        """

        return 1.0

    def heuristic(self, state: State) -> float:
        """Estimate the remaining cost-to-go from ``state``.

        The Monte Carlo planner only uses this value during rollouts; returning
        zero keeps the algorithm unbiased.
        """

        return 0.0


@dataclass
class PlanResult(Generic[State, Action]):
    """Represents the output of the Monte Carlo planner."""

    actions: Sequence[Action]
    cost: float
    solved: bool
    iterations: int
    nodes_expanded: int
    message: str = ""

    def pretty_plan(self) -> str:
        """Return a human readable representation of the resulting plan."""

        actions_repr = " -> ".join(map(str, self.actions)) if self.actions else "<empty>"
        status = "SOLVED" if self.solved else "APPROX"
        return f"[{status}] cost={self.cost:.3f} steps={len(self.actions)}\n{actions_repr}"


__all__ = ["PlanningProblem", "PlanResult", "State", "Action"]

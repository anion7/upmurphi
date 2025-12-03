from __future__ import annotations

"""Monte-Carlo Tree Search based planner for generic deterministic domains."""

from dataclasses import dataclass, field
import math
import time
import random
from typing import Dict, Generic, List, Optional, Sequence, Tuple, TypeVar

from .problem import Action, PlanResult, PlanningProblem, State


@dataclass
class MonteCarloConfig:
    """Configuration switches used by :class:`MonteCarloPlanner`."""

    max_iterations: int = 3000
    exploration_constant: float = 1.4
    rollout_depth: int = 40
    discount_factor: float = 0.99
    goal_reward: float = 100.0
    epsilon_greedy: float = 0.2
    time_limit: Optional[float] = None  # seconds
    random_seed: Optional[int] = None


@dataclass
class _TreeNode(Generic[State, Action]):
    """Single node in the Monte Carlo search tree."""

    state: State
    parent: Optional["_TreeNode[State, Action]"]
    action: Optional[Action]
    path_cost: float = 0.0
    children: Dict[Action, "_TreeNode[State, Action]"] = field(default_factory=dict)
    visits: int = 0
    total_reward: float = 0.0
    untried_actions: Optional[List[Action]] = None

    def is_fully_expanded(self, problem: PlanningProblem[State, Action]) -> bool:
        if self.untried_actions is None:
            self.untried_actions = list(problem.actions(self.state))
        return len(self.untried_actions) == 0

    def add_child(
        self,
        action: Action,
        next_state: State,
        step_cost: float,
    ) -> "_TreeNode[State, Action]":
        child = _TreeNode(
            state=next_state,
            parent=self,
            action=action,
            path_cost=self.path_cost + step_cost,
        )
        self.children[action] = child
        return child


class MonteCarloPlanner(Generic[State, Action]):
    """Generic Monte Carlo Tree Search planner."""

    def __init__(self, config: Optional[MonteCarloConfig] = None) -> None:
        self.config = config or MonteCarloConfig()
        self._rng = random.Random(self.config.random_seed)

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------
    def plan(
        self, problem: PlanningProblem[State, Action]
    ) -> PlanResult[State, Action]:
        """Run MCTS and return the best plan found."""

        root_state = problem.initial_state()
        if problem.is_goal(root_state):
            return PlanResult(
                actions=[],
                cost=0.0,
                solved=True,
                iterations=0,
                nodes_expanded=0,
                message="Initial state already satisfies the goal.",
            )

        root = _TreeNode(state=root_state, parent=None, action=None, path_cost=0.0)
        root.untried_actions = list(problem.actions(root_state))

        best_goal: Optional[_TreeNode[State, Action]] = None
        best_goal_cost = float("inf")
        iterations = 0
        nodes_expanded = 0
        start_time = time.time()

        while iterations < self.config.max_iterations:
            if self.config.time_limit is not None:
                elapsed = time.time() - start_time
                if elapsed >= self.config.time_limit:
                    break

            node = self._select(root, problem)

            if not problem.is_goal(node.state):
                expanded, _ = self._expand(node, problem)
                if expanded is not node:
                    node = expanded
                    nodes_expanded += 1

            reward, _ = self._simulate(node.state, problem)
            self._backpropagate(node, reward)

            if problem.is_goal(node.state) and node.path_cost < best_goal_cost:
                best_goal = node
                best_goal_cost = node.path_cost

            iterations += 1

        if best_goal is not None:
            actions = self._extract_actions(best_goal)
            result = PlanResult(
                actions=actions,
                cost=best_goal.path_cost,
                solved=True,
                iterations=iterations,
                nodes_expanded=nodes_expanded,
                message="Goal reached during search.",
            )
        else:
            actions, approx_cost = self._best_effort_trace(root)
            result = PlanResult(
                actions=actions,
                cost=approx_cost,
                solved=False,
                iterations=iterations,
                nodes_expanded=nodes_expanded,
                message="Goal not proven; returning the most visited branch.",
            )

        return result

    # ------------------------------------------------------------------
    # Core MCTS phases
    # ------------------------------------------------------------------
    def _select(
        self, node: _TreeNode[State, Action], problem: PlanningProblem[State, Action]
    ) -> _TreeNode[State, Action]:
        while True:
            if problem.is_goal(node.state):
                return node

            if node.untried_actions is None:
                node.untried_actions = list(problem.actions(node.state))

            if node.untried_actions:
                return node

            if not node.children:
                return node

            node = self._best_child(node)

    def _expand(
        self, node: _TreeNode[State, Action], problem: PlanningProblem[State, Action]
    ) -> Tuple[_TreeNode[State, Action], float]:
        if node.untried_actions is None:
            node.untried_actions = list(problem.actions(node.state))

        if not node.untried_actions:
            return node, 0.0

        action = self._rng.choice(node.untried_actions)
        node.untried_actions.remove(action)

        next_state = problem.transition(node.state, action)
        step_cost = problem.action_cost(node.state, action, next_state)
        child = node.add_child(action, next_state, step_cost)
        return child, step_cost

    def _simulate(
        self, state: State, problem: PlanningProblem[State, Action]
    ) -> Tuple[float, State]:
        total_reward = 0.0
        discount = 1.0
        current_state = state

        for depth in range(self.config.rollout_depth):
            if problem.is_goal(current_state):
                total_reward += discount * self.config.goal_reward
                return total_reward, current_state

            actions = list(problem.actions(current_state))
            if not actions:
                break

            action, next_state = self._choose_rollout_action(current_state, actions, problem)
            cost = problem.action_cost(current_state, action, next_state)
            total_reward -= discount * cost

            current_state = next_state
            discount *= self.config.discount_factor

        if problem.is_goal(current_state):
            total_reward += discount * self.config.goal_reward
        else:
            total_reward -= discount * problem.heuristic(current_state)

        return total_reward, current_state

    def _backpropagate(self, node: _TreeNode[State, Action], reward: float) -> None:
        current = node
        discount = 1.0
        while current is not None:
            current.visits += 1
            current.total_reward += reward * discount
            discount *= self.config.discount_factor
            current = current.parent

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    def _choose_rollout_action(
        self,
        state: State,
        actions: Sequence[Action],
        problem: PlanningProblem[State, Action],
    ) -> Tuple[Action, State]:
        if self.config.epsilon_greedy <= 0.0:
            action = self._rng.choice(actions)
            return action, problem.transition(state, action)

        scored: List[Tuple[float, Action, State]] = []
        for action in actions:
            next_state = problem.transition(state, action)
            score = problem.heuristic(next_state)
            scored.append((score, action, next_state))

        scored.sort(key=lambda item: item[0])

        if not scored:
            action = self._rng.choice(actions)
            return action, problem.transition(state, action)

        if self._rng.random() < self.config.epsilon_greedy:
            _, action, next_state = self._rng.choice(scored)
        else:
            _, action, next_state = scored[0]
        return action, next_state

    def _best_child(self, node: _TreeNode[State, Action]) -> _TreeNode[State, Action]:
        best_score = -float("inf")
        best = None
        for child in node.children.values():
            exploit = child.total_reward / child.visits if child.visits > 0 else 0.0
            explore = self.config.exploration_constant * math.sqrt(
                math.log(node.visits + 1.0) / (child.visits + 1e-9)
            )
            score = exploit + explore
            if score > best_score:
                best_score = score
                best = child
        if best is None:
            return self._rng.choice(list(node.children.values()))
        return best

    def _extract_actions(self, node: _TreeNode[State, Action]) -> List[Action]:
        actions: List[Action] = []
        current = node
        while current.parent is not None and current.action is not None:
            actions.append(current.action)
            current = current.parent
        actions.reverse()
        return actions

    def _best_effort_trace(self, root: _TreeNode[State, Action]) -> Tuple[List[Action], float]:
        actions: List[Action] = []
        node = root
        while node.children:
            node = max(node.children.values(), key=lambda c: c.visits)
            if node.action is None:
                break
            actions.append(node.action)
            if len(actions) >= self.config.rollout_depth:
                break
        cost = node.path_cost if node else float("inf")
        return actions, cost


__all__ = ["MonteCarloPlanner", "MonteCarloConfig"]

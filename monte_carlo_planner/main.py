from __future__ import annotations

"""Command line interface for the Monte Carlo planning demo.

Run with ``python3 main.py --demo`` to solve the bundled grid world example,
or customize the environment via ``--width/--height/--obstacles`` parameters.
"""

import argparse
from typing import Iterable, List, Sequence, Set, Tuple

from planner import MonteCarloConfig, MonteCarloPlanner
from planner.problem import PlanningProblem
from examples.grid_world import GridWorldProblem, demo_problem

Coord = Tuple[int, int]


def parse_coord(coord_str: str) -> Coord:
    parts = coord_str.split(",")
    if len(parts) != 2:
        raise argparse.ArgumentTypeError("Coordinates must be formatted as 'x,y'.")
    try:
        return int(parts[0]), int(parts[1])
    except ValueError as exc:
        raise argparse.ArgumentTypeError("Coordinates must contain integers.") from exc


def parse_obstacles(obstacles_str: str) -> Set[Coord]:
    if not obstacles_str:
        return set()
    coords: Set[Coord] = set()
    for chunk in obstacles_str.split(";"):
        chunk = chunk.strip()
        if not chunk:
            continue
        coords.add(parse_coord(chunk))
    return coords


def reconstruct_path(problem: PlanningProblem[Coord, str], actions: Sequence[str]) -> List[Coord]:
    path = [problem.initial_state()]
    state = path[0]
    for action in actions:
        state = problem.transition(state, action)
        path.append(state)
    return path


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--demo", action="store_true", help="Run the bundled grid world instance.")
    parser.add_argument("--width", type=int, default=6, help="Grid width (ignored with --demo).")
    parser.add_argument("--height", type=int, default=6, help="Grid height (ignored with --demo).")
    parser.add_argument("--start", type=parse_coord, default=parse_coord("0,0"), help="Start coordinate x,y.")
    parser.add_argument("--goal", type=parse_coord, default=parse_coord("5,5"), help="Goal coordinate x,y.")
    parser.add_argument(
        "--obstacles",
        type=str,
        default="",
        help="Semicolon separated obstacle coordinates (e.g. '1,1;2,2').",
    )
    parser.add_argument("--iterations", type=int, default=2000, help="Number of MCTS iterations.")
    parser.add_argument("--rollout-depth", type=int, default=40, help="Simulation horizon for rollouts.")
    parser.add_argument(
        "--exploration",
        type=float,
        default=1.4,
        help="Exploration constant used by UCT.",
    )
    parser.add_argument("--seed", type=int, default=None, help="Optional random seed.")
    parser.add_argument(
        "--time-limit",
        type=float,
        default=None,
        help="Optional wall-clock time limit in seconds.",
    )
    return parser


def main() -> None:
    parser = build_parser()
    args = parser.parse_args()

    if args.demo:
        problem = demo_problem()
    else:
        problem = GridWorldProblem(
            width=args.width,
            height=args.height,
            start=args.start,
            goal=args.goal,
            obstacles=parse_obstacles(args.obstacles),
        )

    config = MonteCarloConfig(
        max_iterations=args.iterations,
        exploration_constant=args.exploration,
        rollout_depth=args.rollout_depth,
        random_seed=args.seed,
        time_limit=args.time_limit,
    )

    planner = MonteCarloPlanner(config=config)
    result = planner.plan(problem)

    print(result.pretty_plan())

    path = reconstruct_path(problem, result.actions)
    if isinstance(problem, GridWorldProblem):
        print()
        print(problem.render(path))


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
UPMurphi Solve-and-Dispatch Pipeline

End-to-end pipeline:
  1. Compile PDDL+ domain/problem → executable planner (via upmc)
  2. Run the planner (BFS or A*) → plan file
  3. Parse the plan
  4. Dispatch actions for simulation

Usage:
  python3 upmurphi_solve_and_dispatch.py \\
      --domain domain.pddl --problem problem.pddl \\
      --upmc-bin ../../bin/upmc \\
      --algorithm astar \\
      --handler jxb_arm \\
      --discretisation "1 5 2"
"""

import os
import sys
import subprocess
import argparse
import shutil
import tempfile

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from upmurphi_plan_parser import parse_plan_file
from plan_dispatcher import (
    PlanDispatcher, HANDLER_REGISTRY,
    SimulatedActionHandler, JxbArmActionHandler, TurtlebotActionHandler,
)


def find_upmc(hint: str = None) -> str:
    """Locate the upmc binary."""
    candidates = [
        hint,
        os.environ.get("UPMC_BIN"),
        "/workspace/UPMurphi-improved/UPMurphi-master/bin/upmc",
        "/workspace/UPMurphi-master/bin/upmc",
        "./bin/upmc",
        "../bin/upmc",
        "../../bin/upmc",
    ]
    for c in candidates:
        if c and os.path.isfile(c) and os.access(c, os.X_OK):
            return os.path.abspath(c)
    return None


def solve(domain: str, problem: str, upmc_bin: str, work_dir: str,
          discretisation: str = None, algorithm: str = "astar",
          memory_mb: int = 1000, time_limit: int = 0) -> str:
    """
    Run UPMurphi to solve a PDDL+ problem.
    Returns the path to the plan output file, or None on failure.
    """
    domain = os.path.abspath(domain)
    problem = os.path.abspath(problem)
    upmc_bin = os.path.abspath(upmc_bin)

    domain_base = os.path.splitext(os.path.basename(domain))[0]
    problem_base = os.path.splitext(os.path.basename(problem))[0]
    planner_name = f"{domain_base}_planner"
    plan_file = os.path.join(work_dir, f"{problem_base}_plan.pddl")

    os.makedirs(work_dir, exist_ok=True)
    shutil.copy2(domain, work_dir)
    shutil.copy2(problem, work_dir)

    # Step 1: Compile PDDL+ → executable planner
    print(f"\n{'─'*60}")
    print(f"  Step 1: Compiling PDDL+ → planner")
    print(f"  Domain:  {os.path.basename(domain)}")
    print(f"  Problem: {os.path.basename(problem)}")
    print(f"{'─'*60}")

    upmc_cmd = [upmc_bin, os.path.basename(domain), os.path.basename(problem)]
    if discretisation:
        upmc_cmd += ["--custom"] + discretisation.split()

    result = subprocess.run(upmc_cmd, cwd=work_dir,
                            capture_output=True, text=True, timeout=120)
    if result.returncode != 0:
        print(f"  ERROR: upmc failed (exit code {result.returncode})")
        print(result.stdout[-500:] if result.stdout else "")
        print(result.stderr[-500:] if result.stderr else "")
        return None

    planner_path = os.path.join(work_dir, planner_name)
    if not os.path.isfile(planner_path):
        print(f"  ERROR: planner binary not found: {planner_name}")
        return None

    print(f"  ✓ Planner compiled: {planner_name}")

    # Step 2: Run the planner
    print(f"\n{'─'*60}")
    print(f"  Step 2: Running planner ({algorithm})")
    print(f"  Memory: {memory_mb} MB")
    print(f"{'─'*60}")

    planner_cmd = [f"./{planner_name}", f"-m{memory_mb}"]
    if algorithm == "astar":
        planner_cmd.append("-vastar")
    if time_limit > 0:
        planner_cmd.append(f"-tl{time_limit}")

    result = subprocess.run(planner_cmd, cwd=work_dir,
                            capture_output=True, text=True, timeout=600)

    print(result.stdout[-1000:] if result.stdout else "(no output)")

    if not os.path.isfile(plan_file):
        print(f"  WARNING: Plan file not found: {plan_file}")
        return None

    print(f"  ✓ Plan file generated: {os.path.basename(plan_file)}")
    return plan_file


def main():
    parser = argparse.ArgumentParser(
        description="UPMurphi Solve-and-Dispatch: compile, plan, execute"
    )
    parser.add_argument("--domain", required=True, help="PDDL+ domain file")
    parser.add_argument("--problem", required=True, help="PDDL+ problem file")
    parser.add_argument("--upmc-bin", default=None, help="Path to upmc binary")
    parser.add_argument("--work-dir", default=None, help="Working directory")
    parser.add_argument("--discretisation", default=None,
                        help="Custom discretisation: 'timestep mantissa exponent'")
    parser.add_argument("--algorithm", choices=["bfs", "astar"], default="astar")
    parser.add_argument("--memory", type=int, default=1000, help="Memory in MB")
    parser.add_argument("--time-limit", type=int, default=0, help="Max plan length")
    parser.add_argument("--handler", choices=list(HANDLER_REGISTRY.keys()),
                        default="simulated")
    parser.add_argument("--time-scale", type=float, default=0.01)
    parser.add_argument("--plan-only", action="store_true",
                        help="Only solve, do not dispatch")
    parser.add_argument("--plan-file", default=None,
                        help="Skip solving, use existing plan file")

    args = parser.parse_args()

    upmc = find_upmc(args.upmc_bin)
    if not upmc and not args.plan_file:
        print("ERROR: Cannot find upmc binary. Use --upmc-bin or set UPMC_BIN.")
        sys.exit(1)

    work_dir = args.work_dir or tempfile.mkdtemp(prefix="upmurphi_")

    if args.plan_file:
        plan_file = args.plan_file
        print(f"Using existing plan file: {plan_file}")
    else:
        plan_file = solve(
            domain=args.domain,
            problem=args.problem,
            upmc_bin=upmc,
            work_dir=work_dir,
            discretisation=args.discretisation,
            algorithm=args.algorithm,
            memory_mb=args.memory,
            time_limit=args.time_limit,
        )
        if not plan_file:
            print("\nERROR: Planning failed.")
            sys.exit(1)

    if args.plan_only:
        print(f"\n  Plan file: {plan_file}")
        print("  (--plan-only: skipping dispatch)")
        sys.exit(0)

    # Step 3: Parse and dispatch
    print(f"\n{'─'*60}")
    print(f"  Step 3: Parsing and dispatching plan")
    print(f"{'─'*60}")

    plans = parse_plan_file(plan_file)
    if not plans:
        print("ERROR: No plans found in output file.")
        sys.exit(1)

    plan = plans[0]
    handler_cls = HANDLER_REGISTRY[args.handler]
    handler = handler_cls(time_scale=args.time_scale, verbose=True)
    dispatcher = PlanDispatcher(handler, real_time=False)

    success = dispatcher.dispatch(plan)
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()

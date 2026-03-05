#!/usr/bin/env python3
"""
UPMurphi Plan Dispatcher — Standalone Simulation

Reads a UPMurphi plan, dispatches actions in temporal order, and simulates
execution with configurable action handlers. Works both standalone (no ROS)
and as a ROS node (when ROS is available).

Dispatch modes:
  1. Standalone simulation — prints action execution timeline
  2. ROSPlan-compatible — publishes ActionDispatch messages
  3. ROS2 action client — sends goals to action servers

Usage:
  python3 plan_dispatcher.py <plan_file> [--mode sim|rosplan|ros2]
"""

import sys
import time
import json
import threading
from typing import Dict, Callable, Optional
from dataclasses import dataclass

from upmurphi_plan_parser import parse_plan_file, Plan, PlanAction


# ─── Action Handler Interface ────────────────────────────────────

@dataclass
class ActionResult:
    success: bool
    message: str = ""
    actual_duration: float = 0.0


class ActionHandler:
    """Base class for action execution handlers."""

    def execute(self, action: PlanAction) -> ActionResult:
        raise NotImplementedError


class SimulatedActionHandler(ActionHandler):
    """Simulates action execution with time delays."""

    def __init__(self, time_scale: float = 1.0, verbose: bool = True):
        self.time_scale = time_scale
        self.verbose = verbose

    def execute(self, action: PlanAction) -> ActionResult:
        params_str = " ".join(action.parameters)
        if self.verbose:
            print(f"  [EXEC] ({action.action_name} {params_str}) "
                  f"duration={action.duration:.3f}s")

        sim_duration = action.duration * self.time_scale
        if sim_duration > 0:
            time.sleep(sim_duration)

        return ActionResult(
            success=True,
            message=f"Simulated {action.action_name} completed",
            actual_duration=action.duration,
        )


# ─── Domain-Specific Handlers ────────────────────────────────────

class JxbArmActionHandler(ActionHandler):
    """Handler for the jxb robotic arm domain."""

    def __init__(self, time_scale: float = 0.01, verbose: bool = True):
        self.time_scale = time_scale
        self.verbose = verbose
        self.arm_position = "base1"
        self.captured_devices = set()

    def execute(self, action: PlanAction) -> ActionResult:
        name = action.action_name
        params = action.parameters

        if name == "move":
            return self._handle_move(params, action.duration)
        elif name == "capture":
            return self._handle_capture(params, action.duration)
        elif name == "release":
            return self._handle_release(params, action.duration)
        else:
            return self._handle_generic(action)

    def _handle_move(self, params, duration) -> ActionResult:
        if len(params) >= 3:
            robot, from_loc, to_loc = params[0], params[1], params[2]
        else:
            return ActionResult(False, f"move requires 3 params, got {len(params)}")

        if self.verbose:
            print(f"  [MOVE] {robot}: {from_loc} → {to_loc} "
                  f"(duration={duration:.1f}s)")

        time.sleep(duration * self.time_scale)
        self.arm_position = to_loc

        if self.verbose:
            print(f"  [MOVE] ✓ {robot} arrived at {to_loc}")

        return ActionResult(True, f"Moved to {to_loc}", duration)

    def _handle_capture(self, params, duration) -> ActionResult:
        if len(params) >= 3:
            device, location, robot = params[0], params[1], params[2]
        else:
            return ActionResult(False, f"capture requires 3 params, got {len(params)}")

        if self.verbose:
            print(f"  [CAPTURE] {robot}: capturing {device} at {location} "
                  f"(duration={duration:.1f}s)")

        time.sleep(duration * self.time_scale)
        self.captured_devices.add(device)

        if self.verbose:
            print(f"  [CAPTURE] ✓ {device} captured")

        return ActionResult(True, f"Captured {device}", duration)

    def _handle_release(self, params, duration) -> ActionResult:
        if len(params) >= 3:
            device, location, robot = params[0], params[1], params[2]
        else:
            return ActionResult(False, f"release requires 3 params, got {len(params)}")

        if self.verbose:
            print(f"  [RELEASE] {robot}: releasing {device} at {location} "
                  f"(duration={duration:.1f}s)")

        time.sleep(duration * self.time_scale)
        self.captured_devices.discard(device)

        if self.verbose:
            print(f"  [RELEASE] ✓ {device} released")

        return ActionResult(True, f"Released {device}", duration)

    def _handle_generic(self, action: PlanAction) -> ActionResult:
        params_str = " ".join(action.parameters)
        if self.verbose:
            print(f"  [ACTION] ({action.action_name} {params_str}) "
                  f"duration={action.duration:.3f}s")
        time.sleep(action.duration * self.time_scale)
        return ActionResult(True, f"Executed {action.action_name}", action.duration)


class TurtlebotActionHandler(ActionHandler):
    """Handler for the ROSPlan turtlebot domain."""

    def __init__(self, time_scale: float = 0.1, verbose: bool = True):
        self.time_scale = time_scale
        self.verbose = verbose
        self.robot_at = "wp0"

    def execute(self, action: PlanAction) -> ActionResult:
        name = action.action_name
        params = action.parameters

        if name == "goto_waypoint":
            return self._handle_goto(params, action.duration)
        elif name in ("localise", "dock", "undock"):
            return self._handle_simple(name, params, action.duration)
        else:
            if self.verbose:
                print(f"  [ACTION] {name}({', '.join(params)}) "
                      f"dur={action.duration:.1f}s")
            time.sleep(action.duration * self.time_scale)
            return ActionResult(True, f"Executed {name}", action.duration)

    def _handle_goto(self, params, duration) -> ActionResult:
        if len(params) >= 3:
            robot, from_wp, to_wp = params[0], params[1], params[2]
        elif len(params) >= 2:
            robot, to_wp = params[0], params[1]
            from_wp = self.robot_at
        else:
            return ActionResult(False, "goto_waypoint requires >= 2 params")

        if self.verbose:
            print(f"  [GOTO] {robot}: {from_wp} → {to_wp} "
                  f"(duration={duration:.1f}s)")

        time.sleep(duration * self.time_scale)
        self.robot_at = to_wp

        if self.verbose:
            print(f"  [GOTO] ✓ {robot} reached {to_wp}")

        return ActionResult(True, f"Reached {to_wp}", duration)

    def _handle_simple(self, name, params, duration) -> ActionResult:
        if self.verbose:
            print(f"  [{name.upper()}] {', '.join(params)} "
                  f"(duration={duration:.1f}s)")
        time.sleep(duration * self.time_scale)
        if self.verbose:
            print(f"  [{name.upper()}] ✓ done")
        return ActionResult(True, f"{name} completed", duration)


# ─── Dispatcher ──────────────────────────────────────────────────

HANDLER_REGISTRY: Dict[str, type] = {
    "simulated": SimulatedActionHandler,
    "jxb_arm": JxbArmActionHandler,
    "turtlebot": TurtlebotActionHandler,
}


class PlanDispatcher:
    """Dispatches plan actions in temporal order."""

    def __init__(self, handler: ActionHandler, real_time: bool = False):
        self.handler = handler
        self.real_time = real_time
        self.results = []

    def dispatch(self, plan: Plan) -> bool:
        print(f"\n{'='*60}")
        print(f"  Dispatching Plan #{plan.plan_number:05d}")
        print(f"  Actions: {plan.num_actions}")
        print(f"  Planned duration: {plan.total_duration:.3f}s")
        print(f"  Discretisation: {plan.discretisation}")
        print(f"{'='*60}\n")

        all_success = True
        sim_time = 0.0

        for i, action in enumerate(plan.actions):
            if self.real_time and action.timestamp > sim_time:
                wait = action.timestamp - sim_time
                time.sleep(wait)
                sim_time = action.timestamp

            print(f"[{action.timestamp:8.3f}] Action {i+1}/{plan.num_actions}: "
                  f"({action.action_name} {' '.join(action.parameters)})")

            result = self.handler.execute(action)
            self.results.append(result)

            status = "✓ SUCCESS" if result.success else "✗ FAILED"
            print(f"           {status}: {result.message}\n")

            if not result.success:
                all_success = False
                print("  *** Dispatch halted due to action failure ***")
                break

            sim_time = action.timestamp + action.duration

        print(f"{'='*60}")
        succeeded = sum(1 for r in self.results if r.success)
        print(f"  Dispatch complete: {succeeded}/{len(self.results)} actions succeeded")
        if all_success:
            print(f"  Plan executed successfully!")
        else:
            print(f"  Plan execution FAILED")
        print(f"{'='*60}\n")

        return all_success


# ─── Main ────────────────────────────────────────────────────────

def main():
    import argparse

    parser = argparse.ArgumentParser(
        description="UPMurphi Plan Dispatcher — dispatch plans for simulation"
    )
    parser.add_argument("plan_file", help="UPMurphi plan output file (.pddl)")
    parser.add_argument("--handler", choices=list(HANDLER_REGISTRY.keys()),
                        default="simulated",
                        help="Action handler type (default: simulated)")
    parser.add_argument("--time-scale", type=float, default=0.01,
                        help="Time scale for simulation (default: 0.01)")
    parser.add_argument("--real-time", action="store_true",
                        help="Dispatch actions in real-time")
    parser.add_argument("--plan-index", type=int, default=0,
                        help="Plan index to dispatch (default: 0 = first)")
    parser.add_argument("--json-report", type=str, default=None,
                        help="Write dispatch report to JSON file")
    args = parser.parse_args()

    plans = parse_plan_file(args.plan_file)
    if not plans:
        print("Error: No plans found in file.")
        sys.exit(1)

    if args.plan_index >= len(plans):
        print(f"Error: Plan index {args.plan_index} out of range "
              f"(file contains {len(plans)} plans)")
        sys.exit(1)

    plan = plans[args.plan_index]
    handler_cls = HANDLER_REGISTRY[args.handler]
    handler = handler_cls(time_scale=args.time_scale, verbose=True)
    dispatcher = PlanDispatcher(handler, real_time=args.real_time)

    success = dispatcher.dispatch(plan)

    if args.json_report:
        report = {
            "plan_file": args.plan_file,
            "plan_number": plan.plan_number,
            "num_actions": plan.num_actions,
            "planned_duration": plan.total_duration,
            "success": success,
            "results": [
                {"success": r.success, "message": r.message,
                 "actual_duration": r.actual_duration}
                for r in dispatcher.results
            ],
        }
        with open(args.json_report, 'w') as f:
            json.dump(report, f, indent=2, ensure_ascii=False)
        print(f"Report written to {args.json_report}")

    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()

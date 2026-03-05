# AGENTS.md

## Cursor Cloud specific instructions

### Project overview

UPMurphi is a PDDL+ automated planner that compiles domain/problem definitions into executable C++ planner binaries. There are two variants in the repo:

- `UPMurphi-master/` — original v3.0.3
- `UPMurphi-improved/UPMurphi-master/` — fork adding A\* search (`-vastar` flag)

Both share the same build system and workflow. See `UPMurphi-master/README.md` for usage details.

### System dependencies (installed via apt)

`build-essential flex bison byacc libc6-dev-i386 gcc-multilib g++-multilib`

These are not managed by the update script; they must be pre-installed in the VM image.

### Building

Run `make` from `UPMurphi-master/src/` or `UPMurphi-improved/UPMurphi-master/src/`. This produces `upmc` and `pddl2upm` in the respective `bin/` directories.

### Known build gotchas

1. **Stale `lex.yy.cc` in `UPMurphi-improved`**: The `cleanall` Makefile target removes `lex.yy.c` but the PDDL parser generates `lex.yy.cc`. If the build fails with FlexLexer pointer/reference type errors, manually delete `src/UPMurphi_parser/lex.yy.cc` and re-run `make`.

2. **`max` macro vs `std::numeric_limits::max()`**: The legacy `max(a,b)` macro in `include/upm_state.cpp` conflicts with `std::numeric_limits<unsigned long>::max()` in the A\* search code (`include/upm_system.cpp`). This was fixed by parenthesizing: `(std::numeric_limits<unsigned long>::max)()`.

3. **32-bit compilation**: Generated planners are compiled with `-m32`. The `libc6-dev-i386`, `gcc-multilib`, and `g++-multilib` packages must be installed.

### Running a planner (end-to-end test)

```bash
cd UPMurphi-master/ex/generator_linear
../../bin/upmc genlinear.pddl genproblinear.pddl --custom 1 5 2
./genlinear_planner -m 200 -tl 100
```

This compiles the PDDL+ domain/problem into a planner binary, then runs it to find a plan. Output is written to `genproblinear_plan.pddl`.

### ROS2 integration

ROS2 Jazzy is installed (`/opt/ros/jazzy`). Source it with `source /opt/ros/jazzy/setup.bash` (already in `~/.bashrc`).

The `upmurphi_ros_bridge/` package connects UPMurphi plan output to ROS2:

```bash
# Standalone solve + dispatch
cd upmurphi_ros_bridge/scripts
python3 upmurphi_solve_and_dispatch.py \
    --domain path/to/domain.pddl --problem path/to/problem.pddl \
    --algorithm astar --handler simulated

# ROS2 node dispatch
python3 ros2_plan_dispatcher.py --ros-args -p plan_file:=plan.pddl

# Animated Gantt visualization
python3 plan_visualizer.py plan.pddl --animate
```

Published topics: `/upmurphi/action_dispatch`, `/upmurphi/dispatch_status`, `/upmurphi/plan`.

### Linting / testing

There are no dedicated lint or test targets. Correctness is verified by building the tools and running them against example PDDL+ problems in `ex/`. The build itself (via `make`) is the primary quality gate.

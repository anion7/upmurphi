# AGENTS.md

## Cursor Cloud specific instructions

### Project overview

UPMurphi is a C/C++ PDDL+ planning toolchain. There are two variants:
- `UPMurphi-master/` — original UPMurphi v3.0.3
- `UPMurphi-improved/` — fork with A* search (`-vastar` switch)

No web services, databases, or containers. Pure CLI tools built with Make.

### Critical: flex version

The PDDL parser (`pddl2upm`) requires **flex 2.5.39**. The system flex 2.6.4 generates code incompatible with the older `FlexLexer.h` API (pointer vs reference semantics). flex 2.5.39 must be installed from source at `/usr/local/bin/flex` and its `FlexLexer.h` must replace the system copy at `/usr/include/FlexLexer.h`.

### Critical: max macro conflict

The `#define max(a,b)` macro in `include/upm_state.cpp` conflicts with `std::numeric_limits<unsigned long>::max()` calls in `include/upm_system.cpp`. The fix is to parenthesize: `(std::numeric_limits<unsigned long>::max)()`. This fix has been applied to both variants.

### Build commands

```bash
# Build both upmc and pddl2upm (ensure bin/ directory exists first)
mkdir -p UPMurphi-master/bin && cd UPMurphi-master/src && make

# Same for improved variant
mkdir -p UPMurphi-improved/UPMurphi-master/bin && cd UPMurphi-improved/UPMurphi-master/src && make
```

### Running a planner

```bash
# 1. Compile PDDL+ domain/problem into executable planner
cd UPMurphi-master/ex/tank_typed
../../bin/upmc tank_domain.pddl tank_problem.pddl --custom 0.5 5 2

# 2. Run the planner (plan written to tank_problem_plan.pddl)
./tank_domain_planner -m 256 -tl 20
```

### No lint or automated tests

This project has no linter configuration or test suite. Verification is done by building the toolchain and running example planners.

### Gotchas

- UPMurphi compiles planners with `-m32` (32-bit mode), so `gcc-multilib` and `g++-multilib` are required.
- The `bin/` directories may not exist initially; create them before building.
- BFS planning can be extremely slow for large state spaces. Use `-tl <seconds>` to limit plan duration and `-m <MB>` to limit memory.

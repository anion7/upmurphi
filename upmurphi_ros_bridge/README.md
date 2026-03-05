# UPMurphi-ROS Bridge

将 UPMurphi PDDL+ 规划器的求解结果接入 ROS 执行仿真。

## 架构

```
PDDL+ Domain + Problem
        │
        ▼  upmc (UPMurphi compiler)
  Executable Planner
        │
        ▼  planner -vastar / -vbfs
  Plan File (.pddl)
        │
        ▼  upmurphi_plan_parser.py
  Structured Plan (Python objects)
        │
        ├──▶ plan_dispatcher.py         (standalone simulation)
        ├──▶ ros2_plan_dispatcher.py    (ROS2 node dispatch)
        └──▶ ROSPlan format export      (for rosplan_demos integration)
```

## 快速开始

### 1. 解析已有计划
```bash
python3 scripts/upmurphi_plan_parser.py plans/example_plan.pddl --summary
python3 scripts/upmurphi_plan_parser.py plans/example_plan.pddl --rosplan
python3 scripts/upmurphi_plan_parser.py plans/example_plan.pddl --json
```

### 2. 独立仿真执行
```bash
python3 scripts/plan_dispatcher.py plans/example_plan.pddl --handler simulated
python3 scripts/plan_dispatcher.py plans/example_plan.pddl --handler turtlebot
python3 scripts/plan_dispatcher.py plans/example_plan.pddl --handler jxb_arm
```

### 3. 一键求解 + 执行
```bash
python3 scripts/upmurphi_solve_and_dispatch.py \
    --domain path/to/domain.pddl \
    --problem path/to/problem.pddl \
    --algorithm astar \
    --discretisation "1 5 2" \
    --handler simulated
```

### 4. ROS2 节点 (需安装 ROS2)
```bash
ros2 run upmurphi_ros_bridge ros2_plan_dispatcher \
    --ros-args -p plan_file:=plans/example_plan.pddl
```

## Action Handlers

| Handler | 域 | 说明 |
|---------|---|------|
| `simulated` | 通用 | 模拟执行，按时间延迟 |
| `jxb_arm` | jxb_move | 机械臂 move/capture/release |
| `turtlebot` | turtlebot | goto_waypoint / dock / undock |

## 与 ROSPlan 集成

UPMurphi 输出的 PDDL 计划格式与 ROSPlan (POPF) 高度兼容:

```
# UPMurphi 格式
0.000: ( action_name param1 param2) [10.000]

# ROSPlan/POPF 格式
0.000000: (action_name param1 param2) [10.000000]
```

使用 `--rosplan` 选项可将 UPMurphi 计划转换为 ROSPlan 兼容格式:

```bash
python3 scripts/upmurphi_plan_parser.py plan.pddl --rosplan > rosplan_plan.pddl
```

转换后的计划可直接被 ROSPlan 的 plan parser 读取。

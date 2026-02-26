# UPMurphi 算法伪代码 (Algorithm Pseudocode)

## 总体规划流程 (Main Planning Pipeline)

UPMurphi 采用 **Discretise-and-Validate** 方法，将 PDDL+ 连续规划问题离散化后进行状态空间搜索。
整个规划流程分为 6 个阶段：

```
Algorithm: PLAN()
───────────────────────────────────────────────
Phase 1 — 状态空间探索 (State Space Exploration)
    if mode = BFS then
        EXPLORE_BFS()
    else if mode = DFS then
        EXPLORE_DFS()
    else if mode = A* then
        EXPLORE_ASTAR()

Phase 2 — 构建模型动力学 (Build Model Dynamics)
    BUILD_DYNAMICS()

Phase 3 — 寻找控制路径 (Find Control Paths)
    FIND_PATHS()

Phase 4 — 收集计划 (Collect Plans)
    COLLECT_PLANS()

Phase 5 — 输出结果 (Output Results)
    OUTPUT_RESULTS()

Phase 6 — 验证结果 [可选] (Validate Results)
    VALIDATE_RESULTS()
```

---

## Phase 1a: BFS 宽度优先搜索 (Breadth-First Search Exploration)

```
Algorithm: EXPLORE_BFS()
───────────────────────────────────────────────
Input:  PDDL+ 离散化模型, horizon (最大BFS层数)
Output: 状态空间 StateSet, 转移关系, 目标状态集合

 1│  StateSet ← new StateManager(queue_mode=BFS)
 2│  初始化 StartState, Properties, Symmetry, PO 管理器
 3│
 4│  // 生成所有初始状态并加入队列
 5│  for each startstate s₀ do
 6│      StateSet.ADD(s₀, transition=NULL)
 7│  end for
 8│
 9│  // BFS 主循环
10│  while Queue ≠ ∅  AND  CurrentLevel ≤ horizon do
11│      ⟨s, index⟩ ← Queue.DEQUEUE()         // FIFO 出队
12│      curstate ← s
13│      workingstate ← copy(s)
14│
15│      // 对当前状态施加所有可行规则 (actions + processes + events)
16│      deadlocked ← ALLNEXTSTATES(curstate)
17│
18│      if deadlocked then
19│          报告死锁错误
20│
21│      CheckLevel()                           // 检查 BFS 层级
22│
23│      // 提前终止条件
24│      if search_mode = Feasible AND NumGoals > 0 then
25│          break                              // 找到可行解即停
26│      if TableIsFull then
27│          if NumGoals > 0 then  break
28│          else  报告内存不足错误
29│  end while
30│
31│  记录统计信息 (已探索状态数, 目标数, 转移数)
```

---

## Phase 1b: A\* 最佳优先搜索 (A\* Best-First Search Exploration)

```
Algorithm: EXPLORE_ASTAR()
───────────────────────────────────────────────
Input:  PDDL+ 离散化模型, horizon_limit (时间上限)
Output: 状态空间 StateSet, 转移关系, 目标状态集合

 1│  StateSet ← new StateManager(queue_mode=PRIORITY, priority_ref=astar_f)
 2│  初始化 astar_g[], astar_h[], astar_f[], astar_parent[], astar_rule[]
 3│      ∀i: astar_g[i] ← ∞,  astar_f[i] ← ∞
 4│  初始化 StartState, Properties, Symmetry, PO 管理器
 5│
 6│  // 生成所有初始状态, g=0, 加入优先队列
 7│  for each startstate s₀ do
 8│      StateSet.ADD(s₀, transition=NULL)
 9│      // ADD 内部: astar_g[s₀] ← 0, astar_f[s₀] ← h(s₀)
10│  end for
11│
12│  // A* 主循环
13│  while PriorityQueue ≠ ∅ do
14│      ⟨s, index⟩ ← PriorityQueue.DEQUEUE()  // 取 f 值最小的状态
15│      curstate ← s
16│      workingstate ← copy(s)
17│
18│      // 目标检测
19│      if IsGoalState(index) then
20│          expanded_goal ← index
21│          if search_mode = Feasible then
22│              break                           // 找到最优可行解
23│          else
24│              continue                        // 继续搜索其他目标
25│
26│      // 超出时间上限则跳过
27│      if enforce_horizon AND astar_g[index] > horizon_limit then
28│          continue
29│
30│      // 扩展当前状态
31│      deadlocked ← ALLNEXTSTATES(curstate)
32│
33│      if deadlocked then  报告死锁错误
34│
35│      if TableIsFull then
36│          if NumGoals > 0 then  break (用已有目标建计划)
37│          else  报告内存不足错误
38│  end while
39│
40│  记录统计信息
```

---

## 状态添加与 A\* 代价更新 (State Addition & Cost Update)

```
Algorithm: StateManager.ADD(s, transition)
───────────────────────────────────────────────
Input:  state s, transition = {parent_index, applied_rule, step_cost}
Output: true if s is new, false otherwise
        Side-effects: 更新队列, 记录转移, 更新 A* 代价

 1│  ⟨is_new, state_index⟩ ← HashTable.WAS_PRESENT(s)
 2│
 3│  // ═══════ 情况 A: 状态已存在 (重复状态) ═══════
 4│  if NOT is_new then
 5│      if astar_mode AND transition ≠ NULL then
 6│          parent_cost ← astar_g[transition.parent_index]
 7│          candidate   ← parent_cost + transition.step_cost
 8│          // 松弛操作: 如果找到更短路径
 9│          if candidate < astar_g[state_index] then
10│              RECORD_ASTAR_DATA(state_index, transition, candidate)
11│              Queue.ENQUEUE(s, state_index)   // 用新优先级重新入队
12│      return false
13│
14│  // ═══════ 情况 B: 新状态 ═══════
15│  state_valid ← false
16│  goal_state  ← false
17│
18│  if NOT CheckInvariants(s) then
19│      // 无效状态 (违反不变量)
20│      num_errors++
21│      discard s
22│
23│  else if CheckGoals(s) ≠ -1 then
24│      // 目标状态
25│      goal_state ← true
26│      RecordGoal(state_index)
27│      num_goals++
28│      if astar_mode then
29│          state_valid ← true              // A* 模式下目标也入队
30│      else
31│          discard s                        // BFS 模式下目标不继续扩展
32│
33│  else
34│      // 普通合法状态
35│      if NOT astar_mode then
36│          statesNextLevel++
37│      state_valid ← true
38│
39│  // A* 代价计算 (入队之前设置, 保证优先级正确)
40│  if astar_mode AND state_valid then
41│      if transition ≠ NULL then
42│          g ← astar_g[transition.parent_index] + transition.step_cost
43│      else
44│          g ← 0                            // 初始状态
45│      RECORD_ASTAR_DATA(state_index, transition, g, goal_state)
46│
47│  // 入队
48│  if state_valid then
49│      Queue.ENQUEUE(s, state_index)
50│
51│  return true
```

---

## A\* 代价记录 (A\* Cost Recording)

```
Algorithm: RECORD_ASTAR_DATA(index, parent, rule, g_cost, is_goal)
───────────────────────────────────────────────
 1│  astar_parent[index] ← parent
 2│  astar_rule[index]   ← rule
 3│  astar_g[index]      ← g_cost
 4│  astar_h[index]      ← ESTIMATE_HEURISTIC(is_goal)
 5│  astar_f[index]      ← g_cost + astar_h[index]
 6│  astar_goal[index]   ← is_goal


Algorithm: ESTIMATE_HEURISTIC(is_goal)
───────────────────────────────────────────────
 1│  if is_goal then  return 0
 2│  else             return 1          // 默认启发: 非目标→1, 目标→0
```

---

## A\* 优先队列出队 (Priority Queue with Staleness Check)

```
Algorithm: PriorityQueue.DEQUEUE()
───────────────────────────────────────────────
 1│  while PQ ≠ ∅ do
 2│      entry ← PQ.TOP()                       // 取 f 最小的条目
 3│      reference ← astar_f[entry.index]        // 当前真实的 f 值
 4│      if |reference − entry.priority| > ε then
 5│          PQ.POP()                             // 过期条目, 丢弃
 6│          continue
 7│      result ← entry.node
 8│      PQ.POP()
 9│      return result
10│  end while
11│  Error: "Empty priority queue"
```

---

## 状态扩展: 施加所有规则 (Apply All Rules)

```
Algorithm: ALLNEXTSTATES(curstate)
───────────────────────────────────────────────
Input:  curstate (当前状态), curstateindex
Output: true if deadlocked (无规则可施加)

 1│  originalstate ← copy(workingstate)
 2│  deadlocked ← true
 3│  from_index ← curstateindex
 4│
 5│  // 遍历所有规则 (durative actions, instantaneous actions,
 6│  //               processes, events, clock ticks, ...)
 7│  for rule_id = 0 to NumRules-1 do
 8│      if TableIsFull then break
 9│
10│      if rule_id ∈ EnabledRules AND Priority(rule_id) ≤ minPriority then
11│          weight   ← RuleWeight(rule_id)
12│          duration ← RuleDuration(rule_id)
13│
14│          nextstate ← ApplyRule(rule_id, workingstate)  // 施加规则产生后继
15│
16│          if nextstate ≠ curstate then                  // 有效转移
17│              deadlocked ← false
18│
19│              // 计算步代价 (用于 A*)
20│              step_cost ← weight (或 duration, 默认 1.0)
21│
22│              transition ← {parent_index: from_index,
23│                            applied_rule: rule_id,
24│                            step_cost:    step_cost}
25│
26│              // 将后继状态加入状态集合
27│              ⟨new, next_index, is_error⟩ ← StateSet.ADD(
28│                  nextstate, valid=true, transition)
29│
30│              if NOT is_error then
31│                  RecordTransition(from_index → next_index, rule_id)
32│
33│          workingstate ← copy(originalstate)  // 恢复工作状态
34│  end for
35│
36│  // 批量写入转移关系
37│  FlushTransitionBuffer(from_index)
38│
39│  return deadlocked
```

---

## Phase 2: 构建模型动力学 — 反转图 (Build Dynamics — Inverted Graph)

```
Algorithm: BUILD_DYNAMICS()
───────────────────────────────────────────────
Input:  Phase 1 产出的转移文件 (from → to, rule, weight, duration)
Output: 反转转移图 StateGraph (to → from)

 1│  StateGraph ← new GraphManager(num_states)
 2│
 3│  // 第一轮: 统计每个节点的入度 (反转后的出度)
 4│  for each transition (from → to, rule) in TransitionFile do
 5│      StateGraph.inc_outgoing_num(from)   // 统计 from 的"反转出度"
 6│  end for
 7│
 8│  StateGraph.setup_outgoing_lists()         // 分配邻接表内存
 9│
10│  // 第二轮: 填充反转邻接表
11│  for each transition (from → to, rule, weight, duration) in TransitionFile do
12│      StateGraph.add_outgoing(from, to, rule, weight, duration)
13│      // 注: 存储的是 from→to, 但语义是"从 to 可以反向到达 from"
14│  end for
```

---

## Phase 3: 寻找控制路径 — 反向 Dijkstra (Find Paths — Backward Dijkstra)

```
Algorithm: FIND_PATHS()
───────────────────────────────────────────────
Input:  反转图 StateGraph, 目标状态集合 Goals
Output: 每个可控状态的最优后继动作 chosen_edge[]

 1│  distance[]    ← [∞, ∞, ..., ∞]       // 每个状态到目标的距离
 2│  chosen_edge[] ← [∅, ∅, ..., ∅]       // 每个状态选择的最优边
 3│  has_next[]    ← [false, ..., false]   // 标记已找到路径的状态
 4│  Q ← empty queue
 5│
 6│  // 初始化: 从所有目标状态开始反向搜索
 7│  for each goal g ∈ Goals do
 8│      distance[g] ← 0
 9│      Q.ENQUEUE(g)
10│  end for
11│
12│  // 反向 Dijkstra / BFS
13│  while Q ≠ ∅ do
14│      s ← Q.DEQUEUE()
15│
16│      if search_mode = Optimal or Universal_Optimal then
17│          对 s 的邻居按权重排序 (贪心选最小)
18│
19│      // 遍历 s 在反转图中的所有邻居 (即原图中 s 的前驱)
20│      for each edge (s → pred) in StateGraph.outgoing(s) do
21│          new_dist ← distance[s] + edge.weight
22│
23│          if NOT has_next[pred]  OR  new_dist < distance[pred] then
24│              chosen_edge[pred] ← {to: s, rule: edge.rule, weight, duration}
25│              distance[pred]    ← new_dist
26│
27│              if NOT has_next[pred] then
28│                  has_next[pred] ← true
29│                  num_controlled_states++
30│                  Q.ENQUEUE(pred)
31│
32│              if search_mode = Feasible then
33│                  break                  // 可行解只需找到一条路径
34│  end while
35│
36│  // 输出: 将 chosen_edge[] 写入 actions 文件
37│  for each state s with has_next[s] = true do
38│      write (s, chosen_edge[s].rule, chosen_edge[s].to) → ActionsFile
39│  end for
```

---

## Phase 4: 收集计划 (Collect Plans)

```
Algorithm: COLLECT_PLANS()
───────────────────────────────────────────────
Input:  ActionsFile (每个可控状态的最优动作)
Output: 从每个初始状态到目标的完整计划

 1│  // 读取 actions, 构建正向动作图
 2│  ActionGraph ← new GraphManager(num_states)
 3│  for each (state, rule, next_state) in ActionsFile do
 4│      ActionGraph.add_edge(state → next_state, rule)
 5│  end for
 6│
 7│  // 从每个初始状态出发, 沿 ActionGraph 追踪到目标
 8│  for each start_state s₀ do
 9│      BUILD_PLAN_FROM(s₀)
10│  end for


Algorithm: BUILD_PLAN_FROM(start)
───────────────────────────────────────────────
 1│  current ← start
 2│  plan ← []
 3│
 4│  while ActionGraph.OutDegree(current) > 0 do
 5│      edge ← ActionGraph.GetOutgoing(current, 0)
 6│      plan.append(⟨current, edge.rule, edge.weight, edge.duration⟩)
 7│      current ← edge.to
 8│  end while
 9│
10│  // current 现在是目标状态 (无后继动作)
11│  plan.append(⟨current, END⟩)
12│
13│  更新统计: plan_length, plan_duration, plan_weight
14│  return plan
```

---

## 算法流程总览图 (Algorithm Flow Overview)

```
┌─────────────────────────────────────────────────────────────────────┐
│                    PDDL+ Domain + Problem                          │
│                         ↓  pddl2upm                                │
│                    UPMurphi Model (.m)                              │
│                         ↓  upmc                                    │
│                    C++ Source (.cpp)                                │
│                         ↓  g++ -m32                                │
│                    Executable Planner                               │
└─────────────────────┬───────────────────────────────────────────────┘
                      │
    ┌─────────────────▼─────────────────┐
    │  Phase 1: 状态空间探索             │
    │  ┌─────────┬──────────┬─────────┐ │
    │  │   BFS   │   DFS    │   A*    │ │
    │  │  FIFO   │  Stack   │ MinHeap │ │
    │  │  Queue  │          │  (f=g+h)│ │
    │  └────┬────┴────┬─────┴────┬────┘ │
    │       └─────────┼──────────┘      │
    │                 ↓                  │
    │  对每个状态: ALLNEXTSTATES()       │
    │  → 施加所有 enabled rules          │
    │  → ADD(nextstate, transition)      │
    │  → 记录转移 (from → to, rule)      │
    │                                    │
    │  输出: 状态集合, 转移文件, 目标文件 │
    └─────────────────┬──────────────────┘
                      ↓
    ┌─────────────────▼──────────────────┐
    │  Phase 2: 构建反转图               │
    │  将 from→to 转移构建为 to→from     │
    │  反转邻接表, 用于反向路径搜索       │
    └─────────────────┬──────────────────┘
                      ↓
    ┌─────────────────▼──────────────────┐
    │  Phase 3: 反向路径搜索             │
    │  从 Goal 状态出发                  │
    │  反向 Dijkstra 到所有可达状态       │
    │  为每个可控状态选择最优动作          │
    │                                    │
    │  搜索策略:                          │
    │  ├─ Feasible:  任意一条路径         │
    │  ├─ Optimal:   最短/最轻路径        │
    │  └─ Universal: 所有状态的通用策略   │
    └─────────────────┬──────────────────┘
                      ↓
    ┌─────────────────▼──────────────────┐
    │  Phase 4: 收集计划                 │
    │  从每个初始状态沿最优动作链         │
    │  追踪到目标, 生成完整计划序列       │
    └─────────────────┬──────────────────┘
                      ↓
    ┌─────────────────▼──────────────────┐
    │  Phase 5: 输出结果                 │
    │  格式: PDDL+ / Text / CSV / Raw   │
    │                                    │
    │  Phase 6: 自动验证 (可选)          │
    │  调用 VAL 验证器                   │
    └────────────────────────────────────┘
```

---

## A\* vs BFS 对比

```
┌──────────────────┬─────────────────────┬──────────────────────────┐
│     特性          │       BFS           │         A*               │
├──────────────────┼─────────────────────┼──────────────────────────┤
│ 队列类型          │ FIFO Queue          │ Min-Heap (按 f 排序)     │
│ 扩展顺序          │ 按层级 (level)       │ 按 f = g + h            │
│ g 代价            │ 不追踪              │ 累计步代价               │
│ h 启发            │ 无                  │ 目标→0, 非目标→1        │
│ 松弛操作          │ 无                  │ 找到更短路径时重新入队    │
│ 目标处理          │ 目标不入队           │ 目标入队, 展开时检测      │
│ 终止条件(Feasible)│ NumGoals > 0        │ 展开到目标节点时           │
│ 过期条目处理       │ 无                  │ 出队时检查优先级是否过期  │
│ 最优性保证        │ 层数最优             │ 代价最优 (h 可容)        │
└──────────────────┴─────────────────────┴──────────────────────────┘
```

# 水下铺缆机器人动态路线规划算法设计

> **版本**: 3.0
> **状态**: Released As-built Design Baseline（D10 双向审计已完成；本发布是算法设计基线，不代表生产就绪）
> **基于需求**: PRD.md v1.0  
> **代码基线**: `bd102b44ad217a42fd1dda815f61599b7dc10b79`（D09 完成提交，2026-08-26；D10 仅执行发布审计与文档修订，不改变算法行为）
> **验证基线**: T01-T47、T58-T65 的 Level 1 自动化证据与 D10 发布验证；T48-T54 外部验证、T55 基线对比和 T56 消融仍未完成
> **文档目的**: 记录当前 `underwater_planner` 的已实现行为、规范性安全要求、验证证据和尚未获得的外部能力声明
> **修订说明**: v3.0 由 D01-D08 逐域完成 as-built 对齐，D09 收敛公共模块架构、接口所有权、复杂度、Level 1 证据、外部验证状态、风险/限制、待标定参数、总结和术语，D10 最终完成设计到实现/测试及公共模块到设计的双向审计并发布。当前基线包含 T58-T65 对快照/空间域、参数解析与 production 门禁、失败结果时间、粗糙度、碰撞地图版本、几何起点来源和主循环滞回集成的修复。生产参数、独立标定、DAVE/Gazebo、水池、海试、三算法基线和消融仍未完成；v3.0 发布不构成生产就绪或外部验收声明。v2.6 将承诺段安全事件判定与异步撤租从路径稳定性职责中分离为 `CommitmentSafetyEvaluator` / `CommitmentSafetySupervisor`；v2.5 固化搜索、平滑和时序化超时以及旧计划最新上下文复检；v2.4 固化机械历史与跨原语积分；v2.3 固化缆线插值与曲率评价间距；v2.2 固化坐标、supercover、历史窗口和分析配置审计；v2.1 固化最低地形置信度；v2.0 建立几何/时序、模型、包络和租约的版本闭环。

---

## 1. 文档范围与目的

### 1.1 目的

本文档描述基线 `bd102b44ad217a42fd1dda815f61599b7dc10b79` 上 `underwater_planner` 路线规划算法的现状设计。文档面向实现维护人员、测试人员、算法验证人员和后续外部验收人员，提供：

- 已实现核心算法的数学模型、输入输出、运行流程和约束设计
- 作为规范性要求持续生效的硬约束、失败关闭和版本一致性语义
- 当前公共模块、自动化测试和确定性闭环证据的可追踪基线
- 尚待外部仿真、独立标定、水池和海试验证的能力与生产门禁

本文档不是仅供后续开发的前瞻方案，也不是生产就绪声明。v3.0 中关于“已实现”或“已验证”的陈述必须能够追踪到当前公共契约、实现、自动化测试或确定性报告；尚未取得相应证据的内容只作为规范性设计要求、TBD、未实现项或外部阻塞项保留。D01-D09 完成分域对齐和横向收敛，D10 在代码基线 `bd102b44ad217a42fd1dda815f61599b7dc10b79` 上完成最终双向审计后发布本版。

### 1.2 范围

当前代码基线实现并在 Level 1 范围内验证的算法职责包括：

- **主机器人局部路径规划**：基于版本化参考路线和局部地图生成几何路径与受约束的时序剖面
- **2.5D 海床通行性评价**：从高程地图派生坡度、台阶、粗糙度等图层，并对完整机器人足迹执行硬约束评价
- **缆线触地点预测与约束**：从实际缆线状态和计划执行剖面预测触地点，检查机械约束、统计包络和施工走廊
- **稳定重规划与执行授权**：滚动规划、路径滞回、近端承诺段、剩余计划复检、短期租约与异步撤租
- **主动补探协调**：识别信息缺口、生成补探目标并关联地图更新；Level 1 仅验证算法闭环，不代表双机仿真或实机能力已验收
- **一致性与降级管理**：冻结同步输入、拒绝版本回退和消息乱序，并在安全证据不足时失败关闭或受控停车

当前仓库和 v3.0 首版范围不实现：

- 原始传感器数据处理和地图构建（上游模块）
- 主机器人底层运动控制（下游模块）
- 前置机器人完整三维路径规划（独立模块）
- 高保真柔性电缆动力学（第一阶段范围外）
- 整条路径联合走廊越界概率 `epsilon_path` 与误差相关长度方法
- 整条路径联合地形梯度失效概率

当前代码基线也不构成 DAVE/Gazebo 双机闭环、长时通信异常、独立地形/缆线/机器人能力标定、水池试验或海试的完成证据。未经这些外部验证，不得宣称生产就绪、正式验收通过或在目标平台达到 2-5 Hz。

### 1.3 设计原则

1. **证据约束陈述**：代码事实由当前公共契约和实现支撑，验证结论由可重复测试或报告支撑，外部能力只由对应外部试验支撑。
2. **参数化且版本化**：所有物理量使用带单位、来源和版本的配置；缺少生产必需参数或独立标定版本时失败关闭，示例值和 `non_production_capability_profile` 不得冒充生产能力。
3. **硬约束优先**：安全、空间域、机械、版本和执行授权是硬门禁，不能被软代价、降级开关或经验默认值覆盖。
4. **风险语义隔离**：机器人碰撞、地形局部风险和缆线触地点局部风险分别建模；首版只声明局部逐点评价，不声称整路径联合风险保证。
5. **不可变与可审计**：规划和复检使用同一同步、不可变、完整版本化上下文；结果保留时间戳、运行域、参数/模型版本和结构化失败诊断。
6. **确定性可复现**：相同输入、参数、随机种子和版本依赖产生逐字段一致的结果，资源耗尽、超时和无解均保留可重放证据。
7. **已验证边界内授权**：只有通过独立复检并取得有效短期租约的时序路径可以执行；任何关键证据变化均撤销租约并进入安全路径。
8. **工程融合创新**：明确区分成熟算法、工程融合和新增机制，不把 Level 1 算法验证外推为外部平台能力。

### 1.4 证据层级与陈述规则

| 层级 | 文档中的含义 | 最低证据 | 当前发布规则 |
|---|---|---|---|
| **代码事实** | 当前提交中存在的类型、接口、状态转换或算法行为 | 固定提交上的公共契约/实现，以及对应单元或确定性测试 | D10 双向复核通过后可以写为“已实现”；后续代码变化必须重新核验 |
| **规范性安全要求** | 即使实现重构也不得弱化的硬约束、失败关闭、版本和风险语义 | 设计条款、实现门禁和负向/边界测试三者可追踪 | 可以使用“必须/禁止”；发现不一致时记录缺口，不能改文档掩盖实现问题 |
| **Level 1 验证结论** | 在合成输入和确定性测试驱动器中观察到的算法行为 | 可重复的 CTest、JUnit 或结构化确定性报告 | 只能声明算法级、合成场景范围内通过 |
| **外部能力声明** | 仿真、标定、实机、长时性能或甲方验收能力 | T48-T54 对应的外部环境、独立数据或实物试验报告 | 当前全部阻塞，不得写为已完成或由 Level 1 替代 |

代码事实描述“当前是什么”，规范性安全要求描述“任何合规实现必须是什么”。若两者冲突，以失败关闭原则维持安全边界，并将差异记录为代码或设计决策任务；不得通过降低硬约束、扩大未标定置信区间或删除风险声明取得表面一致。

### 1.5 v3.0 as-built 发布基线

本轮对齐冻结以下事实基线：

- Git 提交：`bd102b44ad217a42fd1dda815f61599b7dc10b79`，提交主题 `docs: converge D09 architecture and evidence`；该点包含 D01-D09 文档对齐及 T58-T65 代码修复，D10 不修改算法源文件或测试标识。
- 生产核心：`underwater_planner::core`，34 个 `.cpp` 实现单元，32 个 `include/underwater_planner/core/*.hpp` 公共头文件。
- 测试支持：`underwater_planner::test_support`，包含确定性闭环驱动器、结构化报告和合成夹具；它不是生产接口。
- 自动化入口：37 个 `test_*.cpp` 测试源文件，CMake 注册 38 个 CTest 项，其中 `level1_closed_loop_report_json` 是对 T47 结构化报告的独立格式/内容校验。

公共核心头文件按职责分组如下；第 14.2 节给出 D09 收敛后的逐文件所有权和公共 seam：

| 职责 | 公共头文件（省略 `.hpp`） |
|---|---|
| 契约、配置与诊断 | `data_contract`, `parameter_config`, `version`, `versioned_snapshot`, `synchronized_validation_inputs`, `planning_result`, `algorithm_diagnostics` |
| 地形与机器人通行性 | `terrain_analyzer`, `traversability_evaluator`, `step_traversal_rules` |
| 缆线状态、模型与约束 | `cable_state_tracker`, `cable_model`, `cable_laying_evaluator`, `cable_corridor_evaluator`, `cable_uncertainty_envelope_builder`, `cable_uncertainty_envelope_manager`, `reference_progress_tracker` |
| 搜索、平滑与候选复检 | `merge_goal_generator`, `hybrid_astar_planner`, `path_smoother`, `path_candidate_verifier`, `trajectory_parameterizer`, `timed_cable_candidate_verifier` |
| 稳定性、授权与安全 | `plan_validity_evaluator`, `execution_lease_monitor`, `stability_manager`, `commitment_safety` |
| 协调、状态与主循环 | `scout_coordinator`, `planning_state_machine`, `message_consistency`, `communication_recovery`, `main_planning_loop` |

当前 CTest 清单如下：

| 验证域 | CTest 名称 |
|---|---|
| 基础契约与输入 | `synthetic_fixtures`, `data_contract`, `parameter_config`, `versioned_snapshots`, `synchronized_validation_inputs` |
| 地形与通行性 | `terrain_analyzer`, `step_geometry`, `traversability_evaluator`, `directional_slope`, `step_traversability`, `level1_terrain_traversability_matrix` |
| 缆线、搜索、平滑与时序 | `cable_state_tracker`, `cable_model`, `cable_laying_evaluator`, `cable_corridor_evaluator`, `cable_uncertainty_envelope_builder`, `cable_uncertainty_envelope_manager`, `reference_progress_tracker`, `merge_goal_generator`, `hybrid_astar_planner`, `path_smoother`, `path_candidate_verifier`, `trajectory_parameterizer`, `timed_cable_candidate_verifier`, `level1_search_cable_matrix` |
| 结果、复用、授权与状态 | `planning_result`, `plan_validity_evaluator`, `execution_lease_monitor`, `stability_manager`, `commitment_safety`, `scout_coordinator`, `planning_state_machine`, `main_planning_loop`, `level1_smoothing_timing_stability_matrix` |
| 一致性、诊断与闭环 | `message_consistency`, `algorithm_diagnostics`, `level1_closed_loop_scenarios`, `level1_closed_loop_report_json` |

T47 的确定性闭环报告 `build/verify/t47_closed_loop_report.json` 使用 `closed-loop-regression/v1` schema、固定种子 `47006`、SI 单位、运行域 `competition-level1/v1` 和风险语义 `POINTWISE_ONLY:NO_PATH_JOINT_RISK_GUARANTEE`。报告覆盖 8 个主场景、4 个持续故障注入运行和 18 个周期，记录 12 个授权周期、6 次安全停车及 5 次旧租约拒绝证明；SHA-256 为 `DBCF77C742B00FEA4F085D1B2236C5FAEC42D46BBA24806242208E1EAC1A9879`。该报告证明的是固定基线上的 Level 1 确定性算法闭环，不证明外部仿真、真实传感器、底层控制、生产频率或实物布放性能。

### 1.6 外部门禁与生产边界

| 计划项 | 当前状态 | 正式能力声明所需证据 |
|---|---|---|
| T48 DAVE/Gazebo 双机算法闭环 | `blocked-external` | 外部仿真器和机器人接口中的补探、地图更新、重规划与距离约束闭环 |
| T49 长时与通信异常仿真 | `blocked-external` | 代表性 1-2 km、30-60 分钟运行及通信中断/乱序下的资源和安全记录 |
| T50 地形梯度风险策略标定 | `blocked-external` | 独立地形真值数据上的覆盖率、策略版本、数据集和运行域证据 |
| T51 缆线模型与横向统计包络标定 | `blocked-external` | 独立缆线真值、有效运行包络和对抗轨迹审计 |
| T52 机器人能力与执行租约阈值标定 | `blocked-external` | 实机几何、运动、制动、放缆/张力跟踪和撤租阈值数据 |
| T53 水池短距离算法验证 | `blocked-external` | 机器人、缆线和真值系统同步记录的水池试验报告 |
| T54 海试与甲方指标验收 | `blocked-external` | 真实海床条件下的落点精度、走廊符合率、连续铺设距离和验收记录 |

第 15.2 节生产参数门禁和第 20 节待标定清单继续完整生效。在 T48-T54 及其生产就绪审计依赖完成前，系统只能使用明确标记的非生产能力配置开展算法验证；不得签发或宣传生产能力结论。

### 1.7 D10 双向发布审计

D10 以 `bd102b44ad217a42fd1dda815f61599b7dc10b79` 为固定代码点执行发布审计。正向审计从设计条款追到公共契约、生产实现和可执行证据；反向审计从 32 个生产公共头文件、关键状态和安全失败路径追回本文的规范章节。32/32 个 `core/*.hpp` 均列入第 14.2 节，34 个生产 `.cpp` 均由 `underwater_planner_core` 构建，38/38 个 CTest 名称均在第 1.5 或第 18 节形成证据入口。source-only helper 与 `underwater_planner::test_support` 保持非生产边界。

**设计到实现与测试**：

| 设计域 | 公共契约/生产所有者 | 可执行证据 |
|---|---|---|
| 第 1-4、13、14.3 节：范围、状态、单位、快照和数据一致性 | `data_contract`、`parameter_config`、`versioned_snapshot`、`synchronized_validation_inputs`、`planning_result` | `data_contract`、`parameter_config`、`versioned_snapshots`、`synchronized_validation_inputs`、`planning_result` |
| 第 5 节：地形与机器人通行性 | `terrain_analyzer`、`traversability_evaluator`、`step_traversal_rules` | 地形/台阶/通行性五个模块测试与 `level1_terrain_traversability_matrix` |
| 第 6-7 节：缆线状态、模型、机械和走廊风险 | `cable_state_tracker`、`reference_progress_tracker`、`cable_model`、`cable_laying_evaluator`、`cable_corridor_evaluator`、`cable_uncertainty_envelope_builder`、`cable_uncertainty_envelope_manager` | 八个缆线公共接口测试与 `level1_search_cable_matrix` |
| 第 8 节：并线目标与增广 Hybrid A* | `merge_goal_generator`、`hybrid_astar_planner` | `merge_goal_generator`、`hybrid_astar_planner`、`level1_search_cable_matrix` |
| 第 9 节：平滑、几何复检与时序化 | `path_smoother`、`path_candidate_verifier`、`trajectory_parameterizer`、`timed_cable_candidate_verifier` | 四个同名公共接口测试与 `level1_smoothing_timing_stability_matrix` |
| 第 10 节：复检、滞回、承诺安全和执行授权 | `plan_validity_evaluator`、`stability_manager`、`commitment_safety`、`execution_lease_monitor` | 四个同名测试、`planning_result`、`main_planning_loop` 与稳定性矩阵 |
| 第 11-12、13.6-13.7 节：补探、状态机、消息与恢复 | `scout_coordinator`、`planning_state_machine`、`message_consistency`、`communication_recovery` | `scout_coordinator`、`planning_state_machine`、`message_consistency`、`main_planning_loop`、Level 1 闭环 |
| 第 14-17 节：模块编排、参数、主循环与资源边界 | `main_planning_loop`、`algorithm_diagnostics` 及第 14.2 节完整公共接口集 | `main_planning_loop`、`algorithm_diagnostics`、四个 Level 1 矩阵/闭环入口 |
| 第 18-22 节：验证、限制、标定与追踪 | CMake/CTest 清单、确定性报告、`project.md/PLAN.md` | 统一 `tools/verify.ps1`；T48-T56 未完成项按第 1.6、18-20 节和 PLAN 保持可见 |

**公共实现到设计**：第 14.2 节逐文件拥有全部 32 个生产公共头文件；第 14.3 节拥有跨模块值类型；各模块的算法和门禁分别回指第 5-13 节；`MainPlanningLoop`、状态机和诊断回指第 16-17 节；对应测试回指第 18.2 节。没有公共头文件只存在于代码而缺少设计所有权，也没有设计中的生产模块脱离 CMake 构建或公共 seam。

**关键状态与安全失败路径**：

| 状态/失败路径 | 规范章节 | 失败关闭与证据 |
|---|---|---|
| `PlanningState`、SI 单位、单调时间和不可变结果 | 3.3、13.3-13.5、14.3 | 无效/非有限/回退输入拒绝；契约、快照、同步输入和发布测试 |
| 地形无效、粗糙度/坡度/台阶/碰撞越限 | 5.3-5.7、16.3 | `TERRAIN_INVALID`/`INPUT_INVALID` 或整段不可通行；18.2.1-18.2.2 矩阵 |
| 缆线模型无效、走廊违规和包络 breach | 6.6、7.4-7.7、10.5、16.6 | 普通候选拒绝或 `COVARIANCE_ENVELOPE_BREACH` 撤租停车；18.2.4 与闭环测试 |
| 搜索无解、包络下无解和三类资源耗尽 | 8.3-8.8、16.6、17.1 | 分别映射 `NO_SOLUTION`、`NO_SOLUTION_UNDER_COVARIANCE_ENVELOPE`、`TIMEOUT`；18.2.3 |
| 平滑、G2、时序和停车约束失败 | 9.1-9.7、16.3、16.6 | 无部分轨迹发布；原始路径也必须走同一完整复检；18.2.5-18.2.6 |
| 最新上下文复检、滞回、租约和执行偏差失败 | 10.2-10.5、13.5、16.4-16.6 | keep/switch 前成对复检，任何授权矛盾撤租并受控停车；18.2.7 |
| 补探阻塞、通信降级/恢复和状态回退 | 11-12、13.6-13.7、16.1 | 谨慎剖面必须获新租约，恢复需新同步证据；模块测试与 Level 1 闭环 |

审计没有把 Level 1 证据外推为生产能力。T48-T54 仍为 `blocked-external`，T55 为 `ready-for-agent`，T56 受 T55 阻塞，T57 继续受 T50-T52 的生产标定依赖阻塞；`epsilon_path`、误差相关长度、整路径联合地形风险和高保真柔性缆线动力学仍未实现。

---

## 2. 问题定义

### 2.1 核心规划问题

给定：
- 全局期望缆线落点参考路线 $\mathcal{R}_{\text{ref}}$
- 主机器人当前状态 $\mathbf{x}_{\text{robot}} = (x, y, \theta, v, \kappa)$
- 主机器人位姿与控制跟踪协方差 $\Sigma_{\text{robot}}$
- 当前缆线状态估计 $\mathbf{x}_{\text{cable}} = (\delta, \Sigma_{\delta})$
- 放缆控制状态（出缆速度、机器人对地速度、张力）
- 局部 2.5D 高程地图 $\mathcal{M}(x, y) = \{h, \sigma_h, \text{confidence}, \text{timestamp}\}$
- 机器人运动学约束 $\mathcal{C}_{\text{kin}}$（最小转弯半径、曲率限制）
- 地形能力约束 $\mathcal{C}_{\text{terrain}}$（最大坡度、台阶高度）
- 缆线布放模型参数 $\mathcal{P}_{\text{cable}}$
- 机器人允许作业区域 $\mathcal{W}_{\text{robot}}$
- 缆线允许施工走廊 $\mathcal{W}_{\text{cable}}$
- 单位置/相关区段风险策略 $\epsilon_{\text{point}}$（TBD，真实施工前必须确定）

求解：
- 主机器人可跟踪路径 $\mathcal{P}_{\text{robot}} = \{\mathbf{p}_i = (x_i, y_i, \theta_i)\}_{i=1}^N$
- 预测缆线落点路径 $\mathcal{P}_{\text{cable}} = \{\mathbf{c}_j = (x_j^c, y_j^c)\}_{j=1}^M$
- 终端缆线状态与模型有效性
- 规划状态 $\mathcal{S} \in \{\text{SUCCESS}, \text{NO\_SOLUTION}, \text{WAITING\_MAP}, \ldots\}$
- 前置机器人补探目标 $\mathcal{T}_{\text{scout}}$（可选）

使得：
1. $\mathcal{P}_{\text{robot}}$ 位于 $\mathcal{W}_{\text{robot}}$ 并满足所有硬约束 $\mathcal{C}_{\text{kin}}$, $\mathcal{C}_{\text{terrain}}$, $\mathcal{C}_{\text{safety}}$
2. $\mathcal{P}_{\text{cable}}$ 位于缆线走廊 $\mathcal{W}_{\text{cable}}$ 内
3. $\mathcal{P}_{\text{cable}}$ 与 $\mathcal{R}_{\text{ref}}$ 偏差最小化
4. 路径平滑、曲率连续、计算实时

其中第1、2项由硬约束、直接剪枝和验证门禁保证；在满足全部硬约束的可行域内，第3、4项通过现有加权软代价共同优化，以缆线落点质量为主要目标，并兼顾路线效率、曲率平滑性与地形适宜性。该表述不表示采用严格字典序优化。

### 2.2 关键特征

**与传统路径规划的区别**：
1. 缆线落点质量是主要软优化目标，而不是仅优化机器人中心轨迹
2. 机器人路径既是实现落点目标的手段，其长度、曲率和平滑性也参与加权软代价
3. 需要同时满足机器人通行性和缆线布放适宜性

**与完全未知探索的区别**：
1. 存在前期规划的参考路线，不是自由探索
2. 局部修正而非全局重新规划
3. 参考路线既是软约束引导，也是硬约束边界

---

## 3. 坐标系与符号约定

### 3.1 坐标系

- **世界坐标系** $\{W\}$：东北天（ENU）或任务定义的固定坐标系
- **机器人坐标系** $\{R\}$：原点位于机器人几何中心，$x$ 轴指向前进方向
- **地图坐标系** $\{M\}$：与世界坐标系对齐的栅格地图坐标系

### 3.2 核心符号

| 符号 | 含义 | 单位 |
|------|------|------|
| $(x, y, \theta)$ | 机器人位姿 | m, m, rad |
| $\kappa$ | 当前路径曲率 | m$^{-1}$ |
| $\rho_{\min}$ | 最小转弯半径 | m (TBD) |
| $\kappa_{\max}$ | 最大曲率 $= 1/\rho_{\min}$ | m$^{-1}$ |
| $u_{\max}$ | 最大空间曲率变化率 $|d\kappa/ds|_{\max}$ | m$^{-2}$ (TBD) |
| $\alpha_{\max}^{\text{up}}$ | 最大纵向上坡角 | rad (TBD) |
| $\alpha_{\max}^{\text{down}}$ | 最大纵向下坡角 | rad (TBD) |
| $\alpha_{\max}^{\text{lateral}}$ | 最大横向坡度角（侧翻限制） | rad (TBD) |
| $\alpha_{\max}^{\text{roll}}$ | 最大履带支撑滚转角 | rad (TBD) |
| $h_{\max}^{\text{climb}}$ | 最大向上爬台阶高度 | m (TBD) |
| $h_{\max}^{\text{drop}}$ | 最大向下落差 | m (TBD) |
| $B_{\text{eff}}$ | 左右履带有效支撑中心距 | m (TBD) |
| $d_{\text{safe}}$ | 基础安全距离 | m (TBD) |
| $d_{\text{margin,robot}}$ | 机器人碰撞不确定性裕量 | m (函数) |
| $d_{\text{upper}}$ | 缆线横向误差的保守机会约束替代界 | m |
| $\mathcal{W}_{\text{robot}}$ | 机器人本体允许进入的作业区域 | 区域集合 |
| $\mathcal{W}_{\text{cable}}$ | 预测缆线触地点允许进入的施工走廊 | 区域集合 |
| $w_{\text{corridor}}^{\text{nominal}}$ | 期望走廊半宽（软约束） | m (TBD) |
| $w_{\text{corridor}}^{\text{max}}$ | 允许走廊半宽上限（硬约束） | m (TBD) |
| $L_{\text{td}}$ | 放缆点到触地点的等效水平距离 | m (TBD) |
| $L_{\psi}$ | 缆线方向响应长度 | m (TBD) |
| $\psi_c$ | 触地点到放缆点的缆线水平朝向 | rad |
| $\delta$ | 缆线方向相对机器人航向的滞后角 | rad |
| $\mathbf{p}_r=(x_r,y_r)$ | 放缆点相对机器人本体的偏置 | (m, m) (TBD) |
| $\mathbf{p}_{\text{release}}(s)$ | 世界坐标系中的放缆点位置 | (m, m) |

**TBD**: To Be Determined，待实测标定

### 3.3 公共契约的单位、时间与状态约定

本节只规定跨模块共享的运行时语义；数学章节中的简写不得生成另一套近似数据结构。当前唯一公共定义位于 `src/underwater_planner/include/underwater_planner/core/data_contract.hpp`，快照、同步捕获、发布和参数边界分别由同目录的 `versioned_snapshot.hpp`、`synchronized_validation_inputs.hpp`、`planning_result.hpp` 与 `parameter_config.hpp` 扩展。

- 所有公共物理量使用 SI，字段后缀是契约的一部分：`_m`、`_m2`、`_rad`、`_rad2`、`_s`、`_mps`、`_mps2`、`_per_m`、`_per_m2` 和 `_n` 不得省略或隐式换算。
- `MonotonicTime.nanoseconds` 是应用单调时钟的纳秒计数，不是 UTC、ROS wall time 或可回拨的系统时间；`Duration.nanoseconds` 是持续时间。默认值 `-1` 表示无效，不能解释为零时刻。
- 角度进入公共契约前规范化到 $[-\pi,\pi)$；非有限角度拒绝而不是静默归一化。全部序列化数值即使属于失败结果也必须有限，协方差必须对称且半正定。
- 默认构造的状态、路径、误差预算和规划结果只是装配占位，不是有效输入或安全结果；只有相应 `validate(...)` 成功后才能跨公共边界。
- 公共 `PlanningState` 的稳定集合为 `SUCCESS`、`PATH_VALID`、`WAITING_MAP`、`REQUEST_SCOUT`、`NO_SOLUTION`、`NO_SOLUTION_UNDER_COVARIANCE_ENVELOPE`、`COVARIANCE_ENVELOPE_BREACH`、`INPUT_INVALID`、`MAP_EXPIRED`、`TIMEOUT`、`COMMUNICATION_DEGRADED`、`MANUAL_OVERRIDE`、`INIT`、`NORMAL_PLANNING`、`PLANNING_WITH_CAUTION` 和 `EMERGENCY_STOP`。模块内部的 `BOUNDARY_STATE_INVALID`、`VALIDATION_CONTEXT_INVALID`、`DEADLINE_EXCEEDED` 等是阶段状态或结构化原因，不得伪装成新的 `PlanningState` 枚举值。

---

## 4. 总体算法架构

### 4.1 架构概览

系统采用**分层滚动规划 + 主动补探协同**架构：

```
┌─────────────────────────────────────────────────────────────┐
│                  全局参考路线管理                             │
│  - 期望缆线落点路线 R_ref                                     │
│  - 允许走廊边界                                               │
│  - 路段属性和约束                                             │
└────────────────────┬────────────────────────────────────────┘
                     │
┌────────────────────▼────────────────────────────────────────┐
│               局部地图与派生图层                              │
│  输入: 2.5D 高程地图 (h, σ_h, confidence, timestamp)         │
│  派生: 坡度、台阶、粗糙度、障碍、通行性、落点适宜性           │
└────────────────────┬────────────────────────────────────────┘
                     │
┌────────────────────▼────────────────────────────────────────┐
│          滚动局部路径规划 (核心)                              │
│  1. 窗口提取: 提取前方有限距离的参考线段和地图窗口            │
│  2. 通行性评价: 硬约束检查 (坡度/台阶/障碍/未知区域)          │
│     - 方向相关: 纵向/横向坡度分别检查                         │
│  3. Hybrid A* 搜索: 参考线引导 + 曲率约束                    │
│     - 增广缆线滞后角状态，逐运动原语传播触地点                  │
│  4. 多候选并线: 生成并评价多个回归接入点                      │
│  5. 路径平滑: 曲率受限平滑 + 平滑后复检                       │
│  6. 缆线落点预测: 同一触地点模型高精度重积分                   │
│  7. 落点约束验证: 分级检查（PASS/MARGINAL/VIOLATION）         │
│     - 考虑触地点协方差和模型有效性                            │
│  8. 稳定性判断: 路径切换滞回 + 近端承诺段                     │
│     - 安全事件可打破承诺段                                    │
└────────┬───────────────────────────────────┬────────────────┘
         │                                   │
         │ 无解/信息不足                      │ 成功
         ▼                                   ▼
┌────────────────────────┐        ┌──────────────────────────┐
│   主动补探协调(改进)    │        │   规划结果输出            │
│ - 信息缺口识别          │        │ - 机器人路径              │
│ - 缺口紧迫度评估        │        │ - 预测缆线路径            │
│ - 分级响应策略:         │        │ - 误差预算                │
│   ├ BLOCKING: 停车等待  │        │ - 地图版本/时间戳         │
│   ├ URGENT: 减速继续    │        │ - 结构化状态              │
│   └ SCHEDULED: 后台请求 │        └──────────────────────────┘
└────────┬───────────────┘
         │                         └──────────────────────────┘
         │ 等待新地图
         ▼
┌────────────────────────────────────────────────────────────┐
│              状态机与重规划触发                               │
│  - 周期触发 (目标 2-5 Hz；外部目标平台尚未验证)               │
│  - 事件触发 (新地图/路线失效/通信异常)                        │
│  - 降级策略 (减速/停车/人工接管)                              │
└─────────────────────────────────────────────────────────────┘
```

### 4.2 核心模块

| 模块 | 职责 | 关键算法 |
|------|------|----------|
| **SnapshotManager / SynchronizedValidationInputCapturer** | 接受版本化地图、参考线和双空间域，并冻结原子复检输入 | 重复/乱序/过期/回退拒绝、revision 稳定性、年龄/同步/序列门禁 |
| **TerrainAnalyzer / IncrementalTerrainAnalyzer** | 从高程地图派生版本化、方向无关的表面/台阶/缆线地形层 | 鲁棒平面、梯度协方差、去趋势粗糙度、台阶几何、影响区增量更新 |
| **TraversabilityEvaluator** | 评价机器人通行性 | 碰撞分类与膨胀、完整足迹扫掠、梯度不确定性边界、方向相关台阶与履带支撑评价 |
| **CableLayingEvaluator** | 评价缆线机械可行性与落点适宜性 | 最大曲率/禁放区/悬空代理硬约束，偏好曲率/粗糙度软代价 |
| **MergeGoalGenerator** | 生成并线目标 | 沿锁定参考线组合并线距离、航向偏置和滞后角，经 `CableModel` 反解并用完整终端足迹与地形门禁筛选 |
| **HybridAStarPlanner** | 核心搜索算法 | 五元基础键、机械历史多标签、路径无关统计包络、完整原语硬门禁、前进 Dubins 启发与周期解析扩展 |
| **PathSmoother** | 生成并内部审计曲率受限几何候选 | 等长分段 clothoid、G2 边界/残差、拓扑管与阶段截止时间 |
| **PathCandidateVerifier** | 对完整机器人路径进行独立几何与当前地图复检 | 三点有符号曲率、G2 拼接、作业区、完整足迹碰撞与方向地形扫掠 |
| **TrajectoryParameterizer** | 将已验证几何路径转换为可执行时序轨迹 | 速度/加速度/横向加速度/停车距离与放缆控制联合约束 |
| **TimedCableCandidateVerifier** | 对完整时序候选执行缆线高精度复检 | 实际状态重预测、路径协方差、机械/走廊/包络审计和减速后新版本复检 |
| **CableStateTracker** | 当前缆线状态估计 | 基于实际执行轨迹与放缆遥测持续传播 $\delta$ |
| **ReferenceProgressTracker** | 当前任务参考线进度估计 | 只用已执行铺设和局部参考关联持续传播 $s_{\text{prog}}$ |
| **CableModel** | 缆线触地点预测 | 从状态快照进行纯预测、轨迹输出与有效性判定 |
| **CableUncertaintyEnvelopeBuilder** | 离线/受控生成横向不确定性包络 | 在版本化 $\Gamma_H$ 内执行有界可达性和上界验证 |
| **CableUncertaintyEnvelopeManager** | 运行时锁定与失效管理 | 按参考线、传感器模式、运行域、缆线模型和执行运行包络的完整版本元组提供已验证包络 |
| **CableCorridorEvaluator** | 缆线走廊风险评价 | 横向协方差投影、保守机会约束和分级统计 |
| **StabilityManager** | 路径稳定性管理 | 滞回、承诺段 |
| **CommitmentSafetyEvaluator / Supervisor** | 承诺段安全覆盖与独立安全通道 | 完整机器人/缆线证据聚合、STOP 优先级、认证停车距离、异步撤租 |
| **PlanValidityEvaluator** | 评价剩余路径能否继续执行 | 当前状态重预测、全硬约束复检、验证租约 |
| **ExecutionLeaseMonitor** | 监控获批剖面执行偏差 | 版本配对、速度/出缆/张力/加速度阈值与异步撤租 |
| **ScoutCoordinator** | 前置机器人协调 | 补探目标生成 |
| **PlanningStateMachine** | 状态机与降级 | 事件触发、超时处理 |
| **MessageConsistencyGate / CommunicationRecoveryGate** | 消息流一致性和通信恢复授权 | 类型化去重、乱序缓冲、水位、恢复后完整上下文和新租约门禁 |
| **MainPlanningLoop** | 单周期生产编排与原子授权发布 | 锁定输入、候选链、最新上下文成对复检、滞回决策、失败收敛和撤租停车 |
| **AlgorithmDiagnosticsRecorder / Replayer** | 记录、汇总和离线重放算法证据 | 强类型参数/版本/耗时/内存/风险记录、P50/P95/P99 和逐字段复现 |

### 4.3 数据流

```
地图输入 → 版本锁定 → 快照冻结
                ↓
        地形图层派生（当前串行，可增量）
    ┌──────┬──────┬──────┬──────┐
  坡度   台阶  粗糙度 障碍  未知
    └──────┴──────┴──────┴──────┘
                ↓
   碰撞层/方向通行性与落点适宜性分别评价
                ↓
        Hybrid A* 搜索
                ↓
        路径平滑与 G2 复检
                ↓
        生成版本化时序执行剖面
                ↓
        按执行剖面预测缆线
                ↓
        按固定顺序执行完整约束复检
    ┌──────┬──────┬──────┐
  碰撞   曲率  落点走廊
    └──────┴──────┴──────┘
                ↓
   最新上下文成对复检与稳定性判断
                ↓
      新租约原子发布 / 撤租停车
```

补探请求、类型化消息门禁、通信恢复和状态机位于该单周期之外的编排层；它们不能绕过 `MainPlanningLoop` 的同步输入、完整复检和租约发布边界。当前实现没有地形/约束并行执行器，图中的分支表示证据类别，不表示线程并行。

---

## 5. 地形分析与通行性评价

本章描述当前实现的三段式公开边界，而不是一个预先压缩的“通行性布尔图”：

1. `TerrainAnalyzer` / `IncrementalTerrainAnalyzer` 从版本化 `MapSnapshot` 生成方向无关且只读共享的 `TerrainLayers`。
2. `TraversabilityEvaluator::evaluate_collision_layer` 生成带机器人碰撞误差预算的栅格分类，`evaluate_collision_sweep` 对完整复杂足迹作自适应连续扫掠。
3. `TraversabilityEvaluator::evaluate` 在同一地形版本和标定策略上检查方向坡度、台阶穿越及左右履带支撑。

调用方必须同时取得第 2、3 项的通过结果，才能把运动原语视为机器人可通行；任一阶段的输入、协方差或地形状态无效都失败关闭。`evaluate_collision_layer` 和方向门禁执行本章所列完整版本/运行域检查；`evaluate_collision_sweep` 使用完整 `MapVersion` 与地形分析配置版本绑定碰撞层和地形层。当前没有一个可绕过这些阶段的方向无关最终通行性栅格。

### 5.1 局部地形平面与坡度估计

**目的**：从含噪高程地图估计与机器人支撑尺度一致的局部地形梯度，并同时获得粗糙度和估计质量。

**输入**：
- 高程地图 $h(x, y)$ 及单元有效性、测量方差和时间戳
- 地图分辨率 $\Delta r$ (TBD)
- 以物理尺寸定义的拟合窗口 $W_{\text{surface}}$，尺度应与履带接地区域匹配

**鲁棒局部平面拟合**：

对每个有效栅格中心 $(x_0,y_0)$，拟合：

$$
\hat h(x,y)=a(x-x_0)+b(y-y_0)+c
$$

$$
(a,b,c)=\arg\min_{a,b,c}
\sum_{i\in W_{\text{surface}}}w_i\,
\rho\!\left(h_i-\hat h(x_i,y_i)\right)
$$

当前 `TerrainAnalyzer` 使用直径为 `surface_window_size_m` 的圆形物理窗口。基础权重为单元置信度、关于窗口半径的高斯距离权重、测量方差倒数和相对窗口内最新样本的指数时间衰减之积；随后执行至少两轮、至多 `maximum_irls_iterations` 轮 Huber IRLS。配置不能关闭鲁棒重加权。中心单元本身不可用时，即使邻域有足够样本也不插值成有效表面。

**方向无关输出**：

$$
\mathbf g=\nabla h=(a,b),
\qquad
\alpha=\arctan(\|\mathbf g\|)
$$

`SurfaceEstimate` 当前输出：

- 局部拟合高程 $c$、梯度 $(a,b)$ 和方向无关坡度角
- 梯度协方差 $\Sigma_g$
- 去趋势残差粗糙度 $\sigma_{\text{rough}}$
- 未加权绝对残差 P95
- 有效支撑覆盖率 $r_{\text{support}}$
- 拟合状态 `VALID / INSUFFICIENT_SUPPORT / ILL_CONDITIONED / INVALID_COVARIANCE / DISCONTINUOUS`

协方差取最终加权法方程逆矩阵的梯度子块，并显式检查有限性、对称性和半正定性。有效覆盖不足返回 `INSUFFICIENT_SUPPORT`，法方程不可解返回 `ILL_CONDITIONED`，IRLS 未收敛或协方差不可表示返回 `INVALID_COVARIANCE`；这些状态都不得退回零坡度或仅用点估计。`DISCONTINUOUS` 不是拟合成功的别名，而是在台阶几何确认后标记其过渡带，后续只有与已识别台阶相交的方向门禁能够解释该状态。中心差分仅是测试比较概念，不是当前实现路径。

### 5.2 台阶检测

**目的**：提取与机器人航向无关的高程不连续边缘，供后续完整足迹评价。

当前台阶检测先对可用高程作局部中值滤波，在栅格横纵相邻对之间生成观测突变或未知边界候选，再按空间连通性和候选类型组成边缘分量。每个分量以一致的低到高法向建立切向范围，并在过渡带外分别拟合两侧支撑平面；台阶高度是在边缘中心处评价两平面的完整垂直差，不得根据任何机器人航向缩放。

每个台阶输出：

```cpp
struct StepEdge {
    Polyline2D extent;
    Vector2d normal_low_to_high;
    double height_m;
    double transition_width_m;
    double confidence;
};

struct StepEstimate {
    StepEdge edge;
    StepEstimateStatus status;
};
```

台阶成立至少要求：

- 两侧均有足够有效支撑点
- 两侧拟合面高度差 $h_{\text{step}} \geq h_{\text{step}}^{\min}$
- 边缘法向和范围可稳定估计
- 高度差超过局部测量噪声的置信阈值

`status == VALID` 时，`extent` 是沿边缘切向的两个不同端点，`normal_low_to_high` 是从低侧指向高侧的单位法向，`height_m` 为正的完整高度，`transition_width_m` 非负且 `confidence` 位于 $(0,1]$。`StepLayer` 同时保留被拒绝的估计；早退状态允许 `StepEdge` 只部分填充或保持默认空范围/零值，调用方不得消费其几何。被拒绝状态区分双侧支撑不足、噪声不显著、低于最小高度、法向不稳定、范围不足、重复端点和低置信度，不能把失败候选伪装成“无台阶”。只有 `VALID` 估计会把相交表面单元标记为 `DISCONTINUOUS`。

`TerrainAnalyzer` 只描述台阶几何。向上爬阶、向下落阶、斜向接触或沿边缘骑跨均由 `TraversabilityEvaluator` 根据运动方向和履带支撑区域判断。

对于与高置信台阶相交的拟合窗口，不得用跨越两侧的单一平面代表地形；应保留边缘两侧各自的支撑面，或将跨边缘表面估计标记为不连续。

### 5.3 粗糙度评价

**目的**：量化海床表面粗糙程度，影响机器人稳定性和缆线布放质量。

**方法**：复用第5.1节局部平面拟合的去趋势残差：

$$
\sigma_{\text{rough}}(i,j)=
\sqrt{
\frac{
\sum_{k\in W_{\text{surface}}}w_k
\left(h_k-\hat h(x_k,y_k)\right)^2
}{
\sum_{k\in W_{\text{surface}}}w_k
}}
$$

平滑斜面拟合残差接近零，不会因为坡度或窗口增大被错误判为粗糙。当前同时记录绝对残差 P95；没有实现 MAD 输出。`TraversabilityEvaluator` 在与方向坡度相同的完整自适应扫掠足迹上逐有效表面检查 `robot.maximum_roughness_m`：超过阈值返回 `roughness_exceeded`，并在 `TraversabilityResult.maximum_detrended_roughness_rms_m` 记录最坏值；等值边界通过。

**软代价**：粗糙度作为路径代价的一部分：

$$
c_{\text{rough}}(i, j) = w_{\text{rough}} \cdot \sigma_{\text{rough}}(i, j)
$$

其中 $w_{\text{rough}}$ 为权重 (TBD)。

**硬门禁与失败关闭**：`robot.maximum_roughness_m` 是有限、非负且 production 必填的机器人能力字段，由 `make_robot_capability` 显式装配到 `RobotCapability`。缺失或非有限装配失败；有效表面粗糙度缺失、非有限或为负时返回 `TERRAIN_INVALID` 与 `roughness_invalid`。任一完整足迹样本越限都会拒绝整段。该硬门禁独立于搜索软代价和缆线粗糙度代价；外部机器人能力标定仍属于第20.2节待完成事项。

### 5.4 障碍物与膨胀

**障碍检测**：

当前实现只消费 `MapSnapshot.cells[].obstacle` 及其可选单位法向，不从高程突变自动推导障碍标记。高程不连续由第5.2节的台阶层和方向门禁独立处理；原始传感器/地图构建侧如何生成障碍标记不在本算法范围内。

**误差预算分离**：

机器人碰撞只使用机器人相对障碍物的位置协方差 $\Sigma_{\text{robot-rel}}$，包括定位、控制跟踪和障碍地图配准误差。缆线模型协方差 $\Sigma_c$ 不得进入机器人障碍膨胀。

若碰撞检查可获得局部障碍法向 $\mathbf n_{\text{obs}}$，方向裕量为：

$$
d_{\text{margin,robot}}=
z_{1-\epsilon_{\text{robot}}}
\sqrt{
\mathbf n_{\text{obs}}^T
\Sigma_{\text{robot-rel}}
\mathbf n_{\text{obs}}
}
$$

若栅格形态学膨胀只能使用单一半径，则采用保守各向同性上界：

$$
d_{\text{margin,robot}}=
z_{1-\epsilon_{\text{robot}}}
\sqrt{\lambda_{\max}(\Sigma_{\text{robot-rel}})}
$$

$$
d_{\text{total,robot}}=d_{\text{safe}}+d_{\text{margin,robot}}
$$

$\epsilon_{\text{robot}}$ 是机器人碰撞风险参数，与缆线走廊风险和任务布放成功率均不同。它是 production 必填且必须独立标定的策略字段；相应外部标定门禁完成前不能把示例值视为生产保证。

**当前碰撞层行为**：`evaluate_collision_layer` 只接受与 `TerrainLayers` 同一地图版本、地形配置版本和运行域的地图与策略。策略版本、标定数据集、`epsilon_robot`、最低地图置信度和安全距离必须有效；机器人相对障碍物协方差必须有限、对称且半正定。局部障碍法向有效时使用方向方差，否则使用最大特征值的各向同性上界。实现按栅格中心的欧氏距离直接膨胀障碍和地图边界，不声称使用距离变换。

输出分类区分 `TRAVERSABLE / STEP_DISCONTINUITY_REQUIRES_VALIDATION / OBSTACLE / INFLATED_OBSTACLE / UNKNOWN / LOW_CONFIDENCE / INVALID_TERRAIN / MAP_BOUNDARY / INPUT_INVALID`。原始障碍不会被其他膨胀区覆盖；台阶不连续单元也不直接视为可通行，只能作为候选进入后续方向相关台阶门禁。结果审计记录地图版本、地形配置版本、碰撞策略版本、`epsilon_robot`、运行域、标定数据集和 `robot-relative-obstacle-pointwise-only` 风险语义；当前不提供整段路径联合碰撞概率保证。

### 5.5 未知与低置信度区域

**保守禁行原则**：

$$
\text{traversable}_{\text{known}}(i, j) = 
\begin{cases}
\text{false} & \text{if } \text{confidence}(i, j) < \text{confidence}_{\min} \\
\text{false} & \text{if } \text{unknown}(i, j) = \text{true} \\
\text{true} & \text{otherwise}
\end{cases}
$$

其中 $\text{confidence}_{\min}$ 对应 `RobotCollisionRiskPolicy.minimum_map_confidence`。它必须大于0且不超过1，并与碰撞策略版本、标定数据集和运行域一起锁定；相应外部标定门禁完成前仍是待标定能力。

**信息缺口识别**：

碰撞派生层把未知、低置信度和无效表面分别标为不可通行，并保留带栅格位置和原因的 `InformationGap`。把这些缺口关联到参考线、排序并触发补探属于第11节的后续职责；地形/碰撞层本身不绕过禁行结论。

### 5.6 机器人通行性综合评价

通行性是具体机器人的运动段属性，不再预先压缩成方向无关的单一布尔栅格。调用方对 `MotionSegment` 依次执行碰撞层扫掠和完整地形门禁：

1. `evaluate_collision_sweep`：完整外轮廓对障碍、膨胀区、地图边界、未知和低置信度分类的连续扫掠。
2. `evaluate`：足迹覆盖区表面有效性、纵向上坡/下坡和横向坡度、台阶边缘相交、完整爬阶/落阶高度以及左右履带支撑。

**完整足迹与履带支撑区**：

$$
\text{Footprint}(x, y, \theta) = \{ (x', y') \mid (x', y') \in \text{RobotShape} \text{ transformed to } (x, y, \theta) \}
$$

$$
\text{CollisionFree}(x, y, \theta) = \bigwedge_{(x', y') \in \text{Footprint}(x, y, \theta)} \text{traversable}_{\text{robot}}(x', y')
$$

除外轮廓外，必须分别配置左、右履带的有效支撑多边形 $F_L,F_R$。机器人几何未定型前保留为 TBD，不使用整车矩形均值替代履带支撑。

最终硬约束融合为：

$$
\text{Traversable}(\text{segment})=
\text{CollisionLayerValid}
\land \text{CollisionSweepFree}
\land \text{DataValid}
\land \text{SlopeValid}
\land \text{RoughnessValid}
\land \text{StepValid}
\land \text{TrackSupportValid}
$$

碰撞层只接受 `collision_candidate()` 的单元参与扫掠：普通 `TRAVERSABLE` 可继续，`STEP_DISCONTINUITY_REQUIRES_VALIDATION` 必须再通过方向门禁，其余分类均失败。完整足迹在相邻输入位姿之间按平移加“足迹半径 × 航向变化”的边界位移自适应细分，并用半个最大采样间距作为保守扫掠裕量。

`evaluate_collision_sweep` 在足迹扫掠前使用完整 `MapVersion` 相等比较绑定 `CollisionLayerResult.source_map_version` 与 `TerrainLayers.source_map_version`，比较范围包含 `map_id`、`sequence_number`、`timestamp` 和 `coordinate_frame`，并同时要求地形分析配置版本一致。任一字段错配都返回 `INPUT_INVALID`，保持 `collision_free=false`、扫掠计数为零和有限的数值诊断，不执行足迹扫掠。

粗糙度门禁属于 `DataValid` 的机器人能力子约束：完整自适应扫掠足迹上的每个有效 `SurfaceEstimate` 都必须提供有限、非负的 `detrended_roughness_rms_m`，并满足 `<= RobotCapability.maximum_roughness_m`。结果保留最坏粗糙度和独立限制因素；缺失或非有限值失败关闭为 `TERRAIN_INVALID`，不能降级为零值或软代价。

### 5.7 方向相关的通行性评价

**职责划分**：
- **TerrainAnalyzer**：输出方向无关的局部平面、梯度、去趋势粗糙度、台阶几何和估计质量
- **TraversabilityEvaluator**：结合机器人航向、完整足迹、履带支撑区和能力参数作出判定

**问题背景**：

履带式海床机器人面对同一坡面时:
- **纵向爬坡**：主要受爬坡能力限制
- **横向切过**：主要受侧倾/侧翻风险限制

使用方向无关的单一坡度阈值可能过于保守(禁止可行的横向通过),或遗漏风险(允许危险的侧翻)。

**坡度方向分解**：

定义机器人前向和左向单位向量：

$$
\mathbf t=(\cos\theta,\sin\theta),
\qquad
\mathbf l=(-\sin\theta,\cos\theta)
$$

将局部平面梯度估计 $\hat{\mathbf g}$ 及其协方差 $\Sigma_g$ 投影到机器人方向：

$$
\mu_{\text{long}}=\hat{\mathbf g}\cdot\mathbf t,
\qquad
\sigma_{\text{long}}=\sqrt{\mathbf t^T\Sigma_g\mathbf t}
$$

$$
\mu_{\text{lat}}=\hat{\mathbf g}\cdot\mathbf l,
\qquad
\sigma_{\text{lat}}=\sqrt{\mathbf l^T\Sigma_g\mathbf l}
$$

`TerrainGradientRiskPolicy` 提供经独立数据验证的局部梯度覆盖系数 $\beta_g$。若采用已验证的二维高斯误差模型，可取 $\beta_g=\sqrt{\chi^2_{2,1-\epsilon_{g,\text{local}}}}$；否则必须使用标定运行域内的经验或确定性覆盖系数，不能仍标注为高斯概率保证。

由同一二维覆盖集合得到方向保守界：

$$
q_{\text{long}}^{\min}=\mu_{\text{long}}-\beta_g\sigma_{\text{long}},
\qquad
q_{\text{long}}^{\max}=\mu_{\text{long}}+\beta_g\sigma_{\text{long}}
$$

$$
q_{|\text{lat}|}^{\max}=|\mu_{\text{lat}}|+\beta_g\sigma_{\text{lat}}
$$

纵向分量保留符号：正值为上坡，负值为下坡。不能先计算总坡度角再乘方向余弦；该近似会在中大坡度下低估分量。$\epsilon_{g,\text{local}}$ 只描述单个局部地形估计覆盖，不得解释为整条路径的联合失效概率。

**方向相关约束检查**：

$$
\text{traversable}_{\text{slope\_directional}}(x,y,\theta) = 
\begin{cases}
\text{false} & \text{if } \arctan(q_{\text{long}}^{\max}) > \alpha_{\max}^{\text{up}} \\
\text{false} & \text{if } \arctan(q_{\text{long}}^{\min}) < -\alpha_{\max}^{\text{down}} \\
\text{false} & \text{if } \arctan(q_{|\text{lat}|}^{\max}) > \alpha_{\max}^{\text{lateral}} \\
\text{false} & \text{if coverage policy or }\Sigma_g\text{ is invalid} \\
\text{true} & \text{otherwise}
\end{cases}
$$

其中 $\alpha_{\max}^{\text{up}}$、$\alpha_{\max}^{\text{down}}$ 和 $\alpha_{\max}^{\text{lateral}}$ 均为独立 TBD 参数，机器人定型前不填入假定数值。运动原语必须在第8.3.1节规定的全部扫掠样本及完整足迹有效表面估计上取这些保守界的最坏值，不能只检查中心栅格。

**台阶方向依赖**：

对于法向由低侧指向高侧的台阶，定义：

$$
\eta=\mathbf t\cdot\mathbf n_{\text{step}}
$$

$\eta$ 只用于判断接近方向和接触过程，**不得用于缩放 $h_{\text{step}}$**。判定顺序如下：

1. 先计算运动原语扫掠足迹与 `StepEdge.extent` 是否相交，以及起止足迹位于边缘哪一侧
2. 从低侧跨到高侧时，无论斜交角度如何，均用完整 $h_{\text{step}}$ 检查 $h_{\max}^{\text{climb}}$
3. 从高侧跨到低侧时，均用完整 $h_{\text{step}}$ 检查 $h_{\max}^{\text{drop}}$
4. $|\eta|$ 较小时，或足迹持续覆盖边缘但未完成跨越时，判为沿边缘行驶/骑跨，重点检查履带支撑和侧倾
5. 在 $|\eta|$ 阈值过渡区，同时执行完整高度检查和骑跨检查，采用更保守结果

$h_{\max}^{\text{climb}}$ 与 $h_{\max}^{\text{drop}}$ 为独立 TBD 参数，不能互相代替。

**履带支撑与侧倾评价**：

当前实现在左右履带支撑区域内以表面单元 `support_ratio`（上限为1）作权重，计算加权中位数支撑高程：

$$
\bar h_L=\operatorname{RobustLocation}\{h(p):p\in F_L\},
\qquad
\bar h_R=\operatorname{RobustLocation}\{h(p):p\in F_R\}
$$

$$
\Delta h_{\text{track}}=\bar h_L-\bar h_R,
\qquad
\alpha_{\text{roll}}=\arctan\left(
\frac{\Delta h_{\text{track}}}{B_{\text{eff}}}
\right)
$$

其中 $B_{\text{eff}}$ 为左右履带有效支撑中心距，不是整车外宽。当前 `RobustLocation` 是加权中位数；每侧覆盖率为支持权重总和除以足迹相交单元数，局部落差为最高与最低支持高程之差，离群点使用加权四分位距的 1.5 IQR 栅栏判定。任一侧低于 `minimum_track_support_ratio`、局部落差超过 `maximum_step_drop_m`、存在高程离群点或支撑滚转角超限都加入硬限制因素。

当 $|\alpha_{\text{roll}}|>\alpha_{\max}^{\text{roll}}$、任一履带支撑覆盖不足或足迹骑跨台阶导致局部悬空时，判为不可通行。前后支撑区、俯仰和对角扭转风险当前未实现，只能作为后续扩展，不能计入现有硬门禁。

**调用位置**：

Hybrid A* 搜索扩展节点时，对每个候选运动原语调用 `TraversabilityEvaluator`，评价整段扫掠足迹及其台阶相交过程。

具体接口见第14.2节。接口返回所有去重限制因素、纵横坡度均值与保守界、逐事件台阶穿越诊断、履带滚转角、局部落差、离群点和支撑覆盖率，以及完整的地形风险审计。无效风险策略返回 `RISK_POLICY_INVALID`，地形配置版本或运行域错配返回 `VERSION_MISMATCH`，无效协方差和地形分别返回 `COVARIANCE_INVALID` 与 `TERRAIN_INVALID`；调用方不得用局部均值或旧结果回退。

### 5.8 增量地形更新与版本失效

`IncrementalTerrainAnalyzer` 缓存上一份 `MapSnapshot`、序列化后的完整 `TerrainAnalysisConfig` 和 `shared_ptr<const TerrainLayers>`：

- 首次调用执行 `FULL_REBUILD`；同一版本、同一配置且逐字段相同的地图返回 `CACHE_HIT`，共享完全相同的只读派生层对象。
- 同一版本携带不同地图负载或配置、同一地图标识发生序号/时间戳回退、或单元变化落在声明的 `update_regions` 外时抛出 `invalid_argument`，不得复用缓存。
- 地图标识、网格几何或完整配置改变，或没有声明更新区时执行全量重建并报告 `source_version_invalidated=true`。
- 可增量更新时，每个声明区按局部平面物理窗口半径向外扩张并裁剪到地图；只有扩张区内的表面和缆线地形单元重新计算，其余单元从上一只读层复制。诊断记录扩张区、重算/复用单元数和更新模式；任何新地图版本的增量或全量结果都报告 `source_version_invalidated=true`，只有首次构建和缓存命中为 `false`。
- 台阶依赖的影响范围大于单个表面窗口。单元可用性变化、更新区内台阶候选几何变化，或既有台阶双侧支撑带内的拟合值变化都会使缓存全量失效；否则保留 `StepLayer`，仅在重算单元上重新标记不连续带。

每个新地图版本成功发布时都生成新的只读 `TerrainLayers` 并绑定当前 `MapVersion`；缓存命中返回原共享对象，其他旧共享对象不被原地修改。增量结果必须逐字段等价于同一输入的全量分析，不能缓存最终通行性或绕过当前机器人状态、碰撞策略和梯度风险策略。

---

## 6. 缆线落点适宜性评价

### 6.1 目的

机器人通行性不等同于缆线布放适宜性。机器人可以安全通过的区域，缆线可能因悬空、过度弯曲或粗糙海床而不适合布放。

### 6.2 缆线落点适宜性指标

**输入**：预测缆线落点位置 $(x_c, y_c)$ 及其处的地形信息。

`CableLayingEvaluator` 必须分别输出：

- `hard_feasible`：机械保护和禁放条件是否全部满足
- `failure_reasons`：违反的硬约束及对应位置/区段
- `soft_cost`：只用于可行方案之间排序的适宜性代价

**硬约束**，任何代价权重都不能覆盖：

1. 缆线绝对曲率不超过经制造商数据或试验确认的机械上限
2. 触地点扫掠不与禁放区、障碍物占地区或无有效地形数据区相交
3. 第一阶段的保守悬空代理不超过已标定阈值

**软指标**，只在硬约束通过后参与排序：

1. 曲率超过偏好值但尚未达到机械上限
2. 中等地形起伏风险和海床粗糙度
3. 参考线偏离

### 6.3 弯曲约束

**局部曲率计算**：

对预测缆线路径 $\mathcal{P}_{\text{cable}} = \{\mathbf{c}_j\}_{j=1}^M$，计算离散曲率：

$$
\kappa_{\text{cable}}(j) \approx \frac{\Delta \theta_j}{\Delta s_j}
$$

其中：
- $\Delta \theta_j$：相邻三点形成的转角
- $\Delta s_j$：弧长增量

定义两个不同阈值：

$$
0 < \kappa_{\text{cable}}^{\text{preferred}}
< \kappa_{\text{cable}}^{\max}
\leq \frac{1}{R_{\text{bend,min}}}
$$

其中 $R_{\text{bend,min}}$ 为制造商或试验确定的最小允许弯曲半径，$\kappa_{\text{cable}}^{\max}$ 是包含工程安全裕量后的硬上限。

**硬约束**：

$$
|\kappa_{\text{cable}}(j)| \leq \kappa_{\text{cable}}^{\max}
$$

左右转必须使用同一绝对值规则。超过上限时返回 `CABLE_CURVATURE_EXCEEDED` 并剪枝或拒绝整条候选路径。

**软代价**：

$$
c_{\text{bend}}(j) = 
w_{\text{bend}}\left[
\max\left(0,
|\kappa_{\text{cable}}(j)|-\kappa_{\text{cable}}^{\text{preferred}}
\right)
\right]^2
$$

该软代价不能代替最大曲率硬检查。离散曲率使用三个不同位置的采样点计算；点间距过小、重复点或数值非有限时，评价返回无效而不是按零曲率处理。首版评价只接受 `linear` 与 `cable-mean-spatial-lag` 两种明确按相邻触地点分段线性解释的规则，每段弧长必须在容差内等于几何弦长；其他插值规则 fail closed。曲率计算先消除同一折线上的共线细分点，再以生产必填的固定物理间距 $L_{\kappa,\text{eval}}$ 在每个规范化转角两侧插值取点，使共线输入重采样既不能降低最大曲率，也不能改变硬判定。弯曲软成本中的 $\Delta s_j$ 取该固定评价窗口在当前任务有效弧长域内的覆盖长度。增量 `evaluate_segment` 只结算右侧已获得完整 $L_{\kappa,\text{eval}}$ 窗口的转角，并将窗口不足的转角留在机械记忆中供后续原语结算；完整路径 `evaluate` 才允许在真实路径终点裁剪右侧窗口。已结算转角不得在后续原语重复计费，因此整段评价与任意原语切分后的增量累计必须产生相同硬判定和弯曲软成本。

### 6.4 地形起伏风险评估

**目的**：评估缆线路径沿线的地形起伏,识别可能导致悬空的区域。

**说明**：第一阶段不建立完整三维缆线动力学，不能声称精确预测实际悬垂形状。采用经试验标定的保守地形代理作为运行域硬门禁。

**方法**：

沿预测触地点轨迹使用固定物理长度 $L_{\text{support,eval}}$ 的后向滑动窗口查询海床高程变化，使增量搜索在生成当前样本时即可完成判定：

$$
\Delta h_{\text{terrain}}(j) =
\max_{s \in [s_j-L_{\text{support,eval}}, s_j]} h(s)
-
\min_{s \in [s_j-L_{\text{support,eval}}, s_j]} h(s)
$$

其中 $s$ 为沿缆线路径的弧长参数。候选起点之前的窗口部分必须来自当前实际 `laying_memory`，不得假定平坦或截断；任务真正起点没有历史时，使用显式起点边界策略并记录有效窗口长度。只查询 $[s_j-L_{\text{support,eval}},s_j]$ 内仍会影响当前或后续候选的历史，规范化记忆为几何插值而保留的窗口外包围点不得单独触发硬失败。窗口与路径采样间隔无关；禁放、障碍、未知和低置信度单元使用保守栅格 supercover 检查，掠过单元边角也不能被点采样跳过，且不得插值成平坦海床。

**风险分级**：

$$
\text{terrain\_risk}(j) = 
\begin{cases}
\text{HIGH} & \text{if } \Delta h_{\text{terrain}}(j) > \Delta h_{\text{support}}^{\max} \\
\text{MEDIUM} & \text{if } \Delta h_{\text{terrain}}(j) > \Delta h_{\text{medium}} \\
\text{LOW} & \text{otherwise}
\end{cases}
$$

其中 $\Delta h_{\text{support}}^{\max}$ 是独立试验确定的硬门禁，$\Delta h_{\text{medium}}<\Delta h_{\text{support}}^{\max}$ 是软风险阈值，均为 TBD。原有 1.0 m/0.5 m 示例值不具有安全依据，不作为默认值。

**硬约束**：

$$
\Delta h_{\text{terrain}}(j)
\leq \Delta h_{\text{support}}^{\max}
$$

超过该阈值时返回 `CABLE_SUPPORT_PROXY_EXCEEDED`。这是首版保守运行域门禁，不等价于实际悬空高度；在完成柔性缆线垂向模型或足够的实测验证前，不得据此宣称已经精确验证悬空量。

**软代价**：

$$
\ell_{\text{terrain\_risk}}(j) = w_{\text{terrain\_risk}} \cdot \text{risk\_score}(j)
$$

其中:
$$
\text{risk\_score}(j) = 
\begin{cases}
1.5 & \text{if terrain\_risk}(j) = \text{MEDIUM} \\
0.0 & \text{if terrain\_risk}(j) = \text{LOW}
\end{cases}
$$

`HIGH` 不进入软代价计算，在实现中直接通过 `hard_feasible=false` 表达。第二阶段可接入经验证的柔性电缆动力学模型替换该保守代理。

### 6.5 粗糙度影响

缆线落点处粗糙度影响长期磨损：

$$
c_{\text{cable\_rough}}(j) = w_{\text{cable\_rough}} \cdot \sigma_{\text{rough}}(x_c^j, y_c^j)
$$

若项目材料或合同给出不可接受的粗糙度/尖锐凸起上限，则该上限另行配置为硬约束；在没有物理依据时，粗糙度只作为软代价，不把经验权重伪装成安全阈值。

### 6.6 参考线偏离代价

对触地点均值 $\boldsymbol\mu_c$，在搜索状态携带的任务进度 $s_{\text{prog}}$ 处定义单位法向 $\mathbf n_{\text{ref}}$ 和有符号横向均值误差：

$$
\mu_\perp=
\mathbf n_{\text{ref}}^T
\left(
\boldsymbol\mu_c-\mathbf r_{\text{ref}}(s_{\text{prog}})
\right)
$$

**首版参考线假设**：甲方给定的前期规划路线视为确定量，直接取：

$$
\Sigma_{\text{ref}}=0
$$

只有当参考线来自含误差的测绘重建、坐标转换或其他随机估计时，才扩展 $\Sigma_{\text{ref}}$ 及必要的交叉协方差。首版横向标准差为：

该假设不允许忽略机器人定位到甲方参考坐标系之间的变换误差；这部分必须已经包含在世界坐标系下的 $\Sigma_c$ 中。若无法做到，首版 $\Sigma_{\text{ref}}=0$ 假设不成立。

$$
\sigma_\perp=
\sqrt{
\mathbf n_{\text{ref}}^T
\Sigma_c
\mathbf n_{\text{ref}}
}
$$

**分级约束**：

定义双层走廊:
- 期望走廊: $w_{\text{corridor}}^{\text{nominal}}$ (软约束,精度目标)
- 绝对上限: $w_{\text{corridor}}^{\text{max}}$ (硬约束,物理/合同边界)

**精确双侧高斯概率**：

$$
P(-w\leq E_\perp\leq w)=
\Phi\!\left(\frac{w-\mu_\perp}{\sigma_\perp}\right)
-
\Phi\!\left(\frac{-w-\mu_\perp}{\sigma_\perp}\right)
$$

验证阶段由实际路径协方差计算 $\sigma_\perp$。搜索阶段不传播该路径相关量，而使用第7.5.1节的 $\bar\sigma_\perp(s_{\text{prog}})$。两阶段都不逐点计算上述精确概率，而使用**保守机会约束替代式**（conservative chance-constraint surrogate）：

$$
d_{\text{upper}}=
|\mu_\perp|
+z_{1-\epsilon_{\text{point}}/2}\sigma_{\perp,\text{eval}}
$$

其中搜索阶段取 $\sigma_{\perp,\text{eval}}=\bar\sigma_\perp(s_{\text{prog}})$，验证和性能评估阶段取 $\sigma_{\perp,\text{eval}}=\sigma_{\perp,\text{real}}$，并先执行包络覆盖审计。

$d_{\text{upper}}\leq w$ 是满足双侧概率要求的工程充分条件，不是精确等价式。$\epsilon_{\text{point}}$ 描述单个评价位置或相关区段因不确定性导致误判的统计风险，取值必须由风险需求和标定覆盖率确定，不能用任意 `confidence` 分数代替。

其中 $\Phi$ 为标准正态分布函数，$z_p=\Phi^{-1}(p)$。当 $\sigma_\perp=0$ 时按确定量处理，令 $d_{\text{upper}}=|\mu_\perp|$，不计算除零形式。协方差非有限、非半正定或残差分布未完成覆盖率标定时，评价返回无效状态，不能降级成某个经验 `confidence` 乘数。

**触地点走廊软代价**：

$$
c_{\text{td\_corridor}}(j) = 
\begin{cases}
w_{\text{td,center}} \cdot \mu_\perp^2 & \text{if } d_{\text{upper}} < w_{\text{corridor}}^{\text{nominal}} \\
w_{\text{td,margin}} \cdot \left(\frac{d_{\text{upper}} - w_{\text{corridor}}^{\text{nominal}}}{w_{\text{corridor}}^{\text{max}} - w_{\text{corridor}}^{\text{nominal}}}\right)^2 & \text{if } w_{\text{corridor}}^{\text{nominal}} \leq d_{\text{upper}} < w_{\text{corridor}}^{\text{max}} \\
\infty & \text{if } d_{\text{upper}} \geq w_{\text{corridor}}^{\text{max}}
\end{cases}
$$

该代价只评价预测触地点 $\boldsymbol\mu_c$，不得以机器人中心到 $\mathcal R_{\text{ref}}$ 的距离替代。机器人允许作业区域由 $\mathcal W_{\text{robot}}$ 独立检查。

**约束状态分级**：

$$
\text{constraint\_status}(j) = 
\begin{cases}
\text{PASS} & \text{if } d_{\text{upper}}(j) < w_{\text{corridor}}^{\text{nominal}} \\
\text{MARGINAL} & \text{if } w_{\text{corridor}}^{\text{nominal}} \leq d_{\text{upper}}(j) < w_{\text{corridor}}^{\text{max}} \\
\text{VIOLATION} & \text{if } d_{\text{upper}}(j) \geq w_{\text{corridor}}^{\text{max}}
\end{cases}
$$

**可接受边缘段**：`MARGINAL` 不是硬约束违反，但其累计长度必须满足

$$
L_{\text{marginal}}\leq L_{\text{marginal}}^{\max}
$$

其中 $L_{\text{marginal}}^{\max}$ 为任务策略中的必填阈值（TBD）。$L_{\text{marginal}}$ 按触地点轨迹与 MARGINAL 区间的弧长交集累计，区间边界使用保守插值并计入离散裕量，不能用 `marginal_count × 固定步长` 近似。超过该阈值的候选必须拒绝，不设置可绕过该门禁的“非严格模式”；任何 `VIOLATION` 均拒绝。

**首版路径风险边界**：搜索实现 $\bar\sigma_\perp(s_{\text{prog}})\rightarrow d_{\text{upper}}\rightarrow$ 分级结果；候选验证实现 $\Sigma_c(\gamma)\rightarrow\sigma_{\perp,\text{real}}\rightarrow$ 包络审计与精确分级。首版不实现整条路径联合越界概率。$\epsilon_{\text{path}}$ 和误差相关长度 $L_{\text{error-correlation}}$ 保留为 TBD。不得将离散采样点数量直接当作独立试验次数。

**与任务成功率分离**：

甲方布放成功率指标定义实际铺设结果中合格长度的比例：

$$
R_{\text{lay}}=
\frac{L_{\text{actual,in-tolerance}}}{L_{\text{actual,total}}}
\geq 0.8
$$

$R_{\text{lay}}$ 是任务性能指标；$\epsilon_{\text{point}}$、$\epsilon_{\text{path}}$ 是模型判断错误的统计风险指标。二者必须同时记录，且绝对不能因为 $R_{\text{lay}}\geq0.8$ 就设置 $\epsilon=0.2$。

### 6.7 综合落点适宜性

先计算硬可行性：

$$
\text{CableLayingFeasible}=
\text{CurvatureValid}
\land\text{SupportProxyValid}
\land\text{ForbiddenAreaClear}
\land\text{TerrainDataValid}
$$

只有 `CableLayingFeasible=true` 时才计算软代价：

$$
\ell_{\text{cable\_suitability}}(j)=
c_{\text{bend}}(j)
+\ell_{\text{terrain\_risk}}(j)
+c_{\text{cable\_rough}}(j)
$$

$$
C_{\text{cable\_suitability}}=
\sum_{j=1}^{M}
\ell_{\text{cable\_suitability}}(j)\Delta s_j
$$

搜索阶段对每个运动原语产生的完整触地点段执行硬检查；最终验证阶段从当前 `CableState` 对拼接后的完整机器人路径重积分，并对完整触地点路径再次执行同一组硬检查。

规划中的触地点局部成本密度为 $c_{\text{td,corridor}}+\ell_{\text{cable,suitability}}$。两项只组合一次，禁止再额外叠加另一个 $d_{\text{cable-ref}}^2$ 造成重复计权。

---

## 7. 参数化缆线布放模型

### 7.1 模型目的

将主机器人路径 $\mathcal{P}_{\text{robot}}$ 映射为预测缆线触地点轨迹 $\mathcal{P}_{\text{cable}}$，使最终铺设中心线约束能够直接参与规划搜索和优化。

本节中的“落点”统一指缆线首次稳定接触海床的触地点。第一阶段假设缆线接触海床后不发生显著横向滑移，因此最终铺设中心线近似等于触地点轨迹。

### 7.2 第一阶段简化模型

**设备与模型假设**：

1. 缆线从机器人上的固定放缆点主动释放
2. 放缆控制器使出缆速度跟随机器人对地速度，并维持可控张力
3. 触地点位于放缆点后方，等效水平距离在单次规划窗口内近似恒定
4. 缆线水平方向随机器人航向渐进响应，而非瞬时对齐
5. 缆线接触海床后不发生显著横向滑移
6. 第一阶段只覆盖前进铺设，不允许在持续出缆时倒车或原地转向
7. 第一阶段不考虑高保真流固耦合、垂向悬链线和大幅松弛

如果计划剖面中的出缆速度、张力设定值或其批准跟踪误差超出标定范围，模型必须返回对应的非 `VALID` 状态，规划结果不得继续宣称满足落点精度约束。当前遥测只用于初始化和执行偏差监控，不能作为整条未来路径的恒定速度或张力输入。

模型有效条件至少包括：

$$
|v_{\text{payout}}-v_{\text{ground}}| \leq \epsilon_{\text{payout}},
\qquad
T_{\min} \leq T \leq T_{\max}
$$

**模型参数**：

| 参数 | 符号 | 单位 | 说明 |
|------|------|------|------|
| 放缆点偏置 | $\mathbf{p}_r = (x_r, y_r)$ | m | 相对机器人本体坐标系 (TBD) |
| 触地点距离 | $L_{\text{td}}$ | m | 放缆点到触地点的等效水平距离 (TBD) |
| 方向响应长度 | $L_{\psi}$ | m | 缆线水平方向收敛到机器人航向的空间尺度 (TBD) |
| 允许出缆速度误差 | $\epsilon_{\text{payout}}$ | m/s | 模型有效范围 (TBD) |
| 允许张力范围 | $[T_{\min},T_{\max}]$ | N | 模型标定与有效范围 (TBD) |
| 模型过程噪声 | $Q_{\text{model}}$ | m$^2$ | 触地点残差协方差模型 (TBD) |

### 7.3 放缆点位置计算

对于机器人位姿 $\mathbf{x}_i = (x_i, y_i, \theta_i)$，放缆点在世界坐标系中的位置：

$$
\begin{bmatrix}
x_{\text{release}} \\
y_{\text{release}}
\end{bmatrix}
=
\begin{bmatrix}
x_i \\
y_i
\end{bmatrix}
+
\begin{bmatrix}
\cos\theta_i & -\sin\theta_i \\
\sin\theta_i & \cos\theta_i
\end{bmatrix}
\begin{bmatrix}
x_r \\
y_r
\end{bmatrix}
$$

### 7.4 触地点状态与预测

定义 $\psi_c(s)$ 为从触地点指向放缆点的缆线水平朝向，$s$ 为机器人路径弧长。触地点均值为：

$$
\mathbf{c}(s) = \mathbf{p}_{\text{release}}(s) - L_{\text{td}}
\begin{bmatrix}
\cos\psi_c(s) \\
\sin\psi_c(s)
\end{bmatrix}
$$

缆线方向按空间距离渐进跟随机器人航向：

$$
\frac{d\psi_c}{ds} = \frac{\operatorname{wrap}(\theta-\psi_c)}{L_{\psi}}
$$

定义相对滞后角：

$$
\delta = \operatorname{wrap}(\psi_c-\theta)
$$

对于长度为 $\Delta s$、曲率近似恒定为 $\kappa$ 的运动原语，采用以下闭式近似传播：

$$
\delta_{k+1} = \operatorname{wrap}\left(
\delta_k e^{-\Delta s/L_{\psi}}
- \kappa L_{\psi}\left(1-e^{-\Delta s/L_{\psi}}\right)
\right)
$$

搜索阶段可以使用该闭式更新；最终验证阶段使用同一微分模型在更小步长上数值积分。两阶段只能改变积分精度，不能使用不同的物理模型。

**初始状态**：滚动规划必须从 `CableStateTracker` 提供的当前状态快照开始。Tracker 只根据实际执行轨迹、放缆遥测和可选触地点观测更新，不能把尚未执行的候选路径终端状态当作当前状态。任务启动或状态丢失时，可由已执行机器人轨迹和最近触地点观测估计；无法可靠初始化时返回 `INITIAL_STATE_UNCERTAIN` 并扩大预测不确定性。

### 7.5 不确定性传播

预测缆线触地点的不确定性来源：

1. 机器人定位误差 $\sigma_{\text{loc}}$
2. 模型参数误差 $\sigma_{L_{\text{td}}}$, $\sigma_{L_{\psi}}$
3. 海床地形变化导致的沉降不确定性
4. 初始滞后角 $\delta_0$ 的估计误差
5. 出缆速度跟踪误差和张力波动

**误差预算**：

将定位、模型参数、初始状态和沉降扰动组成联合随机量 $\mathbf{z}$，使用触地点模型对其做一阶传播：

$$
\Sigma_c \approx J_z\Sigma_zJ_z^T + Q_{\text{model}},
\qquad
J_z = \frac{\partial \mathbf{c}}{\partial \mathbf{z}}
$$

不能把不同物理量的标准差未经灵敏度映射直接相加。横向走廊约束如何从 $\Sigma_c$ 提取置信裕量在第6.6节统一定义。

$J_z$ 一般取决于到达当前节点的完整机器人路径，过程噪声也可能随路径累计，因此实际 $\Sigma_c=\Sigma_c(\gamma)$ 具有路径记忆。即使两个节点具有相同的机器人位姿、滞后角和参考路线进度，它们的实际协方差也未必相同。搜索阶段不得在忽略该差异的同时使用路径相关协方差做硬约束，否则仅按基础键保留最低代价标签的合并不成立。

该不确定性用于：
- 判断预测触地点是否满足允许走廊的统计风险要求
- 评价缆线自身的障碍和落点适宜性风险

该协方差不得用于机器人本体障碍膨胀；机器人使用第5.4节独立误差预算。

#### 7.5.1 搜索阶段横向不确定性包络

首版不把实际协方差或参数灵敏度加入 Hybrid A* 状态，而在每次规划前选择固定的传感器健康模式 $m_{\text{sensor}}$，并定义设计运行域内的有界候选历史集合：

$$
\Gamma_H(s;m_{\text{sensor}})=
\left\{
\gamma\;\middle|\;
L(\gamma)\leq L_H,
T(\gamma)\leq T_H,
|\kappa|\leq\kappa_{\max},
|\delta|\leq\delta_{\max},
\gamma\text{ 满足受控出缆、张力和传感器健康假设}
\right\}
$$

最大绕行长度、规划时长、初始不确定性范围、允许运动原语、认证执行运行包络（对地速度、加减速度、横向加速度、出缆跟踪误差和张力范围）及传感器可用性都属于 $\Gamma_H$ 的版本化定义。每个包络必须显式绑定 `cable_model_version`、`execution_operating_envelope_version`、`reference_line_version`、`sensor_mode`、`operating_domain_id` 和 `generator_version`；`operating_domain_id` 不能替代这些独立版本字段。搜索只使用完全匹配的包络，不把当前某一时刻的标量遥测外推到候选路径。最终接受的 `ExecutionProfile` 必须完全落在包络绑定的同一执行运行包络内。传感器模式在一次规划请求内保持不变；任一依赖版本或健康状态变化时当前包络和规划结果立即失效并触发重规划。不得把包络外推到 USBL、DVL 或其他定位信息无限期失效的工况。

参考路线按连续任务进度 $s_{\text{prog}}$ 分段。对每个进度段只为走廊实际消费的横向风险量构造包络：

$$
\bar v_\perp(s_m;m_{\text{sensor}})
\geq
\sup_{\gamma\in\Gamma_H(s_m;m_{\text{sensor}})}
\mathbf n_m^T\Sigma_c(\gamma,s_m)\mathbf n_m,
\qquad
\bar\sigma_\perp(s_m)=\sqrt{\bar v_\perp(s_m)}
$$

其中 $\mathbf n_m$ 必须与 `CableCorridorEvaluator` 在同一参考线版本和同一进度段使用的法向一致。内部可以传播完整协方差和参数灵敏度，但不得为了形式统一而构造过度保守的全矩阵上界。

**包络生成算法**：

1. 按弧长、航向、$\delta$ 和 $s_{\text{prog}}$ 对 $\Gamma_H$ 内的可达集合分箱
2. 对所有允许运动原语进行区间传播或分支定界，传播均值状态、参数灵敏度集合和过程噪声
3. 在每个进度段求横向方差的确定性上界，并单独计算和记录状态分箱、数值积分及参考法向离散余项 $\rho_{\text{env,disc}}$
4. 只剪除经集合包含或风险上界证明支配的可达集合；该生成器不得套用基础状态键的最低当前代价规则丢弃历史
5. 使用独立蒙特卡洛、对抗场景和海试数据检查包络紧致度及遗漏工况，但样本最大值不能单独冒充上述上确界

运行时在相邻进度分段之间使用上包络查询，禁止可能低估的普通线性插值：

$$
\bar\sigma_\perp(s)=
\max\{\bar\sigma_\perp(s_m),\bar\sigma_\perp(s_{m+1})\}
+\rho_{\text{env,disc}}
$$

搜索阶段的绝对走廊硬约束为：

$$
|\mu_\perp(s_i)|
+z_{1-\epsilon_{\text{point}}/2}\bar\sigma_\perp(s_i)
+\rho_{\text{cable,sweep}}
\leq d_{\max}
$$

$\rho_{\text{env,disc}}$、$\rho_{\text{cable,sweep}}$ 和统计分位数对应不同误差来源，必须分别记录，不得相互代替。

### 7.6 缆线模型在规划中的使用

**统一模型原则**：搜索和验证使用同一触地点均值模型。搜索阶段只允许采用较粗积分步长和第7.5.1节规定的横向不确定性包络，不得替换状态方程、逐节点传播路径相关协方差或另设经验落点公式。

**在搜索过程中**：
- 每个候选节点携带离散化的滞后角状态 $\delta$
- 扩展运动原语时传播 $\delta$ 并生成该原语上的触地点轨迹
- 所有速度、加速度、出缆误差和张力判断均使用已认证执行运行包络；当前标量遥测仅初始化搜索，不代表未来原语的执行量
- 使用 $s_{\text{prog}}$ 对应的 $\bar\sigma_\perp$ 检查整段触地点轨迹是否在允许走廊内
- 计算触地点偏离和布放适宜性代价

**在路径验证中**：
- 先对平滑后的几何路径生成不改变几何的 `TimedPath`；参数化失败即不存在可执行候选
- 从相同初始 `CableState` 出发，按 `ExecutionProfile` 的逐点对地速度、出缆速度和张力设定值进行高精度重积分
- 传播路径相关的实际 $\Sigma_c$，检查包络覆盖、全路径触地点约束和模型有效性
- 如果验证失败,重新规划或调整参数

### 7.7 搜索状态、验证与模型接口

#### 7.7.1 Markov 状态

仅使用机器人位姿 $(x,y,\theta)$ 无法区分具有不同缆线朝向、任务阶段或机械约束历史的到达路径。搜索标签必须同时包含滞后角、参考路线连续任务进度，以及机械评价未来仍会读取的有限历史：

$$
\mathbf{x}_{\text{search}} = (x,y,\theta,\delta,s_{\text{prog}},\mathcal M_{\text{lay}})
$$

其中 $\mathcal M_{\text{lay}}$ 是 `CableLayingEvaluator` 定义的规范化有界记忆，至少保存最后两个不同触地点，以及长度不小于 $\max(L_{\text{support,eval}}, 2L_{\kappa,\text{eval}})$ 的后向滑动窗口样本。前者维持基础三点曲率状态，后者确保原语末端尚未获得完整右窗口的转角能在后续扩展中以相同左窗口结算。超过该长度且不再影响任何未来曲率、硬约束或软代价的历史必须丢弃，因此不需要保存完整路径。

基础搜索键仍为离散化后的 $(i_x,i_y,i_\theta,i_\delta,i_s)$，但每个基础键对应一个标签集合。$s_{\text{prog}}$ 表示沿参考路线的任务阶段，不是每个节点重新执行全局最近点匹配得到的几何投影。不同 $\delta$、不同任务进度，或 $\mathcal M_{\text{lay}}$ 未被证明未来等价的标签不得直接合并。

实际 $\Sigma_c(\gamma)$ 仍有路径记忆。首版搜索只使用第7.5.1节中与具体到达历史无关的 $\bar\sigma_\perp(s_{\text{prog}})$，不把实际协方差加入标签。机械曲率和悬空代理的路径记忆则不能由该包络消除，必须通过 $\mathcal M_{\text{lay}}$ 显式保留。最终验证传播实际协方差，但不得把实际协方差重新反馈为同一基础键下的路径相关搜索约束。

为控制状态数量：

- 限制 $|\delta| \leq \delta_{\max}$，超出标定范围的节点直接判为模型无效
- $\Delta_\delta$ 可粗于机器人航向分辨率，但必须通过误差实验确定
- 只保留在认证执行运行包络内满足张力、出缆跟踪和走廊约束的缆线状态标签
- 只有 `future_equivalent(a.memory, b.memory)` 经证明对所有后续原语产生相同机械判定和增量代价时，才允许在两者之间应用最低代价支配；默认规则是记忆不同即不可比较
- 若活动标签数达到 `maximum_active_labels`，或扩展数/单调时钟达到各自预算，返回带区分诊断的 `TIMEOUT`，不得按最低当前代价任意删除不可比较标签；当前没有独立 byte-budget 字段

#### 7.7.2 搜索阶段

每次扩展运动原语时：

1. 沿原语传播机器人位姿
2. 使用第7.4节状态方程传播 $\delta$
3. 计算原语内所有采样点的触地点 $\mathbf{c}$
4. 在认证执行运行包络内保守检查模型有效范围、触地点走廊硬约束和缆线机械硬约束
5. 仅对硬约束通过的触地点段积分偏差和适宜性软代价

搜索阶段的均值积分和硬约束采样必须符合第8.3.1节。第一个标签从当前实际 `CableState.laying_memory` 初始化，后继仅使用父标签记忆的副本推进。走廊评价只读取已锁定版本的横向包络，不读取候选节点携带的路径相关方差。搜索所用缆线模型版本、执行运行包络版本、参考线版本、传感器模式和运行域必须与横向不确定性包络逐项匹配；最终时序剖面超出该执行包络时不得进入缆线验证。

#### 7.7.3 验证阶段

候选机器人几何路径确定并通过 G2 复检后，先由 `TrajectoryParameterizer` 生成 `TimedPath`。随后从相同初始 `CableState`（包括当前实际 `laying_memory`）出发，按该时序剖面以更小积分步长重新预测触地点轨迹，并执行：

- 模型有效性检查：逐点出缆速度误差、张力设定值、运动模式和滞后角均在标定范围内
- 包络审计：$\sigma_{\perp,\text{real}}(s)\leq\bar\sigma_\perp(s)+\epsilon_{\text{env}}$
- 全轨迹走廊分级检查
- 对完整触地点轨迹执行最大绝对曲率、禁放区、地形数据有效性和悬空代理硬检查
- 仅在硬检查通过后计算地形适宜性软代价
- 状态剖面与终端 `CableState` 输出，供路径诊断、承诺段传播和执行状态跟踪

`laying_memory` 只保存触地点弧长和几何位置，不缓存地图高程或可行性结论；`CableLayingEvaluator` 每次都用当前锁定的地形图层重新查询这些位置。地图版本变化因此会重新评价历史窗口，而不会沿用旧地图结果。

正常情况下，包络内通过搜索的路径不应再因实际协方差更大而失败。若包络审计失败，返回 `COVARIANCE_ENVELOPE_BREACH`，使当前包络版本和依赖它的规划结果失效，并停车或切换到经批准的降级模式；这不是普通候选路径失败。其他由均值积分、平滑或地形变化导致的验证失败仍按候选路径失败处理。不得通过放宽硬走廊掩盖模型问题。

#### 7.7.4 模块接口

`CableModel` 是搜索与最终验证共享的深模块。搜索入口接收几何运动段和认证执行运行包络，最终验证入口只接收完整 `TimedPath`；调用方不自行实现状态传播或误差拼装。以下接口以 `core/cable_model.hpp` 与 `core/data_contract.hpp` 为唯一契约，旧版文档中的 `Path`、`Matrix2`、`CableModelParams` 和 PascalCase 方法名仅是概念称呼，不是可编译接口。

```cpp
enum class CableModelValidity {
    valid, initial_state_uncertain, payout_tracking_out_of_range,
    tension_out_of_range, lag_angle_out_of_range, motion_mode_out_of_range,
    input_invalid, sensor_mode_unapproved, operating_domain_mismatch,
    execution_envelope_version_mismatch, covariance_invalid
};

struct CableHistorySample {
    double touchdown_arc_length_m;
    Vector2m touchdown_position_m;
};

struct CableConstraintMemory {
    std::vector<Vector2m> previous_distinct_touchdown_points_m; // 最后两个不同点
    std::vector<CableHistorySample> trailing_support_samples;
    double retained_arc_length_m;
    uint64_t canonical_signature; // 快速筛选，命中后仍须逐样本比较
};

struct CableState {
    CableStateKind kind;
    double lag_angle_rad;             // delta
    std::optional<double> lag_angle_variance_rad2; // SEARCH 必须为空
    MonotonicTime timestamp;
    CableConstraintMemory laying_memory;
    uint64_t sequence_number;
};

struct CableContext {
    CableTelemetry current_telemetry; // 只表示预测起点的同步实测状态
    ExecutionOperatingEnvelope execution_envelope; // SEARCH使用的认证未来运行域
    PredictionMode mode;          // SEARCH仅预测均值；VALIDATION传播实际协方差
    SensorHealthMode sensor_mode;
    uint64_t uncertainty_envelope_version;
    uint64_t uncertainty_envelope_generator_version;
    uint64_t robot_uncertainty_profile_version;
    std::vector<RobotUncertaintySample> robot_uncertainty_profile;
};

struct CablePrediction {
    CableState terminal_state;
    GeometricPath touchdown_path;
    std::vector<double> robot_arc_length_profile_m;
    std::vector<CableState> state_profile;
    std::optional<std::vector<Covariance2dM2>> touchdown_covariance_profile_m2;
    CableModelValidity validity;
    CableModelDependencyVersions dependencies;
    std::vector<std::string> issues;
};

class CableModel {
public:
    CablePrediction predict_search(
        const CableState& initial_state,
        const GeometricPath& robot_segment,
        const CableContext& context
    ) const;

    CablePrediction predict(
        const CableState& initial_state,
        const TimedPath& robot_path,
        const CableContext& context
    ) const;

    CableMeanSample predict_touchdown_mean(
        const Pose2d& robot_pose, double lag_angle_rad) const;
    CableInverseMeanSample inverse_touchdown_mean(
        Vector2m touchdown_target_m, double touchdown_heading_rad,
        double robot_heading_rad, double lag_angle_rad,
        MonotonicTime timestamp) const;
    CableModelIdentity identity() const;
    void set_parameters(const CableModelParameters& params);
    uint64_t version() const noexcept;
};
```

**接口不变量**：

- 输入轨迹弧长必须单调，且包含足以积分每个运动原语的采样点
- `VALIDATION` 只接受逐点具有有限、单调时间和完整执行剖面的 `TimedPath`；纯几何 `Path` 必须在接口处拒绝
- 剖面首样本必须与 `current_telemetry` 在批准跟踪误差内连续；后续预测读取逐样本计划量及其已标定误差模型，不把起点遥测常量外推
- `SEARCH` 可使用几何原语，但只能在 `execution_envelope` 覆盖的运行域内做保守预测；不得用 `current_telemetry` 代替未来剖面
- 验证使用的执行剖面必须落在搜索及 $\Gamma_H$ 绑定的认证运行包络内
- `VALID` 只表示输入处于模型标定范围，不等同于走廊约束通过
- `VALIDATION` 模式下 `touchdown_path` 与 `touchdown_covariance_profile_m2` 必须逐点对齐，所有协方差有限且半正定；`SEARCH` 模式不得输出或消费路径相关协方差
- `SEARCH` 模式只读取和输出 `lag_angle`，`lag_angle_variance` 必须为空；不得将候选终端方差传给下一搜索节点
- `CableModel` 不解释或推进 `laying_memory`；增量搜索必须用 `CableLayingEvaluator` 返回的 `terminal_memory` 替换预测终端状态中的该字段
- `CableModel` 只预测均值和协方差，不解释 `confidence`，也不决定 PASS/MARGINAL/VIOLATION
- 搜索和验证必须使用同一组物理参数与相同初始状态
- 搜索使用的横向包络必须与参考线、传感器健康模式、设计运行域、缆线模型版本、执行运行包络版本和包络版本逐项一致
- 当前状态必须由实际执行数据持续传播，不能在每次重规划时重置为零，也不能提前写入候选路径终端状态

---

## 8. 参考线引导的 Hybrid A* 规划

### 8.1 算法概述

**核心思想**：

在传统 Hybrid A* 基础上，用全局参考路线评价预测缆线触地点和任务进度，同时保持机器人安全、运动学与缆线走廊硬约束不可违反。参考路线不是机器人中心线。

**与传统 Hybrid A* 的区别**：

| 方面 | 传统 Hybrid A* | 本方案 |
|------|----------------|--------|
| 目标 | 最短路径 | 使预测触地点接近期望参考线的安全路径 |
| 启发函数 | 到目标的欧氏距离 | 到反解机器人接入状态的 Dubins 下界 |
| 代价函数 | 路径长度 + 曲率 | 机器人运动代价 + 触地点走廊代价 + 缆线适宜性 |
| 终点 | 固定目标点 | 由多个触地点接入目标反解的机器人状态 |

**域分离不变量**：

- 机器人位姿只对 $\mathcal W_{\text{robot}}$、碰撞、地形和运动学负责
- 触地点只对 $\mathcal W_{\text{cable}}$、参考线偏差和缆线机械约束负责
- 除非任务另外提供独立的机器人引导线，否则不得计算机器人中心到 $\mathcal R_{\text{ref}}$ 的代价或把机器人限制在缆线走廊内

### 8.2 状态空间

**状态定义**：

$$
\mathbf{x} = (x, y, \theta, \delta, s_{\text{prog}}, \mathcal M_{\text{lay}})
$$

- $(x, y)$：机器人平面位置
- $\theta$：航向角
- $\delta$：缆线方向相对机器人航向的滞后角
- $s_{\text{prog}}$：沿参考路线的连续任务进度状态，不是当前位置的全局最近点
- $\mathcal M_{\text{lay}}$：跨原语机械曲率与固定物理窗口评价所需的规范化有限历史

**离散化**：

- 空间分辨率：$\Delta_{xy}$ (TBD, 建议 0.2-0.5 m)
- 角度分辨率：$\Delta_{\theta}$ (TBD, 建议 $\pi/36$ = 5°)
- 缆线滞后角分辨率：$\Delta_{\delta}$ (TBD，必须通过搜索复杂度与落点误差实验确定)
- 参考路线进度分辨率：$\Delta_s$ (TBD，必须小于会造成分支混淆的最短参考线间隔)

搜索以 $(i_x,i_y,i_\theta,i_\delta,i_s)$ 作为基础键，每个基础键保存多个机械历史标签。不得合并具有不同缆线动态状态、不同任务进度或未来不等价 $\mathcal M_{\text{lay}}$ 的节点。

当前公共契约将上述边界拆成 `HybridAStarBaseKey` 与标签携带的 `CableState`。内部 `SearchNode` 另外保存连续机器人位姿、连续 `ReferenceProgress`、父节点、入边机器人/触地点轨迹、五分量累计成本和启发值；`HybridAStarStateTraceEntry` 只发布可审计的基础键、连续位姿、滞后角和参考进度，不暴露内部队列或标签存储。五个索引均由对应连续量除以版本化分辨率后取最近整数；航向和滞后角先规范化。`CableState` 在搜索中强制为 `search_mean`，不携带路径相关滞后角方差。

**任务进度传播**：$s_{\text{prog}}$ 从任务起点或上一已执行承诺段的终端进度初始化。对子运动原语 $p$，只允许在父节点进度附近关联参考线：

$$
s_{k+1}\in
\left[
s_k-\epsilon_{\text{back}},
s_k+\alpha_sL_p+\epsilon_s
\right]
$$

其中 $L_p$ 为该原语弧长，防止短原语在交叉或邻近参考线段之间跳支。使用终端预测触地点和缆线方向，在局部窗口内最小化无量纲投影目标：

$$
J_{\text{proj}}(s)=
\left(\frac{d_\perp(s)}{d_0}\right)^2
+\lambda_\theta
\left(\frac{\Delta\theta_{\text{cable-ref}}(s)}{\theta_0}\right)^2
$$

第一阶段持续铺设只允许前进原语，进度应在 $\epsilon_{\text{back}}$ 容差内非递减。若未来增加非铺设倒车机动，$s_{\text{prog}}$ 仍表示任务阶段；短距离倒车不得自动解释为任务进度回退。

当前实现由 `ReferenceProgressAssociator::propagate_candidate` 对触地点轨迹逐样本传播进度。请求结构、同步时间、参考线版本或坐标系依赖无效时，`HybridAStarPlanner::plan` 在扩展前返回 `input_invalid` 并记录输入无效或依赖错配；搜索过程中某个触地点样本无法在局部窗口内关联时，才递增 `reference_association_rejection_count` 并剪枝该后继。两类失败都不会退回全局最近点或静默沿用父节点终值。

### 8.3 运动原语

使用曲率约束的运动原语集合 $\mathcal{U}$：

$$
\mathcal{U} = \{ (\Delta s, \kappa) \mid \Delta s > 0, |\kappa| \leq \kappa_{\max} \}
$$

**常用原语集**：

- 直行：$\kappa = 0$
- 左转/右转：$\kappa = \pm \kappa_{\max}$
- 中等转弯：$\kappa = \pm 0.5 \kappa_{\max}$

**运动学更新**（前进恒曲率精确积分）：

对于原语 $(L_p,\kappa)$，令 $\theta_{k+1}=\theta_k+\kappa L_p$。当 $\kappa=0$ 时：

$$
\begin{cases}
x_{k+1}=x_k+L_p\cos\theta_k\\
y_{k+1}=y_k+L_p\sin\theta_k
\end{cases}
$$

当 $\kappa\neq0$ 时：

$$
\begin{cases}
x_{k+1}=x_k+\dfrac{\sin(\theta_k+\kappa L_p)-\sin\theta_k}{\kappa}\\
y_{k+1}=y_k+\dfrac{\cos\theta_k-\cos(\theta_k+\kappa L_p)}{\kappa}
\end{cases}
$$

原语内硬约束采样姿态也必须由同一精确曲线计算。中点近似不得用于生成运动几何，只用于第8.3.1节的软成本积分。

每个原语还必须按第7.4节传播 $\delta$，并生成该原语对应的触地点轨迹。机器人运动学有效但触地点模型无效的原语同样不可接受。

**倒车处理**：机器人本体即使允许倒车，第一阶段受控放缆模型也只覆盖前进铺设。持续出缆时禁用负向和原地转向原语；非铺设机动必须由独立任务状态处理，不能沿用本模型宣称落点约束有效。

#### 8.3.1 原语内软成本积分与硬约束扫掠

中点近似只允许用于沿原语平滑变化且实现明确选择中点的软成本。对于原语 $e$ 上的代价密度 $c(s)$，一种允许的近似为：

$$
J_e \approx c(s_{\text{mid}})\,L_e
$$

其中 $L_e$ 为原语弧长。当前 `HybridAStarPlanner` 仅对机器人去趋势粗糙度代价使用原语中点；触地点走廊代价对相邻走廊样本的代价密度取梯形平均，并乘对应**机器人原语弧长**增量；缆线机械/地形软代价由 `CableLayingEvaluator` 对完整触地点段累计。碰撞、绝对缆线走廊、坡度、台阶、落差、禁放区和机械约束等硬门禁不得只在中点检查。

所有硬约束采用**自适应离散扫掠检查**，不宣称进行了数学意义上的连续碰撞检测。设地图栅格分辨率为 $r$，机器人位姿参考点到 footprint 最远点的距离为 $R_f$，原语总航向变化为 $\Delta\theta_e$，取 $0<\eta\leq0.5$，原语的检查区间数至少为：

$$
N_e=
\left\lceil
\frac{L_e+R_f|\Delta\theta_e|}{\eta r}
\right\rceil
$$

在 $i=0,\ldots,N_e$ 的 $N_e+1$ 个姿态检查硬约束，必须包含原语起点和终点。相邻检查姿态应满足：

$$
\Delta s_i+R_f|\Delta\theta_i|\leq\eta r
$$

各类硬约束分别吸收采样间隙，不共用未经论证的裕度：

1. **碰撞**：各采样姿态检查 $F\oplus B(\rho_{\text{sweep}})$ 覆盖的全部栅格，不得只检查机器人中心、边界顶点或 footprint 中点。首版取 $\rho_{\text{sweep}}=\eta r/2$ 作为扫掠离散化补偿；该项与定位误差、地图误差的障碍膨胀分别记录。
2. **绝对缆线走廊**：在各采样点按第7.4节连续传播候选原语的缆线状态，并要求

   $$
   |\mu_\perp(s_i)|+z_{1-\epsilon_{\text{point}}/2}\bar\sigma_\perp(s_i)
   \leq d_{\max}-\rho_{\text{cable,sweep}}
   $$

   $\rho_{\text{cable,sweep}}$ 是触地点轨迹与 $d_{\text{upper}}$ 传播的独立离散化裕度，必须通过模型上界或验证实验确定，首版暂记为 TBD；不得未经证明令其等于 $\rho_{\text{sweep}}$。
3. **坡度、台阶和落差**：在各采样姿态评价完整 footprint、左右履带支撑区及原语与台阶边缘的相交过程，并使用各自的测量和离散化裕度。
4. **缆线机械保护**：将父标签的 $\mathcal M_{\text{lay}}$ 与新触地点段一起传给 `CableLayingEvaluator`，检查 $|\kappa_{\text{cable}}|\leq\kappa_{\text{cable}}^{\max}$、禁放区、地形数据有效性和固定物理窗口的悬空代理，并取得后继标签的规范化 `terminal_memory`。曲率检查不得在原语边界重置三点差分；历史与当前段组成的对称曲率窗口只有在转角右侧达到 $L_{\kappa,\text{eval}}$ 后才结算，窗口不足的转角随 `terminal_memory` 延迟到后续原语。

若任何采样点或其保守裕度检查违反硬约束，整个运动原语立即判为无效。碰撞扫掠对每个采样姿态覆盖完整 footprint 栅格；缆线禁放、未知、低置信和地形查询对相邻触地点段使用栅格 supercover，不能只检查离散端点。原语采样分辨率改变时，软成本和硬约束判定应在规定容差内保持一致。这里的“连续扫掠”指自适应离散和保守裕度组合，不声称数学连续碰撞检测。

### 8.4 代价函数设计

安全性由本节之前定义的硬约束、直接剪枝和候选验证门禁保证。在满足全部硬约束的可行域内，以缆线落点质量为主要优化目标，并由下述现有加权代价同时兼顾路线效率、曲率平滑性与地形适宜性；各软目标之间不采用严格字典序比较。

当前 `HybridAStarCostComponents` 固定发布五个非负分量：`robot_length`、`robot_curvature`、`touchdown_corridor`、`cable_suitability` 和 `robot_terrain`。`solution_cost` 等于这五项之和；软成本只在作业区、碰撞、通行性、缆线模型、参考关联、统计包络/走廊及机械硬门禁全部通过后累计。

**总代价**：

$$
g(\mathbf{x}) = g_{\text{path}}(\mathbf{x}) + g_{\text{touchdown}}(\mathbf{x}) + g_{\text{terrain}}(\mathbf{x})
$$

**路径成本** $g_{\text{path}}$：

$$
g_{\text{path}} = w_{\text{length}} \cdot \Delta s + w_{\text{curvature}} \cdot |\kappa| \cdot \Delta s
$$

- 鼓励短路径和低曲率

**触地点目标代价** $g_{\text{touchdown}}$：

$$
g_{\text{touchdown},e}=J_{\text{corridor},e}+J_{\text{cable},e}
$$

$$
J_{\text{corridor},e}=\sum_{i=1}^{n}
\frac{c_{i-1}+c_i}{2}
\left(s^{\text{robot}}_i-s^{\text{robot}}_{i-1}\right),
\qquad
J_{\text{cable},e}=\texttt{CableLayingEvaluation.soft\_cost}
$$

其中 $c_{\text{td,corridor}}$ 使用第6.6节的 $\mu_\perp$、风险裕量和任务进度：在 `nominal_half_width_m` 内按 `touchdown_center_cost_weight * mean_lateral_error_m^2`，在名义边界外按归一化剩余裕量的平方和 `touchdown_margin_cost_weight` 计价；相邻样本密度取平均后乘机器人弧长间隔。$\ell_{\text{cable,suitability}}$ 是 `CableLayingEvaluator.soft_cost` 已完成物理区间积分的结果，不能由搜索端再次乘原语长度。机器人中心到参考线的距离不进入该式。

**地形代价** $g_{\text{terrain}}$：

$$
g_{\text{terrain},e} =
w_{\text{robot-terrain}} \cdot \sigma_{\text{rough}}(s_{\text{mid}})\cdot L_e
$$

路径长度与曲率项解析乘 $L_e$；机器人粗糙度项按中点乘 $L_e$；触地点走廊和缆线适宜性按各自已经积分的弧长区间累加。不得对已积分的 `laying.soft_cost` 再乘一次 $L_e$，也不得将 `MergeGoal.soft_cost` 重复加入搜索路径成本。

**硬约束**：如果违反硬约束，返回无穷大代价或直接剪枝：

$$
g(\mathbf{x}) = 
\begin{cases}
\infty & \text{if robot/cable hard constraint violation} \\
g_{\text{path}} + g_{\text{touchdown}} + g_{\text{terrain}} & \text{otherwise}
\end{cases}
$$

### 8.5 启发函数设计

**目标**：提供不混淆机器人中心线和缆线参考线的搜索下界。

**启发值**：

$$
h(\mathbf{x}) = w_{\text{length}}\min_{g\in\mathcal G}
\min\left(
d_{\text{Dubins}}(q,g_{\text{robot}}),
\max\left(d_{xy}(q,g)-\epsilon_p,
R_{\min}(|\Delta\theta_g|-\epsilon_\theta),0\right)
\right)
$$

其中第二项是到机器人目标位置/航向容差集合的运动学下界，负余量截为零；六族前进 Dubins 精确目标距离与容差域下界取较小值，再在全部并线目标间取最小。滞后角、参考进度、触地点和其他非负软成本不进入启发值，因此当前启发只声明为机器人路径长度分量的可接受下界。

开放队列按 `(g+h, h, g, label_id, node_id)` 升序确定性排序；不存在触地点偏差或目标生成软成本的隐藏 tie-breaker。若未来使用 Weighted A*，必须显式配置权重并在结果中标记次优界，不能通过任意增大“参考线启发权重”隐式改变算法语义。

### 8.6 搜索算法流程

**伪代码**：

```
function HybridAStarSearch(
    start,
    initial_cable_state,
    search_cable_context,
    terrain_gradient_risk_policy,
    corridor_risk_policy,
    goal_candidates,
    map,
    robot_operating_area,
    reference_line,
    initial_progress,
    locked_uncertainty_envelope
):
    open_set = PriorityQueue()
    label_store = CableAwareLabelStore(
        memory_equivalence=cable_laying_evaluator.future_equivalent,
        global_active_label_budget=maximum_active_labels
    )
    epsilon_g = configured_numeric_tolerance()
    search_started_at = steady_clock.now()

    envelope_dependencies = locked_uncertainty_envelope.envelope.dependencies
    assert locked_uncertainty_envelope.envelope_version == (
        search_cable_context.uncertainty_envelope_version
    )
    assert envelope_dependencies.reference_line_version == reference_line.version
    assert envelope_dependencies.sensor_mode == search_cable_context.sensor_mode
    assert envelope_dependencies.cable_model_version == cable_model.version()
    assert envelope_dependencies.execution_operating_envelope_version == (
        search_cable_context.execution_envelope.version
    )
    assert envelope_dependencies.operating_domain_id == (
        search_cable_context.execution_envelope.operating_domain_id
    )
    assert terrain_gradient_risk_policy.isReady()
    assert terrain_gradient_risk_policy.terrain_analysis_config_version == (
        terrain_layers.analysis_config_version
    )
    assert terrain_gradient_risk_policy.operating_domain_id == (
        terrain_layers.operating_domain_id
    )

    initial_envelope_query = uncertainty_envelope_manager.query(
        locked_uncertainty_envelope,
        initial_progress.arc_length_m,
        planning_timestamp
    )
    if initial_envelope_query.status != VALID:
        return NO_SOLUTION_UNDER_COVARIANCE_ENVELOPE

    initial_touchdown = cable_model.predict_touchdown_mean(
        start,
        initial_cable_state.lag_angle
    )
    if initial_touchdown.validity != VALID:
        return INPUT_INVALID

    initial_reference = reference_line.query(initial_progress.arc_length_m)
    if initial_reference is None:
        return NO_SOLUTION_UNDER_COVARIANCE_ENVELOPE
    initial_corridor_bound = evaluate_search_corridor_bound(
        corridor_risk_policy,
        initial_reference,
        initial_touchdown.touchdown_position_m,
        initial_envelope_query.lateral_stddev_upper_bound_m,
        rho_cable_sweep
    )
    if (initial_corridor_bound.validity != VALID or
            not initial_corridor_bound.hard_feasible):
        return NO_SOLUTION_UNDER_COVARIANCE_ENVELOPE

    collision_layer = traversability_evaluator.evaluate_collision_layer(
        map, terrain_layers, robot_collision_risk_policy
    )
    initial_sweep = MotionSegment([start])
    if collision_layer.invalid or initial_sweep.dependencies_invalid:
        return INPUT_INVALID
    if (not robot_operating_area.contains_footprint_with_clearance(initial_sweep)
            or not collision_free(initial_sweep, collision_layer)
            or not traversable(initial_sweep, terrain_layers)):
        return NO_SOLUTION

    initial_memory = cable_laying_evaluator.canonicalize_memory(
        initial_cable_state.laying_memory,
        cable_laying_limits,
        cable_history_boundary
    )
    if initial_memory is None:
        return INPUT_INVALID
    
    augmented_start = AugmentedState(
        start,
        initial_cable_state.lag_angle,
        initial_progress
    )
    start_cable_state = initial_cable_state.mean_only()
    start_cable_state.laying_memory = initial_memory
    start_node = Node(
        augmented_start,
        cable_state=start_cable_state,
        g=0,
        h=heuristic(augmented_start)
    )
    label_store.insert_initial(discretize(augmented_start), start_node)
    open_set.push(
        start_node,
        priority=(start_node.g + start_node.h, start_node.h, start_node.g,
                  start_node.label_id, start_node.node_id)
    )
    
    while not open_set.empty() and expanded_count < maximum_expansions:
        if steady_clock.now() - search_started_at >= maximum_planning_duration:
            return TIMEOUT(reason="HYBRID_ASTAR_DEADLINE_EXCEEDED")
        current = open_set.pop()
        
        # 被同一机械记忆等价类中的低代价标签支配后，旧队列项失活
        if not label_store.is_active(current.label_id):
            continue

        expanded_count += 1
        
        # 检查是否到达任一候选目标
        if is_goal(current, goal_candidates):
            return reconstruct_path(current)
        
        # 普通原语加周期性解析候选；解析候选只取到每个目标的
        # 六族前进Dubins最短路径的下一个非零段，后续轮次继续连接
        expansion_candidates = motion_primitives
        if analytic_expansion_interval > 0 and (
                expanded_count % analytic_expansion_interval == 0):
            for goal in goal_candidates:
                analytic_attempt_count += 1
                next_segment = next_forward_dubins_segment(current, goal)
                if next_segment is not None:
                    expansion_candidates.append(next_segment)

        for motion in expansion_candidates:
            if steady_clock.now() - search_started_at >= maximum_planning_duration:
                return TIMEOUT(reason="HYBRID_ASTAR_DEADLINE_EXCEEDED")
            robot_segment = apply_motion(current.state.robot_pose, motion)
            
            # 硬约束检查
            if not robot_operating_area.contains_swept_footprint(robot_segment):
                continue
            if not is_valid(robot_segment, map):
                continue
            
            # 方向相关的通行性检查(见5.7节)
            traversability = traversability_evaluator.evaluate(
                robot_segment, terrain_layers, terrain_gradient_risk_policy
            )
            if not traversability.traversable:
                continue

            # 使用同一触地点模型传播缆线状态和整段触地点轨迹
            cable_prediction = cable_model.predict_search(
                current.cable_state.dynamic_mean(),
                robot_segment,
                search_cable_context
            )

            if cable_prediction.validity != VALID:
                continue

            progress_profile = update_reference_progress_locally(
                current.state.s_prog,
                cable_prediction.touchdown_path,
                cable_prediction.state_profile,
                reference_line,
                forward_limit=alpha_s * motion.length + epsilon_s,
                backward_tolerance=epsilon_back
            )
            if not progress_profile.valid:
                continue

            corridor_profile = []
            corridor_valid = True
            for touchdown, progress in zip(
                    cable_prediction.touchdown_path.points,
                    progress_profile.values):
                envelope_query = uncertainty_envelope_manager.query(
                    locked_uncertainty_envelope,
                    progress.arc_length_m,
                    planning_timestamp
                )
                reference_point = reference_line.query(progress.arc_length_m)
                if envelope_query.status != VALID or reference_point is None:
                    corridor_valid = False
                    break
                corridor_step = evaluate_search_corridor_bound(
                    corridor_risk_policy,
                    reference_point,
                    Vector2m(touchdown.x_m, touchdown.y_m),
                    envelope_query.lateral_stddev_upper_bound_m,
                    rho_cable_sweep
                )
                if (corridor_step.validity != VALID or
                        not corridor_step.hard_feasible):
                    corridor_valid = False
                    break
                corridor_profile.append(corridor_step)
            if not corridor_valid:
                continue

            laying_step = cable_laying_evaluator.evaluate_segment(
                current.cable_state.laying_memory,
                cable_prediction.touchdown_path,
                cable_prediction.state_profile,
                terrain_layers,
                cable_laying_limits,
                cable_history_boundary
            )
            if not laying_step.valid or not laying_step.hard_feasible:
                continue

            next_cable_state = cable_prediction.terminal_state.mean_only()
            next_cable_state.laying_memory = laying_step.terminal_memory

            next_state = AugmentedState(
                robot_segment.end_pose,
                cable_prediction.terminal_state.lag_angle,
                progress_profile.terminal_progress
            )

            # 机器人和触地点代价均按本运动原语累计
            g_new = current.g + cost(
                robot_segment, cable_prediction, corridor_profile,
                laying_step.soft_cost, reference_line
            )
            h_new = heuristic(next_state, goal_candidates)

            next_node = Node(
                next_state,
                cable_state=next_cable_state,
                g=g_new,
                h=h_new,
                parent=current
            )
            insertion = label_store.try_insert(
                discretize(next_state),
                next_node,
                epsilon_g
            )
            if insertion == RESOURCE_LIMIT:
                return TIMEOUT(
                    reason="HYBRID_ASTAR_ACTIVE_LABEL_BUDGET_EXHAUSTED"
                )
            if insertion == ACCEPTED:
                open_set.push(
                    next_node,
                    priority=(g_new + h_new, h_new, g_new,
                              next_node.label_id, next_node.node_id)
                )

    if not open_set.empty() and expanded_count >= maximum_expansions:
        return TIMEOUT(reason="HYBRID_ASTAR_EXPANSION_BUDGET_EXHAUSTED")
    if envelope_was_the_only_recorded_hard_failure:
        return NO_SOLUTION_UNDER_COVARIANCE_ENVELOPE
    return NO_SOLUTION
```

公共入口是 `HybridAStarPlanner::plan(const HybridAStarPlanningRequest&)`。它在扩展前验证非空目标、有限值、同步起点时间、参考线/模型/地图/分析配置/风险策略/作业区/锁定包络的依赖版本和运行域；结构或数值无效与依赖错配分别记录 `HYBRID_ASTAR_INPUT_INVALID`、`HYBRID_ASTAR_DEPENDENCY_MISMATCH`，状态均失败关闭为 `input_invalid`。随后依次检查起点包络查询、起点触地点走廊、完整机器人足迹和实际机械历史规范化，起点已经命中目标也不能绕过这些门禁。

每个普通或解析原语都走同一验证链：机器人作业区净距、版本化碰撞层与完整足迹扫掠、方向通行性、`CableModel::predict_search`、局部参考进度、逐样本统计包络和走廊、`CableLayingEvaluator`。解析候选不是一次性“直连成功”：当前实现每次只加入到各目标的六族前进 Dubins 最短路径的下一个非零恒曲率段，只有该段通过全部门禁才计入 `analytic_expansion_accepted_count`；后续节点可继续产生下一段，因而支持经逐段验证的多段解析连接。Reeds-Shepp、倒车解析段和绕过完整验证的 shortcut 均未实现。

只有 frontier 真正耗尽才返回无解。若统计包络不可用/走廊违规是唯一记录到的硬失败，返回 `no_solution_under_covariance_envelope`；与碰撞、作业区、通行性、模型、参考关联或机械失败混合时返回普通 `no_solution`。活动标签上限、扩展上限和单调时钟截止时间均返回 `timeout`，并分别记录 `HYBRID_ASTAR_ACTIVE_LABEL_BUDGET_EXHAUSTED`、`HYBRID_ASTAR_EXPANSION_BUDGET_EXHAUSTED` 或 `HYBRID_ASTAR_DEADLINE_EXCEEDED`，不得归类为无解。

### 8.7 多候选接入点

**问题**：单一固定终点可能导致无解或不优路径。

**解决方案**：先在参考线上生成多个缆线触地点目标，再通过同一触地点模型反解对应的机器人终端状态。搜索到达任一完整增广目标即可。

**当前接入点生成**：

1. 对 `merge_distances_m` 中每个正距离计算 $s_g=s_{\text{current}}+d_{\text{merge}}$，并查询锁定参考线的触地点目标 $(\mathbf c_g,\psi_{\text{ref},g},s_g)$
2. 取 `terminal_heading_offsets_rad` 与 `terminal_lag_angles_rad` 的笛卡尔积，形成 $(\theta_g,\delta_g)$；缆线目标航向固定为参考切向，滞后角必须位于模型标定域
3. 使用触地点模型反解放缆点和机器人中心：

   $$
   \mathbf p_{\text{release},g}=\mathbf c_g+L_{\text{td}}
   \begin{bmatrix}\cos\psi_{c,g}\\\sin\psi_{c,g}\end{bmatrix}
   $$

   $$
   \mathbf p_{\text{robot},g}=\mathbf p_{\text{release},g}-R(\theta_g)\mathbf p_r
   $$

4. 用同一个 `CableModel` 正向预测复核位置/航向闭环误差，再检查机器人完整终端足迹位于 $\mathcal W_{\text{robot}}$ 且终端地形可通行
5. 每个 `MergeGoal` 保存机器人位姿、$\delta_g$、$s_g$、触地点/缆线航向、并线距离、生成参数/模型/参考线/作业区版本和生成阶段软成本
6. 生成阶段软成本仅由 `merge_distance_cost_weight * merge_distance_m` 与 `absolute_lag_cost_weight * abs(lag_angle_rad)` 组成；稳定升序排序后由 `maximum_goal_count` 截断

不得直接令 $\mathbf p_{\text{robot},g}=\mathbf c_g$。即使直线铺设，放缆点偏置和 $L_{\text{td}}$ 也通常使两者不同。

`MergeGoalGenerationResult` 对每次尝试保留结构化拒绝原因：`invalid_input`、`reference_version_mismatch`、`merge_progress_outside_reference`、`lag_angle_out_of_range`、`forward_model_mismatch`、`robot_footprint_outside_area`、`terrain_evaluation_invalid`、`terminal_terrain_not_traversable` 和 `goal_limit_reached`。输入/参考版本错误失败关闭；地形评价依赖无效会清空已生成目标并返回 `valid_input=false`；单个候选越界或不可行只拒绝该候选。结果同时审计生成参数、缆线模型、参考线、作业区、标定数据集、运行域、坐标系及地形梯度风险。

**目标判定**：

$$
\begin{aligned}
\text{is\_goal}(\mathbf{x},\mathcal G)=\exists g\in\mathcal G:\;&
\|\mathbf p_{\text{robot}}-\mathbf p_{\text{robot},g}\|\leq\epsilon_p\\
&\land |\operatorname{wrap}(\theta-\theta_g)|\leq\epsilon_\theta\\
&\land |\operatorname{wrap}(\delta-\delta_g)|\leq\epsilon_\delta\\
&\land |s_{\text{prog}}-s_g|\leq\epsilon_s\\
&\land \|\mathbf c(\mathbf x)-\mathbf c_g\|\leq\epsilon_c
\end{aligned}
$$

最终目标判定读取预测触地点，不以机器人到参考点的距离替代。候选经离散容差命中后，仍需执行第9节平滑和完整缆线复检。

**选择策略**：`MergeGoal.soft_cost` 只决定生成结果的稳定排序和数量截断，不加入 `HybridAStarCostComponents`，避免并线距离/滞后偏好被重复计权。搜索在全部目标的联合目标集合上按第8.4节五分量路径代价工作；命中任一完整增广目标才成功。缆线落点质量是主要软优化目标，同时兼顾路线效率、曲率平滑性与地形适宜性，不对这些软目标执行严格字典序选择。

### 8.8 搜索优化

**前进 Dubins 解析扩展**：
- `analytic_expansion_interval == 0` 时禁用；否则按已扩展节点计数周期性地对每个目标尝试六族前进 Dubins 最短连接
- 每次尝试只返回下一个非零直线/左转/右转段，曲率为 $0$ 或 $\pm1/R_{\min}$；多段连接通过后续节点继续生成
- 解析段与普通原语共用自适应采样、版本化地图、机器人/缆线全部硬门禁和多标签插入规则；中段障碍或其他失败会拒绝该段
- 当前没有 Reeds-Shepp、倒车、增量地图搜索、自适应状态分辨率或早停优化，不得把未来优化写成现状能力

**Hybrid 多标签剪枝**：
- 对每个离散基础键 $(i_x,i_y,i_\theta,i_\delta,i_s)$ 保存一个活动标签集合，每个标签携带自己的 `CableConstraintMemory`
- `canonical_signature` 只用于缩小候选集合；命中后仍逐样本调用 `future_equivalent`，不同签名也执行 fallback 比较，以处理陈旧签名、有符号零或哈希碰撞
- 只有机械记忆经 `future_equivalent` 证明等价时，才用 $g_{\text{new}}<g_{\text{old}}-\epsilon_g$ 替换旧标签；已有标签满足 $g_{\text{old}}\leq g_{\text{new}}+\epsilon_g$ 时丢弃新标签，不等价标签即使当前代价更高也不得据此删除
- 被等价低代价标签替换的队列项标记为失活，弹出时通过 `label_id` 跳过；低代价标签仍可重开对应等价类
- 活动标签达到全局 `maximum_active_labels` 时返回 `TIMEOUT` 及标签统计，不通过固定每基础键数量或“只留前 K 个”静默剪枝

**资源与确定性诊断**：
- `maximum_expansions`、`maximum_active_labels` 和 `maximum_planning_duration_s` 是三个独立硬预算；截止时间使用注入的单调稳态时钟，检查发生在弹出节点前及每个后继前
- 当前没有独立的最大字节数参数；内存通过活动标签数间接限制，并报告 `fixed_bytes_per_search_label`、`peak_observed_bytes_per_search_label`、活动标签峰值及每基础键标签 P50/P95/P99，平台预算必须用这些证据反推 `maximum_active_labels`
- 诊断逐类累计模型、参考关联、包络、走廊、作业区、碰撞、通行性和机械拒绝数，记录最坏硬约束、五个解成本分量、解析尝试/通过数、等价丢弃/替换、旧队列跳过和固定队列规则
- 结果指纹覆盖状态、版本/运行域/点风险语义、预算与诊断、机器人/触地点路径、公开状态轨迹、终端滞后角和终端参考进度；相同输入、固定时钟行为和参数下的完整结果仍必须由测试逐字段复现

---

## 9. 路径平滑、独立几何复检与时序参数化

### 9.1 处理边界与 G2 目标

Hybrid A* 返回的 `GeometricPath` 由运动原语拼接而成，可能存在曲率跳变或局部次优。当前处理链把三个职责明确分开：`PathSmoother` 只生成并审计曲率受限的几何候选，`PathCandidateVerifier` 使用当前地图和机器人能力独立复检完整机器人路径，`TrajectoryParameterizer` 在不改变几何的前提下生成执行剖面。任何一步成功都不能替代后续安全门禁。

路径点的公共契约为 $(s,x,y,\theta,\kappa)$。目标是位置、切向航向和曲率连续的 **G2** 几何，并满足平台允许的空间曲率变化率 $u=d\kappa/ds$。`PathSmoother::smooth` 与公共 `auditPathGeometry` / `PathCandidateVerifier` 的起点曲率都只能来自同步实际状态或获批承诺段终端；其起点 `PathBoundary` 必须携带非零来源序列、非负位姿/曲率单调时间戳和允许来源，两时间戳之差不得超过 `maximum_boundary_time_skew`。曲率缺失、来源为 `planned_goal` 或时间不同步时，平滑入口返回 `boundary_state_invalid`，独立复检入口返回 `input_invalid`，均不得默认置零或进入地图扫掠。终点边界同样必须显式携带有限曲率，并继续允许 `planned_goal` 来源。

### 9.2 当前分段 clothoid 实现

当前默认求解器版本为 `path-smoother/clothoid-v1`。输入搜索路径必须通过 `GeometricPath` 公共校验，并声明 `constant-curvature-exact` 插值；输出声明 `clothoid-linear-curvature/v1`。分段状态为：

$$
\mathbf q(s)=(x(s),y(s),\theta(s),\kappa(s)),\qquad
u(s)=\frac{d\kappa}{ds}
$$

$$
\frac{dx}{ds}=\cos\theta,\qquad
\frac{dy}{ds}=\sin\theta,\qquad
\frac{d\theta}{ds}=\kappa,\qquad
\frac{d\kappa}{ds}=u
$$

每段的 $u_i$ 为常量。默认实现按 `ceil(total_length / spatial_step_m)` 生成不超过 512 段的等长离散；段长必须不小于 `minimum_segment_length_m`。初始 $u_i$ 由原始路径曲率插值得到，再投影到 $|u_i|\leq u_{\max}$ 和 $|\kappa_i|\leq\kappa_{\max}$。求解先迭代修正终端位置、航向和曲率，再在保持终端容差、曲率门禁和拓扑管的前提下优化目标。clothoid 位置积分使用带四阶导数误差上界的复合 Simpson 积分；当前单坐标误差预算为 $10^{-10}$ m，超过实现的积分区间上限时失败关闭。

`PathSmoothingSolver` 是可注入的公共求解器边界，测试可用它构造假收敛或超时；无论求解器实现为何，`PathSmoother` 都必须独立重算残差后才可能返回成功。

### 9.3 目标、硬约束与求解状态

当前目标为：

$$
J=w_{\text{deviation}}J_{\text{deviation}}
+w_{\kappa}J_{\kappa}
+w_uJ_u
+w_{\text{length}}J_{\text{length}}
$$

其中偏差、曲率平方、曲率变化率平方和长度均按弧长积分。权重必须有限、非负且至少一项为正。平滑目标不包含机器人中心到缆线参考线的代理项；`reference_line_proxy` 审计值固定为零。机器人路径到原始 `constant-curvature-exact` 曲线的拓扑距离按原始圆弧而非端点弦计算，候选在段内按不大于半个拓扑管半径的间距检查并加入未采样弧长裕量。

硬门禁包括：

1. 分段 clothoid 动力学残差、严格递增弧长和最小段长；
2. 正负方向对称的 $|\kappa|\leq\kappa_{\max}$ 与 $|u|\leq u_{\max}$；
3. 起终位置、航向和曲率残差；
4. 候选不得离开原始路径的版本化拓扑管；
5. `SmoothingLimits` 的版本、输出路径版本、边界同步容差、残差预算、目标权重和正的阶段超时必须完整且有限。

`SmoothingStatus` 的当前状态集合为 `success`、`boundary_state_invalid`、`seed_infeasible`、`solver_timeout`、`solver_failed`、`constraint_residual_exceeded` 和 `trackability_validation_failed`。默认求解器的同一 `steady_clock` 绝对截止时间覆盖终端修正和目标优化的全部有界循环；超时保留为 `solver_timeout`，不返回部分路径。求解器声称成功但候选残差、拓扑管或公共几何无效时分别返回残差超限或可跟踪性失败，不得伪装为成功。

### 9.4 独立几何审计

平滑器输出的 $\kappa_i$ 是路径状态，不能在发布前用相邻位置点覆盖。`PathCandidateVerifier` 先调用 `auditPathGeometry`，从位置独立重算切向航向、内部点的三点有符号曲率以及相邻样本的最大曲率变化率：

$$
\kappa_{\text{geom},i}=
\frac{2(\mathbf v_1\times\mathbf v_2)}
{\|\mathbf v_1\|\,\|\mathbf v_2\|\,\|\mathbf v_1+\mathbf v_2\|}
$$

$$
|\kappa_{\text{geom},i}-\kappa_i|\leq\epsilon_{\kappa},\qquad
\left|\frac{\kappa_{i+1}-\kappa_i}{\Delta s_i}\right|
\leq u_{\max}+\epsilon_u
$$

审计还检查起终 G2 残差、元数据曲率绝对上限、航向残差、非有限值、退化切向、退化三点曲率和过短段，并记录已计算的航向、曲率和曲率变化率最坏索引与位置。该审计独立于 `PathSmoothingMetadata`；修改路径点曲率元数据不能绕过三点几何残差门禁。`auditPathGeometry` 对起点来源执行显式白名单，只接受 `synchronized_actual_state` 或 `committed_segment_terminal`；`planned_goal` 或未知来源返回 `start_boundary_provenance_invalid`。起点曲率缺失、零来源序列、负位姿/曲率时间返回输入无效，时间差超过 `maximum_boundary_time_skew` 返回 `boundary_timestamp_skew_exceeded`。这些失败在边界残差计算和任何作业区、碰撞或通行性扫掠前结束；同一个目标 `PathBoundary` 仍允许 `planned_goal`，该起点门禁不得套用于终点。

`mergePathsG2` 只接受坐标系、参考线版本和插值规则一致的两条有效路径，并同时检查拼接点的位置、航向和曲率残差。拼接形成新的几何对象后清除旧段的平滑审计，调用方必须对完整拼接结果重新审计。

### 9.5 当前处理流程

```python
smoothing = path_smoother.smooth(raw_path, start_boundary, goal_boundary,
                                 smoothing_limits)
if smoothing.status == success:
    candidate_geometry = smoothing.path
else:
    # 不自动放行原始路径；先走同一 PathSmoother 可跟踪性契约
    raw_check = path_smoother.validateTrackability(
        raw_path, raw_path, start_boundary, goal_boundary, smoothing_limits)
    if not raw_check.valid:
        return classify_smoothing_failure(smoothing.status)
    candidate_geometry = raw_path

tail = trajectory_parameterizer.parameterize(
    candidate_geometry, synchronized_execution_state,
    certified_execution_envelope, parameterization_limits)
if tail.status != success:
    return classify_parameterization_failure(tail.status)

complete_timed_path = merge_authorized_commitment_if_present(tail.trajectory)
robot_check = path_candidate_verifier.verify(
    complete_timed_path.geometry, complete_start_boundary,
    complete_goal_boundary, smoothing_limits, current_verification_context)
if not robot_check.valid:
    return reject_candidate()

cable_check = timed_cable_candidate_verifier.verify(complete_timed_path, ...)
```

平滑器本身不读取地图，也不执行足迹、碰撞或地形评价。这样可保证求解器成功、内部残差成功、当前地图完整路径复检和最终缆线复检是四份不能互相替代的证据。

### 9.6 当前地图上的完整机器人路径复检

`PathCandidateVerifier` 对平滑候选和原始回退候选使用同一公共入口。其顺序为：

1. 执行第9.4节的起点来源门禁、独立几何与 G2 边界审计；来源或边界元数据无效时返回 `input_invalid`，并保持碰撞/通行性扫掠计数为零；
2. 校验当前 `MapSnapshot`、`TerrainLayers`、机器人作业区、地形分析配置、坐标系及所有数值阈值；
3. 按 `maximum_sweep_spacing_fraction * map.resolution_m` 对完整路径各段加密，检查完整足迹及 `operating_area_clearance_m`；
4. 使用当前机器人能力、足迹、碰撞风险策略和机器人相对障碍协方差重建碰撞层，并对完整加密路径扫掠；
5. 使用当前地形梯度风险策略复检方向坡度、台阶、履带支撑、粗糙度和其他通行性硬门禁。

几何航向/曲率/曲率变化率失败在相应量已计算时保留最坏索引与位置；作业区、碰撞和通行性失败保留失败样本索引与位置。输入/上下文无效和 G2 边界残差失败只保证结构化 `issues`、`reason` 或边界残差，不保证存在有意义的失败位置。状态集合为 `input_invalid`、`geometry_invalid`、`boundary_residual_exceeded`、`curvature_limit_exceeded`、`curvature_rate_limit_exceeded`、`operating_area_violation`、`collision_violation`、`traversability_violation` 和 `valid`；只有 `valid` 才能进入缆线复检。缆线走廊、机械和统计包络不属于该类职责，必须由第7.7节的完整 `TimedPath` 复检另行给出证据。

### 9.7 时序参数化与执行剖面版本

`TrajectoryParameterizer` 只接受已选定的 `GeometricPath`、同步的 `TrajectoryInitialState`、版本化 `ExecutionOperatingEnvelope` 和 `TrajectoryParameterizationLimits`，不另收一份 `RobotCapability` 或 `TerrainLayers`。认证运行包络是速度、加减速度、横向加速度、最大停车距离、出缆速度/加速度、出缆跟踪误差和张力范围的唯一参数化能力边界；空间曲率及其变化率已经由几何阶段门禁，当前参数化器不宣称另有时域曲率变化跟踪模型。

参数化器逐字段原样复制输入几何，生成 `piecewise-linear-execution/v1` 的 `ExecutionProfile`。每个样本包含弧长、相对时间、对地速度/加速度、出缆速度/加速度和张力设定值；剖面还携带认证运行包络版本、停止点和完整 `approved_tracking_limits`。速度先按运行包络、$v^2|\kappa|$、前向加速和后向制动传播收紧。当前停车距离门禁使用认证最小地面加速度的制动幅值：

$$
d_{\text{required}}=\frac{v_0^2}{2|a_{\text{brake}}|}+d_{\text{margin}}
$$

当 `require_terminal_stop=true` 时，要求 $d_{\text{required}}$ 同时不超过剩余几何长度和认证 `maximum_stopping_distance_m`，并将终点速度压到零；认证制动能力非正、后向传播与同步初速冲突或终速超过请求上限均返回 `stopping_constraint_infeasible`。该公式是当前 Level 1 参数化实现，不替代第10节面向坡度、载荷和真实平台的外部认证停车模型。

出缆速度当前跟随逐点对地速度并裁剪到认证范围；出缆加速度、速度跟踪误差和张力设定值逐样本失败关闭。参数化器使用可注入单调时钟，正的单一绝对截止时间覆盖输入遍历、前向/后向传播、样本生成、最终公共契约校验及成功提交边界；到期返回 `deadline_exceeded` 且不携带部分 `TimedPath`。

`ParameterizationStatus` 的当前状态集合为 `success`、`deadline_exceeded`、`initial_state_invalid`、`execution_envelope_mismatch`、`dynamics_infeasible`、`payout_infeasible`、`stopping_constraint_infeasible` 和 `numerically_invalid`。只有 `success` 携带完整 `TimedPath`。剖面使用请求中的非零 `execution_profile_version`；若该字段为零但几何路径已有非零版本，当前实现回退到参数化限制版本。跨调用的语义版本管理由 `ExecutionProfileVersioner` 执行：完全相同内容保持版本，运行包络、插值、停止点、任一执行样本或批准跟踪限制改变都必须递增，改变内容复用旧版本或版本回退均无效。

主循环将 `solver_timeout` 映射为 `SMOOTHING_DEADLINE_EXCEEDED`，将 `deadline_exceeded` 映射为 `PARAMETERIZATION_DEADLINE_EXCEEDED`，两者进入 `PlanningState::timeout`。平滑超时或不可行时，只有原始路径通过同一可跟踪性契约且随后通过完整机器人/缆线复检才可继续，并保留原始平滑根因；边界无效属于输入无效。参数化的初始状态、包络或数值无效映射为输入无效，动力学、出缆或停车不可行映射为无解。任何失败结果都不能携带可执行轨迹；旧计划是否可继续只能在规划结束后的最新同步上下文中按第10节全量复检并取得新租约。

固定发布顺序为：几何搜索 → 平滑或原始路径同等可跟踪性检查 → 时序参数化 → 承诺段时序拼接 → 当前地图完整机器人路径复检 → 完整 `TimedPath` 缆线复检 → 构造不可变 `PlanningResult` → 最新同步上下文复检并签发租约。缆线验证后不得修改执行剖面；减速或放缆控制变化必须生成语义上正确的新剖面版本并重新走完整验证链。

---

## 10. 稳定重规划与路径滞回

### 10.1 问题

地图微小变化或多个等价路径的存在可能导致规划结果左右反复切换，引起：
- 控制器振荡
- 执行器磨损
- 缆线布放不连续

### 10.2 路径切换滞回

**滞回判据**：

仅当新路径显著优于当前路径时才切换。当前 `PathHysteresisConfig.relative_cost_threshold` 是相对阈值，默认值为 `0.1`，要求 $0\leq r_{\text{hys}}<1$：

$$
\text{switch} = 
\begin{cases}
\text{true} & \text{if } c_{\text{new}} < c_{\text{current}} - c_{\text{current}}r_{\text{hys}} \\
\text{false} & \text{otherwise}
\end{cases}
$$

当前路径、新路径或代价非有限，路径弧长不严格递增，代价为负，或阈值配置无效，独立的 `should_switch_path` 均返回“不因质量切换”。`decide_path_switch` 在任何候选直切或滞回比较之前先要求 `candidate_cost` 有限且非负；候选代价无效且当前计划本次复检有效时保持当前路径及其新租约，否则返回 `stop`。新旧计划都有效时，当前代价无效也只能保持当前路径，不能使候选获得发布权。`AuthorizedPlanningResultPublisher` 在原子发布边界重复执行同一有限非负门禁，防止错误适配器绕过决策层。

**安全前置条件**：滞回只比较已经在同一 `PlanValidationContext` 下通过完整验证的新旧剩余路径。判定顺序固定为：

1. 候选路径有效但候选代价非有限或为负：有本次复检有效的当前路径则保持，否则停车
2. 当前路径无有效租约或复检失败：禁止保持当前路径；候选路径及代价有效时直接切换，不执行代价滞回
3. 新路径复检失败：不得发布新路径；当前路径只有在独立复检通过并取得新租约后才可继续
4. 新旧路径均有效：才允许比较代价并应用滞回阈值

因此 `should_switch_path` 只处理质量稳定性，不拥有覆盖任何安全验证结果的权限。主循环在候选决策前只捕获一次最新 `SynchronizedValidationInputs`，以同一个 `decision_validation_at` 分别按 `publication_candidate` 和 `authorized_current` 角色复检候选与当前不可变计划；候选普通复检失败也必须先完成当前计划复检，再把两份 `PlanValidityEvaluation` 及对应审计代价交给 `decide_candidate`。`MainPlanningLoopStages::decide_candidate` 是基类非虚入口并实际委托该阶段基类持有的已配置 `StabilityManager`，所有具体阶段适配器均不能恢复无条件选择有效候选的策略，同时仍可在构造阶段注入经验证的相对代价和拓扑阈值配置。

新旧路径均满足硬约束后，滞回比较使用现有加权总代价及切换阈值；落点质量是主要软优化目标，但不会在数学意义上绝对优先于路线效率、曲率平滑性或地形适宜性。

**拓扑变化检测**：`topology_distance_threshold_m` 是可选的正有限标定值。未配置时，当前实现只使用相对代价滞回；配置后，新旧路径的对称 Hausdorff 距离严格大于阈值可触发切换，即使代价改善未越过相对阈值。该机制仍不能绕过完整复检和租约配对。

**拓扑距离**：当前实现使用路径采样点集合之间的对称 Hausdorff 距离判断路径相似度。

$$
d_{\text{topo}}(\mathcal{P}_1, \mathcal{P}_2) = \max \left\{ \max_{p \in \mathcal{P}_1} \min_{q \in \mathcal{P}_2} \|p - q\|, \max_{q \in \mathcal{P}_2} \min_{p \in \mathcal{P}_1} \|p - q\| \right\}
$$

如果已配置 $d_{\text{topo}}^{\max}$ 且 $d_{\text{topo}} > d_{\text{topo}}^{\max}$，认为拓扑不同。

### 10.3 近端承诺段

**目的**：避免修改机器人当前正在跟踪或无法及时响应的路径段。

**承诺段定义**：

从当前机器人位置向前延伸的固定距离或时间窗口内的路径不可修改。

$$
L_{\text{commit}} = \max\left(
v_{\text{robot}}t_{\text{commit}},
d_{\text{stop,lease}}+d_{\text{safety\_margin}}
\right)
$$

其中：
- $v_{\text{robot}}$：当前非负对地速度；它必须处于获批速度范围内，并与当前授权剖面在机器人投影进度处的插值速度一致
- $t_{\text{commit}}$：承诺时间（如 2-5 秒，TBD）
- $d_{\text{stop,lease}}$：按当前获批剖面、坡度、载荷和制动能力计算的承诺区间最坏停车距离

若认证最坏停车距离缺失/非有限、当前速度未与授权剖面同步，或当前已验证剩余轨迹不足以覆盖该长度，`extract_commitment_segment` 分别返回 `stopping_distance_unavailable`、`input_invalid` 或 `authorization_range_insufficient`，不能通过缩短承诺段继续运动；应拒绝签发运动租约或生成可在现有安全距离内停车的新剖面。承诺段只能从已授权剩余 `TimedPath` 中插值裁剪，不能越过其终点；原授权停止点可位于所提取的移动前缀之外，不代表该前缀独立获得新的停车授权。

**实现方式**：

新规划的路径必须在承诺段端点处与当前路径满足 G2 拼接契约。承诺段切片必须保留终端航向和已发布曲率：

```python
def plan_with_commitment(current_trajectory, robot_state):
    # 计算承诺段
    s_robot = current_position_on_path(current_trajectory.geometry, robot_state)
    s_commit = s_robot + L_commit
    commit_trajectory = current_trajectory.slice(s_robot, s_commit)
    commit_segment = commit_trajectory.geometry
    
    # 从承诺段终点开始规划
    start_for_new_plan = commit_segment.end_boundary_g2()
    new_path = hybrid_a_star_search(start_for_new_plan, ...)

    smoothing = path_smoother.smooth(
        new_path,
        start_boundary=start_for_new_plan,
        goal_boundary=new_path.goal_boundary_g2(),
        ...
    )
    if smoothing.status == success:
        candidate_tail = smoothing.path
    else:
        raw_check = path_smoother.validateTrackability(
            new_path, new_path, start_for_new_plan,
            new_path.goal_boundary_g2(), ACTIVE_SMOOTHING_LIMITS)
        if not raw_check.valid:
            return classify_smoothing_failure(smoothing.status)
        candidate_tail = new_path

    timed_tail = trajectory_parameterizer.parameterize(candidate_tail, ...)
    if timed_tail.status != success:
        return classify_parameterization_failure(timed_tail.status)

    merge = stability_manager.merge_timed_paths(
        commit_trajectory, timed_tail.trajectory,
        g2_join_tolerances, execution_join_tolerances,
        final_verifier=verify_complete_timed_path)
    if not merge.valid:
        return COMMITMENT_MERGE_FAILED

    # 合并器保留承诺段原剖面，以 max(prefix, tail)+1 生成新剖面版本；
    # 缺少完整路径最终验证器或验证失败时不返回轨迹。
    return merge.trajectory
```

#### 10.3.1 安全事件覆盖规则

**安全覆盖原则**：安全硬约束和安全事件拥有覆盖权；只有在安全可行的前提下，才讨论可实现性、稳定性和加权路径质量。

承诺段机制的目的是保证路径稳定性,但**不得**优先于安全性。

**可打破承诺段的安全事件**：

以下事件具有覆盖承诺段的权限:

1. **紧急停车**：
   - 人工急停指令
   - 系统严重故障(定位失效、通信中断、电源异常)
   - 传感器关键失效

2. **承诺段内新障碍**：
   - 地图更新发现承诺段内存在碰撞风险
   - 新检测到的障碍物

3. **定位异常**：
   - 定位置信度低于安全阈值: $\text{confidence}_{\text{loc}} < \text{confidence}_{\text{critical}}$
   - 位置跳变超过阈值: $\|\mathbf{x}_{\text{new}} - \mathbf{x}_{\text{expected}}\| > \Delta x_{\text{jump}}^{\max}$

4. **地形约束骤变**：
   - 高分辨率地图更新发现承诺段内存在不可通行地形
   - 地图置信度骤降

5. **缆线、执行与依赖异常**：
   - 当前缆线状态或完整时序缆线复检异常
   - 对地/出缆速度、加速度或张力执行偏差
   - 任一规划依赖版本变化
   - 机器人硬约束复检失败或完整验证证据不可用

`CommitmentSafetyEvaluator` 对完整机器人路径验证、完整时序缆线验证和异步事件统一排序，固定动作为 `STOP > REPLAN_URGENT > CONTINUE`；同动作内按类型化事件优先级确定唯一原因。没有异步事件的常规检查必须同时具有机器人和缆线完整验证，缺一即 `validation_unavailable/STOP`。异步事件适配器则可不等待规划周期或完整验证返回，直接触发撤租和安全通道。

**决策与执行顺序**：

```python
observation = CommitmentSafetyObservation(
    robot_validation=verify_complete_robot_path(commitment_trajectory),
    cable_validation=verify_complete_timed_cable(commitment_trajectory),
    observed_event=latest_async_event,
    obstacle_stopping=certified_obstacle_stopping_evidence
)
safety = commitment_safety_evaluator.evaluate(observation)

if not safety.is_safe:
    # supervisor 先把租约持久撤销，再调用独立安全停车通道；
    # STOP 与 REPLAN_URGENT 均不得在本周期进入搜索或继续旧承诺。
    enforcement = commitment_safety_supervisor.enforce(
        safety, active_lease_sequence, now)
    current_authorization = None
    if safety.action == REPLAN_URGENT:
        request_fresh_synchronized_replan_next_cycle()
    continue
```

`new_obstacle` 只有在认证停车距离严格小于障碍距离时才是 `REPLAN_URGENT`；证据缺失、非有限、为负或两者相等时均为 `STOP`。`CommitmentSafetySupervisor` 构造时必须具有独立安全停车通道，且 `revokeLease` 的持久撤销必须先于通道回调。下一周期若输入仍携带同一已撤销租约，主循环忽略该承诺起点并从最新同步实测状态重新规划。

**普通事件不打破承诺段**：

以下情况仍保持承诺段不变:
- 地图优化更新(无新障碍或约束变化)
- 优化型重规划(寻找更优路径)
- 参考路线微调
- 缆线落点预测更新(未触发硬约束违反)

这些情况下,新路径仍从承诺段末端开始规划。

**停车距离预留**：

承诺段长度 $L_{\text{commit}}$ 应满足：

$$
L_{\text{commit}} \geq d_{\text{stop,lease}} + d_{\text{safety\_margin}}
$$

其中:
- $d_{\text{stop,lease}}$：租约必须携带当前获批剖面在坡度、载荷和制动能力下的认证最坏停车距离。当前 Level 1 `TrajectoryParameterizer` 只按 `v^2/(2|a_{\text{brake}}|)+margin` 及认证包络的最大停车距离做合成门禁，尚未实现或验证坡度/载荷相关停车模型，因此该诊断值在外部标定完成前不能冒充生产租约的认证停车距离
- $d_{\text{safety\_margin}}$：安全裕量(TBD,建议2-5m)

确保即使在承诺段末端发现危险,仍有足够距离停车。

### 10.4 滚动窗口管理

**窗口大小**：

$$
L_{\text{window}} = L_{\text{commit}} + L_{\text{plan}} + L_{\text{buffer}}
$$

其中：
- $L_{\text{commit}}$：承诺段长度
- $L_{\text{plan}}$：实际规划段长度（如 30-50 m，TBD）
- $L_{\text{buffer}}$：前瞻缓冲（如 10-20 m，TBD）

**窗口滑动**：

随机器人前进，窗口中心跟随移动。当机器人超过窗口前 1/3 位置时触发窗口更新。

**全局路线管理**：

- 保持全局参考路线 $\mathcal{R}_{\text{ref}}$（可以很长，如 30 km）
- 只对当前窗口内的局部参考线段进行规划
- 每次规划使用局部参考线段 + 局部地图快照

### 10.5 当前路径复用与验证租约

`PlanningResult` 是规划流水线的可变装配类型；只有 `validate(candidate)` 通过、序号严格递增并由 publisher 复制后，才成为只读 `ImmutablePlanningResult`。执行侧通过 `AuthorizedPlanningResultPublisher` 原子观察“不可变计划 + 已授权剩余 `TimedPath` + 匹配租约 + 有限非负路径代价”，不能先观察裸计划再补装授权或成本。候选代价取自本周期 `HybridAStarSearchDiagnostics.solution_cost`，经 `PlanningCandidateMetadata.path_cost` 进入决策和授权；续租保持当前授权的原始审计代价。继续使用旧路径时不修改其原始地图、参考线、包络或剖面证据，而是由 `PlanValidityEvaluator` 对**剩余时序轨迹**生成短期 `PlanValidationLease`。

**统一复检输入**：

- 同一次原子捕获得到的 `RobotState`、`CableState`、`ReferenceProgress`、放缆遥测、执行剖面跟踪状态和 tracker receipt
- 当前不可变地图快照、机器人作业区域、缆线施工走廊和参考线
- 当前传感器健康模式、已验证不确定性包络、认证执行运行包络、走廊风险策略和地形梯度风险策略
- 当前计划及其尚未执行的 `TimedPath`

**当前固定复检顺序**：

1. 校验不可变计划、同步来源证明、所有状态/遥测/跟踪/地图的有限性、最大年龄、未来时间和同步容差；缺失原子来源或过期输入分别失败关闭为 `input_invalid` 或 `input_expired`
2. 用 `ExecutionTrackingState.tracked_arc_length_m` 关联旧计划，只裁剪已确认执行的前缀；位置、航向、实测曲率或跟踪剖面/运行包络版本不匹配时返回 `state_mismatch`
3. 检查地图、参考线、机器人作业区、缆线施工走廊、地形梯度风险策略、走廊风险策略、缆线模型、统计包络及生成器、执行运行包络、传感器模式和运行域是否形成逐项版本匹配的当前上下文
4. 裁剪原 `TimedPath` 并把剩余相对时间从零重锚，保持原 `execution_profile_version`；当前对地速度/加速度、出缆速度/加速度或张力无法在批准跟踪误差内与原剖面连续衔接时返回 `execution_profile_mismatch`，不得在续租中暗改剖面
5. 验证裁剪剖面的结构和批准边界，并要求原批准停止点到当前进度的距离不少于 `maximum_stopping_distance_m + stopping_safety_margin_m`
6. 以同步实测位姿和曲率作为起点边界，对剩余几何执行 G2、完整足迹、当前地图/地形和 $\mathcal W_{\text{robot}}$ 独立复检
7. 从**当前** `CableState`（包括实际 `laying_memory`）出发，按剩余执行剖面重新预测触地点；不得复用计划生成时缓存的剩余缆线路径、旧预测或候选标签记忆
8. 使用当前参考进度、参考线、锁定包络、区间上界证书和风险策略执行走廊复检，并执行缆线机械硬约束复检
9. 全部通过且租约有效窗口仍为正时，将租约注册为当前包络的依赖并递增租约序列；任何无效数值、模型越界、注册失败或硬约束失败均不得生成租约，失败也不得消耗租约序列

租约有效期为：

$$
t_{\text{lease,expire}}=
\min\left(
t_{\text{now}}+T_{\text{reuse,max}},
t_{\text{map,expire}},
t_{\text{robot,expire}},
t_{\text{cable,expire}},
t_{\text{progress,expire}},
t_{\text{telemetry,expire}},
t_{\text{tracking,expire}},
t_{\text{envelope,valid\_until}}-T_{\text{envelope,margin}}
\right)
$$

机器人失效时刻同时受位姿和曲率时间戳中较早者限制。地图/参考线/双空间域/地形梯度风险策略/走廊风险策略版本变化、执行剖面或执行运行包络版本变化、传感器模式变化、缆线模型版本或有效性变化、定位跳变或统计包络失效必须异步撤销当前租约并触发重规划，不能等待下一个 2-5 Hz 周期。执行器还必须连续监测对地速度、出缆速度、地面/出缆加速度和张力：既检查批准的绝对范围，也检查相对剖面插值期望的偏差；任一越界立即持久撤销租约并请求受控停车/重规划。反馈过期/未来时间、计划/租约/剖面未配对、完整依赖不一致、剖面内容在同版本下变化、计划/租约/反馈序列回退也执行相同拒绝。进入续租裕量只返回 `renewal_required` 并请求完整复检，不延长当前截止时间。

**适用入口**：下列所有操作必须调用同一 `validateRemainingPlan`，不得各自实现简化版 `validate_path(map)`：

- 周期之间继续发布当前路径
- 新规划超时后的旧路径回退
- 新路径被滞回拒绝后的旧路径保持
- 通信短时降级时沿承诺段继续执行

执行器只接受“不可变计划 + 租约授权的剩余 `TimedPath` + 未过期且未撤销的租约”组合，且三者的计划序号和执行剖面版本必须一致。重新验证只可裁剪原剖面和重锚相对时间；需要重新参数化时必须生成新的不可变 `PlanningResult`，完成缆线复检并取得匹配的新租约，不能改写旧计划的生成证据或悄悄延长其有效期。

---

## 11. 主动补探协调

本节描述当前 `ScoutCoordinator` 的 Level 1 算法边界。它识别信息缺口、评估紧迫度、生成可审计目标、约束双机距离，并维护“请求 -> 地图更新 -> 重规划”的确定性生命周期。它不负责前置机器人的三维路径规划、底层控制、传感器处理或地图构建；这些外部链路仍受 T48-T49 阻塞。当前 `MainPlanningLoop::run_cycle` 也不直接持有 `ScoutCoordinator`：外层协调器先处理补探/状态事件，只有获得可规划的新鲜同步上下文后才调用第 16 节的单周期规划入口。

### 11.1 信息缺口识别

`identify_gaps_result(reference_line, map, planning_horizon_m, candidate_detour)` 是保留输入状态和诊断的公共入口。它先完整校验参数、地图、参考线、正规划窗口和可选绕行几何；任一失败返回 `ScoutGapScanValidity::input_invalid` 和 `issues`，不得把空 `gaps` 解释为“没有缺口”。`identify_gaps`/ `identifyGaps` 是只返回向量的便利包装，安全编排应使用带状态的结果接口。

有效扫描按物理 `sample_interval_m` 从参考线起始弧长采样到 `min(reference_end, reference_start + planning_horizon_m)`，并检查可选候选绕行上的每个几何点：

- 地图外、`known == false` 或不可查询位置记录为 `InformationGapReason::unknown`；
- 已知但 `confidence < minimum_map_confidence` 记录为 `low_confidence`；
- 每个缺口保留栅格位置、空间中心、参考进度区间、最低置信度、完整源地图版本和参考线版本；
- 相邻样本只有在参考进度和空间距离同时落入 `merge_distance_m` 加采样裕量时合并，混合原因取更保守的 `unknown`；
- 最终结果按参考进度、区间和原因稳定排序，相同输入逐字段确定。

候选绕行的缺口使用其真实空间位置，同时通过参考线局部投影获得任务进度；后续地图关联必须重新检查这个真实目标单元，不能因为同进度的参考线单元已知而误判完成。

### 11.1.1 信息缺口紧迫度评估

`assess_gap_urgency` 使用当前 `RobotState`、单调 `current_reference_progress_m`、规划窗口和可选的**已批准剩余 `TimedPath`**。它不使用未批准候选，也不使用写死速度：

```text
distance_to_gap = max(0, gap.start_progress - current_reference_progress)
safe_path_remaining = min(distance_to_gap, approved_path.remaining_from_robot)
time_to_gap = approved ExecutionProfile 在当前弧长与目标弧长间的插值时间差
```

若参数、状态、区间或规划窗口无效，或者已批准路径缺失/无效/无法形成可审计时间，结果失败关闭为 `BLOCKING + STOP_AND_WAIT`，并设置 `used_conservative_fallback`。正常四级判定为：

```text
if blocks_window and safe_path_remaining < minimum_safe_distance:
    BLOCKING
else if time_to_gap < planning_lead_time:
    URGENT
else if time_to_gap < planning_horizon / average_velocity:
    SCHEDULED
else:
    INFORMATIONAL
```

`v_avg`、`L_safe_min` 和 `t_lead` 全部来自版本化 `ScoutCoordinationParameters`。分级键由地图 id/序号、参考线版本、量化进度区间和量化空间目标组成；不可表示的量化值失败关闭。升级和降级分别使用 `hysteresis_distance_m`、`hysteresis_time_s`，避免阈值附近反复切换。

| 紧迫度 | `recommended_action` | 允许的编排行为 |
|---|---|---|
| `blocking` | `STOP_AND_WAIT` | 撤租并停车，签发补探请求，进入等待地图 |
| `urgent` | `REQUEST_VALIDATED_REDUCED_SPEED_PROFILE` | 请求新的谨慎执行剖面；只有完整复检并取得新租约后才能切换 |
| `scheduled` | `REQUEST_SCOUT_BACKGROUND` | 后台签发补探请求，当前获批计划仍受原租约约束 |
| `informational` | `RECORD_ONLY` | 只记录，不签发请求 |

任何级别都不能直接对现有速度命令乘固定比例。

### 11.2 补探目标与优先级

`generate_scout_target` 只接受与当前参考线版本/坐标系一致、尚未完全落在当前任务进度之前（后方）的缺口。目标使用缺口真实中心，航向取区间中点处参考线切向，并记录：

- 传感器覆盖半径推导的 `coverage_fraction`；
- 最低置信度、区间长度和覆盖率推导的 `information_value`；
- 相对当前任务进度的 `forward_progress_m`；
- 前置机到目标的 `estimated_arrival_cost_m`；
- 源地图、参考线、补探策略、参数 profile 和运行域版本。

目标必须位于 `scout_corridor_half_width_m` 内，且主机器人到目标的距离不得超过 `communication_max_distance_m`。排序首先按紧迫度，保证 `blocking` 不会被可调权重压过；同级再按信息价值、前向临近度和到达代价形成的 `priority`，最后用进度和地图序号稳定打破平局。

### 11.3 双机距离约束

`assess_distance_constraint` 同时维护三个独立语义，通信硬约束为
`scout_main_distance_m <= communication_max_distance_m`：

- 超过 `communication_max_distance_m` 是硬失败，返回 `recover_communication`、`stop_and_recover_communication` 和降级标记；
- 正常期在 `desired_scout_distance_m` 前继续前探；
- 超过 `stop_scout_distance_m` 后保持位置，只有回到严格小于 `continue_scout_distance_m` 才释放 hold。

无效参数或位姿同样保持 hold 并失败关闭。期望距离、继续阈值、停止阈值和通信硬上限不能互相代替。

### 11.4 请求去重、地图关联与超时

`issue_scout_request` 只签发 `blocking`、`urgent` 或 `scheduled` 目标。目标必须绑定当前策略/profile/运行域，源地图时间不得晚于请求时间，且超时时间必须可表示。活动请求使用地图 id/序号、参考线版本、量化进度区间和量化空间目标形成的 `GapKey` 去重；重复请求返回原 `request_sequence` 和 `revision`，不重复发送。

新请求以 `revision = 1`、`awaiting_map_update` 和版本化 `request_timeout` 开始。阻塞请求携带 `waiting_map`，紧急请求只建议申请已验证减速剖面，计划中请求保持当前获批计划。每次关联或终止都单调增加 revision。

`correlate_map_update` 只接受同地图 id/坐标系、严格更新序号、请求后生成且不晚于观察时刻的地图，并要求参考线版本保持一致。它重新检查整个缺口进度区间以及真实空间目标单元：

- 仍有未知/低置信单元：`associated_unresolved`，记录最新关联地图版本但不触发重规划；
- 全部解决：`completed`，记录完成地图版本，设置 `invalidate_old_plan = true` 和 `trigger_replanning = true`；
- 到达或超过 `expires_at`：原子转为 `timed_out`，设置 `waiting_map` 并使旧计划失效；
- 重复、乱序、错版本、未知请求或终态请求：结构化拒绝，不改变已提交状态。

`expire_scout_requests(now)` 对所有到期活动请求执行同一终止规则。等待地图状态不签发运动租约；地图解决只产生重规划指令，本身不授权运动。

### 11.5 当前验证边界

Level 1 的 `test_scout_coordinator` 覆盖参考线/绕行缺口、保守紧迫度、滞回、目标审计、距离策略、请求去重、实际目标单元关联、超时和确定性重放；`test_level1_closed_loop_scenarios` 的 unknown-gap 场景覆盖“停车无授权 -> 地图更新 -> 新周期重规划 -> 新租约授权”。这些证据只证明确定性算法闭环，不证明 DAVE/Gazebo 双机通信、长时延迟、真实前置机运动或地图构建已完成。

---

## 12. 规划状态机与降级策略

`PlanningStateMachine::dispatch(event, context)` 是确定性事件决策器。它只读取一次不可变 `PlanningDecisionContext`，不在分派期间读取地图、跟踪器、publisher 或租约监控器；适配层必须从同一同步输入帧和当前执行授权装配 context。输出 `PlanningDecision` 固定保留前后状态、动作、有序指令、事件/配置/profile/运行域、恢复 revision/lease 及结构化原因。

### 12.1 公共状态、事件和动作

公共 `PlanningState` 的完整集合为：

| 状态 | 当前语义 |
|---|---|
| `INIT` | 尚未接收首个有效周期触发 |
| `NORMAL_PLANNING` | 正在或应开始正常规划 |
| `PLANNING_WITH_CAUTION` | 已安装经完整复检和新租约授权的谨慎剖面 |
| `SUCCESS` | 新候选已完成最新上下文复检并取得授权 |
| `PATH_VALID` | 当前不可变计划已在最新上下文复检并取得新租约 |
| `WAITING_MAP` | 等待补探地图；不隐含运动授权 |
| `REQUEST_SCOUT` | 已请求补探；继续、谨慎或停车由租约证据决定 |
| `NO_SOLUTION` | 当前约束下无普通可行解 |
| `NO_SOLUTION_UNDER_COVARIANCE_ENVELOPE` | 当前锁定统计包络下无解，不代表物理问题必然无解 |
| `COVARIANCE_ENVELOPE_BREACH` | 实际验证超过锁定包络，相关授权不可继续 |
| `INPUT_INVALID` | 输入、上下文或授权证据无效 |
| `MAP_EXPIRED` | 地图超过允许时效 |
| `TIMEOUT` | 规划阶段或总周期超时 |
| `COMMUNICATION_DEGRADED` | 通信中断，执行受降级模式和租约约束 |
| `MANUAL_OVERRIDE` | 人工接管锁定态 |
| `EMERGENCY_STOP` | 急停锁定态 |

阶段内部状态和 `PlanningCycleStatus` 不能冒充新的公共状态。输入事件覆盖周期、新地图、参考线/作业区/机器人状态变化、路径/租约失效、通信丢失/恢复、定位无效、规划结果、补探、人工接管和急停。输出动作只有 `none`、`continue_authorized_path`、`begin_planning`、`reduce_speed`、`controlled_stop`、`emergency_stop` 和 `manual_takeover`。

公共 `PlanningDirective` 的完整集合为 `revoke_current_lease`、`request_controlled_stop`、`request_emergency_stop`、`start_planning`、`continue_authorized_path`、`switch_to_validated_cautious_profile`、`request_scout`、`request_manual_takeover` 和 `stop_automatic_planning`。`PlanningDecision::directives` 是有序序列，消费者必须按序执行；撤销租约不得晚于停车、重规划或人工接管指令。

### 12.2 事件顺序与重规划触发

配置缺少版本、profile、运行域、正规划周期、失败上限或严格递增的短/中通信阈值时，状态机直接进入 `INPUT_INVALID` 并输出 `revoke_current_lease -> request_emergency_stop`。

普通事件要求非零且严格递增的 `sequence_number`、非负且不回退的单调时间。以下安全事件可以越过旧事件水位立即执行，避免乱序阻止停车：路径失效、通信丢失、定位无效、租约失效、包络越界、输入无效、地图过期、急停请求和关键系统故障。安全事件只有在自身序号/时间确实更新时才推进水位，旧安全事件不能污染正常事件顺序。

当前显式重规划触发为：

- 首个 `periodic_tick`，以及距离上次规划触发达到 `planning_period` 的周期 tick；
- 严格更新的地图序号；
- 参考线、机器人作业区或机器人状态变化；
- 路径失效、租约续签需要或租约失效；
- 通信恢复取得完整新授权后同时继续当前授权路径并启动规划。

地图/任务/状态变化和路径失效均先输出撤租与停车指令，再在同一有序列表末尾输出 `start_planning`；是否允许受控停车由 12.5 节证据决定。重复地图只返回 `MAP_NOT_NEWER`，不触发规划。

### 12.3 规划结果、补探和失败转换

`planning_succeeded`、`planning_with_caution`、`planning_timed_out` 的旧计划继续分支都要求新的 `RecoveryAuthorization`：同步源 revision 和 lease sequence 必须分别严格超过状态机已接受水位，且 `synchronized_snapshot_valid`、`lease_live`、`dependencies_match` 全为真。缺失或重复证据失败关闭并停车。

- `planning_succeeded` -> `SUCCESS`，继续新授权路径；
- `planning_with_caution` -> `PLANNING_WITH_CAUTION`，只切换到已验证谨慎剖面；
- `planning_timed_out` 有新授权 -> 保留 `TIMEOUT` 审计状态并继续，否则停车；
- `waiting_for_map` 或 `scout_requested` 只有在 `degraded_profile_lease_live` 时才可请求补探并切换到已验证谨慎剖面，否则先撤租并停车，再请求补探；
- `planning_failed` 和 `covariance_solution_unavailable` 分别进入两个无解状态并累计连续失败；达到配置上限后追加人工接管请求；
- 包络越界、输入无效和地图过期进入对应失败状态并撤租停车。

### 12.4 通信降级与恢复

`communication_lost` 总是进入 `COMMUNICATION_DEGRADED`，但动作按证据分层：

| 中断区间 | 继续条件 | 动作 |
|---|---|---|
| 短时：`0 <= outage < short_limit` | 当前租约有效且降级传感器模式已批准 | 继续当前授权路径 |
| 中等：`short_limit <= outage <= medium_limit` | 已验证谨慎剖面租约有效 | 切换到谨慎剖面 |
| 长时、无效时间或条件缺失 | 无 | 撤租并受控/急停 |

`CommunicationRecoveryGate` 在恢复边界记录每个消息流的水位、同步源 revision 和活动租约序号。只有地图、参考线、机器人/缆线/进度/遥测/跟踪、规划结果和验证租约九个执行上下文流都在恢复时刻之后推进，且它们与同一个 `SynchronizedValidationInputs + PlanningResult + PlanValidationLease` 完整配对，再由 `ExecutionLeaseMonitor` 确认新租约授权，才生成 `RecoveryAuthorization`。

`communication_restored` 没有该证据时保持降级并停车；证据新鲜时进入 `PATH_VALID`，先继续新授权路径，再触发一次规划。通信恢复本身绝不能复活恢复边界前的计划或租约。

### 12.5 锁定状态与安全停车

`MANUAL_OVERRIDE` 和 `EMERGENCY_STOP` 是锁定状态。普通规划成功不能解除它们；分别需要 `manual_control_released` 或 `emergency_stop_cleared`，并同时提交严格更新的恢复授权。状态不匹配的解除事件拒绝，缺少新授权时保持锁定。

安全指令的优先级固定为 `emergency_stop > controlled_stop > continue_authorized_path`。人工急停和关键故障输出：

```text
revoke_current_lease -> request_emergency_stop
```

其他停车事件调用同一 `SafeStopContext` 门禁：

```text
effective_braking_deceleration =
    min(maximum_braking_deceleration, terrain_limited_braking_deceleration)

required_stopping_distance =
    abs(ground_speed) * control_reaction_time
    + ground_speed^2 / (2 * effective_braking_deceleration)
    + safety_margin
```

只有速度、两种制动减速度、剩余安全距离、安全裕量和反应时间全部有限且范围有效，`terrain_braking_model_certified == true`，并且所需距离不超过剩余安全距离，才输出 `controlled_stop`。证据缺失、模型未认证、输入无效或距离不足均升级为 `EMERGENCY_STOP`。当前 Level 1 只验证该决策契约；真实坡度/载荷制动能力仍受 T52 外部标定阻塞。

---

## 13. 数据一致性与版本管理

### 13.1 目的

防止异步输入、版本回退和发布后修改造成不一致决策。当前实现有三个不同但连续的冻结边界：

1. `SnapshotManager` 安装并持有一个值语义的 `VersionedPlanningSnapshot`。
2. `SynchronizedValidationInputCapturer` 从一次来源线性化点冻结状态、遥测、快照和依赖版本。
3. `PlanningResultPublisher` 只把通过完整校验且序号严格递增的候选复制为 `ImmutablePlanningResult`。

任何边界失败都关闭本次决策，不返回可执行的部分对象；不能把捕获前后的字段拼接成一个“最新”上下文。

### 13.2 版本化规划快照

`versioned_snapshot.hpp` 的 `VersionedPlanningSnapshot` 是以下四项的唯一组合快照：

| 成员 | 必需版本与内容 | 失效或回退规则 |
|---|---|---|
| `MapSnapshot map` | `MapVersion{map_id, sequence_number, timestamp, coordinate_frame}`；栅格尺寸、分辨率、原点、`MapCell`、`derived_configuration_version`、`update_regions` | 版本不完整或同 `map_id` 序号回退拒绝；未来时间始终拒绝，`maximum_age > 0` 时另行拒绝过期时间；同版本异负载拒绝 |
| `ReferenceLine reference_line` | 非零 `uint32` 版本、坐标系、严格递增连续弧长及切/法向 | 版本回退拒绝；坐标系必须与地图一致 |
| `RobotOperatingArea robot_operating_area` | 非零 `uint32` 版本、非空 `id`、有效非空多边形 | 与缆线走廊独立验证；回退或无效几何拒绝 |
| `CableCorridor cable_corridor` | 非零 `uint32` 版本、非空 `id`、有效非空多边形 | 与机器人作业区独立验证；回退或无效几何拒绝 |

`SnapshotManager::update` 只产生 `accepted`、`duplicate`、`out_of_order`、`expired`、`version_rollback` 或 `invalid`。当前实现对四个成员分别执行“同版本同负载”检查，即使其他成员版本推进，旧版本携带新负载仍拒绝；完整版本与负载相同才是幂等 `duplicate`。默认 `Duration{0}` 仅关闭过期年龄上限，不关闭未来时间和版本因果门禁。`latest()` 返回值副本，`is_current()` 比较完整负载，不存在设计中另一个可变 `lock()` 对象。

`SpatialDomainConfig` 的版本文本必须是 canonical 非零十进制无符号数；生产规划周期在冻结同步输入后，用 `validate_spatial_domain_snapshot` 将其显式转换为 `uint32`，并逐项核对机器人作业区/缆线走廊的 id 与版本。空字符串、带前缀版本、id 错配或版本错配均关闭本次规划。

### 13.3 原子同步输入与完整依赖元组

`PlanningDependencyVersions` 的当前唯一字段集合为：`map_version`、`reference_line_version`、`robot_operating_area_version`、`cable_corridor_version`、`terrain_gradient_policy_version`、`corridor_risk_policy_version`、`cable_model_version`、`uncertainty_envelope_version`、`uncertainty_envelope_generator_version`、`execution_operating_envelope_version`、`execution_profile_version`、`sensor_mode` 和 `operating_domain_id`。任何字段缺失、为零、未知或与快照/跟踪状态不匹配都使验证上下文无效；走廊版本也参与捕获竞态检测、结果/诊断一致性、复检租约和执行端异步依赖比较。

`ValidationInputSource::advance_trackers_and_capture_frame()` 必须在同一个来源侧锁内，用同一批已执行证据推进 `CableStateTracker` 与 `ReferenceProgressTracker`，然后返回 `TrackerSynchronizedFrame{source_revision, tracker_update_receipt, frame}`。`frame` 必须同时包含：

- `RobotState`、`CableState`、`ReferenceProgress`、`CableTelemetry` 和 `ExecutionTrackingState`；
- 完整 `VersionedPlanningSnapshot`；
- 上述 `PlanningDependencyVersions`。

捕获器串行化自身调用，比较返回的 `source_revision` 与复制后的来源 `revision()`；两者不等、修订为零或捕获期间任何状态/依赖变化时，整帧返回 `validation_context_invalid` 和 `source_changed_during_capture`，不携带 `SynchronizedValidationInputs`。成功结果固定 `PredictionMode::validation`，并保留同批 tracker receipt 作为状态来源证据。

每个状态流、地图和已执行证据流都使用非零序列号；成功捕获建立 watermark，后续相等且仍新鲜的帧允许重读，低于 watermark 的序列拒绝。机器人位姿/曲率、缆线状态、参考进度、放缆遥测、执行跟踪和地图分别检查最大年龄及未来时间，状态时间跨度还必须落在 `synchronization_tolerance` 内。缺失、过期、不同步、非有限数值、跟踪剖面错配、tracker receipt 错配或依赖不全均只返回结构化 `CaptureIssue`，不得静默回退到旧值。

### 13.4 规划结果校验与不可变发布

`PlanningResult` 由 `data_contract.hpp` 唯一定义。它携带结果序号、生成时间、首次有效期、公共状态、机器人时序轨迹、预测缆线路径、终端缆线状态、模型有效性、走廊与机械评价、误差预算、当前依赖字段和 `Diagnostics`。`dependencies()` 必须逐字段得到 13.3 节的同一元组，且 `Diagnostics.dependencies` 与结果完全相等。

发布顺序固定为：

1. `validate(candidate)` 检查所有枚举、依赖、诊断、风险语义、全部序列化浮点字段、结果顶层时间/有效期，以及实际存在的路径负载时间；失败状态中的 NaN、无穷和路径负载不规范角度仍会拒绝。
2. `candidate.sequence_number` 必须严格大于 publisher watermark；重复或回退返回 `sequence_not_monotonic`，且不改变当前结果。
3. publisher 把候选复制到 `shared_ptr<const PlanningResult>`，只通过 `ImmutablePlanningResult` 暴露常量访问；发布后修改装配候选不会改变已发布内容。

只有 `SUCCESS` 或 `PATH_VALID` 能发布可执行轨迹，并必须同时通过 `TimedPath`、缆线路径、走廊硬可行性、机械硬可行性和证据一致性检查。其他 `PlanningState` 可以不带路径负载，但仍必须携带有效序号、顶层时间、完整审计依赖、诊断和有限浮点字段；由于 `terminal_cable_state` 与 `corridor_result` 始终属于 `UP_RESULT 8` 序列化负载，任何要发布或序列化的失败结果也必须由装配方填入本次装配/评估的非负单调时间。`validate(PlanningResult)` 对这两个时间执行状态无关的失败关闭检查，默认 `-1` 或其他负值会阻止 publisher 发布和序列化；当前主规划循环的失败分支返回不可序列化的 `PlanningCycleResult`，不存在绕过该门禁的失败 `PlanningResult` 默认装配路径。有效结果往返保持嵌套时间不变。除此之外，未知枚举、旧 schema 或反序列化后校验失败均拒绝；首版诊断风险语义固定为 `POINTWISE_ONLY:NO_PATH_JOINT_RISK_GUARANTEE`。

### 13.5 时间戳、结果有效期与执行授权

所有上述时间均使用第 3.3 节的应用单调时钟。`PlanningResult.validity_duration` 只限制首次接受该规划结果的最长时间：

$$
t_{\text{initial,expire}}=t_{\text{generate}}+T_{\text{valid}}
$$

首次接受或继续执行还必须持有第10.5节生成的租约：

```cpp
struct PlanValidationLease {
    uint64_t lease_sequence;
    uint64_t plan_sequence_number;
    uint64_t evaluator_config_version;
    std::string parameter_profile_id;
    MonotonicTime validated_at;
    MonotonicTime expires_at;
    double remaining_path_start_arc_length_m;

    MapVersion map_version;
    uint32_t reference_line_version;
    uint32_t robot_operating_area_version;
    uint64_t terrain_gradient_policy_version;
    uint64_t corridor_risk_policy_version;
    uint64_t cable_model_version;
    uint64_t uncertainty_envelope_version;
    uint64_t uncertainty_envelope_generator_version;
    uint64_t execution_operating_envelope_version;
    uint64_t execution_profile_version;
    SensorHealthMode sensor_mode;
    std::string operating_domain_id;
    uint32_t cable_corridor_version;

    MonotonicTime robot_state_timestamp;
    MonotonicTime cable_state_timestamp;
    MonotonicTime cable_telemetry_timestamp;
    MonotonicTime execution_tracking_timestamp;
    double max_ground_speed_tracking_error_mps;
    double max_payout_speed_tracking_error_mps;
    RangeN allowed_tension;
    RangeMps2 allowed_ground_acceleration;
    bool robot_path_validation_passed;
    bool cable_corridor_validation_passed;
    bool cable_laying_validation_passed;
};
```

执行器只接受原子组合 `ImmutablePlanningResult + authorized remaining TimedPath + PlanValidationLease + path_cost`。前三项必须计划序号、剖面版本、剖面内容、运行包络及完整依赖一致，租约的三个硬验证标志均为真；`path_cost` 必须有限且非负；剩余轨迹必须是原不可变轨迹从 `remaining_path_start_arc_length_m` 开始的逐点/逐样本裁剪，不能被调用方替换。`AuthorizedPlanningResultPublisher` 在首次发布时先核验成本和租约，再把计划复制到只读句柄；续租只替换当前授权中的剩余轨迹与严格更新的新租约，保留原不可变计划和审计代价。较旧/重复租约不能覆盖新授权，替换或续租前先持久撤销原租约。

执行端在以下任一条件成立时返回 `revoke_and_controlled_stop`：租约不存在、已过期、已撤销或总时长超过上限，计划/租约/剩余轨迹未配对，地图/参考线/双空间域/地形梯度策略/走廊风险策略/缆线模型/统计包络及生成器/执行运行包络/剖面版本、传感器模式或运行域任一不再是当前上下文，剖面内容在版本不变时被修改，反馈过期/非有限/乱序，或速度、出缆、张力、加减速度跟踪超出租约边界。计划与租约序列水位只在结构、时间、依赖、剖面和反馈通过前置检查后推进，拒绝的伪高序列不能污染水位。偏差越界由独立安全监控持久撤销租约并触发受控停车/重规划，不能等待规划周期；接近截止时间只返回 `renewal_required`，不代表续期成功。

**典型有效期设置**：

- `PlanningResult` 首次接受窗口：TBD，不能替代复检或执行租约
- 正常路径复用租约：不长于 $T_{\text{reuse,max}}$，且到期时间不得晚于地图、机器人状态、缆线状态、参考进度、放缆遥测、执行跟踪或扣除安全裕量后的包络有效期中最早者
- 执行器按 $T_{\text{monitor}}$ 检查租约，并在剩余时间小于 $T_{\text{renew}}$ 时触发完整复检；复检尚未完成时不得预先延长租约
- 批准的降级模式租约：应短于正常租约，具体值 TBD
- 补探等待状态不签发运动租约；停车等待不等于允许继续执行30秒

### 13.6 类型化消息去重与乱序处理

`MessageConsistencyGate` 为十类消息流分别冻结去重键、最大年龄、有限序列窗口和是否允许乱序。配置必须有非零版本、profile、运行域、风险语义，并且每个流恰好一个有效策略；缺项、重复策略、非正最大年龄或零窗口使 gate 构造失败。

| 消息流 | 类型化去重键 | 乱序策略 |
|---|---|---|
| `map_update` | `map_id_and_sequence`：`MapUpdateKey` 的 `map_id + map_sequence`，序号必须等于 envelope sequence；`coordinate_frame` 参与载荷校验但不参与去重相等性 | 在 `reorder_sequence_window` 内缓冲 |
| `scout_response` | `ScoutResponseKey{request_sequence, revision}`，revision 必须等于 envelope version | 在 `reorder_sequence_window` 内缓冲 |
| `planning_result` | `PlanningResultKey{plan_sequence, initial_acceptance_validity}` | 立即接受并记录序列缺口 |
| `validation_lease` | `ValidationLeaseKey{lease_sequence}` | 立即接受并记录序列缺口 |
| 参考线、机器人状态、缆线状态、参考进度、放缆遥测、执行跟踪 | 非空 `MessageIdKey` | 立即接受并记录序列缺口 |

每次 `receive(message, now)` 按固定顺序检查：流/非零序号/非零版本/单调时间、键类型与载荷匹配、未来生成时间、接收时间回退、规划结果自身首次接受有效期、流最大年龄、已接受或已缓冲去重键、同序号竞争、序号回退、版本回退和接收窗口。规划结果在 `age >= initial_acceptance_validity` 时即拒绝；其他流在 `age > maximum_age` 时拒绝。

不允许乱序的流收到前跳序号时立即接受，但返回 `MissingSequenceRange` 并把前跳点设为下一提交序号。允许乱序的地图/补探响应先缓冲，只有从 `next_sequence` 开始连续时才按序释放；释放时重新检查年龄和版本，过期或回退的缓冲项进入 `discarded`，不能因为先进入缓冲区而绕过门禁。

只有 `ready` 中实际提交的消息推进 `MessageStreamWatermark{sequence, version, generated_at, typed_identity}`。重复、无效、未来、过期、窗口外和版本回退消息均不推进水位；已接受去重键只保留配置窗口内的有界历史。每个决定的 audit 固定保留原 envelope、评估时间、策略版本、profile、运行域和配置的非空风险语义；当前 Level 1/生产约定使用 `per-stream-consistency-only`，不能把单流顺序保证夸大为跨流原子一致性。

### 13.7 状态回退与恢复边界

回退防护由多个所有权边界共同完成，而不是一个通用 `UpdateState(version)`：

1. `MessageConsistencyGate` 独立维护每个消息流的序号、版本、接收时间、缓冲和 watermark；新序号携带旧版本仍拒绝。
2. `SnapshotManager` 对地图、参考线和双空间域执行同版本同载荷与版本回退检查。
3. `SynchronizedValidationInputCapturer` 用来源 revision、逐流序号和 tracker receipt 冻结跨流原子帧；消息 gate 的单流接受不能替代该捕获。
4. `PlanningStateMachine` 对普通事件执行严格序号/时间水位，同时允许第 12.2 节列出的旧安全事件立即停车而不污染较新的正常事件水位。
5. `PlanningResultPublisher`、`AuthorizedPlanningResultPublisher` 和 `ExecutionLeaseMonitor` 分别维护计划与租约高水位；结构、时间、依赖、剖面或反馈未通过前置校验的伪高序列不能推进水位。
6. `ScoutCoordinator` 的请求 sequence 固定、revision 单调；终态请求拒绝后续地图更新，关联地图必须严格晚于源地图和上次关联版本。

通信恢复另建立一次性因果边界。调用 `CommunicationRecoveryGate::begin_resynchronization` 时记录所有流当前 watermark、同步源 revision、活动 lease sequence 和 `restored_at`。恢复评估要求九个执行上下文流在该边界之后全部推进，消息生成时间不早于 `restored_at`，并与同一个同步输入、不可变计划、严格更新的新租约和执行监控授权逐字段配对。任一流未推进返回 `awaiting_resynchronization`，上下文混合返回 `context_mismatch`，租约/执行授权无效返回 `authorization_invalid`；只有 `authorized` 结果能生成第 12 节接受的恢复证据。

---
## 14. 软件模块划分

### 14.1 核心模块架构

```
外部适配层（不在当前仓库实现）
  传感器/建图、参考路线、机器人/缆线遥测、执行反馈、控制与补探传输
                              │
                              ▼
输入与不可变契约层
  data_contract / parameter_config / versioned_snapshot
  message_consistency / synchronized_validation_inputs
  cable_state_tracker / reference_progress_tracker
                              │
                              ▼
派生安全证据层
  terrain_analyzer ──► traversability_evaluator
  cable_model ───────► cable_laying_evaluator
       │              cable_corridor_evaluator
       └─────────────► cable_uncertainty_envelope_builder / manager
                              │
                              ▼
候选生成与完整复检层
  merge_goal_generator ─► hybrid_astar_planner ─► path_smoother
  ─► path_candidate_verifier ─► trajectory_parameterizer
  ─► timed_cable_candidate_verifier
                              │
                              ▼
授权、安全与发布层
  plan_validity_evaluator ─► stability_manager
  commitment_safety ───────► execution_lease_monitor
  planning_result（不可变计划与原子 publisher）
                              │
                              ▼
编排与审计层
  main_planning_loop / planning_state_machine / scout_coordinator
  communication_recovery / algorithm_diagnostics
```

箭头表示主要证据和控制依赖，不表示所有 C++ include。`MainPlanningLoop` 是单周期编排所有者；`PlanningStateMachine` 管理事件到指令的转换；`ScoutCoordinator`、`MessageConsistencyGate` 和 `CommunicationRecoveryGate` 位于外层协调边界。当前仓库没有 ROS 2 adapter、上游地图构建、下游控制器或前置机器人运动规划实现，不能把这些外部职责计入 `underwater_planner::core`。

`underwater_planner_core` 由 34 个生产 `.cpp` 组成并公开 32 个 `core/*.hpp`。源目录中的 `planar_geometry.hpp`、`terrain_analysis_primitives.hpp`、`step_geometry.hpp` 和 `storage_estimation.hpp` 是实现细节；不得从适配层或测试外部依赖。`underwater_planner_test_support` 的 3 个 `.cpp` 和 2 个 `testing/*.hpp` 只服务合成夹具、确定性闭环与报告，不是生产接口。

### 14.2 公共接口目录与所有权

下表是当前生产公共头文件的闭集。表中的“主要公共 seam”用于定位契约，不替代头文件本身；未列出的 source-only 函数不构成公共承诺。

| 层 | 公共头文件 | 所有权与主要公共 seam |
|---|---|---|
| 契约 | `version.hpp` | API 版本查询 `api_version()` |
| 契约 | `data_contract.hpp` | SI/时间/状态/路径/风险/诊断/结果类型，`validate(...)` 与确定性序列化 |
| 配置 | `parameter_config.hpp` | `ParameterConfig`、严格 loader/serializer、production 门禁与空间域快照核对 |
| 快照 | `versioned_snapshot.hpp` | 地图、参考线、双空间域、`SnapshotManager` 和回退/时效门禁 |
| 同步输入 | `synchronized_validation_inputs.hpp` | `SynchronizedValidationInputCapturer::capture` 原子冻结 tracker、执行和依赖证据 |
| 发布契约 | `planning_result.hpp` | `ImmutablePlanningResult` 与 `PlanningResultPublisher` 的只读、单调、原子发布 |
| 诊断 | `algorithm_diagnostics.hpp` | 实验记录、汇总、性能预算评估与真实主循环离线重放 |
| 地形 | `terrain_analyzer.hpp` | 全量/增量表面、粗糙度、缆线地形和台阶图层派生 |
| 通行性 | `traversability_evaluator.hpp` | 碰撞层/扫掠、方向坡度、台阶与履带支撑硬门禁 |
| 通行性 | `step_traversal_rules.hpp` | 台阶接近方向有效域的共享内联规则；不拥有台阶提取或整段评价 |
| 缆线状态 | `cable_state_tracker.hpp` | 仅由已执行运动、遥测和确认观测推进 `CableState`/机械历史 |
| 任务进度 | `reference_progress_tracker.hpp` | 实际进度 tracker 与候选局部关联 associator |
| 缆线模型 | `cable_model.hpp` | 搜索均值传播、完整时序均值/协方差传播和反解终端状态 |
| 机械约束 | `cable_laying_evaluator.hpp` | 曲率、禁放区、支撑代理与缆线适宜性；候选记忆复制推进 |
| 走廊风险 | `cable_corridor_evaluator.hpp` | 点/区间走廊分级、横向协方差投影和保守上界证书 |
| 统计包络 | `cable_uncertainty_envelope_builder.hpp` | 在认证运行域生成横向不确定性上包络和可达性证书 |
| 统计包络 | `cable_uncertainty_envelope_manager.hpp` | 注册、完整键查询、锁定、审计和依赖失效 |
| 并线目标 | `merge_goal_generator.hpp` | 反解多个接入目标、完整终端门禁与逐目标拒绝诊断 |
| 搜索 | `hybrid_astar_planner.hpp` | 增广 Hybrid A*、多标签、完整原语验证、资源耗尽和确定性诊断 |
| 平滑 | `path_smoother.hpp` | 分段 clothoid 候选、边界来源、求解状态和内部几何残差审计；不拥有当前地图安全复检 |
| 机器人复检 | `path_candidate_verifier.hpp` | 独立曲率/G2 审计和当前地图上的完整路径机器人复检 |
| 时序化 | `trajectory_parameterizer.hpp` | 生成带版本的速度/加速度/放缆/张力剖面及停车门禁 |
| 缆线复检 | `timed_cable_candidate_verifier.hpp` | 完整时序高精度缆线重预测、机械/走廊/包络复检与减速重预测 |
| 计划复检 | `plan_validity_evaluator.hpp` | 当前计划或发布候选的最新状态全量复检和新租约签发 |
| 稳定性 | `stability_manager.hpp` | 相对代价/可选拓扑滞回、承诺段提取及 G2/时序拼接；不拥有异步安全事件 |
| 承诺安全 | `commitment_safety.hpp` | 安全事件聚合、STOP 优先级、先撤租后调用独立停车通道 |
| 执行授权 | `execution_lease_monitor.hpp` | 计划/剖面/租约/上下文配对、反馈偏差和持久撤租 |
| 补探 | `scout_coordinator.hpp` | 信息缺口、紧迫度、目标、距离和请求生命周期；不发布控制命令 |
| 状态机 | `planning_state_machine.hpp` | 类型化事件、9 类指令、恢复授权和安全停车决策 |
| 消息一致性 | `message_consistency.hpp` | 十类消息流去重、乱序缓冲、缺口、水位、时效和回退门禁 |
| 通信恢复 | `communication_recovery.hpp` | 恢复边界后新快照、计划、租约和执行授权的组合门禁 |
| 主循环 | `main_planning_loop.hpp` | 锁定输入、阶段编排、候选/当前成对复检、滞回、失败收敛和授权原子发布 |

主要跨模块所有权规则如下：

1. `data_contract.hpp` 和 `versioned_snapshot.hpp` 只定义共享值与不可变快照，不执行规划编排；配置文本只由 `parameter_config.hpp` 解释。
2. `TerrainAnalyzer` 产生方向无关地形证据；机器人方向能力只由 `TraversabilityEvaluator` 解释。缆线协方差不进入机器人碰撞预算。
3. 搜索复用 `CableModel`、`CableLayingEvaluator`、`CableCorridorEvaluator`、`TraversabilityEvaluator` 与锁定包络的公共语义；它不拥有这些模型的第二套近似契约。
4. `PathSmoother` 的成功只表示平滑问题收敛且内部残差合格；`PathCandidateVerifier` 和 `TimedCableCandidateVerifier` 分别拥有机器人与缆线完整候选复检。
5. `PlanValidityEvaluator` 签发短期验证租约，`StabilityManager` 只比较已经成对验证的结果，`ExecutionLeaseMonitor` 和 `CommitmentSafetySupervisor` 可异步撤租。
6. `MainPlanningLoop` 是授权发布的唯一生产编排 seam；状态机、补探和通信恢复只能提供输入或指令，不能直接制造可执行计划。

#### SnapshotManager

```cpp
class SnapshotManager {
public:
    SnapshotUpdateResult update(
        const VersionedPlanningSnapshot& snapshot,
        MonotonicTime now);
    std::optional<VersionedPlanningSnapshot> latest() const;
    bool is_current(const VersionedPlanningSnapshot& snapshot) const;
};
```

`SnapshotManager` 接受完整 `VersionedPlanningSnapshot`，不单独拼接地图、参考线或双空间域。它拒绝无效、重复、乱序、过期和版本回退输入；原子复检所需的机器人/缆线/执行状态由 `SynchronizedValidationInputCapturer` 在其独立公共边界冻结。

#### TerrainAnalyzer

```cpp
enum class TerrainEstimateStatus {
    valid,
    insufficient_support,
    ill_conditioned,
    invalid_covariance,
    discontinuous
};

struct SurfaceEstimate {
    double elevation_m;
    double gradient_x;
    double gradient_y;
    double slope_angle_rad;
    GradientCovariance gradient_covariance;
    double detrended_roughness_rms_m;
    double residual_p95_m;
    double support_ratio;
    TerrainEstimateStatus status;
};

struct TerrainLayers {
    MapVersion source_map_version;
    uint64_t analysis_config_version;
    std::string operating_domain_id;
    double surface_fit_window_size_m;
    SurfaceLayer surface;
    CableLayingTerrainLayer cable_laying;
    StepLayer steps;
};

class TerrainAnalyzer {
public:
    TerrainLayers analyze(
        const MapSnapshot& height_map,
        const TerrainAnalysisConfig& config
    ) const;
};

enum class TerrainAnalysisUpdateMode {
    full_rebuild,
    incremental_update,
    cache_hit
};

struct IncrementalTerrainAnalysisDiagnostics {
    TerrainAnalysisUpdateMode mode;
    std::size_t recomputed_cell_count;
    std::size_t reused_cell_count;
    bool source_version_invalidated;
    std::vector<MapUpdateRegion> expanded_update_regions;
};

struct IncrementalTerrainAnalysisResult {
    std::shared_ptr<const TerrainLayers> layers;
    IncrementalTerrainAnalysisDiagnostics diagnostics;
};

class IncrementalTerrainAnalyzer {
public:
    IncrementalTerrainAnalysisResult analyze(
        const MapSnapshot& height_map,
        const TerrainAnalysisConfig& config
    );
};
```

`TerrainAnalysisConfig` 的全部字段都参与缓存兼容性比较；地图的 `derived_configuration_version` 必须等于 `config_version`。完整的状态、台阶拒绝原因和增量失效语义见第5.1、5.2和5.8节。

#### MergeGoalGenerator / HybridAStarPlanner

```cpp
class MergeGoalGenerator {
public:
    MergeGoalGenerator(
        CableModelParameters cable_model_parameters,
        MergeGoalGenerationParameters generation_parameters,
        RobotCapability robot_capability,
        TrackFootprint track_footprint
    );

    MergeGoalGenerationResult generate(
        const ReferenceProgress& current_progress,
        const ReferenceLine& reference_line,
        const RobotOperatingArea& robot_operating_area,
        const TerrainLayers& terrain,
        const TerrainGradientRiskPolicy& terrain_gradient_risk_policy
    ) const;
};

class HybridAStarPlanner {
public:
    HybridAStarPlanner(
        CableModelParameters cable_model_parameters,
        ReferenceProgressAssociationParameters progress_parameters,
        HybridAStarSearchParameters search_parameters,
        CableCorridorRiskPolicy corridor_risk_policy,
        CableUncertaintyEnvelopeManager& envelope_manager,
        HybridAStarSteadyClock clock = [] {
            return std::chrono::steady_clock::now();
        }
    );

    HybridAStarPlanningResult plan(
        const HybridAStarPlanningRequest& request
    ) const;
};
```

`MergeGoalGenerationParameters` 唯一持有并线距离、终端航向偏置、终端滞后角、目标数量、正向闭环容差和生成排序权重；`MergeGoalGenerationResult` 返回目标、逐候选拒绝诊断及模型/参考线/作业区/运行域审计。

`HybridAStarPlanningRequest` 聚合起点机器人/缆线状态、搜索 `CableContext`、初始 `ReferenceProgress`、锁定 `ReferenceLine`、非空 `MergeGoal` 集合、`HybridAStarPrimitiveSweepContext`、锁定统计包络、规划时间和非零随机种子。地图、地形、机器人作业区、碰撞/梯度风险策略、机器人能力、履带足迹、缆线机械限制及历史边界都属于 `primitive_sweep_context`，不通过隐式全局对象读取。

`HybridAStarPlanningResult` 直接使用公共 `PlanningState`，并返回机器人/触地点 `GeometricPath`、终端 `CableState`、终端 `ReferenceProgress`、`HybridAStarStateTraceEntry` 序列和 `HybridAStarSearchDiagnostics`。当前类没有 `setParameters`、公开 `search`、公开 `computeCost` 或公开 `heuristic`；参数在构造时冻结，内部辅助函数不是公共接口。

#### TraversabilityEvaluator

```cpp
struct TerrainGradientRiskPolicy {
    uint64_t version;
    uint64_t terrain_analysis_config_version;
    double epsilon_local;
    double coverage_multiplier; // beta_g
    GradientCoverageModel coverage_model;
    std::string calibration_dataset_id;
    std::string operating_domain_id;
    bool coverage_calibrated;
};

struct RobotCapability {
    double maximum_roughness_m;
    // ... remaining slope, step and track-support capability fields ...
};

std::optional<RobotCapability> make_robot_capability(
    const RobotParameterConfig& parameters);

struct TraversabilityResult {
    // ...
    double maximum_detrended_roughness_rms_m;
    std::vector<TraversabilityLimitingFactor> limiting_factors;
};

class TraversabilityEvaluator {
public:
    TraversabilityEvaluator(
        RobotCapability capability,
        TrackFootprint track_footprint
    );

    CollisionLayerResult evaluate_collision_layer(
        const MapSnapshot& map,
        const TerrainLayers& terrain,
        const Covariance2dM2& robot_relative_obstacle_covariance_m2,
        const RobotCollisionRiskPolicy& policy
    ) const;

    CollisionSweepResult evaluate_collision_sweep(
        const MotionSegment& segment,
        const TerrainLayers& terrain,
        const CollisionLayerResult& collision_layer,
        double maximum_sweep_spacing_fraction
    ) const;

    TraversabilityResult evaluate(
        const MotionSegment& segment,
        const TerrainLayers& terrain,
        const TerrainGradientRiskPolicy& gradient_risk_policy
    ) const;
};
```

`make_robot_capability` 是 production 参数到运行时能力的显式装配入口；缺失、非有限或非法能力字段返回空值，调用方不得以示例默认值继续规划。`roughness_exceeded` 与 `roughness_invalid` 是独立可审计限制因素；粗糙度越限使整段不可通行，无效粗糙度同时使结果 `TERRAIN_INVALID`。`CollisionLayerResult` 保存逐栅格分类、信息缺口、地图/配置/策略版本和碰撞风险审计；`CollisionSweepResult` 保存完整足迹是否无碰撞、扫掠位姿/单元数量以及离散化裕量。扫掠入口在任何几何评价前要求碰撞层与地形层的完整 `MapVersion` 和分析配置版本一致，错配返回 `INPUT_INVALID`。`TraversabilityResult` 保存是否可通行、最坏粗糙度、去重限制原因、最坏纵坡均值及上下界、横坡绝对值上界、完整 `TerrainGradientRiskAudit`、逐事件台阶穿越、完整台阶高度、履带滚转角、局部落差、离群点、左右履带最小支撑覆盖率和最差地形估计状态。搜索器必须组合碰撞扫掠与方向门禁结果，不自行重复地形几何判断，也不得把台阶不连续分类当作普通可通行单元。

#### CableStateTracker

```cpp
class CableStateTracker {
public:
    // 仅使用已实际执行的运动、触地点观测和同步放缆遥测推进状态及 laying_memory
    CableTrackerSnapshot update(
        const ExecutedRobotSegment& executed_segment,
        const CableTelemetry& telemetry,
        const std::optional<TouchdownObservation>& observation
    );

    CableTrackerSnapshot snapshot() const;
    CableTrackerSnapshot begin_new_task(MonotonicTime timestamp);
    CableTrackerSnapshot mark_state_lost(MonotonicTime timestamp,
                                         std::string reason);
};
```

`CableStateTracker` 不接收候选规划路径。其 `laying_memory` 只由已执行铺设或确认触地点观测推进；候选标签的记忆由 `CableLayingEvaluator` 在搜索内部复制推进，避免未来状态污染当前状态。除显式任务起点外，实际记忆覆盖长度不足 $L_{\text{support,eval}}$、末端点无效或与当前触地点观测不连续时，状态快照无效并禁止规划。

#### ReferenceProgressTracker

```cpp
class ReferenceProgressTracker {
public:
    explicit ReferenceProgressTracker(
        ReferenceProgressAssociationParameters parameters);

    ReferenceProgressSnapshot reset_for_new_task(
        const ReferenceLine& reference, double initial_progress_m,
        MonotonicTime timestamp);

    ReferenceProgressSnapshot update_from_executed_laying(
        const ExecutedTouchdownSegment& executed_touchdown,
        const ReferenceLine& reference
    );

    ReferenceProgressSnapshot snapshot() const;
};
```

Tracker 只接受已执行铺设或经确认的触地点观测，不接受候选路径终端进度。参考线版本改变时必须显式重置或完成版本间进度迁移。

#### CableModel

```cpp
class CableModel {
public:
    // 搜索使用认证运行包络做保守传播
    CablePrediction predict_search(
        const CableState& initial_state,
        const GeometricPath& robot_segment,
        const CableContext& context
    ) const;

    // 最终验证必须使用完整时序剖面；仅VALIDATION传播路径相关协方差
    CablePrediction predict(
        const CableState& initial_state,
        const TimedPath& robot_path,
        const CableContext& context
    ) const;
    
    void set_parameters(const CableModelParameters& params);
};
```

#### CableLayingEvaluator

```cpp
enum class CableLayingFailure {
    none,
    curvature_exceeded,
    support_proxy_exceeded,
    forbidden_area_intersection,
    terrain_data_invalid,
    numerically_invalid,
    duplicate_touchdown_point,
    mechanical_history_incomplete
};

struct CableLayingLimits {
    uint64_t version;
    std::string operating_domain_id;
    double preferred_curvature_per_m;
    double maximum_curvature_per_m;
    double curvature_evaluation_spacing_m;
    double support_evaluation_length_m;
    double medium_support_proxy_range_m;
    double maximum_support_proxy_range_m;
    double minimum_terrain_confidence;
    double minimum_distinct_touchdown_distance_m;
    double bend_weight;
    double terrain_risk_weight;
    double roughness_weight;
};

struct CableLayingEvaluation {
    bool valid;
    bool hard_feasible;
    std::vector<CableLayingFailure> failure_reasons;
    std::vector<CableLayingFailureSegment> failure_segments;
    uint64_t limits_version;
    uint64_t terrain_map_sequence;
    uint64_t terrain_analysis_config_version;
    std::string operating_domain_id;
    std::string risk_semantics;
    double maximum_absolute_curvature_per_m;
    std::optional<Vector2m> maximum_absolute_curvature_position_m;
    double maximum_support_proxy_range_m;
    std::optional<Vector2m> maximum_support_proxy_position_m;
    double terminal_support_window_length_m;
    double soft_cost;
    CableConstraintMemory terminal_memory;
};

class CableLayingEvaluator {
public:
    std::optional<CableConstraintMemory> canonicalize_memory(
        const CableConstraintMemory& memory,
        const CableLayingLimits& limits,
        CableHistoryBoundary history_boundary) const;

    CableLayingEvaluation evaluate_segment(
        const CableConstraintMemory& initial_memory,
        const GeometricPath& touchdown_segment,
        const std::vector<CableState>& state_profile,
        const TerrainLayers& terrain,
        const CableLayingLimits& limits,
        CableHistoryBoundary history_boundary
    ) const;

    CableLayingEvaluation evaluate(
        const CableConstraintMemory& initial_memory,
        const GeometricPath& touchdown_path,
        const std::vector<CableState>& state_profile,
        const TerrainLayers& terrain,
        const CableLayingLimits& limits,
        CableHistoryBoundary history_boundary
    ) const;

    bool future_equivalent(
        const CableConstraintMemory& a,
        const CableConstraintMemory& b
    ) const;
};
```

`CableLayingEvaluator` 是缆线机械硬约束及其有限历史推进的唯一判定入口。搜索调用 `evaluate_segment`，按第6.3节延迟结算右侧曲率窗口不足的转角；最终验证调用 `evaluate`，从当前实际 `laying_memory` 开始对重积分得到的完整触地点路径执行终点裁剪复检。两种形式必须使用同一组阈值和相同地形语义；路径坐标系必须与锁定地图一致，结果审计必须记录限制版本、地图序列、地形分析配置版本和运行域，无效评价也不得丢失这些输入依赖。`hard_feasible=false` 必须显式剪枝或拒绝候选，禁止仅通过增加软代价继续放行。

`future_equivalent` 默认要求两个规范化记忆逐样本一致。若实现使用量化等价，必须证明量化误差已由曲率和地形硬门禁中的独立保守裕度覆盖；仅比较 `canonical_signature` 哈希相等不能作为证明。任何不能证明等价的记忆均保留为同一基础键下的不同标签。

#### CableUncertaintyEnvelopeBuilder / Manager

```cpp
struct CableUncertaintyEnvelope;
struct OperatingDomain;
struct ExecutionOperatingEnvelope;

class CableUncertaintyEnvelopeBuilder {
public:
    CableUncertaintyEnvelope buildCertifiedUpperBound(
        const OperatingDomain& gamma_h,
        const ReferenceLine& reference,
        SensorHealthMode sensor_mode,
        const CableModelParameters& model,
        const ExecutionOperatingEnvelope& execution_envelope,
        uint64_t generator_version,
        MonotonicTime generation_timestamp
    ) const;
};

class CableUncertaintyEnvelopeManager {
public:
    EnvelopeRegistrationResult registerValidated(
        uint64_t envelope_version, CableUncertaintyEnvelope envelope,
        EnvelopeCoverageCertification coverage_certification);
    EnvelopeInvalidationResult setCurrentContext(
        const EnvelopeLookupKey& context, uint64_t context_sequence,
        MonotonicTime changed_at);
    std::optional<LockedCableUncertaintyEnvelope> getValidated(
        const EnvelopeLookupKey& key, MonotonicTime now);
    EnvelopeQueryResult query(
        const LockedCableUncertaintyEnvelope& locked,
        double reference_progress_m, MonotonicTime now);
    bool registerDependentPlan(
        uint64_t plan_sequence,
        const LockedCableUncertaintyEnvelope& locked, MonotonicTime now);
    bool registerDependentLease(
        uint64_t lease_sequence, uint64_t plan_sequence,
        const LockedCableUncertaintyEnvelope& locked, MonotonicTime now);
    EnvelopeInvalidationResult invalidate(
        uint64_t envelope_version, MonotonicTime invalidated_at);
    EnvelopeInvalidationResult expire(MonotonicTime now);
    EnvelopeAuditResult auditActualLateralStddev(
        const LockedCableUncertaintyEnvelope& locked,
        double reference_progress_m, double actual_lateral_stddev_m,
        double audit_tolerance_m, MonotonicTime audited_at);
};
```

Builder 可以在部署前或受控后台任务中运行，不能在搜索超时预算内临时用少量随机样本生成“上界”。Manager 以参考线、传感器模式、运行域、缆线模型和执行运行包络的完整版本元组检索，只发布通过验证且依赖完全匹配的不可变包络快照。模型或执行运行包络升级后，旧包络不得凭相同 `operating_domain_id` 继续使用。

#### CableCorridorEvaluator

```cpp
class CableCorridorEvaluator {
public:
    explicit CableCorridorEvaluator(CableCorridorRiskPolicy policy);
    CableCorridorResult evaluate_pointwise(
        const CableCorridorEvaluationInput& input) const;
};
```

`CableCorridorEvaluationInput` carries the reference-line snapshot, predicted touchdown geometry, per-sample reference progress and covariance, plus a versioned `CableCorridorIntervalBoundCertificate`. `CableCorridorRiskPolicy` requires a calibrated `epsilon_point`, nominal/absolute half-widths, a marginal-length limit and a conservative boundary margin. `evaluate_pointwise` returns `PASS`/`MARGINAL`/`VIOLATION` per sample, integrates interval lengths (including the boundary margin), and records the deterministic-reference and coordinate-transform uncertainty gates. `CableCorridorResult` carries the fixed risk string `POINTWISE_ONLY:NO_PATH_JOINT_RISK_GUARANTEE`; the uncertainty envelope manager and timed verifier use `POINTWISE_ENVELOPE_ONLY:NO_PATH_JOINT_RISK_GUARANTEE` for their separate envelope-audit semantics. `epsilon_path` and an error-correlation length are intentionally not represented in the first-version runtime contract.

首版接口将参考线明确标记为确定量，内部固定 $\Sigma_{\text{ref}}=0$。搜索评价只消费锁定版本的横向包络；验证评价传播真实 $\Sigma_c$ 并审计其未超过包络。只有未来出现真实参考线误差来源时才扩展接口，不提前引入无数据支撑的交叉协方差。

#### PathSmoother

```cpp
enum class PathBoundarySource {
    synchronized_actual_state,
    committed_segment_terminal,
    planned_goal
};

struct PathBoundary {
    double x_m;
    double y_m;
    double heading_rad;
    std::optional<double> curvature_per_m;
    PathBoundarySource curvature_source;
    MonotonicTime pose_timestamp;
    MonotonicTime curvature_timestamp;
    std::uint64_t source_sequence_number;
};

struct SmoothingLimits {
    std::uint64_t version;
    std::uint64_t output_path_version;
    double spatial_step_m;
    double maximum_curvature_per_m;
    double maximum_curvature_rate_per_m2;
    double minimum_segment_length_m;
    double topology_tube_radius_m;
    Duration timeout;
    Duration maximum_boundary_time_skew;
    PathConstraintResiduals allowed_residuals;
    SmoothingObjectiveWeights objective_weights;
};

struct TrackabilityResult {
    bool valid;
    std::string reason;
    PathConstraintResiduals residuals;
};

enum class SmoothingStatus {
    success,
    boundary_state_invalid,
    seed_infeasible,
    solver_timeout,
    solver_failed,
    constraint_residual_exceeded,
    trackability_validation_failed
};

struct SmoothingResult {
    SmoothingStatus status;
    std::optional<GeometricPath> path;
    PathConstraintResiduals residuals;
    Audit audit;
};

class PathSmoother {
public:
    SmoothingResult smooth(
        const GeometricPath& raw_path,
        const PathBoundary& start,
        const PathBoundary& goal,
        const SmoothingLimits& limits
    ) const;

    TrackabilityResult validateTrackability(
        const GeometricPath& path,
        const GeometricPath& raw_path,
        const PathBoundary& start,
        const PathBoundary& goal,
        const SmoothingLimits& limits
    ) const;
};
```

`success` 只在求解器成功、公共几何有效、动力学/G2/曲率残差和拓扑管复核均通过后返回。`PathSmoother` 不接收地图；起点曲率必须来自同步实际状态或承诺段元数据，接口不提供隐式零曲率默认值。`PathSmoothingSolver` 是其可注入求解 seam，`SmoothingResult::Audit` 和成功路径的 `PathSmoothingMetadata` 记录求解器版本/状态、限制版本、迭代数、目标分解、最大曲率、最大曲率变化率和全部残差。

#### PathCandidateVerifier

```cpp
struct PathCandidateVerificationContext {
    MapSnapshot map;
    TerrainLayers terrain;
    RobotOperatingArea robot_operating_area;
    Covariance2dM2 robot_relative_obstacle_covariance_m2;
    RobotCollisionRiskPolicy collision_risk_policy;
    RobotCapability robot_capability;
    TrackFootprint track_footprint;
    TerrainGradientRiskPolicy terrain_gradient_risk_policy;
    double maximum_sweep_spacing_fraction;
    double operating_area_clearance_m;
    double geometric_curvature_tolerance_per_m;
    double heading_tolerance_rad;
    double curvature_rate_tolerance_per_m2;
};

enum class PathCandidateVerificationStatus {
    valid,
    input_invalid,
    geometry_invalid,
    boundary_residual_exceeded,
    curvature_limit_exceeded,
    curvature_rate_limit_exceeded,
    operating_area_violation,
    collision_violation,
    traversability_violation
};

class PathCandidateVerifier {
public:
    PathCandidateVerificationResult verify(
        const GeometricPath& path,
        const PathBoundary& start,
        const PathBoundary& goal,
        const SmoothingLimits& smoothing_limits,
        const PathCandidateVerificationContext& context
    ) const;
};
```

该模块先通过 `auditPathGeometry` 对起点曲率来源执行 `synchronized_actual_state` / `committed_segment_terminal` 白名单，并从位置独立重算航向、三点曲率和曲率变化率，再在当前地图上复检完整足迹、碰撞和方向地形。非法 `planned_goal` 起点返回 `start_boundary_provenance_invalid` / `input_invalid` 且不进入扫掠；终点仍允许 `planned_goal`。`mergePathsG2` 是独立自由函数；拼接后会清除旧 `smoothing` 元数据，必须重新调用本模块。该模块不执行缆线验证，也没有可替代主循环阶段超时的内部成功状态。

#### TrajectoryParameterizer

```cpp
enum class ParameterizationStatus {
    success,
    deadline_exceeded,
    initial_state_invalid,
    execution_envelope_mismatch,
    dynamics_infeasible,
    payout_infeasible,
    stopping_constraint_infeasible,
    numerically_invalid
};

struct TrajectoryInitialState {
    double ground_speed_mps;
    double payout_speed_mps;
    double payout_acceleration_mps2;
    double tension_n;
};

struct TrajectoryParameterizationLimits {
    std::uint64_t version;
    double sample_period_s;
    double terminal_speed_mps;
    double stopping_distance_margin_m;
    bool require_terminal_stop;
    Duration timeout;
    std::uint64_t execution_profile_version;
};

struct ParameterizationResult {
    ParameterizationStatus status;
    std::optional<TimedPath> trajectory;
    ParameterizationDiagnostics diagnostics;
};

class TrajectoryParameterizer {
public:
    using Clock = std::function<MonotonicTime()>;
    TrajectoryParameterizer();
    explicit TrajectoryParameterizer(Clock clock);

    ParameterizationResult parameterize(
        const GeometricPath& geometry,
        const TrajectoryInitialState& initial_state,
        const ExecutionOperatingEnvelope& certified_envelope,
        const TrajectoryParameterizationLimits& limits
    ) const;
};
```

该模块逐字段保持输入几何不变，并按 `certified_envelope.limits` 联合满足速度、加减速度、横向加速度 $v^2|\kappa|$、终端停车距离、出缆速度/加速度、出缆速度跟踪误差和张力范围。曲率变化率由前置几何审计负责；当前接口不接收 terrain 或 capability 的第二份近似数据。正的阶段 `timeout` 通过默认 `steady_clock` 或测试注入的单调时钟覆盖全部循环和成功提交边界。只有 `success` 返回完整 `TimedPath`；`deadline_exceeded` 不携带部分剖面，主循环保留独立的 `PARAMETERIZATION_DEADLINE_EXCEEDED` 根因。

#### PlanValidityEvaluator

```cpp
enum class PlanValidationAction { reuse, replan, stop };

enum class PlanValidationStatus {
    valid,
    state_mismatch,
    context_mismatch,
    input_expired,
    execution_profile_mismatch,
    execution_profile_invalid,
    stopping_distance_insufficient,
    robot_constraint_violation,
    cable_model_invalid,
    covariance_envelope_unavailable,
    covariance_envelope_breach,
    cable_corridor_invalid,
    cable_corridor_violation,
    cable_laying_invalid,
    input_invalid
};

struct PlanValidityEvaluation {
    PlanValidationAction action;
    PlanValidationStatus status;
    bool valid;
    uint64_t evaluator_config_version;
    std::string parameter_profile_id;
    std::shared_ptr<const TimedPath> remaining_path;
    std::optional<PlanValidationLease> lease;
    std::optional<CablePrediction> cable_prediction;
    Diagnostics diagnostics;
    std::vector<std::string> issues;
};

struct PlanValidationCorridorIntervalBound {
    uint64_t certificate_version;
    double upper_bound_error_m;
};

struct PlanValidationContext {
    TerrainLayers terrain;
    CableContext cable_context;
    CableCorridorRiskPolicy corridor_policy;
    PlanValidationCorridorIntervalBound corridor_interval_bound;
    ReferenceProgressAssociationParameters reference_progress_parameters;
    CableLayingLimits laying_limits;
    CableHistoryBoundary history_boundary;
    std::optional<LockedCableUncertaintyEnvelope> locked_envelope;
    CableUncertaintyEnvelopeManager* envelope_manager;
    PathCandidateVerificationContext path_context;
    SmoothingLimits smoothing_limits;
    PathBoundary goal_boundary;
    double envelope_audit_tolerance_m;
    bool reference_is_deterministic;
    bool covariance_includes_coordinate_transform_error;
};

class PlanValidityEvaluator {
public:
    PlanValidityEvaluator(CableModel cable_model,
                          PlanValidityEvaluatorConfig config);

    PlanValidityEvaluation validateRemainingPlan(
        const ImmutablePlanningResult& plan,
        const SynchronizedValidationInputs& inputs,
        const PlanValidityContext& context,
        MonotonicTime now);

    PlanValidityEvaluation validatePublicationCandidate(
        const ImmutablePlanningResult& plan,
        const SynchronizedValidationInputs& inputs,
        const PlanValidityContext& context,
        MonotonicTime now);

    uint64_t next_lease_sequence() const;
};
```

该模块内部复用 `PathCandidateVerifier`、`CableModel` 和 `TimedCableCandidateVerifier` 的公共安全入口，调用方不得只传地图调用缩减版布尔函数。同步状态和冻结快照属于 `SynchronizedValidationInputs`；算法阈值、派生地形、当前锁定包络和验证器上下文属于 `PlanValidityContext`，两者不能重新拼成混合 revision。验证开始时逐项比对计划、同步输入、验证上下文和包络携带的缆线模型、执行运行包络、参考线、双空间域、传感器模式、运行域、地形梯度策略和走廊风险策略版本；任一缺失或错配返回 `context_mismatch` 或更早的 `input_invalid`，而不是旧设计中的笼统 `VALIDATION_CONTEXT_INVALID`。

仅当 `status == valid` 时，`remaining_path`、`lease` 和 `cable_prediction` 三项同时存在；剩余时序轨迹必须按当前同步状态裁剪，缆线必须从当前实测状态重预测，滞回只能比较其几何和代价。`validateRemainingPlan` 用于已授权当前计划，以 `ExecutionTrackingState.tracked_arc_length_m` 定位其当前执行进度，并要求计划剖面版本等于同步输入中执行器实际跟踪的版本；`validatePublicationCandidate` 用于尚未发布的候选，以候选路径首点弧长作为当前实测状态的候选坐标原点，仍检查同一实测状态、执行连续性及全部冻结依赖，但允许候选剖面版本相对当前跟踪版本单调前进。该区别同时适用于从零起算的新路径和保留原弧长的承诺前缀拼接路径。候选版本回退、执行跟踪状态与同步依赖自身错配仍失败关闭。无法与相应剖面连续衔接时返回 `execution_profile_mismatch`。其他状态不得携带可执行租约，失败不得推进 `next_lease_sequence`。租约持久撤销不属于该评估器，由 `ExecutionLeaseMonitor::revokeLease` 负责。`covariance_envelope_breach` 必须沿用第7.7节系统故障语义并触发停车和包络失效。

主循环中的 `capture_synchronized_validation_inputs()` 是适配层的一致快照操作：它先用同批已执行运动、触地点观测和放缆遥测推进 `CableStateTracker` 与 `ReferenceProgressTracker`，再原子冻结当前机器人状态、参考进度、执行剖面跟踪状态、缆线状态、遥测，以及当前地图/参考线/作业区/地形梯度策略/走廊风险策略/缆线模型/传感器模式/统计包络/执行运行包络组合。返回的 `SynchronizedValidationInputs.cable_context_mode` 固定为 `validation`；适配层构造 `PlanValidityContext.cable_context` 时必须使用同一冻结依赖和当前遥测并保持该模式，搜索调用方只能从同一快照派生 `search` 视图。包络检索键必须包含参考线版本、传感器模式、运行域、缆线模型版本和执行运行包络版本。任一输入过期、版本组合不完整或捕获期间发生版本变化时返回无效，不得退回循环开始时的旧快照。

#### ExecutionLeaseMonitor

```cpp
struct ActiveExecutionContext {
    MapVersion map_version;
    uint32_t reference_line_version;
    uint32_t robot_operating_area_version;
    uint64_t terrain_gradient_policy_version;
    uint64_t corridor_risk_policy_version;
    uint64_t cable_model_version;
    uint64_t uncertainty_envelope_version;
    uint64_t uncertainty_envelope_generator_version;
    uint64_t execution_operating_envelope_version;
    uint64_t execution_profile_version;
    SensorHealthMode sensor_mode;
    std::string operating_domain_id;
    uint32_t cable_corridor_version;
};

struct ExecutionFeedback {
    uint64_t plan_sequence_number;
    uint64_t execution_profile_version;
    MonotonicTime timestamp;
    double ground_speed_mps;
    double ground_acceleration_mps2;
    double payout_speed_mps;
    double payout_acceleration_mps2;
    double tension_n;
    double tracked_arc_length_m;
    uint64_t sequence_number;
};

enum class ExecutionAuthorizationStatus {
    authorized,
    renewal_required,
    revoke_and_controlled_stop
};

class ExecutionLeaseMonitor {
public:
    ExecutionAuthorization evaluate(
        const PlanningResult& plan,
        const TimedPath& authorized_remaining_trajectory,
        const PlanValidationLease& lease,
        const ActiveExecutionContext& active_context,
        const ExecutionFeedback& feedback,
        MonotonicTime now
    ) const;

    void revokeLease(uint64_t lease_sequence,
                     std::string reason_code,
                     std::string reason,
                     MonotonicTime at) const;
    bool isRevoked(uint64_t lease_sequence) const;
};
```

监控器先校验配置不变量、计划/剩余轨迹/租约配对、独立单调时钟上的租约/反馈时效和完整剖面结构，再将 `active_context` 与计划和租约的完整依赖版本元组逐项比较。只有这些检查通过才推进计划、租约和反馈序列水位；拒绝的高序列不能毒化有效状态。随后按 `tracked_arc_length_m` 对当前剖面插值得到期望值，并同时检查批准绝对范围与相对期望偏差。反馈过期/未来时间、版本或内容不匹配、已撤销/过期/乱序授权、同租约剖面指纹变化、反馈乱序或任一偏差越界时返回 `revoke_and_controlled_stop`，在 monitor 内持久撤销租约并请求受控停车/重规划；不得把异常反馈只记录为告警。进入 `renewal_margin` 返回 `renewal_required`，只触发完整复检，不延长或改写当前租约。

#### CommitmentSafetyEvaluator / CommitmentSafetySupervisor

```cpp
struct ObstacleStoppingEvidence {
    double obstacle_distance_m;
    double certified_stopping_distance_m;
};

struct CommitmentSafetyObservation {
    optional<PathCandidateVerificationResult> robot_validation;
    optional<TimedCableCandidateResult> cable_validation;
    CommitmentSafetyEvent observed_event;
    optional<ObstacleStoppingEvidence> obstacle_stopping;
};

class CommitmentSafetyEvaluator {
public:
    CommitmentSafetyCheckResult evaluate(
        const CommitmentSafetyObservation& observation
    ) const;
};

class CommitmentSafetySupervisor {
public:
    // 可由地图、定位、缆线、执行或版本事件适配器直接调用，不等待规划周期。
    CommitmentSafetyEnforcement handle_event(
        const CommitmentSafetyObservation& observation,
        uint64_t active_lease_sequence,
        MonotonicTime now
    ) const;

    CommitmentSafetyEnforcement enforce(
        const CommitmentSafetyCheckResult& safety,
        uint64_t active_lease_sequence,
        MonotonicTime now
    ) const;
};
```

`CommitmentSafetyEvaluator` 聚合完整机器人路径验证、完整时序缆线验证和异步事件；同时出现多个事件时固定采用 `STOP > REPLAN_URGENT > CONTINUE`，不得让较弱事件掩盖更强硬约束失败。新障碍只有在认证最坏停车距离严格小于障碍距离时才返回 `REPLAN_URGENT`，证据缺失、非有限、为负或停车距离不小于障碍距离时均返回 `STOP`。`CommitmentSafetySupervisor` 构造时必须拒绝缺失的安全停车通道；执行安全动作时必须先把活动租约写入持久撤销集合，再调用该独立通道。下一周期若再次收到同一已撤销承诺租约，主循环忽略该承诺并从最新同步实测状态规划。

#### StabilityManager

```cpp
struct PathHysteresisConfig {
    double relative_cost_threshold{0.1};
    std::optional<double> topology_distance_threshold_m;
};

enum class PathSwitchAction {
    keep_current,
    switch_to_candidate,
    stop
};

struct PathSwitchDecision {
    PathSwitchAction action;
    std::optional<PlanValidationLease> lease;
    std::shared_ptr<const TimedPath> remaining_path;
    std::string reason;
};

struct CommitmentSegmentConfig {
    double commitment_time_s;
    double safety_margin_m;
    std::optional<double> certified_worst_case_stopping_distance_m;
};

struct PathG2MergeLimits {
    double position_tolerance_m;
    double heading_tolerance_rad;
    double curvature_tolerance_per_m;
};

struct ExecutionJoinTolerances {
    double ground_speed_mps;
    double ground_acceleration_mps2;
    double payout_speed_mps;
    double tension_n;
    double payout_acceleration_mps2;
};

struct TimedPathMergeResult {
    bool valid;
    std::optional<TimedPath> trajectory;
    std::string reason;
};

class StabilityManager {
public:
    bool should_switch_path(
        const GeometricPath& current_path,
        const GeometricPath& new_path,
        double current_cost,
        double new_cost) const;

    PathSwitchDecision decide_path_switch(
        const std::optional<PlanValidityEvaluation>& current,
        const PlanValidityEvaluation& candidate,
        double current_cost,
        double candidate_cost,
        MonotonicTime now
    ) const;

    CommitmentExtractionResult extract_commitment_segment(
        const TimedPath& current_trajectory,
        const RobotState& robot_state,
        const CommitmentSegmentConfig& config
    ) const;

    TimedPathMergeResult merge_timed_paths(
        const TimedPath& commitment,
        const TimedPath& new_tail,
        const PathG2MergeLimits& geometric_tolerances,
        const ExecutionJoinTolerances& execution_tolerances,
        const TimedPathFinalVerifier& final_verifier
    ) const;
};
```

`decide_path_switch` 只接受完整的 `PlanValidityEvaluation`。候选无效而当前计划有效时携带当前最新租约保持旧计划；当前计划无效而候选及其代价有效时直接切换；候选代价无效、两者都无效或有效租约来自不同同步验证上下文时按第 10.2 节保持或停车。只有二者有效、未过期、验证配置/profile/完整依赖和四类同步输入时间一致时才调用相对代价/可选 Hausdorff 滞回。规划跨越任一租约截止时间时，带 `RevalidationCallback` 的重载必须重新捕获一次同步上下文并成对复检；缺少回调失败关闭。

主循环候选决策 seam 的当前契约为：

```cpp
struct PlanningCandidateMetadata {
    uint64_t sequence_number;
    MonotonicTime timestamp;
    Duration validity_duration;
    double path_cost;
    ErrorBudget error_budget;
    Diagnostics diagnostics;
};

struct AuthorizedPlanningResult {
    ImmutablePlanningResult plan;
    std::shared_ptr<const TimedPath> remaining_path;
    PlanValidationLease lease;
    double path_cost;
};

PathSwitchDecision MainPlanningLoopStages::decide_candidate(
    const std::optional<PlanValidityEvaluation>& validated_current,
    const PlanValidityEvaluation& validated_candidate,
    double current_cost,
    double candidate_cost,
    const SynchronizedValidationInputs& latest_inputs,
    MonotonicTime now
);
```

`PlanningCandidateMetadata.path_cost` 来自本周期搜索诊断的加权 `solution_cost`；`AuthorizedPlanningResult.path_cost` 将已发布计划的比较基准与不可变计划、剩余轨迹和租约一起保存。`MainPlanningLoop` 在一次最新输入捕获后，以同一个验证时刻复检候选与当前计划，再调用上述 seam。`MainPlanningLoopStages::decide_candidate` 是非虚的公共决策入口并在基类内直接委托构造时配置的 `StabilityManager::decide_path_switch`，因此生产阶段实现和仓库确定性闭环适配器都不能恢复“有效候选总是切换”的旧策略；适配器只可通过只读的决策输入观察 hook 记录调用。keep 分支调用 `reauthorize_current`，switch 分支调用带候选代价的 `publish`，stop 分支撤租并请求停车。实验重放以 `algorithm-experiment/v3` 记录并验证初始授权代价。

`extract_commitment_segment` 只裁剪已授权剩余轨迹，要求认证停车距离和实测/剖面速度同步。`merge_timed_paths` 原样复制承诺前缀，要求运行包络、插值规则和全部批准跟踪限制一致，检查位置/航向/曲率及对地速度/加速度、出缆速度/加速度和张力连接残差，把尾段弧长/时间重锚后生成 `max(prefix_version, tail_version)+1` 的新剖面版本。缺少 `TimedPathFinalVerifier`、完整路径验证失败、版本溢出或任一残差失败均不返回轨迹；合并成功不能替代第 9 节机器人和第 7.7 节缆线完整复检。

#### ScoutCoordinator

`scout_coordinator.hpp` 所有补探专用类型；公共入口按“扫描 -> 紧迫度 -> 目标 -> 请求生命周期”分层：

```cpp
class ScoutCoordinator {
 public:
  InformationGapScanResult identify_gaps_result(
      const ReferenceLine&, const MapSnapshot&, double planning_horizon_m,
      const std::optional<GeometricPath>& candidate_detour) const;

  GapUrgencyAssessment assess_gap_urgency(
      const InformationGap&, const RobotState&,
      double current_reference_progress_m,
      const std::optional<TimedPath>& approved_remaining_path,
      double planning_horizon_m) const;

  std::vector<PrioritizedGapAssessment> assess_gaps(...) const;
  ScoutTargetGenerationResult generate_scout_target(...) const;
  std::vector<ScoutTarget> generate_scout_targets(...) const;
  ScoutDistanceAssessment assess_distance_constraint(
      const Pose2d& main_robot_pose, const Pose2d& scout_robot_pose) const;

  ScoutRequestIssueResult issue_scout_request(
      const ScoutTarget&, MonotonicTime now);
  ScoutMapUpdateResult correlate_map_update(
      std::uint64_t request_sequence, const ReferenceLine&,
      const MapSnapshot& updated_map, MonotonicTime now);
  ScoutRequestExpiryResult expire_scout_requests(MonotonicTime now);
  std::optional<ScoutRequest> scout_request(
      std::uint64_t request_sequence) const;
};
```

`InformationGapScanResult`、`GapUrgencyAssessment`、`ScoutTarget`、`ScoutRequest` 和各 disposition 明确保留有效性、版本、时间和结构化问题。当前设计与新调用统一使用 snake_case 入口；头文件中仍存在 `identifyGaps`、`assessGapUrgency`、`generateScoutTarget` 和 `isDistanceConstraintSatisfied` 兼容包装，但它们不形成第二套数据契约，也不应出现在新的设计伪代码中。补探结果不会直接调用 `MainPlanningLoop` 或发布控制命令，编排边界见第 11.5 和第 16 节。

### 14.3 数据结构

本节不再复制 C++ 结构体的近似版本。下表给出当前公共契约的唯一所有者；正文中的 `Path`、`Pose`、`PlanningRequest` 或 `PlanningResultVersionMetadataView` 等数学/概念称呼不得被实现为第二套同义运行时类型。`ExecutionOperatingEnvelope` 则是 `cable_model.hpp` 中的实际公共类型。

| 公共边界 | 唯一定义 | 本设计采用的实际类型 |
|---|---|---|
| 基础状态、路径、风险、诊断、结果 | `core/data_contract.hpp` | `MonotonicTime`、`Duration`、`Pose2d`、`RobotState`、`CableState`、`CableTelemetry`、`ReferenceProgress`、`GeometricPath`、`TimedPath`、`ErrorBudget`、`PlanningDependencyVersions`、`Diagnostics`、`PlanningResult` |
| 地图、参考线与双空间域 | `core/versioned_snapshot.hpp` | `MapSnapshot`、`ReferenceLine`、`RobotOperatingArea`、`CableCorridor`、`VersionedPlanningSnapshot` |
| 原子复检输入 | `core/synchronized_validation_inputs.hpp` | `ExecutionTrackingState`、`TrackerUpdateReceipt`、`ValidationInputFrame`、`SynchronizedValidationInputs`、`ValidationInputCaptureResult` |
| 结果发布边界 | `core/planning_result.hpp` | `ImmutablePlanningResult`、`PlanningResultPublication`、`PlanningResultPublisher` |
| 参数与门禁 | `core/parameter_config.hpp` | `ParameterConfig`、`ParameterProfileMode`、`ParameterValidationResult` |

关键类型的 as-built 语义如下：

| 类型 | 当前字段边界与校验语义 |
|---|---|
| `Pose2d` / `RobotState` | 位姿含 `x_m`、`y_m`、`heading_rad`、位姿时间；机器人状态另含对地速度、当前曲率、独立曲率时间和非零流序号。平滑起点仍需按第 9 节检查位姿/曲率同步，失败原因是阶段级 `BOUNDARY_STATE_INVALID`。 |
| `CableState` / `ReferenceProgress` | 缆线状态显式区分 `tracked` 与 `search_mean`，携带滞后角、可选方差、机械历史、时间和序号；参考进度绑定参考线版本并携带弧长、时间和序号。 |
| `GeometricPath` | `PathPoint` 使用弧长、位置、航向和曲率的显式 SI 字段；弧长严格递增、航向已规范化。`PathMetadata` 携带路径版本、坐标系、参考线版本、插值规则和可选平滑证据。不存在公共 `Path` 别名。 |
| `ExecutionProfile` / `TimedPath` | profile 携带自身版本、执行运行包络版本、插值规则、显式停止点、样本和获批跟踪限制；每个样本含弧长、相对时间、对地速度/加速度、出缆速度/加速度与张力设定。样本弧长和时间单调，必须与几何、横向加速度及全部获批范围一致；任何语义字段变化都产生新版本。 |
| `ErrorBudget` | 分离机器人位置协方差与逐触地点协方差，分别携带 `epsilon_robot`、局部地形梯度 `epsilon`、`epsilon_point` 和首版为空的 `epsilon_path`；标定数据、风险策略、模型、包络生成器、执行运行包络、运行域和传感器模式均可审计。首版两个 path-joint 标志必须为 `false`。 |
| `Diagnostics` | 固定包含 schema、随机种子、输入版本、单位系统、运行域、风险语义、完整依赖元组和结构化 entries；不能用自由文本替代版本或风险字段。 |
| `PlanningResult` | 使用第 3.3 节完整 `PlanningState`；同时携带时序机器人轨迹、几何缆线路径、终端缆线状态、模型/走廊/机械结果、误差预算、13.3 节依赖字段和诊断。实际字段包含 `uncertainty_envelope_generator_version`，不得再沿用遗漏该字段的旧概念视图。 |

`CableValidationStatus` 的公共值为 `pass`、`marginal`、`violation`。`CorridorEvaluationValidity` 除风险策略、协方差、分布和包络状态外，还区分 `input_invalid`、`reference_version_mismatch` 与 `coordinate_transform_error_missing`；`CableCorridorResult` 必须携带 `hard_feasible`、逐点依据、边缘/违规长度、区间上界证书、评估时间、运行域、标定数据和明确的 pointwise-only 风险语义。算法公式与当前字段含义已经由 D04 对齐，后续重构不得删除这些审计边界。

模块专用请求、状态和结果继续由各自公共头文件所有，例如 `PlanningEvent`、`PathBoundary`、`PlanValidationLease` 和搜索/平滑状态；第 10、12、14.2 与 16 节的伪代码只是调用关系，不是替代公共契约的 schema。

---

## 15. 参数体系

### 15.1 参数分层

`parameter_config.hpp` 的 `ParameterConfig` 是当前唯一聚合契约，根字段为 `schema_version`、`profile_id`、`mode` 和 `operating_domain_id`。其余参数只按下列实现分组表达；本节后续的物理/算法分类只解释来源，不构成另一套配置 schema。

| 配置组 | 当前职责 |
|---|---|
| `robot` | 机器人标定版本/数据集、几何、曲率与速度能力、上下坡/横坡、爬阶/落差、履带支撑、粗糙度、障碍裕量和机器人不确定性存在性 |
| `terrain_gradient_risk` | 局部梯度策略、地形分析配置、覆盖模型/系数、独立数据集与运行域 |
| `robot_collision_risk` | 机器人碰撞 `epsilon`、最低地图置信度、策略/数据集与运行域 |
| `spatial_domains` | 机器人作业区和缆线走廊各自的 id、版本和非空证明 |
| `execution` | 执行运行包络、速度/加减速度、出缆/张力、采样、停车与跟踪/G2 阈值 |
| `cable` | 缆线模型/数据集、放缆点、触地点模型、协方差传播、机械曲率、支撑代理、地形置信度和禁放区 |
| `statistical_risk` | 走廊风险策略、独立标定、统计包络/生成器/执行版本、`epsilon_point`、走廊宽度、边缘长度和包络审计域 |
| `path_reuse` | 最大复用时长、机器人/缆线/遥测/执行跟踪最大年龄、租约监控周期和续租提前量 |
| `search` | 标签预算、状态离散化、参考进度关联、连续扫掠裕量和五项软代价权重 |
| `task` | 铺设合格长度比例、通信/前探距离和当前补探策略的版本、阈值、滞回、权重与超时 |

`ParameterProfileMode` 只有 `production` 与 `non_production_capability_profile`。当前两种模式都会检查 schema、profile id、运行域、模式配对，以及机器人、碰撞风险和部分补探数值；production 再执行大部分能力、版本、标定、依赖和硬约束门禁。`ParameterValidationResult.non_production` 是当前明确的非生产标记；它不构成机器人能力证据，也不能被下游报告删去后冒充生产配置。

当前参数校验已完成 T60 代码门禁：两种 profile 对全部 optional 数值执行有限性和非负/比例边界检查；production 强制机器人长/宽/高、最小转弯半径、终端速度和铺设成功率目标（`R_lay >= 0.8`），拒绝 pending/未知地形覆盖模型，要求地形风险运行域与根运行域一致，并校验地形策略、分析配置和独立数据集的版本 provenance 一致，同时要求至少一个受支持且唯一的传感器健康模式。`production_ready` 只表示这些结构化代码门禁通过，不代表外部机器人能力或地形/缆线风险标定已完成；T50-T52、T57 仍保持阻塞。

除 `schema_version = parameter-config/v1` 和少数明确的模型标签字符串外，默认构造值都是“未提供”，不是生产默认能力。所有示例、合成夹具和未标定数值必须使用非生产 profile；生产模式不能用示例值、零值、字段别名或跨组回退补齐缺项。

**设计概念到当前配置组的归属**（所有 TBD 保持未提供）：

**机器人标定参数**（`robot`、`robot_collision_risk` 与 `execution`）：
- 几何尺寸（长、宽、高）
- 左右履带有效支撑多边形和中心距 $B_{\text{eff}}$
- 最小转弯半径 $\rho_{\min}$
- 最大上坡角 $\alpha_{\max}^{\text{up}}$（TBD，保持空值）
- 最大下坡角 $\alpha_{\max}^{\text{down}}$（TBD，保持空值）
- 最大向上台阶 $h_{\max}^{\text{climb}}$（TBD，保持空值）
- 最大向下落差 $h_{\max}^{\text{drop}}$（TBD，保持空值）
- 最大横坡、支撑滚转角和最小履带支撑率（TBD）
- 基础安全距离 $d_{\text{safe}}$
- 机器人定位与控制跟踪协方差 $\Sigma_{\text{robot}}$
- 对地速度、加速/制动、横向加速度和停车距离能力边界

**缆线模型参数**（`cable`、`statistical_risk` 与 `execution`，待标定）：
- 放缆点偏置 $(x_r, y_r)$
- 触地点等效水平距离 $L_{\text{td}}$
- 方向响应长度 $L_{\psi}$
- 最大有效滞后角 $\delta_{\max}$
- 出缆速度跟踪误差上限 $\epsilon_{\text{payout}}$
- 有效张力范围 $[T_{\min},T_{\max}]$
- 放缆速度/加速度能力、剖面跟踪误差和张力设定/实测偏差边界
- 触地点协方差传播参数与残差模型 $\Sigma_c$
- 制造商最小弯曲半径 $R_{\text{bend,min}}$
- 偏好曲率 $\kappa_{\text{cable}}^{\text{preferred}}$、机械硬上限 $\kappa_{\text{cable}}^{\max}$ 与固定曲率评价间距 $L_{\kappa,\text{eval}}$
- 悬空代理物理窗口 $L_{\text{support,eval}}$ 与硬门禁 $\Delta h_{\text{support}}^{\max}$
- 缆线禁放区图层及其数据有效性规则

**算法参数**（`terrain_gradient_risk`、`search` 与各模块受控配置，可调优但仍需版本）：
- 地图分辨率 $\Delta r$
- 搜索离散化 $\Delta_{xy}$, $\Delta_{\theta}$, $\Delta_{\delta}$
- 搜索活动标签总预算（达到预算返回 `TIMEOUT`，不得截断每状态机械历史标签）
- 地形平面拟合窗口、稳健损失和最小有效支撑率
- 局部梯度覆盖风险 $\epsilon_{g,\text{local}}$、覆盖系数 $\beta_g$ 及标定数据版本
- 规划窗口大小 $L_{\text{window}}$
- 承诺段长度 $L_{\text{commit}}$ 或 $t_{\text{commit}}$
- 路径滞回阈值 $\Delta c_{\text{threshold}}$
- 平滑器离散步长、求解超时、信赖域半径和约束残差容差
- 最大空间曲率变化率 $u_{\max}=|d\kappa/ds|_{\max}$ 与 G2 拼接容差
- 代价权重 $w_{\text{length}}$, $w_{\text{curvature}}$, $w_{\text{td,center}}$, $w_{\text{td,margin}}$, $w_{\text{rough}}$, ...
- 规划超时 $T_{\text{plan}}^{\max}$
- 地图有效期 $T_{\text{map}}^{\max}$
- 路径复用最长租约 $T_{\text{reuse,max}}$、租约监控周期与续租提前量
- 机器人状态、缆线状态和放缆遥测的最大年龄
- 时序参数化采样间隔、执行剖面跟踪阈值和剖面版本规则
- 单位置/相关区段风险 $\epsilon_{\text{point}}$
- 整窗口风险 $\epsilon_{\text{path}}$（首版TBD，不实现）
- 误差相关长度 $L_{\text{error-correlation}}$（首版TBD，不实现）

**任务参数**（`spatial_domains` 与 `task`，由任务定义）：
- 允许施工走廊宽度 $w_{\text{corridor}}$
- 允许落点误差（待公司确认）
- 实际布放合格长度比例目标 $R_{\text{lay}}\geq0.8$
- 通信距离上限 $d_{\text{comm}}^{\max}$
- 期望前探距离 $d_{\text{scout}}^{\text{desired}}$
- 补探策略版本、地图置信度、采样/合并、最小安全距离、规划提前量、平均速度和紧迫度滞回
- 传感器覆盖、探测走廊、继续/停止距离、四项排序权重和请求超时

### 15.2 配置文件结构

`load_parameter_config` 接受确定性的 YAML-like 标量子集：可选 `UP_CONFIG 1` 头、缩进 section 或 dotted key、每行一个 `key: value`、注释和显式 `null`。`serialize_parameter_config` 统一输出带 `UP_CONFIG 1` 的 dotted-key 审计形式。loader 已拒绝未知键、非有限浮点数、基本格式错误、未知 profile mode、非法布尔值、带符号/尾随字符/溢出的无符号值和重复 canonical 标量键；`statistical_risk.sensor_health_mode` 是唯一显式允许重复并按输入顺序追加的列表字段。加载后仍必须用与文件 `mode` 相同的模式调用 `validate_parameters`；严格解析不等同于完整 production 参数就绪。

以下只是可解析的最小非生产 profile，不包含任何能力值，也不是生产模板：

```yaml
UP_CONFIG 1
schema_version: parameter-config/v1
profile_id: design-example-unvalidated
mode: non_production_capability_profile
operating_domain_id: synthetic-only
```

生产配置必须使用 15.1 节的真实字段名补齐全部相关组，并通过 `production_ready(config)`；缺字段时保留为空并失败关闭，不能从本文件、测试夹具或相邻字段推导默认值。未知 profile mode 在 loader 阶段结构化拒绝；严格标量解析只保证文本不会被静默重解释，随后必须调用 `validate_parameters` 执行 T60 的全部字段有限性和 production 能力/标定门禁。地形覆盖模型必须是 `calibrated_gaussian`、`empirical_bounded` 或 `deterministic_bounded` 之一；统计包络的 `sensor_health_mode` 至少声明一个受支持且唯一的 `imu`、`depth`、`odometry` 或 `localization` 模式。通过这些代码门禁仍不等价于 T50-T52 的外部标定完成。

**历史概念映射示意（不是当前 loader schema，所有数字均为未验证非生产占位，禁止用于 production）**：

下列嵌套名称只用于阅读旧设计概念；当前实现只接受 `ParameterConfig` 中定义的 canonical key，不能直接加载本块，也不得将这些名称恢复成第二套 schema。

```text
robot:
  type: "main_robot"  # 或 "scout_robot"
  dimensions:
    length: 2.5  # TBD
    width: 1.2   # TBD
    height: 0.8  # TBD
  kinematics:
    min_turning_radius: 3.0  # TBD
    max_curvature: 0.333     # 1/min_turning_radius
    max_curvature_rate_per_meter: null # u_max, 1/m^2, 需控制器和平台试验确认
    curvature_state_max_age: null       # s, 起点曲率与位姿同步门禁
    allow_reverse: false     # TBD
  capability:
    min_ground_speed: null          # m/s, 前进铺设下限, TBD
    max_ground_speed: null          # m/s, TBD
    max_acceleration: null          # m/s^2, TBD
    max_deceleration: null          # m/s^2, 正值, TBD
    max_lateral_acceleration: null  # m/s^2, TBD
    max_slope_up: null              # rad, 最大上坡角, TBD
    max_slope_down: null            # rad, 最大下坡角, TBD
    max_slope_lateral: null         # rad, 最大横坡, TBD
    max_support_roll: null          # rad, 履带支撑滚转上限, TBD
    max_step_climb: null            # m, 最大向上台阶, TBD
    max_step_drop: null             # m, 最大向下落差, TBD
    min_track_support_ratio: null   # 左右履带最小有效支撑率, TBD
    max_roughness: null             # 去趋势粗糙度上限, TBD
  track_support:
    effective_spacing: null         # B_eff, m, TBD
    left_polygon: null              # TBD
    right_polygon: null             # TBD
  uncertainty:
    localization_covariance: null   # 2x2, TBD
    control_tracking_covariance: null # 2x2, TBD

cable_model:
  version: null                       # 生产模式必须使用冻结参数版本
  release_point_offset:
    x: -1.0  # TBD
    y: 0.0   # TBD
  touchdown_distance: 2.0           # L_td, TBD
  direction_response_length: 1.5    # L_psi, TBD
  max_lag_angle: 0.52               # rad, 模型有效范围, TBD
  lag_angle_resolution: 0.087       # rad, 搜索离散化, TBD
  payout_control:
    max_speed_tracking_error: 0.05  # m/s, TBD
    min_speed: null                 # m/s, TBD
    max_speed: null                 # m/s, TBD
    max_acceleration: null          # m/s^2, TBD
    max_tension_tracking_error: null # N, TBD
    min_tension: 10.0               # N, TBD
    max_tension: 100.0              # N, TBD
  model_process_noise_covariance: null # Q_model, 2x2, 待独立数据标定
  uncertainty_envelope:
    validated_version: null           # 生产模式必须引用已验证版本
    generator_version: null           # 确定性上界生成器版本
    cable_model_version: null         # 必须等于当前cable_model.version
    execution_operating_envelope_version: null # 必须等于获批执行运行包络版本
    valid_until: null                  # 认证失效时间；过期后Manager不得返回
    operating_domain_id: null         # Gamma_H定义版本, TBD
    maximum_candidate_length: null    # L_H, m, TBD
    maximum_planning_duration: null   # T_H, s, TBD
    progress_resolution: null         # s_prog分段分辨率, m, TBD
    discretization_margin: null       # rho_env,disc, m, TBD
    audit_tolerance: null             # epsilon_env, m, TBD
    sensor_health_modes: []           # 已覆盖并验证的模式
    generation_method: "bounded_reachability"
  laying_constraints:
    manufacturer_min_bend_radius: null # R_bend,min, m, 生产模式必填
    preferred_curvature: null          # 1/m, 软代价起点, TBD
    maximum_curvature: null            # 1/m, 硬上限且 <= 1/R_bend,min
    curvature_evaluation_spacing: null # L_kappa,eval, m, 生产模式必填
    support_evaluation_length: null     # L_support,eval, m, 固定物理窗口
    medium_support_proxy_range: null    # m, 软风险阈值
    maximum_support_proxy_range: null   # Delta h_support^max, m, 硬门禁
    minimum_terrain_confidence: null    # 缆线触地点地形最低置信度, 生产模式必填
    forbidden_area_layer: "cable_forbidden"
  corridor:
    nominal_width: 10.0             # 期望走廊(软约束), TBD
    absolute_max: 15.0              # 绝对上限(硬约束), TBD
    tolerable_marginal_length: 2.0  # 允许的边缘段长度(m), TBD
    reference_covariance: "fixed_zero" # 首版甲方参考线视为确定量
  risk:
    policy_version: null             # 任一风险参数或走廊阈值变化时递增
    epsilon_point: null             # 单位置/相关区段统计风险, TBD
    epsilon_path: null              # 整窗口联合风险, TBD, 首版不实现
    error_correlation_length: null  # m, TBD, 首版不实现
    distribution_model: "gaussian_pending_calibration"
    calibration_dataset_id: null    # 独立验证数据集标识, TBD

planner:
  map_resolution: 0.2        # TBD
  terrain_analysis:
    config_version: null            # 写入TerrainLayers.analysis_config_version
    surface_window_size: null       # m, 应与履带接地尺度匹配, TBD
    robust_loss: "huber"
    minimum_fit_support_ratio: null # TBD
    step_detection_min_height: null # m, TBD
    gradient_risk:
      policy_version: null          # 生产模式必须引用已验证版本
      terrain_analysis_config_version: null # 必须匹配当前派生地形图层
      epsilon_local: null           # 单个局部梯度覆盖风险，不是路径联合风险
      coverage_multiplier: null     # beta_g，与coverage_model及epsilon一致
      coverage_model: "gaussian_pending_calibration"
      calibration_dataset_id: null  # 独立验证数据集
      operating_domain_id: null     # 海床/声呐/深度等标定运行域
  search_resolution:
    xy: 0.5                  # TBD
    theta: 0.087             # 5 deg
    cable_lag_angle: 0.087   # 5 deg, TBD
    reference_progress: null # Delta_s, m, TBD
  search_resource_budget:
    maximum_active_labels: null # 全局活动标签上限；达到时返回TIMEOUT
    maximum_expansions: null    # 运行时HybridAStarSearchParameters字段
    maximum_planning_duration: null # s，运行时字段由已校验超时配置装配
    analytic_expansion_interval: null # 0禁用；正数按扩展节点周期尝试
    equivalent_label_cost_tolerance: null # m, epsilon_g
  primitive_validation:
    maximum_footprint_motion_ratio: 0.5 # eta, 相对地图分辨率
    collision_sweep_margin: null         # profile门禁；实际搜索扫掠报告eta*r/2
    cable_sweep_margin: null             # rho_cable,sweep, m, 独立标定
  merge_goal_generation:
    merge_distances: null                # m，正值列表
    terminal_heading_offsets: null       # rad，有限值列表
    terminal_lag_angles: null            # rad，必须在模型标定域
    maximum_goal_count: null
    forward_position_tolerance: null     # m
    forward_heading_tolerance: null      # rad
  planning_window:
    length: 50.0             # TBD
    buffer: 20.0             # TBD
  commitment_segment:
    time: 3.0                # TBD
    safety_margin: 3.0       # m, 安全裕量, TBD
    max_brake_deceleration: 0.5  # m/s^2, TBD
    critical_confidence_threshold: 0.3  # 定位置信度安全阈值, TBD
    max_position_jump: 0.5   # m, 位置跳变阈值, TBD
  path_hysteresis:
    cost_threshold: 0.1      # 10% of current cost
  smoothing:
    method: "clothoid_direct_collocation"
    spatial_step: null              # m, TBD
    solver_timeout: null            # s, TBD
    topology_tube_radius: null      # m, 不得越过已验证拓扑邻域
    minimum_segment_length: null    # m, 数值门禁
    residual_tolerance:
      dynamics: null
      curvature_audit: null
      curvature_rate: null
    g2_join_tolerance:
      position: null                # m
      heading: null                 # rad
      curvature: null               # 1/m
  trajectory_parameterization:
    execution_operating_envelope_version: null # 与统计包络逐项绑定
    profile_version_source: "monotonic"
    sample_period: null             # s, TBD
    terminal_speed: 0.0             # 无后续获批剖面时必须可停车
    stopping_distance_margin: null  # m, TBD
    ground_speed_tracking_error: null # m/s, 租约撤销阈值
    ground_acceleration_tolerance: null # m/s^2, 租约撤销阈值
    payout_speed_tracking_error: null # m/s, 不大于模型epsilon_payout
    tension_tracking_error: null    # N, 租约撤销阈值
  weights:
    length: 1.0
    curvature: 2.0
    touchdown_center_deviation: 10.0
    touchdown_margin: 20.0
    roughness: 1.0
  timeouts:
    planning_max: 0.5        # seconds
    map_validity: 30.0       # seconds
  plan_reuse_validation:
    reuse_max: null                 # T_reuse,max, s, 不长于安全监控周期
    robot_state_max_age: null       # s, 生产模式必填
    cable_state_max_age: null       # s, 生产模式必填
    cable_telemetry_max_age: null   # s, 生产模式必填
    execution_tracking_max_age: null # s, 生产模式必填
    lease_monitor_period: null      # s, 执行器检查过期/撤销的周期
    lease_renewal_margin: null      # s, 到期前触发复检，不直接延长旧租约

task:
  laying_success_ratio_target: 0.8
  communication_max_m: null
  scout_desired_distance_m: null
  scout_policy_version: null
  scout_minimum_map_confidence: null
  scout_sample_interval_m: null
  scout_merge_distance_m: null
  scout_minimum_safe_distance_m: null
  scout_planning_lead_time_s: null
  scout_average_velocity_mps: null
  scout_urgency_hysteresis_distance_m: null
  scout_urgency_hysteresis_time_s: null
  scout_sensor_coverage_radius_m: null
  scout_corridor_half_width_m: null
  scout_continue_distance_m: null
  scout_stop_distance_m: null
  scout_blocking_priority_weight: null
  scout_information_value_weight: null
  scout_forward_progress_weight: null
  scout_arrival_cost_weight: null
  scout_request_timeout_s: null
```

**能力参数门禁**：

- 真实施工模式下，$\alpha_{\max}^{\text{up}}$、$\alpha_{\max}^{\text{down}}$、$h_{\max}^{\text{climb}}$、$h_{\max}^{\text{drop}}$ 任一为空时，参数校验必须失败，系统不得进入自动铺设状态
- $\kappa_{\max}$、$u_{\max}$、当前曲率时间同步容差或 G2 拼接三项容差任一为空时，生产参数校验必须失败
- 对地速度/加减速度/横向加速度、停车距离、放缆速度/加速度及执行跟踪阈值任一为空时，生产参数校验必须失败
- 仿真和算法测试可以加载单独的 `non_production_capability_profile`，但 `ParameterValidationResult.non_production` 必须保持为 `true` 并传播到诊断/报告，不能作为机器人能力或验收结论
- 不得用上坡值自动代替下坡值，也不得用爬阶值自动代替允许落差

**地形梯度风险门禁**：

- `policy_version`、`terrain_analysis_config_version`、`epsilon_local`、`coverage_multiplier`、`calibration_dataset_id` 或 `operating_domain_id` 任一为空时，生产模式不得把方向坡度判为安全
- 必须验证 $0<\epsilon_{g,\text{local}}<1$、$\beta_g>0$ 且覆盖模型与独立数据相符；若使用高斯 $\chi^2$ 系数，必须完成二维梯度残差覆盖率检验
- 当前地图的地形分析配置或梯度风险策略版本变化时，相关路径租约立即失效并触发完整复检

**空间域门禁**：

- $\mathcal W_{\text{robot}}$ 和 $\mathcal W_{\text{cable}}$ 必须分别存在、非空并携带版本；生产配置不得让二者引用同一字段后隐式共享语义
- 搜索仅用 $\mathcal W_{\text{robot}}$ 检查机器人完整扫掠足迹，仅用 $\mathcal W_{\text{cable}}$ 检查触地点轨迹
- 机器人作业区域或缆线参考线/走廊版本变化时，依赖旧组合的路径立即失效并触发重规划

**执行剖面门禁**：

- 搜索执行运行包络必须有版本且与 $\Gamma_H$ 的运行域一致；最终 `ExecutionProfile` 的每个样本均不得超出该包络
- 统计包络记录的 `execution_operating_envelope_version` 必须与搜索及参数化实际使用的版本相同；版本升级立即使旧包络、计划和租约失效
- `TimedPath` 的时间或剖面字段缺失、非有限、非单调，或剖面与几何弧长不一致时，缆线模型和执行器都必须拒绝
- `ExecutionProfileVersioner` 必须对运行包络、插值、停止点、任一执行样本和批准跟踪限制的语义变化签发递增版本；内容变化复用旧版本或版本回退必须失败关闭
- 当前 `TrajectoryParameterizer` 的停车门禁仅使用认证包络中的恒定制动下界、最大停车距离和显式安全裕量；生产配置仍必须提供经坡度/载荷外部标定的停车能力，不得把 Level 1 合成公式提升为平台认证
- 对地速度、出缆速度、张力或加减速度实测偏差超出租约边界时异步撤销租约；不得通过修改控制器倍率继续使用原剖面的缆线预测

**缆线机械参数门禁**：

- $R_{\text{bend,min}}$、$\kappa_{\text{cable}}^{\max}$、$L_{\kappa,\text{eval}}$、$L_{\text{support,eval}}$、$\Delta h_{\text{support}}^{\max}$ 或禁放区图层定义任一为空时，生产参数校验必须失败
- 必须满足 $0<\kappa_{\text{cable}}^{\text{preferred}}<\kappa_{\text{cable}}^{\max}\leq1/R_{\text{bend,min}}$；不得通过提高 $w_{\text{bend}}$ 代替机械硬上限
- 悬空代理阈值只能在其标定运行域内使用；超出缆线类型、张力、海床类型或窗口尺度时按输入无效处理

**统计风险参数门禁**：

- `epsilon_point` 为空或触地点残差分布尚未完成覆盖率标定时，真实施工模式不得宣称走廊概率约束有效
- `CableCorridorRiskPolicy.version` 和 `maximum_marginal_length_m` 必须有效；名义/绝对走廊宽度、统计风险参数或边缘段阈值任一变化都必须生成新策略版本并撤销旧租约
- 当前参考线版本、传感器健康模式、设计运行域、缆线模型版本或执行运行包络版本没有完全匹配的已验证横向包络时，真实施工模式不得启动或继续自动铺设
- 统计包络的生成器版本、缆线模型版本和执行运行包络版本必须显式非空；不得只凭相同 `operating_domain_id` 接受旧包络
- $L_H$、$T_H$、$\rho_{\text{env,disc}}$、$\rho_{\text{cable,sweep}}$ 或包络审计容差任一为空时，生产参数校验必须失败
- 传感器健康模式变化时必须使当前包络和依赖它的规划结果失效并触发重规划，不得继续使用原包络外推
- `epsilon_path` 与 `error_correlation_length` 首版允许为空，但规划结果必须明确标记“未提供整窗口联合风险保证”
- `laying_success_ratio_target: 0.8` 只用于实际铺设后的任务性能统计，禁止自动转换成任何 `epsilon` 参数

**路径复用参数门禁**：

- `reuse_max`、机器人状态/缆线状态/放缆遥测/执行跟踪四类输入最大年龄、`lease_monitor_period` 或 `lease_renewal_margin` 任一为空时，生产模式不得复用路径；参考进度与地图年龄属于 `ValidationInputCaptureLimits` 的运行时捕获门禁，不得假称已由 `ParameterConfig.path_reuse` 提供
- 必须满足 $0<T_{\text{monitor}}<T_{\text{reuse,max}}$ 且 $0<T_{\text{renew}}<T_{\text{reuse,max}}$；租约到期或输入过期后只能重新执行完整复检，不能原地延长时间戳

**搜索标签参数门禁**：

- `ParameterConfig.search` 持有 profile 中的离散化、局部进度关联、扫掠/走廊裕量、五类代价权重、`maximum_active_labels` 和 `equivalent_label_cost_tolerance_m`；具体版本、目标容差、最小转弯半径、原语集合、`maximum_expansions`、`analytic_expansion_interval` 和截止时间由 `AlgorithmRuntimeParameterSnapshot.search` 的 `HybridAStarSearchParameters` 冻结。二者不得被描述为同一个 schema。
- `maximum_active_labels`、`maximum_expansions` 和 `maximum_planning_duration_s` 必须为正；`analytic_expansion_interval` 可为零以显式禁用；$0<\eta\leq0.5$；除路径长度权重必须为正外，其余软权重可为零但不能覆盖硬门禁。
- 当前没有独立 byte-budget 字段。`maximum_active_labels` 必须结合 `fixed_bytes_per_search_label`、`peak_observed_bytes_per_search_label` 和最坏 `CableConstraintMemory` 动态容量满足平台内存预算；达到计数上限前不会按成本静默丢弃不等价历史。
- 达到标签、扩展或规划时间上限使用同一安全 `TIMEOUT` 回退流程，但诊断必须分别为 `HYBRID_ASTAR_ACTIVE_LABEL_BUDGET_EXHAUSTED`、`HYBRID_ASTAR_EXPANSION_BUDGET_EXHAUSTED` 和 `HYBRID_ASTAR_DEADLINE_EXCEEDED`。
- `collision_sweep_margin_m` 当前属于 production profile 的正值门禁，但 `HybridAStarSearchParameters` 不接收独立碰撞裕量；搜索实际使用并审计的是 $\rho_{\text{sweep}}=\eta r/2$。后续装配层不得假称任意 profile 数值已替代该公式，若要改为独立可配置值必须先修改公共运行时契约和扫掠验证。

### 15.3 参数标定策略

**仿真阶段**：估计值只写入显式 `non_production_capability_profile`，保留数据来源和未标定标记，不得成为 production 默认值
**水池实验**：标定关键物理参数（转弯半径、爬坡能力）
**地形梯度覆盖标定**：
- 使用具有独立真值的多坡向、多粗糙度和不同声呐质量数据计算二维梯度残差，不用生成 $\Sigma_g$ 的同一拟合集作验收
- 检查标准化二维残差的覆盖率；仅在高斯假设通过时使用 $\chi^2$ 系数，否则冻结经验或确定性覆盖系数及其运行域
- 分别验证接近上坡、下坡和横坡能力边界的保守拒绝率，并记录策略版本、数据集和失效条件
**缆线模型标定**：
- 直线、不同速度和不同受控张力下估计 $L_{\text{td}}$ 及其有效范围
- 多组恒定曲率左右转试验拟合 $L_{\psi}$，验证稳态关系 $\delta_{\text{ss}} \approx -\kappa L_{\psi}$
- 使用独立试验数据验证触地点横向误差，不能用同一批数据同时拟合和验收
- 在独立数据上检查标准化横向残差的覆盖率；只有覆盖率与高斯模型相符时才能使用 $z_{1-\epsilon/2}$
- 如果残差明显重尾或偏斜，改用独立验证集的经验分位数或其他经验证的保守界，不得继续标注为高斯概率保证
- 记录出缆速度误差、张力和滞后角越界条件，形成 `CableModelValidity` 阈值
- 对每个生产传感器健康模式冻结 $\Gamma_H$，用有界可达性生成 $\bar\sigma_\perp(s_{\text{prog}})$，并记录缆线模型、执行运行包络、参考线、运行域和生成器版本
- 用独立随机、对抗和海试轨迹审计 $\sigma_{\perp,\text{real}}\leq\bar\sigma_\perp+\epsilon_{\text{env}}$；样本测试用于发现包络缺口，不替代确定性上界生成
**执行剖面标定**：
- 在不同坡度、曲率、速度和载荷下标定机器人加速、制动、横向加速度及停车距离边界
- 标定放缆机速度/加速度能力、出缆跟踪误差和张力跟踪误差，并冻结 `ExecutionOperatingEnvelope` 版本
- 用独立闭环试验验证参数化剖面的跟踪覆盖率；据此确定租约撤销阈值，且该阈值不得超出缆线模型和 $\Gamma_H$ 的有效范围
**海试前**：冻结第一版缆线模型参数与有效范围
**海试中**：验证参数；任何调整都生成新的参数、标定数据集和运行域版本，不能原地修改已冻结 production profile

---

## 16. 当前主规划周期与外层编排

当前实现没有一个同时轮询消息、控制前置机并生成路径的单体循环。职责分为两个层次：

1. 外层协调器通过第 13.6 节消息 gate 接收入站流，使用第 11 节 `ScoutCoordinator` 和第 12 节状态机处理补探、等待地图、通信降级/恢复和周期触发。
2. `MainPlanningLoop::run_cycle(request)` 只执行一次从同步输入到“原子授权发布、旧计划新租约复用或撤租停车”的规划周期。它不直接发布速度命令，也不直接调用 `ScoutCoordinator` 或 `PlanningStateMachine`；停车和安全通道由 `MainPlanningLoopStages` 边界回调。

### 16.1 外层补探、消息和恢复编排

```text
ingress.receive(message, now)
  -> 只处理 decision.ready；buffered/duplicate/rejected 不改变业务状态

if communication restored:
  recovery.begin_resynchronization(ingress, source_revision, lease_sequence, now)
  -> 收齐恢复边界后的九个流
  -> capture synchronized inputs
  -> revalidate immutable plan and obtain a strictly newer lease
  -> execution monitor confirms authorization
  -> dispatch communication_restored(recovery_authorization)
  -> 证据不全则保持 COMMUNICATION_DEGRADED 并停车

scan = scout.identify_gaps_result(reference, map, horizon, optional_detour)
if scan invalid:
  dispatch input_invalid
else:
  assessments = scout.assess_gaps(... approved_remaining_timed_path ...)
  targets = scout.generate_scout_targets(...)
  issue_scout_request(non-informational target)

if request is blocking or timed_out:
  dispatch waiting_for_map
  -> 不调用 run_cycle，不签发运动租约
if correlated map remains unresolved:
  keep waiting; do not replan
if correlated map completes:
  invalidate old plan
  dispatch new_map with strictly newer map sequence
  -> 后续触发一个 run_cycle
```

补探完成、消息被接受或通信恢复只产生规划触发/恢复证据；它们自身都不能越过执行租约边界。

### 16.2 单周期不可变输入与规划起点

`run_cycle` 首先记录 `cycle_sequence`、随机种子、触发时间和完整 `AlgorithmRuntimeParameterSnapshot`，并从 runtime profile 获得总周期上限。无效请求、空时钟或无有效周期上限直接按 `INPUT_INVALID` 失败关闭。

输入边界顺序从一次初始捕获开始：

```text
capture_inputs
  -> capture SynchronizedValidationInputs
  -> production 模式核对双空间域 id/version
  -> 冻结 initial source_revision + complete dependencies

committed_start provenance precheck
  -> 已撤销租约改从同步实测状态开始
  -> 非零计划/租约序号、终端状态、前缀和依赖必须有效
  -> dependency mismatch 在地形分析前触发 commitment override

terrain_analysis
  -> 输出必须绑定同一个地图版本、分析配置和运行域

optional commitment_validation
  -> 仅当请求携带未撤销的 CommittedPlanningStart
```

若请求中的承诺租约已经被监控器持久撤销，该承诺被忽略，本周期从同步实测 `RobotState + CableState + ReferenceProgress` 开始。未撤销的承诺起点必须具有有效计划/租约序号、完整依赖、逐点逐样本保持的获批前缀和合法终端机器人状态：

- 依赖变化直接作为 `dependency_version_change` 承诺安全事件处理；
- 先在锁定地形上下文中完整复检获批前缀的机器人和缆线约束；
- 从同步实际缆线状态重新预测终端缆线状态，并使用复检输出的终端参考进度；
- 任一安全事件调用 16.5 节覆盖路径，本周期不进入搜索。

### 16.3 候选生成阶段顺序

承诺安全通过后，候选严格按下列 `PlanningCycleStage` 顺序生成：

```text
search
  -> HybridAStarPlanningResult.state == SUCCESS
  -> robot_path 公共契约有效

smoothing
  -> success: 使用平滑路径
  -> failure: raw_path_trackability_validation
       只有原始搜索路径通过同一可跟踪性契约才可继续
       同时保留 smoothing timeout/infeasible 根因

parameterization
  -> 为选定几何生成完整、版本化 TimedPath

optional commitment_merge
  -> 原样保留获批前缀
  -> 检查 G2 和执行样本连续性

complete_robot_path_validation
  -> 对完整拼接时序轨迹执行独立几何审计和当前地图复检

cable_validation
  -> 从 initial_inputs.cable_state 对完整 TimedPath 独立重预测
  -> 依次通过包络、走廊和机械/地形硬门禁
  -> 终端缆线状态由本次预测和 laying terminal memory 装配

candidate_assembly
  -> 轨迹、触地点、终端缆线状态、走廊/机械结果直接取阶段产物
  -> 绑定 initial 的 locked input dependencies、随机种子和 search solution_cost
  -> execution_profile_version 取本周期完整时序轨迹的剖面版本且不得低于 initial
  -> validate(PlanningResult)
```

平滑失败后使用原始搜索路径不是默认降级：`boundary_state_invalid`、矛盾的 success 输出或原始路径不可跟踪都终止本周期。承诺拼接失败不允许丢弃获批前缀继续。完整机器人路径和完整时序缆线复检都发生在候选装配之前。

### 16.4 最新上下文成对复检、滞回与原子授权

候选装配后只再捕获一次决策上下文：

```text
decision_context_capture
  -> source_revision 不得低于 initial revision
  -> candidate 的 locked input dependencies 必须仍与 latest 完全相同
  -> execution_profile_version 是参数化输出，不参与 locked input 比较

decision_validation_at = candidate_revalidation.started_at

candidate_revalidation(publication_candidate, latest_inputs, same_time)
optional current_plan_revalidation(authorized_current, latest_inputs, same_time)

candidate_decision
  -> MainPlanningLoopStages::decide_candidate 非虚入口
  -> 委托构造时配置的 StabilityManager
  -> switch_candidate / keep_current / stop

lease_acquisition
  -> 所选 lease、plan、remaining TimedPath、latest inputs 和检查时间完整配对

publication
  -> 先持久撤销旧 lease
  -> switch: publish(candidate, remaining_path, new_lease, candidate_cost)
  -> keep: reauthorize_current(remaining_path, new_lease)
```

这里的 locked input dependencies 是除 `execution_profile_version` 外的 `PlanningDependencyVersions` 字段。地图、参考线、作业区、缆线走廊、地形/走廊策略、缆线模型、不确定性包络、执行运行包络、传感器模式和运行域在 initial、candidate 与 latest 之间都不得变化；只有执行剖面身份由本周期参数化阶段产生，可以递增但不能相对 initial 回退。

候选普通复检失败时仍必须先完成当前计划复检，再由同一 `StabilityManager` 决定 keep/stop。候选或当前计划出现 `covariance_envelope_breach` 或 `input_invalid` 时覆盖普通滞回结果并停车。候选代价必须有限且非负；候选无效而当前有效只能保持当前计划，当前也无效则 stop。

publisher 只原子暴露 `ImmutablePlanningResult + exact remaining TimedPath + PlanValidationLease + path_cost`。旧租约在 switch/keep 前撤销；新发布在返回前若跨过总周期截止，也立即撤销新租约、清除授权并请求停车。成功 switch 返回 `PlanningCycleStatus::success/SUCCESS`，成功 keep 返回 `current_plan_reused/PATH_VALID`。

### 16.5 承诺安全覆盖

`CommitmentSafetyEvaluator` 在搜索前处理新障碍、定位变化、地图/依赖变化、边界脱离和完整机器人/缆线复检结果。任何 `STOP` 或 `REPLAN_URGENT` 都交给 `CommitmentSafetySupervisor`：

```text
revoke committed lease
  -> clear matching atomic authorization
  -> request independent safety-stop channel
  -> return commitment_overridden
```

`REPLAN_URGENT` 只设置 `urgent_replan_required`；本周期仍立即结束，下一周期从同步实测状态重规划。再次提交已撤销的承诺租约时，主循环必须忽略它，不能恢复旧承诺起点。

### 16.6 失败分类与旧计划复用

所有普通失败经 `finish_failure` 收敛到一个结构化 `PlanningFailure{cause, stage, reason_code, message}`。搜索 deadline、标签/其他预算耗尽映射 `TIMEOUT`；普通搜索/平滑/参数化/机器人或缆线不可行映射 `NO_SOLUTION`；锁定包络不可用映射 `NO_SOLUTION_UNDER_COVARIANCE_ENVELOPE`；包络越界映射 `COVARIANCE_ENVELOPE_BREACH`；上下文、边界、数值、装配、租约或发布矛盾映射 `INPUT_INVALID`。互斥 timeout 标志同时出现属于输入无效，不能任选一个根因。

只有实现显式标记为可恢复的 timeout 或普通不可行分支可以尝试旧计划复用。复用流程不能使用周期起点的检查结果：

```text
capture latest synchronized inputs again
  -> revision >= initial/decision revision
  -> 同 revision 时 dependencies 必须逐字段相同
  -> revalidate current immutable plan as authorized_current
  -> validation == valid/reuse
  -> remaining_path 与 strictly newer lease 完整有效
  -> revoke old lease
  -> atomically reauthorize current plan
```

候选/当前包络 breach、输入无效、承诺安全覆盖、锁定包络不可用、承诺拼接失败和任何未标记为可恢复的分支禁止复用。复检、租约或原子续签任一步失败时，主循环撤销当前租约、清除 publisher 当前授权，并调用 `request_controlled_stop`。若总周期截止是在普通失败收敛或旧计划复检期间越过，状态始终保留 `cycle_timeout/TIMEOUT` 根因；旧计划仍可基于截止后捕获的最新上下文完成全量复检并取得严格更新租约，但不得把状态改写为普通 `PATH_VALID`。这与新候选在 publication 边界越过截止不同：后者必须立即撤销刚签发的租约并停车。

### 16.7 诊断与确定性边界

每个已执行阶段记录开始时间、真实耗时、source revision、完整依赖和成功标志；`PlanningCycleArtifacts` 保留地形、承诺、搜索、平滑、参数化、拼接、完整机器人/缆线复检、候选、成对复检和滞回决策。随机种子、runtime 参数、初始/决策输入捕获序列、撤租和发布结果进入 `algorithm-experiment/v3`，供确定性重放逐字段比较。

当前 Level 1 闭环证明成功路径、unknown-gap 两周期补探、承诺覆盖、超时/无解复用门禁、消息乱序和依赖变化下的授权边界；它不证明目标平台 2-5 Hz、DAVE/Gazebo 双机链路、长时通信或生产参数已验收。

---

## 17. 复杂度与实时性分析

### 17.1 计算复杂度

#### Hybrid A* 搜索

**时间复杂度**：

假设：
- 栅格地图尺寸：$N_x \times N_y$
- 角度离散化：$N_{\theta}$
- 缆线滞后角离散化：$N_{\delta}$
- 参考进度离散数：$N_s$
- 基础键数量：$N_{\text{base}} = N_x N_y N_{\theta} N_{\delta} N_s$
- 基础键 $k$ 下不可比较的机械历史标签数：$K_M(k)$，总活动标签数 $N_L=\sum_k K_M(k)$
- 每个状态扩展 $b$ 个后继（运动原语数量）

在给定有限规划窗口与全局标签预算下，普通后继的队列操作复杂度为 $O(bN_L\log N_L)$；每逢解析扩展周期还对每个目标求六族前进 Dubins 路径并加入最多一个非零段，因此分支项包含目标数 $|\mathcal G|$。`future_equivalent` 的逐样本比较成本还与 $\max(L_{\text{support,eval}}, 2L_{\kappa,\text{eval}})$ 内保留样本数成正比；签名未命中时会对该基础键的其他活动标签执行 fallback 比较。不能再用单标签 $O(N_{\text{base}}\log N_{\text{base}})$ 估计本实现。

实际情况：
- $\delta$ 受标定范围约束，但同一基础键可能保留多个机械历史标签
- 启发函数只包含机器人到多目标容差集合的路径长度下界；参考线、模型、作业区、地形、统计包络和机械门禁在后继验证中剪枝
- 当前实现记录 $K_M$ 的 P50/P95/P99、总活动标签峰值、等价丢弃/替换、解析尝试/通过和各类剪枝计数；原有 50-200 ms 估计不再作为依据

#### 地形分析

- 鲁棒局部平面与去趋势粗糙度：$O(N_xN_yw^2I_{\text{robust}})$
- 台阶边缘和两侧支撑面拟合：取决于候选边缘数及局部窗口大小
- 当前障碍膨胀逐个障碍扫描全栅格，最坏为 $O(N_{\text{obs}}N_xN_y)$；尚未实现距离变换

其中 $w$ 为窗口边长（栅格数），$I_{\text{robust}}$ 为稳健拟合迭代次数。不能在未指定积分图、滑窗更新或增量拟合实现时将总复杂度写成 $O(N_xN_y)$。

各栅格拟合可并行，但当前实现为串行。局部地图更新在配置、网格和台阶依赖兼容时按物理拟合半径扩张更新区，只重算受影响表面单元；台阶候选或支撑依赖变化会全量重建。原有10-50 ms估计不再作为依据，需在实际窗口、地图分辨率和平台上基准测试。

#### 路径平滑

- 决策变量规模随配点数 $N_{\text{path}}$ 线性增长，约为 $O(N_{\text{path}})$
- 非线性求解耗时取决于稀疏求解器、初值、活动约束和迭代次数，不能由变量数量直接推出统一的 $O(I_{\max}N_{\text{path}})$ 总时间上界
- 独立扫掠复检为 $O(N_{\text{substep}}N_{\text{footprint}})$，其中 $N_{\text{substep}}$ 由第8.3.1节的自适应规则确定
- 必须记录求解器 P50/P95/P99 耗时、超时率、迭代数和最大约束残差；在实际平台基准完成前不提供典型耗时声明

#### 时序参数化

- 对固定采样数 $N_{\text{path}}$ 的前向/后向速度约束传播为 $O(N_{\text{path}})$；若实现使用迭代优化器，必须另行报告迭代上限和实测分位延迟
- 参数化、完整剖面验证和版本生成计入规划关键路径，不能在发布后异步补写

#### 缆线落点预测

- 单个运动原语传播：$O(N_{\text{substep}})$
- 全路径预测：$O(N_{\text{path}} \times N_{\text{substep}})$

搜索阶段使用较粗积分，验证阶段使用较细积分；典型耗时需在实际计算平台重新测量。

#### 剩余路径执行复检

- 设剩余路径含 $N_{\text{remain}}$ 个验证区段，机器人完整足迹扫掠为 $O(N_{\text{remain}}N_{\text{substep}}N_{\text{footprint}})$
- 从当前 `CableState` 重新预测并复检走廊与机械约束为 $O(N_{\text{remain}}N_{\text{substep}})$；不能通过读取计划内缓存的 `cable_path` 降低该项复杂度
- 该成本发生在周期续租、超时回退和滞回决策入口。实现必须共享只读地形派生层，但不能缓存会绕过当前状态或当前上下文的安全结论

### 17.2 空间复杂度

**地图与派生层存储**：

- `MapSnapshot` 按每格一个 `MapCell` 保存 double 精度高程、方差、置信度、时间、障碍/禁放标志和可选障碍法向；`TerrainLayers` 另保存每格 `SurfaceEstimate`、`CableLayingTerrainCell` 以及按候选边缘数量增长的 `StepEstimate`。
- 当前渐近空间为 $O(N_xN_y + N_{\text{step}})$。`std::optional`、`std::vector` 容量、ABI 对齐和字符串使逐格实际字节依赖编译器与构建，旧的“20-30 B/格”和 6.25 MB 示例不是当前实现证据，已经撤销。
- `IncrementalTerrainAnalyzer` 同时持有最近输入地图、规范化配置和 `shared_ptr<const TerrainLayers>` 缓存；缓存命中共享只读派生层，配置/网格/版本或台阶依赖失配时全量失效。当前没有由核心库管理的多版本地图历史池。

**搜索状态空间**：

- 每个标签额外保存连续位姿、`CableState`、`ReferenceProgress`、五元基础键、成本/父节点、入边机器人和缆线预测、最长 $\max(L_{\text{support,eval}}, 2L_{\kappa,\text{eval}})$ 的规范化触地点历史及稳定标签标识
- 总空间为 $O(N_L M_H)$，其中 $M_H$ 是单个 `CableConstraintMemory` 的最大采样数；不能按每个基础键只有一个节点估算
- T43 的当前 MSVC/Level 1 证据测得固定标签对象 1,472 B，计入路径、缆线历史、预测和字符串动态容量的观测峰值 13,001 B，测试进程峰值 RSS 14,348,288 B。该数值只用于当前构建和合成场景的预算校准，不是其他编译器、地图窗口或生产平台的保证
- `maximum_active_labels` 是当前唯一搜索内存硬上限代理；没有独立的最大字节数或按基础键固定 K 标签剪枝。生产 profile 必须用目标平台的重新测量结果反推上限

### 17.3 滚动窗口扩展性

**当前可证明的扩展边界**：

- 搜索请求显式接收局部 `MapSnapshot`、局部目标集和有界 `maximum_active_labels` / `maximum_expansions` / deadline；单周期资源由这些输入与预算约束，不直接按全局任务距离分配搜索标签。
- 参考线自身仍以 `ReferenceLine.points` 全量驻留，其空间为 $O(N_{\text{ref}})$；当前核心没有分块/流式参考线存储，也没有 30 km 内存或吞吐实测。
- 地图窗口长度、分辨率、标签预算和机械历史物理窗口共同决定单周期成本，不能笼统写为 $O(L_{\text{window}}^2)$。长距离可扩展性仍需 T49 的 1-2 km、30-60 分钟外部运行证据。
- 生产总内存预算默认由 `AlgorithmPerformanceBudget.maximum_total_memory_bytes = 100 MiB` 表达。T43 仅在当前 MSVC/Level 1 进程观测到峰值 14,348,288 B；这不是目标平台或长期无增长保证。

### 17.4 实时性指标

**目标频率**：2-5 Hz（初步目标，TBD）

**延迟预算**：

| 阶段 | 目标耗时 | 最坏情况 |
|------|----------|----------|
| 地图锁定与输入捕获 | TBD（目标平台实测） | 受输入同步和周期截止约束 |
| 鲁棒/增量地形分析 | TBD（目标平台实测） | 全量失效必须计入 |
| 增广状态 Hybrid A* 搜索 | TBD（目标平台实测） | 标签、扩展和 deadline 三类硬预算 |
| 路径平滑与独立机器人复检 | TBD（目标平台实测） | 平滑 deadline 与完整扫掠均计入 |
| 时序参数化与完整缆线复检 | TBD（目标平台实测） | 时序 deadline、重预测与包络审计均计入 |
| 最新上下文成对计划复检 | TBD（目标平台实测） | 必须小于租约续签预算 |
| 滞回、租约与原子发布 | TBD（目标平台实测） | 发布边界不得越过周期截止 |
| **总周期** | **5 Hz 目标对应不大于 200 ms** | **安全硬截止 500 ms；尚无生产平台达标声明** |

加入鲁棒地形拟合和缆线增广状态后，旧版各阶段经验耗时作废。目标频率保留，但在基准测试完成前不宣称已经满足。

**超时处理**：

如果单次规划超过 500 ms：
- 返回 TIMEOUT 状态
- 搜索截止时间、全局活动标签预算、搜索扩展预算、平滑截止时间和时序参数化截止时间分别记录 `HYBRID_ASTAR_DEADLINE_EXCEEDED`、`HYBRID_ASTAR_ACTIVE_LABEL_BUDGET_EXHAUSTED`、`HYBRID_ASTAR_EXPANSION_BUDGET_EXHAUSTED`、`SMOOTHING_DEADLINE_EXCEEDED` 和 `PARAMETERIZATION_DEADLINE_EXCEEDED` 唯一根因，不互相折叠
- 使用规划结束时重新捕获的同步状态和当前完整上下文调用 `validateRemainingPlan`
- 只有取得新租约时才继续使用旧计划；否则撤销旧租约并停车
- 记录超时事件，用于参数调优

**优化状态**：

| 机制 | 当前状态 |
|---|---|
| 影响区增量地形更新与只读缓存共享 | 已实现；台阶/配置/网格依赖变化显式全量失效 |
| 搜索活动标签、扩展和单调截止预算 | 已实现；耗尽统一安全返回 `TIMEOUT` |
| 地形层并行计算 | 未实现，当前串行 |
| 远近自适应地图/搜索分辨率 | 未实现 |
| 找到任意可接受路径即提前终止 | 未作为独立优化实现；当前终止由目标、资源和 deadline 规则决定 |
| 距离变换、GPU 加速 | 未来目标，当前障碍膨胀仍为逐障碍全栅格扫描 |

---

## 18. 测试方案

### 18.1 测试层次

```
┌─────────────────────────────────────────────────────────────┐
│  Level 4: Sea Trial / Field Test                            │
│  - 真实海床环境                                               │
│  - 实际机器人与缆线                                           │
│  - 验收指标验证                                               │
└─────────────────────────────────────────────────────────────┘
                          ▲
┌─────────────────────────────────────────────────────────────┐
│  Level 3: Water Tank Experiment                             │
│  - 可控环境                                                   │
│  - 标定物理参数                                               │
│  - 部分验收测试                                               │
└─────────────────────────────────────────────────────────────┘
                          ▲
┌─────────────────────────────────────────────────────────────┐
│  Level 2: DAVE/Gazebo Integration Test                      │
│  - 双机协同仿真                                               │
│  - 在线数据流验证                                             │
│  - 长时运行测试                                               │
└─────────────────────────────────────────────────────────────┘
                          ▲
┌─────────────────────────────────────────────────────────────┐
│  Level 1: Unit & Deterministic Scenario Test                │
│  - 合成高程图                                                 │
│  - 确定性算法测试                                             │
│  - 边界与异常测试                                             │
└─────────────────────────────────────────────────────────────┘
```

下表冻结的是 v3.0 发布基线中实际存在的证据，不复制 ticket 的动态状态；当前任务进度只以 `project.md/PLAN.md` 为准：

| 验证层/活动 | v3.0 发布基线中的证据 | 仍需的证据入口 |
|---|---|---|
| Level 1 单元与确定性场景 | 37 个 `test_*.cpp`、38 个 CTest；四个 Level 1 矩阵/闭环入口及结构化报告 | 后续变更继续由同一统一入口回归 |
| Level 2 DAVE/Gazebo 与长时通信 | 无外部仿真器、机器人接口或 30-60 分钟报告 | T48-T49 定义所需外部证据 |
| Level 3 水池 | 无独立标定、机器人/缆线/真值系统或试验报告 | T50-T53 定义所需外部证据 |
| Level 4 海试/验收 | 无真实海床、设备或甲方验收记录 | T54 定义所需外部证据 |
| 三算法基线对比 | 无同场景三算法结果 | T55 定义固定场景、种子、参数、结果与统计脚本 |
| 核心机制消融 | 无消融结果 | T56 定义待检验机制；不得把设计中的计划表当成结果 |

Level 1 的“通过”只证明当前编译器和合成输入上的公共行为、失败关闭和确定性；不外推为频率、长距离、模型覆盖率、真实铺设精度或生产能力。

### 18.2 Level 1：单元与确定性场景测试

#### 18.2.1 地形分析测试

**测试场景**：

1. **平坦地形**：验证不产生虚假陡坡
2. **多方向理想斜面**：验证梯度大小、方向和协方差
3. **同粗糙度不同坡度**：验证去趋势粗糙度不随整体倾斜改变
4. **台阶**：验证完整高度、低到高法向和边缘范围
5. **斜向评价不变性**：改变机器人测试航向不应改变 `TerrainAnalyzer` 输出的台阶高度
6. **噪声与孤立异常点**：验证鲁棒平面拟合
7. **支撑不足**：验证返回 `INSUFFICIENT_SUPPORT`，而不是零坡度
8. **协方差无效**：非有限、非半正定或与拟合结果不匹配的 $\Sigma_g$ 返回 `INVALID_COVARIANCE`

**测试方法**：

```python
def test_surface_fit_separates_slope_and_roughness():
    height_map = create_plane(a=0.3, b=-0.2, residual_noise=0.01)

    layers = terrain_analyzer.analyze(height_map, analysis_config)
    estimate = layers.surface.at(center)

    assert np.allclose(estimate.gradient, [0.3, -0.2], atol=0.01)
    assert estimate.detrended_roughness == pytest.approx(0.01, abs=0.005)
    assert estimate.status == VALID
```

#### 18.2.2 通行性评价测试

**测试场景**：

1. **简单障碍物**：验证膨胀和碰撞检查
2. **复杂足迹**：验证完整足迹碰撞
3. **未知区域**：验证保守禁行
4. **机器人碰撞裕量**：验证机器人相对障碍物协方差传播，且不包含缆线协方差
5. **坡度投影**：验证先投影梯度再取 `atan`，保留上下坡符号
6. **20 cm 台阶斜交**：正交和45度斜交均按完整20 cm检查
7. **向上/向下穿越**：分别使用独立的爬阶和落差参数
8. **沿边缘骑跨**：验证左右履带支撑高差和滚转角
9. **单侧支撑缺失**：验证均值不会掩盖局部悬空
10. **方向阈值过渡区**：同时执行完整高度与骑跨检查
11. **均值安全但上界越限**：均值纵坡/横坡低于能力上限，但保守投影界越限时必须拒绝
12. **各向异性梯度协方差**：固定 $\hat{\mathbf g}$ 并旋转机器人航向，验证 $\mathbf t^T\Sigma_g\mathbf t$ 与 $\mathbf l^T\Sigma_g\mathbf l$ 分别进入对应边界
13. **上下坡非对称边界**：同一纵向均值与方差分别检查 $q_{\text{long}}^{\max}$ 和 $q_{\text{long}}^{\min}$，不得用绝对值抹平上下坡能力差异
14. **风险策略未标定**：覆盖模型、数据集或策略版本缺失时返回输入无效，不允许退回均值坡度
15. **局部风险语义**：结果明确记录 $\epsilon_{g,\text{local}}$ 且 `terrain_gradient_path_joint_risk_implemented=false`，不得宣称已经提供整条路径联合地形风险保证
16. **分析配置错配**：策略绑定的地形分析配置版本或运行域与当前 `TerrainLayers` 不同，整段评价返回输入无效
17. **机器人粗糙度硬门禁**：中心表面安全但完整足迹边缘样本的去趋势粗糙度超过能力阈值时拒绝整段；等值阈值通过，非有限或负值失败关闭并记录最坏值
18. **碰撞扫掠完整地图版本门禁**：完全匹配时执行扫掠；`map_id`、`sequence_number`、`timestamp`、`coordinate_frame` 或地形分析配置版本任一单独错配时，在足迹评价前失败关闭

**不变量**：

- 障碍膨胀后，原始障碍仍为不可通行
- 完整足迹任一点在障碍内 → 整体不可通行
- 未知区域默认不可通行
- 台阶接近角不得缩放真实台阶高度
- 扫掠足迹未与台阶边缘相交时，不得仅凭附近台阶误判为正在跨越
- 任一履带支撑覆盖率不足时，不得以左右平均高程判为可通行
- 所有扫掠样本均使用同一锁定策略版本；任一点的保守方向坡度界越限即整段不可通行
- 任一有效足迹表面 `detrended_roughness_rms_m > robot.maximum_roughness_m` 即整段不可通行且等值边界通过；无效、非有限或负粗糙度不得被当作零或软代价，必须失败关闭为 `TERRAIN_INVALID`

**当前 Level 1 证据绑定**：`test_level1_terrain_traversability_matrix.cpp` 将上述 `18.2.1-1..8`、`18.2.2-1..18` 和 8 条不变量共 34 项，逐项绑定到 `test_terrain_analyzer.cpp`、`test_step_geometry.cpp`、`test_traversability_evaluator.cpp`、`test_directional_slope.cpp` 与 `test_step_traversability.cpp` 中已标记且实际执行的公共接口测试。`18.2.2-17` 与 `18.2.2-invariant-8` 绑定 `roughness_hard_gate_covers_edges_and_fails_closed`，`18.2.2-18` 绑定 `collision_sweep_requires_the_complete_map_version`；矩阵还要求五个测试可执行文件成功，不把仅存在的测试名称算作证据。

矩阵之外，`test_terrain_analyzer.cpp` 还核验完整配置/地图派生版本匹配、同版本负载篡改拒绝、更新区外变化拒绝、版本回退拒绝、物理窗口影响区扩张、只读快照共享、局部重算与全量重算逐字段等价，以及台阶依赖变化触发全量失效。`test_traversability_evaluator.cpp`、`test_directional_slope.cpp` 和 `test_step_traversability.cpp` 还覆盖无效输入有限输出、确定性重复、地图边界、非法障碍法向、未解释不连续带和多台阶事件排序。这些是第5.1-5.8节的补充 as-built 证据，不新增 Level 2-4 或外部标定声明。

T63 已由 `collision_sweep_requires_the_complete_map_version` 闭合：测试分别构造仅时间戳、map id、sequence、coordinate frame 和分析配置版本错配，并验证结果为有限、不可误判为无碰撞且未执行足迹扫掠；完全匹配路径则实际执行扫掠。

#### 18.2.3 Hybrid A* 搜索测试

**测试场景**：

| 场景 | 输入 | 期望输出 |
|------|------|----------|
| 平坦直线 | 无障碍，直线触地点参考路线 | 预测触地点贴近参考线，机器人中心保持模型要求的偏置 |
| 单侧障碍 | 触地点参考线附近存在障碍，一侧可绕行 | 机器人安全绕行，预测触地点在可行后回归参考线 |
| 双侧可选 | 两侧均可绕行，代价不同 | 选择代价低的一侧 |
| 狭窄通道 | 宽度接近机器人尺寸 | 通过或无解 |
| 无解场景 | 包络或其他硬约束下完全封闭 | 仅包络/走廊失败返回 `NO_SOLUTION_UNDER_COVARIANCE_ENVELOPE`；混合或其他失败返回 `NO_SOLUTION` |
| 原语中段障碍 | 起点、终点和中点无碰撞，中间采样姿态碰撞 | 原语无效 |
| 转弯角点扫掠 | 车体中心无碰撞但外侧角点扫过障碍 | 原语无效 |
| 走廊采样间隙 | 相邻触地点样本合格但区间上界可能越界 | 通过独立缆线离散裕度保守拒绝 |
| 分辨率变化 | 改变原语长度或内部采样密度 | 软成本和硬约束结论在规定容差内一致 |
| 等价记忆低成本重开 | 较高成本标签先到达同一基础键，后到标签的机械记忆经证明未来等价且代价更低 | 替换等价标签并重新扩展，旧队列项按 `label_id` 跳过 |
| 不等价机械历史 | 两条路径到达相同 $(i_x,i_y,i_\theta,i_\delta,i_s)$，但前序触地点使跨边界曲率或后向悬空窗口内容不同 | 两个标签同时保留，低当前代价不得删除另一个 |
| 记忆签名碰撞 | 两个不同规范化历史被构造为相同 `canonical_signature` | `future_equivalent` 逐样本比较后判为不等价，不合并标签 |
| 标签资源耗尽 | 不可比较历史标签达到全局活动标签预算 | 返回带标签统计的 `TIMEOUT`，不静默保留前 K 个 |
| 参考线交叉 | 相同位姿和滞后角对应交叉前后两个任务阶段 | 不同 i_s 节点不合并 |
| 短原语邻近分支 | 0.5 m 原语附近存在相隔较远的参考线分支 | 进度增量不超过 alpha_s L_p + epsilon_s |
| 机器人/缆线域分离 | 机器人中心位于缆线走廊外但足迹位于机器人作业区，预测触地点在缆线走廊内 | 路径可行，不因机器人偏离缆线参考线被拒绝 |
| 代理量反例 | 机器人中心位于缆线走廊内但预测触地点越界 | 原语按缆线走廊硬约束拒绝 |
| 接入目标反解 | 非零放缆点偏置和 $L_{\text{td}}$ | 机器人目标与触地点目标不同，正向预测后触地点命中参考目标 |
| 代价单计权 | 固定同一触地点轨迹并改变机器人到参考线距离 | 触地点走廊代价不变，不出现第二份参考偏差成本 |

上表 19 个场景及下述 4 个返回路径硬不变量由 `level1_search_cable_matrix` 按 `18.2.3-*` 标识自动核对到公共测试源码并执行。当前 `hybrid_astar_planner` 还包含 46 项行为检查，额外覆盖：六族前进 Dubins 对目标容差域的可接受启发；关闭/启用解析扩展、偏离格点目标、精确曲线和多段连接；解析中段障碍拒绝；初始目标不能绕过走廊/机械门禁；依赖版本错配和非有限地形软成本在扩展前失败关闭；扩展预算、活动标签预算和单调时钟截止时间三种独立 `TIMEOUT`；普通无解与纯包络无解分类；以及固定输入下路径、五分量成本、队列统计和确定性指纹逐字段复现。

`merge_goal_generator` 的 7 项检查另覆盖反解/正向闭环、多个并线距离和滞后角边界、完整凸/凹作业区终端足迹、终端地形、生成软成本稳定排序/截断及版本/运行域审计。测试显式断言 `forward_model_mismatch`、`lag_angle_out_of_range`、`robot_footprint_outside_area`、`terminal_terrain_not_traversable` 和 `goal_limit_reached` 五类枚举原因，并验证参考版本错配与并线进度越界失败关闭；尚未逐枚举断言 `invalid_input`、`reference_version_mismatch`、`merge_progress_outside_reference` 和 `terrain_evaluation_invalid` 全部原因，因此不得声称九类拒绝均有独立枚举覆盖。这里的 Level 1 证据证明的是合成输入上的公共接口行为，不证明真实平台并线精度、规划频率或整路径联合风险。

**不变量检查**：

```python
def test_path_constraints(
    path,
    initial_laying_memory,
    terrain_layers,
    terrain_gradient_risk_policy,
    params
):
    laying_memory = initial_laying_memory
    for segment in path.segments:
        # 按8.3.1节自适应采样；检查膨胀 footprint 覆盖的全部栅格
        result = traversability_evaluator.evaluate(
            segment, terrain_layers, terrain_gradient_risk_policy
        )
        assert result.traversable, result.failure_reason

        # 缆线走廊使用独立离散裕度，不得复用机器人扫掠裕度
        corridor = cable_corridor_evaluator.evaluate_search_segment(segment)
        assert corridor.validity == VALID
        assert corridor.violation_count == 0

        laying = cable_laying_evaluator.evaluate_segment(
            laying_memory,
            segment.touchdown_path,
            segment.cable_state_profile,
            terrain_layers,
            params.cable_laying_limits
        )
        assert laying.valid
        assert laying.hard_feasible, laying.failure_reasons
        laying_memory = laying.terminal_memory
        
        # 曲率约束
        kappa = compute_curvature(segment)
        assert abs(kappa) <= params.kappa_max, "Curvature exceeds limit"
```

#### 18.2.4 缆线落点测试

**测试场景**：

1. **稳态直行**：$\delta$ 收敛到零，触地点距离收敛到 $L_{\text{td}}$
2. **恒定曲率转弯**：$\delta$ 收敛到由 $\kappa$ 和 $L_{\psi}$ 决定的稳态值
3. **左右转对称性**：相反曲率产生符号相反、幅值一致的滞后角
4. **滚动窗口连续性**：以实际执行状态Tracker在窗口起点的快照初始化时，触地点轨迹无跳变
5. **积分一致性**：搜索步长逐渐减小时，预测结果收敛到验证阶段结果
6. **模型有效范围**：出缆速度误差、张力或滞后角越界时返回对应状态
7. **不同放缆点偏置**：验证机器人路径到触地点路径的映射
8. **各向异性协方差投影**：旋转参考线法向时，$\sigma_\perp^2=\mathbf n^T\Sigma_c\mathbf n$
9. **确定参考线**：首版始终使用 $\Sigma_{\text{ref}}=0$，不虚构参考线误差
10. **保守替代式**：满足 $d_{\text{upper}}\leq w$ 时，精确高斯区间概率不低于目标；两者不要求数值相等
11. **质量分数隔离**：改变地图 `confidence` 不得直接乘缩 $\sigma_\perp$；低质量应触发门禁
12. **风险参数门禁**：`epsilon_point` 为空或分布未标定时返回风险评价无效
13. **指标隔离**：$R_{\text{lay}}=0.8$ 不会生成 $\epsilon_{\text{point}}=0.2$
14. **协方差路径记忆**：构造终端 $(x,y,\theta,\delta,s_{\text{prog}})$ 相同但曲率历史不同的路径，验证实际 $\Sigma_c$ 可以不同
15. **包络覆盖**：对 $\Gamma_H$ 的边界、分箱余项和所有批准传感器健康模式验证 $\sigma_{\perp,\text{real}}\leq\bar\sigma_\perp+\epsilon_{\text{env}}$
16. **上包络查询**：进度分段之间的查询结果不低于相邻端点上界，并包含 $\rho_{\text{env,disc}}$
17. **包络失效**：实际横向标准差超过包络时返回 `COVARIANCE_ENVELOPE_BREACH`，使包络和依赖路径失效
18. **运行域隔离**：参考线版本、传感器模式或运行域不匹配时拒绝规划，不外推包络
19. **搜索协方差隔离**：搜索节点不携带或消费路径相关协方差，同一 key 使用相同 $\bar\sigma_\perp(s_{\text{prog}})$
20. **左右曲率对称硬门禁**：$+\kappa$ 与 $-\kappa$ 幅值相同时得到相同判定，任一满足 $|\kappa|>\kappa_{\text{cable}}^{\max}$ 都被拒绝
21. **权重不可覆盖硬约束**：将 $w_{\text{bend}}$、$w_{\text{terrain\_risk}}$ 设为零仍不能放行最大曲率或悬空代理超限路径
22. **禁放区与未知数据**：触地点扫掠接触禁放区、未知区或低置信度区时返回对应硬失败原因
23. **悬空代理采样不变性**：改变触地点采样间隔时，固定 $L_{\text{support,eval}}$ 窗口的最大地形起伏及硬判定在规定容差内一致
24. **完整路径复检**：承诺段与新规划段各自可行但拼接处违反缆线最大曲率时，最终验证必须拒绝完整路径
25. **机械历史不可合并**：构造相同基础键但不同末端两触地点或后向地形窗口的标签，其中只有高当前代价标签的后继满足机械约束；搜索必须保留并找到该安全分支
26. **实际历史初始化**：候选第一原语自身合法但与已铺缆线边界组合后曲率超限；从当前 `laying_memory` 增量评价时必须立即拒绝
27. **地图更新重评历史窗口**：保持实际触地点历史不变但更新其覆盖区域的地形图层；复检必须按新地图重新查询高程，不得从 `laying_memory` 读取缓存可行性
28. **拒绝无时序路径**：最终 `CableModel::predict` 收到纯几何路径、空剖面或非单调时间时返回输入无效
29. **计划减速重预测**：相同几何生成减速剖面时分配新版本，并按新 `TimedPath` 重新执行缆线预测，不能复用原预测
30. **搜索包络包含性**：每个获批最终剖面的速度、加速度、出缆误差和张力范围均落在搜索及 $\Gamma_H$ 绑定的认证执行运行包络内
31. **模型版本隔离**：保持参考线、传感器模式和运行域不变，仅改变 `cable_model_version`；Manager 必须拒绝旧包络，不能因 `operating_domain_id` 相同而返回
32. **执行包络版本隔离**：仅升级 `execution_operating_envelope_version`；旧统计包络、计划和租约全部失效，重新生成匹配包络前禁止自动铺设
33. **生成依赖可审计**：包络输出完整记录生成器、缆线模型、执行运行包络、参考线、传感器模式和运行域版本，任一字段缺失时生产校验失败
34. **MARGINAL 累计门禁**：分别构造累计长度小于、等于和大于 `maximum_marginal_length` 的路径；前两者可进入后续验证，大于阈值者无条件拒绝，不存在非严格模式绕过；改变采样间隔后累计弧长和边界判定在规定容差内不变

**关键验证**：

- 每个点均输出 $\mu_\perp$、$\sigma_\perp$、$d_{\text{upper}}$ 和分级依据
- 相同机器人位姿但不同初始 $\delta$ 产生不同触地点，且搜索不会错误合并状态
- 相同位姿和 $\delta$ 但不同 $s_{\text{prog}}$ 的节点不会错误合并
- 搜索和最终验证使用相同参数及相同初始状态
- 搜索和最终验证使用相同初始 `laying_memory` 与 `CableLayingLimits`；最终结果包含机械硬约束判定及失败位置
- 正常候选验证不应因 $\sigma_{\perp,\text{real}}>\bar\sigma_\perp$ 失败；发生时按包络系统故障处理
- 改变路径采样间隔不会被错误解释为已经获得不同的整窗口联合风险保证

**当前 Level 1 证据绑定（D04）**：缆线链路由八个公共接口测试可执行文件核验：`test_cable_state_tracker.cpp`（10 项，实际历史初始化、固定支撑窗口、触地点观测校正、观测中断/状态丢失）、`test_reference_progress_tracker.cpp`（11 项，局部有界关联、交叉路线阶段保持、歧义/回退请求、实际执行单调推进、版本/时间门禁和确定性）、`test_cable_model.cpp`（14 项，均值传播、左右转对称、搜索/验证积分一致性、逐点执行剖面、路径记忆协方差、版本与模型有效性门禁）、`test_cable_laying_evaluator.cpp`（19 项，固定物理窗口曲率、supercover 禁放/未知区、悬空代理、增量/完整路径边界、地图重查询、机械记忆等价与硬门禁）、`test_cable_corridor_evaluator.cpp`（6 项，法向协方差投影、保守分位数、区间上界证书、`PASS/MARGINAL/VIOLATION` 和边缘弧长门禁）、`test_cable_uncertainty_envelope_builder.cpp`（9 项，可达集合传播、分箱余项、传感器/运行域/资源门禁、依赖审计）、`test_cable_uncertainty_envelope_manager.cpp`（6 项，完整查找元组、上包络查询、上下文原子失效、过期、协方差越界停车语义）和 `test_timed_cable_candidate_verifier.cpp`（6 项，完整时序高精度复检、实际历史拼接、减速后新版本重预测、包络越界撤销依赖）。共 81 项行为检查；相关用例按各模块职责核验 seed、输入版本、SI 单位、单调时间、运行域和 pointwise-only 风险语义，不要求每个测试程序重复打印全部审计字段。`powershell -NoProfile -ExecutionPolicy Bypass -File tools/verify.ps1` 是该组证据的统一入口；Level 2-4、外部标定和 `epsilon_path`/误差相关长度仍未实现，不能由这些 Level 1 结果替代。

#### 18.2.5 平滑与 G2 拼接测试

**测试场景**：

1. **非零起始曲率**：承诺段终端曲率非零，平滑结果起点的位置、航向和曲率均在容差内一致
2. **左右转对称性**：正负曲率分别满足 $|\kappa|\leq\kappa_{\max}$，不能遗漏负曲率超限
3. **曲率变化率边界**：构造最大曲率合法但 $|d\kappa/ds|>u_{\max}$ 的路径，必须拒绝
4. **拼接错位**：位置相同但航向或曲率不连续时，`mergePathsG2` 必须失败
5. **几何独立审计**：篡改路径点中的曲率元数据，使其与三点几何曲率不一致，必须返回残差超限
6. **原始路径回退**：平滑器超时且原始 Hybrid A* 路径存在曲率跳变时，不得直接发布原始路径
7. **完整路径复检**：承诺段和新段分别合法但拼接后的扫掠检查失败时，拒绝完整路径
8. **求解器假收敛**：求解器报告收敛但动力学或边界残差超限时，不得返回 `success`
9. **重采样一致性**：改变输出采样间隔后，最大曲率、最大曲率变化率和 G2 判定在规定容差内一致
10. **边界状态缺失**：当前实际曲率缺失或时间不同步时返回 `boundary_state_invalid`，不得默认零曲率
11. **无机器人参考线代理**：在拓扑信赖域内平移机器人原始路径但保持同一验证触地点结果，平滑目标中不存在机器人到缆线参考线的独立代价项
12. **独立起点来源门禁**：`auditPathGeometry` / `PathCandidateVerifier` 只接受同步实际状态或承诺段终端作为起点曲率来源；`planned_goal`、缺失曲率、零序列、负时间和超同步容差均在扫掠前失败，而 `planned_goal` 终点仍合法

**关键不变量**：

- 已发布路径的每个点同时携带 $s,x,y,\theta,\kappa$，且航向、曲率来自同一参数曲线
- 所有路径发布前均检查位置、航向、曲率三项拼接残差
- 平滑失败不会自动把未通过相同验证的原始路径标记为安全

**当前 Level 1 证据绑定（D06/T64）**：`test_path_smoother.cpp` 的 18 项公共行为检查覆盖平滑入口的边界来源/时间同步、非零正负曲率、clothoid 同源状态、拓扑管、积分误差界、目标权重、假收敛、重采样及超时无路径；`test_path_candidate_verifier.cpp` 的 6 项公共行为检查覆盖曲率元数据篡改、位置/航向/曲率 G2 拼接、完整承诺前缀在当前地图上的扫掠、完整路径通行性复检，以及独立复检的起点来源白名单、缺失/无版本/负时间/超同步容差失败关闭、合法 `planned_goal` 终点、零扫掠副作用和确定性重放。`test_level1_smoothing_timing_stability_matrix.cpp` 按设计编号确认本节 12 个场景和 3 个不变量均有已标记且实际调用的公共接口测试，不把仅存在的函数名算作证据。

#### 18.2.6 时序参数化测试

**测试场景**：

1. **几何不变性**：参数化前后 $(s,x,y,\theta,\kappa)$ 逐点一致，G2 与曲率审计结果不变
2. **动力学边界**：速度、加速、制动和 $v^2|\kappa|$ 均不越过认证运行包络；空间曲率变化率由前置几何测试覆盖，不把它重复表述为当前参数化器的时域能力
3. **放缆边界**：出缆速度/加速度、与对地速度的允许误差及张力设定值均在有效范围
4. **停车约束**：任一要求停止的终端都满足可用距离内的制动约束；不可满足时参数化失败
5. **剖面完整性**：弧长单调、时间严格递增、字段有限且插值覆盖整条几何路径
6. **版本变化**：`ExecutionProfileVersioner` 对完全相同内容保留版本；运行包络、插值、停止点、任一执行样本或批准跟踪限制变化都产生递增的 `execution_profile_version`，旧版本复用和版本回退均被拒绝
7. **承诺段不变性**：新尾段参数化与时序拼接不得改变承诺段的几何或执行样本，连接处速度、加速度、出缆速度和张力设定满足容差

**当前 Level 1 证据绑定（D06）**：`test_trajectory_parameterizer.cpp` 的 6 项公共行为检查覆盖几何逐字段不变、完整版本化剖面、速度/加减速/横向加速度、出缆/张力、停车距离、认证包络和统一单调截止时间；`test_data_contract.cpp` 覆盖严格递增时间、几何/样本弧长配对、批准限制和剖面语义版本；`test_stability_manager.cpp` 覆盖承诺前缀不变及几何/执行连接连续性。Level 1 矩阵将本节 7/7 项逐项绑定并执行相关测试。当前证据只证明合成输入下的 Level 1 契约，不证明待标定的真实平台停车、制动、出缆或张力能力。

#### 18.2.7 稳定性测试

**测试场景**：

1. **轻微地图变化**：主循环在同一最新上下文成对复检后，新旧路径代价接近 → 保持当前计划并原子安装其新租约
2. **显著改善**：主循环在同一最新上下文成对复检后，新路径代价明显更低 → 撤销旧租约并原子切换候选
3. **振荡抑制**：连续轻微变化 → 路径保持稳定
4. **超时与缆线状态变化**：旧路径在当前地图上仍无碰撞，但当前滞后角、张力或放缆状态已改变并导致重新预测失败 → `TIMEOUT` 不得复用旧路径，必须停车
5. **传感器模式变化**：租约签发后切换传感器健康模式 → 异步撤销租约，执行器立即拒绝后续路径命令
6. **滞回安全优先**：旧路径成本接近新路径但当前完整复检失败 → 不调用滞回来保留旧路径；候选验证和有限非负代价门禁均通过时直接切换
7. **禁止缓存缆线路径**：修改当前 `CableState` 并在旧计划中放入可行的缓存 `cable_path` → 复检必须忽略缓存，从当前状态重新预测并按新结果判定
8. **计划与租约配对**：租约的 `plan_sequence_number` 与发布计划不一致，或较旧 `lease_sequence` 覆盖新租约 → 执行器拒绝
9. **规划期间租约到期**：规划开始时旧租约有效、结束时已到期 → 超时回退或滞回前必须再次调用 `validateRemainingPlan`，不能沿用循环开始时的结果
10. **上下文逐项失效**：分别改变地图版本、参考线版本、机器人作业区版本、缆线施工走廊版本、地形梯度风险策略版本、走廊风险策略版本、缆线模型版本、不确定性包络及生成器版本、执行运行包络/剖面版本、传感器模式或运行域 → 每一种变化都撤销旧租约，且只有在新上下文完整复检通过后才能重新执行
11. **运行偏差撤租**：对地速度、出缆速度、张力或加减速度任一超出租约阈值 → 异步撤销租约并受控停车/重规划
12. **减速禁止复用旧租约**：控制端请求速度倍率变化时，原剖面租约不能继续授权；只有新剖面完成缆线复检后才能执行
13. **剖面配对**：计划、剩余时序轨迹和租约的剖面版本任一不一致 → 执行器拒绝
14. **旧计划续用**：从当前同步状态裁剪原剩余剖面并按该剖面从当前 `CableState` 重预测；无法连续衔接时复用失败，不得暗改剖面或复用旧缆线路径
15. **超时回退**：已过期或剖面版本不匹配的租约不能因规划超时而继续使用
16. **一致快照门禁**：在捕获过程中改变机器人状态、缆线遥测、执行跟踪状态或任一上下文版本 → 整次捕获无效且不得签发租约，禁止把捕获前后的字段拼成上下文
17. **跟踪状态必填**：缺少、过期或与执行剖面版本不匹配的 `ExecutionTrackingState` → 同步捕获失败，或复检返回 `input_invalid` / `state_mismatch`，主循环不得沿用旧租约
18. **紧急重规划先撤租**：`CommitmentSafetyEvaluator` 返回 `REPLAN_URGENT` → `CommitmentSafetySupervisor` 在启动新规划前撤销旧租约并触发受控停车通道，旧承诺段不再获得执行授权；下一周期重复提交该撤销租约时必须从同步实测状态规划
19. **候选代价门禁**：候选代价为负、`NaN` 或无穷 → 有本次复检有效的当前计划时保持并续签其新租约，否则停车；原子发布边界再次拒绝同类非法代价
20. **成对上下文一致性**：候选与当前计划的复检租约在地图、参考线、双空间域、风险策略、模型、包络、运行包络、传感器模式、运行域或同步输入时间任一不一致 → `stop`，不得进入质量滞回或发布
21. **候选剖面版本与弧长原点前进**：旧计划执行进度非零、候选从当前实测位姿以弧长零重新起算且携带新单调剖面版本时，候选使用自身首点弧长完成全量复检并取得绑定新版本的租约；同一计划若冒充当前授权计划并使用旧计划跟踪弧长复检，则因状态/跟踪版本不匹配被拒绝
22. **候选失败仍成对决策**：候选普通复检失败且当前计划在同一最新上下文有效 → 两份复检工件均保留并由 `StabilityManager` 返回 keep；无有效当前计划则 stop，不得另行捕获第三个上下文绕过成对决策

**测试方法**：

```python
def test_path_hysteresis():
    current_plan = plan(map_v1)
    candidate_plan = plan(add_small_noise(map_v1))
    inputs = capture_synchronized_validation_inputs()

    old_v = evaluator.validateRemainingPlan(
        current_plan, inputs, validation_context, now
    )
    new_v = evaluator.validatePublicationCandidate(
        candidate_plan, inputs, validation_context, now
    )

    decision = stability_manager.decide_path_switch(
        old_v, new_v, cost(old_v.remaining_path.geometry),
        cost(new_v.remaining_path.geometry), now
    )
    if candidate_improvement_is_within_relative_threshold:
        assert decision.action == keep_current
        assert authorized_publisher.reauthorize_current(
            decision.remaining_path, decision.lease
        ).published()
```

**当前 Level 1 证据绑定（D07/T65）**：`test_level1_smoothing_timing_stability_matrix.cpp` 将本节 `18.2.7-1..22` 全部 22 项逐项绑定到 `test_stability_manager.cpp`、`test_plan_validity_evaluator.cpp`、`test_execution_lease_monitor.cpp`、`test_synchronized_validation_inputs.cpp`、`test_commitment_safety.cpp` 和 `test_main_planning_loop.cpp` 中已标记且实际调用的公共接口测试，并另执行 10 条支持链接。场景 1、2、6、9、19、20、22 直接绑定主循环集成测试，覆盖轻微改善保持、显著改善切换、当前计划失效直切、规划期间租约到期后的成对复检、非法候选代价保持/停车、上下文不一致停车及候选失败后的同上下文 keep/stop；场景 21 用真实 `PlanValidityEvaluator` 覆盖旧计划非零进度下从零起算候选的执行剖面版本前进，且当前计划入口继续严格绑定旧计划跟踪弧长和剖面版本。`T65-cost-invariant` 证明原子 publisher 无法被错误适配器用非法代价绕过，`T65-config-invariant` 证明非默认滞回配置到达基类非虚决策入口。`test_planning_result.cpp` 独立覆盖只读复制发布、完整依赖保留、序号单调和失败结果嵌套时间门禁；`test_planning_state_machine.cpp` 覆盖撤租先于停车/重规划、有认证停车距离才允许受控停车、超时复检、紧急事件覆盖和恢复需新同步快照/新租约。该证据证明主循环基类的不可覆盖决策入口实际调用已配置 `StabilityManager` 并原子处理 keep/switch；仍不证明 T52 尚未完成的真实平台制动、执行偏差阈值和生产租约有效期标定。

### 18.3 Level 2：DAVE/Gazebo 集成测试

本节定义 T48-T49 所需验收证据；D09 固定基线中没有相应报告，下列步骤和指标均不是已取得结果。动态进度只见 `project.md/PLAN.md`。

#### 18.3.1 双机协同测试

**测试目标**：验证前置机器人探测 → 地图更新 → 主机规划的闭环。

**测试步骤**：

1. 启动 DAVE 仿真环境
2. 初始化前置机器人和主机器人
3. 主机器人发现信息缺口
4. 发送补探请求给前置机器人
5. 前置机器人前往目标探测
6. 地图更新
7. 主机器人重新规划
8. 验证路径生成成功

**验证指标**：

- 双机距离始终不超过任务标定的 $d_{\text{comm}}^{\max}$；当前没有固定 50 m 生产值
- 补探请求到地图更新延迟 < 阈值
- 重规划成功率

#### 18.3.2 长时运行测试

**测试目标**：验证长时间、长距离运行稳定性。

**测试设置**：

- 模拟 1-2 km 路线（代表性局部路段）
- 注入周期性障碍和地图变化
- 运行 30-60 分钟

**监控指标**：

- 内存泄漏检测
- CPU 使用率
- 规划成功率
- 平均规划耗时
- 路径切换频率

#### 18.3.3 通信异常测试

**测试场景**：

1. 短时通信中断（< 5 秒）
2. 中等中断（5-15 秒）
3. 长时中断（> 15 秒）
4. 消息延迟和乱序

**验证**：

- 系统正确进入降级状态
- 恢复后正常运行
- 无状态回退或不一致决策

### 18.4 Level 3 & 4：水池与海试

#### 18.4.1 水池实验

**目标**：

- 标定机器人物理参数（转弯半径、爬坡能力）
- 标定缆线模型参数（触地点距离、方向响应长度）
- 确认出缆速度跟踪误差和有效张力范围
- 验证短距离布放精度

**实验内容**：

1. 直线铺设：测量落点误差
2. 转弯铺设：验证缆线模型
3. 简单障碍绕行：验证规划算法
4. 定位误差注入：分别验证机器人碰撞裕量和缆线横向协方差投影

**测量方法**：

- 水下定位系统提供真值
- 人工/声呐测量实际缆线落点
- 对比预测与实际位置

#### 18.4.2 海试

**目标**：

- 真实环境验证
- 验收指标测试
- 长距离能力演示

**验收指标测试**（根据 PRD）：

1. **落点精度**：
   - 横向误差均值、RMS、最大值
   - 分位数分布（如 P50, P90, P95）
   
2. **走廊符合率**：
   - 位于允许走廊内的缆线长度比例
   - 超出走廊的最长连续长度
   
3. **任务完成率**：
   - 成功完成的路段数 / 总路段数
   - 连续铺设距离
   
4. **布放成功率**（任务性能指标）：
   - 甲方目标：$R_{\text{lay}}\geq0.8$
   - 计算公式：
     $$
     R_{\text{lay}} = \frac{\text{实际铺设后符合位置容差的缆线长度}}{\text{实际总铺设长度}}
     $$
   - 同时单独报告 $\epsilon_{\text{point}}$ 对应的预测区段统计置信度；不得将20%不合格长度直接解释为20%模型误判概率
   - 首版不报告 $\epsilon_{\text{path}}$ 保证，直到误差相关长度和联合风险方法完成标定

### 18.5 基线对比实验

本节定义 T55 所需证据。D09 固定基线中没有二维 A*、标准 Hybrid A* 与本方案的同场景结果，不得使用“工程融合创新”措辞暗示已经取得对比优势；动态进度只见 `project.md/PLAN.md`。

**对比算法**：

1. **二维 A***：忽略航向和曲率约束
2. **标准 Hybrid A***：无参考线引导
3. **本方案**：参考线引导 + 缆线约束 + 稳定重规划

**对比指标**：

| 指标 | 说明 |
|------|------|
| 规划成功率 | 有解场景占比 |
| 计算耗时 | 平均/最大规划时间 |
| 路径长度 | 主机器人路径总长 |
| 触地点参考线偏离 | 预测/实际缆线触地点与参考路线的平均横向距离 |
| 缆线落点误差 | 横向误差 RMS |
| 超出走廊比例 | 违反约束的长度比例 |
| 路径平滑性 | 曲率变化率 |
| 路径稳定性 | 切换频率、抖动指标 |

**测试场景集**：

- 简单场景（平坦、稀疏障碍）：10 个
- 中等场景（坡面、多障碍）：20 个
- 复杂场景（狭窄通道、连续障碍）：10 个

### 18.6 消融实验

本节定义 T56 所需证据。D09 固定基线中没有消融结果；表中的“验证目标”是待检验假设，不是已证明贡献，依赖与动态进度只见 `project.md/PLAN.md`。

**消融项**：

| 机制 | 消融方式 | 验证目标 |
|------|----------|----------|
| 触地点参考线目标 | 移除 $c_{\text{td,corridor}}$，只保留硬走廊 | 触地点中心线软目标对落点精度的贡献 |
| 缆线落点模型 | 用机器人中心错误代替触地点参与目标和走廊评价 | 直接优化触地点而非机器人代理量的贡献 |
| 机器人碰撞协方差裕量 | 只使用固定 $d_{\text{safe}}$ | 机器人定位/控制误差裕量的贡献 |
| 缆线走廊概率界 | 只检查触地点均值 $|\mu_\perp|$ | 横向协方差投影与 $d_{\text{upper}}$ 的贡献 |
| 多候选并线点 | 只使用单一目标 | 多候选对路径质量的改善 |
| 路径滞回 | 移除滞回判断 | 稳定性机制的效果 |
| 主动补探 | 未知区域直接禁行，不补探 | 补探闭环对任务完成率的贡献 |

**评价方法**：

- 相同场景下对比消融前后的关键指标
- 量化每个机制的贡献

---

## 19. 风险、限制与未来扩展

### 19.1 已知限制

1. **受控触地点模型的适用范围**：
   - 第一阶段只适用于出缆速度跟随对地速度且张力处于标定范围的主动放缆
   - 不考虑流固耦合、垂向悬链线、触地后横向滑移和大幅松弛
   - 实际触地点或最终沉降中心线可能偏离预测
   - 缓解：显式返回模型有效性，通过水池实测标定 $L_{\text{td}}$、$L_{\psi}$ 和误差模型

2. **2.5D 地形表示**：
   - 不能处理悬岩、洞穴等复杂三维地形
   - 缓解：标记为禁行区域或人工介入

3. **定位与地图误差**：
   - 依赖上游定位和建图精度
   - 缓解：通过分域协方差传播和风险门禁吸收部分误差，但无法提供绝对确定性保证

4. **计算平台限制**：
   - 实时性取决于实际计算平台性能
   - 缓解：算法实验记录、标签/扩展/deadline 预算和 500 ms 安全失败；目标平台优化与降频策略尚未验证

5. **通信约束**：
   - 通信距离、失联分段阈值和补探运动能力均待任务/平台标定
   - 缓解：当前核心只提供距离评估、消息一致性和恢复门禁；真实通信降级仍需 T48-T49

6. **局部而非整路径联合风险**：
   - 机器人碰撞、地形梯度和缆线走廊使用彼此隔离的局部风险语义；`epsilon_path`、误差相关长度和整路径联合地形风险未实现
   - 缓解：诊断、结果和实验记录固定声明 `POINTWISE_ONLY:NO_PATH_JOINT_RISK_GUARANTEE`，禁止把采样点数量解释为独立试验次数

7. **搜索与运动集合边界**：
   - 当前是前进 Dubins 解析扩展和配置的恒曲率原语；Reeds-Shepp、倒车、原地转向、自适应分辨率和独立 byte-budget 未实现
   - 缓解：活动标签、扩展、时间和机械历史门禁失败关闭；平台能力确定后再决定是否扩展运动集合

8. **机器人支撑与停车模型边界**：
   - 当前履带模型不含前后支撑、俯仰和对角扭转；Level 1 停车模型不含坡度/载荷相关实机认证
   - 缓解：无认证停车距离时拒绝运动授权；T52 完成前不签发 production 能力声明

9. **系统集成边界**：
   - 当前仓库不含 ROS 2 adapter、原始传感器/地图构建、控制器、前置机器人运动规划或真实安全停车实现
   - 缓解：所有外部系统只能通过第 14 节公共 seam 提供版本化输入、执行反馈和独立停车通道；T48/T53/T54 验证前保持外部阻塞

10. **Level 1 与外部证据边界**：
    - 38 个 CTest 和确定性闭环报告不证明 2-5 Hz、1-2 km 长时稳定、模型统计覆盖、水池精度或海试验收
    - 缓解：第 18 节分层记录状态，未执行的基线/消融和外部试验不得写入结论

### 19.2 技术风险

| 风险 | 当前控制 | 剩余证据/处置 |
|------|----------|---------------|
| 物理能力或生产阈值未标定 | production 参数门禁、非生产 profile 标记、无认证停车距离失败关闭 | T50-T52 独立数据与平台标定 |
| 缆线模型在真实运行域覆盖不足 | 模型有效性、路径相关协方差、认证包络审计和 breach 撤租停车 | T51/T53；覆盖失败时签发新模型/包络版本，不能扩大经验置信度掩盖 |
| 地图过期、回退或混合版本 | `SnapshotManager`、消息水位、同步输入 revision 和完整依赖元组 | T48-T49 验证真实数据流频率与乱序边界 |
| 搜索/平滑/时序或复检超时 | 唯一根因、500 ms 周期截止、旧计划最新上下文全量复检和新租约 | 目标平台诊断样本；超时率不达标时优化或缩小经批准运行域 |
| 标签内存增长 | `maximum_active_labels`、每标签/进程内存采样和预算评估 | T49 长时无增长证据；当前没有独立 byte-budget |
| 复杂地形或统计包络下无解 | 区分普通无解与包络下无解，信息缺口触发补探，任何失败分支撤租或停车 | 外层任务调整、补探或人工介入；不得降低硬门禁取得路径 |
| 通信恢复导致陈旧授权 | 十流一致性门禁、恢复后完整同步上下文、更新计划与新租约 | T49 长时通信故障注入 |
| Level 1 结果被外推为生产性能 | 固定风险语义、证据层级和第 18 节状态表，D10 发布审计已确认边界 | T53-T54 外部验收及 T57 生产就绪审计 |

### 19.3 未来扩展方向

1. **高保真缆线模型**：
   - 引入流体力学、拖曳力、松弛和沉降
   - 有限元或集中质量法建模
   - 数据驱动或机器学习方法

2. **闭环落点反馈**：
   - 使用声呐或视觉直接观测已布放缆线
   - 校正模型参数或实时调整路径

3. **三维路径规划**：
   - 扩展到完整三维空间
   - 处理复杂地形和水层流场

4. **多机协同优化**：
   - 多个前置机器人协同探测
   - 主机与前置机联合优化

5. **学习与自适应**：
   - 从历史数据学习地形特征
   - 自适应调整代价权重和参数

6. **预测性规划**：
   - 基于地形趋势预测前方地形
   - 主动请求探测高风险区域

---

## 20. 待标定参数清单

以下参数在设计阶段无法确定，需要通过实际机器人、独立数据、实验或任务确认后标定。`[ ]` 表示外部值/证据未闭合，不表示代码中不存在对应字段或校验；`[x]` 只表示定义已冻结，也不表示相关生产能力已经验收。运行时 canonical schema 只有 `ParameterConfig`，本节按工程来源重组，不形成第二套键名。

| `ParameterConfig` 组 | 本节归属 | 当前结论 |
|---|---|---|
| 根 `schema_version/profile_id/mode/operating_domain_id` | 15.1/15.2 | 契约已实现；production profile 仍取决于下列外部值 |
| `robot` | 20.1-20.2 | 字段与有限性/范围门禁已实现；几何、能力、履带与协方差待 T52 |
| `terrain_gradient_risk` / `robot_collision_risk` | 20.2-20.3、20.6 | 版本/数据集/运行域/epsilon 门禁已实现；覆盖率与能力待 T50/T52 |
| `spatial_domains` | 20.5 | id、canonical 版本与非空门禁已实现；任务区域/走廊由任务提供 |
| `execution` | 20.2、20.4、20.6-20.7 | profile/跟踪/G2 字段门禁已实现；真实速度、制动、放缆、张力阈值待 T52 |
| `cable` | 20.4 | 模型、协方差、机械与地形门禁已实现；数值和运行域待 T51/T53 |
| `statistical_risk` | 20.4-20.6 | 点风险和包络依赖门禁已实现；`epsilon_path`/相关长度明确未实现 |
| `path_reuse` | 20.6-20.7 | 年龄、监控和续租字段门禁已实现；生产时长/偏差阈值待 T52 |
| `search` | 20.6 | canonical 离散化、关联、预算、扫掠和五权重字段已实现；任务调优与 byte-budget 未闭合 |
| `task` | 20.5、20.7 | 补探、通信和成功率字段门禁已实现；任务阈值和外部系统能力待确认 |

### 20.1 机器人几何与运动学

- [ ] 主机器人长、宽、高尺寸
- [ ] 机器人足迹形状（矩形/圆形/多边形）
- [ ] 左右履带有效支撑多边形 $F_L,F_R$
- [ ] 左右履带有效支撑中心距 $B_{\text{eff}}$
- [ ] 最小转弯半径 $\rho_{\min}$
- [ ] 最大曲率 $\kappa_{\max} = 1/\rho_{\min}$
- [ ] 最大空间曲率变化率 $u_{\max}=|d\kappa/ds|_{\max}$
- [ ] 当前实际曲率的估计来源、时间同步容差和失效语义
- [ ] 是否允许倒车
- [ ] 是否允许原地转向

### 20.2 机器人能力

- [ ] 最小/最大铺设对地速度
- [ ] 最大加速度、最大制动减速度和最大横向加速度
- [ ] 各速度/坡度/载荷组合下的停车距离模型及安全裕度
- [ ] 最大上坡角 $\alpha_{\max}^{\text{up}}$（机器人开发完成前保持空值）
- [ ] 最大下坡角 $\alpha_{\max}^{\text{down}}$（机器人开发完成前保持空值）
- [ ] 最大向上台阶 $h_{\max}^{\text{climb}}$（机器人开发完成前保持空值）
- [ ] 最大向下落差 $h_{\max}^{\text{drop}}$（机器人开发完成前保持空值）
- [ ] 最大横坡角 $\alpha_{\max}^{\text{lateral}}$
- [ ] 最大履带支撑滚转角 $\alpha_{\max}^{\text{roll}}$
- [ ] 左右履带最小有效支撑率
- [ ] 最小台阶检测高度 $h_{\text{step}}^{\min}$
- [ ] 最大允许粗糙度 $\sigma_{\text{rough}}^{\max}$ 的外部机器人能力标定数据仍待完成；运行时代码门禁已接入参数校验、`make_robot_capability` 和 `TraversabilityEvaluator`
- [ ] 基础障碍安全距离 $d_{\text{safe}}$

### 20.3 传感器与定位

- [ ] 定位误差标准差 $\sigma_{\text{loc}}$
- [ ] 地图配准误差 $\sigma_{\text{map}}$
- [ ] 高程测量误差 $\sigma_h$
- [ ] 地图分辨率 $\Delta r$
- [ ] 地图更新频率
- [ ] 生产传感器健康模式及各模式允许的最长失联/降级时间
- [ ] 前置机器人传感器覆盖宽度
- [ ] 前置机器人有效探测距离

### 20.4 缆线模型

- [ ] 放缆机最小/最大速度和最大加速度
- [ ] 张力设定值范围、张力跟踪误差及响应延迟
- [ ] 放缆点相对机器人本体的偏置 $(x_r, y_r)$
- [ ] 触地点等效水平距离 $L_{\text{td}}$
- [ ] 缆线方向响应长度 $L_{\psi}$
- [ ] 最大有效滞后角 $\delta_{\max}$
- [ ] 出缆速度跟踪误差上限 $\epsilon_{\text{payout}}$
- [ ] 有效张力范围 $[T_{\min},T_{\max}]$
- [ ] 初始缆线状态估计方法与误差 $\sigma_{\delta_0}$
- [ ] 触地点模型过程噪声协方差 $Q_{\text{model}}$
- [ ] 包络运行域 $\Gamma_H$：最大候选长度 $L_H$、最大规划时长 $T_H$、速度/张力和传感器健康边界
- [ ] 横向不确定性包络 $\bar\sigma_\perp(s_{\text{prog}})$、生成器版本及其绑定的缆线模型/执行运行包络版本
- [ ] 包络进度分辨率、离散余量 $\rho_{\text{env,disc}}$ 和审计容差 $\epsilon_{\text{env}}$
- [ ] 电缆硬走廊扫掠离散裕度 $\rho_{\text{cable,sweep}}$
- [ ] 制造商最小弯曲半径 $R_{\text{bend,min}}$
- [ ] 缆线偏好曲率 $\kappa_{\text{cable}}^{\text{preferred}}$ 与机械硬上限 $\kappa_{\text{cable}}^{\max}$
- [ ] 悬空代理物理窗口 $L_{\text{support,eval}}$ 与硬门禁 $\Delta h_{\text{support}}^{\max}$
- [ ] 缆线禁放区图层、膨胀规则和数据有效性阈值
- [ ] 缆线沉降高度 $\delta h_{\text{settle}}$

### 20.5 任务与约束

- [ ] 允许施工走廊宽度 $w_{\text{corridor}}$（可能分路段）
- [ ] 走廊风险策略版本及 `MARGINAL` 累计长度硬门禁 $L_{\text{marginal}}^{\max}$
- [ ] 机器人允许作业区域 $\mathcal W_{\text{robot}}$、版本来源及边界语义
- [ ] 允许落点横向误差阈值（用于成功率计算）
- [x] 布放成功率公式 $R_{\text{lay}}=L_{\text{actual,in-tolerance}}/L_{\text{actual,total}}$
- [x] 甲方布放成功率目标 $R_{\text{lay}}\geq0.8$
- [ ] 通信距离硬上限 $d_{\text{comm}}^{\max}$
- [ ] 期望前探距离 $d_{\text{scout}}^{\text{desired}}$
- [ ] 探测走廊宽度 $w_{\text{scout}}$
- [ ] 补探策略版本、最低地图置信度、采样间隔和缺口合并距离
- [ ] 最小安全剩余距离、规划提前量、平均速度和距离/时间滞回
- [ ] 前置机传感器覆盖半径、继续/停止距离及四项目标排序权重

### 20.6 算法参数

- [ ] 搜索空间离散化 $\Delta_{xy}$, $\Delta_{\theta}$, $\Delta_{\delta}$, $\Delta_s$
- [ ] 任务进度局部关联参数 $\epsilon_{\text{back}}$、$\alpha_s$、$\epsilon_s$、$d_0$、$\theta_0$、$\lambda_\theta$
- [ ] 等价机械历史标签代价支配的数值容差 $\epsilon_g$
- [ ] 搜索全局活动标签上限、扩展上限、单调时钟截止时间、解析扩展周期及机械历史标签内存基准；当前无独立 byte-budget
- [ ] 机器人位置/航向、滞后角、参考进度和触地点五项目标容差，以及最小转弯半径
- [ ] 普通恒曲率运动原语集合、原语集版本和输出路径版本
- [ ] 并线距离、终端航向偏置/滞后角集合、正向闭环容差、最大目标数及生成排序权重
- [ ] 原语自适应扫掠比例 $\eta$ 和碰撞离散裕度 $\rho_{\text{sweep}}$
- [ ] 局部平面拟合窗口 $W_{\text{surface}}$（使用物理尺寸）
- [ ] 稳健损失参数和最小拟合支撑率
- [ ] 局部梯度风险 $\epsilon_{g,\text{local}}$、覆盖系数 $\beta_g$、策略版本及独立标定数据集
- [ ] 台阶最小检测高度、边缘连续长度和置信阈值
- [ ] 台阶接近方向分类阈值 $\eta_{\min}$ 及过渡区宽度
- [ ] 规划窗口长度 $L_{\text{window}}$, $L_{\text{plan}}$, $L_{\text{buffer}}$
- [ ] 承诺段时间 $t_{\text{commit}}$ 或长度 $L_{\text{commit}}$
- [ ] 路径滞回阈值 $\Delta c_{\text{threshold}}$（如当前代价的 5-10%）
- [ ] 平滑空间步长、求解超时、最小段长和拓扑信赖域半径
- [ ] 动力学残差 $\epsilon_{\text{dyn}}$、曲率审计残差 $\epsilon_{\kappa}$、曲率变化率残差 $\epsilon_u$
- [ ] G2 拼接的位置、航向和曲率容差
- [ ] 时序参数化采样周期、终端速度、停车距离裕度和版本生成规则
- [ ] 认证执行运行包络版本及其与统计包络的绑定关系
- [ ] 对地速度、加减速度、出缆速度和张力的租约撤销阈值
- [ ] 五分量搜索代价权重 $w_{\text{length}}$, $w_{\text{curvature}}$, $w_{\text{td,center}}$, $w_{\text{td,margin}}$, $w_{\text{robot-terrain}}$，以及 `CableLayingLimits` 内部缆线适宜性权重
- [ ] 机器人碰撞风险参数 $\epsilon_{\text{robot}}$
- [ ] 单位置/相关区段走廊风险 $\epsilon_{\text{point}}$
- [ ] 整窗口走廊风险 $\epsilon_{\text{path}}$（首版TBD且不实现）
- [ ] 误差相关长度 $L_{\text{error-correlation}}$（首版TBD且不实现）
- [ ] 最小置信度阈值 $\text{confidence}_{\min}$

### 20.7 时间与频率

- [ ] 规划周期 $T_{\text{period}}$ (目标 2-5 Hz)
- [ ] 规划超时 $T_{\text{plan}}^{\max}$ (目标 500 ms)
- [ ] 地图有效期 $T_{\text{map}}^{\max}$
- [ ] 规划结果有效期 $T_{\text{valid}}$
- [ ] 路径复用最长租约 $T_{\text{reuse,max}}$
- [ ] 机器人状态、缆线状态与放缆遥测最大年龄
- [ ] 执行剖面跟踪状态最大年龄和异步撤租响应时间
- [ ] 租约监控周期 $T_{\text{monitor}}$ 与续租提前量 $T_{\text{renew}}$
- [ ] 补探请求超时 $T_{\text{scout}}^{\max}$
- [ ] 通信中断容忍时间（短/中/长阈值）

### 20.8 计算平台

- [ ] 实际使用的处理器型号和主频
- [ ] 可用内存大小
- [ ] 部署位置（主机、前置机或岸端）

### 20.9 仿真与验证

- [ ] ROS 2 发行版（如 Humble, Iron）
- [ ] DAVE/Gazebo 版本
- [ ] ArduSub 模型名称
- [ ] 主机器人 Gazebo 模型
- [ ] 水池场地尺寸和条件
- [ ] 海试场地和条件

---

## 21. PRD 需求追踪

### 21.1 核心能力需求映射

| PRD 需求 | 对应算法设计 | 验证方法 |
|----------|--------------|----------|
| 参考路线引导的局部修正 | 参考线引导 Hybrid A* (第 8 节) | 场景测试：偏离-回归行为 |
| 机器人通行性评价 | 2.5D 地形分析 (第 5 节) | 单元测试：坡度/台阶/碰撞 |
| 缆线落点约束 | 参数化缆线模型 (第 7 节) | 缆线落点测试 (18.2.4) |
| 可爬则爬 | 硬约束与代价函数 (第 8.4 节) | 场景测试：可爬坡面优先通过 |
| 局部绕行与并线 | 多候选并线点 (第 8.7 节) | 场景测试：单侧/双侧绕行 |
| 路径平滑与复检 | 曲率受限平滑 (第 9 节) | 平滑后约束验证 |
| 稳定重规划 | 路径滞回与承诺段 (第 10 节) | 稳定性测试 (18.2.7) |
| 主动补探 | 信息缺口识别与补探协调 (第 11 节) | DAVE 双机测试 (18.3.1) |
| 双机距离约束 | 距离硬约束与滞回 (第 11.3 节) | 双机协同测试 |
| 地图版本管理 | 版本锁定与时间戳 (第 13 节) | 消息乱序测试 |
| 降级策略 | 状态机与降级 (第 12 节) | 异常测试 (18.3.3) |

### 21.2 User Stories 覆盖

PRD 定义了 90 条 User Stories。下列“覆盖”只描述本设计或当前核心代码的责任边界，不代表外部系统和验收已经完成：

- **规划功能** (1-13)：核心算法与 Level 1 公共行为已实现；真实平台性能仍待 T48-T54。
- **前置机协调** (16-22)：补探协调、距离和消息契约已实现；前置机运动、真实通信和双机闭环待 T48-T49。
- **地图接口** (23-28)：版本化输入契约已实现；原始感知和地图构建属于上游且不在本仓库。
- **约束与优化** (29-33)：核心硬门禁与软代价已实现并经 Level 1 验证；生产阈值待 T50-T52。
- **缆线工程** (34-40)：简化触地点模型、机械/走廊约束和局部风险证据已实现；模型标定和水池证据待 T51/T53。
- **验收指标** (41-45)：指标和试验方案已定义；外部水池、海试与甲方验收待 T53-T54。
- **系统集成** (46-50)：公共 seam 和一致性要求已定义；ROS 2、仿真器、控制器与安全停车通道不在当前仓库，待 T48-T54。
- **操作员交互** (51-55)：核心状态/指令和人工接管门禁已实现；操作界面与现场流程属于外部系统。
- **开发与测试** (56-90)：Level 1 自动化证据已完成；Level 2-4、三算法基线和消融分别待 T48-T56，不能写为全部完成。

### 21.3 测试需求映射

| PRD Testing Decision | 对应测试方案 |
|----------------------|--------------|
| 确定性场景测试 | Level 1 测试 (18.2) |
| 边界与异常测试 | Level 1 边界测试 |
| 算法对比 | 基线对比 (18.5) |
| 消融实验 | 消融实验 (18.6) |
| ROS 2 集成 | Level 2 DAVE 测试 (18.3) |
| 长时/长距离 | 长时运行测试 (18.3.2) |
| 水池与海试 | Level 3 & 4 (18.4) |

---

## 22. 总结

### 22.1 总体算法架构

本设计采用**分层滚动规划 + 主动补探协同**架构：

安全约束作为硬约束优先保证；在满足全部硬约束的可行域内，以缆线落点质量为主要优化目标，并兼顾路线效率、曲率平滑性与地形适宜性。现有实现通过加权软代价进行权衡，不采用严格字典序优化。

1. **全局参考路线管理**：保留完整期望缆线落点路线
2. **局部地形分析**：从 2.5D 高程地图派生方向无关的表面、台阶和缆线地形层，再由机器人策略生成碰撞与方向通行性结果
3. **参考线引导 Hybrid A***：以 $(x,y,\theta,\delta,s_{\text{prog}})$ 为基础键，并按有限缆线机械历史保留可重开多标签，避免错误合并未来可行性不同的路径
4. **缆线落点预测与约束**：搜索使用与参考线、传感器模式、缆线模型和执行运行包络完整绑定的横向不确定性包络，候选验证传播实际路径协方差
5. **完整复检与时序授权**：平滑后独立机器人复检、时序化、完整缆线复检和最新上下文 `PlanValidityEvaluator` 共同签发短期租约
6. **稳定重规划与异步安全**：相对代价滞回、承诺段、执行偏差监控、承诺安全覆盖及先撤租后停车；Level 1 已验证，长时外部稳定性尚未验证
7. **主动补探协调**：信息缺口识别、补探目标生成、双机距离和请求生命周期；前置机运动与真实通信不在核心实现
8. **状态机、消息与恢复**：类型化消息一致性、规划状态/指令和恢复后新快照/新租约门禁
9. **主循环与诊断**：单周期锁定依赖、候选/当前成对复检、原子发布，以及可重放的耗时、内存、版本和风险证据

### 22.2 核心算法创新点

**明确说明**：本方案不发明新的搜索算法或地形表示方法，而是**工程融合**以下成熟技术和项目特定机制：

**使用的成熟算法**：
- Hybrid A* 搜索框架
- 2.5D 高程地图表示
- 栅格障碍膨胀（当前为逐障碍欧氏距离扫描；距离变换是未来优化目标）
- Clothoid / 曲率剖面平滑方法

**工程融合机制/待验证创新主张**（T55 基线与 T56 消融尚未执行，当前只能陈述设计差异，不能声称优于基线）：
1. **触地点参考线引导**：参考线通过触地点走廊代价、任务进度和反解接入目标参与搜索；可接受启发只使用机器人到反解目标的 Dubins 下界
2. **缆线落点直接参与规划**：预测落点作为硬约束和软代价实时影响搜索，而非规划后的被动统计
3. **参数化布放模型**：用增广滞后角状态表达可控主动放缆触地点，使搜索与验证共享同一模型
4. **分域误差预算**：机器人碰撞、地形梯度估计和缆线走廊分别使用独立误差模型；方向坡度使用二维梯度覆盖集合，缆线使用横向投影与保守机会约束替代界
5. **稳定重规划机制**：路径滞回 + 承诺段，抑制地图微变导致的路径振荡
6. **主动补探闭环**：信息缺口 → 补探请求 → 地图更新 → 重规划的显式协调

### 22.3 最核心的算法设计

1. **参考线引导的 Hybrid A***（第 8 节）：
   - 在硬约束可行域内，代价函数以触地点走廊偏离所表达的落点质量为主要软目标，并加权兼顾机器人路径长度、曲率和地形适宜性；不使用机器人中心到缆线参考线的代理偏差，也不采用严格字典序优化
   - 多候选并线点策略
   - 每个搜索节点实时验证缆线落点约束

2. **参数化缆线布放模型**（第 7 节）：
   - 受控放缆触地点状态模型
   - 放缆点偏置、触地点距离与方向响应长度配置
   - 增广滞后角、任务进度和有限机械约束记忆共同保持搜索所需的 Markov 性
   - 搜索与验证使用同一模型

3. **稳定重规划**（第 10 节）：
   - 滞回判据：新路径必须显著优于当前路径才切换
   - 承诺段：锁定近端不可修改的路径段
   - 滚动周期：使用局部快照、参考窗口和有界搜索预算；长距离扩展能力仍需 T49 验证

### 22.4 仍然待标定的参数

**关键物理参数**（阻塞实现的优先级最高）：
- 机器人几何尺寸和足迹
- 最小转弯半径
- 最大上坡/下坡角、最大爬阶/落差和履带支撑参数
- 缆线放缆点偏置、触地点距离和方向响应长度

**任务参数**（影响验收）：
- 允许施工走廊宽度
- 允许落点误差阈值
- 布放成功率目标已确定为 $R_{\text{lay}}\geq0.8$；具体位置容差仍需确认

**算法参数**（影响性能，可实验调优）：
- 代价权重
- 规划频率和超时
- 滞回阈值
- $\epsilon_{\text{robot}}$、$\epsilon_{\text{point}}$
- $\epsilon_{\text{path}}$ 与 $L_{\text{error-correlation}}$（首版保留TBD）
- $s_{\text{prog}}$ 离散化与局部关联参数、$\epsilon_g$
- 横向不确定性包络的运行域、分段和离散裕度

**计算平台**（影响实时性）：
- 处理器型号
- 部署位置
- 实际可达频率

完整清单见第 20 节。

### 22.5 PRD 中可能阻塞实现的问题

**不阻塞**：PRD 已经明确系统边界、输入输出、约束分层和参数化原则，算法设计可以进行。

**需要后续确认但不阻塞开发**：
1. **机器人物理参数**：开发和仿真估计值只进入显式非生产 profile，实测标定及独立验证前不形成 production 能力值
2. **缆线模型细节**：第一阶段使用简化模型，预留扩展接口
3. **验收与风险阈值**：$R_{\text{lay}}\geq0.8$ 已明确；位置容差、$\epsilon_{\text{point}}$、$\epsilon_{\text{robot}}$ 仍待确认
4. **计算平台**：算法设计与平台解耦，部署时适配

**建议优先确认**（不阻塞但影响方案选择）：
1. 主机器人是否允许倒车（影响运动原语集）
2. 地图分辨率和更新频率（影响实时性设计）
3. 实际计算平台（影响复杂度预算）

---

## 附录 A：关键公式汇总

**方向坡度**：
$$
\mu_{\text{long}}=\hat{\mathbf g}\cdot\mathbf t,
\quad
\sigma_{\text{long}}=\sqrt{\mathbf t^T\Sigma_g\mathbf t},
\qquad
\mu_{\text{lat}}=\hat{\mathbf g}\cdot\mathbf l,
\quad
\sigma_{\text{lat}}=\sqrt{\mathbf l^T\Sigma_g\mathbf l}
$$
$$
q_{\text{long}}^{\min/\max}=\mu_{\text{long}}\mp\beta_g\sigma_{\text{long}},
\qquad
q_{|\text{lat}|}^{\max}=|\mu_{\text{lat}}|+\beta_g\sigma_{\text{lat}}
$$

**去趋势粗糙度**：
$$
\sigma_{\text{rough}}=
\sqrt{\frac{\sum_iw_i(h_i-\hat h_i)^2}{\sum_iw_i}}
$$

**台阶接近方向与履带滚转**：
$$
\eta=\mathbf t\cdot\mathbf n_{\text{step}}
\quad\text{（只分类接近方式，不缩放 }h_{\text{step}}\text{）}
$$
$$
\alpha_{\text{roll}}=\arctan\left(
\frac{\bar h_L-\bar h_R}{B_{\text{eff}}}
\right)
$$

**机器人障碍裕量（各向同性保守形式）**：
$$
d_{\text{total,robot}}=d_{\text{safe}}+
z_{1-\epsilon_{\text{robot}}}
\sqrt{\lambda_{\max}(\Sigma_{\text{robot-rel}})}
$$

**缆线走廊保守机会约束替代式**：
$$
\sigma_\perp=\sqrt{\mathbf n_{\text{ref}}^T\Sigma_c\mathbf n_{\text{ref}}},
\qquad
\bar\sigma_\perp(s_{\text{prog}})\geq
\sup_{\gamma\in\Gamma_H(s_{\text{prog}})}\sigma_\perp(\gamma,s_{\text{prog}})
$$
$$
d_{\text{upper,search}}=|\mu_\perp|+
z_{1-\epsilon_{\text{point}}/2}\bar\sigma_\perp(s_{\text{prog}})
+\rho_{\text{cable,sweep}}
$$

首版固定 $\Sigma_{\text{ref}}=0$。搜索使用路径无关的横向包络，验证传播实际 $\Sigma_c(\gamma)$ 并审计包络覆盖。$d_{\text{upper}}$ 是概率约束的充分条件，不是精确等价式；$R_{\text{lay}}\geq0.8$ 也不意味着 $\epsilon=0.2$。

**缆线触地点预测**：
$$
\mathbf{c}(s) = \mathbf{p}_{\text{release}}(s) - L_{\text{td}}
\begin{bmatrix}\cos\psi_c(s) \\ \sin\psi_c(s)\end{bmatrix},
\qquad
\frac{d\psi_c}{ds} = \frac{\operatorname{wrap}(\theta-\psi_c)}{L_{\psi}}
$$

**Hybrid A* 代价函数**：
$$
\mathbf x=(x,y,\theta,\delta,s_{\text{prog}},\mathcal M_{\text{lay}}),
\qquad
k=(i_x,i_y,i_\theta,i_\delta,i_s)
$$

$k$ 仅为基础键；每个 $k$ 下按 $\mathcal M_{\text{lay}}$ 的未来等价类保存多个活动标签。
$$
g_e=
w_{\text{length}}L_e
+w_{\text{curvature}}|\kappa|L_e
+\sum_{i=1}^{n}\frac{c_{i-1}+c_i}{2}
\left(s^{\text{robot}}_i-s^{\text{robot}}_{i-1}\right)
+\texttt{CableLayingEvaluation.soft\_cost}
+w_{\text{robot-terrain}}\sigma_{\text{rough}}(s_{\text{mid}})L_e
$$

其中 $c_i$ 是第8.4节定义的第 $i$ 个触地点走廊代价密度，只读取预测触地点与参考线的关系。机器人中心仅检查 $\mathcal W_{\text{robot}}$、通行性和运动学，不计算到缆线参考线的偏差。

当前五个公开成本分量为机器人长度、机器人曲率、触地点走廊、缆线适宜性和机器人地形。走廊项按相邻样本密度的梯形平均乘机器人弧长间隔；缆线适宜性直接使用 `CableLayingEvaluator.soft_cost`，不重复乘原语长度。生成阶段 `MergeGoal.soft_cost` 仅用于目标排序/截断，不加入搜索 $g$。

可接受启发为六族前进 Dubins 精确目标距离与机器人目标位置/航向容差域下界的较小者，再对所有并线目标取最小并乘 $w_{\text{length}}$。开放队列固定按 `(g+h, h, g, label_id, node_id)` 升序排序；滞后角、参考进度、触地点和其他软成本不进入启发或隐藏次级键。

相同基础键使用机械历史多标签；只在 `future_equivalent` 证明等价的标签内按最低代价替换并跳过失活队列项。实际路径协方差不进入首版搜索标签，机械曲率与悬空代理所需的有限历史必须进入。

**路径滞回判据**：
$$\text{switch} = (c_{\text{new}} < c_{\text{current}} - \Delta c_{\text{threshold}})$$

---

## 附录 B：术语表

| 术语 | 定义 |
|------|------|
| 参考路线 | 前期规划的期望缆线落点中心线 |
| 参考路线进度状态 | $s_{\text{prog}}$，搜索携带的连续任务阶段，不是全局最近点投影 |
| 横向不确定性包络 | 在版本化设计运行域 $\Gamma_H$ 内覆盖路径相关 $\sigma_\perp$ 的标量上界 |
| 机器人作业区域 | 机器人完整足迹允许进入的区域，与缆线施工走廊独立 |
| 施工走廊 | 参考路线两侧允许预测缆线触地点进入的范围，不约束机器人中心 |
| 同步复检输入 | `SynchronizedValidationInputs`；在同一 revision 原子冻结机器人、缆线、参考进度、执行跟踪和完整依赖版本的只读证据 |
| 执行剖面 | `ExecutionProfile`；与几何路径同弧长域的版本化速度、加速度、放缆和张力样本及批准跟踪限制 |
| 验证租约 | `PlanValidationLease`；计划仅在指定依赖、时间窗和执行剖面范围内继续执行的短期授权，不是安全证据缓存 |
| 原子授权发布 | 不可变计划、批准剩余 `TimedPath`、租约和比较代价作为一组发布；任一项缺失都不暴露可执行命令 |
| pointwise-only 风险 | 仅对单位置或局部相关区段给出风险上界；不包含 `epsilon_path` 或整路径联合地形风险保证 |
| 非生产能力配置 | `non_production_capability_profile`；仅用于合成/算法验证，必须在验证结果中保留非生产标记 |
| 缆线机械历史 | `CableConstraintMemory`；覆盖曲率/支撑物理窗口的实际或候选触地点有限历史，决定未来机械可行性 |
| 搜索基础键 / 多标签 | 五元离散键 $(i_x,i_y,i_\theta,i_\delta,i_s)$ 可保留多个机械历史不等价标签；只在证明 `future_equivalent` 后按代价替换 |
| 2.5D 地图 | 每个 $(x,y)$ 位置只有一个高程值的高程图 |
| 通行性 | 机器人是否可以安全通过某区域 |
| 落点适宜性 | 某区域是否适合布放缆线 |
| Hybrid A* | 混合状态空间 A* 搜索，结合离散和连续状态 |
| 承诺段 | 机器人近端已锁定的、不可修改的路径段 |
| 滞回 | 对同一最新同步上下文中均有效的新旧计划执行相对代价阈值和可选拓扑距离判断；安全事件可覆盖 |
| 滚动窗口 | 由任务参数提供的局部参考/地图规划范围，随已执行进度更新；当前无固定 50 m 生产默认值 |
| 信息缺口 | 地图未覆盖或置信度低的区域 |
| 补探 | 前置机器人前往指定位置进行探测以填补信息缺口 |
| 受控停车 | 先撤销活动租约，再通过外部独立安全停车通道请求停车；核心库不实现底层制动控制 |
| 生产公共接口 | `include/underwater_planner/core/*.hpp` 的 32 个头文件；source-only helper 和 `testing/*.hpp` 不属于该集合 |
| 测试支持 | `underwater_planner::test_support`；合成夹具、确定性闭环驱动器和报告代码，不得被生产适配层依赖 |

---

**文档结束**

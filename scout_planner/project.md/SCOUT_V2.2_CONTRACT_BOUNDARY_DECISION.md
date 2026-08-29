# 前导机器人 V2.2 契约边界决策记录

> **状态**：已冻结；Tickets 02-11 已发布公共契约，ticket 12 已发布契约对齐的 V2.2 正式设计  
> **决策日期**：2026-08-27  
> **正式基线**：`SCOUT_MOTION_ALGORITHM_DESIGN_V2.2.md`  
> **公共基线**：系统 `CONTEXT.md`、ADR 0001-0003、v1 系统集成/状态机契约和 `interfaces/`  
> **适用范围**：前导任务、规划、执行授权、FCU、地图和铺缆协同的语义所有权与 seam；不直接修改公共契约

## 1. 决策摘要

1. 跨进程、跨 NUC 的公共事实只由 Protobuf v3 定义；前导内部 C++、ROS 2 和 MAVLink 都是 adapter，不得增加或竞争公共语义。
2. 首版搜索冻结为**时间感知 3D state-lattice A\***：空间占据是三维的，搜索状态还包含离散航向、动作模式和有界到达时间标签。禁止再称为经典 Hybrid A*。
3. 主机铺缆运动和前导 AUV 运动是两个隔离的执行授权域。每个域内部只有一个软件 `ExecutionAuthority`；任何权威都不能为另一机器人的执行器签发、续租或撤销授权。
4. 前导 `ExecutionAuthority` 与前导执行 adapter 位于 Scout NUC 的同一安全时钟域。主机只发布补探任务、预测、协调条件和取消事实，不发布前导执行授权。
5. 世界/地图事实统一为 `mission_enu`，机器人本体向量统一为 `base_link`/FLU。NED/FRD 只存在于 Scout NUC 到 FCU 的 MAVLink adapter 内。
6. 公共瞬时 ENU yaw 规范化到 `[-pi, pi)`；连续轨迹用“规范化初始 yaw + 不重置的 yaw offset 控制点”保留绕转信息，避免与公共 yaw 规范化及内容 hash 规则冲突。
7. 安全 deadline、lease、watchdog 和顺序判断只使用接收设备本地单调时钟。跨设备同步观测时间只用于观测对齐和审计。
8. 生产者会话、消息 sequence、业务版本和内容身份保持四个正交概念，任何一个都不能替代另一个。

## 2. 语义所有权

### 2.1 所有权矩阵

| 事实或动作 | 唯一所有者 | 规范表示 | 其他层的职责 |
|---|---|---|---|
| 补探任务及取消 | 主机任务编排权威 | 公共 Protobuf `ScoutMission` 及后续生命周期消息 | 前导 C++ 显式适配为内部任务值对象；ROS 2 只传输和编排生命周期 |
| 前导导航状态 | 定位权威 | 后续公共三维导航快照 | 前导规划器只读消费；MAVLink/ROS 状态不得形成竞争事实 |
| 混合三维地图快照 | 感知建图权威 | 后续公共、可分片、可验证的地图快照 | `MapChunk` 只承载规范载荷；规划器不得猜测编码或缓存旧安全结论 |
| 传感器几何与健康 | 传感器配置/健康权威 | 后续公共版本化配置和状态 | 规划器绑定版本；ROS 参数不能覆盖生产配置 |
| 海流、能力和能源 | 各自估计或配置权威 | 后续公共版本化快照/profile | C++ 使用适合计算的内部类型，但不能外推或补默认值 |
| 铺缆机器人预测与协调条件 | 主机预测/协调权威 | 后续公共预测和协调消息 | 前导只判断自身计划是否满足，不修改主机预测 |
| 前导候选计划 | `ScoutMotionPlanner` | 后续公共 `ScoutPlanningResult` | 候选无执行权；ROS 发布成功不能转化为授权 |
| 前导执行授权与软件撤销 | Scout `ExecutionAuthority` | 后续前导专用原子授权包和撤销消息 | Planner/SafetySupervisor 只提交候选、事实或撤销请求 |
| 前导计划目标到 FCU setpoint 的转换 | Scout 执行 adapter | 版本化 MAVLink/FCU profile | 可拒绝或更保守限幅，必须显式反馈；不得重规划、放宽或静默改目标 |
| FCU 实测状态与本地保护 | FCU | MAVLink 遥测和本地保护事实 | 执行 adapter 转换为公共反馈；本地保护可立即停止，但不能签发软件 lease |
| 前导任务状态 | Scout Mission 状态所有者 | 公共正交状态域 | Planner、adapter 和 FCU 只能提交事件，不能各自发布竞争状态 |

### 2.2 四个 seam

后续实现只暴露四个外部 seam，复杂性留在深 module 内：

1. **公共契约 seam**：Protobuf 消息、状态、代码、profile、Manifest 和 hash 规则。
2. **规划 seam**：`ScoutMotionPlanner::plan(ScoutPlanningContext, deadline, cancellation)`；ROS 2 不逐步编排搜索、平滑和验证内部 module。
3. **授权 seam**：Scout `ExecutionAuthority` 原子接受候选及最新事实，输出授权包或结构化拒绝；调用方不能自行拼装 plan 与 lease。
4. **执行 seam**：Scout 执行 adapter 原子安装授权包、按固定执行历元采样并发布反馈；FCU/MAVLink 细节不进入规划接口。

内部可有测试 seam，但不得为了测试把搜索器、验证器或 MAVLink 字段暴露成新的公共事实。

## 3. 坐标、航向和时间

### 3.1 坐标与 adapter 规则

| 位置 | 世界/地图 | 机器人本体 | 约束 |
|---|---|---|---|
| 公共 Protobuf | `mission_enu`，右手 ENU | `base_link`，右手 FLU | SI、有限值、显式 frame；未知 frame 整体拒绝 |
| 前导核心 C++ | 与公共契约相同 | 与公共契约相同 | 不允许核心算法接收 NED/FRD |
| ROS 2 adapter | 显式适配到上述规范 frame | 显式适配到上述规范 frame | 变换及外参是版本化依赖，不允许依赖隐式 TF 最新值 |
| MAVLink/ArduSub adapter | 在此处唯一执行 ENU <-> NED | 在此处唯一执行 FLU <-> FRD | 每轴和每方向必须有 golden test；转换失败时不得下发 setpoint |

### 3.2 连续轨迹 yaw

V2.1 的内部展开航向 `tilde_psi in R` 是必要的，因为转速、角加速度、传感器朝向和连续 Bézier 导数不能在 `pi/-pi` 处跳变。公共 v1 又要求瞬时 ENU yaw 规范化到 `[-pi, pi)`，且 hash 规范会规范化 ENU yaw。V2.2 采用以下无歧义表示：

- 计划保存一个 `initial_yaw_rad in [-pi, pi)`；
- 每个 yaw 控制量保存为相对该初始值的 `yaw_offset_rad`，它是旋转位移而不是需规范化的瞬时 ENU yaw；
- 第一段首控制点 offset 必须为零，后续段不重置 offset；
- 相邻段的位置、yaw offset 及所要求导数连续；
- 内部展开航向由 `initial_yaw_rad + yaw_offset_rad` 唯一重建；
- 对外状态/反馈中的瞬时 `yaw_rad` 仍规范化到 `[-pi, pi)`。

Ticket 08 必须把这一表示写入 schema 和 canonical hashing 测试，不能直接发布可能超出范围的绝对 `yaw_rad` 控制点。

### 3.3 时钟域与 deadline

- 主机和 Scout NUC 的单调时钟不可直接比较。
- `MessageHeader.source_clock_domain_id` 说明消息生成时间所属域；域变化或生产者重启使旧会话事实失效。
- Scout lease、执行历元、反馈新鲜度和软件撤销 deadline 全部属于 Scout NUC 安全时钟域。
- FCU watchdog 使用 FCU 本地 receive tick、session 和 sequence，不比较 NUC 绝对单调时间。
- `SynchronizedObservationTime` 只用于跨设备观测对齐和审计，不驱动 lease、watchdog 或任务 deadline。
- Ticket 02 已将原 `ScoutMission.deadline_monotonic_ns` 明确发布为发送方域的 `business_deadline_monotonic_ns`，并由 `ScoutMissionDecision` 返回 Scout 本地 receive/admission 窗口。Scout NUC 不得把发送方业务 deadline 与本地 `now` 直接比较；该字段不构成前导安全 deadline。

### 3.4 身份与顺序

| 概念 | 范围 | 规则 |
|---|---|---|
| 生产者会话 | 一次进程或设备生命周期 | 重启创建新 UUID；旧会话不复活 |
| 消息 sequence | `(producer_id, producer_session_id, stream_id)` | 严格递增，只解决投递顺序 |
| 业务版本/sequence | 任务、地图、计划、lease、bundle 等各自权威 | 表达演进顺序，不能证明内容相同 |
| 内容身份 | 规范化不可变内容 | SHA-256；证明身份/完整性，不是来源认证 |

同一业务 sequence 加同一内容身份是幂等重复；同 sequence 不同身份是完整性冲突并失败关闭。所有待执行安全对象必须按 `interfaces/HASHING.md` 规范化，并绑定兼容的 `ContractManifest`。

## 4. 搜索算法决定

### 4.1 选定算法

首版名称固定为：

> **时间感知 3D state-lattice A\***  
> 英文标识建议：`TimeAwareStateLatticeAStar3d`

其基础状态为：

```text
(voxel_x, voxel_y, voxel_z, discrete_yaw_bin, action_mode)
```

节点还记录累计代价、预计到达时间、能量下界和父节点；只有时间相关约束确实需要时，才允许每个基础状态保留少量有界到达时间标签。

选择离散航向的原因：

- 固定朝向传感器的覆盖与可见性依赖航向；
- 能力包络显式约束 yaw rate 和 yaw acceleration；
- 原语已经包含 `delta_yaw` 和正时长；
- 平滑器需要具有可行朝向演化的几何/粗时序种子。

### 4.2 明确排除的名称

- **不是纯 3D A\***：纯 `(x,y,z)` 状态不能保留航向相关的传感器、能力和运动原语可行性。
- **不是经典 Hybrid A\***：首版不在连续状态上做车辆模型传播，也不承诺 Dubins/Reeds-Shepp analytic expansion；离散状态与预定义原语更符合 state-lattice。
- “3D”描述占据与平移空间维度，不表示整个搜索状态只有三个维度。

设计、类型、测试和诊断中不得继续使用旧 Hybrid 搜索器名称。若未来改为连续状态 Hybrid A* 或删除航向维，必须先发布新的设计决策，不能只改类名。

## 5. 两个隔离的执行授权域

```text
主机铺缆域
Laying Planner -> Main ExecutionAuthority -> Main ExecutionAdapter -> STM32F4
       |
       +---- ScoutMission / prediction / coordination ----+
                                                        |
前导运动域                                              v
ScoutMotionPlanner -> Scout ExecutionAuthority -> Scout ExecutionAdapter -> FCU
                           ^                    |
                           | facts/revoke req   +-> explicit execution feedback
                 Scout SafetySupervisor
```

跨域箭头不携带执行授权。具体规则：

1. 现有 `ImmutablePlan`、`AuthorizedExecutionBundle`、履带/放缆/张力反馈和 CAN profile 保持主机铺缆域专用语义。
2. Tickets 08-10 已新增前导专用 `ScoutPlan`、`ScoutAuthorizedExecutionBundle`、`ScoutExecutionLease`、`ScoutBundleAck`、`ScoutExecutionFeedback`、`ScoutExecutionRevocation` 和 `ScoutExecutionRevocationAck`。所有前导消息禁止复用主机专用 payload 并用空字段表示 AUV。
3. Scout `ExecutionAuthority` 是前导域内唯一 bundle/lease/revocation 业务序列所有者；主机权威不能写入这些水位。
4. Scout `ExecutionAuthority` 与 Scout 执行 adapter 必须共享 Scout NUC 安全时钟域。跨设备授权若存在，必须在目标域重新签发；本首版不提供这种路径。
5. 任务取消、地图变化、协调变化、健康下降和安全监督器输出只形成事实或撤销请求；只有 Scout `ExecutionAuthority` 发布规范软件撤销。
6. FCU 本地保护和独立硬急停/最小风险路径可不等待权威立即停止。解除后仍须新会话/新上下文、重规划和新前导授权包。
7. 公共状态已由 ticket 01A 区分主机与前导授权实例：保留现有主机 `STATE_DOMAIN_EXECUTION_AUTHORITY` 作为弃用别名，新增 `STATE_DOMAIN_MAIN_EXECUTION_AUTHORITY` 与 `STATE_DOMAIN_SCOUT_EXECUTION_AUTHORITY`，并以不同消息类型隔离授权；订阅者不得根据 `producer_id` 猜测。

Ticket 01A 已在获得明确跨模块授权后闭合原系统缺口：根 `CONTEXT.md`、ADR 0003、系统集成与状态契约现均把唯一性量化为“**每个物理执行授权域恰有一个软件权威**”，主机与 Scout 使用独立规范发布者、本地时钟、状态身份、消息类型和水位，跨设备提案必须在目标域重新验证并签发。Tickets 09-11 已正式发布原子授权、执行反馈、精确撤销、状态转换、稳定代码与不可变配置；ticket 12 已将这些权威语义双向追踪到正式 V2.2 设计。

## 6. V2.1 与公共 v1 的差异及兼容策略

| 差异 | V2.2 决定 | 追踪 ticket |
|---|---|---|
| 内部 `SurveyRequest` 与公共 `ScoutMission` 形成两套身份 | 公共任务身份唯一；内部仅是显式、无损适配后的值对象，不再发布竞争版本 | 02 |
| 公共任务的跨域单调 deadline 含义不安全 | 拆分发送方业务 deadline 与 Scout 本地 admission/age 门禁 | 02 |
| `MapChunk` 载荷编码和混合地图结构未定义 | 由公共 payload schema、编码、压缩、重组和 hash 规则唯一规定 | 03 |
| `ScoutStatus` 只有 2D pose + z，无法表达完整三维导航事实 | 新增完整导航快照；状态摘要只引用其版本，不能替代快照 | 04 |
| 传感器、海流、能力和能源只有设计内部结构 | 增加版本化公共事实/profile，生产值不得缺省 | 05、06 |
| 铺缆预测和协调仅有内部近似结构 | 增加时间区间、保守占据、不确定性、LossPolicy 和内容身份 | 07 |
| 公共 `PlanningResult`/`ImmutablePlan` 是铺缆二维路径和放缆剖面 | 新增前导专用四维候选计划；候选无授权语义 | 08 |
| 展开 yaw 与公共 yaw/hash 规范冲突 | 使用规范化初始 yaw + 连续 yaw offset 控制点 | 08 |
| 公共授权包和反馈是主机履带/缆线专用 | 新增前导专用 bundle、反馈、ACK 和撤销，不重解释旧字段 | 09、10 |
| 公共授权状态域和 ADR 使用未限定单例 | 明确每个物理执行域唯一，并给状态和消息强类型域身份 | 01A、09、11、12 |
| 前导结果码、故障码、TimingProfile、InterfaceLimits、LossPolicy 未闭合 | 使用稳定注册表和不可变 profile；未知安全值整体拒绝 | 11 |
| V2.1 多处使用 Hybrid A* 名称 | 全部改为时间感知 3D state-lattice A* | 12 及后续实现 tickets |

Ticket 04 Evidence：`interfaces/proto/underwater/contracts/v1/{common,state,profiles}.proto`
发布独立导航 stream、完整三维位姿/体速度、两类协方差、有效状态、版本、
时钟和内容身份；`interfaces/SCOUT_NAVIGATION_STATE.md`、`HASHING.md`、
非生产 `profiles/integration-v1.json` 与精确 Manifest feature gate 闭合
新鲜度、同会话版本水位、失败关闭和 ENU/NED、FLU/FRD golden vectors；
`interfaces/tests/test_scout_navigation_state.py` 提供公共 seam 可执行证据。

Ticket 07 Evidence：`MainRobotPrediction` 以同步对齐历元和连续闭区间三维扫掠球体发布
铺缆机器人有限时域占据，`ScoutCoordinationConstraint` 精确绑定任务/预测并发布分离、几何通信、
链路保证边界和双向 LossPolicy 引用；规范、hash、profile、Manifest 和 12 项可执行 seam 测试位于
`interfaces/SCOUT_MAIN_ROBOT_COORDINATION.md`、`interfaces/HASHING.md`、
`interfaces/profiles/integration-v1.json`、`interfaces/compatibility/contract-manifest-v1.json` 和
`interfaces/tests/test_scout_coordination_contract.py`。完整契约一致性套件 84/84 通过（2026-08-27）。

Tickets 01A/09 Evidence：根 `CONTEXT.md`、ADR 0003、系统集成/状态契约和
`interfaces/proto/underwater/contracts/v1/{common,execution,state,profiles}.proto`
发布独立主机铺缆/前导运动授权域、Scout 原子 plan/lease/Bundle/ACK、固定本地
执行历元、授权区间、精确依赖/profile 与不可复活水位；规范、hash、非生产 profile、
Manifest 和可执行反例位于 `interfaces/SCOUT_AUTHORIZATION_BUNDLE.md`、
`interfaces/HASHING.md`、`interfaces/profiles/integration-v1.json`、
`interfaces/compatibility/contract-manifest-v1.json`、
`interfaces/tests/test_{execution_authority_domains,scout_authorization_bundle_contract}.py`。

Ticket 10 Evidence：`interfaces/proto/underwater/contracts/v1/{common,execution,profiles}.proto`
发布独立前导反馈、精确撤销与撤销 ACK stream，反馈绑定 Bundle/plan/trajectory/lease
身份、固定历元时间、profile/applied/measured 三视图、显式限幅、控制模式、风险动作和
FCU 会话；撤销绑定强类型前导域、稳定原因、停止/风险动作与规范身份，并以先落水位和
本地停止、后等待高优先级幂等 ACK 的规则失败关闭。规范、hash、非生产 profile、Manifest
和可执行参考消费者位于 `interfaces/SCOUT_EXECUTION_FEEDBACK_REVOCATION.md`、
`interfaces/HASHING.md`、`interfaces/profiles/integration-v1.json`、
`interfaces/compatibility/contract-manifest-v1.json` 和
`interfaces/tests/test_scout_execution_feedback_revocation_contract.py`；完整契约一致性套件
138/138 通过（2026-08-28）。

Ticket 11 Evidence：`interfaces/registry/scout-state-transitions-v1.json` 与
`interfaces/proto/underwater/contracts/v1/{codes,state,diagnostics,profiles}.proto`
发布精确的前导任务/授权/逐链路通信转换、稳定 trigger 与 Scout code、安全效果和
审计拒绝载荷；非生产 `interfaces/profiles/integration-v1.json` 原子绑定时序、资源、
代码/转换注册表、能力、能源、传感器、planner/SafetyGate、两条 LossPolicy 和风险动作规则。
规范、hash、Manifest 及可执行参考消费者位于
`interfaces/SCOUT_STATE_CODES_PROFILES.md`、`interfaces/HASHING.md`、
`interfaces/compatibility/contract-manifest-v1.json` 和
`interfaces/tests/test_scout_state_codes_profiles_contract.py`；完整契约一致性套件
148/148 通过（2026-08-28）。

兼容规则：

- 保留现有主机消息的字段号、数值身份和主机专用语义；通过新增前导消息和 stream 建立新能力，不重解释旧 payload。
- 涉及安全关键字段、状态或授权语义的变更不能作为“可忽略 minor 字段”发布；不兼容组合必须由 Manifest 和 feature gate 拒绝。
- 不支持前导授权能力的旧 peer 可以维持非执行的任务/诊断观察，但不能武装或执行前导计划。
- 未知枚举、缺失安全字段、错误 frame/时钟域、版本回退、同序列不同 hash、超限消息和非有限值全部失败关闭，不提供隐式降级。
- ROS 2、C++ 和 MAVLink adapter 必须做逐字段双向测试；能反序列化不等于安全语义兼容。

## 7. 首版非目标

- 不在 A1 修改 Protobuf、代码注册表、profile、系统 ADR 或状态机实现。
- 不把前导计划并入主机履带/放缆/张力 `ExecutionProfile`。
- 不提供主机对 Scout FCU 的执行器接管，也不提供 Scout 对 STM32F4 的控制路径。
- 不使用 hash/CRC 充当认证；v1 仍依赖受信、隔离的双 NUC 链路。
- 不定义经典 Hybrid A*、多拓扑搜索、强制 SCP、完整在线 6DOF 集合证明、概率风险预算或恢复流管。
- 不把到达扫描点、传感器开启或计划覆盖率当作探测完成证据。
- 不声明 DAVE/Gazebo、HIL、水池、海试、生产参数或真实水下安全认证已完成。

## 8. 下游发布门禁

Tickets 02-11 每票必须同时提供：

1. schema/注册表/profile 的规范变更；
2. 正常、边界、非法、重复、乱序、过期、重启和未知安全值测试；
3. 内部 C++ <-> Protobuf 与 ROS 2 <-> Protobuf 逐字段双向适配要求；
4. Manifest、InterfaceLimits 和 canonical content identity 规则；
5. 与本决策相应行的可追踪 Evidence。

Ticket 12 已按以下发布门禁完成 V2.2：

- 本文所有差异均已闭合或明确标为外部门禁；
- 设计章节与公共 schema/状态/代码/profile 双向可追踪；
- 系统级 `ExecutionAuthority` 唯一性已明确限定到物理执行授权域；
- 搜索名称、数学状态、类名、测试名和诊断一致；
- 完整契约一致性套件和 mixed-version fail-closed 测试通过；
- 未夸大仿真、实机、生产参数或认证状态。

## 9. A1 验收映射

| A1 验收项 | 决策证据 |
|---|---|
| 公共 Protobuf、C++、ROS 2、FCU/MAVLink 语义所有权 | 第 2 节所有权矩阵和四个 seam |
| 坐标、展开 yaw、时钟域、会话、版本和身份 | 第 3 节 |
| 搜索算法分类 | 第 4 节 |
| 前导授权及与主机域隔离 | 第 5 节 |
| 差异、兼容、非目标和后续追踪 | 第 6-8 节 |

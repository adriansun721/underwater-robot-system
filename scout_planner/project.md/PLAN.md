# 前导机器人规划系统实施计划

> 主要设计基线：`SCOUT_MOTION_ALGORITHM_DESIGN_V2.2.md`
> 次要参考：系统公共契约、状态机、铺缆侧协同需求与仿真/硬件边界
> A1 决策基线：`SCOUT_V2.2_CONTRACT_BOUNDARY_DECISION.md`
> 当前状态：A1 与 Tickets 01A、02-25 已完成；混合三维地图查询、能力/能源门禁、双机器人协调评价、探测几何、确定性 state-lattice 搜索、约束平滑、独立计划探测证据和连续几何验证已在不可变规划上下文之上闭合，tickets 26-27 是下一工程前沿

## 1. 使用规则

本文件是前导机器人后续开发的唯一动态进度入口。详细 ticket 位于 `.scratch/scout-v1/issues/`，一票一文件；设计细节以当前发布的前导设计基线为主，其他模块只用于核对跨模块边界和兼容性。

任务状态只使用：

- `[ ] ready-for-agent`：所有阻塞依赖已完成，可立即领取。
- `[ ] blocked`：仍有未完成的内部依赖。
- `[ ] blocked-external`：内部工作已准备好，但缺少明确的跨模块修改范围、外部仿真、硬件、标定或试验条件。
- `[x] done`：所有验收标准通过并记录 Evidence。

每个 ticket 完成时必须：

1. 更新本文件中的状态。
2. 在对应 ticket 和本文件追加 `Evidence:`。
3. 执行相关测试；契约、接口、安全语义或设计发生变化时执行完整契约一致性测试。
4. 更新前导设计中的需求追踪关系，不得用测试默认值冒充生产能力。
5. 发现跨模块问题时先登记新的阻塞 ticket，不顺手修改无关模块。

## 2. 基线与边界决策

- 前导 V2.2 设计是任务分解与实施的正式基线；V2.1 只作为其历史简化来源。
- `interfaces/` 和 `docs/` 是公共语义权威；前导内部 C++ 类型不得扩展或复制不兼容的公共语义。
- Ticket 02 已把前导任务摘要升级为版本化 admission/取消/完成证据生命周期；Ticket 03 已发布五层不可变混合三维地图及分片重组契约；Tickets 04-06 已发布导航、传感器/海流及能力/能源输入；Ticket 07 已发布铺缆机器人预测与前导协调条件；Ticket 08 已发布无授权语义的前导四维计划结果；Tickets 01A/09 已发布隔离的双授权域和前导原子授权包；Ticket 10 已发布前导执行反馈、精确撤销与撤销 ACK；Ticket 11 已闭合正交状态、稳定代码、原子配置、逐链路 LossPolicy 与恢复链；Ticket 12 已把上述权威语义双向追踪到 V2.2 正式设计。
- A1 已将首版搜索冻结为“时间感知 3D state-lattice A*”：状态包含三维体素、离散航向和动作模式，可带有界到达时间标签；不得称为纯 3D A* 或经典 Hybrid A*。
- 首版保持 V2.2 继承的简化原则：已知自由空间授权、有限候选、能力包络、统一安全验证、短时 lease 和独立最小风险监督。
- 多拓扑、强制 SCP、完整在线 6DOF 集合证明、概率风险预算和恢复流管不是首版前置依赖。

## 3. 全局完成定义

- 同一输入和版本集合能够确定性重放。
- 公共消息、内部类型、状态机、结果码、hash 和依赖版本可双向追踪。
- UNKNOWN、STALE、CONFLICTED、过期依赖、未知安全枚举和非有限值均失败关闭。
- 只有通过独立安全复检的精确计划前缀能够获得短时授权。
- lease 过期、依赖变化、跟踪偏差或健康下降能够异步撤销，不等待主规划线程。
- 探测完成必须绑定真实观测和新地图版本，不能由“到达目标点”代替。
- ROS 2 adapter 只做同步、转换和生命周期编排，不修改算法或授权结论。
- 生产参数必须绑定设备、数据集、运行域、日期和版本；非生产值明确标记。

## 4. Ticket 清单

### A. 设计与公共契约

| ID | 状态 | Ticket | Blocked by |
|---|---|---|---|
| 01 | `[x] done` | 冻结前导 V2.2 契约边界 | None |
| 01A | `[x] done` | 对齐系统级双执行授权域契约 | 01；明确的跨模块修改范围 |
| 02 | `[x] done` | 打通补探任务生命周期契约 | 01 |
| 03 | `[x] done` | 固化混合三维地图快照契约 | 01 |
| 04 | `[x] done` | 固化前导导航状态契约 | 01 |
| 05 | `[x] done` | 固化传感器与海流契约 | 01 |
| 06 | `[x] done` | 固化能力与能源契约 | 01 |
| 07 | `[x] done` | 固化双机器人预测与协调契约 | 01, 02 |
| 08 | `[x] done` | 发布前导 4D 计划结果契约 | 02-07 |
| 09 | `[x] done` | 发布前导授权包与租约契约 | 08, 01A |
| 10 | `[x] done` | 发布前导执行反馈与撤销契约 | 09 |
| 11 | `[x] done` | 闭合前导状态、代码和配置规则 | 02, 07, 10, 01A |
| 12 | `[x] done` | 发布契约对齐后的 V2.2 设计基线 | 03-11 |

### B. 核心工程与输入闭环

| ID | 状态 | Ticket | Blocked by |
|---|---|---|---|
| 13 | `[x] done` | 建立可重复的前导算法工程 | 12 |
| 14 | `[x] done` | 实现核心类型与 Protobuf 双向适配 | 13 |
| 15 | `[x] done` | 捕获原子一致的规划上下文 | 14 |
| 16 | `[x] done` | 实现混合三维地图查询 | 14 |
| 17 | `[x] done` | 实现能力、海流与能源硬门禁 | 14 |
| 18 | `[x] done` | 实现双机器人协调评价 | 14 |
| 19 | `[x] done` | 实现探测动作与观测几何 | 14, 16 |

### C. 搜索、轨迹与验证

| ID | 状态 | Ticket | Blocked by |
|---|---|---|---|
| 20 | `[x] done` | 打通确定性 3D state-lattice A* | 16 |
| 21 | `[x] done` | 升级为时间感知协同搜索 | 17-20 |
| 22 | `[x] done` | 实现规范五次 Bézier 轨迹库 | 14 |
| 23 | `[x] done` | 实现约束平滑与时间参数化 | 16, 17, 21, 22 |
| 24 | `[x] done` | 生成独立的计划探测证据 | 19, 23 |
| 25 | `[x] done` | 实现连续几何与导数验证器 | 16, 22, 23 |
| 26 | `[ ] blocked` | 闭合统一安全验证报告 | 17, 18, 24, 25 |
| 27 | `[ ] blocked` | 打通 ScoutMotionPlanner 主链 | 15, 21, 23, 26 |

### D. 授权、撤销与降级

| ID | 状态 | Ticket | Blocked by |
|---|---|---|---|
| 28 | `[ ] blocked` | 实现 SafetyGate 短时授权 | 09, 27 |
| 29 | `[ ] blocked` | 实现 LeaseMonitor 与旧计划复用 | 10, 28 |
| 30 | `[ ] blocked` | 实现独立 SafetySupervisor | 16-18, 29 |
| 31 | `[ ] blocked` | 闭合前导运行状态机 | 11, 28-30 |

### E. 集成与验证

| ID | 状态 | Ticket | Blocked by |
|---|---|---|---|
| 32 | `[ ] blocked` | 完成确定性 Level 1 闭环 | 27-31 |
| 33 | `[ ] blocked` | 实现 ROS 2 适配与可视化闭环 | 12, 15, 27, 31 |
| 34 | `[ ] blocked` | 完成 SIL/DAVE/Gazebo 双机联调 | 32, 33；外部仿真环境 |
| 35 | `[ ] blocked` | 完成 HIL 与水池参数辨识 | 34；FCU、机器人和水池 |
| 36 | `[ ] blocked` | 完成海试与生产就绪审计 | 35；海试平台与验收方案 |

## 5. 依赖前沿

Tickets 01A、02-16 已完成；系统现以强类型消息、状态域、本地时钟和独立水位隔离主机铺缆与前导运动授权，前导四维候选只有经 `ScoutAuthorizedExecutionBundle` 原子绑定并通过安装门禁后才获得有限前缀执行权。Ticket 13 建立独立 ROS 无关 C++ 核心与确定性三维夹具，Ticket 14 将 19 类权威公共消息无损映射为不可默认构造的纯 C++ 核心值；Ticket 15 在该边界上捕获不可变规划上下文，Ticket 16 从其中精确绑定的地图身份构造五层失败关闭查询并发布确定性 supercover。下一工程前沿由 tickets 17-18 推进能力/海流/能源门禁和双机协调评价，Ticket 20 已可消费地图查询实现确定性 3D state-lattice A*：

```text
01 [done] -> 01A [done] -> 09 [done] -> 10 [done] -> 11 [done] -> 12 [done] -> 13 [done] -> 14 [done]
|
├─ 02 [done] ─┬─ 07 [done] ─┐
├─ 03 [done] ─┤              │
├─ 04 [done] ─┤              ├─ 08 [done] -> 09 [done]
├─ 05 [done] ─┤              │                  -> 10 [done] -> 11 [done] -> 12 [done]
└─ 06 [done] ─┴──────────────┘

10 -> 11 -> 12 -> 13 -> 14 -> 15 [done] -> 16 [done] -> 20 [done]
                                      \-> 17 / 18 / 19 [ready]
16 -> 20 -> 21 -> 23 -> 24/25 [done] -> 26 -> 27
27 -> 28 -> 29 -> 30/31 -> 32 -> 33/34 -> 35 -> 36
```

可并行边界：

- 契约与设计阶段：01A、02-16 已完成；17-20 可按依赖立即领取。
- 系统授权域：主机/Scout 双域已闭合；后续契约不得重解释旧主机消息或共享域间水位。
- 核心阶段：16 已完成；17-20 可按门禁、协调、探测几何和搜索分工。
- 算法阶段：22 可与 20-21 并行；25 可在轨迹和地图接口稳定后独立开发。
- 外部验证阶段只有在前序自动化证据完成后才能启动，不得用仿真成功替代契约或单元验证。

Ticket 01 Evidence: `project.md/SCOUT_V2.2_CONTRACT_BOUNDARY_DECISION.md`；从系统根目录执行 `powershell -NoProfile -ExecutionPolicy Bypass -File interfaces/tests/run.ps1`，公共契约一致性测试 15/15 通过（2026-08-27）。

Ticket 02 Evidence: `interfaces/proto/underwater/contracts/v1/{common,cooperation,profiles}.proto`、`interfaces/SCOUT_MISSION_LIFECYCLE.md`、`interfaces/HASHING.md`、`interfaces/profiles/integration-v1.json` 和精确 Manifest feature gate 闭合任务唯一身份、required/allowed ENU AABB、发送方业务 deadline 与 Scout 本地 admission、接受/拒绝、取消/ACK、计划预测证据、实际 observation IDs + 严格更新地图版本的完成证据及完成 ACK。现有稳定代码已覆盖所需结果/诊断，无需新增竞争身份。`interfaces/tests/test_scout_mission_lifecycle.py` 新增 13 项公共 seam 检查；从系统根目录执行 `powershell -NoProfile -ExecutionPolicy Bypass -File interfaces/tests/run.ps1`，schema 编译、序列化往返、非法/未知枚举、幂等/冲突规则、adapter 无损映射、时序、hash、Manifest 与原有契约完整套件 28/28 通过（2026-08-27）。

Ticket 03 Evidence: `interfaces/proto/underwater/contracts/v1/{mapping,cooperation,profiles}.proto`、`interfaces/HYBRID_MAP_SNAPSHOT.md`、`interfaces/HASHING.md`、非生产 `interfaces/profiles/integration-v1.json` 和精确 `scout_hybrid_map_snapshot_v1` Manifest gate 闭合五层 ENU/X-fastest 地图载荷、版本/hash/时间/区域/外参与建图参数依赖、穷举体素状态、确定性编码、版本化压缩、分片完整性/重组/缺片 ACK 及整图资源门禁。`interfaces/tests/test_hybrid_map_snapshot.py` 新增 10 项公共 seam 检查，以可执行参考消费者覆盖完整语义验证、乱序/重复、混合元数据、缺片、CRC/SHA、未知枚举、版本水位、超限和悬垂障碍；完整契约一致性套件 38/38 通过（2026-08-27）。

Ticket 04 Evidence: `interfaces/proto/underwater/contracts/v1/{common,state,profiles}.proto`、`interfaces/SCOUT_NAVIGATION_STATE.md`、`interfaces/HASHING.md`、非生产 `interfaces/profiles/integration-v1.json` 和精确 `scout_navigation_state_v1` Manifest gate 闭合定位权威三维 ENU/FLU 位姿、体速度、位置/姿态协方差、有效状态、版本/会话/时钟/观测时间、负零/NFC canonical identity、精确 Manifest/profile、同域新鲜度、delivery sequence/完整字节幂等、退役会话拒绝、unknown field/资源超限失败关闭、逐字段 adapter 要求及 ENU/NED、FLU/FRD 双向转换规则。`interfaces/tests/test_scout_navigation_state.py` 新增 13 项公共 seam 检查，以编译后 Protobuf 和可执行参考消费者覆盖完整往返、合法/阈值边界消费、非有限/非法姿态/大尺度非对称或非半正定协方差/frame/时钟/过期/未知/超限拒绝、hash 篡改、Manifest/profile、会话/消息/版本水位、完整 header 篡改拒绝、adapter 要求和单轴/单方向 golden vectors；完整契约一致性套件 51/51 通过（2026-08-27）。

Ticket 05 Evidence: `interfaces/proto/underwater/contracts/v1/{common,sensing,profiles}.proto`、`interfaces/SCOUT_SENSOR_AND_CURRENT.md`、`interfaces/HASHING.md`、非生产 `interfaces/profiles/integration-v1.json` 和精确 `scout_sensor_and_current_v1` Manifest gate 闭合固定 FLU 外参、视场/量程/分辨率、已知自由射线遮挡、标定来源与运行域，独立几何/健康版本及同域有效期，以及 ENU 局部仿射海流模型的时空适用域、完整分量/范数误差界和可选梯度。`interfaces/tests/test_scout_sensor_current_contract.py` 新增 10 项公共 seam 检查，以编译后 Protobuf 和可执行参考消费者覆盖完整往返、非平凡安装方向与量程边界、同传感器配对、健康变化/过期、海流越域/超龄/frame、误差界/梯度、session/sequence/version 水位、canonical identity、unknown field、资源上限、精确 Manifest/profile artifact 和 NON_PRODUCTION 拒绝；完整契约一致性套件 61/61 通过（2026-08-27）。

Ticket 06 Evidence: `interfaces/proto/underwater/contracts/v1/{common,capability,profiles}.proto`、`interfaces/SCOUT_CAPABILITY_AND_ENERGY.md`、`interfaces/HASHING.md`、非生产 `interfaces/profiles/integration-v1.json` 和精确 `scout_capability_and_energy_v1` Manifest gate 发布彼此独立的能力 profile、推进器健康、能源模型与能源状态流，闭合 nominal/DEGRADED_A/DEGRADED_B 精确 profile 绑定、速度/加速度/垂向/偏航/姿态/推进余量、运行域禁止外推、主动制动延迟与最小减速度、保守功率模型、返航或风险动作加 reserve 的能源硬门禁，以及设备/数据集/运行域/日期/版本/hash 生产证据。`interfaces/tests/test_scout_capability_energy_contract.py` 新增 11 项公共 seam 检查，以编译后 Protobuf 和可执行参考消费者覆盖完整往返、健康降级与复检、边界内消费/越域拒绝、主动制动距离、仅够到达但不够返航/风险动作、非有限/过期/未知/未注册值、session/sequence/version 水位、canonical identity、unknown field、资源上限、精确 Manifest/profile artifact 和 NON_PRODUCTION 拒绝；完整契约一致性套件 72/72 通过（2026-08-27）。

Ticket 07 Evidence: `interfaces/proto/underwater/contracts/v1/{common,cooperation,profiles}.proto`、`interfaces/SCOUT_MAIN_ROBOT_COORDINATION.md`、`interfaces/HASHING.md`、非生产 `interfaces/profiles/integration-v1.json` 和精确 `scout_main_robot_coordination_v1` Manifest gate 发布独立铺缆机器人预测与前导协调流，闭合任务/预测精确配对、同步对齐历元、连续闭区间三维扫掠占据、显式不确定度与发送方有效窗、Scout 本地接收年龄、分离/几何通信硬边界、链路保证边界及双向 LossPolicy 引用。`interfaces/tests/test_scout_coordination_contract.py` 新增 12 项公共 seam 检查，覆盖移动占据、共享端点/预测末端、过期/间隙/越界、时钟域/任务/预测错配、版本水位、非有限/未知/超限、约束冲突、链路保证过度声明和失联后双流新会话恢复；完整契约一致性套件 84/84 通过（2026-08-27）。

Ticket 08 Evidence: `interfaces/proto/underwater/contracts/v1/{common,codes,planning,profiles}.proto`、`interfaces/SCOUT_4D_PLANNING_RESULT.md`、`interfaces/HASHING.md`、`interfaces/registry/codes-v1.json`、非生产 `interfaces/profiles/integration-v1.json` 和精确 `scout_4d_planning_result_v1` Manifest gate 发布独立且无授权语义的前导四维计划结果，以 `mission_enu` 三维五次 Bézier 控制点、规范初始 yaw 和连续 yaw offset 唯一重建 C2 轨迹，原子绑定 Tickets 02-07 输入、TimingProfile、InterfaceLimits、计划探测证据及独立验证报告，并闭合结果优先级、失败载荷、canonical identity、资源、未知字段/枚举、重复/乱序和重启门禁。`interfaces/tests/test_scout_planning_result_contract.py` 新增 16 项公共 seam 检查；完整契约一致性套件 100/100 通过（2026-08-27）。

Ticket 01A Evidence: 根 `CONTEXT.md`、ADR 0003、`docs/system-integration-contract.md`、`docs/state-machine-contract.md` 及 `interfaces/proto/underwater/contracts/v1/{execution,state}.proto` 将唯一性限定为每个物理执行授权域一个软件权威，保留主机 v1 线语义并新增前导强类型域/状态，固定各自 NUC 本地时钟、跨设备重新签发、独立硬停止和不共享水位规则。精确 `independent_execution_authority_domains_v1` Manifest gate 与 `interfaces/tests/test_execution_authority_domains.py` 提供双域反例；完整契约一致性套件 127/127 通过（2026-08-28）。

Ticket 09 Evidence: `interfaces/proto/underwater/contracts/v1/{common,execution,profiles,state}.proto`、`interfaces/SCOUT_AUTHORIZATION_BUNDLE.md`、`interfaces/HASHING.md`、非生产 `interfaces/profiles/integration-v1.json` 和精确 `scout_authorization_bundle_v1` Manifest gate 发布独立 `ScoutExecutionLease`、`ScoutAuthorizedExecutionBundle`、`ScoutBundleAck` 与专用 stream，原子绑定 ScoutPlan/hash、完整依赖、固定执行历元、严格授权区间、lease、TimingProfile、InterfaceLimits 和 SafetyGate 配置，闭合安装 ACK、幂等/冲突、错误域/时钟/计划/依赖/profile/窗口拒绝、未知字段/资源门禁及撤销/过期水位不可复活。`interfaces/tests/test_scout_authorization_bundle_contract.py` 新增 25 项公共 seam 检查，包含规范发布者、完整依赖与 SAFE 指标、嵌套轨迹/测量证据身份及 C2 连续性重算、本机精确 profile 与时限、ACK 消费、不完整五次曲线、未知安全结果及 delivery/business 序列正交性；完整契约一致性套件 127/127 通过（2026-08-28）。

Ticket 10 Evidence: `interfaces/proto/underwater/contracts/v1/{common,execution,profiles}.proto`、`interfaces/SCOUT_EXECUTION_FEEDBACK_REVOCATION.md`、`interfaces/HASHING.md`、非生产 `interfaces/profiles/integration-v1.json` 和精确 `scout_execution_feedback_revocation_v1` Manifest gate 发布独立前导反馈、精确撤销与撤销 ACK stream。反馈绑定 Bundle/plan/trajectory/lease 身份、固定历元时间、profile/applied/measured 三视图、显式限幅、控制模式、风险动作、FCU 会话和命令水位；撤销绑定强类型前导域、稳定原因、停止/风险动作与规范身份，水位落盘和本地停止不等待 ACK，高优先级重试保持字节级幂等。`interfaces/tests/test_scout_execution_feedback_revocation_contract.py` 新增 11 项公共 seam 检查，覆盖往返、事实到结构化撤销原因、独立重试安全通道、跟踪偏差、健康下降、地图变化、通信丢失、lease 到期、ACK 丢失、事件/关联/因果链、旧会话、时间/命令回退、延迟反馈、静默限幅、未知 enum/wire field、超限、冲突重试和不复活；完整契约一致性套件 138/138 通过（2026-08-28）。

Ticket 11 Evidence: `interfaces/registry/scout-state-transitions-v1.json` 以稳定 trigger code 发布 ScoutMission、ScoutExecutionAuthority、MAIN_SCOUT_COOP 与 SCOUT_NUC_FCU 四个独立状态实例的精确合法转换和风险动作规则；`interfaces/proto/underwater/contracts/v1/{codes,state,diagnostics,profiles}.proto`、`interfaces/registry/codes-v1.json`、非生产 `interfaces/profiles/integration-v1.json` 和精确 `scout_state_codes_profiles_v1` Manifest gate 补齐 Scout Outcome/Fault/Diagnostic/trigger 身份、故障安全效果和单向恢复链、原子能力/能源/传感器配置引用、逐链路 LossPolicy、FCU 超时顺序及状态/审计/profile 资源上限。`interfaces/SCOUT_STATE_CODES_PROFILES.md` 与 `interfaces/HASHING.md` 固化未知安全枚举、非法转换、缺失生产证据和不兼容 profile 的拒绝/审计规则。`interfaces/tests/test_scout_state_codes_profiles_contract.py` 新增 10 项公共 seam 可执行检查；完整契约一致性套件 148/148 通过（2026-08-28）。

Ticket 12 Evidence: `project.md/SCOUT_MOTION_ALGORITHM_DESIGN_V2.2.md` 发布契约对齐后的正式实施基线，将首版搜索唯一命名为时间感知 3D state-lattice A*，以 `ScoutMission`、`ScoutPlanningDependencies`、`ScoutPlanningResult`、正交 ScoutMission/ExecutionAuthority/逐链路状态、`ScoutAuthorizedExecutionBundle`、三视图反馈和精确撤销替换旧跨边界草图；附录 D 对公共 schema/注册表/profile/Manifest 与设计章节建立双向追踪，附录 E 记录 descriptor、代码注册表、状态转换注册表、NON_PRODUCTION 配置及 11 个精确 feature 身份，章节 28/30 明确保留 DAVE/Gazebo、HIL、水池、海试与生产标定门禁。Ticket 13 仅追加 25.8 工程基线追踪后，设计文件 SHA-256 为 `299fd7537db76b5f034994242ad6def45a4a109e08bf5c99af8c34ea8a4fecac`；Ticket 15 追加 19.2、Ticket 16 追加 4.6 实施追踪后，当前设计文件 SHA-256 为 `60ede49426945cb075b6df0a3d627fe7e6dc1024d60dcaa77e5a668e883ea5a3`，均未改变公共 wire 语义；绑定的 Manifest 文件 SHA-256 仍为 `cf560519ebe19d879311818335a2aa1e25f61f44793ae171021c483be8b2b8a3`；从系统根目录执行 `powershell -NoProfile -ExecutionPolicy Bypass -File interfaces/tests/run.ps1`，schema 编译、hash、Manifest 与完整契约一致性套件 148/148 通过（2026-08-28）。本 ticket 未修改公共 schema、注册表或 profile，不构成生产就绪或外部验证声明。

Ticket 13 Evidence: `CMakeLists.txt`、`CMakePresets.json`、`tools/verify.{ps1,sh}` 与 `src/scout_core/` 建立独立 ROS 无关 C++17 核心；所有目标启用 warnings-as-errors，`static-check` 按编译器选择 MSVC `/analyze`、GNU `-fanalyzer` 或 Clang `clang-tidy`，CTest 生成 `build/verify/ctest.xml` JUnit。公开 `deterministic_fixture.hpp` seam 覆盖平坦海床、悬垂障碍、狭窄通道、UNKNOWN、STALE、CONFLICTED 和移动主机，其中移动主机以连续闭区间保守扫掠球表达物理半径、位置不确定度与占据半径；固定输入/种子/版本逐字段重放一致，并在包括夹具构造前失败在内的测试路径携带 seed、SI 单位、`mission_enu`、Scout 本地时钟域、输入版本和 `non_production=true`。2026-08-28 最终独立冷工作流 2/2 通过，配置、编译、静态分析和测试共 8,288 ms，CTest 0.06 s，七场景夹具进程 7 ms、峰值 RSS 4,676 KiB；完整口径见 `project.md/TICKET_13_BASELINE.md`。本证据不引用其他模块测试，不构成生产、SIL/HIL 或实机能力证明。

Ticket 14 Evidence: `src/scout_core/include/scout_planner/core/protobuf_adapter.hpp`、`src/scout_core/src/protobuf_adapter.cpp` 与构建期生成的全量 v1 Protobuf C++ 类型发布 19 个强类型 ROS 无关 `CoreContract` 值对象及单一双向适配/校验入口。核心值没有默认构造；递归值树逐字段保留整数宽度、浮点、bool、UTF-8 text、bytes、enum 类型/数值、message presence、repeated 顺序、字段号和 schema 身份。适配器集中拒绝 unknown/missing/unspecified、NaN/Inf、非 NFC text、非规范 frame/yaw、负值或逆序时间、版本回退、错误 identity、资源超限、非单位四元数、非对称/非半正定协方差、地图维度不符、非 C2 五次段、结果/候选依赖错配和失败结果伪候选；identity 按 `HASHING.md` 清除 header/self-identity 后从嵌套叶节点向根递归确定性序列化、SHA-256 并恒定时间比较，消费端不可绕过，生产者可显式重建完整身份树。`golden-schema-check` 将排序后的全部 v1 descriptor 固定为 `44ded468a534e8c75722d98125d3eb49c2aebf3a7ab0f2ee8cecc251907cf7f1`。`src/scout_core/test/test_protobuf_adapter.cpp` 的公共 seam 测试与既有核心/夹具测试在 `tools/verify.ps1` 下 3/3 通过；系统根 `interfaces/tests/run.ps1` 完整公共契约套件 148/148 通过（2026-08-28）。本 ticket 未实现 Ticket 15 上下文捕获、Ticket 16 地图查询、Tickets 17-18 算法门禁，也不构成 ROS 2、SIL/HIL、实机或生产参数证据。

Ticket 15 Evidence: `src/scout_core/include/scout_planner/core/planning_context.hpp` 与 `src/scout_core/src/planning_context.cpp` 发布 `PlanningContextBuilder::capture` 和不可默认构造的 `ScoutPlanningContext`。捕获入口以 source generation 前后相等证明读取期间未更新，按每类 Scout 本地 receipt 最大年龄（含等号边界）、消息本地观察/有效窗和精确时钟域失败关闭；跨设备关键观测必须同步有效、在不确定度上限内且总跨度不超过配置。任务同时携带经统一 Protobuf/core seam 验证规范身份的 `ScoutMissionDecision`，仅 `ACCEPTED`、本地 admission 窗有效、decision receive tick 等于捕获的 mission receipt 且精确绑定任务/协调版本时可进入上下文；预测、协调、传感器对、能力/健康、能源、地图、导航、海流和激活配置逐项与 `ScoutPlanningDependencies` 的版本/内容身份匹配，运行域精确一致。Delivery 水位按 stream 固定 canonical producer/session，并为同流逻辑对象保存可精确重复的独立位置，任何新交付仍须严格超过 stream 水位；业务水位按逻辑对象维护，预测/协调 session 必须原子刷新，所有水位只在整次成功后提交。发布快照复制输入、receipt、配置、捕获时刻和 generation，只暴露 const 视图。`src/scout_core/test/test_planning_context.cpp` 以公开 seam 场景覆盖缺失/拒绝/过期 admission、decision/receipt 拼接、竞态、过期/未来边界、部分更新、配对/时钟/运行域/同步错配、竞争 publisher、同流多传感器稳定重捕获、字典序与 sequence 反向、新交付落后 stream 水位、单次捕获混合 session、逻辑 ID 变化下的序列回退、版本/会话、地图源时钟隔离和不可变性；`tools/verify.ps1` 下配置、warnings-as-errors、MSVC `/analyze`、golden descriptor 与 CTest 4/4 通过；系统根 `interfaces/tests/run.ps1` 完整公共契约套件 148/148 通过（2026-08-28）。本 ticket 未实现地图查询、能力/能源数值门禁、协调几何评价、ROS 2 或外部/生产验证。

Ticket 16 Evidence: `src/scout_core/include/scout_planner/core/hybrid_map_query.hpp` 与 `src/scout_core/src/hybrid_map_query.cpp` 发布与 `map_id`、`map_version` 和规范 content identity 精确绑定的不可变点查询及确定性线段 supercover。查询直接投影 Ticket 14 已校验的纯 C++ 核心值，不建立第二个 Protobuf seam；组合海床、三维占据、ESDF、允许水体、声明地图区域和连续语义 AABB，返回有效 FREE/OCCUPIED/UNKNOWN/STALE/CONFLICTED、含边界保守净空、质量、地图/外参/建图参数版本、地图时间/区域、语义限制、X-fastest 体素序列及带 ENU 位置的信息缺口。机体、定位、跟踪、地图与离散误差保持五个有限非负 SI 输入，其总和保守膨胀允许水体及语义边界。地图外/边界净空不足、不确定占据、缺块、非有限 ESDF、未知语义和身份错配失败关闭，2.5D 海床不覆盖三维悬垂事实。`src/scout_core/test/test_hybrid_map_query.cpp` 覆盖自由点、悬垂物、薄障碍、薄语义区、裕量膨胀的允许水体/NO_ENTRY、ESDF、边界、UNKNOWN/STALE/CONFLICTED、无效数值/依赖和角点 supercover；`tools/verify.ps1` 下 warnings-as-errors、MSVC `/analyze`、golden descriptor 与 CTest 5/5 通过，系统根 `interfaces/tests/run.ps1` 完整公共契约套件 148/148 通过（2026-08-28）。本 ticket 未改变公共 wire 契约，不构成生产地图精度、参数标定、执行授权、ROS 2 或外部验证声明。

Ticket 18 Evidence: `src/scout_core/include/scout_planner/core/coordination_evaluator.hpp` 与 `src/scout_core/src/coordination_evaluator.cpp` 发布纯 C++ 双机器人协调评价 seam。评价器按同一 `mission_enu` 同步时间轴消费主机连续闭区间扫掠球和 Scout 时序采样，使用相对线性运动二次最小值连续检查分离边界，并检查通信距离、最早失败时间和最小 margin；严格拒绝任务/预测/时钟域错配、过期接收、未同步或超不确定度、预测间隙/终止越界、不可行边界和未知/无模型链路保证。几何距离模式不会声明链路质量，标定模式仅在携带 profile 时声明。`src/scout_core/test/test_coordination_evaluator.cpp` 覆盖交叉、平行安全、预测边界、过期和确定性失败时间；`tools/verify.ps1 -Preset verify` 下配置、warnings-as-errors、MSVC `/analyze`、golden descriptor 与 CTest 6/6 通过（2026-08-29）。本 ticket 未修改公共 wire 契约，不构成链路质量标定、生产安全或执行授权证据。

Ticket 19 Evidence: `src/scout_core/include/scout_planner/core/survey_action.hpp` 与 `src/scout_core/src/survey_action.cpp` 发布 ROS 无关的 `SurveyActionPlanner`。实现将补探任务固定拆为 approach/observe/exit 三段，要求导航解有效、传感器几何已生产批准且健康为 NOMINAL、进入和退出 supercover 全部 known-free；依据固定外参、FOV、量程、位姿/量程误差和地图遮挡，对 required region 的确定性体素中心采样计算保守覆盖率，并对 mandatory 子体积执行完整覆盖门禁。报告包含观测姿态、驻留约束、进入/退出目标、覆盖计数、几何/健康版本；结构化失败码覆盖区域、几何/健康、进入/退出及覆盖不可行。`test/test_survey_action.cpp` 覆盖动作阶段顺序和结构化失败值对象；`tools/verify.ps1 -Preset verify` 配置、MSVC `/W4 /WX`、`/analyze`、golden descriptor 与 CTest 7/7 通过（2026-08-29）。本 ticket 未修改公共 wire 契约，不产生 SurveyCompletionEvidence、观测 ID、地图更新或执行授权证据。

Ticket 20 Evidence: `src/scout_core/include/scout_planner/core/state_lattice_astar.hpp` 与 `src/scout_core/src/state_lattice_astar.cpp` 发布 ROS 无关的 `TimeAwareStateLatticeAStar3d`。状态由三维体素、离散航向和动作模式组成；26 邻域运动原语按连续 `HybridMapQuery::query_supercover` 检查，UNKNOWN/STALE/CONFLICTED、占据、净空和允许水体限制均失败关闭。距离与时间构成非负代价，目标 AABB 欧氏距离/最大速度构成启发；节点、open queue、内存、注入单调 deadline 与取消均具有限制和结构化结果。`test/test_state_lattice_astar.cpp` 覆盖确定性路径、起点目标、预算耗尽与取消；`tools/verify.ps1 -Preset verify` 下 CTest 8/8 通过（2026-08-29）。本 ticket 只产生几何搜索种子，不替代平滑、能力/能源、协调/安全验证或执行授权。

Ticket 21 Evidence: `StateLatticeState` 增加有界 Scout-local 到达时间标签，`StateLatticeSearchConfig` 提供时间量化、有限标签预算、可等待转换、连续闭区间移动主机占据/分离/通信约束和能源下界预算。每条运动与动作原语在起点/中点/终点采样检查动态约束，预测区间必须从零连续覆盖；能源剪枝保留返航与 reserve 下界，最终结果仍只是规划搜索种子。观测终点可生成非空 approach/observe/exit `StateLatticeActionSeed`，可选 exit region 会在 observe 后继续搜索至退出区域。`test/test_state_lattice_astar.cpp` 新增时间标签、协同边界、能源不足和完整动作种子回归；`tools/verify.ps1 -Preset verify` 下配置、warnings-as-errors、MSVC `/analyze`、golden descriptor 与 CTest 8/8 通过（2026-08-29）。本 ticket 未修改公共 wire 契约，不构成平滑、连续验证、生产能力/能源/通信标定、执行授权或实机证据。

Ticket 22 Evidence: `src/scout_core/include/scout_planner/core/quintic_bezier.hpp` and `src/scout_core/src/quintic_bezier.cpp` provide immutable validated quintic position/yaw segments, nanosecond-contiguous timing, analytic position/velocity/acceleration/jerk and yaw derivatives, derivative control polygons with convex-hull bounds, C2 seam validation, and deterministic SHA-256 content identity. `test/test_quintic_bezier.cpp` covers linear and non-zero-boundary derivatives, normalized yaw, segment boundaries, deterministic identity, invalid duration/frame and non-contiguous input. `tools/verify.ps1 -Preset verify` passed MSVC strict warnings, `/analyze`, golden schema and CTest 9/9 (2026-08-29). This ticket does not implement smoothing, safety validation, execution authorization, or production parameter calibration.

Ticket 23 Evidence: `src/scout_core/include/scout_planner/core/trajectory_smoother.hpp` and `src/scout_core/src/trajectory_smoother.cpp` add a deterministic ROS-independent `TrajectorySmoother` seam. It constructs positive ESDF-clearance FeasibleTube balls with adjacent-overlap gates, converts search seeds to canonical timed quintic position/yaw segments, and hard-rejects invalid boundary states, tube-escaping control points, non-positive/non-contiguous timing, C2 discontinuity, non-finite values, or convex-hull speed/acceleration/yaw violations. Deterministic recovery order is offset shrink, point densification, then duration extension; collision margins are never relaxed. `test/test_trajectory_smoother.cpp` covers straight/3D paths, non-zero boundary velocity, unknown map, dynamic infeasibility, and malformed inputs. `tools/verify.ps1 -Preset verify` passed strict MSVC warnings, `/analyze`, golden schema and CTest 10/10 (2026-08-29). This ticket does not implement capability calibration, continuous collision validation, execution authorization, ROS 2, SIL/HIL, or production readiness.

Ticket 24 Evidence: `src/scout_core/include/scout_planner/core/survey_plan_evidence.hpp` and `src/scout_core/src/survey_plan_evidence.cpp` add `SurveyPlanEvidenceEvaluator`, independently sampling the final immutable `BezierTrajectory4d` against the captured map and nominal sensor geometry/health. It recomputes conservative FOV, range, known-free occlusion, coverage ratio, predicted covered AABB, endpoint/dwell gates, and mandatory-region coverage; the result binds mission, baseline map, trajectory, sensor geometry/health, planner configuration, and evaluator configuration identities and permanently marks `completion_evidence=false`. It never emits observation IDs or resulting-map completion claims. `test/test_survey_plan_evidence.cpp` covers the non-completion value contract, and `test/test_planning_context.cpp` covers invalid sampling configuration fail-closed behavior. `tools/verify.ps1 -Preset verify` passed strict MSVC warnings, `/analyze`, golden schema and CTest 11/11 (2026-08-29). Dedicated cache/smoothing/occlusion/version/determinism scenario regressions remain a follow-up test gap. This ticket does not implement execution-time `SurveyCompletionEvidence`, observation ingestion, map advancement, execution authorization, ROS 2, or production readiness.

Ticket 25 Evidence: `src/scout_core/include/scout_planner/core/continuous_geometry_validator.hpp` and `src/scout_core/src/continuous_geometry_validator.cpp` add the independent `ContinuousGeometryValidator`. It checks analytic derivative-control-polygon bounds for speed, acceleration, yaw rate and yaw acceleration, then validates each quintic segment with a control-polygon fast path and adaptive chord/supercover proof using the `(Delta t)^2 * a_max / 8` curve-deviation bound. Occupied, restricted-water, negative-clearance and boundary failures reject; unknown or unproven minimum intervals return `OUTCOME_VALIDATION_INCONCLUSIVE` rather than passing sparse samples. Reports retain trajectory/map content identities, minimum clearance, earliest failure offset, refinement depth and checked interval count. `test/test_continuous_geometry_validator.cpp` covers clear geometry, derivative limits, unknown occupancy and occupied geometry; the occupied case compares against a 2049-sample independent oracle with free endpoints and an interior obstacle. `tools/verify.ps1 -Preset verify` passed strict MSVC warnings, `/analyze`, golden schema and CTest 12/12 (2026-08-29). The design tracking addition changes `SCOUT_MOTION_ALGORITHM_DESIGN_V2.2.md` SHA-256 to `4778875ef1242ffa52f3b847e0ed01138e12019f893d5f443974ddfd5a5697ec`. This ticket does not implement unified capability/energy/coordination safety reporting, execution authorization, ROS 2, or production readiness.

## 6. 设计章节追踪

| 前导设计范围 | Tickets |
|---|---|
| 0-3 范围、输入输出、坐标时间 | 01, 04, 12, 14, 15 |
| 4 混合三维地图 | 03, 16, 20, 25 |
| 5-6 能力、海流、能源 | 05, 06, 17, 21, 26 |
| 7 双机器人协调 | 02, 07, 18, 21, 26 |
| 8 探测任务与覆盖 | 02, 05, 19, 24, 26 |
| 9 搜索 | 01, 20, 21 |
| 10-11 平滑与时序 | 22, 23, 25 |
| 12 最小风险动作 | 06, 10, 30 |
| 13-14 验证与候选排序 | 08, 25-27 |
| 15-17 授权、复用、状态机 | 09-11, 28-31 |
| 18-21 主算法、契约、架构、并发 | 08-15, 27-33 |
| 22-24 参数、实时性、降级 | 06, 11, 17, 21, 29-31 |
| 25-26 测试与指标 | 13, 32-36 |
| 27-31 实施路线、范围与完成定义 | 全部 |

## 7. 外部阻塞与首版限制

- 生产 Timing/Profile、能力、制动、海流、能源、传感器和通信参数仍需外部标定。
- FCU/MAVLink 最终控制模式和坐标转换需 HIL 证据。
- DAVE/Gazebo、水池和海试不属于纯算法仓库可独立完成的条件。
- 首版不声称概率安全、完整 6DOF 在线证明、高保真水动力或复杂链路质量保证。
- 前导完成证据只证明指定区域按契约完成观测和地图更新，不证明该区域一定可以铺缆。

# 前导状态、代码与配置契约 v1

本文是 ticket 11 的规范契约，约束前导任务、前导运动授权、逐通道通信和最小风险动作。公共 schema、`registry/codes-v1.json`、`registry/scout-state-transitions-v1.json` 与激活中的 profile 共同构成执行语义；自由文本只能用于解释，不能驱动状态、授权或安全动作。

## 1. 正交状态与唯一发布者

- `SCOUT_MISSION` 只描述精确补探任务的生命周期，由 Scout runtime 发布。
- `SCOUT_EXECUTION_AUTHORITY` 只描述前导 FCU 运动授权，由 Scout motion ExecutionAuthority 发布；它不能签发、撤销或恢复主机铺缆授权。
- `MAIN_SCOUT_COMMUNICATION` 与 `SCOUT_FCU_COMMUNICATION` 都使用通信状态枚举，但以不同 `ChannelId`、时序和 LossPolicy 独立演进。
- 风险动作不是新授权域。任务进入 `RISK_ACTION` 前必须先撤销任务 lease，由独立安全监督器按照已激活规则选择动作；风险动作不能产生新探测授权。

每个转换必须精确匹配 `scout-state-transitions/v1` 的 domain、previous state、next state 和 `CodeRef trigger`，并携带注册表要求的证据。一次转换只改变一个 domain。合法转换生成 `AUDIT_STATE_TRANSITION`；an illegal transition 必须整体拒绝并生成 `AUDIT_ILLEGAL_TRANSITION_REJECTED`，不能强制映射到相邻状态。

通信恢复、条件已消失、fault clear 或急停复位都不能直接把前导授权从 `REVOKED` 改成 `AUTHORIZED`。`RESYNCHRONIZING -> NORMAL` 也只恢复链路状态。

## 2. 稳定代码与未知值

所有 Outcome、Fault、Diagnostic 与 transition trigger 都引用 `underwater-system-codes` 的精确版本。Fault 的严重度、安全效果、锁存方式、清除权威和恢复链由本地同版本代码注册表决定，发送方不得在消息里重新解释。Diagnostic 只报告事实或拒绝原因，永不授予执行权。

接收端遇到 unknown safety-critical enum、未知稳定代码、未知 wire field、代码注册表 hash 不一致或 trigger 与转换表不一致时必须失败关闭：拒绝原消息，保持原状态和授权水位不变，并写入包含原始数值、stream、Outcome、Diagnostic、correlation 和 cause 的 `SafetyAuditEvent`。

## 3. 不可变配置与激活

`ScoutConfigurationProfile` 原子引用：

- TimingProfile、InterfaceLimits、code registry 和 state transition registry；
- 能力 profile、能源模型和一个或多个传感器几何 profile；
- planner 与 SafetyGate 配置；
- `CHANNEL_MAIN_SCOUT_COOP` 和 `CHANNEL_SCOUT_NUC_FCU` 各自的 LossPolicy；
- 所有允许的风险动作规则。

每个引用必须具有非空 ID、正版本和 32-byte SHA-256 identity。接收端必须在 prepare 阶段重新计算并比对内容身份，在 activate 阶段原子切换整套快照并创建新会话；不能部分沿用旧引用，也不能用 schema 默认值补齐安全字段。

生产激活要求 profile 本身及所有能力、能源、传感器、通信和安全依赖都带有适用设备、数据集、运行域、日期和版本证据。任何 missing production parameter 或 incompatible profile 都必须拒绝且审计。`integration/v1` 明确是 NON_PRODUCTION，只可用于联调，不可被生产消费者武装。

## 4. 超时顺序与逐链路 LossPolicy

前导 FCU 链路满足：heartbeat publish period < degraded timeout < lost timeout <= software revoke timeout < FCU command watchdog <= Scout hard-safety timeout。反馈发布、stale warning、software revoke、lease renewal 和 lease expiry 仍按同一激活 TimingProfile 比较，且只使用 Scout NUC 本地单调时钟。

主机—前导协同链路允许在 DEGRADED 时完成已授权且仍安全的短前缀，但禁止新授权；LOST 必须撤销并进入配置的 RETURN 风险动作。Scout NUC—FCU 链路在 DEGRADED 即撤销并保护停车，LOST 使用本地 BRAKE。两条链路不得共享状态、水位或恢复结果。

链路恢复必须经过新会话、兼容 Manifest/profile、post-boundary 水位和新鲜输入证据；`recovery.restores_authorization` 固定为 false。

## 5. 故障恢复链

条件已消失只是触发条件当前不再成立，不等于故障已显式清除。恢复必须按单向顺序完成：

1. 由注册的清除权威显式清除 fault；若有急停，先完成受认证本地复位。
2. 执行本地自检并验证硬件、FCU 模式、能力、能源、传感器、地图及协调证据。
3. 激活兼容契约与完整配置，建立新会话并丢弃旧水位。
4. 重新规划并由独立安全验证器复检。
5. 签发严格更新的新授权执行包和 lease。

任何一步都不能复活旧 Bundle、旧 lease 或旧 FCU command sequence。审计排队或持久化故障不得阻塞撤销、保护停车或急停；丢失审计记录必须以稳定 Fault/Diagnostic 显式暴露。

## 6. 资源与兼容门禁

InterfaceLimits 分别限制 StateTransition、ScoutStatus、FaultReport、DiagnosticEvent、SafetyAuditEvent、ScoutConfigurationProfile 和 LossPolicy 的完整序列化大小。超限对象整体拒绝，不得截断诊断、状态证据或配置引用后继续。Manifest 必须精确声明 `scout_state_codes_profiles_v1`、state registry identity、code registry identity 和完整 Scout configuration identity；未列出的混合版本失败关闭。

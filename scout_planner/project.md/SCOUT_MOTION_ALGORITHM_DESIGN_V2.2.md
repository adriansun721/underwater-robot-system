# 面向异构双机器人协同铺缆任务的前导机器人运动规划算法

> **版本**：V2.2 Contract-Aligned Engineering Baseline  
> **发布日期**：2026-08-28  
> **定位**：功能完整、工程简化、数学闭环的前导机器人实现基线  
> **适用对象**：8 推进器水下前导探测机器人  
> **协同对象**：水下铺缆机器人 / 动力缆舱及其铺缆规划器  
> **替代关系**：继承 V2.1 的简化算法主体，并以公共 v1 契约闭合跨进程、跨 NUC 与执行边界  
> **核心原则**：功能保留、机制收敛、统一验证、短时授权、失败安全  
> **规范优先级**：跨边界 wire 语义以 `interfaces/proto/underwater/contracts/v1/`、稳定注册表、激活 profile、`interfaces/HASHING.md` 与兼容 Manifest 为权威；本文中的 C++ 与伪代码只描述无损内部适配和算法行为，不得扩展公共语义。  
> **重要声明**：本文是算法与软件实施基线，不是生产就绪声明。DAVE/Gazebo 双机联调、HIL、目标硬件与控制器验证、水池、海试及生产参数标定均未完成。

---

## 0. 执行摘要

### 0.1 设计目标

前导机器人不是单纯的三维避障 AUV，而是铺缆任务中的主动探测与局部协同机器人。其核心职责是：

1. 在混合三维地图中生成可跟踪的安全轨迹；
2. 在铺缆机器人到达信息缺口前完成补探；
3. 保持与铺缆机器人的安全分离和通信约束；
4. 在海流、推进器能力、能源和传感器运行范围内运动；
5. 生成覆盖指定认证体积的探测计划与执行后证据；
6. 对轨迹进行独立安全复检；
7. 只执行获得短时授权的轨迹前缀；
8. 发生地图、定位、通信、推进器或能源异常时及时撤销授权并进入最小风险处置。

V2.2 不追求在线安全认证级集合证明，而追求：

> **能够实现、能够实时运行、能够仿真和实机验证、能够逐步增强。**

### 0.2 从 V2.1 继承的简化

V2.2 保留 V2.1 已从 V2.0 收敛的任务功能与实现层次，并以公共契约替换旧的跨边界草图：

| V2.0 机制 | V2.2 处理 |
|---|---|
| L0/L1/L2 多级保证 | 合并为候选 / 可执行 / 降级三类运行语义 |
| 时空多标签多拓扑搜索 | 默认单主搜索 + 有界备选重搜 |
| 独立安全走廊子系统 | 收敛为平滑器内部的局部可行管 |
| 强制信赖域 SCP | 改为确定性约束平滑；SCP 作为可选增强后端 |
| 在线完整 6DOF 区间动力学证明 | 改为标定后的能力包络 + 推进器分配检查 |
| L2 恢复策略库与流管覆盖 | 改为独立最小风险监督器 + 在线动作可执行性检查 |
| 区间算术连续证明 | 改为解析导数界 + 自适应扫掠验证 |
| 大量独立证据层 | 合并为统一 `PlanValidationReport` |
| 复杂候选词典序 | 改为硬约束先过滤、有限软目标排序 |

### 0.3 主算法链

最终主链固定为：

$$
\boxed{
\text{同步上下文}
\rightarrow
\text{混合3D地图}
\rightarrow
\text{补探任务}
\rightarrow
\text{时间感知3D搜索}
\rightarrow
\text{约束轨迹平滑}
\rightarrow
\text{时序与能力检查}
\rightarrow
\text{统一安全验证}
\rightarrow
\text{短时执行授权}
\rightarrow
\text{在线监控与重规划}
}
$$

探测闭环为：

$$
\boxed{
\text{探测请求}
\rightarrow
\text{探测轨迹}
\rightarrow
\text{实际观测}
\rightarrow
\text{地图更新}
\rightarrow
\text{完成证据}
\rightarrow
\text{铺缆侧重新规划}
}
$$

### 0.4 不允许退化的核心原则

以下原则不能为了简化而删除：

- UNKNOWN / STALE / CONFLICTED 不能直接作为已授权自由空间；
- 优化成功不等于轨迹可执行；
- 规划器不能直接向推进器写控制指令；
- 轨迹、上下文、依赖版本和授权必须可追踪；
- 地图、定位、健康状态等关键依赖变化后必须复检；
- 超时不能把“当前最好候选”冒充成功；
- 探测“到达目标点”不能冒充探测完成；
- 铺缆可行性仍由铺缆规划器独立判断。

---

# 1. 系统范围与任务契约

## 1.1 前导机器人负责

前导机器人负责：

- 接收或选择补探任务；
- 规划进入、扫描/驻留、退出的完整动作；
- 生成自身三维运动轨迹；
- 根据固定传感器几何评价可见性和覆盖率；
- 维持与铺缆机器人的距离和通信约束；
- 检查海流、速度、加速度、姿态、推进器、能源可执行性；
- 发布计划探测证据；
- 执行后发布探测完成所需的观测关联信息；
- 在异常时停止继续向外探索并进入最小风险处置。

## 1.2 铺缆机器人负责

铺缆机器人负责：

- 维护全局铺缆参考路线；
- 判断哪里存在地图或探测缺口；
- 发布公共 `ScoutMission`，并负责其取消与完成证据确认生命周期；
- 发布自身预测占据区域；
- 独立判断前导返回的地图证据是否足以支持铺缆；
- 独立执行履带通行、触地点、张力、曲率、悬空等铺缆约束验证。

前导机器人不输出“该区域一定可以铺缆”的结论。

## 1.3 本文不负责

本文不实现：

- 原始图像、声呐点云重建算法；
- SLAM / 定位滤波器内部算法；
- AUV 底层姿态控制器；
- 推进器驱动器；
- 铺缆机器人主规划器；
- 高保真 CFD 或完整水动力流固耦合。

但本文定义它们必须向规划器提供的稳定接口。

---

# 2. 输入、输出与同步上下文

## 2.1 一次规划的不可变上下文

一次求解只使用一个不可变同步上下文：

$$
\mathcal C_k =
\{
X_k,
M_k,
C_k,
G_k,
K_k,
E_k,
R_k,
P_{M,k},
L_k,
\Theta_k
\}.
$$

其中：

- $X_k$：前导机器人当前状态及不确定性；
- $M_k$：混合三维地图快照；
- $C_k$：海流估计及误差界；
- $G_k$：声呐/摄像机固定安装几何与健康状态；
- $K_k$：车辆运动能力包络；
- $E_k$：能源状态；
- $R_k$：补探请求；
- $P_{M,k}$：铺缆机器人预测占据；
- $L_k$：双机协调条件；
- $\Theta_k$：参数和版本集合。

规划开始后不得在求解内部调用 `latestMap()`、`latestPose()` 等更新输入。

## 2.2 时间同步

安全 deadline、新鲜度与水位只在接收方本地时钟域判断。对在 Scout NUC
接收的每个输入 $z_i$，记录本地接收时刻 $r_i$，并在同一 Scout NUC
时钟域内定义：

$$
a_i = t_{\mathrm{capture}}^{\mathrm{scout}} - r_i
$$

为数据年龄。

必须满足：

$$
0 \le a_i \le a_i^{\max},
$$

只有带 `SynchronizedObservationTime` 且同步有效的观测，才允许用同步观测时间
检查跨设备对齐，并要求关键观测之间：

$$
|t_i^{\mathrm{sync}}-t_j^{\mathrm{sync}}|
\le \Delta t_{\mathrm{sync}}^{\max}.
$$

主机发送的 `ScoutMission.business_deadline_monotonic_ns` 属于主机时钟域，Scout
不得与本地 `now` 比较；本地接收与 admission 有效窗由 `ScoutMissionDecision`
在 Scout 时钟域返回。规划 deadline、lease、执行历元、反馈新鲜度和软件撤销
deadline 全部使用 Scout NUC 本地单调时钟；FCU watchdog 使用 FCU 本地接收 tick。

任一硬依赖超龄或同步观测错配时，规划结果返回稳定
`OUTCOME_DEPENDENCY_STALE`。时钟域错误在消息/ContextBuilder 门禁以
`OUTCOME_CLOCK_DOMAIN_MISMATCH` 整体拒绝，不能形成一个看似正常启动过的规划
结果。两者都不得继续使用部分新数据和部分旧数据拼接计划。

## 2.3 输出

规划器输出：

$$
Y =
\{
\tau,
V,
A,
\mathcal E_{\mathrm{survey}},
\mathcal R_{\mathrm{validation}},
D
\}.
$$

其中：

- $\tau$：规范四维轨迹 $(x,y,z,\psi)$；
- $V$：速度和角速度参考；
- $A$：加速度参考；
- $\mathcal E_{\mathrm{survey}}$：规划阶段探测覆盖预测；
- $\mathcal R_{\mathrm{validation}}$：统一安全验证结果；
- $D$：诊断、失败原因、版本和统计信息。

只有 `OUTCOME_SUCCESS` 可以携带可申请授权的候选轨迹。

### 2.3.1 公共四维计划结果契约追踪

Ticket 08 将上述输出固化为
`interfaces/proto/underwater/contracts/v1/planning.proto` 中独立的
`ScoutPlanningResult` 与 `ScoutPlan`，不复用铺缆侧二维
`ImmutablePlan`。规范轨迹以 `mission_enu` 三维五次 Bézier 控制点、规范化
初始 ENU yaw、连续不折返的 yaw offset、段起始偏移和正时长唯一重建；计划
原子绑定 Tickets 02-07 的完整依赖内容身份、`SurveyPlanEvidence` 和独立
`ScoutPlanValidationReport`。详细消费门禁、hash、结果优先级、资源上限及
C++/ROS 2 adapter 要求见 `interfaces/SCOUT_4D_PLANNING_RESULT.md` 与
`interfaces/HASHING.md`。该候选没有执行权；Ticket 09 已通过独立
`ScoutAuthorizedExecutionBundle`、`ScoutExecutionLease` 与 `ScoutBundleAck`
定义前导专用授权面，具体门禁见 `interfaces/SCOUT_AUTHORIZATION_BUNDLE.md`。
`integration/v1` 仍为非生产联调配置，契约测试不能证明算法、
仿真、实机能力或安全认证。

---

# 3. 坐标、时间与数值约定

## 3.1 坐标系

- 世界系：`mission_enu`，右手 ENU；
- 机器人系：`base_link`，右手前—左—上（FLU）；
- 地图进入规划器前统一到世界系；
- 传感器视场在传感器系定义，通过版本化外参转换到世界系。

## 3.2 航向

内部使用展开航向：

$$
\tilde\psi \in \mathbb R.
$$

公共瞬时 ENU yaw 发布时规范化到 $[-\pi,\pi)$。规范轨迹发布一个
`initial_yaw_rad`，并以相对它、跨段不重置且不在 $\pm\pi$ 归一化的
`yaw_offset_control_points_rad` 唯一重建内部展开航向。

## 3.3 单调时间

轨迹执行全部使用单调时钟。

ROS 时间、墙上时间只用于：

- 日志；
- 仿真同步；
- 回放显示。

ROS 时间跳变、仿真 reset 或 replay seek 必须重新创建规划生命周期。

## 3.4 数值约束

所有公共输入必须满足：

- 浮点有限；
- 单位 SI；
- 协方差对称半正定；
- 段时长严格为正；
- 版本字段存在；
- frame 明确；
- 数组维度正确。

任何 NaN、Inf、非法四元数、负时间、逆序上下界等直接返回
`OUTCOME_INPUT_INVALID`。

---

# 4. 混合三维地图

## 4.1 地图组成

前导规划使用：

$$
M =
\{
H_{\mathrm{floor}},
V_{\mathrm{occ}},
D_{\mathrm{esdf}},
W_{\mathrm{allowed}},
S_{\mathrm{semantic}}
\}.
$$

其中：

- $H_{\mathrm{floor}}(x,y)$：海床高程及质量；
- $V_{\mathrm{occ}}(x,y,z)$：三维占据栅格；
- $D_{\mathrm{esdf}}(p)$：距最近障碍物的有符号/非负距离；
- $W_{\mathrm{allowed}}$：机器人允许进入水体；
- $S_{\mathrm{semantic}}$：禁入区、通信阴影、特殊区域。

2.5D 海床层不能推断悬垂物不存在。

## 4.2 体素状态

体素统一为：

- `FREE`
- `OCCUPIED`
- `UNKNOWN`
- `STALE`
- `CONFLICTED`

执行路径只能进入当前有效的 `FREE` 区域。

`UNKNOWN` 可作为未来探测目标，但不能作为当前授权路径的自由空间。

## 4.3 保守障碍

机器人几何安全裕量定义为：

$$
r_{\mathrm{safe}}
=
r_{\mathrm{body}}
+
r_{\mathrm{loc}}
+
r_{\mathrm{track}}
+
r_{\mathrm{map}}
+
r_{\mathrm{disc}}.
$$

若使用 ESDF，则名义轨迹中心需满足：

$$
D_{\mathrm{esdf}}(p(t))
\ge
r_{\mathrm{safe}}.
$$

当机器人必须考虑完整姿态外形时，用姿态相关外接体：

$$
r_{\mathrm{body}}=r_{\mathrm{body}}(\phi,\theta).
$$

首版可按标定运行范围取其最大值：

$$
r_{\mathrm{body}}^{\max}
=
\max_{(\phi,\theta)\in\Omega_{\mathrm{att}}}
r_{\mathrm{body}}(\phi,\theta).
$$

这样避免在线进行复杂旋转多面体集合运算。

## 4.4 地图版本依赖

计划必须记录：

- `map_version`
- `map_content_hash`
- `map_timestamp`
- `map_region_bbox`
- `sensor_extrinsic_version`
- `mapping_parameter_version`

地图更新时只对与授权轨迹相关的区域触发强制复检。

## 4.5 公共契约追踪

Ticket 03 将本章输入 seam 固化为公共
`interfaces/proto/underwater/contracts/v1/mapping.proto` 中的
`HybridMapSnapshot`，并由 `interfaces/HYBRID_MAP_SNAPSHOT.md` 规范五层载荷、
ENU 网格、版本/内容身份、时钟域、外参/建图参数依赖、分片重组和失败关闭
规则。后续 Ticket 14 只能做无损双向适配，Ticket 16 才实现地图查询；本章
不把 schema 可解析、分片传输成功或测试 profile 视为地图安全性或生产能力
证明。

## 4.6 查询实现追踪

Ticket 16 以 ROS 无关的 `HybridMapQuery` 作为本章唯一算法查询 seam。实例只从
Ticket 14 已验证且与规划依赖的 `map_id`、`map_version`、
`map_content_identity` 精确一致的不可变 `HybridMapSnapshot` 构造；点查询同时
组合海床、三维占据、ESDF、允许水体、地图边界和连续语义 AABB，并返回有效
体素状态、包含边界的保守净空、海床质量、来源版本/时间/区域、语义限制和带
ENU 位置的信息缺口。
线段查询使用 X-fastest 稳定顺序的确定性三维 supercover，包含仅接触网格面、
棱和角的体素，并对连续语义区域执行线段/AABB 相交，不能用端点或体素中心
抽样跳过薄障碍。机体、定位、跟踪、地图和离散误差保持为五个有限非负 SI
输入后再求和；该总和除约束 ESDF、海床和地图边界外，还保守膨胀允许水体
体素与连续语义 AABB。地图外、边界净空不足、UNKNOWN、STALE、CONFLICTED、缺块、
非有限 ESDF、未知语义及版本/identity 错配均在适配或查询边界失败关闭。该
seam 只提供规划事实，不授予执行权，也不构成生产地图精度或参数标定证据。

---

# 5. 机器人状态与运动能力模型

## 5.1 名义规划状态

规划状态为：

$$
q =
[x,y,z,\tilde\psi]^T.
$$

一阶导数：

$$
\dot q =
[\dot x,\dot y,\dot z,\dot{\tilde\psi}]^T.
$$

二阶导数：

$$
\ddot q =
[\ddot x,\ddot y,\ddot z,\ddot{\tilde\psi}]^T.
$$

## 5.2 实际状态输入

实际状态至少包含：

$$
X =
\{
p,
\Theta,
\nu,
\Sigma_p,
\Sigma_\Theta,
h
\}.
$$

其中：

- $p$：位置；
- $\Theta=(\phi,\theta,\psi)$：姿态；
- $\nu=(u,v,w,p_r,q_r,r)$：体速度；
- $\Sigma_p,\Sigma_\Theta$：状态不确定性；
- $h$：推进器健康状态。

### 5.2.1 公共导航契约追踪

Ticket 04 将本节定位输入 seam 固化为公共
`interfaces/proto/underwater/contracts/v1/state.proto` 中的
`ScoutNavigationState`。该快照由定位权威发布，原子绑定三维 ENU 位置、
从 `base_link`/FLU 到 `mission_enu` 的单位四元数、FLU 体线/角速度、ENU
位置协方差、FLU 小角度姿态协方差、导航有效状态、业务版本、源会话、
源时钟域、观测时间、时序 profile 与内容身份。规范消费门禁、版本水位、
失败关闭及 FCU 边界的 ENU/NED、FLU/FRD 双向 golden vectors 见
`interfaces/SCOUT_NAVIGATION_STATE.md`；`ScoutStatus` 只保留任务级摘要，
不能替代该定位事实。后续 Ticket 14 只能进行无损双向适配，Ticket 15
才负责与其他依赖捕获原子一致上下文；本节不把 schema 往返或联调时序
profile 视为定位精度、生产新鲜度或实机坐标转换证明。

规划器仍然只优化 $(x,y,z,\psi)$，滚转和俯仰由底层控制器保持在批准范围：

$$
|\phi| \le \phi_{\max},
\qquad
|\theta| \le \theta_{\max}.
$$

## 5.3 海流

海流估计为：

$$
c^W(p,t)=\hat c^W(p,t)+e_c,
\qquad
\|e_c\|\le \rho_c.
$$

名义对水速度：

$$
v_{\mathrm{water}}
=
R_z^T(\tilde\psi)
\left(
\dot p-\hat c^W
\right).
$$

为了避免在线复杂鲁棒动力学传播，能力验证使用保守速度裕量：

$$
\|v_{\mathrm{water}}\|
+\rho_{v,c}
\le
v_{\mathrm{water}}^{\max}.
$$

其中 $\rho_{v,c}$ 由海流误差和定位/跟踪误差标定得到。

### 5.3.1 公共传感器与海流契约追踪

Ticket 05 将海流输入 seam 固化为公共
`interfaces/proto/underwater/contracts/v1/sensing.proto` 中的
`ScoutCurrentEstimate`。该快照在 `mission_enu` 中原子绑定局部仿射海流
模型、严格三维适用域、同一源时钟域的观测/有效区间、完整分量误差界与
范数误差界、可选空间梯度、运行域、模型版本/来源及内容身份。缺少梯度时
只能在相同完整误差界内使用常值模型，不能由 adapter 合成零梯度；越域、
超龄、frame 错配、误差界不完整或非生产 profile 均失败关闭。规范消费门禁
见 `interfaces/SCOUT_SENSOR_AND_CURRENT.md`。后续 Ticket 14 只允许无损双向
适配，Ticket 15 捕获其精确版本与内容身份，Tickets 17、21 和 26 才分别
执行能力门禁、时间感知搜索和统一安全复检；本节不把 integration profile
视为实机海流范围或生产标定证据。

## 5.4 能力包络

V2.2 采用 `VehicleCapabilityEnvelope` 代替在线完整 6DOF 集合证明。

能力包络定义：

$$
\mathcal K(h)=
\{
v_{\max},
a_{\max},
r_{\max},
\dot r_{\max},
v_{z,\max},
a_{z,\max},
\phi_{\max},
\theta_{\max},
\tau_{\mathrm{margin}}
\}.
$$

轨迹必须满足：

$$
\|\dot p\| \le v_{\max},
$$

$$
\|\ddot p\| \le a_{\max},
$$

$$
|\dot{\tilde\psi}| \le r_{\max},
$$

$$
|\ddot{\tilde\psi}| \le \dot r_{\max}.
$$

若推进器分配器提供在线力/力矩检查接口，还要求：

$$
\tau_{\mathrm{req}}(t)\in
\mathcal W_{\mathrm{thr}}(h)
\ominus
\mathcal B_{\rho_\tau}.
$$

没有在线分配检查时，轨迹必须落在已经通过 SIL/HIL/水池标定的能力包络内。

超出标定包络直接返回：

`CAPABILITY_INFEASIBLE`

不得外推。

## 5.5 推进器健康

推进器健康状态按离散 profile 管理：

- `NOMINAL`
- `DEGRADED_A`
- `DEGRADED_B`
- ...

每个 profile 有独立能力包络。

健康组合发生变化时：

1. 当前候选失效；
2. 当前执行授权立即进入复检；
3. 若新能力不足，撤销授权；
4. 安全监督器选择可执行的最小风险动作。

---

# 6. 能源模型

## 6.1 简化功率模型

首版采用：

$$
P(t)
=
P_{\mathrm{hotel}}
+
P_{\mathrm{motion}}(v_{\mathrm{water}},a,\dot\psi,h).
$$

若有推进器反馈，可进一步使用：

$$
P_{\mathrm{motion}}
=
\sum_i \hat P_i(f_i).
$$

## 6.2 计划能量

轨迹计划能量：

$$
E_{\mathrm{plan}}
=
\int_0^{T}
P(t)\,dt.
$$

保守执行条件：

$$
E_{\mathrm{available}}
\ge
E_{\mathrm{plan}}
+
E_{\mathrm{return}}
+
E_{\mathrm{reserve}}.
$$

若当前任务不要求返航，则至少保留：

$$
E_{\mathrm{available}}
\ge
E_{\mathrm{plan}}
+
E_{\mathrm{risk\_action}}
+
E_{\mathrm{reserve}}.
$$

能源不足返回：

`ENERGY_INSUFFICIENT`

能源不是低优先级软代价。

## 6.3 公共能力与能源契约追踪

Ticket 06 将第 5-6 章输入 seam 固化为公共
`interfaces/proto/underwater/contracts/v1/capability.proto` 中彼此独立的
`ScoutCapabilityProfile`、`ScoutThrusterHealthState`、
`ScoutEnergyModelProfile` 与 `ScoutEnergyState`。能力 profile 按精确 nominal
或退化健康组合绑定水相对速度、平移/垂向能力、偏航导数、姿态、推进余量、
主动制动和运行域；健康变化必须引用新的精确 profile 并使既有候选与授权进入
复检，adapter 不得缩放 nominal profile 生成退化能力。能源模型保留独立
`energy_model_version`，能源状态显式携带可用量、reserve、返航与风险动作需求，
并按本章两个不等式失败关闭；只够到达不构成可执行能源证据。

规范消费门禁见 `interfaces/SCOUT_CAPABILITY_AND_ENERGY.md`。后续 Ticket 14
只能实现逐字段无损双向适配，Ticket 15 捕获四条流的精确版本和内容身份，
Tickets 17、21、26、28-30 才实现能力/能源判断、搜索剪枝、统一复检、授权与
最小风险动作；本节不把 `integration/v1`、schema 可解析或参考消费者通过
视为实机能力、制动、能源标定或生产安全证明。

### 6.3.1 Ticket 17 实现追踪

`CapabilityEnergyGate` 是当前 ROS 无关能力/能源判断 seam。它只消费
`ScoutPlanningContext` 的不可变快照和显式轨迹采样，按海流适用 AABB、局部
仿射梯度（若存在）及完整误差界计算对水速度（按采样 ENU 偏航变换至本体相对
速度），并在无梯度时要求调用方提供
非负、有限的已标定保守加速度/偏航误差裕量。能力、健康 profile、运行域、
当前有效性或生产批准不满足时失败关闭；能源按保守功率模型积分计划、返航或
风险动作、reserve，并返回首个失败采样与能力/能源 margin 及版本水位。该
实现不提供在线推进器分配证明，也不把联调 fixture 或当前 profile 视为生产
标定证据；测试证据记录在 `project.md/PLAN.md` Ticket 17 和对应 issue。

---

# 7. 双机器人协调

## 7.1 铺缆机器人预测

铺缆侧提供预测占据：

$$
P_M(t)
\subset \mathbb R^3.
$$

前导机器人名义位置为 $p_S(t)$。

首版使用保守包围球约束：

$$
d_{\mathrm{SM}}(t)
=
\inf_{x\in P_M(t)}
\|p_S(t)-x\|.
$$

要求：

$$
d_{\mathrm{SM}}(t)
\ge
d_{\mathrm{sep}}^{\min}
+
r_S^{\mathrm{safe}}.
$$

## 7.2 最大协同距离

为维持水下无线协同：

$$
d_{\mathrm{SM}}(t)
\le
d_{\mathrm{comm}}^{\max}.
$$

若存在标定链路质量模型：

$$
Q_{\mathrm{link}}(t)\ge Q_{\min}.
$$

否则只声明“几何通信距离约束通过”，不声明实际链路质量保证。

## 7.3 协调条件

协调信息至少包含：

```cpp
struct CoordinationConstraint {
    MissionIdentity mission;
    PredictionIdentity prediction;
    double min_separation_m;
    double max_communication_distance_m;
    MainRobotPrediction main_prediction;
    LinkAssuranceBasis link_assurance_basis;
    ProfileRef scout_loss_policy;
    ProfileRef main_loss_policy;
    SenderClockValidity source_validity;
    uint64_t coordination_version;
    ContentIdentity coordination_content_identity;
};
```

协调条件按 Scout 本地接收年龄过期后禁止开始新的向外探索。发送方 validity
只在其原时钟域内有意义，不能与 Scout 本地时钟直接比较。

## 7.4 公共预测与协调契约追踪

`interfaces/proto/underwater/contracts/v1/cooperation.proto` 中的
`MainRobotPrediction` 使用同步观测历元与连续闭区间扫掠球表达三维预测占据，
`ScoutCoordinationConstraint` 精确绑定任务与预测身份、分离边界、几何通信
边界、链路保证依据和两条方向独立的 `LossPolicy` 引用。消费、hash、时序、
版本水位及恢复规则见 `interfaces/SCOUT_MAIN_ROBOT_COORDINATION.md`；预测与协调
只提供规划事实，不向任一机器人授予执行权。

---

# 8. 补探任务与探测覆盖

## 8.1 ScoutMission 与内部 SurveyTask

铺缆侧发布的唯一公共任务身份是 `ScoutMission`。下列内部值对象只能由已通过
admission 的公共消息逐字段无损适配，不能获得独立业务版本或被重新发布为竞争事实：

```cpp
struct SurveyTask {
    uint64_t mission_id;
    uint64_t mission_version;
    ContentIdentity mission_content_identity;
    Region3dEnu required_region;
    Region3dEnu allowed_scout_region;
    double required_coverage_ratio;
    double required_resolution_m;
    Duration maximum_evidence_age;
    std::optional<SenderDomainBusinessDeadline> business_deadline;
    SurveyUrgency urgency;
    uint64_t coordination_version;
};
```

`business_deadline` 只保留发送方业务事实，不参与 Scout 本地安全计时。任务接受、
拒绝、取消/ACK、完成证据/ACK 的身份、幂等和冲突规则以
`interfaces/SCOUT_MISSION_LIFECYCLE.md` 为准。

## 8.2 探测动作

一个完整探测动作不是一个点，而是：

$$
A_{\mathrm{survey}}
=
A_{\mathrm{approach}}
\oplus
A_{\mathrm{observe}}
\oplus
A_{\mathrm{exit}}.
$$

必须保证：

- 能安全进入；
- 观测期间姿态与速度满足传感器要求；
- 可完成最低驻留/扫描时间；
- 能安全退出或停止。

## 8.3 传感器可见性

设传感器固定视场为：

$$
\operatorname{FOV}(q(t),G).
$$

考虑位姿误差、量程误差和遮挡后，保守覆盖体积：

$$
V_{\mathrm{cover}}^{-}
=
\bigcup_{t\in I_{\mathrm{obs}}}
\left[
\operatorname{FOV}(q(t),G)
\ominus
\mathcal E_{\mathrm{pose}}
\ominus
\mathcal E_{\mathrm{range}}
\right]
\setminus O_{\mathrm{occ}}.
$$

覆盖率：

$$
C_{\mathrm{survey}}
=
\frac{
\operatorname{Vol}
\left(
V_{\mathrm{required}}
\cap
V_{\mathrm{cover}}^{-}
\right)
}{
\operatorname{Vol}
\left(
V_{\mathrm{required}}
\right)
}.
$$

要求：

$$
C_{\mathrm{survey}}
\ge
C_{\min}.
$$

强制子体积必须完全覆盖时：

$$
V_{\mathrm{mandatory}}
\subseteq
V_{\mathrm{cover}}^{-}.
$$

### 8.3.1 公共传感器几何与健康追踪

Ticket 05 将观测几何和设备健康分别固化为 `ScoutSensorGeometry` 与
`ScoutSensorHealthState`。几何快照原子绑定固定 `base_link`/FLU 外参、
sensor-frame +X 视场、量程/分辨率、已知自由射线遮挡策略、运行域、设备与
标定来源；健康快照独立维护 `health_version`、同域观测/有效期和稳定故障码。
规划上下文必须同时捕获 `geometry_version` 与 `health_version` 的精确内容
身份，任一变化都触发相关计划复检，不能用健康变化静默改写量程或几何。
未标定、DEGRADED/FAILED、过期、未知枚举或非生产 profile 不得生成新的
可授权探测计划。后续 Ticket 19 才在该公共几何上实现覆盖体积与进入—驻留—
退出动作，Ticket 24 才生成计划探测证据；schema 往返本身不是覆盖率或实机
安装方向证明。

## 8.4 规划证据与完成证据

保留两个概念，但不建立复杂多级证据体系。

`SurveyPlanEvidence`：

- 说明当前计划按模型执行时预计能否完成覆盖；
- 属于规划结果的一部分。

`SurveyCompletionEvidence`：

- 只在实际执行和地图更新后生成；
- 绑定非空真实观测 ID、严格更新的地图版本/内容身份和覆盖结果；
- 只有铺缆侧接受精确 `SurveyCompletionAck` 后，任务状态所有者才能完成相应
  `SCOUT_COMPLETED` 转换。

因此：

$$
\text{PlanEvidence} \neq \text{CompletionEvidence}.
$$

“声呐打开”或“到达扫描点”不能标记任务完成。

---

# 9. 时间感知 3D state-lattice A*

## 9.1 设计选择

V2.2 默认使用一个主搜索器：

> **时间感知 3D state-lattice A\***（`TimeAwareStateLatticeAStar3d`）。

“3D”只描述占据与平移空间；搜索基础状态仍包含离散航向和动作模式。
它不是只搜索 $(x,y,z)$ 的纯 3D A*，也不是在连续状态上按车辆模型传播、带
Dubins/Reeds-Shepp analytic expansion 的经典 Hybrid A*。首版在离散状态上
使用预定义运动原语，因此 `state-lattice A*` 是唯一准确名称。

不默认维护大量多标签、多同伦候选。

若主候选在平滑或验证阶段失败，可以最多执行 $K_{\mathrm{retry}}$ 次受限备选重搜。

## 9.2 搜索状态

搜索节点为：

$$
n =
(i,j,k,b_\psi,m).
$$

其中：

- $(i,j,k)$：三维体素索引；
- $b_\psi$：离散航向；
- $m$：`CRUISE / OBSERVE / WAIT`。

节点记录：

$$
\chi(n)=
\{g,t,E,parent\}.
$$

每个基础状态默认保留一个最佳标签，降低内存和队列复杂度。

若动态双机约束导致明显的时间不可交换性，可为同一基础状态最多保留 $N_t^{\max}$ 个离散到达时间标签；默认值应很小。

## 9.3 运动原语

运动原语：

$$
u_j=
(\Delta x,\Delta y,\Delta z,\Delta\psi,\Delta t).
$$

必须满足粗粒度能力约束：

$$
\frac{\|\Delta p\|}{\Delta t}
\le v_{\max}^{\mathrm{search}},
$$

$$
\frac{|\Delta\psi|}{\Delta t}
\le r_{\max}^{\mathrm{search}}.
$$

搜索层只做保守早期拒绝，不替代最终轨迹验证。

## 9.4 边碰撞检查

每条原语使用 supercover / swept sampling 检查：

$$
D_{\mathrm{esdf}}(p_s)
\ge r_{\mathrm{search}}
\quad
\forall p_s\in\operatorname{Sweep}(u_j).
$$

`UNKNOWN / STALE / CONFLICTED` 均不可进入当前可执行搜索路径。

## 9.5 时间相关双机约束

边上的预计到达时间：

$$
t_{k+1}=t_k+\Delta t.
$$

在边采样时间上检查：

$$
d_{\mathrm{SM}}(t)
\ge d_{\mathrm{sep}}^{\min},
$$

$$
d_{\mathrm{SM}}(t)
\le d_{\mathrm{comm}}^{\max}.
$$

## 9.6 搜索能量下界

边能量下界：

$$
\Delta E_{\mathrm{lb}}
=
P_{\mathrm{lb}}(u_j)\Delta t.
$$

若：

$$
E_{\mathrm{used}}
+
\Delta E_{\mathrm{lb}}
+
E_{\mathrm{return}}^{\mathrm{lb}}
+
E_{\mathrm{reserve}}
>
E_{\mathrm{available}},
$$

直接剪枝。

## 9.7 搜索代价

只使用非负软代价：

$$
\Delta g =
w_L\frac{\Delta s}{L_0}
+
w_T\frac{\Delta t}{T_0}
+
w_C\Phi(D_{\mathrm{esdf}})
+
w_Q C_{\mathrm{comm}}
+
w_O C_{\mathrm{survey}}
+
w_E\frac{\Delta E_{\mathrm{lb}}}{E_0}.
$$

硬安全约束永远不通过代价权重交易。

## 9.8 启发函数

基础启发：

$$
h(n)
=
w_L\frac{d_{\mathrm{goal}}}{L_0}
+
w_T
\frac{d_{\mathrm{goal}}}
{v_{\mathrm{ground}}^{\max}T_0}.
$$

只在能量下界被证明可接受时加入能量项。

## 9.9 搜索终止

可能结果：

- `FOUND_PATH`
- `FOUND_SURVEY_PATH`
- `NO_PATH`
- `TIMEOUT`
- `CANCELLED`
- `DEPENDENCY_INVALID`

搜索成功只意味着获得几何/粗时序种子。
这些是搜索器内部终止值，不能直接作为 wire 数值：`NO_PATH` 映射为
`OUTCOME_NO_SOLUTION`，`TIMEOUT`/`CANCELLED` 分别映射到同名稳定 Outcome，
`DEPENDENCY_INVALID` 按事实映射为 `OUTCOME_DEPENDENCY_STALE` 或
`OUTCOME_INPUT_INVALID`；成功值只继续进入平滑与独立验证，不能直接发布成功计划。

---

# 10. 局部可行管与轨迹平滑

## 10.1 简化原则

V2.2 不维护独立的 SafeCorridor 子系统。

搜索路径周围建立内部 `FeasibleTube`，仅用于约束平滑器。

对路径采样点 $p_i$：

$$
\rho_i
=
D_{\mathrm{esdf}}(p_i)
-
r_{\mathrm{safe}}.
$$

若：

$$
\rho_i \le 0,
$$

该路径不能进入平滑。

局部可行球：

$$
B_i=
\{p:\|p-p_i\|\le \alpha\rho_i\},
\qquad
0<\alpha<1.
$$

相邻球必须有足够重叠，否则增加路径采样或直接重新搜索。

## 10.2 规范五次 Bézier 轨迹

仍使用分段五次 Bézier，因为：

- 轨迹可唯一重建；
- 位置、速度、加速度解析；
- 易于做 $C^2$ 拼接；
- 可用于控制和回放。

第 $i$ 段：

$$
p_i(s)
=
\sum_{j=0}^{5}
B_j^5(s)P_{i,j},
\qquad
s\in[0,1].
$$

航向：

$$
\tilde\psi_i(s)
=
\sum_{j=0}^{5}
B_j^5(s)\Psi_{i,j}.
$$

时间映射：

$$
t=t_i+s\Delta T_i,
\qquad
\Delta T_i>0.
$$

## 10.3 连续性

相邻段满足：

$$
p_i(1)=p_{i+1}(0),
$$

$$
\dot p_i(1)=\dot p_{i+1}(0),
$$

$$
\ddot p_i(1)=\ddot p_{i+1}(0).
$$

航向同样满足：

$$
\tilde\psi_i(1)=\tilde\psi_{i+1}(0),
$$

$$
\dot{\tilde\psi}_i(1)=\dot{\tilde\psi}_{i+1}(0),
$$

$$
\ddot{\tilde\psi}_i(1)=\ddot{\tilde\psi}_{i+1}(0).
$$

因此轨迹为 $C^2$。

## 10.4 平滑目标

优化变量为内部控制点和段时长。

硬约束：

- 起点状态；
- 终点/观测目标区域；
- 控制点位于对应 `FeasibleTube`；
- 段时长正；
- $C^2$ 连续；
- 粗速度和加速度上限。

软目标：

$$
J=
w_LJ_L
+
w_AJ_A
+
w_JJ_{\mathrm{jerk}}
+
w_TJ_T
+
w_{\mathrm{obs}}J_{\mathrm{obs}}.
$$

其中：

$$
J_L=\int_0^T \|\dot p(t)\|dt,
$$

$$
J_A=\int_0^T \|\ddot p(t)\|^2dt,
$$

$$
J_{\mathrm{jerk}}
=
\int_0^T
\|\dddot p(t)\|^2dt.
$$

## 10.5 求解器

生产首版不要求 SCP。

允许使用：

- 稀疏 SQP；
- constrained nonlinear least squares；
- 小规模 QP + 时间参数化；
- 经过确定性配置的其他约束优化器。

但求解器输出必须经过第 13 节统一验证。

求解器失败：

`SMOOTHING_FAILED`

而不是绕过平滑硬约束。

## 10.6 平滑失败处理

顺序固定：

1. 缩小平滑偏移；
2. 增加局部控制点；
3. 延长段时长；
4. 尝试一个备选搜索路径；
5. 仍失败则重新规划或返回失败。

不通过“放宽碰撞裕量”调通。

---

# 11. 时间参数化与控制参考

## 11.1 导数

对 Bézier 曲线解析计算：

$$
v(t)=\dot p(t),
$$

$$
a(t)=\ddot p(t),
$$

$$
j(t)=\dddot p(t).
$$

航向：

$$
r(t)=\dot{\tilde\psi}(t),
$$

$$
\dot r(t)=\ddot{\tilde\psi}(t).
$$

## 11.2 对水参考

$$
v_{\mathrm{water}}(t)
=
R_z^T(\tilde\psi)
\left[
\dot p(t)-\hat c(p,t)
\right].
$$

若海流空间梯度可用：

$$
a_{\mathrm{water}}
\approx
R_z^T
\left[
\ddot p
-\partial_t \hat c
-(\nabla \hat c)\dot p
\right]
-rS_zv_{\mathrm{water}}.
$$

若梯度不可用，则不声称精确计算对水加速度，仅使用已标定保守能力 margin。

## 11.3 轨迹能力条件

要求：

$$
\|v_{\mathrm{water}}\|
+\rho_{v}
\le
v_{\mathrm{water}}^{\max},
$$

$$
\|a_{\mathrm{water}}\|
+\rho_a
\le
a_{\mathrm{water}}^{\max},
$$

$$
|r|+\rho_r
\le
r_{\max},
$$

$$
|\dot r|+\rho_{\dot r}
\le
\dot r_{\max}.
$$

## 11.4 推进器分配可选复检

若 `ThrusterCapabilityChecker` 可用：

$$
\tau_{\mathrm{req}}(t)
=
M\dot\nu
+
C(\nu)\nu
+
D(\nu)\nu
+
g(\eta).
$$

检查：

$$
\exists f:
\quad
B(h)f=\tau_{\mathrm{req}},
$$

$$
f_{\min}(h)+\rho_f
\le f
\le
f_{\max}(h)-\rho_f.
$$

同时：

$$
|\dot f|
\le
\dot f_{\max}.
$$

如果该模型尚未完成标定，生产 profile 必须退回已验证 `VehicleCapabilityEnvelope`，不能使用未标定动力学假装更精确。

---

# 12. 最小风险动作

## 12.1 设计原则

V2.2 删除在线 L2 流管覆盖。

最小风险动作由独立 `SafetySupervisor` 管理：

$$
\Pi_R=
\{
\pi_{\mathrm{brake}},
\pi_{\mathrm{hold}},
\pi_{\mathrm{retreat}},
\pi_{\mathrm{return}},
\pi_{\mathrm{ascend}}
\}.
$$

规划器只提供当前轨迹、地图和状态信息，不直接决定推进器输出。

## 12.2 动作可执行性

每个动作有前置条件：

```cpp
struct RiskActionCondition {
    HealthProfile required_health;
    double minimum_energy_j;
    double minimum_clearance_m;
    bool requires_localization;
    bool requires_map;
    bool requires_communication;
};
```

## 12.3 制动距离

首版保守制动距离：

$$
d_{\mathrm{stop}}
=
vT_{\mathrm{latency}}
+
\frac{v^2}{2a_{\mathrm{brake}}^{\min}}
+
d_{\mathrm{margin}}.
$$

若局部剩余净空：

$$
d_{\mathrm{clear}}
<
d_{\mathrm{stop}},
$$

则 `brake` 不能被称为安全停车动作。

## 12.4 停留和上浮

“零推力”不等于悬停。

`hold` 必须满足：

- 当前健康 profile 支持悬停；
- 海流处于批准范围；
- 能源充足；
- 周围净空满足漂移 margin。

`ascend` 必须满足：

- 上方 3D 体素有已知自由走廊；
- 不违反水面/深度边界；
- 不进入铺缆机器人占据区域。

## 12.5 最小风险选择

过滤不可执行动作后按：

1. 即时碰撞风险；
2. 越界风险；
3. 失联与能源风险；
4. 设备损失风险；
5. 任务损失；
6. 时间和能耗；

排序。

没有满足前置条件的动作时必须明确报告风险，不伪造安全状态。

---

# 13. 统一安全验证器

## 13.1 目的

V2.2 将多个复杂证明层合并为一个：

`PlanSafetyValidator`

它只接受：

- 不可变上下文；
- 规范轨迹；
- 计划探测证据。

它不接受“优化器已成功”作为验证条件。

## 13.2 验证内容

至少检查：

1. 轨迹格式与时间；
2. 起点一致性；
3. 连续碰撞与允许水体；
4. 深度和海床净空；
5. 速度、加速度、角速度、角加速度；
6. 海流下能力包络；
7. 推进器健康/可选分配检查；
8. 能源；
9. 双机最小分离；
10. 双机最大通信距离；
11. 探测进入—驻留—退出；
12. RequiredCertificationVolume 覆盖率；
13. 依赖版本与有效期；
14. 最小风险动作的基本可用性；
15. 数值有限性和采样收敛。

## 13.3 碰撞连续验证

对每个 Bézier 段使用两级验证。

第一级：控制点凸包快速过滤。

若整个控制点凸包与保守障碍距离足够大，则该段通过几何快速检查。

第二级：自适应扫掠。

对区间 $[t_a,t_b]$，定义端点弦：

$$
\ell(t)
=
p(t_a)
+
\frac{t-t_a}{t_b-t_a}
[p(t_b)-p(t_a)].
$$

利用加速度上界：

$$
\|p(t)-\ell(t)\|
\le
\frac{(t_b-t_a)^2}{8}
\max_{t\in[t_a,t_b]}
\|\ddot p(t)\|.
$$

记：

$$
\rho_{\mathrm{curve}}
=
\frac{\Delta t^2}{8}a_{\max,\mathrm{seg}}.
$$

若整个弦扫掠到障碍的最小 ESDF 距离满足：

$$
D_{\min,\ell}
>
r_{\mathrm{safe}}
+
\rho_{\mathrm{curve}},
$$

则该时间区间可判定为安全。

否则继续二分。

达到：

$$
\Delta t \le \Delta t_{\min}
$$

仍不能确认时返回：

`OUTCOME_VALIDATION_INCONCLUSIVE`

而不是放行。

该方法比完整区间动力学证明简单，但仍避免只检查稀疏离散点。

## 13.4 导数上界

五次 Bézier 的一阶导控制点：

$$
D_j^{(1)}
=
\frac{5}{\Delta T}
(P_{j+1}-P_j).
$$

二阶导控制点：

$$
D_j^{(2)}
=
\frac{20}{\Delta T^2}
(P_{j+2}-2P_{j+1}+P_j).
$$

因此可利用凸包性质得到：

$$
\|\dot p(t)\|
\le
\max_j\|D_j^{(1)}\|,
$$

$$
\|\ddot p(t)\|
\le
\max_j\|D_j^{(2)}\|.
$$

航向同理。

这使速度/加速度检查不依赖密集采样。

## 13.5 探测验证

验证器重新计算：

$$
C_{\mathrm{survey}}
\ge C_{\min}.
$$

不使用规划阶段缓存的“coverage=true”。

## 13.6 统一验证结果

```cpp
struct ScoutPlanValidationReport {
    ScoutPlanValidationStatus status;
    OutcomeCode primary_outcome;
    std::optional<Duration> earliest_failure_time_offset;
    std::optional<double> minimum_collision_margin_m;
    std::optional<double> minimum_separation_margin_m;
    std::optional<double> minimum_energy_margin_j;
    std::optional<double> minimum_capability_margin;
    std::optional<double> survey_coverage_ratio;
    uint32_t refinement_depth;
    ContentIdentity validated_dependencies_content_identity;
    ContentIdentity validated_trajectory_content_identity;
    ContentIdentity validated_survey_evidence_content_identity;
    ContentIdentity validation_report_content_identity;
    std::vector<CodeRef> diagnostics;
};
```

只有：

`status == SCOUT_PLAN_VALIDATION_SAFE`

且报告精确绑定依赖、轨迹和探测证据三个内容身份的候选可以进入授权流程。

---

# 14. 候选排序

## 14.1 先硬后软

候选必须先满足全部硬约束。

不可通过：

- 更短；
- 更省电；
- 覆盖率更高；

来抵消碰撞、通信或能源硬约束。

## 14.2 软排序目标

对已验证候选：

$$
J_{\mathrm{rank}}
=
w_1J_{\mathrm{clearance}}
+
w_2J_{\mathrm{survey}}
+
w_3J_{\mathrm{energy}}
+
w_4J_{\mathrm{time}}
+
w_5J_{\mathrm{smooth}}.
$$

其中所有量无量纲化。

建议优先级：

1. 更大最小安全余量；
2. 更高探测覆盖；
3. 更大能源余量；
4. 更短任务时间；
5. 更平滑、更短路径。

## 14.3 候选数量

默认：

$$
K_{\mathrm{candidate}}
\le 3.
$$

不在一次规划里生成大量拓扑候选。

第一候选安全通过即可结束，只有验证失败或任务质量明显不足时才尝试备选。

---

# 15. 执行授权

## 15.1 为什么保留授权

执行租约在 V2.2 中继续保留，因为它能解决两个实际问题：

- 计划生成后环境已经变化；
- 控制端不能把任何缓存轨迹都视为当前可执行。

租约不能单独授权。前导运动域唯一的软件权威是 Scout NUC 上的 Scout
`ExecutionAuthority`；只有它发布的完整 `ScoutAuthorizedExecutionBundle`
能够授权 Scout FCU 的有限轨迹前缀。主机铺缆域的权威、计划、租约、Bundle
和水位不得适配或代理这一授权。

## 15.2 ScoutExecutionLease 与原子 Bundle

```cpp
struct ScoutExecutionLease {
    uint64_t lease_sequence;
    uint64_t plan_sequence;
    ContentIdentity plan_content_identity;
    MonotonicTime validated_at;
    MonotonicTime expires_at;
    Duration authorized_start_time_offset;
    Duration authorized_end_time_offset;
    ScoutPlanningDependencies dependencies;
    ContentIdentity content_identity;
};

struct ScoutAuthorizedExecutionBundle {
    ScoutMotionExecutionAuthorityDomain domain;
    uint64_t bundle_sequence;
    ScoutPlan plan;
    ScoutExecutionLease lease;
    MonotonicTime valid_not_before;
    MonotonicTime execution_epoch;
    ProfileRef timing_profile;
    ProfileRef interface_limits;
    ProfileRef safety_gate_configuration;
    ContentIdentity bundle_content_identity;
};
```

上述 C++ 只是 `execution.proto` 的逐字段内部表示；实际 wire 消息还必须携带
规范 `MessageHeader`。Bundle 的计划、租约、依赖、验证报告、轨迹、三个 profile
引用和内容身份必须一次性校验并原子安装，禁止从分离缓存拼装授权。执行历元固定，
不得按接收、安装或首帧 FCU 命令时间平移。

## 15.3 短时授权

授权区间：

$$
I_A=[t_a,t_b].
$$

必须满足：

$$
0\le t_a<t_b\le T_{\mathrm{validated}}.
$$

授权持续时间：

$$
T_A=t_b-t_a.
$$

不允许执行超出已验证时域的计划后缀。

## 15.4 授权前新鲜复检

Scout `ExecutionAuthority` 在签发前调用 SafetyGate，以最新不可变事实重新检查：

- 最新状态是否仍接近轨迹入口；
- 地图相关区域是否变化；
- 推进器健康是否变化；
- 协调条件是否过期；
- 能源是否低于计划所需；
- 计划 hash 是否匹配。

状态入口条件：

$$
\|p_{\mathrm{now}}-p(t_a)\|
\le
e_p^{\max},
$$

$$
|\operatorname{wrap}(\psi_{\mathrm{now}}-\psi(t_a))|
\le
e_\psi^{\max},
$$

并可增加速度误差：

$$
\|v_{\mathrm{now}}-v(t_a)\|
\le
e_v^{\max}.
$$

## 15.5 授权撤销

以下条件立即撤销：

- 新障碍进入授权区间；
- 相关地图变 STALE / CONFLICTED；
- 定位误差超过运行域；
- 跟踪误差超过阈值；
- 推进器健康下降；
- 能源余量不足；
- 双机距离或通信条件越界；
- 探测传感器失效；
- 计划 / 参数 / 版本 hash 错配；
- 授权超时。

Planner 和 SafetySupervisor 只提交事实或撤销请求；只有 Scout
`ExecutionAuthority` 发布规范 `ScoutExecutionRevocation`。FCU 本地保护和独立
硬急停路径可以不等待权威立即停止，但不能签发软件 lease。撤销后规划线程不直接
控制机器人，由 SafetySupervisor 依激活规则选择最小风险动作。

Ticket 10 将执行时公共 seam 固化为
`interfaces/proto/underwater/contracts/v1/execution.proto` 中彼此独立的
`ScoutExecutionFeedback`、`ScoutExecutionRevocation` 与
`ScoutExecutionRevocationAck`。反馈原子绑定精确 Bundle/plan/trajectory/lease、固定执行
历元偏移、FCU 会话和命令序列，并分离原始剖面目标、保守限幅后的应用目标与实测状态；
任何限幅、控制模式和风险动作都必须显式可见。撤销先持久化前导域水位并启动本地停止，
再进行高优先级幂等重试；ACK 丢失、旧 FCU 会话、轨迹时间回退、延迟反馈和 lease 到期
均不能续租或恢复授权。完整门禁见
`interfaces/SCOUT_EXECUTION_FEEDBACK_REVOCATION.md`；本节不把非生产时序值或参考消费者
视为 FCU、制动、跟踪阈值或实机通信证据。

---

# 16. 滚动规划与计划复用

## 16.1 滚动窗口

$$
T_{\mathrm{window}}
=
T_{\mathrm{authorized}}
+
T_{\mathrm{planning}}
+
T_{\mathrm{survey}}
+
T_{\mathrm{buffer}}.
$$

规划可看得更远，但授权只覆盖短前缀。

## 16.2 计划不可变

计划一旦生成：

- 曲线不修改；
- hash 不修改；
- 依赖版本不修改；
- 不原地延长有效期。

续用旧计划时生成新 lease，不修改旧 plan。

## 16.3 旧计划复用

新规划超时时：

1. 取旧计划剩余段；
2. 用最新同步上下文重新验证；
3. 通过则签发新短租约；
4. 不通过则撤销并进入最小风险处置。

## 16.4 复用条件

旧轨迹可复用当且仅当：

$$
H_{\mathrm{plan}}^{old}
=
H_{\mathrm{plan}}^{current}
$$

对于轨迹内容成立，并且关键依赖仍兼容。

关键依赖变化时必须重新验证，不复用旧 `safe=true` 布尔值。

---

# 17. 状态机

## 17.1 正交状态域

运行时不得维护一个组合式“全局 Scout 状态”。公共状态由四个正交实例组成：

| 实例 | 公共表示 | 唯一发布者 |
|---|---|---|
| 前导任务 | `STATE_DOMAIN_SCOUT_MISSION` + `ScoutMissionState` | Scout runtime |
| 前导运动授权 | `STATE_DOMAIN_SCOUT_EXECUTION_AUTHORITY` + `ExecutionAuthorityState` | Scout `ExecutionAuthority` |
| 主机—前导协同链路 | `CHANNEL_MAIN_SCOUT_COOP` + `CommunicationState` | 该通道状态所有者 |
| Scout NUC—FCU 链路 | `CHANNEL_SCOUT_NUC_FCU` + `CommunicationState` | 该通道状态所有者 |

风险动作不是授权域；急停状态也保持独立。`derived_scout_can_execute` 是由精确
Bundle、live lease、任务阶段、逐链路策略和本地安全状态计算的只读谓词，不能
直接写入。

## 17.2 任务主转换

```text
IDLE
  -> PLANNING
  -> READY
  -> EXECUTING
  -> OBSERVING
  -> WAITING_MAP
  -> REPLANNING
  -> EXECUTING
  -> COMPLETED
```

异常：

```text
EXECUTING
  -> DEGRADED
  -> RISK_ACTION
  -> REPLANNING / FAILED
```

恢复不能直接从 `RISK_ACTION` 回到执行。只有 `RISK_ACTION_COMPLETED` 与新鲜规划
前置条件可进入 `REPLANNING`；`RISK_ACTION_FAILED` 进入 `FAILED`。若存在锁存故障
或会话/配置失效，还必须显式清除故障并完成自检、兼容 Manifest/profile 激活、
新会话、新同步上下文和重新规划。授权域独立按
`NO_AUTHORIZATION -> AUTHORIZED -> REVOKING -> REVOKED` 演进，只有新的 authority
session 复位到 `NO_AUTHORIZATION` 后的新授权，或完整复检后严格更新的新 Bundle，
才能重新建立授权。

## 17.3 单调安全原则

状态机内部安全严重度只允许在没有新证据时保持或升级，不允许因为“规划线程返回成功”自动降级风险。

例如，任务/安全处置的概念严重度可以描述为：

```text
NORMAL < DEGRADED < LEASE_REVOKED < RISK_ACTION
```

该概念顺序不是公共状态枚举，也不得驱动跨域隐式转换。恢复到较低严重度必须经过：

- 新同步快照；
- 新验证；
- 新授权。

## 17.4 公共状态、代码与配置契约追踪

Ticket 11 将本章状态图固化为
`interfaces/registry/scout-state-transitions-v1.json` 中可执行的精确转换表，
分别维护 ScoutMission、ScoutExecutionAuthority、MAIN_SCOUT_COOP 和
SCOUT_NUC_FCU 状态实例；一次转换只改变一个域，合法转换和非法拒绝都绑定
稳定 trigger `CodeRef` 与 `SafetyAuditEvent`。风险动作必须在任务授权撤销后
由独立监督器选择，不能授予新探测动作。

`interfaces/proto/underwater/contracts/v1/{codes,state,diagnostics,profiles}.proto`
与 `interfaces/registry/codes-v1.json` 固化 Scout Outcome/Fault/Diagnostic、
严重度、安全效果、锁存、清除权威及单向恢复链；
`interfaces/profiles/integration-v1.json` 作为 NON_PRODUCTION 联调快照原子绑定
TimingProfile、InterfaceLimits、代码/转换注册表、能力、能源、传感器、
planner/SafetyGate 配置、逐链路 LossPolicy 与风险动作规则。未知安全枚举、
非法转换、缺失生产证据和不兼容 profile 的失败关闭及审计要求见
`interfaces/SCOUT_STATE_CODES_PROFILES.md`。Ticket 31 只能实现该公共语义，
不能另建竞争状态、代码或隐式默认配置。

---

# 18. 端到端主算法

## 18.1 Planner

```text
PLAN(context, scout_local_deadline, cancellation):

    validate_manifest_profile_and_resource_limits(context)
    validate_exact_dependency_identities(context.dependencies)
    validate_admitted_mission(context.mission)

    if dependencies_stale:
        return FAILURE(OUTCOME_DEPENDENCY_STALE, context.dependencies)

    task = select_survey_action(context.mission, context)

    if task == none:
        return FAILURE(OUTCOME_NO_SOLUTION, context.dependencies)

    search_result = time_aware_3d_search(
        context.map,
        context.state,
        task,
        context.main_robot_prediction,
        context.capability,
        context.energy,
        scout_local_deadline)

    if search_result.failed:
        return FAILURE(search_result.outcome, context.dependencies,
                       search_result.diagnostics)

    for path in bounded_candidate_paths(search_result):

        trajectory = smooth_and_parameterize(
            path,
            task,
            context)

        if trajectory.failed:
            continue

        survey_evidence = evaluate_survey_plan(
            trajectory,
            context.mission,
            context.map,
            context.sensor)

        if survey_evidence.failed:
            continue

        validation = validate_plan(
            trajectory,
            survey_evidence,
            context,
            scout_local_deadline)

        if validation.status == SCOUT_PLAN_VALIDATION_SAFE:
            plan = freeze_scout_plan(
                trajectory,
                survey_evidence,
                validation,
                context.dependencies)
            return SCOUT_PLANNING_RESULT(
                OUTCOME_SUCCESS, plan, context.dependencies)

    return FAILURE(select_terminal_outcome(), context.dependencies,
                   ordered_diagnostics)
```

成功结果必须携带一个精确候选，失败结果不得携带候选；两者都携带同一求解所用
`ScoutPlanningDependencies`、Scout 本地 `evaluated_at_monotonic_ns`、稳定
`OutcomeCode`/`CodeRef` 和规范内容身份。多个失败同时存在时按附录 B 的稳定优先级
选唯一主结果，其余只作为诊断。

## 18.2 授权

```text
AUTHORIZE(candidate, latest_context, authority_state):

    require canonical Scout ExecutionAuthority and Scout clock domain
    verify exact candidate, plan, trajectory, evidence and report identities
    require validation.status == SCOUT_PLAN_VALIDATION_SAFE

    verify_dependency_compatibility(candidate.dependencies, latest_context)

    verify_current_state_at_entry(candidate, latest_context)

    validate_authorized_prefix(candidate, latest_context)
    choose fixed execution_epoch and non-empty authorized time interval
    bind exact TimingProfile, InterfaceLimits and SafetyGate configuration

    if any_check_failed:
        return structured rejection without advancing authorization watermarks

    return atomically issue ScoutAuthorizedExecutionBundle
```

执行 adapter 只原子安装完整 Bundle 并发布 `ScoutBundleAck`。独立 lease、候选发布、
ACK、状态或诊断都不授权，ACK 也不续租。

## 18.3 执行监控

```text
MONITOR():

    check_tracking_error
    check_map_updates
    check_health
    check_energy
    check_coordination
    check_sensor_health
    check_lease_expiry

    if any hard condition violated:
        submit exact fact with event/correlation identity
        ExecutionAuthority.persist watermark and publish exact revocation
        local execution path starts requested stop without waiting for ACK
        SafetySupervisor.select_configured_risk_action()
```

公共监控证据按 Ticket 10 的三视图反馈契约消费；跟踪偏差、健康下降、相关地图变化、
通信丢失和 lease 到期必须形成可区分的 `ScoutRevocationReason`。反馈或 ACK 只记录事实，
不能直接改变授权状态，也不能在通信恢复后复活旧 Bundle。

## 18.4 探测完成

```text
COMPLETE_SURVEY():

    bind actual observation ids
    wait for new immutable map snapshot
    compute achieved coverage
    build SurveyCompletionEvidence
    publish to laying planner
```

---

# 19. 公共数据契约

## 19.1 依赖版本

V2.2 使用公共 `ScoutPlanningDependencies` 作为唯一跨边界依赖闭包。每个版本都与
对应内容身份绑定，profile 使用完整 `ProfileRef`；不能退化为只有版本号的集合：

```cpp
struct ScoutPlanningDependencies {
    MissionIdentity mission;
    MapIdentity map;
    VersionedContentIdentity navigation;
    std::vector<ScoutSensorDependency> sensors; // sensor_id 唯一且升序
    VersionedContentIdentity current_model;
    ProfileRef capability_profile;
    VersionedContentIdentity thruster_health;
    ProfileRef energy_model;
    VersionedContentIdentity energy_state;
    VersionedContentIdentity main_robot_prediction;
    VersionedContentIdentity coordination;
    ProfileRef planner_configuration;
    ProfileRef timing_profile;
    ProfileRef interface_limits;
    ContentIdentity dependencies_content_identity;
};
```

概念展开后必须无损覆盖 `planning.proto` 的全部字段，包括：

```text
mission_id / mission_version / mission_content_identity
map_id / map_version / map_content_identity
navigation_version / navigation_content_identity
sensor_id / geometry_version+identity / health_version+identity
current_model_id / current_model_version / current_content_identity
capability_profile / thruster_health_version+identity
energy_model / energy_store_id / energy_state_version+identity
prediction_id / prediction_version+identity
coordination_version+identity
planner_configuration / timing_profile / interface_limits
dependencies_content_identity
```

内部算法可以使用适合计算的强类型视图，但不能遗漏、发明默认值或重新发布竞争的
依赖身份。

## 19.2 PlanningContext

```cpp
struct ScoutPlanningContext {
    ScoutNavigationState navigation;
    HybridMapSnapshot map;
    ScoutCurrentEstimate current;
    std::vector<PairedSensorGeometryAndHealth> sensors;
    ScoutCapabilityProfile capability;
    ScoutThrusterHealthState thruster_health;
    ScoutEnergyModelProfile energy_model;
    ScoutEnergyState energy_state;
    MainRobotPrediction main_robot_prediction;
    ScoutCoordinationConstraint coordination;
    AdmittedScoutMission mission;
    ScoutPlanningDependencies dependencies;
    ActivatedScoutConfigurationProfile configuration;
    MonotonicTime scout_local_captured_at;
};
```

`ScoutPlanningContext` 是核心内部不可变值对象，不是新增 wire 消息。Ticket 15
必须证明其每个值与 `dependencies` 中的版本、内容身份、profile 和本地接收时刻
原子一致。

**Ticket 15 实施绑定**：ROS 无关核心以
`scout_planner/core/planning_context.hpp` 中的
`PlanningContextBuilder::capture(source, activated_configuration, captured_at)`
作为唯一捕获 seam。source 在读取前后提供单调 generation；任一输入更新使整次捕获
返回 `capture_race`。Builder 对每类 local receipt、Scout 本地观察/有效窗、同步观测、
任务/预测配对、运行域、配置 profile 和上述完整依赖逐项失败关闭。`AdmittedScoutMission`
同时携带经统一 Protobuf/core seam 验证内容身份的 `ScoutMissionDecision`；只有 disposition
为 `ACCEPTED`、本地 admission 窗仍有效、decision receive tick 与捕获的 mission receipt
相等且精确绑定任务与协调版本时才可进入上下文。
Builder 固定每个 stream 的 canonical producer，以每个逻辑对象保留交付位置，并只在全部校验成功
后原子推进 producer session、delivery sequence 与业务版本水位；同流多个传感器的旧缓存值
允许作为精确重复保留，但任何新交付必须严格超过整个 stream 水位，单次捕获也不得混合
producer/session。成功值复制输入、receipt、
激活配置、捕获时刻和 generation，规划器只能取得 const 视图；后续 latest 缓存更新不能
替换已发布上下文成员。确定性公开 seam 证据位于
`src/scout_core/test/test_planning_context.cpp`。这些联调时序值和夹具仍是
NON_PRODUCTION，不构成生产新鲜度或同步精度标定。

## 19.3 规范轨迹

```cpp
struct QuinticBezierSegment4d {
    Duration start_time_from_plan;
    Duration duration;
    std::array<Vector3M, 6> position_control_points;
    // 相对 CanonicalTrajectory4d.initial_yaw_rad 的连续旋转位移；不按
    // +/-pi 归一化，也不在分段边界重置。
    std::array<double, 6> yaw_offset_control_points_rad;
};

struct CanonicalTrajectory4d {
    std::string frame_id;
    double initial_yaw_rad;  // ENU，规范化到 [-pi, pi)
    std::vector<QuinticBezierSegment4d> segments;
    Hash256 content_hash;
};
```

Implementation tracking (Ticket 22): `scout_planner::core::BezierTrajectory4d` and
`QuinticBezierSegment` implement this canonical four-dimensional representation at the
ROS-independent core boundary. Construction enforces `mission_enu`, finite control points,
positive contiguous nanosecond durations and physical-time C2 continuity. The segment API
exposes analytic samples, third-order position and second-order yaw derivatives, derivative
control polygons and convex-hull limits. A deterministic SHA-256 over the core's canonical
binary encoding is retained as the internal trajectory content identity; public wire identities
continue to be produced and checked by the `ProtobufAdapter` seam. Neither identity is an
execution authorization.

## 19.4 Plan

```cpp
struct ScoutPlan {
    uint64_t plan_sequence;
    MonotonicTime created_at;
    ScoutTrajectory4d trajectory;
    ScoutPlanningDependencies dependencies;
    SurveyPlanEvidence survey_evidence;
    ScoutPlanValidationReport validation_report;
    ContentIdentity plan_content_identity;
};
```

## 19.5 结果码

公共结果使用 `codes.proto` 的稳定 `OutcomeCode`，而不是另建
`PlanningResultCode`。算法终止结果集合为：`OUTCOME_SUCCESS`、
`OUTCOME_INPUT_INVALID`、`OUTCOME_DEPENDENCY_STALE`、
`OUTCOME_TIMEOUT`、`OUTCOME_NO_SOLUTION`、
`OUTCOME_CANCELLED`、`OUTCOME_SMOOTHING_FAILED`、
`OUTCOME_CAPABILITY_INFEASIBLE`、`OUTCOME_ENERGY_INSUFFICIENT`、
`OUTCOME_COORDINATION_INFEASIBLE`、`OUTCOME_SURVEY_INFEASIBLE`、
`OUTCOME_VALIDATION_REJECTED`、`OUTCOME_VALIDATION_INCONCLUSIVE` 和
`OUTCOME_NUMERICALLY_INVALID`。无探测动作与无路径都收敛为
`OUTCOME_NO_SOLUTION`；更细原因只能使用同版本注册表中的 `CodeRef` 诊断。
`OUTCOME_VERSION_INCOMPATIBLE`、`OUTCOME_RESOURCE_LIMIT_EXCEEDED`、
`OUTCOME_SEQUENCE_REJECTED` 与 `OUTCOME_CLOCK_DOMAIN_MISMATCH` 属于消息/配置消费
门禁结果，不得混入规划算法的终止优先级。

任务/授权/通信状态、终止 Outcome、持续 Fault 与观察性 Diagnostic 相互分离。
自由文本不得驱动控制、授权或状态转换。

---

# 20. 软件架构

## 20.1 分层

```text
Perception / Mapping / Localization
            |
            v
ROS2 Adapters
            |
            v
ContextBuilder
            |
            v
ScoutMotionPlanner
    ├─ SurveyTaskPlanner
    ├─ TimeAwareStateLatticeAStar3d
    ├─ TrajectorySmoother
    ├─ TrajectoryParameterizer
    ├─ SurveyEvaluator
    └─ PlanSafetyValidator
            |
            v
Scout ExecutionAuthority
    └─ SafetyGate / LeaseMonitor
            |
            v
Scout ExecutionAdapter / FCU

SafetySupervisor  <---- independent runtime safety path
```

## 20.2 核心模块

| 模块 | 职责 |
|---|---|
| ContextBuilder | 同步状态、地图、版本和参数 |
| SurveyTaskPlanner | 生成进入/扫描/退出任务 |
| TimeAwareStateLatticeAStar3d | 离散航向/动作模式的三维 state-lattice 与有界到达时间搜索 |
| TrajectorySmoother | 五次 Bézier 平滑 |
| TrajectoryParameterizer | 时间、速度、加速度参考 |
| SurveyEvaluator | FOV、遮挡、覆盖率 |
| PlanSafetyValidator | 统一安全复检 |
| SafetyGate | 在新鲜状态下执行授权前硬门禁并向唯一 ExecutionAuthority 返回结构化决定；不得签发 lease/Bundle |
| Scout ExecutionAuthority | 前导域唯一原子 Bundle/lease/规范撤销发布者 |
| LeaseMonitor | 执行时提交带因果身份的撤销事实；不得发布撤销或维护 authority 水位 |
| Scout ExecutionAdapter | 原子安装 Bundle、固定历元采样、MAVLink/FCU 适配与三视图反馈 |
| SafetySupervisor | 最小风险动作 |

相对 V2.0，算法内部模块数量明显减少；授权与执行 seam 保持独立，不能为了
模块数量更少而合并进 Planner 或 ROS 节点。

## 20.3 对外深接口

```cpp
class ScoutMotionPlanner {
public:
    [[nodiscard]] virtual ScoutPlanOutcome plan(
        const ScoutPlanningContext& context,
        PlanningDeadline deadline,
        const CancellationToken& cancellation) const = 0;
};
```

ROS 层不能逐个编排内部模块。该接口只产出无授权候选；授权 seam 是
`ExecutionAuthority::authorize(candidate, latest_facts)`，执行 seam 是
`ExecutionAdapter::install(bundle)`，两者都必须保持深接口与失败关闭。

## 20.4 ROS 2 节点职责

`ScoutPlannerNode` 只负责：

- 订阅输入；
- 缓存最新值；
- ContextBuilder 捕获同步快照；
- 请求、取消、deadline；
- ROS 消息与核心 C++ 类型转换；
- 发布计划、状态、诊断。

禁止：

- ROS 回调内执行大规模搜索；
- ROS 节点修改验证结果；
- ROS 节点直接延长租约；
- PlannerNode 直接发推进器指令。

---

# 21. 并发与生命周期

## 21.1 执行通道

最少分为：

1. 输入回调；
2. 地图派生；
3. 主规划 worker；
4. Scout ExecutionAuthority / SafetyGate / LeaseMonitor；
5. SafetySupervisor；
6. 日志与回放。

安全通道不得等待主规划线程。

## 21.2 取消

新高优先级任务可取消旧规划。

已经被取消的规划即使最后形成 `OUTCOME_SUCCESS`，结果也必须丢弃并发布
`OUTCOME_CANCELLED` 失败结果。

## 21.3 Deadline

搜索、平滑、验证都接收绝对 deadline。

若：

$$
T_{\mathrm{remaining}}
<
T_{\mathrm{validation}}^{\mathrm{reserve}},
$$

则不再启动新的候选优化，直接返回 `OUTCOME_TIMEOUT`。

---

# 22. 参数体系

## 22.1 参数分层

参数分为：

### vehicle

- 速度上限；
- 加速度上限；
- 航向角速度；
- 航向角加速度；
- 滚俯角允许范围；
- 制动能力；
- 健康 profile。

### map

- 体素尺寸；
- ESDF 分辨率；
- 地图最大年龄；
- UNKNOWN 策略；
- 安全 margin。

### search

- 邻接原语；
- 航向离散；
- 最大节点数；
- 最大规划窗口；
- 候选重试数。

### smooth

- Bézier 段长度；
- 控制点偏移；
- 目标权重；
- 最小段时长。

### survey

- FOV；
- 量程；
- 入射角；
- 覆盖率；
- 驻留时间；
- 遮挡 margin。

### coordination

- 最小分离；
- 最大通信距离；
- 失联策略。

### energy

- reserve；
- return reserve；
- 功率模型。

### validation

- 自适应细分最小时间；
- 数值容差；
- 入口误差；
- 租约持续时间。

## 22.2 生产禁止伪默认值

以下参数在生产 profile 中必须显式标定：

- 机体几何；
- 速度/加速度能力；
- 制动能力；
- 推进器健康 profile；
- 跟踪误差 margin；
- 定位误差；
- 地图误差；
- 传感器量程和 FOV；
- 最小通信距离余量；
- 能耗；
- 最小能源 reserve。

测试值必须标记 `NON_PRODUCTION`。

## 22.3 原子配置与激活

生产运行不得从上述分层中独立挑选参数。公共 `ScoutConfigurationProfile` 必须
原子绑定 TimingProfile、InterfaceLimits、代码/状态转换注册表、能力 profile、
能源模型、传感器几何、planner/SafetyGate 配置、两条通道各自的 LossPolicy 及
允许的风险动作规则。prepare 阶段重算并验证全部内容身份，activate 阶段原子切换
配置并创建新会话；任一缺失、未知或不兼容引用都拒绝整个配置，不能沿用旧值补齐。

---

# 23. 实时性与复杂度

## 23.1 搜索复杂度

若搜索状态数为 $N$、平均分支数 $b$：

$$
T_{\mathrm{search}}
=
O(bN\log N).
$$

相对 V2.0，多标签数量被严格限制。

## 23.2 平滑复杂度

设 Bézier 段数 $m$，变量数：

$$
N_z=O(m).
$$

生产实现应：

- 限制最大段数；
- 使用稀疏结构；
- 允许 warm start；
- 失败后不无限迭代。

## 23.3 验证复杂度

设自适应最终区间数为 $N_I$：

$$
T_{\mathrm{validate}}
=
O(N_I C_q),
$$

$C_q$ 为一次 ESDF、能力、双机和观测查询成本。

## 23.4 频率分层

不要求所有模块同频：

- 控制器：高频；
- LeaseMonitor：高频；
- SafetyGate：中高频；
- 主规划：事件 + 周期；
- 地图更新：感知频率；
- SurveyCompletionEvidence：观测后触发。

同一激活 TimingProfile 至少满足：

```text
Scout feedback publish < stale warning < software revoke < lease duration
Scout revocation retry < revocation ACK timeout
Scout FCU heartbeat < degraded < lost <= software revoke
  < FCU command watchdog <= Scout hard-safety timeout
```

所有安全比较只在对应本地时钟域内进行。

主规划不需要追求几十 Hz。

对于前导机器人，稳定的亚秒到数秒滚动规划在大量场景中是可以接受的，但最终目标频率必须由仿真和实机任务速度决定。

## 23.5 资源硬上限

以下全部必须有 hard limit：

- 搜索节点；
- open queue；
- 候选数量；
- 平滑变量；
- 验证细分深度；
- 地图窗口；
- 日志队列。

消息或配置超过 `InterfaceLimits` 时在公共 seam 整体拒绝并返回
`OUTCOME_RESOURCE_LIMIT_EXCEEDED`。已进入算法后的搜索节点/时间预算耗尽返回：

`OUTCOME_TIMEOUT` 或 `OUTCOME_VALIDATION_INCONCLUSIVE`

而不是降低安全检查。

---

# 24. 异常与降级

## 24.1 定位异常

若：

$$
\sigma_{\mathrm{loc}}
>
\sigma_{\mathrm{loc}}^{\max},
$$

停止新的向外探索。

优先：

1. 减速；
2. 保持/局部撤退；
3. 等待定位恢复；
4. 必要时进入批准的最小风险动作。

## 24.2 地图异常

相关地图变：

- UNKNOWN；
- STALE；
- CONFLICTED；

当前授权区间若受影响则撤销。

## 24.3 通信异常

若与铺缆机器人通信中断：

- 不开始新探测；
- 按 `CHANNEL_MAIN_SCOUT_COOP` 的独立 LossPolicy 执行；
- 已有短轨迹只有在策略明确允许且仍满足本地安全时才能完成；
- LOST 必须撤销并选择配置的 RETURN 风险动作，不无限等待。

Scout NUC—FCU 通道独立使用 `CHANNEL_SCOUT_NUC_FCU` 的 LossPolicy：DEGRADED
即撤销并保护停车，LOST 使用本地 BRAKE。两条通道不得共享状态、水位或恢复结果；
恢复只建立新会话和通信状态，不恢复旧 Bundle 或 lease。

## 24.4 推进器异常

健康 profile 变化：

- 立即更新 capability；
- 当前 lease 复检；
- 不可执行则撤销；
- SafetySupervisor 选择动作。

## 24.5 规划失败

规划失败不等于立即停车。

处理顺序：

1. 尝试验证旧剩余轨迹；
2. 若仍可执行，签发短 lease；
3. 同时重规划；
4. 旧轨迹不能继续则撤销；
5. 进入最小风险动作。

---

# 25. 验证与测试体系

## 25.1 测试层级

| 层级 | 内容 |
|---|---|
| T0 | 类型、单位、frame、版本、序列化 |
| T1 | 地图、搜索、样条、覆盖、能力、能源单元测试 |
| T2 | 属性测试和独立几何 oracle |
| T3 | 确定性集成场景 |
| T4 | ROS2 SIL |
| T5 | DAVE/Gazebo 联合仿真 |
| T6 | HIL / 水池 |
| T7 | 分阶段海试 |

## 25.2 必须拒绝的反例

至少包括：

- 两个采样点之间存在薄障碍；
- 2.5D 海床上方存在悬垂物；
- UNKNOWN 被授权；
- 地图相关区域过期仍继续执行；
- 当前顺流可行但对水速度越界；
- 推进器健康变化后继续使用旧能力；
- 能量够到目标但不够撤退/返航；
- 前导与铺缆机器人发生占据冲突；
- 距离在通信上限内但最小分离不满足；
- 单一观测点可达但退出路径不可行；
- FOV 看见目标中心但覆盖率不足；
- 轨迹 hash 错配仍被控制端接受；
- 规划结束时当前状态已明显偏离轨迹入口；
- lease 已过期仍执行；
- 新障碍进入授权前缀但没有撤销；
- 制动距离大于剩余净空却选择 brake；
- 上方未知却选择 ascend；
- 跟踪误差超过门限仍沿用旧 lease。

## 25.3 搜索测试

覆盖：

- 空旷 3D；
- 狭窄通道；
- 上绕 / 下绕；
- 悬垂障碍；
- 死胡同；
- 通信距离限制；
- 铺缆机器人移动占据；
- 能源不足；
- deadline；
- 节点预算耗尽。

## 25.4 平滑测试

覆盖：

- 直线；
- 90° 转弯；
- 3D 上下绕；
- 短段；
- 高曲率；
- 起终速度非零；
- 航向跨 $\pm\pi$；
- $C^2$ 拼接；
- 控制点 tube 越界；
- 求解失败 fallback。

## 25.5 验证器测试

验证器必须有独立 oracle。

尤其对碰撞：

- 高密度 brute-force 采样只作为差分 oracle；
- 生产验证使用解析偏差界 + 自适应细分；
- 两者随机场景结果应一致或生产验证更保守。

## 25.6 探测测试

覆盖：

- 正常覆盖；
- 遮挡；
- 量程边缘；
- 固定摄像机/声呐朝向；
- 部分 required volume；
- mandatory 子体积缺失；
- 执行轨迹与计划轨迹偏差；
- 新地图未关联实际 observation id。

## 25.7 故障注入

注入：

- 定位跳变；
- DVL 丢失；
- USBL 超龄；
- 地图乱序；
- 地图冲突；
- 海流突变；
- 推进器降级；
- 能源下降；
- 双机失联；
- planner timeout；
- solver NaN；
- ROS time reset。

## 25.8 Ticket 13 可重复工程基线

Ticket 13 已在 `src/scout_core/` 建立 ROS 无关 C++17 核心和公开确定性夹具
seam，并由 `tools/verify.ps1` / `tools/verify.sh` 通过同一入口执行严格编译、
静态检查、CTest 与 JUnit 输出。夹具覆盖平坦海床、悬垂障碍、狭窄通道、
UNKNOWN、STALE、CONFLICTED 与铺缆机器人移动占据；移动占据使用与第 7.4 节
一致的连续闭区间保守扫掠球，显式区分物理半径、位置不确定度和最终占据半径。

所有规定夹具均绑定固定种子、SI 单位、`mission_enu`、Scout 本地测试时钟域、
有序输入版本及 `non_production=true`。这些元数据在夹具构造前即建立，因此构造、
测试或拒绝路径的失败输出都可重放。首次冷构建、CTest、测试进程耗时和峰值 RSS
记录在 `project.md/TICKET_13_BASELINE.md`。该基线只证明工程与夹具可重复性，
不证明搜索、协调可行性、SIL/HIL、生产实时性或实机安全能力。

## 25.9 Ticket 14 核心类型与 Protobuf 适配证据

Ticket 14 在 `src/scout_core/include/scout_planner/core/protobuf_adapter.hpp` 发布
19 类不可默认构造的强类型 `CoreContract`，以纯 C++ 递归值树无损保留公共字段号、
presence、repeated 顺序、整数宽度、浮点、enum 类型与 bytes/text 区分；Protobuf
只存在于独立 adapter 实现和构建期生成目标中，未引入 ROS 类型或另建公共语义。

单一 adapter 校验入口集中执行第 3.4 节的有限性、frame、时间、版本、协方差与
结构门禁，并覆盖 ENU/FLU 协方差 frame 与规范缩放容差、地图维度、单位四元数、
五次段 C2 连续性、首段零 yaw offset、SAFE 报告三重身份与硬指标、结果/候选依赖
精确绑定、唯一升序集合、activated message/collection 上限以及成功/失败结果载荷；
unknown field/enum、缺失 safety presence、非 NFC text 和非规范瞬时 yaw 均整体
拒绝。内容身份依照 `interfaces/HASHING.md` 的精确清除规则从嵌套叶节点向根递归
重算确定性 Protobuf SHA-256 并恒定时间比较，消费端不可关闭身份校验；生产者通过
显式 canonicalize-and-identify seam 构造完整身份树。排序后的全部 v1 schema
descriptor 由构建门禁固定为附录 E 的 SHA-256，避免 adapter 静默漂移。

`src/scout_core/test/test_protobuf_adapter.cpp` 从公共 bytes/core seam 覆盖 19 类
逐字段双向往返、golden descriptor/hash、版本回退、非有限输入、数值边界、
嵌套身份篡改、SAFE 绑定、依赖错配、排序、资源超限和失败结果反例；
`tools/verify.ps1` 与系统公共
契约 148 项回归均通过。该证据只证明
无损适配和入口校验，不实现 Ticket 15 的原子上下文或 Ticket 16 的地图查询，也不构成生产、SIL/HIL 或实机证据。

## 25.10 Ticket 18 双机器人协调评价证据

Ticket 18 在 `src/scout_core/include/scout_planner/core/coordination_evaluator.hpp`
和 `src/scout_core/src/coordination_evaluator.cpp` 发布纯 C++ 协调评价 seam。
它沿 `mission_enu` 同步时间轴消费 Scout 时序采样与主机连续闭区间保守扫掠球，
用相对线性运动的连续最小距离检查分离和通信边界，并返回最早失败时间、最小
margin 与几何/标定链路声明边界。任务、预测、时钟域、同步不确定度、本地接收
新鲜度、预测连续性和预测终止均失败关闭，不以单点当前位置或发送方 deadline
静默回退。测试覆盖交叉、平行远离、预测边界、过期和确定性结果；验证证据记录
在 `project.md/PLAN.md` Ticket 18 及对应 issue。该实现不提供生产链路质量、SIL/HIL
或执行授权证据。

## 25.11 Ticket 19 探测动作与观测几何证据

Ticket 19 在 `src/scout_core/include/scout_planner/core/survey_action.hpp`
和 `src/scout_core/src/survey_action.cpp` 固化 `SurveyActionPlanner`。规划结果
始终包含有序的 approach、observe、exit 三段以及观测姿态和驻留时间；进入与
退出路径通过 `HybridMapQuery` supercover 要求所有穿越体素 known-free。覆盖率
由版本化传感器外参、姿态四元数、水平/垂直 FOV、量程边界、位姿/量程误差和
遮挡射线共同决定，以 required region 体素中心做确定性保守采样；mandatory
子体积采用逐样本完整覆盖门禁。几何未生产批准、健康非 NOMINAL/有活动故障、
导航无效、区域非法、路径不可行或覆盖不足均返回稳定结构化失败码。该 seam
只产生规划期 pointwise 覆盖估计，不发布 observation ID、完成证据、地图更新、
整路径联合风险保证或执行授权。

## 25.12 Ticket 20 确定性 3D state-lattice A* 证据

Ticket 20 在 `src/scout_core/include/scout_planner/core/state_lattice_astar.hpp`
和 `src/scout_core/src/state_lattice_astar.cpp` 固化
`TimeAwareStateLatticeAStar3d`。基础状态是三维体素、离散航向和动作模式；
26 邻域运动原语使用 `HybridMapQuery::query_supercover` 连续检查，所有未知、
过期、冲突、占据、净空不足和允许水体外状态均不可进入。搜索使用非负距离/时间
代价与到目标 AABB 的速度下界启发，并对节点、open queue、内存、注入单调时钟
deadline 和取消提供确定性失败语义。该 seam 仅输出几何搜索种子，不授予执行权，
不替代后续平滑、能力/能源、协调和安全验证。

---

# 26. 性能指标

至少记录：

## 26.1 搜索

- 搜索时间 P50/P95/P99；
- 扩展节点数；
- open queue 峰值；
- 失败率；
- 节点预算耗尽率。

## 26.2 平滑

- 求解时间；
- 迭代次数；
- fallback 次数；
- 平滑前后路径长度；
- 最大速度/加速度 margin。

## 26.3 验证

- 验证时间；
- 自适应区间数；
- 最深细分；
- 最小碰撞 margin；
- `INCONCLUSIVE` 比例。

## 26.4 任务

- 探测覆盖率；
- 探测任务完成时间；
- 地图证据刷新时间；
- 铺缆侧因缺图停车时间；
- 前导平均通信距离；
- 能耗；
- 重规划次数；
- lease 撤销次数。

## 26.5 安全监督

- 从异常发生到 lease 撤销的延迟；
- 从撤销到 risk action 下发的延迟；
- 错误继续执行次数必须为 0。

---

# 27. 实施路线

## 27.1 开发任务

后续实现以 `project.md/PLAN.md` 为唯一动态进度入口。Tickets 01A、02-12 闭合
系统边界与公共契约；本基线发布后，工程任务为：

| ID | 交付 |
|---|---|
| 13-15 | 可重复工程、公共类型无损适配、原子规划上下文 |
| 16-19 | 混合地图查询、能力/海流/能源、协调、探测动作与观测几何 |
| 20-21 | 确定性与时间感知 `TimeAwareStateLatticeAStar3d` |
| 22-26 | 五次 Bézier、平滑时序、探测证据、连续验证与统一安全报告 |
| 27 | `ScoutMotionPlanner` 主链 |
| 28-31 | Scout SafetyGate/ExecutionAuthority、lease、监督器与正交状态机 |
| 32-33 | 确定性 Level 1 闭环、ROS 2 adapter 与可视化 |
| 34-36 | DAVE/Gazebo、HIL/水池、海试与生产就绪审计 |

## 27.2 关键路径

```text
12 -> 13 -> 14 -> 15/16/17/18/19
16 -> 20 -> 21 -> 23 -> 24/25 -> 26 -> 27
22 -----------------------> 23
27 -> 28 -> 29 -> 30/31 -> 32 -> 33/34 -> 35 -> 36
```

## 27.3 开发顺序建议

第一阶段先打通：

> 工程与无损适配 → 原子上下文 → 地图/输入门禁

第二阶段增加：

> state-lattice 搜索 → 五次轨迹 → 平滑时序 → 探测与独立安全验证

第三阶段增加：

> Planner 主链 → 原子 Bundle/lease → 异步精确撤销 → 最小风险动作与状态机

第四阶段：

> Level 1 → ROS 2/SIL/DAVE/Gazebo → HIL → 水池参数辨识 → 海试审计

如果后期性能和任务需要，再增加：

- 多拓扑搜索；
- 更完整 6DOF 在线动力学；
- SCP；
- 恢复策略流管；
- 概率风险预算。

这些属于增强项，不再是基础版本的前置依赖。

---

# 28. 首版实施范围与生产门禁

首版算法实现建议限定：

- 局部滚动三维地图；
- 静态环境障碍为主；
- 铺缆机器人具有有限时域预测；
- 固定朝向声呐和摄像机；
- 一个 nominal 推进器 profile；
- 少量明确退化 profile；
- 已标定海流范围；
- 已知自由空间才能授权；
- 固定短 lease；
- 固定最小风险动作集合；
- 不声明复杂概率安全保证。

该范围足以实现前导机器人的核心算法闭环，但在 ticket 34-36 完成前不得称为
生产可执行或真实水下安全能力。当前 `integration/v1` 为 NON_PRODUCTION；其
TimingProfile、InterfaceLimits、能力、制动、海流、能源、传感器、通信与风险动作
数值只能用于联调，不能作为生产标定或验收证据。

未完成外部门禁明确包括：DAVE/Gazebo 双机联调、真实 ROS 2/MAVLink/FCU 闭环、
HIL、制动与跟踪阈值辨识、水池、海试、生产 profile 证据及最终安全审计。

---

# 29. 后续可选增强

V2.2 明确预留但不强制实现：

## 29.1 多拓扑

当复杂洞穴、狭窄水体等场景出现明显局部最优问题时，引入 K-best / homotopy 多候选。

## 29.2 SCP

当简单 constrained smoother 难以同时满足复杂动力学、观测和时序耦合时，再切换为 SCP。

## 29.3 完整 6DOF 在线验证

当实机参数辨识充分、算力允许时，用：

$$
M\dot\nu+C(\nu)\nu+D(\nu)\nu+g(\eta)=Bf+d
$$

替换能力包络的一部分保守限制。

## 29.4 概率风险预算

当地图无法提供确定性 FREE 时，单独引入概率风险模式。

概率模式必须在接口和日志中独立标识，不能把 confidence threshold 直接冒充确定性安全。

## 29.5 恢复流管

只有当项目确实需要更高级安全声明时，再引入离线恢复策略流管和在线集合包含验证。

---

# 30. 完成定义

V2.2 设计发布完成必须满足：

1. 本文到公共 schema、状态、代码、profile、hash 与 Manifest，以及反向映射均可追踪；
2. 搜索名称与数学状态一致，且不再使用旧 Hybrid 搜索器命名；
3. 主算法、结果、正交状态、依赖身份、原子授权、反馈与完成证据与公共 v1 一致；
4. schema 编译、hash/Manifest 与完整契约一致性套件通过，并在 ticket 12 记录身份；
5. 所有外部验证和生产标定继续作为未完成门禁。

后续基础算法与 Level 1 完成仍必须满足：

1. 同一输入可以确定性重放；
2. 混合三维地图能处理悬垂物和 UNKNOWN；
3. `TimeAwareStateLatticeAStar3d` 能生成合法三维路径；
4. 五次 Bézier 输出可唯一重建；
5. 轨迹满足 $C^2$；
6. 能力包络、海流和能源检查闭环；
7. 双机分离和通信距离检查闭环；
8. SurveyPlanEvidence 能计算真实保守覆盖率；
9. PlanSafetyValidator 能拒绝所有关键反例；
10. 只有安全验证通过的计划能够由前导权威原子签发 Bundle 与 lease；
11. lease 过期或关键依赖变化能异步撤销；
12. SafetySupervisor 在 planner 阻塞时仍能工作；
13. ROS2 adapter 不承担算法判断；
14. Level 1 确定性端到端场景全部通过；
15. 任何生产参数都没有伪默认值。

DAVE/Gazebo、HIL、水池、海试与生产参数证据分别属于 tickets 34-36；它们不是
V2.2 设计发布或 Level 1 算法完成的已完成证据，也不能被前述自动化测试替代。

---

# 31. 结论

V2.2 将前导机器人收敛为一套功能完整但不过度复杂、且跨边界语义闭合的工程规划系统：

$$
\boxed{
\begin{aligned}
&\text{混合3D地图}
+\text{时间感知3D搜索}\\
&+\text{规范五次轨迹}
+\text{车辆能力包络}\\
&+\text{双机距离/通信约束}
+\text{探测覆盖}\\
&+\text{统一安全验证}
+\text{短时执行授权}\\
&+\text{在线撤销}
+\text{最小风险动作}
\end{aligned}
}
$$

它保留了真正决定前导机器人能否完成任务的功能：

- 能找到路；
- 能避障；
- 能应对海流和平台能力限制；
- 能维持双机协同；
- 能完成指定区域探测；
- 能给铺缆侧提供地图证据；
- 能在环境变化时重规划；
- 能在异常时停止继续冒险执行。

同时不再把以下高级机制作为首版前置条件：

- 大规模多标签多拓扑；
- 强制 SCP；
- 在线完整 6DOF 集合证明；
- L1/L2 双层保证体系；
- 恢复流管连续覆盖。

因此，V2.2 的定位是：

> **先建立强工程闭环，再按真实仿真和实机需求逐级增加复杂度。**

这比为了理论完整性一次性堆叠全部高级机制，更适合作为当前前导机器人仓库的实际开发基线。

---

# 附录 A：核心公式

## A.1 对水速度

$$
v_{\mathrm{water}}
=
R_z^T(\psi)
(\dot p-\hat c).
$$

## A.2 ESDF 净空

$$
D_{\mathrm{esdf}}(p(t))
\ge
r_{\mathrm{body}}
+r_{\mathrm{loc}}
+r_{\mathrm{track}}
+r_{\mathrm{map}}
+r_{\mathrm{disc}}.
$$

## A.3 双机分离

$$
d_{\mathrm{SM}}(t)
\ge
d_{\mathrm{sep}}^{\min}.
$$

## A.4 通信距离

$$
d_{\mathrm{SM}}(t)
\le
d_{\mathrm{comm}}^{\max}.
$$

## A.5 能源

$$
E_{\mathrm{available}}
\ge
E_{\mathrm{plan}}
+
E_{\mathrm{return}}
+
E_{\mathrm{reserve}}.
$$

## A.6 Survey coverage

$$
C_{\mathrm{survey}}
=
\frac{
\operatorname{Vol}
(V_{\mathrm{required}}\cap V_{\mathrm{cover}}^{-})
}{
\operatorname{Vol}(V_{\mathrm{required}})
}
\ge C_{\min}.
$$

## A.7 Bézier

$$
p(s)
=
\sum_{j=0}^{5}
B_j^5(s)P_j.
$$

## A.8 制动距离

$$
d_{\mathrm{stop}}
=
vT_{\mathrm{latency}}
+
\frac{v^2}{2a_{\mathrm{brake}}^{\min}}
+
d_{\mathrm{margin}}.
$$

## A.9 曲线偏离弦线界

$$
\|p(t)-\ell(t)\|
\le
\frac{\Delta t^2}{8}
\max\|\ddot p(t)\|.
$$

---

# 附录 B：ScoutPlanningResult 主 Outcome 优先级

当多个失败同时存在时，主失败原因必须按以下规范优先级选择：

1. `OUTCOME_INPUT_INVALID`
2. `OUTCOME_DEPENDENCY_STALE`
3. `OUTCOME_CAPABILITY_INFEASIBLE`
4. `OUTCOME_ENERGY_INSUFFICIENT`
5. `OUTCOME_COORDINATION_INFEASIBLE`
6. `OUTCOME_NO_SOLUTION`
7. `OUTCOME_SMOOTHING_FAILED`
8. `OUTCOME_SURVEY_INFEASIBLE`
9. `OUTCOME_VALIDATION_REJECTED`
10. `OUTCOME_VALIDATION_INCONCLUSIVE`
11. `OUTCOME_TIMEOUT`
12. `OUTCOME_CANCELLED`
13. `OUTCOME_NUMERICALLY_INVALID`

诊断中可以同时记录全部次级原因，但公共结果只有一个主因。

---

# 附录 C：从 V2.0/V2.1 到 V2.2 的迁移规则

1. `AssuranceLevel L0/L1/L2` 不进入 V2.2 核心 API；
2. 原 `RecoveryCoverageVerifier` 从首版依赖图移除；
3. 原 `IndependentContinuousValidator` 收敛为 `PlanSafetyValidator`；
4. 原 `SafeCorridorBuilder` 收敛为 `TrajectorySmoother` 内部 `FeasibleTube`；
5. 原 `SpatiotemporalTopologyPlanner3d` 收敛为离散状态与预定义原语的
   `TimeAwareStateLatticeAStar3d`，不得称为经典 Hybrid A*；
6. 原 `CanonicalSplineOptimizer + SCP` 简化为 `TrajectorySmoother`，SCP 保留为可选后端；
7. 原完整在线 6DOF 鲁棒集合检查改为 `VehicleCapabilityEnvelope`；
8. 推进器分配复检保留为可选硬门禁；
9. 独立 lease 不再授权；只保留前导专用 `ScoutExecutionLease`，并由
   `ScoutAuthorizedExecutionBundle` 原子绑定计划、依赖、执行历元和 profile；
10. `SurveyPlanEvidence` 与 `SurveyCompletionEvidence` 保留；
11. `SafetySupervisor` 保留独立运行通道；
12. 旧的请求草图只保留为历史概念，公共唯一任务身份是 `ScoutMission`，内部
    只允许无损 `SurveyTask` 值对象；
13. 单一组合 `ScoutState` 被正交的任务、前导授权、两条通信通道与急停状态替代；
14. 所有被删除的高级机制均可在后续设计/profile 中增量恢复，不要求重写公共主接口。

---

# 附录 D：双向契约追踪矩阵

## D.1 设计章节到公共权威

| 设计章节 | 公共 schema / 注册表 | 规范与配置 | 精确 Manifest feature |
|---|---|---|---|
| 0-3 范围、输入输出、坐标、时间、身份 | `common.proto`、`cooperation.proto`、`planning.proto` | `CONTEXT.md`、`HASHING.md`、`SCOUT_MISSION_LIFECYCLE.md` | `scout_mission_lifecycle_v1`、`scout_4d_planning_result_v1` |
| 4 混合三维地图 | `mapping.proto`、`cooperation.proto`、`profiles.proto` | `HYBRID_MAP_SNAPSHOT.md` | `scout_hybrid_map_snapshot_v1` |
| 5.2 导航 | `state.proto`、`common.proto`、`profiles.proto` | `SCOUT_NAVIGATION_STATE.md` | `scout_navigation_state_v1` |
| 5.3、8.3 传感器与海流 | `sensing.proto`、`profiles.proto` | `SCOUT_SENSOR_AND_CURRENT.md` | `scout_sensor_and_current_v1` |
| 5.4、6 能力、推进器与能源 | `capability.proto`、`profiles.proto` | `SCOUT_CAPABILITY_AND_ENERGY.md` | `scout_capability_and_energy_v1` |
| 7 双机器人预测与协调 | `cooperation.proto`、`state.proto`、`profiles.proto` | `SCOUT_MAIN_ROBOT_COORDINATION.md` | `scout_main_robot_coordination_v1` |
| 8 任务、计划/完成证据 | `cooperation.proto`、`planning.proto` | `SCOUT_MISSION_LIFECYCLE.md`、`SCOUT_4D_PLANNING_RESULT.md` | `scout_mission_lifecycle_v1`、`scout_4d_planning_result_v1` |
| 9-14 搜索、轨迹、验证、排序 | `planning.proto`、`codes.proto` | `SCOUT_4D_PLANNING_RESULT.md`、`HASHING.md` | `scout_4d_planning_result_v1` |
| 15-16 授权、租约与复用 | `execution.proto`、`planning.proto`、`state.proto` | `SCOUT_AUTHORIZATION_BUNDLE.md`、ADR 0001/0003 | `independent_execution_authority_domains_v1`、`scout_authorization_bundle_v1` |
| 17 状态机 | `state.proto`、`codes.proto`、`diagnostics.proto`、`profiles.proto`、`registry/scout-state-transitions-v1.json`、`registry/codes-v1.json` | `SCOUT_STATE_CODES_PROFILES.md`、`state-machine-contract.md` | `scout_state_codes_profiles_v1`、`independent_execution_authority_domains_v1` |
| 18.1、19 规划主算法与结果 | `planning.proto`、`codes.proto` | `SCOUT_4D_PLANNING_RESULT.md`、`HASHING.md` | `scout_4d_planning_result_v1` |
| 18.2 授权主算法 | `execution.proto`、`planning.proto` | `SCOUT_AUTHORIZATION_BUNDLE.md`、`system-integration-contract.md` | `scout_authorization_bundle_v1`、`independent_execution_authority_domains_v1` |
| 18.3、24 执行监控、撤销与降级 | `execution.proto`、`codes.proto`、`diagnostics.proto`、`profiles.proto` | `SCOUT_EXECUTION_FEEDBACK_REVOCATION.md`、`SCOUT_STATE_CODES_PROFILES.md` | `scout_execution_feedback_revocation_v1`、`scout_state_codes_profiles_v1` |
| 20-21 软件 seam 与并发 | 上述强类型消息及 `MessageHeader` | `system-integration-contract.md`、各 adapter requirements | 上述全部相关 feature |
| 22-23 参数、时序、资源 | `profiles.proto`、`codes.proto` | `profiles/integration-v1.json`（NON_PRODUCTION）、`SCOUT_STATE_CODES_PROFILES.md` | `scout_state_codes_profiles_v1` |
| 25-30 验证、路线、范围、完成定义 | 全部公共 schema 与注册表 | `interfaces/tests/run.ps1`、`compatibility/contract-manifest-v1.json`、`project.md/PLAN.md` | Manifest 中全部 Scout features |

表内未带 `interfaces/` 前缀的公共文件均相对于系统根目录的 `interfaces/`；ADR 与
系统契约位于 `docs/`。

## D.2 公共权威到设计章节

| 公共权威 | 设计消费章节 | 设计约束 |
|---|---|---|
| `common.proto` / `HASHING.md` / Contract Manifest | 2、3、19、21、22、附录 E | frame、时钟域、会话/sequence/业务版本/内容身份正交，规范 SHA-256 与精确兼容门禁 |
| `ScoutMission` 生命周期与完成证据 | 1、2、8、18.4、19 | 公共任务身份唯一；发送方业务 deadline 不驱动 Scout 本地安全；完成必须绑定真实观测与严格更新地图 |
| `HybridMapSnapshot` | 2、4、9、10、13、15、24 | 五层不可变地图、UNKNOWN/STALE/CONFLICTED 失败关闭、版本变化复检 |
| `ScoutNavigationState` | 2、3、5、15、18、24 | 完整 3D ENU/FLU 事实、同域新鲜度、NED/FRD 只在 FCU adapter |
| 传感器/海流契约 | 5.3、8、13、18、22 | 几何与健康独立版本、海流运行域与误差界、未标定值不授权 |
| 能力/推进器/能源契约 | 5.4、6、9、11-13、18、22、24 | profile 不外推、健康变化复检、返航/风险动作与 reserve 硬门禁 |
| 预测/协调契约 | 7、9、13、18、24 | 连续三维占据、分离/几何通信硬边界、双向 LossPolicy 分离 |
| `ScoutPlanningResult` / `ScoutPlan` | 2.3、10-14、18.1、19、20 | 候选无授权；五次 Bézier、连续 yaw offset、完整依赖、独立验证与结果优先级 |
| `ScoutAuthorizedExecutionBundle` | 15、16、18.2、20-24 | 每域唯一权威、原子安装、固定历元、短前缀、profile 精确匹配、水位不复活 |
| Scout feedback / revocation / ACK | 15.5、18.3、20-24、26 | profile/applied/measured 三视图、精确因果身份、先持久化并本地停止、ACK 不授权 |
| 状态、代码、诊断、配置及转换注册表 | 12、17-19、22-24、附录 B | 正交状态、稳定 CodeRef、非法转换拒绝/审计、原子配置、单向恢复链 |

---

# 附录 E：V2.2 发布所绑定的公共基线身份

本设计发布绑定 `interfaces/compatibility/contract-manifest-v1.json` 当前声明的
以下身份；任何变更都要求重新执行完整契约一致性套件并评估是否发布新设计基线：

| 项目 | 发布身份 |
|---|---|
| Manifest | `underwater-contract-manifest/v1`，schema `1.0` |
| Protobuf descriptor SHA-256 | `44ded468a534e8c75722d98125d3eb49c2aebf3a7ab0f2ee8cecc251907cf7f1` |
| code registry | `underwater-system-codes` v1，`0974029fe79c9c55220e485fec1e8bdf2e800613207be8a3101a5b8008724199` |
| Scout state transition registry | `scout-state-transitions/v1` v1，`24cb6199d5d7bb89e4548ad2e417ffea6029e10b6b7e7f6eb050d9142bd3601e` |
| integration / Scout configuration | `integration/v1` / `scout/integration/v1` v1，`fbb887dfd6c176f9455648201711f29ed3c80bffcac61b5f3632da4f6d9670bf`，`production=false` |

精确 feature 集合为：

```text
scout_mission_lifecycle_v1
scout_hybrid_map_snapshot_v1
scout_navigation_state_v1
scout_sensor_and_current_v1
scout_capability_and_energy_v1
scout_main_robot_coordination_v1
scout_4d_planning_result_v1
independent_execution_authority_domains_v1
scout_authorization_bundle_v1
scout_execution_feedback_revocation_v1
scout_state_codes_profiles_v1
```

`approved_mixed_versions` 为空；能解析 schema 不代表安全语义兼容。上述
integration/Scout configuration 明确为 NON_PRODUCTION，不能证明真实车辆能力、
制动、时延、通信、传感器、能源、安全认证或生产就绪。

## Ticket 25 实施追踪

`ContinuousGeometryValidator` 在 `src/scout_core/` 独立消费不可变五次
Bézier 轨迹和混合三维地图。它使用导数控制多边形的解析速度、加速度、偏航
角速度和角加速度上界，并对每段采用弦线 `supercover` 与
`(Delta t)^2 a_max / 8` 曲线偏离界的自适应二分。占据、禁入水体、负净空和
海床/地图边界失败关闭；在最小区间或细分深度内仍不能证明时报告
`OUTCOME_VALIDATION_INCONCLUSIVE`，不会由稀疏采样放行。报告绑定轨迹与地图
content identity，并记录最小净空、最早失败时间、细分深度和检查区间数。

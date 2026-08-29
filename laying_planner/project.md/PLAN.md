# 水下铺缆机器人算法实施计划

> 设计基线：`ALGORITHM_DESIGN_REVISED.md` v3.0
> 计划状态：v3.0 设计发布完成；T48-T57 按各自状态继续
> 范围：只开发 `underwater_planner` 算法及其算法验证；不实现原始传感器处理、地图构建、底层运动控制或前置机器人完整三维规划。

## 1. 使用规则

本文件是后续算法开发的唯一进度入口，但不替代算法设计文档。实现细节与本计划冲突时，以设计基线为准；需要改变算法设计时，先修订设计文档并更新其版本，再调整本计划。

任务状态只使用以下四种：

- `[ ] ready-for-agent`：依赖已完成，可立即领取。
- `[ ] blocked`：仍有未完成的前置任务。
- `[ ] blocked-external`：算法已准备好，但缺少机器人参数、独立标定数据或试验条件。
- `[x] done`：验收标准全部通过，证据已记录。

每次开发结束必须同时完成：

1. 更新任务状态，不允许只按“代码已写”判定完成。
2. 在任务下追加一行 `Evidence:`，记录测试命令、测试报告或标定数据版本。
3. 若接口、参数或安全语义变化，更新设计章节追踪矩阵。
4. 若发现设计缺口，记录为新任务并声明阻塞边，不得用临时默认值绕过。

## 2. 全局完成定义

每个开发任务只有同时满足以下条件才可标记 `done`：

- 行为与所列设计章节一致，硬约束不能被软代价、降级开关或经验默认值覆盖。
- 正常、边界、无效输入和确定性复现测试全部通过。
- 版本、时间戳、运行域和风险语义出现在结果与诊断中，可审计且不可静默回退。
- 新代码通过编译、静态检查和相关测试；没有未解释的非有限值、隐式单位或未版本化参数。
- 算法结果不夸大保证：首版不声称实现整窗口联合走廊风险、整路径联合地形风险或高保真柔性缆线动力学。

## 3. 开发起点

本计划最初按**从零开发**执行。开发启动时只采用当时的 `ALGORITHM_DESIGN_REVISED.md` v2.6 实现基线，不继承、不评估也不依赖仓库内任何更早实现、测试、接口或历史进度，所有任务初始均视为未完成。D01-D10 已将当前设计基线发布为 v3.0；后续实现必须直接满足当前设计基线及本计划验收标准，不能因旧代码中存在同名模块而跳过任务。

## 4. 可执行任务

任务编号已按依赖拓扑排序。`Blocked by` 只列真正阻止任务开始的直接依赖。

### A. 从零建立工程与不可变契约

#### T01 - 从零建立算法工程与可重复测试骨架

**Status:** `[x] done`
**Blocked by:** None - can start immediately  
**What it delivers:** 从设计基线创建最小算法工程骨架，一条命令即可编译空白算法核心、运行新建测试并生成确定性的合成地图、参考线、机器人和缆线状态夹具。

- [x] 从零配置编译、单元测试、静态检查和测试报告入口，不接纳旧测试作为完成证据。
- [x] 新建测试失败输出包含随机种子、输入版本和单位信息。
- [x] 合成夹具覆盖平面、斜坡、台阶、障碍、未知区和参考线交叉。
- [x] 固定输入重复运行得到逐字段一致的结果。
- [x] 记录从零实现的初始编译、测试、内存和耗时数据，不引用历史实现的经验结果。

Evidence: `powershell -NoProfile -ExecutionPolicy Bypass -File tools/verify.ps1`；MSVC 19.51 + CMake 4.3.1 初始冷配置、编译、`/analyze` 与测试共 6.952 s，最终 CTest 1/1 通过（11 项行为检查，0.05 s），测试进程峰值 RSS 4,956 KiB；失败诊断包含 seed、输入版本、单位、时间戳、运行域和显式非安全保证风险语义，JUnit：`build/verify/ctest.xml`。

#### T02 - 固化算法公共数据契约与单位

**Status:** `[x] done`
**Blocked by:** T01 (completed)  
**What it delivers:** 设计第 3、13、14 节中的状态、路径、时序剖面、风险结果和诊断类型具有唯一语义，后续模块不再各自定义近似结构。

- [x] 表达机器人状态、缆线状态、参考进度、几何路径、时序路径、规划结果和错误预算。
- [x] 所有角度、长度、时间、曲率、速度、张力和协方差字段明确单位与有限性要求。
- [x] 状态枚举覆盖设计中的成功、无解、超时、输入无效、等待地图、包络失效和降级状态。
- [x] 类型测试覆盖默认构造、无效值拒绝、角度归一化和序列化往返。

Evidence: `powershell -NoProfile -ExecutionPolicy Bypass -File tools/verify.ps1`；MSVC 19.51 编译、`/analyze` 与 CTest 2/2 通过（公共契约 12 项行为检查，统一验证 3.973 s）；确定性 `UP_RESULT 1` 逐字段序列化往返与 golden-schema 指纹、未知 schema/枚举拒绝及 SI 单位/完整依赖版本/运行域/pointwise-only 风险诊断均通过，JUnit：`build/verify/ctest.xml`。

#### T03 - 实现分层参数配置与生产门禁

**Status:** `[x] done`
**Blocked by:** T02  
**What it delivers:** 机器人、地形、缆线、规划、稳定性、补探和任务参数按设计第 15、20 节加载并校验，生产模式无法使用空值或示例值冒充能力参数。

- [x] 支持生产能力配置与显式 `non_production_capability_profile`，输出携带非生产标记。
- [x] 实现能力、梯度风险、空间域、执行剖面、缆线机械、统计风险、路径复用和标签预算门禁。
- [x] 检查上下坡、爬阶/落差、机器人/缆线区域和各类 epsilon 的语义隔离。
- [x] 任一必填版本、标定数据集或物理参数缺失时给出结构化失败原因。

Evidence: `powershell -NoProfile -ExecutionPolicy Bypass -File tools/verify.ps1`; MSVC 19.51 + CMake 4.3.1 配置、严格编译、`/analyze` 与 CTest 3/3 通过（parameter gates 5 项行为检查，测试 0.10 s，总验证 6.441 s），覆盖生产/非生产标记、八组参数门禁、双空间域语义隔离、版本/标定缺失结构化 issue、确定性 dotted-key 配置加载及未知键/非有限数值拒绝，JUnit：`build/verify/ctest.xml`。

#### T04 - 实现版本化地图、参考线与双空间域快照

**Status:** `[x] done`
**Blocked by:** T02  
**What it delivers:** 一次规划只读取同一不可变地图版本、参考线版本、机器人作业区和缆线施工走廊，不会混合新旧数据。

- [x] 地图快照携带高程、方差、置信度、时间戳、派生配置版本和更新区域。
- [x] 参考线支持连续弧长、切向/法向、版本和局部窗口查询。
- [x] 机器人作业区与缆线走廊分别存在、非空、版本化且独立校验。
- [x] 重复、乱序、过期和版本回退输入被拒绝并产生诊断。

Evidence: `powershell -NoProfile -ExecutionPolicy Bypass -File tools/verify.ps1`; MSVC 19.51 + CMake 4.3.1 配置、严格编译、`/analyze` 与 CTest 4/4 通过，覆盖地图有限性/方差/置信度、连续弧长与局部窗口、双空间域独立校验，以及重复/过期/乱序/版本回退诊断，JUnit：`build/verify/ctest.xml`。

#### T05 - 捕获原子一致的规划与复检输入

**Status:** `[x] done`
**Blocked by:** T03, T04  
**What it delivers:** 机器人状态、缆线遥测、执行跟踪状态、地图和全部策略/模型版本只能作为同一同步快照参与安全决策。

- [x] 捕获期间任一字段或依赖版本变化会使整次快照无效。
- [x] 对每类输入执行最大年龄、时间同步容差和序列号检查。
- [x] 缺少或错配执行跟踪状态时返回 `VALIDATION_CONTEXT_INVALID`。
- [x] 禁止把捕获前后的字段拼成一个上下文，测试可稳定复现该竞态。

Evidence: MSVC 19.51 以 `/W4 /WX /permissive-` 编译并以 `/analyze` 检查 `synchronized_validation_inputs.cpp`；`ctest --test-dir build/verify --output-on-failure --output-junit D:/underwater-robot/build/verify/ctest.xml` 5/5 通过（T05 覆盖同批执行证据推进与整帧原子冻结、修订门禁、逐类年龄/未来时间戳、同步容差、所有输入及证据来源的非零/防回退序列、跟踪状态必填/剖面配对、完整依赖版本元组及确定性竞态复现；总测试 0.19 s），JUnit：`build/verify/ctest.xml`。

### B. 2.5D 地形与机器人通行性

#### T06 - 用鲁棒局部平面替换生产地形估计

**Status:** `[x] done`
**Blocked by:** T03, T04  
**What it delivers:** 含噪高程图可生成方向无关的梯度、梯度协方差、去趋势粗糙度、支撑覆盖率和明确拟合状态。

- [x] 使用物理尺寸窗口、方差/距离/时效权重和稳健损失拟合局部平面。
- [x] 输出有限且半正定的梯度协方差，并区分支撑不足与病态拟合。
- [x] 平滑斜面不会因坡度或窗口变大被误判为粗糙，支持 RMS 及抗尖峰统计量。
- [x] 平坦、多方向斜面、同粗糙度不同坡度、异常点和缺失数据测试符合第 18.2.1 节。

Evidence: MSVC 19.51 以 `/W4 /WX /permissive-` 从源码重建全部测试程序（40.897 s）；`ctest --test-dir build/verify --output-on-failure --output-junit D:/underwater-robot/build/verify/ctest.xml` 6/6 通过（T06 覆盖平坦与多方向斜面、方向无关坡度角、Huber 异常点抑制、物理窗口扩大、缺失/病态分类、方差与时效权重、有限对称半正定、不可表示及未收敛协方差、去趋势 RMS/P95、版本门禁及逐字段确定性；总测试 0.14 s），七个核心/支持源文件 `/analyze` 通过（20.585 s）。标准 `tools/verify.ps1` 在当前 VS 2026 CMake/Ninja 调度中于启动 `cl.exe` 前空转，故本次以相同严格编译选项直接调用 MSVC，并由已注册 CTest 清单生成 JUnit。

#### T07 - 提取方向无关的台阶几何

**Status:** `[x] done`
**Blocked by:** T06  
**What it delivers:** 保边处理后的地图输出完整台阶高度、低到高法向、边缘范围、过渡宽度和置信度，不受机器人航向影响。

- [x] 台阶两侧分别拟合支撑面，跨高置信边缘的单平面估计被拆分或标记不连续。
- [x] 台阶成立需满足双侧支撑、噪声显著性、最小高度和稳定法向条件。
- [x] 正交与斜向观察得到相同完整高度，绝不按接近角缩放。
- [x] 重复点、短边缘、未知侧和低置信度返回明确无效状态。

Evidence: `powershell -NoProfile -ExecutionPolicy Bypass -File tools/verify.ps1`；MSVC 19.51 严格编译与 `/analyze` 通过，CTest 7/7 通过（总测试 0.37 s，完整验证 6.438 s）；T07 覆盖保边中值去噪、双侧支撑面完整高度、正交/45 度低到高法向与范围、锐边/0.4 m 多栅格过渡宽度、跨边缘 `discontinuous`、基于拟合截距协方差的噪声显著性、最小高度/法向稳定/双侧覆盖门禁、重复端点/短边/紧邻未知侧/低置信明确状态、孤立尖峰抑制及逐字段确定性；聚焦台阶测试连续重复 20 次通过，JUnit：`build/verify/ctest.xml`。

#### T08 - 分离机器人碰撞误差预算并处理未知区域

**Status:** `[x] done`
**Blocked by:** T03, T04, T06  
**What it delivers:** 障碍检测、距离膨胀和未知区判定仅使用机器人相对障碍物协方差，候选路径能区分碰撞、低置信度和待补探缺口。

- [x] 有局部法向时使用方向裕量，否则使用最大特征值的各向同性保守上界。
- [x] 缆线协方差不进入机器人障碍膨胀，机器人碰撞 epsilon 与其他风险参数隔离。
- [x] 未知、低置信度和无有效地形默认不可通行，同时保留信息缺口位置。
- [x] 障碍膨胀保持原障碍不可通行，地图边界和非有限协方差安全失败。

Evidence: `powershell -NoProfile -ExecutionPolicy Bypass -File tools/verify.ps1`；MSVC 19.51 严格编译与 `/analyze` 通过，CTest 8/8 通过（总测试 0.52 s，完整验证 5.780 s）；T08 覆盖局部障碍法向协方差投影、法向缺失时最大特征值各向同性上界、独立 `epsilon_robot` 生产门禁、未知/低置信/无效地形逐位置缺口、相邻原障碍保留、地图边界禁行、非有限/非半正定协方差及无效法向 fail closed，并验证版本/运行域/标定集/pointwise-only 风险语义与逐字段确定性，JUnit：`build/verify/ctest.xml`。

#### T09 - 实现方向相关的足迹坡度评价

**Status:** `[x] done`
**Blocked by:** T03, T06, T08  
**What it delivers:** 给定机器人位姿或运动片段，完整足迹上的二维梯度及协方差按前向/横向投影形成保守上下坡与侧坡判定。

- [x] 先投影梯度再取 `atan`，上坡和下坡使用不同能力参数并保留符号。
- [x] 风险策略绑定分析配置、独立标定数据集、运行域和局部 epsilon。
- [x] 任一足迹样本上界越限、协方差无效或策略错配即拒绝整段。
- [x] 结果明确声明未实现整路径联合地形风险保证。

Evidence: `powershell -NoProfile -ExecutionPolicy Bypass -File tools/verify.ps1`；MSVC 19.51 严格编译与 `/analyze` 通过，CTest 9/9 通过（总测试 0.60 s，完整验证 6.151 s）；T09 8 组行为检查覆盖梯度先投影后取角、独立上下坡与侧坡能力、自适应整段扫掠/完整足迹及独立记录的坡度离散裕度（单姿态为零）、二维协方差投影及严格半正定门禁、分析配置/独立标定集/运行域/局部 epsilon/覆盖模型白名单绑定、无效地形与输入 fail closed、源地图版本/时间戳审计、local-pointwise-only 非联合风险声明及逐字段确定性，JUnit：`build/verify/ctest.xml`。

#### T10 - 完成台阶穿越与履带支撑评价

**Status:** `[x] done`
**Blocked by:** T07, T08, T09  
**What it delivers:** 完整扫掠足迹可正确区分爬阶、落阶、沿边骑跨和过渡区，并验证左右履带支撑、滚转与局部悬空风险。

- [x] 使用独立的完整外轮廓、左履带和右履带支撑多边形。
- [x] 低到高/高到低分别检查完整台阶高度与爬阶/落差上限。
- [x] 沿边和过渡区执行保守的完整高度与骑跨双重检查。
- [x] 任一履带覆盖不足、滚转超限或局部落差异常即不可通行，并报告限制因素。

Evidence: `powershell -NoProfile -ExecutionPolicy Bypass -File tools/verify.ps1`；MSVC 19.51 严格编译与 `/analyze` 通过，CTest 16/16 通过（总测试 1.34 s，完整验证 6.008 s）；T10 覆盖独立外轮廓/左右履带多边形、正交与 45 度完整台阶高度、独立爬阶/落差门禁、沿边/方向过渡双重检查、扫掠样本间角点连续接触/近邻不误判/接触航向分类、同段低-高-低往返事件与独立方向门禁、左右支撑覆盖/加权中位高程/滚转/局部落差/离群点、Analyzer 到 Evaluator 的真实不连续带侧别支撑链路、多台阶诊断顺序不变性、畸形输入 fail closed 及逐字段确定性，JUnit：`build/verify/ctest.xml`。

### C. 缆线状态、预测与机械约束

#### T11 - 跟踪实际缆线状态与规范化铺设记忆

**Status:** `[x] done`
**Blocked by:** T02, T03  
**What it delivers:** `CableStateTracker` 只根据已执行轨迹、放缆遥测和可选触地点观测持续估计滞后角、协方差及有限机械历史。

- [x] 未执行候选路径的终端状态永远不能写回当前状态。
- [x] 记忆至少保存最后两个不同触地点和覆盖支撑评价物理窗口的实际历史。
- [x] 启动、观测中断和状态丢失返回可审计的不确定初始化状态。
- [x] 滚动窗口起点预测连续，地图更新时历史位置重新查询当前地形而非缓存安全结论。

Evidence: `powershell -NoProfile -ExecutionPolicy Bypass -File tools/verify.ps1`；MSVC 19.51 严格编译与 `/analyze` 通过，CTest 11/11 通过（总测试 0.79 s，完整验证 10.081 s）；T11 10 组行为检查覆盖显式任务起点/状态丢失后机械历史门禁、实际执行段正反向滞后角与方差传播、触地点观测校正及不连续拒绝、基于触地点几何弧长的物理窗口、末端两个不同触地点、分段/整段滚动连续性、候选快照隔离、版本/标定集/运行域/风险语义审计及结构化诊断；聚焦 tracker 测试连续重复 20 次通过，地图高程与安全结论不进入 `CableConstraintMemory`，JUnit：`build/verify/ctest.xml`。

#### T12 - 跟踪单调参考线任务进度

**Status:** `[x] done`
**Blocked by:** T02, T04  
**What it delivers:** `ReferenceProgressTracker` 依据已执行铺设和局部关联传播连续任务进度，交叉参考线处不会跳到错误分支。

- [x] 进度不是每个搜索节点的全局最近点投影，短原语增量受设计上界约束。
- [x] 参考线版本变化、关联歧义和倒退请求产生明确状态。
- [x] 相同几何位姿在交叉前后具有不同任务阶段。
- [x] 局部切向、法向和走廊查询与进度使用同一参考线版本。

Evidence: `powershell -NoProfile -ExecutionPolicy Bypass -File tools/verify.ps1`；提交树从冷构建以 MSVC 19.51 严格编译并通过 `/analyze` 与 CTest 11/11（总测试 2.36 s，完整验证 26.698 s），含现有 T10 工作区改动的同命令亦通过 12/12（总测试 0.86 s，增量完整验证 9.176 s）；T12 11 组行为检查覆盖版本化生产参数映射、实际铺设单调推进与候选关联隔离、短原语增量上界、近邻竞争分支、交叉前后任务阶段、同分歧义、倒退/版本变化/无效输入/执行不连续诊断、同版本切法向与局部走廊查询及逐字段确定性；聚焦 tracker 测试连续重复 20/20 通过；双轴代码审查修正 tracker 候选职责混合、未接入参数门禁和未设计横向硬门禁后复核通过，JUnit：`build/verify/ctest.xml`。

#### T13 - 实现严格的几何路径与时序执行剖面契约

**Status:** `[x] done`
**Blocked by:** T02, T03  
**What it delivers:** 几何路径与未来执行条件明确分离，缆线预测只接受完整、单调、版本化的 `TimedPath`。

- [x] 几何样本包含同源的弧长、位置、航向和曲率。
- [x] 执行样本包含时间、对地速度、加速度、出缆速度/加速度和张力设定。
- [x] 时间不递增、字段缺失/非有限、弧长不一致或版本缺失均被拒绝。
- [x] 任一剖面样本或限制变化都会生成新执行剖面版本。

Evidence: `powershell -NoProfile -ExecutionPolicy Bypass -File tools/verify.ps1`；MSVC 19.51 严格编译与 `/analyze` 通过，CTest 12/12 通过（总测试 0.73 s，最终完整验证 4.554 s）；数据契约 14 项行为检查覆盖同源几何字段、独立采样的弧长域完整覆盖、从零严格递增相对时间、完整有限执行字段、批准速度/加速度/横向加速度/出缆/张力门禁、缺失版本/停止点/插值规则拒绝，以及剖面全部样本、限制、插值、停止点和运行包络变化的单调版本签发与修订拒绝；聚焦契约测试连续重复 20/20 通过；双轴代码复审无剩余发现，JUnit：`build/verify/ctest.xml`。

#### T14 - 实现统一缆线触地点均值模型

**Status:** `[x] done`
**Blocked by:** T11, T12, T13  
**What it delivers:** 搜索和最终验证从同一实际状态快照出发，使用相同空间滞后状态方程和放缆点偏置预测触地点均值与终端状态。

- [x] 支持直行、恒曲率、左右转对称和非零放缆点偏置。
- [x] 搜索只可降低积分精度，不得更换状态方程或使用机器人中心代理触地点。
- [x] 模型逐样本读取计划剖面的速度、出缆和张力，不把当前遥测外推为整段常数。
- [x] 搜索步长收敛到验证结果，输入几何路径而非时序路径时明确拒绝。

Evidence: `powershell -NoProfile -ExecutionPolicy Bypass -File tools/verify.ps1`；MSVC 19.51 严格编译与 `/analyze` 通过，CTest 13/13 通过（总测试 0.88 s，完整验证 11.381 s）；T14 7 组行为检查覆盖直线/恒曲率/左右转对称、非零放缆点偏置、搜索与验证共享空间滞后方程、粗细积分子步输出及搜索收敛、逐样本速度/出缆/张力与认证跟踪误差门禁、实际快照连续性、纯几何验证入口编译期拒绝、版本错配/中间滞后越界 fail closed、参数版本更新及逐字段确定性；聚焦模型测试连续重复 20/20 通过；双轴代码审查修正认证门限、积分子步可见性、中间状态门禁、首样本连续性、重复传播路径和范围 primitive 后复核通过，JUnit：`build/verify/ctest.xml`。

#### T15 - 传播触地点协方差并执行模型有效性门禁

**Status:** `[x] done`
**Blocked by:** T03, T14  
**What it delivers:** 最终验证沿完整时序轨迹传播实际路径相关协方差，并在传感器、张力、出缆或模型运行域越界时停止宣称落点约束有效。

- [x] 协方差包含初始状态、机器人位姿/跟踪和模型过程噪声，保持有限与半正定。
- [x] 相同终端基础状态但不同历史允许得到不同实际协方差。
- [x] 模型返回细分有效性状态和完整版本元组。
- [x] 触地点协方差不进入机器人障碍膨胀。

Evidence: `powershell -NoProfile -ExecutionPolicy Bypass -File tools/verify.ps1`；MSVC 19.51 严格编译与 `/analyze` 通过，CTest 13/13 通过（总测试 0.87 s，完整验证 6.417 s）；T15 12 组缆线模型行为检查覆盖手算初始协方差、六维联合输入及交叉项的 `J Σ J^T`、细步对齐的有限/对称/尺度敏感半正定剖面、滞后与机器人航向跟踪过程噪声、模型参数灵敏度、同终端均值不同跟踪历史、认证横向加速度节点与区间解析极值、传感器/运行域/执行包络/协方差细分门禁、包络生成器与不确定性剖面在内的完整依赖版本元组及逐字段确定性，聚焦测试连续重复 20/20 通过；生产参数门禁拒绝缺失或任意尺度非半正定过程噪声，机器人障碍膨胀接口仍只接收独立的机器人相对障碍物协方差，JUnit：`build/verify/ctest.xml`。

#### T16 - 实现缆线机械硬约束与软适宜性

**Status:** `[x] done`
**Blocked by:** T03, T06, T11, T14  
**What it delivers:** `CableLayingEvaluator` 从实际铺设记忆增量评价触地点轨迹，分别输出硬可行性、失败区段、软代价和终端记忆。

- [x] 左右曲率按绝对值检查机械上限，偏好曲率只产生软代价。
- [x] 禁放区、障碍占地区、未知/低置信地形和重复触地点构成硬失败。
- [x] 固定物理长度后向窗口计算保守悬空代理，历史不足使用显式起点边界策略。
- [x] 采样间隔变化不改变曲率、窗口起伏和硬判定；权重设零也不能绕过硬约束。

Evidence: `powershell -NoProfile -ExecutionPolicy Bypass -File tools/verify.ps1`；MSVC 19.51 严格编译与 `/analyze` 通过，CTest 14/14 通过；T16 17 组行为检查覆盖左右绝对曲率与偏好软代价、坐标系/双向弧长/未知插值 fail-closed、固定物理间距下的共线重采样曲率不变性、转角保守性及任意原语边界增量等价、禁放/障碍/未知/低置信栅格 supercover、重复点拒绝、固定 2 m 物理窗口采样不变性与窗口外历史裁剪、实际历史在当前地图重查、Tracker→Evaluator 曲率记忆契约、显式任务起点有效窗口、首段历史边界曲率、终端记忆弧长、软权重隔离、逐样本机械记忆等价及逐字段确定性；聚焦 evaluator 测试连续重复 20/20 通过；`UP_RESULT 3` 往返保留成功和无效评价的失败区段、限制版本、地图序列、地形分析配置版本、运行域和首版保守悬空代理风险语义，设计基线 v2.4 经双轴代码审查复核，JUnit：`build/verify/ctest.xml`。

#### T17 - 实现触地点走廊风险评价

**Status:** `[x] done`
**Blocked by:** T03, T04, T12, T15  
**What it delivers:** `CableCorridorEvaluator` 以触地点而非机器人中心计算横向误差、协方差投影、保守机会约束和 PASS/MARGINAL/VIOLATION 分级。

- [x] 参考线首版按确定量处理，但坐标变换误差必须已进入触地点协方差。
- [x] `epsilon_point`、走廊策略版本和残差分布标定缺失时评价无效。
- [x] MARGINAL 按轨迹与区间的弧长交集累计，超过强制上限即拒绝且无非严格模式。
- [x] 结果逐点记录均值误差、横向标准差、上界、等级和依据，并声明未实现 `epsilon_path`。

Evidence: `powershell -NoProfile -ExecutionPolicy Bypass -File tools/verify.ps1`；MSVC 19.51 严格编译与 `/analyze` 通过，CTest 15/15 通过（完整验证 6.131 s）；T17 5 组行为检查覆盖触地点有符号横向误差、实际协方差法向投影、双侧高斯保守分位上界、PASS/MARGINAL/VIOLATION 精确边界、确定参考线与坐标变换误差证据、策略/标定/极端尾概率 fail-closed、非有限硬边界、非对称/小尺度及大尺度非半正定协方差、参考线框架/版本/样本对齐门禁、版本化逐区间认证上界误差与正离散边界裕量、两个 PASS 端点间隐藏 VIOLATION 的粗细采样一致拒绝、MARGINAL 累计长度强制拒绝、评价时间戳/运行域/逐点依据和 pointwise-only 风险语义；逐点评价结果不声称已完成统计包络审计，T19/T29 审计前不能发布为有效计划；聚焦测试连续重复 20/20 通过，`UP_RESULT 5` 逐字段往返与 golden-schema 指纹 `3770366873757127612` 通过，设计基线 v2.4 第 6.6、7.5、14、19 节复核通过，JUnit：`build/verify/ctest.xml`。

### D. 搜索用统计包络

#### T18 - 构建有证明依据的横向不确定性包络

**Status:** `[x] done`
**Blocked by:** T03, T15, T17  
**What it delivers:** `CableUncertaintyEnvelopeBuilder` 在版本化设计运行域内生成按参考进度查询的横向标准差保守上界，而不依赖某条候选历史。

- [x] 运行域包含规划长度/时间、初始不确定性、原语、传感器模式和认证执行运行包络。
- [x] 生成过程保留所有不可比较可达集合，只按集合包含或已证明风险上界剪枝。
- [x] 包络包含分箱、积分、扫掠和统计分位误差的独立裕量。
- [x] 输出记录生成器、模型、执行包络、参考线、传感器模式、运行域及数据集版本。

Evidence: `powershell -NoProfile -ExecutionPolicy Bypass -File tools/verify.ps1`；MSVC 19.51 严格编译与 `/analyze` 通过，CTest 16/16 通过（完整验证 6.649 s）；T18 9 组行为检查覆盖版本化长度/时间/初始不确定性/原语/传感器/完整执行子包络、任意相关性下的确定性横向方差上界、实际 `CableModel` 在两种批准传感器模式下的进度对齐覆盖、非可比历史保留与集合包含剪枝证据、原语中段进度扫掠、部分滞后区间求交、解析分箱裕量及积分/法向扫掠/统计分位独立认证余项、执行不确定性与包络版本绑定、结构化时间戳诊断、资源耗尽 fail closed、完整生成依赖和逐字段确定性；聚焦测试连续重复 20/20 通过，输出明确声明仅提供逐点包络、不提供整路径联合风险保证，设计基线 v2.4 第 7.5.1、14.2、18.2.4 节经双轴代码审查复核，JUnit：`build/verify/ctest.xml`。

#### T19 - 锁定、查询并失效统计包络

**Status:** `[x] done`
**Blocked by:** T04, T18  
**What it delivers:** `CableUncertaintyEnvelopeManager` 只返回与当前完整版本元组逐项匹配且通过覆盖审计的包络，并能原子撤销所有依赖结果。

- [x] 进度分段查询保守覆盖相邻端点及离散裕量。
- [x] 任一版本、传感器健康或运行域变化立即使包络、计划和租约失效。
- [x] 实际横向标准差超过包络加审计容差时返回 `COVARIANCE_ENVELOPE_BREACH`。
- [x] 不允许仅凭相同 `operating_domain_id` 接受旧模型或旧执行包络。

Evidence: `powershell -NoProfile -ExecutionPolicy Bypass -File tools/verify.ps1`；MSVC 19.51 严格编译与 `/analyze` 通过，CTest 17/17 通过（总测试 1.45 s，完整验证 8.148 s）；T19 6 组行为检查覆盖独立覆盖审计与精确包络版本/完整生成依赖绑定、认证有效期和过期级联撤销、完整五字段版本元组锁定、相邻进度端点最大值与认证离散余项保守查询、参考线/传感器模式/运行域/缆线模型/执行运行包络逐项变化、单调上下文序列与版本回退拒绝、同键新包络版本替换、包络/计划/租约同锁级联失效、实际横向标准差审计容差精确边界及 `COVARIANCE_ENVELOPE_BREACH` 停车语义、旧模型/旧执行包络隔离、缺失生成依赖拒绝和不可变锁定快照代次防复用；聚焦测试连续重复 20/20 通过，输出明确声明 pointwise-only 且不提供整路径联合风险保证，设计基线 v2.4 第 7.5.1、7.7.3、10.5、13、14.2、15、18.2.4 节复核通过，JUnit：`build/verify/ctest.xml`。

### E. 参考线引导 Hybrid A* 搜索

#### T20 - 反解多个触地点并线目标

**Status:** `[x] done`
**Blocked by:** T12, T14  
**What it delivers:** 多个参考进度处的期望触地点可反解为机器人目标状态，并通过正向缆线模型确认接入质量。

- [x] 非零放缆点偏置和触地点距离下，机器人目标与触地点目标正确分离。
- [x] 候选覆盖不同并线距离与可行航向/滞后角边界。
- [x] 目标在机器人作业区外或正向预测不命中时被拒绝。
- [x] 候选排序只使用可行性与明确软代价。

Evidence: `powershell -NoProfile -ExecutionPolicy Bypass -File tools/verify.ps1`；MSVC 19.51 严格编译与 `/analyze` 通过，CTest 18/18 通过（总测试 1.69 s，完整验证 7.940 s）；T20 7 组行为检查覆盖 `CableModel` 单一公式来源下的非零放缆点偏置/触地点距离反解与正向闭环、不同并线距离、标定滞后角边界、参考切向航向配对、未来候选进度语义、完整终端足迹在凸/凹机器人作业区的硬门禁、终端地形通行性及风险策略绑定、参考进度/版本 fail closed、显式距离与滞后角软成本、稳定排序/目标数量上限及地图/标定集/运行域/空间域审计，聚焦测试连续重复 20/20 通过；设计基线 v2.4 第 7.3、7.4、8.7、14.2、14.3、18.2.3 节复核通过，JUnit：`build/verify/ctest.xml`。

#### T21 - 打通平坦直线的增广 Hybrid A* tracer bullet

**Status:** `[x] done`
**Blocked by:** T10, T14, T17, T19, T20 (completed)  
**What it delivers:** 在平坦无障碍场景中，从当前机器人/缆线/参考进度状态搜索出一条触地点贴近参考线的可验证路径。

- [x] 基础状态键包含离散位置、航向、滞后角和参考进度。
- [x] 运动原语逐步传播机器人、缆线均值和任务进度。
- [x] 搜索使用锁定的统计包络评价走廊，不传播路径相关实际协方差。
- [x] 固定输入、参数和队列规则得到确定性路径与诊断。

Evidence: `powershell -NoProfile -ExecutionPolicy Bypass -File tools/verify.ps1`；MSVC 19.51 严格编译与 `/analyze` 通过，CTest 19/19 通过（总测试 1.67 s，完整验证 10.986 s）；T21 8 组行为检查覆盖平坦直线触地点/机器人偏置、五元基础键、恒曲率原语下机器人/缆线均值/局部任务进度逐步传播、完整锁定包络依赖匹配、起点与原语触地点走廊硬门禁、无路径相关实际协方差、风险分位数/包络离散裕量/缆线扫掠裕量独立审计、逐字段确定性指纹、版本错配 fail closed、初始模型域失效分类、扩展预算 `TIMEOUT` 及名义/绝对走廊宽度策略一致性；聚焦测试连续重复 20/20 通过，双轴代码审查修正预算耗尽误报、风险裕量审计、共享走廊上界公式、共享搜索走廊策略校验及基础键比较后复核；设计基线 v2.4 第 7.5.1、7.7.1、7.7.2、8.1-8.3、8.6、14.2、18.2.3 节核实通过，T22 自适应扫掠、T23 机械历史多标签及 T24 完整代价/Dubins 启发式保持后续任务边界，JUnit：`build/verify/ctest.xml`。

#### T22 - 对运动原语执行自适应连续扫掠

**Status:** `[x] done`
**Blocked by:** T10, T16, T17, T21 (completed)  
**What it delivers:** 每条原语在机器人足迹、地形、缆线触地点和走廊四个独立尺度上自适应采样，端点合法但中间违法的原语无法通过。

- [x] 样本间距由位姿变化、足迹扫掠、地图分辨率和曲率共同约束。
- [x] 对全扫掠足迹执行碰撞、方向坡度、台阶和支撑检查。
- [x] 对完整触地点段执行机械记忆、禁放区、悬空代理和独立走廊离散裕量。
- [x] 原语长度或采样密度变化时软成本积分和硬判定在容差内一致。

Evidence: `powershell -NoProfile -ExecutionPolicy Bypass -File tools/verify.ps1`；MSVC 19.51 严格编译与 `/analyze` 通过，CTest 19/19 通过；T22 16 组 Hybrid A* 行为检查覆盖恒曲率解析圆弧内部样本、版本化 `eta` 与原语弧长/航向变化/足迹半径/地图分辨率共同约束的最大扫掠间距、固定 `eta*r/2` 碰撞离散裕量、完整足迹碰撞栅格、中途障碍及方向坡度硬拒绝、起点足迹预检、凹形作业区样本间窄缺口、地图/地形栅格几何错配 fail closed、逐样本锁定包络走廊门禁、中途绝对走廊越界、完整触地点禁放区/机械硬约束、终端机械记忆传播、机械软成本累加、1 m/2 m 原语及 0.25 m/0.02 m 缆线积分密度下硬判定和解代价一致、扫掠依赖版本与确定性指纹审计；双轴代码审查修正地图/地形物理栅格绑定、作业区扫掠净距和碰撞裕量契约并复核；设计基线 v2.4 第 5.7、6.2、7.4、7.5.1、8.2、8.3、8.3.1、8.4 节核实通过，T23 机械历史多标签与安全支配保持后续任务边界，JUnit：`build/verify/ctest.xml`。

#### T23 - 实现机械历史多标签与安全支配

**Status:** `[x] done`
**Blocked by:** T11, T12, T16, T22  
**What it delivers:** 同一基础搜索键可保留未来机械行为不等价的标签，仅在严格证明 `futureEquivalent` 后按成本替换。

- [x] 规范化记忆包含跨原语曲率所需触地点和物理长度地形窗口。
- [x] 签名只用于候选筛选，最终等价性逐样本核对，哈希碰撞不会误合并。
- [x] 低成本等价标签可重开节点，旧队列项按稳定标签标识跳过。
- [x] 活动标签或时间预算耗尽返回区分原因的 `TIMEOUT`，不得任意保留前 K 个。

Evidence: `powershell -NoProfile -ExecutionPolicy Bypass -File tools/verify.ps1`；MSVC 19.51 严格编译与 `/analyze` 通过，CTest 19/19 通过；Hybrid A* 25 组行为检查覆盖同一五元基础键下不等价机械历史并存、严格逐样本等价支配、真实高成本弯曲历史保留并到达唯一后继目标、低成本等价标签重开和稳定 `label_id` 旧队列项跳过、实际铺设历史不完整时起点目标也不得绕过首标签门禁、全局活动标签预算与单调时钟期限的独立 `TIMEOUT` 原因、P50/P95/P99 与峰值标签统计及确定性指纹，聚焦测试连续重复 20/20 通过；CableLayingEvaluator 19 组检查覆盖最后两个不同触地点、`max(L_support, 2 L_kappa)` 固定物理窗口、稀疏边界段精确插值裁剪、初始记忆规范化、陈旧签名/有符号零重算及模拟哈希碰撞逐样本拒绝；`epsilon_g` 配置加载和生产门禁通过；双轴终审确认无阻碍 T23 完成的规范或规格问题；设计基线 v2.4 第 7.7.1、7.7.2、8.8、15.2、17.1、18.2.3、20.6 节核实通过，JUnit：`build/verify/ctest.xml`。

#### T24 - 完成硬约束优先的代价与启发函数

**Status:** `[x] done`
**Blocked by:** T09, T16, T17, T23  
**What it delivers:** 全部硬约束先剪枝；可行域内按触地点走廊质量、机器人长度/转向、地形和缆线软适宜性进行一致积分与排序。

- [x] 不重复计算机器人到缆线参考线的代理偏差成本。
- [x] 启发函数不高估未完成代价，参考线引导不越过硬边界。
- [x] 可爬地形可通过，机械/走廊/作业区硬约束不可被权重覆盖。
- [x] 记录各成本分量、剪枝原因和最坏约束位置。

Evidence: `powershell -NoProfile -ExecutionPolicy Bypass -File tools/verify.ps1`；MSVC 19.51 `/W4 /WX` 严格编译与 `/analyze` 通过，CTest 19/19 通过（JUnit：`build/verify/ctest.xml`）；Hybrid A* 33 组行为检查覆盖五类显式成本分量、软成本仅在全部硬门禁通过后累计、机器人中心代理偏差反例、触地点走廊密度按机器人原语弧长积分、目标位置/航向容差集上的可接受运动学下界与六族前进 Dubins 精确目标距离、可爬地形软排序、碰撞/作业区/地形/走廊/机械硬剪枝、缆线模型/参考关联/包络不可用独立剪枝计数、起点硬失败和真实最大机械量的约束位置及确定性指纹，聚焦测试连续重复 20/20 通过；设计基线 v2.4 第 6.6、6.7、8.1、8.3.1、8.4、8.5、8.6、18.2.3 节核实通过。

#### T25 - 完成多目标搜索、回归参考线与搜索优化

**Status:** `[x] done`
**Blocked by:** T20, T24  
**What it delivers:** 搜索能在单侧/双侧障碍、狭窄通道和参考线交叉场景选择合适绕行侧并平稳回归可行并线目标。

- [x] 支持多个并线点联合搜索和经完整约束验证的解析扩展。
- [x] 无解时区分普通无解、统计包络下无解、标签预算耗尽和截止时间耗尽。
- [x] 可采用增量地图更新、自适应分辨率和早停，但不改变安全结论。
- [x] 平坦、单侧、双侧、狭窄通道和代理量反例场景通过。

Evidence: `powershell -NoProfile -ExecutionPolicy Bypass -File tools/verify.ps1`；MSVC 严格编译与 `/analyze` 通过，CTest 38/38 通过（总验证 444.327 s，JUnit：`build/verify/ctest.xml`）；当前 Hybrid A* 测试输出 46 组行为检查通过（seed=2121，`input_version=t25-multi-goal-analytic/v1`，SI，pointwise-envelope-only），覆盖多并线目标联合排序、1 cm 级完整增广目标命中、周期六族前进 Dubins 多段解析扩展及逐段完整门禁、解析中段障碍拒绝、普通/统计包络无解与扩展/标签/截止时间预算耗尽独立诊断、平坦/单侧/双侧软成本选择/受限通道/参考线交叉/机器人代理量反例及确定性诊断指纹；聚焦测试连续重复 20/20 通过。双轴代码审查初检发现并修正混合无解误归因、单段解析覆盖不足、场景容差过宽和新测试夹具重复，复审 Standards 0、Spec 0；设计基线 v2.4 第 8.1-8.8、17.1、18.2.3 节核实通过，未启用增量地图、自适应分辨率或早停优化，所有解析段仍使用锁定版本和同一完整安全验证链。

### F. 平滑、时序化与最终候选验证

#### T26 - 实现带 G2 边界的曲率剖面平滑

**Status:** `[x] done`
**Blocked by:** T03, T25 (completed)  
**What it delivers:** 原始搜索路径在拓扑信赖域内转换为满足起终位置、航向、曲率、最大曲率和空间曲率变化率的可跟踪路径。

- [x] 从同步实际曲率或承诺段终点读取 G2 起始边界，不默认零曲率。
- [x] 优化器报告成功后仍独立检查动力学、边界和求解残差。
- [x] 左右曲率对称、重采样一致，输出航向与曲率来自同一参数曲线。
- [x] 平滑目标不加入机器人中心到缆线参考线的重复代价。

Evidence: `powershell -NoProfile -ExecutionPolicy Bypass -File tools/verify.ps1`；MSVC 严格编译与 `/analyze` 通过，CTest 20/20 通过；PathSmoother 18 组行为检查覆盖同步实际/承诺段非零 G2 起始曲率、位姿与曲率时间同步门禁、分段 clothoid 同源航向/曲率、曲率与空间变化率对称硬门禁、加权偏差/曲率/变化率目标的受约束优化、默认求解器终端修正与目标优化全阶段截止时间传播、求解器假收敛独立残差拒绝、按 `constant-curvature-exact` 原始插值构建的拓扑管节点及段内越界拒绝、带四阶导数误差界的自适应 Simpson 积分、原始与输出重采样一致性、超时无原始路径回退、无机器人中心参考线代理代价、版本化平滑审计元数据及 `UP_RESULT 7` 往返；聚焦测试连续重复 20/20 通过；双轴代码审查发现并修正目标权重仅审计、边界同步证据不足、输出采样测试缺失、重复残差类型与求解状态、目标优化失败/超时误报成功、原始恒曲率弧错误按弦建管、固定积分无误差界、插值自由字符串分支和过期验证证据，最终复审 Standards 0、Spec 0；设计基线 v2.4 第 9.1-9.5、9.7、14、18.2.5 中 T26 职责核实通过，独立三点几何审计、完整扫掠/地形复检、G2 拼接与原始路径同等回退验证按计划保留给 T27，JUnit：`build/verify/ctest.xml`。

#### T27 - 实现独立几何审计与完整路径复检

**Status:** `[x] done`
**Blocked by:** T10, T26 (completed)  
**What it delivers:** 平滑路径或原始回退路径只有通过同一 G2、曲率、扫掠和作业区契约后才能进入时序参数化。

- [x] 从几何重新计算航向、曲率和变化率，篡改元数据会被发现。
- [x] 拼接后的完整路径而非仅新尾段执行通行性和边界复检。
- [x] 平滑失败不会自动发布未经同等验证的原始路径。
- [x] 边界残差和最坏扫掠位置包含在诊断中。

Evidence: `powershell -NoProfile -ExecutionPolicy Bypass -File tools/verify.ps1`; MSVC 19.51 严格编译、`/analyze` 与 CTest 21/21 通过（完整验证 11.893 s，T27 测试 0.05 s，JUnit：`build/verify/ctest.xml`）。新增 `PathGeometryAudit` 独立重算航向/三点有符号曲率/最大曲率变化率并检查 G2 边界时间同步与残差；`mergePathsG2` 拒绝位置/航向/曲率错位并清除旧段平滑审计；`PathCandidateVerifier` 对整条候选路径执行自适应作业区、碰撞扫掠和方向地形/台阶复检，分别暴露最坏样本位置；平滑候选和原始回退路径只能分别通过同一验证接口，未实现自动放行回退。设计基线 v2.4 第 9.4、9.6、9.7、18.2.5 与 T27 验收条目核实通过。

#### T28 - 生成受能力与停车距离约束的时序剖面

**Status:** `[x] done`
**Blocked by:** T03, T09, T13, T27  
**What it delivers:** `TrajectoryParameterizer` 保持几何不变，为每点生成满足速度、加减速、横向加速度、停车、出缆和张力限制的版本化执行剖面。

- [x] 前向/后向传播速度限制并验证 `v^2|kappa|`、制动距离和终端停车条件。
- [x] 出缆速度/加速度、跟踪误差和张力样本均处于模型及认证执行运行包络。
- [x] 请求减速生成新剖面而非控制器倍率，几何和 G2 审计保持不变。
- [x] 无法在可用距离停车或任何样本越界时参数化失败。

Evidence: `powershell -NoProfile -ExecutionPolicy Bypass -File tools/verify.ps1`; MSVC 19.51 严格编译、`/analyze` 与 CTest 22/22 通过（总测试 4.54 s，完整验证 8.49 s）；T28 测试覆盖几何逐点不变、版本化完整执行剖面、前向/后向速度与 `v^2|kappa|` 门禁、停车距离不可行、放缆跟踪/加速度/张力边界、缺失认证包络和确定性输出。已核实设计基线 v2.4 第 7.6、7.7.3、18.2.6、15.1 与 TrajectoryParameterizer 接口契约：仅成功返回 TimedPath，首版仍不声称整路径联合风险保证。

#### T29 - 对完整时序候选执行高精度缆线复检

**Status:** `[x] done`
**Blocked by:** T15, T16, T17, T19, T28
**What it delivers:** 拼接后的完整时序轨迹从当前缆线状态重新预测，并同时通过模型有效性、包络审计、机械硬约束和走廊门禁。

- [x] 不复用搜索阶段或旧计划缓存的缆线路径与协方差。
- [x] 任何 `VIOLATION`、超额 MARGINAL、机械失败或模型无效都拒绝候选。
- [x] 包络 breach 使包络及全部依赖计划失效并进入停车路径。
- [x] 候选保存终端缆线状态、终端机械记忆和完整误差预算。

Evidence: `powershell -NoProfile -ExecutionPolicy Bypass -File tools/verify.ps1`; MSVC 19.51 严格编译、`/analyze` 与 CTest 23/23 通过（总测试 5.10 s，完整验证 11.682 s，JUnit：`build/verify/ctest.xml`）。新增 `TimedCableCandidateVerifier` 只接受完整 `TimedPath`，从实际 `CableState` 重新调用 `CableModel::predict`，逐点审计锁定包络并在 breach 时返回 `covariance_envelope_breach`/`stop_required`，随后执行完整走廊和 `CableLayingEvaluator` 硬门禁并保留预测终态、终端机械记忆与 pointwise-only 风险语义；生产输入缺失 fail-closed 测试通过。设计基线 v2.4 第 7.6、7.7.3、9.6、16 节与 T29 验收条目已核实；整路径联合风险保证仍明确未实现。

### G. 稳定重规划与执行租约

#### T30 - 构造不可变且完整版本化的规划结果

**Status:** `[x] done`
**Blocked by:** T05, T29  
**What it delivers:** 每个成功候选具有唯一计划序列、有效期、几何/时序/缆线路径和全部上下文版本，但尚不隐含执行授权。

- [x] 元数据覆盖地图、参考线、双空间域、风险策略、缆线模型、统计包络、执行包络和执行剖面版本。
- [x] 错误预算分别记录机器人碰撞、地形局部风险、缆线横向风险和未实现的联合风险。
- [x] 结果不可原地修改，重参数化或策略变化产生新计划。
- [x] 缺少任一生产必填版本时不能标记 `SUCCESS`。

Evidence: `powershell -NoProfile -ExecutionPolicy Bypass -File tools/verify.ps1`; MSVC 19.51 严格编译、`/analyze` 与 CTest 24/24 通过（总测试 5.21 s，完整验证 9.677 s，JUnit：`build/verify/ctest.xml`）。新增 `PlanningResultPublisher` 在完整 `validate(PlanningResult)` 通过后复制为 `ImmutablePlanningResult`，严格拒绝重复/回退序号并保留完整地图、参考线、双空间域、策略/模型/包络/剖面版本及 `ErrorBudget` 的既有审计语义；新增确定性发布边界测试覆盖别名隔离、版本单调和生产必填版本 fail-closed。已核对设计基线 v2.4 第 9.7、10.5、13.3-13.4、14.3 及 T30 验收条目；结果不携带执行授权，整路径联合风险仍显式未实现。

#### T31 - 复检剩余计划并签发短期执行租约

**Status:** `[x] done`
**Blocked by:** T05, T19, T30  
**What it delivers:** `PlanValidityEvaluator` 从当前同步状态裁剪剩余剖面、重新预测缆线并复检所有硬约束，只有成功才签发与计划和剖面配对的租约。

- [x] 复检不读取缓存缆线路径，不能通过更新时间戳原地续期。
- [x] 输入年龄、上下文版本、剖面连续性和剩余停车距离均纳入判定。
- [x] 返回 `REUSE/REPLAN/STOP` 动作、细分状态、剩余轨迹和新租约。
- [x] 租约序列、计划序列、剖面版本、截止时间和允许执行偏差不可缺失。

Evidence: `powershell -NoProfile -ExecutionPolicy Bypass -File tools/verify.ps1`；MSVC 19.51 严格编译、`/analyze` 与 CTest 25/25 通过（总测试 4.75 s，完整验证 9.547 s，JUnit：`build/verify/ctest.xml`）。`PlanValidityEvaluator` 从不可变 `PlanningResult` 与当前原子同步输入裁剪并重锚剩余 `TimedPath`，从当前实际 `CableState.laying_memory` 重新预测，按固定顺序复检版本、时效/同步来源、剖面连续性、动力学/停车、G2/完整足迹/地形、统计包络、走廊和机械硬约束；成功租约绑定严格递增租约序列、计划/剖面及全部依赖版本、带版本与运行域的 evaluator 配置、最短有效期和执行偏差边界，失败不消耗序列。已逐项核对设计基线 v2.4 第 10.5、13.3-13.4，并核对第 7.7、9.6、14.3 的复检与数据契约；整路径联合走廊/地形风险仍显式未实现。

#### T32 - 异步监控执行偏差并撤销租约

**Status:** `[x] done`
**Blocked by:** T31  
**What it delivers:** `ExecutionLeaseMonitor` 独立于规划循环检查计划/租约配对和实测执行偏差，越界后立即撤租并请求受控停车或重规划。

- [x] 检查对地速度、出缆速度、张力、加减速度、传感器模式和全部依赖版本。
- [x] 老租约、乱序租约、过期租约和剖面错配均被执行端拒绝。
- [x] 控制倍率变化不能延续原租约，必须生成并复检新剖面。
- [x] 监控周期和续租裕量满足配置不变量，撤租原因可审计。

Evidence: `powershell -NoProfile -ExecutionPolicy Bypass -File tools/verify.ps1`; MSVC 19.51 严格编译、`/analyze` 与 CTest 26/26 通过（总测试 5.46 s，完整验证 9.70 s，JUnit：`build/verify/ctest.xml`）。新增 `ExecutionLeaseMonitor` 独立检查计划/租约/剖面与完整依赖版本元组，按弧长插值验证对地速度、出缆速度、张力及地面/出缆加减速度；过期、乱序、旧租约、反馈过期、传感器/版本/剖面错配和偏差越界统一撤销并请求受控停车/重规划，续租裕量仅返回新复检请求；撤租序列持久化以拒绝延迟消息。新增 TimedPath 几何/剖面弧长、插值规则、停止点、有限阈值和租约总时长 fail-closed 门禁。已核对设计基线 v2.4 第 10.5、13.4、14.3 及 T32 验收条目。

#### T33 - 实现路径切换滞回

**Status:** `[x] done`  
**Blocked by:** T31  
**What it delivers:** 只有新旧剩余计划在同一最新上下文中都有效时才比较成本，轻微变化保持旧计划，显著改善才切换。

- [x] 旧计划无效时滞回没有保留权。
- [x] 规划期间租约到期后，在决策前通过回调重新捕获并完整复检。
- [x] 连续小扰动不产生路径振荡，显著改善可稳定切换。
- [x] 发布的计划始终附带本次复检签发的匹配租约。

Evidence: `powershell -NoProfile -ExecutionPolicy Bypass -File tools/verify.ps1`; MSVC 19.51 严格编译、`/analyze` 与 CTest 27/27 通过（总测试约 4.85 s，JUnit：`build/verify/ctest.xml`）。新增 `StabilityManager` 实现相对代价滞回、可选 Hausdorff 拓扑阈值、同一复检上下文/租约配对、未来/过期租约 fail-closed，以及租约过期时强制成对复检回调；回归覆盖阈值边界、连续 20 次小扰动、拓扑强制切换、旧计划/候选有效性、上下文时间戳、租约元数据、复检回调和逐字段确定性。已核对设计基线 v2.4 第 10.2、13.3-13.4、14.3 及 18.2.7 #1/#2/#3/#8/#9；联合路径风险仍未声明实现。

#### T34 - 实现近端承诺段与 G2/时序拼接

**Status:** `[x] done`
**Blocked by:** T27, T28, T31, T33  
**What it delivers:** 正常重规划保留已批准的近端几何和执行样本，新尾段从承诺终点状态规划并满足 G2 与动态连续性。

- [x] 承诺段长度由速度、制动和时延策略约束，不越过旧租约授权范围。
- [x] 新尾段参数化不能修改承诺段的几何、速度、出缆或张力样本。
- [x] 拼接位置、航向、曲率、速度、加速度、出缆和张力均通过容差检查。
- [x] 缆线机械与走廊最终验证覆盖承诺段和新段的完整拼接路径。

Evidence: `powershell -NoProfile -ExecutionPolicy Bypass -File tools/verify.ps1`; MSVC 19.51 严格编译、`/analyze` 与 CTest 27/27 通过（总测试约 5 s，JUnit：`build/verify/ctest.xml`）。新增 `StabilityManager` 承诺段提取和时序合并：按 `max(v*t_commit, d_stop,certified+safety_margin)` 计算长度，未提供认证最坏停车距离或超出旧剩余轨迹时 fail-closed；前缀几何/执行样本原样复制并重基时间，G2 位置/航向/曲率与执行速度/加速度/出缆/张力/出缆加速度连接均有硬容差，新剖面签发递增版本。承诺前缀使用显式 `validate_authorized_prefix`，完整 `TimedPath` 仍严格要求终端零速停止点。时序合并必须提供 T29 等价的完整路径最终验证回调，验证失败或缺失均拒绝发布。T34 测试覆盖长度边界、授权范围、前缀不变性、动态不连续、缺少最终验证和确定性。已核对设计基线 v2.4 第 10.3、13.3-13.4、18.2.5-18.2.7 及 T34 验收条目；联合路径风险未声明实现。

### H. 主动补探、状态机与主循环

#### T35 - 识别信息缺口并评估紧迫度

**Status:** `[x] done`
**Blocked by:** T04, T12, T31  
**What it delivers:** 参考路线和候选绕行范围内的未知/低置信区域形成信息缺口，并按剩余安全距离、到达时间和补探时延分为 BLOCKING/URGENT/SCHEDULED/INFORMATIONAL。

- [x] 缺口包含空间范围、关联参考进度、地图版本、置信度和原因。
- [x] 时间估计使用当前获批剩余剖面；缺失时采用显式保守策略。
- [x] BLOCKING 要求停车，URGENT 只提出新减速剖面请求，不直接改倍率。
- [x] 阈值边界和多缺口排序具有确定性与滞回。

Evidence: `powershell -NoProfile -ExecutionPolicy Bypass -File tools/verify.ps1`; MSVC 19.51 严格编译、`/analyze` 与 CTest 28/28 通过（总测试 5.56 s，完整验证 12.19 s，JUnit：`build/verify/ctest.xml`）。新增 `ScoutCoordinator` 沿版本化参考线采样地图未知/低置信区域，合并为带地图版本、参考线版本、进度范围、中心和最低置信度的缺口；使用已批准 `TimedPath` 插值到达时间，缺失/无效剖面显式采用保守阻塞策略；四级紧迫度输出 BLOCKING/URGENT/SCHEDULED/INFORMATIONAL，其中 URGENT 仅请求新验证减速剖面；版本化键、阈值滞回、稳定排序和不可表示进度的 fail-closed 行为均有确定性测试。已核对设计基线 v2.4 第 11.1、11.1.1、16.1、18.2.8 及 T35 验收条目；未实现整路径联合风险保证。

#### T36 - 生成补探目标并约束双机协同

**Status:** `[x] done`
**Blocked by:** T35  
**What it delivers:** `ScoutCoordinator` 为最有价值缺口生成可审计目标、优先级和请求生命周期，同时保证双机通信距离策略。

- [x] 目标兼顾缺口覆盖、参考路线前向进度、探测价值和到达代价。
- [x] 距离上限、期望间距和滞回独立配置；越界触发相应降级。
- [x] 请求具备版本、去重、超时、完成和地图更新关联。
- [x] 完成“缺口发现 -> 请求 -> 地图更新 -> 重规划”的算法闭环接口。

Evidence: `powershell -NoProfile -ExecutionPolicy Bypass -File tools/verify.ps1`；MSVC 19.51 严格编译与 `/analyze` 通过，CTest 28/28 通过（总测试 4.81 s，完整验证 9.030 s，JUnit：`build/verify/ctest.xml`），聚焦 `ScoutCoordinator` 与参数门禁各连续重复 20/20 通过。目标输出绑定地图/参考线/策略/profile/运行域版本并审计覆盖率、信息价值、前向临近度与到达代价，阻塞紧迫度不可被权重覆盖；双机期望/继续/停止/通信硬上限独立配置且硬上限越界输出结构化停车恢复通信降级；请求以地图、参考线、进度和量化空间目标去重，revision、乱序地图拒绝、实际离线目标单元关联、原子超时 `WAITING_MAP`、完成后旧计划失效与重规划触发均有边界/无效输入/逐字段确定性重放。全部补探参数经统一 `ParameterConfig` 加载、序列化、生产门禁和安全时间转换。双轴代码复审最终 Standards/Spec 均通过；已核对设计基线 v2.4 第 11.2-11.5、12、14.2、16.1、18.3.1 及 T36 验收条目，未实现前置机器人三维规划、底层控制或任何整路径联合风险保证。

#### T37 - 实现规划状态机、触发器与安全停车决策

**Status:** `[x] done`
**Blocked by:** T32, T36  
**What it delivers:** 周期、新地图、路径失效、通信异常、定位异常和租约事件驱动明确状态转换，并统一输出减速、停车或人工接管动作。

- [x] 状态和转换覆盖设计第 12 节的成功、规划、谨慎规划、等待地图、超时、无解和各类无效状态。
- [x] 安全停车使用当前速度、最大制动能力、地形和剩余安全距离校验。
- [x] 紧急事件优先撤租，不能等待新规划完成后再停止旧路径。
- [x] 状态恢复必须使用新同步快照和新租约，不能回退旧序列。

Evidence: `powershell -NoProfile -ExecutionPolicy Bypass -File tools/verify.ps1`；MSVC 19.51 严格编译与 `/analyze` 通过，CTest 29/29 通过（总测试 5.48 s，完整验证 10.333 s，JUnit：`build/verify/ctest.xml`），聚焦状态机连续重复 20/20 通过。`PlanningStateMachine::dispatch(event, synchronized_context)` 的确定性测试覆盖周期、新地图、参考线、机器人作业区与状态变更触发，全部第 12 节状态，通信 5/15 s 边界，谨慎剖面租约，超时复检，连续失败人工接管，定位/租约/急停事件，认证地形制动停车距离，紧急指令 `撤租 -> 停车 -> 重规划` 顺序，乱序安全事件覆盖权，以及人工/急停锁定状态仅凭新同步快照 revision、新租约 sequence 和显式解除事件恢复；已核对设计基线 v2.4 第 10.3.1、10.5、12、13.4、14.1、16 节及 T37 验收条目。双轴代码复审最终无 Standards 硬性违规、无 Spec 缺口。

#### T38 - 打通主规划循环成功路径

**Status:** `[x] done`
**Blocked by:** T29, T30, T34, T37  
**What it delivers:** 设计第 16 节的完整成功链路可在确定性场景运行：快照、地形、搜索、平滑、时序化、缆线验证、候选复检、决策、租约和发布。

- [x] 每阶段只消费本次锁定版本，阶段间不读取可变全局状态。
- [x] 承诺段起点状态和无承诺段实际状态两条路径均通过。
- [x] 发布前再次捕获最新决策上下文，新候选失效时不发布。
- [x] 端到端结果含全部中间诊断、版本和耗时指标。

Evidence: `powershell -NoProfile -ExecutionPolicy Bypass -File tools/verify.ps1`；MSVC 19.51 严格编译与 `/analyze` 通过，CTest 30/30 通过（总测试 4.98 s，完整验证 12.349 s，JUnit：`build/verify/ctest.xml`），聚焦主循环连续重复 20/20 通过。新增 `MainPlanningLoop::run_success_cycle` 按锁定 revision 依次编排快照、地形、承诺段终态推导、搜索、平滑、时序化、承诺段 G2/时序拼接、完整机器人路径独立复检、完整时序缆线复检、候选组装、最新上下文复检、稳定性决策、租约与原子授权发布；承诺段缆线/参考终态由同步实际状态重新预测和机械评价得到，候选的轨迹、触地点、终态、走廊与机械证据均直接取自阶段产物，`execution_profile_version` 可作为新输出单调前进。确定性测试覆盖无承诺段实际起点、授权承诺前缀原样保留与完整复检、阶段锁定版本、全部中间产物/版本/耗时、发布前地图版本变化拒绝，以及不匹配租约无法暴露计划。已核对设计基线 v2.4 第 4、9.6、10.2-10.3、13.3-13.4、14、16 节及 T38 验收条目；双轴复审发现的承诺终态来源、完整机器人路径复检、缆线证据绑定、计划/租约原子发布和依赖版本重复逻辑均已修正。

#### T39 - 完成超时、无解与旧计划复用路径

**Status:** `[x] done`
**Blocked by:** T31, T34, T37, T38  
**What it delivers:** 搜索/平滑/参数化超时或无解时，系统只在规划结束后的最新同步上下文完整复检旧计划并取得新租约后继续，否则停车。

- [x] 区分 deadline、标签预算、包络下无解、普通无解和输入无效。
- [x] 旧租约在规划期间到期、剖面错配或状态变化时不能复用。
- [x] 原始搜索路径只有通过同一可跟踪性契约才可作为平滑失败回退。
- [x] 所有失败路径撤销不再有效的租约并记录唯一根因。

Evidence: `powershell -NoProfile -ExecutionPolicy Bypass -File tools/verify.ps1`；MSVC 19.51 严格编译与 `/analyze` 通过，CTest 30/30 通过（总测试 5.50 s，完整验证 13.753 s，JUnit：`build/verify/ctest.xml`），聚焦主循环与参数化器各连续重复 20/20 通过。`MainPlanningLoop::run_cycle` 以唯一结构化根因区分搜索 deadline、标签/其他预算、包络下无解、普通无解、平滑 deadline/不可行、参数化 deadline/不可行、输入无效及协方差包络 breach；24 组主循环场景与 6 组参数化场景覆盖最新同步 revision/完整依赖门禁、真实单调时钟截止与完成边界、旧租约到期/撤销后高水位/新序列、先撤旧租约再原子切换、剖面/状态错配、同 revision 依赖漂移、剩余 `TimedPath` 插值裁剪与逐点逐样本授权、原始路径同一可跟踪性契约回退、全部失败撤租和受控停车。旧计划只可在规划结束后重新捕获最新同步输入、调用同一 `revalidate_plan` 完整复检并取得严格更新的新租约后继续；候选或旧计划的包络 breach 与无效输入禁止降级复用。已按 T39 修订并核对设计基线 v2.5 第 9.6、10.2、10.5、12、13.3-13.4、14、16、17 节；最终双轴复审 Standards 与 Spec 均 PASS。

#### T40 - 完成承诺段安全覆盖规则

**Status:** `[x] done`
**Blocked by:** T32, T34, T37, T38  
**What it delivers:** 新障碍、地形突变、定位跳变、缆线状态异常、执行偏差或版本变化可打破承诺段并立即进入重新规划或停车。

- [x] 安全检查覆盖剩余承诺段的机器人和缆线完整约束。
- [x] `REPLAN_URGENT` 在启动规划前先撤销旧租约并触发受控停车通道。
- [x] 不允许滞回或承诺语义覆盖安全事件。
- [x] 每类事件具有故障注入测试和可追溯状态转换。

Evidence: `powershell -NoProfile -ExecutionPolicy Bypass -File tools/verify.ps1`；MSVC 19.51 严格编译与 `/analyze` 通过，CTest 31/31 通过（总测试 5.27 s，完整验证 27.491 s，JUnit：`build/verify/ctest.xml`），`execution_lease_monitor|commitment_safety|main_planning_loop` 三组聚焦测试各连续重复 20/20 通过。`CommitmentSafetyEvaluator` 以完整机器人路径及完整时序缆线验证结果覆盖剩余承诺，按 `STOP > REPLAN_URGENT > CONTINUE` 聚合安全事件，并对新障碍使用认证最坏停车距离门禁；`CommitmentSafetySupervisor` 拒绝缺失的安全停车通道，并在独立事件通道中先撤销租约再触发受控停车，主循环随后禁止搜索、滞回及旧承诺复用。故障注入覆盖新障碍、地形变化、定位跳变、缆线异常、执行速度/加速度/张力偏差、全部规划依赖版本漂移、机器人/缆线完整约束失败、验证证据缺失、空停车通道配置及已撤销承诺跨周期复用，均断言结构化事件、唯一原因码、动作和撤租先于停车的可追溯顺序。已按 T40 修订并核对设计基线 v2.6 第 10.3.1、12、14.2、16、17 节。

#### T41 - 完成消息去重、乱序与状态回退防护

**Status:** `[x] done`
**Blocked by:** T05, T19, T32, T38  
**What it delivers:** 地图、参考线、遥测、补探响应、计划和租约在延迟、重复与乱序条件下保持单调版本和一致决策。

- [x] 每类消息定义去重键、序列窗口、最大年龄和版本回退规则。
- [x] 旧地图、旧计划和旧租约不能覆盖新状态。
- [x] 通信恢复后重新同步全部上下文再恢复自动执行。
- [x] 延迟和乱序测试不出现混合快照、状态倒退或无授权执行。

Evidence: `powershell -NoProfile -ExecutionPolicy Bypass -File tools/verify.ps1`；MSVC 19.51 严格编译与 `/analyze` 通过，CTest 32/32 通过（总测试 5.57 s，完整验证 10.165 s，JUnit：`build/verify/ctest.xml`），`message_consistency` 与 `execution_lease_monitor` 聚焦测试各连续重复 20/20 通过。`MessageConsistencyGate` 为地图、参考线、机器人状态、缆线状态、参考进度、放缆遥测、执行跟踪、补探响应、规划结果和验证租约固定类型化去重键、显式最大年龄、有限序列窗口、版本/接收时间回退门禁及缺失序列诊断；允许乱序的地图和补探响应只按连续序列释放，并在释放时重新检查年龄与版本。规划结果同时在到期时刻拒绝自身 `validity_duration` 已失效的首次接收，拒绝诊断携带消息版本/时间戳、策略版本、参数剖面、运行域和显式 `per-stream-consistency-only` 风险语义。`CommunicationRecoveryGate` 要求恢复边界后的完整同步输入、不可变计划和更新租约全部越过水位，地图 ID/坐标系/序列及全部依赖逐项一致，且由 `ExecutionLeaseMonitor` 确认授权后才生成状态机恢复证据；旧地图/计划/租约、未来时间、重复/延迟/乱序、混合上下文和无授权恢复均 fail closed。执行监控只在结构、时间、反馈有限性、租约边界、依赖和剖面完成验证后推进计划/租约高水位，避免无效高序列毒化状态。已核对设计基线 v2.6 第 12.3-12.5、13.1-13.6、16 节及 T41 四项验收；双轴复审发现的计划有效期边界、高水位推进时机、地图身份保留、拒绝诊断审计字段、类型化去重键、重复分派异味和进度证据问题均已修正。

### I. 诊断、性能与确定性验收

#### T42 - 建立统一算法诊断与可复现实验记录

**Status:** `[x] done`
**Blocked by:** T38, T39, T40, T41  
**What it delivers:** 每次规划可还原输入版本、参数、随机种子、约束失败、标签统计、阶段耗时和最终决策。

- [x] 记录搜索活动标签峰值、等价比较、预算耗尽率和 P50/P95/P99 所需原始样本。
- [x] 记录平滑迭代、最大残差，参数化约束裕量和租约复检耗时。
- [x] 相同记录可离线重放并得到相同状态、路径和诊断。
- [x] 日志不把任务成功率、局部 epsilon 和未实现联合风险混为一谈。

Evidence: `powershell -NoProfile -ExecutionPolicy Bypass -File tools/verify.ps1`；MSVC 19.51 严格编译与 `/analyze` 通过，CTest 33/33 通过（总测试 5.22 s，完整验证 23.538 s，JUnit：`build/verify/ctest.xml`），`algorithm_diagnostics` 与 `main_planning_loop` 聚焦测试各连续重复 20/20 通过。统一 `AlgorithmExperimentRecord` 自动保留完整强类型运行参数、随机种子、初始授权、任意长度同步输入捕获序列、租约撤销前态、约束失败、搜索标签/等价比较/预算样本、平滑迭代与残差、参数化裕量、阶段及租约复检原始耗时和最终证据；P50/P95/P99 与预算耗尽率只使用实际执行搜索的样本。离线重放向真实 `MainPlanningLoop` 注入记录输入和初始控制状态，重新执行地形、搜索、平滑、时序化、复检与发布链，并逐字段比较状态、路径、诊断、输入和参数漂移；三次捕获回退及已撤销承诺租约均有回归覆盖。任务成功、机器人点风险、地形局部风险、缆线点风险与未实现 `epsilon_path`/联合风险使用独立字段及固定风险语义。已核对设计基线 v2.6 第 1.3、14.3、17.1-17.4、19.1 节及 T42 四项验收；双轴终审 Standards 与 Spec 均 PASS。

#### T43 - 实现增量地形更新与规划性能预算

**Status:** `[x] done`
**Blocked by:** T06, T25, T42  
**What it delivers:** 地图局部变化只重算受影响地形窗口，搜索与验证在不缓存安全结论的前提下满足可测量的内存和延迟预算。

- [x] 地形派生层支持更新区域扩张和只读共享，版本变化正确失效缓存。
- [x] 测量单标签大小、活动标签峰值、总内存、各阶段 P50/P95/P99 和超时率。
- [x] 在目标平台验证总内存小于 100 MB；未达标时有诊断而非不安全剪枝。
- [x] 只有实测总周期小于目标后才声明 2-5 Hz，500 ms 超时路径始终安全。

Evidence: `powershell -NoProfile -ExecutionPolicy Bypass -File tools/verify.ps1`；MSVC 19.51 严格编译、`/analyze` 与 CTest 33/33 通过，JUnit：`build/verify/ctest.xml`；`main_planning_loop` 与 `terrain_analyzer` 聚焦测试各连续 20/20 通过。局部地图更新按 1.2 m 物理拟合窗扩张并只重算 49/81 格、复用 32/81 格，台阶候选或既有台阶支撑带受影响时显式全量失效，增量台阶不连续覆盖仅访问重算格；相同版本共享不可变 `shared_ptr<const TerrainLayers>`，版本回退、同版本 payload 冲突、未声明更新和网格/配置变化均安全拒绝或全量失效。Hybrid A* 实测每搜索标签固定对象 1,472 B、含路径/缆线历史/预测/协方差/问题字符串容量的观测峰值 13,001 B，测试进程峰值 RSS 14,348,288 B（小于 100 MiB）；统一实验记录自动采样进程内存、总周期、阶段 P50/P95/P99、活动标签峰值、标签分布、预算耗尽率和超时率，无效记录不进入汇总。2-5 Hz 仅在完整实测样本数、内存和总周期均满足门禁时声明，本任务不以合成延迟样本声称生产平台频率；总周期超过 500 ms 返回 `TIMEOUT`，发布前仅在重新捕获、完整复检并签发新租约后续用旧计划，发布跨限则撤销新租约并停车，恰好 500 ms 边界允许。已核对设计基线 v2.6 第 17.1-17.4 节；双轴终审 Standards 与 Spec 均 PASS。

#### T44 - 完成地形与通行性 Level 1 验收矩阵

**Status:** `[x] done`  
**Blocked by:** T06, T07, T08, T09, T10  
**What it delivers:** 设计第 18.2.1、18.2.2 节全部地形与通行性场景及不变量成为自动化回归测试。

- [x] 覆盖鲁棒拟合、台阶、支撑不足、协方差无效和粗糙度去趋势。
- [x] 覆盖完整足迹、障碍裕量、方向坡度、上下坡非对称、斜交台阶和履带支撑。
- [x] 覆盖风险策略/分析配置/运行域错配和联合风险语义标记。
- [x] 测试清单逐条链接回设计编号，无跳过项。

Evidence: `powershell -NoProfile -ExecutionPolicy Bypass -File tools/verify.ps1`；MSVC 19.51 严格编译、`/analyze` 与 CTest 34/34 通过（总测试 11.47 s，JUnit：`build/verify/ctest.xml`），新增 `level1_terrain_traversability_matrix` 验收入口，按设计编号自动校验并执行 18.2.1 的 8 个场景、18.2.2 的 16 个场景和 7 个不变量（31/31），并为 18.2.1-8 追加非半正定协方差支持链路；矩阵连续重复 5/5 通过。补充完整复杂足迹边缘碰撞、真实 `TerrainAnalyzer` 台阶航向不变性、均值安全但保守界越限、各向异性协方差旋转投影、同一非零方差的上下坡非对称边界，以及策略版本/分析配置/运行域和局部非联合风险语义的显式断言。已逐项核对设计基线 v2.6 第 18.2.1、18.2.2 节；双轴终审 Standards 与 Spec 均 PASS。

#### T45 - 完成搜索与缆线 Level 1 验收矩阵

**Status:** `[x] done`
**Blocked by:** T19, T25, T29  
**What it delivers:** 设计第 18.2.3、18.2.4 节全部搜索、触地点、包络、机械历史与走廊场景成为自动化回归测试。

- [x] 覆盖原语中段/角点扫掠、分辨率变化、交叉进度、多标签等价与资源耗尽。
- [x] 覆盖模型连续性、路径记忆协方差、包络覆盖/失效/版本隔离和搜索协方差隔离。
- [x] 覆盖机械曲率、禁放区、悬空代理、实际历史初始化和完整拼接路径复检。
- [x] 覆盖时序路径拒绝、减速重预测、MARGINAL 弧长门禁和风险指标隔离。

Evidence: `powershell -NoProfile -ExecutionPolicy Bypass -File tools/verify.ps1`；MSVC 19.51 严格编译、`/analyze` 与 CTest 35/35 通过（总测试 14.66 s，完整验证 17.899 s，JUnit：`build/verify/ctest.xml`），新增 `level1_search_cable_matrix` 验收入口，按设计编号自动校验并执行 18.2.3 的 19 个搜索场景、4 个返回路径逐段硬不变量审计、18.2.4 的 34 个缆线场景和 7 个关键验证（64/64），矩阵连续重复 5/5 通过。补充转弯外角扫障、机器人/缆线空间域分离、旋转法向各向异性协方差投影、任务成功率与点风险隔离、非零初始滞后收敛与状态隔离、空/非单调执行剖面拒绝、零软权重机械硬门禁、有效减速剖面按新版本重预测、完整认证执行包络逐字段复核、MARGINAL 弧长重采样边界及完整时序候选实际历史拼接复检；修复最终缆线预测将空/非单调执行剖面误报为协方差无效，以及 `timed_cable_candidate_verifier` 仅执行 1/4 个既有测试和 breach 夹具方差/标准差不一致的问题。已逐项核对设计基线 v2.6 第 18.2.3、18.2.4 节；双轴终审 Standards 与 Spec 均 PASS。

#### T46 - 完成平滑、时序与稳定性 Level 1 验收矩阵

**Status:** `[x] done`
**Blocked by:** T28, T31, T32, T33, T34, T40  
**What it delivers:** 设计第 18.2.5 至 18.2.7 节的 G2、时序参数化、路径复用、租约与安全覆盖场景成为自动化回归测试。

- [x] 覆盖非零边界曲率、负曲率、曲率变化率、几何独立审计和求解器假收敛。
- [x] 覆盖动力学/放缆/停车边界、剖面完整性/版本和承诺段不变性。
- [x] 覆盖滞回、快照一致性、租约配对/过期/撤销、运行偏差和紧急重规划。
- [x] 所有安全失败均验证执行端拒绝后续未授权命令。

Evidence: `powershell -NoProfile -ExecutionPolicy Bypass -File tools/verify.ps1`；MSVC 19.51 严格编译与 `/analyze` 通过，CTest 36/36 通过（完整验证 23.018 s，JUnit：`build/verify/ctest.xml`）。新增 `level1_smoothing_timing_stability_matrix` 验收入口，按设计编号执行 18.2.5 的 11 个平滑场景与 3 个关键不变量、18.2.6 的 7 个时序场景、18.2.7 的 18 个稳定性场景（39/39），矩阵连续重复 5/5 通过。补充正负曲率绝对上限、位置相同的航向/曲率 G2 独立残差、当前地图上完整拼接路径的真实扫掠失败、机器人/缆线/执行跟踪状态及全部上下文依赖的捕获中变化、从改变后的实际滞后角重新预测失败，以及同步快照、平滑、时序、完整路径、缆线、上下文、TIMEOUT 复检、运行偏差和紧急承诺事件触发的真实撤租链路；每条安全链路均再次提交旧租约命令并验证执行端返回 `LEASE_ALREADY_REVOKED`。已逐项核对设计基线 v2.6 第 18.2.5、18.2.6、18.2.7 节；双轴终审 Standards 与 Spec 均 PASS。

#### T47 - 建立完整算法闭环的确定性集成场景

**Status:** `[x] done`
**Blocked by:** T38, T39, T40, T44, T45, T46  
**What it delivers:** 在不依赖真实传感器或底层控制的测试驱动器中，完整算法可处理路线偏离、障碍绕行、地图更新、补探、重规划与安全停车。

- [x] 至少包含平坦直行、单/双侧绕行、坡面、台阶、未知缺口和包络 breach 场景。
- [x] 每个周期检查机器人作业区、缆线走廊、机械约束、版本和租约不变量。
- [x] 可注入时间推进、消息乱序、遥测偏差和模型版本变化。
- [x] 生成可供比赛演示和回归比较的结构化报告。

Evidence: `powershell -NoProfile -ExecutionPolicy Bypass -File tools/verify.ps1`；MSVC 19.51 严格编译与 `/analyze` 通过，CTest 38/38 通过（T47 `level1_closed_loop_scenarios` 193.42 s，完整验证 220.349 s，JUnit：`build/verify/ctest.xml`）。确定性驱动器以真实 `HybridAStarPlanner`、平滑、时序化、机器人/缆线独立复检、主规划循环、状态机和执行租约监视器执行 8 个场景、4 个持续故障注入运行和 18 个周期；报告记录 12 个授权周期、6 次安全停车、5 次旧租约拒绝证明，全部周期的机器人作业区、缆线走廊、机械约束、版本和租约均给出 `satisfied` 或 `safe_not_applicable` 证据，无 `not_checked`/`failed`。未知缺口使用同一未知地图完成扫描与规划、同一更新地图完成请求关联与重规划，并真实迁移 `WAITING_MAP -> NORMAL_PLANNING`；路线偏离恢复按参考线投影测量。结构化报告为 `build/verify/t47_closed_loop_report.json`，SHA-256 `DBCF77C742B00FEA4F085D1B2236C5FAEC42D46BBA24806242208E1EAC1A9879`。已逐项核对设计基线 v2.6 第 5.2、5.7、11.5、16、18.1、18.2.2、21 节；风险语义保持 `POINTWISE_ONLY:NO_PATH_JOINT_RISK_GUARANTEE`，未声称整路径联合风险保证，未引入真实传感器、底层控制、外部仿真或上游建图。双轴终审 Standards 与 Spec 均 PASS；Standards 的剩余建议仅为报告类型与场景分派的维护性判断，不影响本 ticket 验收。

### J. 仿真、标定与竞赛验收门禁

以下任务仍属于算法交付的验证门禁，但依赖外部仿真、机器人、缆线和真值数据。本仓库只实现算法适配、数据记录与评价，不扩展到传感器、控制器或前置机器人三维规划。

#### T48 - 接入 DAVE/Gazebo 双机算法闭环

**Status:** `[ ] blocked-external`  
**Blocked by:** T36, T41, T47; external simulator and robot interfaces  
**What it delivers:** 算法在仿真数据流中完成前置探测请求、地图更新、主机重规划和距离约束闭环。

- [ ] 补探请求到地图更新的关联与延迟可测量。
- [ ] 双机距离策略全程满足配置，越界时正确降级。
- [ ] 重规划成功率、失败原因和租约状态可回放。
- [ ] 仿真适配不把上游建图或下游控制逻辑实现进算法核心。

#### T49 - 完成长时与通信异常仿真

**Status:** `[ ] blocked-external`  
**Blocked by:** T43, T48; external simulator  
**What it delivers:** 代表性 1-2 km 路段运行 30-60 分钟，并验证短、中、长通信中断及消息延迟/乱序下的算法稳定性。

- [ ] 记录内存、CPU、成功率、阶段耗时、超时率和路径切换频率。
- [ ] 通信异常进入正确降级状态，恢复后重新同步且无状态回退。
- [ ] 无持续内存增长、标签泄漏、陈旧快照或过期租约执行。
- [ ] 长时结果与单元场景的安全不变量一致。

#### T50 - 标定地形梯度风险策略

**Status:** `[ ] blocked-external`  
**Blocked by:** T06, T09, T44; independent terrain truth dataset  
**What it delivers:** 用独立真值的多坡向、多粗糙度和多声呐质量数据确定梯度覆盖模型、系数和运行域版本。

- [ ] 训练/拟合数据与验收数据严格分离。
- [ ] 验证二维梯度残差覆盖率；重尾或偏斜时改用经验证经验/确定性界。
- [ ] 输出策略版本、分析配置版本、数据集、运行域和局部 epsilon。
- [ ] 未通过覆盖率验收前生产配置保持阻塞。

#### T51 - 标定缆线模型与横向统计包络

**Status:** `[ ] blocked-external`  
**Blocked by:** T15, T18, T19, T45; independent cable truth dataset  
**What it delivers:** 独立试验确定放缆点偏置、触地点距离、方向响应长度、过程噪声、有效张力/出缆范围及各运行域包络。

- [ ] 参数拟合与包络覆盖审计使用不同数据，覆盖边界和对抗轨迹。
- [ ] 每个参考线、传感器模式、模型和执行运行包络组合生成独立版本。
- [ ] 审计 `sigma_perp_real <= sigma_bar + epsilon_env` 并记录所有离散裕量。
- [ ] 包络缺口触发新版本，禁止靠扩大经验 confidence 掩盖。

#### T52 - 标定机器人能力与执行租约阈值

**Status:** `[ ] blocked-external`  
**Blocked by:** T28, T32, T46; robot and closed-loop test data  
**What it delivers:** 实测确定几何足迹、履带支撑、转弯/曲率变化、坡度/台阶、速度/制动、出缆/张力跟踪和租约撤销阈值。

- [ ] 上下坡和爬阶/落差分别标定，不互相替代。
- [ ] 停车距离与执行偏差覆盖率由闭环试验验证。
- [ ] 租约阈值不超出缆线模型和统计包络认证运行域。
- [ ] 所有参数携带设备、数据集、日期和版本；生产门禁通过前保持阻塞。

#### T53 - 完成水池短距离算法验证

**Status:** `[ ] blocked-external`  
**Blocked by:** T50, T51, T52; water tank, robot, cable and truth system  
**What it delivers:** 在直线、转弯、简单绕障和定位误差注入试验中验证触地点预测、碰撞裕量和短距离布放精度。

- [ ] 真值系统同时记录机器人轨迹、执行剖面、实际触地点和环境地形。
- [ ] 分别验证机器人碰撞误差预算和缆线横向协方差，不混用。
- [ ] 预测误差按运行域、传感器模式和路径类型分层报告。
- [ ] 失败样本回灌确定性重放并更新模型或包络版本。

#### T54 - 完成海试与甲方指标验收

**Status:** `[ ] blocked-external`  
**Blocked by:** T49, T53; sea-trial platform and acceptance protocol  
**What it delivers:** 真实海床条件下报告落点精度、走廊符合率、任务完成率、连续铺设距离和 `R_lay >= 0.8`。

- [ ] 报告横向误差均值、RMS、最大值、P50/P90/P95 和最长连续越界长度。
- [ ] `R_lay` 只描述实际合格长度比例，不转换为 `epsilon_point`。
- [ ] 单独报告局部统计风险覆盖；首版明确不提供 `epsilon_path` 保证。
- [ ] 每项结果绑定软件、参数、地图、模型、包络、设备和数据版本。

#### T55 - 完成三算法基线对比

**Status:** `[ ] ready-for-agent`
**Blocked by:** T43, T47  
**What it delivers:** 在统一场景和预算下对比二维 A*、标准 Hybrid A* 与本方案，量化完整方案的收益与代价。

- [ ] 使用设计规定的 10 个简单、20 个中等和 10 个复杂场景。
- [ ] 统一报告成功率、耗时、路径长度、触地点偏离、走廊越界、曲率变化和切换频率。
- [ ] 基线只用于对比，不作为可执行生产路径绕过本方案安全门禁。
- [ ] 数据、随机种子、参数与统计脚本可复现。

#### T56 - 完成核心机制消融实验

**Status:** `[ ] blocked`  
**Blocked by:** T47, T55  
**What it delivers:** 分别量化触地点中心线软目标、触地点模型、机器人碰撞裕量、走廊概率界、多并线点、滞回和主动补探的贡献。

- [ ] 每次只移除一个机制，其他输入、预算和场景保持一致。
- [ ] 消融模式带非生产标记且不能获得执行租约。
- [ ] 报告每项机制对准确率、安全裕量、成功率、稳定性和耗时的影响。
- [ ] 结果直接用于比赛答辩证据，不反向修改硬约束语义。

#### T57 - 执行生产就绪与设计追踪审计

**Status:** `[ ] blocked`  
**Blocked by:** T42, T43, T44, T45, T46, T47, T50, T51, T52, T58, T59, T60, T61, T63, T64, T65
**What it delivers:** 在任何自动铺设或正式验收前，逐项证明设计基线、参数门禁、测试、标定和版本依赖均已闭环。

- [ ] 第 20 节所有首版必需参数均有值、单位、来源、版本和独立验证证据。
- [ ] 第 21 节全部 PRD 能力与测试映射均指向通过的自动化或外部验收证据。
- [ ] 所有已知限制、TBD、未实现联合风险和第一阶段悬空代理在输出与报告中显式披露。
- [ ] 未完成项保持阻塞，禁止通过默认值、关闭检查或修改权重签发生产租约。

#### T58 - 闭合快照与空间域版本依赖契约

**Status:** `[x] done`
**Blocked by:** T04, T05, T30, T31, T32 (completed)
**Discovered by:** D02 as-built contract audit
**Code files:** `src/underwater_planner/include/underwater_planner/core/data_contract.hpp`, `versioned_snapshot.hpp`, `synchronized_validation_inputs.hpp`, `planning_result.hpp`, `plan_validity_evaluator.hpp`, `execution_lease_monitor.hpp`，同模块 `src/*.cpp` 及对应契约/租约测试。
**What it delivers:** 缆线施工走廊与其他快照成员一样具有不可复用的版本负载，并沿验证输入、规划结果、诊断和执行租约传播；参数空间域版本与运行时快照使用可比较的唯一语义。

- [x] `SnapshotManager` 分别检查地图、参考线、机器人作业区和缆线走廊的“同版本同负载”；即使另一成员版本前进，也拒绝任一成员以旧版本携带新负载，并清除当前 map 回退分支的不可达重复判断。
- [x] 无论是否配置最大年龄都拒绝未来地图时间；`Duration{0}` 只能表示不启用过期年龄上限，不能关闭单调因果检查，并覆盖默认 manager 测试。
- [x] 将 `cable_corridor_version` 纳入 `PlanningDependencyVersions`、`PlanningResult`、`Diagnostics`、同步输入、`PlanValidationLease` 与 `ActiveExecutionContext`，更新序列化 schema 和所有依赖相等性检查。
- [x] 统一或显式转换 `SpatialDomainConfig` 的版本表示，验证参数中的机器人作业区/缆线走廊 id 与版本和 `VersionedPlanningSnapshot` 一致；不允许仅凭非空字符串通过。
- [x] 测试覆盖四类成员的同版本异负载、走廊版本回退/推进、捕获中变化、结果往返、租约签发及执行中走廊版本变化异步拒绝。

Evidence: 聚焦严格编译后运行 `ctest --test-dir build/verify -R "^(versioned_snapshots|plan_validity_evaluator|stability_manager)$" --output-on-failure`，3/3 通过（0.46 s）；`powershell -NoProfile -ExecutionPolicy Bypass -File tools/verify.ps1` 完成 MSVC 19.51 严格编译、`/analyze` 与 CTest 38/38（总测试 165.34 s，完整流程 173.854 s，JUnit：`build/verify/ctest.xml`）。测试覆盖四类快照成员同版本异负载、默认 manager 未来地图、走廊回退/推进与捕获竞态、`UP_RESULT 8` 往返、诊断/租约/执行上下文传播、复检与稳定性上下文走廊漂移拒绝，以及空间域 id/canonical 十进制版本匹配。已逐项核对设计第 13.2-13.5、15.1 节与 T58 验收条目；T61 的嵌套失败时间缺口已在后续 T61 ticket 闭合。

#### T59 - 严格解析参数枚举、布尔、无符号数与重复键

**Status:** `[x] done`
**Blocked by:** T03 (completed)
**Discovered by:** D02 as-built contract audit
**Code files:** `src/underwater_planner/include/underwater_planner/core/parameter_config.hpp`, `src/underwater_planner/src/parameter_config.cpp`, `src/underwater_planner/test/test_parameter_config.cpp`。
**What it delivers:** 参数文本不能通过宽松标量解析把拼写错误、负预算或重复安全字段变成另一个有效 production 配置。

- [x] `mode` 只接受 `production` 和 `non_production_capability_profile`；未知文本在加载阶段结构化失败，不得默认映射为 production。
- [x] 布尔字段只接受明确的 `true`/`false`/`1`/`0`；其他文本拒绝，不得静默变成 `false`。
- [x] `maximum_active_labels`、`scout_policy_version` 等无符号字段拒绝符号、尾随字符、溢出和负数，且完整消费输入。
- [x] 对缩进 key 与 dotted key 解析后的 canonical 标量路径做重复检测；重复标量拒绝，不采用 last-write-wins，同时为 `sensor_health_mode` 等可重复列表字段保留显式、可审计的追加语义。
- [x] 保持 canonical 序列化往返确定性，并为以上每种失败模式增加 loader 测试。

Evidence: `powershell -NoProfile -ExecutionPolicy Bypass -File tools/verify.ps1`；MSVC 19.51 严格编译、`/analyze` 与 CTest 38/38 通过（总测试 165.80 s，完整流程 174.037 s，JUnit：`build/verify/ctest.xml`）；参数测试覆盖未知 mode、非法布尔、负数/带尾随字符/带符号/溢出无符号数、缩进与 dotted canonical 重复键，以及 `sensor_health_mode` 追加顺序。T60 实现后，工作区最终源码重新编译并执行 `ctest --test-dir build/verify -R "^parameter_config$" --output-on-failure` 1/1 通过（0.03 s）。已核对设计第 15.2 节；T61 后续已闭合，严格解析和参数门禁均不外推为外部标定完成。

#### T60 - 补全参数有限性与 production 语义门禁

**Status:** `[x] done`
**Blocked by:** T03 (completed)
**Discovered by:** D02 as-built contract audit
**Code files:** `src/underwater_planner/include/underwater_planner/core/parameter_config.hpp`, `src/underwater_planner/src/parameter_config.cpp`, `src/underwater_planner/test/test_parameter_config.cpp`，以及消费相关字段的配置装配测试。
**What it delivers:** `validate_parameters` 对每个已提供字段执行一致的有限性/范围校验，`production_ready` 不再把缺少关键几何、运行域或已标定风险模型的配置判为生产就绪。

- [x] 以字段清单覆盖 `robot`、`terrain_gradient_risk`、`robot_collision_risk`、`spatial_domains`、`execution`、`cable`、`statistical_risk`、`path_reuse`、`search` 和 `task` 的全部 optional 数值；两种 profile 都拒绝 NaN、无穷和越界值。
- [x] production 强制机器人长/宽/高、最小转弯半径及其与最大曲率的一致性，并明确校验 `execution.terminal_speed_mps` 和 `task.laying_success_ratio_target`；任务比例保持性能指标语义，不转换为 epsilon。
- [x] production 拒绝空或 `*_pending_calibration` 的地形覆盖模型，强制 `terrain_gradient_risk.operating_domain_id == ParameterConfig.operating_domain_id`，并校验覆盖模型、策略、数据集和系数的组合。
- [x] production 要求统计包络至少声明一个受支持且可识别的 `sensor_health_mode`，拒绝空集合、未知模式和重复模式。
- [x] 增加逐组非生产有限性、production 缺项、运行域错配、pending 模型、空/未知传感器模式和关键边界值测试；更新 T03 既有能力门禁证据，不夸大外部标定状态。

Evidence: `ctest --test-dir build/verify -R "^parameter_config$" --output-on-failure` 1/1 通过（0.05 s）；新增 T60 测试覆盖 robot/terrain_gradient_risk/robot_collision_risk/execution/cable/statistical_risk/path_reuse/search/task 各组的 NaN、无穷或越界值，production 几何/终端速度/任务比例缺项与 `R_lay < 0.8`，地形运行域错配、版本 provenance 错配、pending/未知覆盖模型，以及空/未知/重复传感器模式。MSVC 19.51 `/W4 /WX /permissive-` 编译通过；已核对设计第 15.1-15.3 节和第 20.1、20.3、20.5、20.6 节。外部 T50-T52 标定仍保持 `blocked-external`，本 ticket 仅完成代码门禁，不宣称生产能力已标定。

#### T61 - 关闭失败规划结果的嵌套时间戳缺口

**Status:** `[x] done`
**Blocked by:** T02 (completed)
**Discovered by:** D02 Spec review
**Code files:** `src/underwater_planner/src/data_contract.cpp`, `src/underwater_planner/test/test_data_contract.cpp`, `src/underwater_planner/test/test_planning_result.cpp`。
**What it delivers:** 所有能够发布或序列化的 `PlanningResult`，包括不携带路径负载的失败结果，都不会保留默认负时间或其他无效嵌套消息时间。

- [x] `validate(PlanningResult)` 对始终序列化的 `terminal_cable_state.timestamp` 与 `corridor_result.evaluation_timestamp` 定义并执行状态无关的有效语义；不得因 payload 为空或 corridor 非 `valid` 而跳过时间检查。
- [x] 失败结果必须填入有效的单调评估/装配时间，继续使用现有 `UP_RESULT 8` schema；默认 `-1` 保持为未装配哨兵，由校验和 publisher 拒绝，不引入含糊的“不适用”时间。
- [x] 测试覆盖 `NO_SOLUTION`、`INPUT_INVALID` 与 `TIMEOUT` 的默认/负嵌套时间拒绝、有效失败结果往返和 publisher 拒绝，保留现有 NaN、负执行时间及 golden-schema 证据。

Evidence: `powershell -NoProfile -ExecutionPolicy Bypass -File tools/verify.ps1`；MSVC 19.51 严格编译、静态分析构建与 CTest 38/38 通过，总测试时间 164.09 s，完整流程 172.900 s，JUnit：`build/verify/ctest.xml`。聚焦 `data_contract` 16 项与 `planning_result` 3 项均通过；覆盖 `NO_SOLUTION`、`INPUT_INVALID`、`TIMEOUT` 的默认/各字段负嵌套时间拒绝、有效失败结果往返和 publisher 拒绝。已核对设计第 3.3、13.4-13.5、14.3 节；`UP_RESULT 8` schema 与风险语义未改变。

#### T62 - 接通机器人粗糙度生产硬门禁

**Status:** `[x] done`
**Blocked by:** T10, T60 (completed)
**Discovered by:** D03 as-built terrain/traversability audit
**Code files:** `src/underwater_planner/include/underwater_planner/core/traversability_evaluator.hpp`, `src/underwater_planner/src/traversability_evaluator.cpp`、生产配置装配入口及相应通行性/配置测试。
**What it delivers:** production 必填的 `robot.maximum_roughness_m` 不再是只校验但无人消费的参数；完整扫掠足迹任一有效表面的去趋势粗糙度越限都会成为可审计的机器人硬拒绝。

- [x] 将 `maximum_roughness_m` 作为有限、非负的机器人能力字段装配到 `TraversabilityEvaluator`，不以搜索软代价或缆线粗糙度代价替代。
- [x] 在与方向坡度相同的完整自适应扫掠足迹上检查 `detrended_roughness_rms_m`，越限返回独立限制因素；无效/非有限粗糙度失败关闭。
- [x] 结果记录最坏粗糙度，测试覆盖中心安全但足迹边缘越限、阈值边界、非有限输入、重复确定性和 production 参数装配。
- [x] 更新第5.3、5.6、14.2、18.2.2、20.2节及 Level 1 证据矩阵，完成前不得声称机器人粗糙度硬门禁已实现。

Evidence: `powershell -NoProfile -ExecutionPolicy Bypass -File tools/verify.ps1` 完成严格配置、MSVC 构建、静态分析与 CTest 38/38；`level1_terrain_traversability_matrix` 33/33 通过，JUnit：`build/verify/ctest.xml`。聚焦 `directional_slope|level1_terrain_traversability_matrix` 2/2 通过。新增测试覆盖完整足迹边缘越限、等值阈值通过、NaN/负值失败关闭、最大粗糙度记录、限制因素、重复确定性和 production `make_robot_capability` 装配；`parameter_config` 覆盖 production 缺失粗糙度阈值。已核对设计第5.3、5.6、14.2、18.2.2、20.2节及 Level 1 证据矩阵。代码门禁已完成，外部机器人能力标定数据仍待完成；T63 的地图时间戳门禁仍未宣称完成。

#### T63 - 闭合碰撞扫掠的完整地图版本门禁

**Status:** `[x] done`
**Blocked by:** T08 (completed)
**Discovered by:** D03 Spec review
**Code files:** `src/underwater_planner/src/traversability_evaluator.cpp`, `src/underwater_planner/test/test_traversability_evaluator.cpp` 及 Level 1 地形/通行性证据矩阵。
**What it delivers:** `evaluate_collision_sweep` 不再接受与 `TerrainLayers` 具有相同地图标识、序号和坐标系但时间戳不同的 `CollisionLayerResult`，公共扫掠接口的地图版本依赖完整失败关闭。

- [x] 使用完整 `MapVersion` 相等比较绑定碰撞层与地形层，不遗漏 `timestamp`；版本错配返回可区分的无效结果且不执行足迹扫掠。
- [x] 测试覆盖仅时间戳不同、map id/sequence/frame 分别不同、分析配置不同和完全匹配，拒绝结果保持有限且不可误判为无碰撞。
- [x] 将新负向测试绑定到 Level 1 地形/通行性矩阵，并更新第5.6、14.2和18.2.2节，不再保留本轮 as-built 限制。

Evidence: TDD 红灯阶段严格编译后运行 `ctest --test-dir build/verify -R "^traversability_evaluator$" --output-on-failure`，唯一失败为仅 `MapVersion.timestamp` 错配仍被扫掠接口接受；改用完整 `MapVersion::operator!=` 后，聚焦 `traversability_evaluator|level1_terrain_traversability_matrix` 2/2 通过（4.14 s），Level 1 地形/通行性证据矩阵扩展为 34/34。`powershell -NoProfile -ExecutionPolicy Bypass -File tools/verify.ps1` 完成 MSVC 19.51 严格构建、`/analyze` 与 CTest 38/38（测试 194.64 s，完整流程 202.232 s，JUnit：`build/verify/ctest.xml`）。公共接口测试覆盖完整匹配，以及 timestamp、map id、sequence、coordinate frame、分析配置版本分别错配；拒绝结果为 `INPUT_INVALID`、`collision_free=false`、零扫掠计数且数值有限。已逐项核对并同步设计第4.3、5.6、14.2、18.2.2节及首页修订说明，未扩大点风险或外部能力保证。

#### T64 - 闭合独立几何复检的起点曲率来源门禁

**Status:** `[x] done`
**Blocked by:** T27 (completed)
**Discovered by:** D06 Spec review
**Code files:** `src/underwater_planner/src/path_candidate_verifier.cpp`、`src/underwater_planner/test/test_path_candidate_verifier.cpp` 及 Level 1 平滑/时序/稳定性证据矩阵。
**What it delivers:** 公共 `auditPathGeometry` / `PathCandidateVerifier` 与 `PathSmoother` 一样独立拒绝把 `planned_goal` 曲率来源用作完整候选的起点 G2 证据，不依赖调用方碰巧构造正确来源。

- [x] 起点只接受 `synchronized_actual_state` 或 `committed_segment_terminal`；`planned_goal`、缺失曲率、零序列、负时间或超出同步容差均返回输入无效，且不进入作业区/碰撞/通行性扫掠。
- [x] 终点继续允许 `planned_goal`，不得把起点 provenance 门禁错误套用到目标边界。
- [x] 公共测试覆盖两类合法起点来源、非法 `planned_goal` 起点和合法目标来源，并断言失败状态、结构化原因、零扫掠副作用及确定性。
- [x] 更新第9.1、9.4、9.6、14.2、18.2.5节和 Level 1 证据矩阵；完成前保持该 as-built 缺口与 D10 阻塞可见。

Evidence: TDD 红灯阶段严格编译后运行 `ctest --test-dir build/verify -R "^path_candidate_verifier$" --output-on-failure`，唯一失败为 `planned_goal` 起点仍进入完整候选复检；加入起点来源白名单与 `input_invalid` 状态映射后聚焦测试通过。随后 `path_candidate_verifier|level1_smoothing_timing_stability_matrix` 2/2 通过（2.10 s），Level 1 第18.2.5节矩阵扩展为 12/12 场景及 3/3 不变量。`powershell -NoProfile -ExecutionPolicy Bypass -File tools/verify.ps1` 完成 MSVC 19.51 严格构建、`/analyze` 与 CTest 38/38（测试 161.80 s，完整流程 168.012 s，JUnit：`build/verify/ctest.xml`）。公共测试覆盖同步实际状态/承诺段终端两类合法起点、合法 `planned_goal` 终点，以及非法 `planned_goal`、缺失曲率、零序列、负位姿/曲率时间和超同步容差的结构化输入失败、零碰撞/通行性扫掠与确定性重放；已逐项核对并同步设计第9.1、9.4、9.6、14.2、18.2.5节及首页修订说明。

#### T65 - 闭合主循环滞回集成与候选代价门禁

**Status:** `[x] done`
**Blocked by:** T33, T38 (completed)
**Discovered by:** D07 Spec review
**Code files:** `src/underwater_planner/include/underwater_planner/core/main_planning_loop.hpp`、`src/underwater_planner/src/main_planning_loop.cpp`、`src/underwater_planner/include/underwater_planner/core/plan_validity_evaluator.hpp`、`src/underwater_planner/src/plan_validity_evaluator.cpp`、`src/underwater_planner/src/deterministic_closed_loop.cpp`、`src/underwater_planner/src/stability_manager.cpp`、`src/underwater_planner/include/underwater_planner/core/algorithm_diagnostics.hpp`、`src/underwater_planner/src/algorithm_diagnostics.cpp` 及对应主循环/验证器/稳定性/诊断/Level 1 闭环测试。
**What it delivers:** 主循环在同一最新同步上下文中复检新旧剩余计划并实际调用 `StabilityManager`，任何候选在取得发布权前都具有有限、非负的可审计代价。

- [x] 扩展候选决策 seam，使其取得同一最新 `SynchronizedValidationInputs` 下的新旧 `PlanValidityEvaluation`；不得用循环开始时的旧租约或复检结果参与滞回。
- [x] 当前计划无效且候选有效时仍须先校验候选代价有限且非负；候选代价无效时，有有效当前计划则保持其新租约，否则停车，不得直接发布候选。
- [x] 主循环基类提供不可覆盖的 `decide_candidate` 生产决策入口并调用 `StabilityManager::decide_path_switch`，确定性闭环适配器沿用该入口，删除“有效候选总是切换”的无条件策略；keep/switch 均原子携带对应剩余轨迹和本次租约。
- [x] 集成测试覆盖轻微改善保持、显著改善切换、旧计划失效后的有限候选直切、非有限/负候选代价拒绝、规划期间租约到期后的成对复检，以及同一上下文门禁。
- [x] 更新第10.2、14.2、16、18.2.7节与 Level 1 证据矩阵；完成前不得宣称主循环已集成路径滞回或所有发布分支都拒绝非有限候选代价。

Evidence: TDD 第一轮在 `test_stability_manager.cpp` 添加旧计划无效/缺失时对负值、`NaN` 和无穷候选代价的直切门禁，严格编译后测试以 `an invalid candidate cost bypassed the direct-switch gate` 红灯失败；把候选代价校验前移到任何 switch 分支之前后转绿。第二轮扩展 `MainPlanningLoopStages::decide_candidate`、候选元数据、原子授权与周期工件，旧接口先以缺失 `path_cost`、override 不匹配和缺少当前计划复检工件的编译红灯失败。规格终审随后发现真实验证器会把承诺拼接产生的新剖面版本误当成当前跟踪版本错配，候选普通失败在当前计划复检前提前返回，且候选错误沿用旧计划跟踪弧长；第三轮先加入 `validatePublicationCandidate` 缺失的严格编译红灯及候选失败绕过成对决策的行为测试，再以显式 `PlanValidationTarget`、候选/当前双入口和非虚 `decide_candidate` 基类门禁修复。候选以自身首点弧长作为当前实测状态的坐标原点，允许执行剖面版本单调前进但仍校验实测状态、连续性和全部冻结依赖；当前计划继续使用 `tracked_arc_length_m` 并严格绑定执行器跟踪版本。主循环只捕获一次最新 `SynchronizedValidationInputs`，以同一 `decision_validation_at` 成对复检候选和当前不可变计划；候选普通失败也通过已配置的 `StabilityManager` 完成 keep/stop，协方差或输入无效仍按实际失败阶段覆盖并停车。keep/switch 分别原子续签或替换，`PlanningCandidateMetadata`、`AuthorizedPlanningResult` 及 `algorithm-experiment/v3` 保留并校验有限非负审计代价。Level 1 总矩阵扩展为 44/44，其中第18.2.7节为 22/22 场景并执行 10 条支持链接，新增旧计划非零进度下从零起算候选的真实验证器证据、候选失败成对 keep/stop 及非默认滞回配置证据。`powershell -NoProfile -ExecutionPolicy Bypass -File tools\verify.ps1` 完成 MSVC 19.51 严格构建、`/analyze` 与 CTest 38/38（测试 192.37 s，完整流程 208.484 s，JUnit：`build/verify/ctest.xml`）；T47 主闭环报告 SHA-256 仍为 `DBCF77C742B00FEA4F085D1B2236C5FAEC42D46BBA24806242208E1EAC1A9879`。已逐项核对并同步设计首页、第10.2、10.5、14.2、16、18.2.7节和本追踪矩阵；最终双轴复审 Standards 与 Spec 均无剩余发现，真实坡度/载荷制动、执行偏差阈值和生产租约有效期仍受 T52 外部标定阻塞。

### K. 算法设计文档 v3.0 对齐

本组任务将 `ALGORITHM_DESIGN_REVISED.md` 从 v2.6 升级为按当前代码核验的 v3.0。代码基线为 `aabfe7f12b43d386e337fa4e2ebd6de338d5f857`（T47 完成提交）。本轮只更新设计与追踪文档，不修改算法行为；若代码、测试和安全设计不能同时成立，必须记录差异并另行立项，不能静默降低硬约束或夸大保证。

每个 D ticket 除满足本计划全局完成定义外，还必须：

1. 用当前公共契约、实现行为和相关测试反查所修改的设计章节。
2. 对“已实现”陈述给出接口、测试或确定性报告证据；缺少证据的内容只能标为设计目标、TBD、外部阻塞或未实现。
3. 核对硬约束、失败关闭、版本依赖、运行域和风险语义没有因文字整理而弱化。
4. 运行与该行为域相关的测试；D10 运行完整验证入口。
5. 在 ticket 下追加 `Evidence:`，记录设计章节、实现契约、测试命令和结果。

#### D01 - 建立 v3.0 现状基线与发布边界

**Status:** `[x] done`
**Blocked by:** None
**What it delivers:** 读者能从文档首页明确区分已实现算法、已验证范围、外部阻塞和首版不保证内容，并能定位 v3.0 所依据的代码与验证基线。

- [x] 将文档状态从“指导后续开发的实现基线”调整为“按当前代码核验的 as-built 设计草案”，最终发布状态留给 D10。
- [x] 重写 v3.0 修订说明、范围和设计原则，明确代码事实、规范性安全要求和外部能力声明的证据层级。
- [x] 记录本次对齐所使用的代码提交、公共模块清单、测试清单和既有 T47 确定性闭环证据。
- [x] 保留生产参数、外部标定、仿真、水池和海试门禁，不把 T48-T54 的阻塞状态写成已完成。

Evidence: 已核对设计 v3.0-draft 首页、第 1.1-1.6 节及第 4.1 节与提交 `aabfe7f12b43d386e337fa4e2ebd6de338d5f857` 上的 `src/underwater_planner/CMakeLists.txt`、32 个公共核心头文件、34 个生产核心实现单元、37 个测试源文件和 38 个 CTest 项，清单逐项匹配且未改变算法行为或测试标识；重新解析 `build/verify/t47_closed_loop_report.json`，确认 8 个主场景、4 个故障注入运行、18 个周期、12 个授权周期、6 次安全停车、5 次旧租约拒绝及 SHA-256 `DBCF77C742B00FEA4F085D1B2236C5FAEC42D46BBA24806242208E1EAC1A9879`，风险语义保持 `POINTWISE_ONLY:NO_PATH_JOINT_RISK_GUARANTEE`。`powershell -NoProfile -ExecutionPolicy Bypass -File tools/verify.ps1` 通过，CTest 38/38（T47 147.21 s，总测试 163.27 s，完整流程 166.837 s），JUnit：`build/verify/ctest.xml`；T48-T54 继续保持 `blocked-external`，正式 v3.0 发布仍仅允许由 D10 完成。双轴终审发现并修复临时 `PLAN1.md` 违反唯一进度源规则的问题，D01-D10 已迁入本文件；复审 Standards 与 Spec 均 PASS。

#### D02 - 对齐公共数据、参数与版本一致性契约

**Status:** `[x] done`
**Blocked by:** D01 (completed)
**What it delivers:** 设计中的输入、输出、单位、参数门禁、不可变快照、同步捕获、规划结果和诊断语义与当前公共契约一致，后续章节共享同一套术语和版本依赖。

- [x] 对齐状态、几何路径、时序剖面、误差预算、诊断、规划结果及其失败状态，清除重复或近似数据结构。
- [x] 对齐地图、参考线、机器人作业区、缆线施工走廊和完整依赖版本元组的冻结、回退与失效规则。
- [x] 对齐生产/非生产参数配置、运行域、标定数据集和风险策略门禁，示例值不得被描述为生产默认值。
- [x] 对齐同步验证输入、不可变结果发布、序列号、消息时间和非有限值失败关闭行为，并由相应契约测试核验。

Evidence: 已逐项核对设计第 3.3、13.1-13.5、14.3、15.1-15.3 节与 `data_contract.hpp/.cpp`、`versioned_snapshot.hpp/.cpp`、`synchronized_validation_inputs.hpp/.cpp`、`planning_result.hpp/.cpp`、`parameter_config.hpp/.cpp` 及其五个公共契约测试；删除第 14.3 节重复/近似结构，补齐 `uncertainty_envelope_generator_version`、`UP_RESULT 7`、SI/单调时间、失败结果非有限浮点拒绝、嵌套失败时间、原子捕获和不可变发布语义。T58、T59、T60、T61 已分别完成；T61 不再作为失败结果嵌套时间的 as-built 限制，T57/D10 仍受其余依赖阻塞。

#### D03 - 对齐地形分析与机器人通行性设计

**Status:** `[x] done`
**Blocked by:** D02 (completed)
**What it delivers:** 地形派生层、增量更新和机器人通行性章节完整描述当前鲁棒平面、台阶、碰撞误差预算、方向坡度与履带支撑行为，并与 Level 1 地形矩阵逐项对应。

- [x] 对齐鲁棒局部平面、协方差、去趋势粗糙度、支撑不足和病态拟合状态。
- [x] 对齐方向无关台阶几何、完整高度、双侧支撑、过渡宽度和无效状态。
- [x] 对齐机器人碰撞不确定性、未知区、复杂足迹、方向坡度、上下坡非对称、斜交台阶和履带支撑硬门禁。
- [x] 对齐增量地形更新的影响区扩张、只读共享、版本失效和回退规则，并核对地形与通行性验收矩阵。

Evidence: 已逐项核对设计第4.2-4.3、5.1-5.8、14.2、17.1、17.4、18.2.1-18.2.2节与 `terrain_analyzer.hpp/.cpp`、`step_geometry.cpp`、`traversability_evaluator.hpp/.cpp` 及五个地形/通行性公共接口测试；补齐三段式碰撞/扫掠/方向门禁、`DISCONTINUOUS`、台阶拒绝原因、梯度与碰撞风险审计、履带加权中位数/IQR/局部落差、增量影响区、只读缓存、全量失效和版本回退语义，并已闭合 T62 粗糙度运行时硬门禁。Level 1 可执行矩阵通过 `33/33`，其中增量单元指标为重算 49、复用 32；`powershell -NoProfile -ExecutionPolicy Bypass -File tools/verify.ps1` 严格构建、静态分析与 CTest 38/38 通过，JUnit：`build/verify/ctest.xml`。T63 的 `evaluate_collision_sweep` 地图时间戳门禁仍是已披露的未实现边界；T57/D10 不再受 T62 阻塞，但继续受 T63 及其他依赖阻塞。

#### D04 - 对齐缆线状态、模型、机械约束与走廊风险设计

**Status:** `[x] done`
**Blocked by:** D02 (completed)
**What it delivers:** 缆线从实际历史初始化、均值与协方差预测、机械约束、统计包络到时序候选复检的完整链路与当前实现一致，风险声明保持局部而非整路径联合保证。

- [x] 对齐实际缆线状态、参考进度、固定历史窗口、触地点均值模型、插值和模型有效性门禁。
- [x] 对齐缆线协方差传播、搜索包络构建/锁定/失效、离散裕量和运行域审计。
- [x] 对齐机械曲率、禁放区、悬空代理、软适宜性、走廊 PASS/MARGINAL/VIOLATION 与边缘弧长门禁。
- [x] 对齐完整时序候选的高精度复检和减速后重预测，并明确 `epsilon_path` 与误差相关长度仍未实现。

Evidence: `powershell -NoProfile -ExecutionPolicy Bypass -File tools/verify.ps1`; MSVC 19.51 严格编译、`/analyze` 与 CTest 全量通过。D04 绑定的八个缆线公共测试共 81 项行为检查：state tracker 10、reference progress tracker 11、model 14、laying 19、corridor 6、envelope builder 9、envelope manager 6、timed verifier 6；覆盖实际历史、参考进度局部关联/歧义/回退/版本边界、固定物理窗口与 supercover、均值/协方差传播、包络依赖与失效、走廊边缘弧长、完整时序重预测和减速新版本。设计第 6-7、8.6 中的缆线门禁、14.2、16 中的最终缆线复检及 18.2.4 节已按当前 `core/*.hpp` 接口和生产实现核对；公共走廊结果与包络审计风险语义分别保持为 `POINTWISE_ONLY:NO_PATH_JOINT_RISK_GUARANTEE` 和 `POINTWISE_ENVELOPE_ONLY:NO_PATH_JOINT_RISK_GUARANTEE`，`epsilon_path` 与误差相关长度仍明确未实现，Level 2-4/外部标定仍未宣称完成。

#### D05 - 对齐增广 Hybrid A* 与多并线目标设计

**Status:** `[x] done`
**Blocked by:** D02 (completed)
**What it delivers:** 搜索章节准确描述当前状态维度、反解并线目标、运动原语连续扫掠、机械历史多标签、硬约束优先代价和资源耗尽语义。

- [x] 对齐搜索状态、离散键、触地点/参考进度传播和多个并线目标生成与拒绝原因。
- [x] 对齐自适应连续扫掠、supercover、转弯外角、原语中段和跨原语软成本积分。
- [x] 对齐机械记忆、多标签等价/支配、代价分解、启发函数和硬约束剪枝。
- [x] 对齐超时、标签/内存预算、无解和确定性诊断，并核对搜索验收矩阵。

Evidence: 已逐项核对设计第7.7.1-7.7.2、8.1-8.8、14.2、15.2、17.1-17.4、18.2.3、20.6节及附录A，与 `merge_goal_generator.hpp/.cpp`、`hybrid_astar_planner.hpp/.cpp`、`reference_progress_tracker`、`CableLayingEvaluator` 和公共测试一致；补齐反解目标的九类拒绝、生成软成本仅排序/截断不重复计入搜索、五元基础键与机械历史多标签、触地点走廊梯形积分/缆线适宜性已积分边界、六族前进 Dubins 目标容差域启发与逐段解析扩展、完整硬门禁、纯包络/混合无解分类、三类资源 `TIMEOUT`、标签内存间接预算及确定性诊断，并删除旧 `MergePointSelector`/概念搜索接口和 Reeds-Shepp 现状声明。聚焦 `merge_goal_generator|hybrid_astar_planner|level1_search_cable_matrix` 3/3 通过（6.61 s），其中并线 7 项、Hybrid A* 46 项行为检查，Level 1 搜索/缆线矩阵 64/64；`powershell -NoProfile -ExecutionPolicy Bypass -File tools/verify.ps1` 完成 MSVC 19.51 严格构建、`/analyze` 与 CTest 38/38（测试 163.21 s，完整流程 166.636 s，JUnit：`build/verify/ctest.xml`）。当前仍只声明 pointwise envelope 风险，未新增 byte-budget、Reeds-Shepp、倒车搜索、增量/自适应分辨率或外部性能保证。

#### D06 - 对齐路径平滑、几何复检与时序参数化设计

**Status:** `[x] done`
**Blocked by:** D02 (completed)
**What it delivers:** 从搜索路径到可授权时序路径的处理链路与当前 G2 边界、独立几何审计、能力限制和停车距离门禁一致。

- [x] 对齐非零/负曲率边界、曲率变化率、求解器状态和 G2 拼接残差。
- [x] 对齐平滑后的完整足迹、完整拼接路径和当前地图独立复检，禁止以求解器成功替代安全验证。
- [x] 对齐速度、加减速、横向加速度、放缆/张力、停车距离和执行剖面版本契约。
- [x] 对齐平滑与时序阶段的超时分类、失败结果和 Level 1 验收矩阵。

Evidence: 已逐项核对设计第4.2、9.1-9.7、10.3、14.2、15.1-15.3、16、18.2.5-18.2.6节与 `path_smoother.hpp/.cpp`、`path_candidate_verifier.hpp/.cpp`、`trajectory_parameterizer.hpp/.cpp`、`data_contract.hpp/.cpp`、`main_planning_loop.cpp` 及对应公共接口测试；修正文档中平滑器错误接收地图、参数化器错误重复接收能力/地形、先几何拼接后参数化、把空间曲率变化率重复写成时域门禁、把 Level 1 恒定制动公式写成生产认证模型等偏差。聚焦 `path_smoother|path_candidate_verifier|trajectory_parameterizer|level1_smoothing_timing_stability_matrix` 4/4 通过（2.45 s），覆盖 PathSmoother 18 项、PathCandidateVerifier 4 项、TrajectoryParameterizer 6 项行为检查及 D06 的 18.2.5 14/14、18.2.6 7/7 设计编号绑定；`powershell -NoProfile -ExecutionPolicy Bypass -File tools/verify.ps1` 完成严格配置/构建、静态分析与 CTest 38/38（测试 181.48 s，完整流程 184.946 s，JUnit：`build/verify/ctest.xml`）。Spec 复审发现公共独立几何复检尚未自行拒绝 `planned_goal` 起点来源，已在设计中如实披露并登记 T64，未用上游正常来源掩盖接口缺口；当前停车诊断仍明确仅为 Level 1 合成门禁，坡度/载荷相关生产停车能力继续受外部标定阻塞。本 ticket 未修改算法行为。

#### D07 - 对齐计划复用、租约、滞回与承诺段安全设计

**Status:** `[x] done`
**Blocked by:** D02 (completed)
**What it delivers:** 旧计划只有在最新同步上下文中完成全量复检并获得新租约后才能继续执行，承诺段安全事件、异步撤租与安全停车优先级与当前实现一致。

- [x] 对齐不可变规划结果、剩余路径复检、短期执行租约签发和执行端授权拒绝语义。
- [x] 对齐获批剖面的版本配对与速度、出缆、张力、加速度偏差监控及异步撤租。
- [x] 对齐路径切换滞回、近端承诺段提取、G2/时序拼接和安全事件覆盖规则。
- [x] 对齐 STOP 优先级、认证停车距离、撤租先于安全停车通道，以及超时/无解分支的旧计划复用限制。

Evidence: 已逐项核对设计第 10.2-10.5、12.5、13.4-13.5、14.2、16、17.1、18.2.7 节与 `planning_result.hpp/.cpp`、`plan_validity_evaluator.hpp/.cpp`、`execution_lease_monitor.hpp/.cpp`、`stability_manager.hpp/.cpp`、`commitment_safety.hpp/.cpp`、`planning_state_machine.hpp/.cpp`、`main_planning_loop.hpp/.cpp` 及对应公共测试；修正绝对/相对滞回阈值、可选 Hausdorff 门禁、承诺速度同步与授权范围、自动递增剖面版本、完整最终验证回调、剩余计划固定复检顺序、租约最短失效时间、原子授权发布、执行偏差/序列水位、STOP 优先级和失败分支旧计划复用边界，并用当前实际类型替换旧 `PlanReuseStatus` 与缩减版 context 接口。D7 聚焦 `planning_result|plan_validity_evaluator|execution_lease_monitor|stability_manager|commitment_safety|planning_state_machine|main_planning_loop|level1_smoothing_timing_stability_matrix` 8/8 通过（1.30 s），其中 18.2.7 稳定性矩阵 18/18 并执行 8 条支持链接；`powershell -NoProfile -ExecutionPolicy Bypass -File tools/verify.ps1` 完成 MSVC 19.51 严格构建、`/analyze` 与 CTest 38/38（测试 186.86 s，完整流程 190.479 s，JUnit：`build/verify/ctest.xml`）。Spec 终审发现 `StabilityManager` 虽有独立实现和矩阵证据，但当前 `MainPlanningLoopStages::decide_candidate` 未接收旧计划复检且确定性适配器无条件选择有效候选；无有效旧计划的直切分支也早于候选代价有限性检查。两项均已在设计中如实披露并登记 T65，未把模块测试外推为主循环集成。本 ticket 只更新设计与追踪文档，未改变算法行为；真实坡度/载荷制动、执行偏差阈值和生产租约有效期仍受 T52 外部标定阻塞。

#### D08 - 对齐主动补探、消息恢复、状态机与主规划循环

**Status:** `[x] done`
**Blocked by:** D02 (completed)
**What it delivers:** 从同步输入到授权发布或安全停车的一次规划周期，以及信息缺口、补探闭环、消息异常和通信恢复行为，都能在设计中沿当前实现完整追踪。

- [x] 对齐信息缺口识别、紧迫度、补探目标、双机距离、请求去重、地图更新关联和超时。
- [x] 对齐规划事件、状态、指令、安全停车和恢复授权，不允许通信恢复绕过新鲜上下文与租约。
- [x] 对齐各消息流的去重键、乱序缓冲、序列缺口、watermark 和状态回退防护。
- [x] 重写主规划循环伪代码，使成功、超时、无解、等待地图、旧计划复用、承诺安全和授权发布路径与实现阶段顺序一致。

Evidence: 已逐项核对设计第 11、12、13.6-13.7、14.2、15.1-15.2、16 节与 `scout_coordinator.hpp/.cpp`、`planning_state_machine.hpp/.cpp`、`message_consistency.hpp/.cpp`、`communication_recovery.hpp/.cpp`、`main_planning_loop.hpp/.cpp` 及对应公共测试；设计现明确补探位于主规划周期外层、紧迫度只读取已批准 `TimedPath`、谨慎速度必须由新剖面和新租约授权、十类消息流的类型化去重/乱序/watermark、恢复边界后的九流同步上下文，以及主循环从输入捕获、承诺 provenance/安全复检、搜索/平滑/时序/完整机器人与缆线验证到成对最新上下文复检、滞回、租约和原子发布的真实顺序。Spec 终审进一步按实现修正补探进度方向、明确 9 个有序 `PlanningDirective`、确认地图去重身份仅由 `map_id + map_sequence` 构成而 `coordinate_frame` 只参与载荷校验，并区分不可变化的 locked input dependencies 与可由本周期参数化递增但不得回退的 `execution_profile_version`。聚焦 `scout_coordinator|planning_state_machine|message_consistency|main_planning_loop` 4/4 通过；D08 文档唯一标题、过时术语、代码围栏和控制字符检查通过。`powershell -NoProfile -ExecutionPolicy Bypass -File tools/verify.ps1` 完成严格工作流、静态分析目标与 CTest 38/38（unknown-gap 两周期闭环包含在 `level1_closed_loop_scenarios`），JUnit：`build/verify/ctest.xml`。本 ticket 只更新设计与追踪文档；T48-T49 双机仿真/长时通信、真实前置机运动、生产补探阈值和 T52 制动标定仍未宣称完成。

#### D09 - 收敛软件架构、验证证据与追踪矩阵

**Status:** `[x] done`
**Blocked by:** D03, D04, D05, D06, D07, D08 (completed)
**What it delivers:** 文档的模块图、接口清单、复杂度、测试、限制、参数清单和需求追踪能够一致指向 D03-D08 已核验的 as-built 行为，不再遗留“代码有、设计目录无”或“设计声称、测试无证据”的断点。

- [x] 重建核心模块架构和接口目录，覆盖当前公共模块，并明确测试支持代码不是生产接口。
- [x] 收敛数据流、模块所有权和跨模块依赖，删除过时别名，保留必要的设计概念到实现类型映射。
- [x] 用现有诊断、内存和确定性闭环证据更新复杂度与 Level 1 测试状态；外部 Level 2-4、基线对比和消融仍按真实状态标注。
- [x] 更新风险、限制、待标定参数、总结和术语表，逐项消除与正文及当前实现的矛盾；按用户指示，旧 PRD 内容不纳入本 ticket，保持第 21 节不变。

Evidence: 已对照 `ALGORITHM_DESIGN_REVISED.md` 第 4、14、15、17-20、22 节和附录 B，与 `src/underwater_planner/CMakeLists.txt`、32 个生产公共 `core/*.hpp`、34 个生产 `.cpp`、4 个 source-only helper、2 个 `testing/*.hpp` 及当前 CTest 清单逐项核验；第 14 节现覆盖 32/32 公共头文件并明确 `underwater_planner::test_support` 非生产边界，删除不存在的 `MapManager`/`MultiLayerMap` 架构和错误职责串联。复杂度撤销旧逐格字节、固定窗口/长距离内存和未实测阶段耗时，改为实际容器渐近量、T43 标签/进程内存证据和已实现/未实现优化状态；Level 1 固定证据、Level 2-4、基线和消融的证据缺口如实分离，动态进度仍只由本计划维护。文档检查确认 90 个代码围栏成对、关键章节唯一、32/32 公共头文件均在第 14 节出现、无遗留 `MapManager`/`MultiLayerMap`/旧固定内存与耗时声明，`git diff --check` 通过。`powershell -NoProfile -ExecutionPolicy Bypass -File tools/verify.ps1` 完成严格工作流并通过 CTest 38/38（测试 158.49 s，完整验证 161.652 s，JUnit：`build/verify/ctest.xml`；确定性闭环 142.82 s）。按 `code-review` skill 从固定点 `4960e6f3b594022083ee29eb8ee23c045d18d538` 执行 Standards/Spec 双轴复审，修正 PLAN 完成状态/Evidence、设计内动态 ticket 状态重复和“文档基线”误标后，两轴发现均已闭合；旧 PRD/第 21 节按用户指示未修改。

#### D10 - 完成 v3.0 双向审计与发布

**Status:** `[x] done`
**Blocked by:** D09, T58, T59, T60, T61, T62, T63, T64, T65 (completed)
**What it delivers:** v3.0 成为可发布、可追踪且不夸大保证的当前设计基线，本计划保存升级证据和剩余阻塞。

- [x] 执行“设计章节 -> 公共契约/实现 -> 测试证据”和“公共模块/关键行为 -> 设计章节”的双向审计，所有缺口均已修正或显式列为阻塞。
- [x] 全文检查版本、术语、章节引用、伪代码、状态枚举、单位、TBD、首版限制和风险语义一致性。
- [x] 运行完整验证入口，确认 3.0 文档没有通过改变测试标识或安全语义掩盖实现差异。
- [x] 将 v3.0 发布状态、基线提交、验证命令、结果和剩余阻塞记录在本计划中。

Evidence: 以固定代码点 `bd102b44ad217a42fd1dda815f61599b7dc10b79`（D09 完成提交）执行 D10 双向发布审计；`ALGORITHM_DESIGN_REVISED.md` 已发布为正式 v3.0，并新增第 1.7 节将设计域映射到公共契约、生产实现和可执行证据，同时将公共模块、关键状态及安全失败路径反向映射到规范章节。自动审计确认 32/32 个生产公共 `core/*.hpp` 全部列入第 14.2 节、34 个生产 `.cpp` 全部由 `underwater_planner_core` 构建、4 个 source-only helper 与 3 个 `underwater_planner::test_support` 实现保持非生产边界、37 个 `test_*.cpp` 对应 38/38 个已记录 CTest，22 个主章节唯一且 90 个代码围栏成对；清除全部 `3.0-draft`/待发布措辞，统一设计首页/第 1.1/1.5/1.7 节与 PLAN 首页的当前基线，明确 PLAN 第 3 节 v2.6 仅是历史开发起点，修正 D10 遗漏 T62 的依赖，并收敛 PLAN 整体状态和第 21.2 节把接口/试验计划覆盖写成全部完成的外部能力过度陈述。`powershell -NoProfile -ExecutionPolicy Bypass -File tools/verify.ps1` 完成严格配置、MSVC 构建、`/analyze` 与 CTest 38/38（0 失败，总测试 174.96 s，完整流程 179.757 s，JUnit：`build/verify/ctest.xml`）；未修改算法源文件、公共接口或测试标识。T48-T54 外部仿真/标定/水池/海试仍为 `blocked-external`，T55 为 `ready-for-agent`，T56 受 T55 阻塞，T57 继续受 T50-T52 生产标定阻塞；`epsilon_path`、误差相关长度、整路径联合地形风险和高保真柔性缆线动力学仍明确未实现，v3.0 发布不构成生产就绪或外部验收声明。

#### D ticket 依赖、追踪与完成定义

```text
D01 -> D02 -> D03 --\
             D04 ---\
             D05 ----+-> D09 -> D10
             D06 ---/
             D07 --/
             D08 -/
T58 -----------------------> D10
T59 -----------------------> D10
T60 -----------------------> D10
T61 -----------------------> D10
T62 -----------------------> D10
T63 -----------------------> D10
T64 -----------------------> D10
T65 -----------------------> D10
```

| 设计范围 | 对齐 ticket |
|---|---|
| 首页、1-4 范围/原则/总体架构 | D01, D02, D09 |
| 5 地形分析与通行性 | D03 |
| 6-7 缆线适宜性、模型与不确定性 | D04 |
| 8 Hybrid A* 与并线 | D05 |
| 9 平滑、几何复检与时序输出 | D06 |
| 10 计划复用、稳定性与承诺安全 | D07 |
| 11-13 补探、状态机、消息与版本 | D02, D08 |
| 14 软件模块与数据结构 | D02-D09 |
| 15 参数体系 | D02, D03-D08, D09 |
| 16 主规划循环 | D08 |
| 17-22 性能、测试、风险、标定与追踪 | D03-D09 |
| 附录公式与术语 | D03-D09 |

D10 只有在以下条件全部满足后才能完成：T58-T65 的公共契约、安全门禁与主循环集成缺口已实现并回写设计，设计文档标记为正式 v3.0；每项“已实现”陈述都有当前代码与测试证据；公共模块、关键状态和安全失败路径能双向追踪到设计章节；外部仿真、标定、水池、海试、基线对比、消融和生产就绪状态与本计划一致；完整验证通过或未运行项被明确记录。

## 5. 关键路径与并行前沿

初始只有 T01 可开始。完成公共契约后可并行推进地形、版本快照、缆线状态和时序契约。算法最短关键路径为：

`T01 -> T02 -> T03/T04 -> T06 -> T09/T10 -> T14/T15/T16/T17 -> T18/T19 -> T21-T25 -> T26-T29 -> T30-T34 -> T37-T41 -> T42-T47 -> T57`

当前建议的并行分工边界：

- 地形与机器人通行性：T06-T10、T44、T50。
- 缆线模型与统计风险：T11-T19、T45、T51。
- 搜索、平滑与时序化：T20-T29、T43、T55-T56。
- 稳定性、状态机与验证：T30-T42、T46-T49。

并行开发必须共享 T02 的公共契约和 T03/T04 的版本语义，不允许复制一套临时类型后再合并。

## 6. 设计章节追踪矩阵

| 设计章节 | 实施任务 |
|---|---|
| 1-4 范围、问题、坐标与架构 | T02-T05, T38 |
| 5 地形分析与通行性 | T06-T10, T44, T47, T50, T62 |
| 6 缆线落点适宜性 | T16-T17, T29, T45 |
| 7 缆线模型与不确定性 | T11-T19, T29, T45, T51 |
| 8 Hybrid A* | T20-T25, T45, T55 |
| 9 路径平滑与复检 | T26-T27, T46, T64 |
| 10 稳定重规划与租约 | T30-T34, T39-T40, T46, T65 |
| 11 主动补探 | T35-T36, T47-T49, T56 |
| 12 状态机与降级 | T37, T39-T41, T46-T49 |
| 13 数据一致性与版本 | T04-T05, T19, T30-T32, T41 |
| 14 软件模块与数据结构 | T02-T41, T64-T65 |
| 15 参数体系 | T03, T50-T52, T57 |
| 16 主规划循环 | T38-T41, T47, T65 |
| 17 复杂度与实时性 | T42-T43, T49 |
| 18 测试方案 | T44-T56, T64-T65 |
| 19 风险、限制与扩展 | 全局完成定义, T42, T57 |
| 20 待标定参数 | T03, T50-T54, T57 |
| 21 PRD 追踪 | T47-T57 |
| 22 总结与核心创新 | T55-T57 |

## 7. 首版明确不实现

- 原始声呐/定位数据处理与地图构建。
- 主机器人底层运动控制。
- 前置机器人完整三维路径规划。
- 高保真柔性缆线三维动力学；首版使用经标定的保守地形起伏代理。
- 整条路径联合走廊越界概率 `epsilon_path` 与误差相关长度方法。
- 整条路径联合地形梯度失效概率。
- 未经实际平台验证的 GPU 加速和性能声明。

这些项目不得被悄悄加入首版，也不得把“未实现”描述为“已由局部风险近似保证”。

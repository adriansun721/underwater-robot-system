# Ticket 13 首次工程基线

记录日期：2026-08-28（Asia/Taipei）

## 口径

- 模块：`scout_planner`，独立构建；未调用 `interfaces/`、`laying_planner/`
  或其他模块测试。
- 主机：Windows，Visual Studio Build Tools 2026 18.9.1。
- 编译器：MSVC 19.51.36256.0，x64，Debug，C++17。
- 构建入口：
  `powershell -NoProfile -ExecutionPolicy Bypass -File tools/verify.ps1 -Preset baseline`。
- 冷构建定义：目标 `build/baseline` 在命令开始前不存在；流程依次执行 CMake
  configure、Ninja build、MSVC `/analyze` 和 CTest。
- 内存口径：七场景夹具测试进程通过操作系统 API 记录的 peak working set；
  不代表编译器、Ninja 或全系统峰值。

## 首次结果

| 指标 | 结果 |
|---|---:|
| configure + build + static analysis + test | 8,288 ms |
| CTest | 2/2 passed，0.06 s |
| 七场景夹具测试 | 7 ms |
| 七场景夹具测试峰值 RSS | 4,676 KiB |
| JUnit | `build/baseline/ctest.xml` |

这些数值仅为非生产开发机上的首次工程基线，不是算法性能、实时性、安全认证、
SIL/HIL 或实机能力证据。

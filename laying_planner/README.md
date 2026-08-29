# Underwater Planner

This repository contains the ROS-agnostic C++ planning core for the underwater
cable-laying robot. ROS 2 adapters will remain outside the core library so the
algorithms and deterministic tests can run without a ROS installation.

## Verify

Windows (PowerShell):

```powershell
powershell -ExecutionPolicy Bypass -File tools/verify.ps1
```

Linux:

```bash
bash tools/verify.sh
```

The verification entry point configures the project, compiles with warnings as
errors, runs the compiler's static analyzer, executes CTest, and writes a JUnit
report to `build/verify/ctest.xml`. Both platform scripts invoke the same
`verify` workflow from `CMakePresets.json`.

Requirements:

- CMake 3.25 or newer
- MSVC or GCC with its built-in static analyzer, or Clang with `clang-tidy`
- Ninja

On Windows, `tools/verify.ps1` discovers CMake, Ninja, and MSVC from the latest
Visual Studio Build Tools installation. CMake does not need to be on `PATH`.

## Layout

```text
src/underwater_planner/include/  Public ROS-agnostic interfaces
src/underwater_planner/src/      Core and deterministic fixture implementation
src/underwater_planner/test/     Level 1 deterministic scenario tests
tools/                            Reproducible verification entry points
project.md/                       Algorithm design baseline and implementation plan
```

# QMEC

QMEC is a personal C++20 engine and editor prototype built to explore engine-level programming, rendering, ECS architecture, and custom physics.

## What is here

- A reusable C++ engine library with custom math, an ECS, scene hierarchy/serialization, and script components.
- A Direct3D 11 renderer with materials, textures, directional lighting, shadow mapping, and debug lines.
- A Qt editor with hierarchy, inspector, editor/game viewports, scene editing, and play mode.
- A custom rigid-body physics prototype with broadphase collision filtering, primitive collision tests, and an experimental GJK/EPA convex path.
- Early vehicle and wheel controller scaffolding in `Sandbox/Game`.

This is a work in progress, not a production-ready physics engine. Collision robustness, editor performance, and vehicle behavior are still being developed.

## Build

Currently supported: Windows x64 with Visual Studio 2022/MSVC, CMake 3.24+, and Qt 6 Widgets for the MSVC 2022 x64 toolchain. The first CMake configure fetches `nlohmann/json` from GitHub, so it needs network access.

From a Visual Studio Developer Command Prompt at the repository root:

```powershell
cmake -S . -B out/build -DQMEC_QT_ROOT="C:/Qt"
cmake --build out/build --config Debug --target QMECEditor
```

If Qt is installed elsewhere, set `QMEC_QT_ROOT` to its installation root. CMake searches that root for a `*/msvc2022_64` Qt package.

## Demo

A short editor/physics demo video will be added after capture.

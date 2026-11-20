# Zero Engine

An ultra-lightweight, zero-dependency 3D game and physics engine written in pure C for 64-bit Linux. Designed specifically for AI coding agents, high-fidelity rendering on low-spec hardware, and rock-solid deterministic simulation.

## Core Features
* **Zero External Dependencies**: Pure C11 linking only system `libX11`, `libGL`, `libc`, and `libm`. No SDL, GLFW, GLAD, GLEW, or GLM.
* **Zero-Malloc Runtime**: Custom 64-byte cache-aligned linear memory arenas and double-buffered scratch allocators backed by Linux `mmap`.
* **Hyper-Math Foundation**: SIMD-accelerated 3D vector and matrix math, with Dual Quaternion skinning to eliminate mesh pinching.
* **Manual GL 3.3 Core Loader**: Direct dynamic dispatch table loaded via `glXGetProcAddressARB`.
* **Headless AI Telemetry**: Native headless simulation mode enabling automated agent-based gameplay verification in milliseconds.

## Quickstart

### Build and Run Tests
```bash
make test
```

### Build Targets
```bash
make          # Compiles build/libzero_engine.a and build/test_zero_engine
make test     # Runs the automated test verification suite
make clean    # Cleans build artifacts
```

## Architecture
See `ZERO_ENGINE_MASTER_ARCHITECTURE.md` for full mathematical derivations, data-oriented memory layouts, and physics specifications.

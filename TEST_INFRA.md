# E2E Test Infra: Zero Engine (Phases 2 through 6)

## Test Philosophy
- **Opaque-Box & Requirement-Driven**: Tests derive directly from `ORIGINAL_REQUEST.md` and `PROJECT.md § Feature Inventory`, without depending on internal implementation details.
- **Methodology**: Systematic 4-Tier verification: Category-Partition (Tier 1), Boundary Value Analysis (Tier 2), Pairwise Combinatorial Testing (Tier 3), and Real-World Simulation Workloads (Tier 4).
- **Zero-Tolerance Enforcement**: Compiled with `gcc -std=c11 -O2 -Wall -Wextra -Werror`. Zero compiler warnings, zero segfaults, zero memory leaks.

---

## Feature Inventory Coverage Matrix
| # | Feature | Requirement | Tier 1 | Tier 2 | Tier 3 | Tier 4 |
|---|---------|-------------|:------:|:------:|:------:|:------:|
| 1 | Memory Arena & Scratch Pools | ARCH §2.2 | 5 | 5 | ✓ | ✓ |
| 2 | Pure X11 & Headless Lifecycle | ARCH §2.1 | 5 | 5 | ✓ | ✓ |
| 3 | Dynamic GL Function Loader | ARCH §2.1 | 5 | 5 | ✓ | ✓ |
| 4 | Vector, Matrix & Dual Quat Math | ARCH §3 | 5 | 5 | ✓ | ✓ |
| 5 | Procedural Geometry & Mesh Upload | Phase 2 R1 | 5 | 5 | ✓ | ✓ |
| 6 | 3D FPS Camera & Raycasting | Phase 2 R1 | 5 | 5 | ✓ | ✓ |
| 7 | GLSL Shader Compilation & Uniforms | Phase 2 R1 | 5 | 5 | ✓ | ✓ |
| 8 | Wavefront OBJ Fast Parser | Phase 3 R2 | 5 | 5 | ✓ | ✓ |
| 9 | Uncompressed 24/32-bit TGA & BMP | Phase 3 R2 | 5 | 5 | ✓ | ✓ |
| 10 | Binary .zmesh Cache with Linux mmap | Phase 3 R2 | 5 | 5 | ✓ | ✓ |
| 11 | Dynamic AABB Tree with SAH | Phase 4 R3 | 5 | 5 | ✓ | ✓ |
| 12 | GJK / EPA Narrowphase & CCD | Phase 4 R3 | 5 | 5 | ✓ | ✓ |
| 13 | Dual XPBD & Sequential Impulse Solver | Phase 4 R3 | 5 | 5 | ✓ | ✓ |
| 14 | Structure-of-Arrays (SoA) Layout | Phase 4 R3 | 5 | 5 | ✓ | ✓ |
| 15 | Analytical Atmospheric Sky & Sun GLSL | Phase 5 R4 | 5 | 5 | ✓ | ✓ |
| 16 | Microfacet Cook-Torrance PBR BRDF | Phase 5 R4 | 5 | 5 | ✓ | ✓ |
| 17 | Cascaded Shadow Maps (CSM) & 4-Tap PCF | Phase 5 R4 | 5 | 5 | ✓ | ✓ |
| 18 | Headless Simulation Determinism & PRNG | Phase 6 R5 | 5 | 5 | ✓ | ✓ |
| 19 | Spatial Telemetry engine_inspect_scene() | Phase 6 R5 | 5 | 5 | ✓ | ✓ |
| 20 | Frustum Culling & Screen-Space Raycast | Phase 6 R5 | 5 | 5 | ✓ | ✓ |
| 21 | Automated Gameplay Assertion Harness | Phase 6 R5 | 5 | 5 | ✓ | ✓ |
| 22 | Interactive Demo Binary (demo_3d_viewport) | Acceptance | 5 | 5 | ✓ | ✓ |

---

## Test Architecture
- **Location**: `tests/test_e2e_engine.c`
- **Invocation**: `make test_e2e` or `gcc -std=c11 -O2 -Wall -Wextra -Werror -Iinclude ... build/libzero_engine.a -lX11 -l:libGL.so.1 -lm -lpthread -o build/test_e2e && ./build/test_e2e`
- **Pass/Fail Semantics**: All test suites must execute with zero assertion failures, printing `[E2E TEST SUITE PASSED]` with exit code 0.

---

## Real-World Application Scenarios (Tier 4)
| # | Scenario | Features Exercised | Description |
|---|----------|--------------------|-------------|
| 1 | Headless Jitter-Free Rigid Body Stacking | F1, F11, F12, F13, F14, F18, F19 | 10-box vertical stack simulated under gravity for 600 ticks; assert velocity decays to zero without jitter. |
| 2 | High-Velocity CCD Projectile Anti-Tunneling | F4, F11, F12, F13, F18, F21 | High-speed sphere ($v = 200\text{ m/s}$) fired at thin box wall ($\text{thickness} = 0.05\text{ m}$); assert impact detected without tunneling. |
| 3 | Asset Pipeline Roundtrip & Zero-Copy Streaming | F1, F8, F9, F10, F5 | Generate OBJ, load TGA/BMP texture, save `.zmesh`, `mmap` `.zmesh`, verify vertex/index parity. |
| 4 | AI Telemetry Spatial Inspection & Perception | F4, F6, F11, F18, F19, F20, F21 | Advance world, query `engine_inspect_scene()`, validate JSON syntax, screen coordinates, distances, contacts. |
| 5 | PBR & CSM Shadow Mathematics & Frustum Culling | F4, F15, F16, F17, F20 | Compute 3-split CSM matrices, texel snapping, evaluate Cook-Torrance BRDF energy conservation ($k_d + k_s \le 1.0$). |

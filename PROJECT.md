# Project: Zero Engine (Phases 2 through 6)

## Architecture
Zero Engine is a high-performance, AI-native, pure C11/C++20 game engine built natively for Linux x86_64 with zero external dependencies (no GLFW, SDL, GLAD, GLEW, GLM, Assimp, STB, Bullet, PhysX). It uses native Linux `libX11`, `libGL.so.1`, and standard system headers (`libc`, `libm`, `sys/mman.h`).

### Subsystem Topology
- **Platform & Runtime Core**: Pure X11, GLX 1.4 context negotiation, dynamic GL function dispatch (`zero_gl_loader`), 64-byte aligned linear and scratch arenas (`zero_memory`), monotonic timing.
- **Hyper-Math Core**: Vectorized SSE/AVX `Vec2`, `Vec3`, `Vec4`, `Mat4`, `Quat`, `DualQuat` in `zero_math.h`.
- **View & Mesh System (Phase 2)**: Modular GLSL shaders, VAO/VBO/EBO mesh buffers, procedural geometry (Cube, Plane, UV Sphere), 3D FPS camera with raycasting.
- **Asset Pipeline Core (Phase 3)**: Fast zero-dependency Wavefront `.obj` parser, uncompressed 24/32-bit `.tga` and `.bmp` loaders, 64-byte aligned binary `.zmesh` cache with Linux `mmap` zero-copy streaming.
- **Physics Simulation Core (Phase 4)**: Self-balancing Dynamic AABB Tree broadphase with SAH, GJK+EPA CCD narrowphase for convex shapes (Sphere, Box, Capsule), dual XPBD/Sequential Impulse solver with warm-starting, cache-coherent Structure-of-Arrays (SoA) layout.
- **Low-Spec High-Fidelity Render Pipeline (Phase 5)**: Analytical Preetham/Bruneton atmospheric sky and sun scattering in GLSL (0.02ms GPU target), Cook-Torrance microfacet PBR (GGX, Schlick, Smith), Cascaded Shadow Mapping (CSM, 3 splits) with 4-tap PCF.
- **AI Agent Semantic Perception Interface (Phase 6)**: Headless simulation runner with deterministic tick loop (dt = 1/60s), structured JSON/text spatial telemetry (`engine_inspect_scene()`), automated gameplay assertion harness.
- **Interactive Binary**: `demo_3d_viewport` natively running on Linux X11/GLX with seamless headless fallback.

---

## Code Layout
```
zero_engine/
├── include/
│   ├── zero_core.h          # Macros, alignment, cache-line constants
│   ├── zero_memory.h        # Linear & scratch arena allocators
│   ├── zero_platform.h      # X11 windowing, event pump, headless detection
│   ├── zero_gl_defs.h       # OpenGL 3.3 Core & GLX 1.4 types, enums, constants
│   ├── zero_gl_loader.h     # Dynamic OpenGL function dispatch table
│   ├── zero_math.h          # Vector, matrix, quaternion, dual quaternion SIMD math
│   ├── zero_shader.h        # Shader compile, link diagnostics, uniform setters
│   ├── zero_mesh.h          # Vertex format, mesh buffers, procedural geometry
│   ├── zero_camera.h        # 3D FPS camera, projection, screen-to-ray
│   ├── zero_obj.h           # (Phase 3) Wavefront OBJ parser
│   ├── zero_image.h         # (Phase 3) 24/32-bit TGA and BMP image loaders
│   ├── zero_zmesh.h         # (Phase 3) Binary mesh container & mmap streaming
│   ├── zero_aabb.h          # (Phase 4) Dynamic AABB Tree with SAH
│   ├── zero_collision.h     # (Phase 4) GJK/EPA & CCD narrowphase
│   ├── zero_physics.h       # (Phase 4) Dual XPBD / Sequential Impulse solver
│   ├── zero_rigidbody_soa.h # (Phase 4) Structure-of-Arrays physics data layout
│   ├── zero_sky.h           # (Phase 5) Analytical atmospheric sky renderer
│   ├── zero_csm.h           # (Phase 5) Cascaded Shadow Maps with 3 splits & 4-tap PCF
│   ├── zero_render_pbr.h    # (Phase 5) Cook-Torrance Microfacet PBR renderer
│   ├── zero_perception.h    # (Phase 6) Spatial telemetry & JSON engine_inspect_scene()
│   └── zero_headless.h      # (Phase 6) Deterministic runner & gameplay assertions
├── src/
│   ├── zero_memory.c
│   ├── zero_platform.c
│   ├── zero_gl_loader.c
│   ├── zero_shader.c
│   ├── zero_mesh.c
│   ├── zero_camera.c
│   ├── zero_obj.c           # (Phase 3)
│   ├── zero_image.c         # (Phase 3)
│   ├── zero_zmesh.c         # (Phase 3)
│   ├── zero_aabb.c          # (Phase 4)
│   ├── zero_collision.c     # (Phase 4)
│   ├── zero_physics.c       # (Phase 4)
│   ├── zero_rigidbody_soa.c # (Phase 4)
│   ├── zero_sky.c           # (Phase 5)
│   ├── zero_csm.c           # (Phase 5)
│   ├── zero_render_pbr.c    # (Phase 5)
│   ├── zero_perception.c    # (Phase 6)
│   └── zero_headless.c      # (Phase 6)
├── tests/
│   ├── test_zero_engine.c   # Unit test verification suite
│   └── test_e2e_engine.c    # Comprehensive 4-Tier E2E acceptance test suite
├── examples/
│   └── demo_3d_viewport.c   # Interactive 3D viewport demo binary
└── Makefile                 # GCC build orchestration with -O2 -Wall -Wextra -Werror
```

---

## Feature Inventory
| # | Feature | Description | Milestone | Source |
|---|---------|-------------|-----------|--------|
| 1 | Makefile Build & Phase 2 Integration | Add all sources to Makefile, export GL_UNSIGNED_INT, verify 339 tests | M1 | Survey 1 |
| 2 | Procedural Geometry & Buffers | Cube (24v/36i), Plane (4v/6i), UV Sphere (parametric), VAO/VBO/EBO | M1 | Request R1 |
| 3 | 3D FPS Camera & Raycasting | Pitch/yaw clamping, WASD translation, screen-to-world raycasting | M1 | Request R1 |
| 4 | GLSL Shader Diagnostics & Uniforms | Compile/link error logs, mat4/vec3/float uniform setters | M1 | Request R1 |
| 5 | Wavefront OBJ Fast Tokenizer | Zero-malloc per-token ASCII parsing of v, vt, vn | M2 | Request R2 |
| 6 | OBJ Multi-Format Face Parser | Parses f v, f v/vt, f v/vt/vn, f v//vn with negative index resolution | M2 | Survey 2 |
| 7 | OBJ Polygon Triangulation | Triangle fan triangulation of arbitrary N-gons | M2 | Survey 2 |
| 8 | OBJ Scratch Vertex Deduplication | Open-addressed hash map in scratch arena mapping (v,vt,vn) to unique vertices | M2 | Survey 2 |
| 9 | Uncompressed 24/32-bit TGA Loader | TrueColor TGA loading, colormap skip, BGR swizzle, orientation bit 5 | M2 | Request R2 |
| 10 | Uncompressed 24/32-bit BMP Loader | 14-byte File + 40-byte DIB header, 4-byte row padding, BGR swizzle | M2 | Request R2 |
| 11 | .zmesh 64-byte Aligned Binary Container | Magic 0x48534D5A, vertex/index counts, offsets, AABB, bounding sphere | M2 | Request R2 |
| 12 | Linux mmap Zero-Copy Streaming | Direct mmap(MAP_SHARED|MAP_POPULATE) for instant VRAM streaming | M2 | Request R2 |
| 13 | Dynamic AABB Tree Node Hierarchy | Binary bounding volume hierarchy with node pool and free list | M3 | Request R3 |
| 14 | Fat AABB Predictive Velocity Expansion | AABB expanded by margin + v*dt to eliminate unnecessary reinsertions | M3 | Survey 2 |
| 15 | Surface Area Heuristic (SAH) Insertion | Greedy surface area cost minimization for optimal subtree insertion | M3 | Request R3 |
| 16 | Dynamic AABB Tree AVL Self-Balancing | Rebalances subtree rotations when height difference > 1 | M3 | Survey 2 |
| 17 | Branch-and-Bound Tree Raycast | Slab-intersection ray traversal against tree nodes | M3 | Survey 2 |
| 18 | Convex Support Functions | Extreme point evaluations for Sphere, Box (OBB), and Capsule | M3 | Survey 2 |
| 19 | GJK Simplex Solver | 1D to 4D simplex Voronoi region tracking for convex shapes | M3 | Request R3 |
| 20 | Expanding Polytope Algorithm (EPA) | Polytope expansion for contact normal, penetration depth, contact points | M3 | Request R3 |
| 21 | Continuous Collision Detection (CCD) | Swept-volume conservative advancement eliminating tunneling | M3 | Request R3 |
| 22 | XPBD Compliant Position Projection | Decoupled compliance parameter alpha = 1 / (k*dt^2) for soft constraints | M3 | Request R3 |
| 23 | Persistent Contact Manifold | Tracks up to 4 contact points per body pair with warm-starting | M3 | Request R3 |
| 24 | Sequential Impulse Solver | Warm-started normal and Coulomb friction velocity constraint solver | M3 | Request R3 |
| 25 | Structure-of-Arrays (SoA) Layout | 64-byte aligned contiguous float arrays for SIMD integration | M3 | Request R3 |
| 26 | Analytical Atmospheric Sky Model | Preetham/Bruneton Rayleigh lambda^-4 & Mie scattering in pure GLSL (0.02ms) | M4 | Request R4 |
| 27 | Analytical Sun Disc & Solar Corona | Finite angular diameter (0.533 deg) and forward Henyey-Greenstein aureole | M4 | Survey 3 |
| 28 | Horizon Ground Haze Blend | Smooth optical scattering transition across ground horizon | M4 | Survey 3 |
| 29 | GGX Normal Distribution D(h) | Trowbridge-Reitz microfacet orientation distribution | M4 | Request R4 |
| 30 | Schlick Fresnel F(v,h) | Angle-dependent reflectance with metallic/roughness parameterization | M4 | Request R4 |
| 31 | Smith Joint Masking-Shadowing G | Schlick-GGX height-correlated shadowing canceling singularity | M4 | Request R4 |
| 32 | CSM Frustum 3-Split Log-Uniform Blend | Practical 3-split logarithmic-uniform partition (near, mid, far) | M4 | Request R4 |
| 33 | CSM Rotation-Invariant Bounding Sphere | Minimal enclosing sphere for cascade slices eliminating rotation wobbly | M4 | Survey 3 |
| 34 | CSM Light Ortho Texel Snapping | Snapping light-space bounds to discrete integer texel steps | M4 | Survey 3 |
| 35 | CSM Depth Map FBO Generation | 3-target depth FBO setup without color buffers | M4 | Survey 3 |
| 36 | 4-Tap PCF Fragment Shadow Filtering | Slope-scaled depth bias and 4-tap diagonal grid percentage-closer filtering | M4 | Request R4 |
| 37 | Headless Simulation Runner | Deterministic tick runner (dt = 1/60s) with XorShift64 PRNG | M5 | Request R5 |
| 38 | Spatial Telemetry engine_inspect_scene() | Structured JSON/text serialization of tick, camera, visible entities, contacts | M5 | Request R5 |
| 39 | Camera Frustum Culling | Entity AABB testing against 6 view-projection frustum planes | M5 | Survey 3 |
| 40 | Screen-Space Coordinate Projection | 3D world to [u, v] screen space and NDC depth projection | M5 | Survey 3 |
| 41 | Automated Gameplay Assertion Harness | Deterministic assertion macros with diagnostic telemetry dumping on failure | M5 | Request R5 |
| 42 | Interactive Demo demo_3d_viewport | Native Linux X11/GLX runtime with PBR, CSM, sky, WASD camera, headless fallback | M6 | Request R1-R5 |
| 43 | 4-Tier E2E Test Suite | Comprehensive opaque-box test suite verifying 100% of features | E2E | Architecture §1 |

---

## Milestones
| # | Name | Scope | Dependencies | Status |
|---|------|-------|-------------|--------|
| M1 | Build Integration & Phase 2 Finalization | Update Makefile, fix GL_UNSIGNED_INT, verify 339 tests | none | IN_PROGRESS |
| M2 | Phase 3: Zero-Dependency Asset Pipeline | OBJ parser, 24/32-bit TGA/BMP, .zmesh mmap, GL texture loader | M1 | PLANNED |
| M3 | Phase 4: Hyper-Grade Physics Subsystem | Dynamic AABB Tree SAH, GJK/EPA CCD, Dual XPBD/SI solver, SoA | M1 | PLANNED |
| M4 | Phase 5: Low-Spec High-Fidelity Rendering Pipeline | GL FBO loader, Preetham sky, Cook-Torrance PBR, CSM 4-tap PCF | M1, M2 | PLANNED |
| M5 | Phase 6: AI Agent Semantic Perception Interface | Headless simulation runner, engine_inspect_scene() JSON, assertion harness | M1, M3 | PLANNED |
| M6 | Final Integration & Interactive Demo | demo_3d_viewport compilation & runtime, 100% E2E test pass | M2, M3, M4, M5 | PLANNED |

---

## Interface Contracts

### M1 ↔ M2 (Asset Pipeline)
- `ZeroArena` & `ZeroScratch` from `include/zero_memory.h` are used for all allocations. Zero `malloc`/`free` permitted.
- `ZeroMeshData` (`include/zero_mesh.h`) is the universal mesh representation populated by `zero_obj_load_file` and serialized by `zero_zmesh_save_file`.
- `ZeroGL` (`include/zero_gl_loader.h`) is extended with texture dispatch: `glGenTextures`, `glBindTexture`, `glTexImage2D`, `glTexParameteri`, `glActiveTexture`, `glDeleteTextures`, `glUniform1i`.

### M1 ↔ M3 (Physics Subsystem)
- `Vec3`, `Quat`, `Mat4` from `include/zero_math.h` are used across all bounding boxes, transforms, and impulses.
- `ZeroContactManifold` (`include/zero_collision.h`) provides contact normal, point_a, point_b, penetration depth.
- `ZeroRigidBodySoA` (`include/zero_rigidbody_soa.h`) stores contiguous aligned arrays for positions, velocities, forces, inertias, AABBs.

### M2 / M3 ↔ M4 (Rendering Pipeline)
- `ZeroCSM` (`include/zero_csm.h`) generates shadow maps sampled by `ZeroPBRShader` (`include/zero_render_pbr.h`).
- `ZeroSkyRenderer` (`include/zero_sky.h`) renders atmospheric background sky in a dedicated pass before or after PBR geometry.
- `ZeroGL` is extended with FBO dispatch: `glGenFramebuffers`, `glBindFramebuffer`, `glFramebufferTexture2D`, `glCheckFramebufferStatus`, `glDeleteFramebuffers`, `glDrawBuffers`.

### M3 ↔ M5 (Perception Interface)
- `engine_inspect_scene()` queries `ZeroDynamicAABBTree` and active `ZeroContactManifold` list from physics core.
- `zero_headless_step()` drives rigid body integration and collision detection deterministically at 60 Hz.

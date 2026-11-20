# Zero Engine: Master Architecture & Technical Specification

**Version:** 1.0.0  
**Target Environment:** Linux x86_64 (Kernel 5.x–7.x, glibc)  
**Dependencies:** 0 External Libraries (Pure C11/C++20, `libX11.so.6`, `libGL.so.1`, `libc`, `libm`)  
**Design Paradigm:** AI-Native, Code-First, Zero-Malloc Frame Loop, Cache-Coherent DOD  

---

## 1. System Topology & Core Subsystems

```
+-----------------------------------------------------------------------------------------+
|                                    AI AGENT INTERFACE                                   |
|   - Declarative Code-as-Scene DSL               - Headless Simulation & Determinism     |
|   - Semantic Spatial Telemetry (JSON/Text)      - Spatial Query Raycasts & Overlaps     |
+--------------------------------------------+--------------------------------------------+
                                             |
+--------------------------------------------v--------------------------------------------+
|                                  PLATFORM & RUNTIME CORE                                |
|   - Pure X11 Windowing & Event Pump             - Zero-Malloc Linear & Scratch Arenas   |
|   - Direct GLX 1.4 Context Negotiation          - 64-Byte Cache Aligned Allocators      |
|   - Manual GL 3.3 Core Function Dispatch        - POSIX High-Precision Monotonic Clock  |
+---------------------+---------------------------------------------+---------------------+
                      |                                             |
+---------------------v-----------------------+   +-----------------v---------------------+
|              HYPER-MATH CORE                |   |          ASSET PIPELINE CORE          |
| - SSE/AVX Vectorized Vec3, Vec4, Mat4       |   | - Zero-Dep Wavefront .obj Fast Parser |
| - Dual Quaternions (Skeletal Skinning)      |   | - Uncompressed .tga / .bmp Importer   |
| - Analytical Inverse Kinematics (FABRIK)    |   | - Direct mmap Binary Cache (.zmesh)   |
+---------------------+-----------------------+   +-----------------+---------------------+
                      |                                             |
+---------------------v---------------------------------------------v---------------------+
|                                 PHYSICS SIMULATION CORE                                 |
| - Dynamic AABB Tree Broadphase (Branch-Free Surface Area Heuristic)                     |
| - GJK + EPA Narrowphase & Continuous Collision Detection (CCD Swept Volume)             |
| - Dual Solver Core: XPBD (Soft Bodies/Cloth) + Sequential Impulse with Warm-Starting    |
| - Structure-of-Arrays (SoA) SIMD Contiguous Cache Memory Layout                         |
+--------------------------------------------+--------------------------------------------+
                                             |
+--------------------------------------------v--------------------------------------------+
|                          LOW-SPEC HIGH-FIDELITY RENDER PIPELINE                         |
| - Analytical Atmospheric Scattering & Sky Model (Preetham / Bruneton Equations)          |
| - Microfacet PBR (Cook-Torrance BRDF: GGX Normal Distribution + Schlick Fresnel)         |
| - Cascaded Shadow Mapping (CSM) with 4-Tap Percentage-Closer Filtering (PCF)            |
| - Lightweight Post-Processing (HDR Reinhard / ACES Tone Mapping, Vignette)              |
+-----------------------------------------------------------------------------------------+
```

---

## 2. Platform & Hardware Layer

### 2.1 Pure X11 / GLX Lifecycle (Zero Middleware)
* **Display & Windowing**: Direct interface to `libX11.so.6` (`XOpenDisplay`, `XCreateWindow`, `XNextEvent`).
* **Headless Detection**: If `DISPLAY` is unset or `--headless` flag is passed, the engine bypasses X11/GLX entirely, activating the headless AI simulation harness and CPU rasterizer buffer.
* **Modern OpenGL 3.3 Core Profile**:
  * Context created via `glXCreateContextAttribsARB` with `GLX_CONTEXT_CORE_PROFILE_BIT_ARB`.
  * Double buffering with 24-bit depth and 8-bit stencil buffers via `glXChooseFBConfig`.
* **Zero-Library Dynamic GL Loader**:
  * We eliminate GLAD and GLEW. Function pointers (`glGenVertexArrays`, `glShaderSource`, `glDrawElements`, etc.) are resolved directly using a single dynamic dispatch table loaded via `glXGetProcAddressARB`.

### 2.2 Memory Management: Zero-Malloc Runtime
* **Permanent Arena**: Dedicated heap block mapped once at startup via `mmap(MAP_PRIVATE | MAP_ANONYMOUS)` for permanent systems (scene graph, static meshes).
* **Frame Scratch Arenas**: Double-buffered scratch arenas reset at the boundary of each simulation frame ($O(1)$ pointer reset). Zero dynamic `malloc`/`free` during active gameplay.
* **Cache Coherency**: All allocations enforced at 64-byte alignment (`alignas(64)`) to match x86_64 L1 cache lines.

---

## 3. Hyper-Math & Animation Layer

* **Vector & Matrix SIMD**:
  * `Vec3`, `Vec4`, `Mat4` written with intrinsic SSE2/AVX instructions (`_mm_add_ps`, `_mm_mul_ps`, `_mm_dp_ps`).
  * Matrix multiplication unrolled with fused multiply-add (`_mm_fmadd_ps`).
* **Dual Quaternions ($\hat{q} = q_r + \epsilon q_d$)**:
  * Blends rotation and translation in a single continuous 8-float mathematical entity.
  * **Zero Mesh Pinching**: Completely solves the classic "candy-wrapper" artifact seen when twisting joints with standard matrix skinning.
  * **Computational Efficiency**: Skinning requires only 8 floating-point numbers per bone transform compared to 16 for a $4 \times 4$ matrix, halving uniform buffer bandwidth.

---

## 4. Hyper-Grade Physics Subsystem

### 4.1 Broadphase: Dynamic AABB Tree
* Incremental self-balancing binary bounding-volume hierarchy.
* Uses Surface Area Heuristic (SAH) and Morton spatial hashing for $O(N \log N)$ pair pruning.
* Fat AABB expansion with velocity prediction: $\text{AABB}_{\text{fat}} = \text{AABB} + \mathbf{v} \cdot \Delta t + \delta_{\text{margin}}$.

### 4.2 Narrowphase: GJK + EPA with Continuous Collision Detection
* **Gilbert-Johnson-Keerthi (GJK)**: Evaluates the Minkowski difference of arbitrary convex polyhedra, capsules, and spheres.
* **Expanding Polytope Algorithm (EPA)**: Finds the exact minimal penetration depth and contact normal when simplex contains origin.
* **Continuous Collision Detection (CCD)**: Employs conservative advancement along the swept trajectory, eliminating tunneling for high-velocity projectiles.

### 4.3 Unified Solver: XPBD + Sequential Impulse
* **Warm-Starting**: Contact manifolds persist across frames. Cached normal and tangent friction impulses are applied at the start of the tick, achieving rock-solid stacking of 50+ rigid bodies without jitter.
* **XPBD (Extended Position-Based Dynamics)**:
  * Metric compliance parameter $\alpha = \frac{1}{k \cdot \Delta t^2}$ decoupled from frame rate.
  * Solves soft bodies, cloth deformation, ropes, and articulated ragdoll joint limits in a unified position-level constraint projection loop.

---

## 5. Low-Spec High-Fidelity Rendering Pipeline

* **Analytical Sky & Atmosphere**:
  * Implements the Preetham/Bruneton analytical sky model in GLSL:
    $$\beta_{\text{Rayleigh}}(\lambda) = \frac{8\pi^3(n^2-1)^2}{3N\lambda^4}$$
  * Generates cinematic sunrise, golden hour, and twilight with sun disc and Mie aerosol scattering for 0.02ms of GPU time.
* **Microfacet PBR (Cook-Torrance)**:
  * Diffuse: Lambertian $\frac{c}{\pi}$.
  * Specular: $f_r = \frac{D \cdot F \cdot G}{4(\mathbf{n} \cdot \mathbf{l})(\mathbf{n} \cdot \mathbf{v})}$.
  * $D$: GGX / Trowbridge-Reitz normal distribution.
  * $F$: Schlick approximation for Fresnel reflectance.
  * $G$: Smith joint masking-shadowing function.
* **Cascaded Shadow Maps (CSM)**:
  * 3 directional shadow splits covering near, mid, and distant camera view frustums.
  * 4-tap Percentage-Closer Filtering (PCF) for soft, anti-aliased shadows on low-end integrated Intel/AMD GPUs.

---

## 6. AI Agent Semantic Perception Interface

Engineered for **blind (text-only) AI models** and AI coding IDEs:

### 6.1 Declarative Code-as-Scene
Games and levels are instantiated purely in readable code:
```c
Scene* scene = scene_create("demo_world");
Entity* sun = entity_spawn_directional_light(scene, (Vec3){-0.5f, -0.8f, 0.3f}, COLOR_SUN_GOLD);
Entity* player = entity_spawn_humanoid(scene, (Vec3){0.0f, 1.0f, 0.0f});
```

### 6.2 Structured Spatial Snapshot (`engine_inspect_scene()`)
Any AI agent or automated test can call a single API function to receive a structured JSON/plain-text snapshot of the world state:
```json
{
  "tick": 1420,
  "camera": {
    "pos": [0.0, 1.8, -4.0],
    "forward": [0.0, -0.2, 0.98]
  },
  "visible_entities": [
    {
      "id": "player",
      "distance": 4.12,
      "screen_coord": [0.50, 0.52],
      "state": "grounded_idle",
      "velocity": [0.0, 0.0, 0.0],
      "aabb_min": [-0.3, 0.0, -0.3],
      "aabb_max": [0.3, 1.8, 0.3]
    }
  ],
  "active_contacts": [
    {"entity_a": "player", "entity_b": "terrain", "normal": [0.0, 1.0, 0.0], "depth": 0.001}
  ]
}
```

### 6.3 Headless Simulation & Deterministic Testing
* The engine can advance time deterministically: `engine_step(1.0f / 60.0f)`.
* AI coding agents can write automated test scripts that verify gameplay mechanics, physics collisions, and pathfinding in milliseconds without rendering a single pixel to screen.

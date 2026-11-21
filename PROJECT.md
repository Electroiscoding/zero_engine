# Project: Zero Drift: Apex (Standalone 3D Racing Game)

## Architecture
"Zero Drift: Apex" is a high-performance, high-fidelity standalone 3D Racing Game built strictly and exclusively on top of the freestanding Zero Engine (`libzero_engine.a`) with zero external dependencies beyond `-lX11 -l:libGL.so.1 -lm -lpthread`.

The game features:
- **Full Game UI & Flow (ZeroUI)**: Main menu, car selection, track selection, settings, in-game racing HUD with analog/digital gauges, pause menu, and podium leaderboard.
- **Advanced XPBD Vehicle Physics & Automatic Transmission**: Raycast suspension with XPBD compliance and anti-roll bars, Brush tire friction model with Kamm's circle and power-slide drift mechanics, aerodynamics (splitter, wing, ground-effect diffuser, active airbrake), and 6-speed automatic transmission with PRND/reverse.
- **Autonomous AI Opponent Racers**: 4-car racing grid (Player + 3 AI rivals) navigating the 120-waypoint spline track using pure pursuit steering, anticipatory corner braking, forward-cone obstacle detection, lateral avoidance, and OBB inter-car collision resolution.
- **High-Fidelity Multi-Camera & Track Environment**: 4 dynamic camera views (Chase Cam with 2nd-order spring-damper lag and speed-sensitive FOV, Hood Cam, Cockpit View, Trackside TV Broadcast Cam / Bumper Cam); procedural spline track extrusion with asphalt, rumble curbs, barriers, and gantry; lit by Cascaded Shadow Maps (3 splits, 4-tap PCF) and Bruneton diurnal atmospheric sky.
- **Native Freestanding Audio Synthesizer**: Tri-backend architecture (dynamic ALSA `dlopen("libasound.so.2")` -> `aplay` raw pipe -> in-memory software ring buffer) with procedural sound synthesis: multi-harmonic RPM pitch-shifted V8 engine roar, lateral tire slip squeal, aerodynamic wind rush, and UI countdown chimes.
- **Standalone Game Target & Deterministic Verification**: Compiles via `make race_game` under `-std=c11 -O2 -Wall -Wextra -Werror`, passes 100% of master engine tests (`make test`), and verifies 100% of headless multi-car simulation tests.

---

## Code Layout
```
zero_engine/
├── include/
│   ├── zero_audio.h           # (M1) Freestanding procedural audio synthesizer & tri-backend
│   ├── zero_vehicle_physics.h # (M2) Advanced XPBD suspension, Brush tire model, 6-speed auto transmission
│   ├── zero_ai_racers.h       # (M3) Multi-car grid, pure pursuit AI steering, avoidance, OBB collision
│   ├── zero_race_camera.h     # (M4) 4-mode camera controller with spring-damper lag & FOV dilation
│   ├── zero_track_mesh.h      # (M4) Procedural spline track geometry (road, curbs, barriers, gantry)
│   ├── zero_race_ui.h         # (M5) ZeroUI game state machine, menus, analog/digital HUD, minimap
│   └── (existing engine headers: zero_core.h, zero_memory.h, zero_platform.h, zero_gl_loader.h,
│        zero_math.h, zero_mesh.h, zero_shader.h, zero_camera.h, zero_csm.h, zero_sky.h,
│        zero_ui.h, zero_vehicle.h, zero_track_spline.h, zero_physics.h)
├── src/
│   ├── zero_audio.c           # (M1) Procedural sound synthesis (engine, tire, wind, UI) & PCM streaming
│   ├── zero_vehicle_physics.c # (M2) XPBD suspension, anti-roll, tire friction, powertrain simulation
│   ├── zero_ai_racers.c       # (M3) AI lookahead, corner braking, overtaking, OBB collision solver
│   ├── zero_race_camera.c     # (M4) Camera mode updates, spring-damper math, FOV dilation, transitions
│   ├── zero_track_mesh.c      # (M4) Spline extrusion, curb striping, barrier generation
│   ├── zero_race_ui.c         # (M5) Menu flow rendering, tachometer gauge, speedometer, podium
│   ├── zero_race_game.c       # (M6) Standalone executable entry point, game loop, rendering pipeline
│   └── (existing engine sources)
├── tests/
│   ├── test_zero_engine.c     # Master engine verification suite (430 tests passing)
│   └── test_race_integration.c # (E2E) Headless multi-car race, telemetry, audio, camera verification
└── Makefile                   # Build orchestration with 'race_game' target and -Werror
```

---

## Feature Inventory
| # | Feature | Description | Milestone | Source |
|---|---------|-------------|-----------|--------|
| 1 | Freestanding Tri-Backend Audio Output | Dynamic ALSA `dlopen` -> `aplay` pipe -> software ring buffer with zero external link flags | M1 | Survey 3 / R5 |
| 2 | Procedural V8 Engine Sound Model | Multi-harmonic saw/square wave pitch-scaled by RPM + throttle-modulated resonant lowpass | M1 | Survey 3 / R5 |
| 3 | Procedural Tire Slip Squeal Model | 2-pole resonant bandpass noise modulated by lateral slip angle and stick-slip tremolo | M1 | Survey 3 / R5 |
| 4 | Procedural Aerodynamic Wind Rush | Speed-scaled lowpass noise with quadratic acoustic power scaling | M1 | Survey 3 / R5 |
| 5 | Procedural UI Beeps & Countdown Chimes | Harmonic sine bursts for menu navigation and 3-2-1-GO race countdown | M1 | Survey 3 / R5 |
| 6 | XPBD Compliant Suspension & Anti-Roll Bars | Unilateral compliant distance constraints with front/rear sway bar roll coupling | M2 | Survey 2 / R2 |
| 7 | Brush Tire Friction & Drift Model | Fiala brush tire friction model with Kamm's circle and handbrake rear-grip attenuation | M2 | Survey 2 / R2 |
| 8 | Multi-Element Aerodynamics | Dynamic pressure scaling, front splitter, rear wing downforce, ground diffuser, active airbrake | M2 | Survey 2 / R2 |
| 9 | 6-Speed Automatic Transmission | Polynomial engine torque curve (480 Nm @ 4500 RPM), kickdown, shift lockout, PRND reverse | M2 | Survey 2 / R2 |
| 10 | Spline Parameterization & Curvature | Arc length, tangent, normal, curvature, and cornering speed limits along `g_track_spline[120][2]` | M3 | Survey 2 / R3 |
| 11 | Pure Pursuit AI Steering & Corner Braking | Lookahead pure pursuit steering with anticipatory corner braking | M3 | Survey 2 / R3 |
| 12 | AI Lateral Avoidance & Overtaking | Forward-cone detection of opponent cars and lateral offset evasive maneuvers | M3 | Survey 2 / R3 |
| 13 | 4-Car Grid & OBB Collision Solver | Staggered grid (Player + 3 AI rivals) with pairwise OBB collision impulse resolution | M3 | Survey 2 / R3 |
| 14 | 4-Mode Dynamic Camera System | Chase Cam (2nd-order spring-damper lag, speed FOV), Hood Cam, Cockpit Cam, Bumper/TV Cam | M4 | Survey 3 / R4 |
| 15 | Procedural Spline Track Mesh Extrusion | Asphalt road with lane UVs, alternating red/white rumble curbs, Armco barriers, start gantry | M4 | Survey 3 / R4 |
| 16 | CSM & Diurnal Sky Shading Integration | 3-cascade shadow map rendering with 4-tap PCF and Bruneton sky lighting | M4 | Survey 3 / R4 |
| 17 | ZeroUI Game State Machine & Menus | Main Menu, Car Select, Track Select, Settings, Pause, and Leaderboard Podium | M5 | Survey 3 / R1 |
| 18 | In-Game Racing HUD Widgets | Analog/arc tachometer with redline flash, digital speedometer, gear, lap/pos, split delta, minimap | M5 | Survey 3 / R1 |
| 19 | Standalone Game Loop & Binary Target | `build/race_game` executable compiled via `make race_game` under `-Werror` | M6 | Survey 1 / Acceptance |
| 20 | Headless Deterministic Race Simulation | Deterministic fixed-step (`dt = 1/60s`) loop for automated testing | M6 | Survey 2 / R6 |
| 21 | Comprehensive 4-Tier E2E Test Suite | Autonomous multi-car integration test in `tests/test_race_integration.c` | E2E | Survey 3 / R6 |

---

## Milestones
| # | Name | Scope | Dependencies | Status |
|---|------|-------|-------------|--------|
| M1 | Native Freestanding Audio Synthesizer | `include/zero_audio.h`, `src/zero_audio.c` (tri-backend, engine roar, tire squeal, wind, chimes) | none | IN_PROGRESS |
| M2 | Advanced Vehicle Physics & Transmission | `include/zero_vehicle_physics.h`, `src/zero_vehicle_physics.c` (XPBD suspension, Brush tires, 6-speed auto) | none | IN_PROGRESS |
| M3 | Autonomous AI Opponent Racers | `include/zero_ai_racers.h`, `src/zero_ai_racers.c` (4-car grid, pure pursuit, avoidance, OBB collision) | M2 contract | IN_PROGRESS |
| M4 | Multi-Camera System & Track Visuals | `include/zero_race_camera.h`, `src/zero_race_camera.c`, `include/zero_track_mesh.h`, `src/zero_track_mesh.c` | none | IN_PROGRESS |
| M5 | Complete Game Flow & Menus (ZeroUI) | `include/zero_race_ui.h`, `src/zero_race_ui.c` (menus, HUD gauges, minimap, podium) | none | IN_PROGRESS |
| M6 | Final Game Integration & Standalone Binary | `src/zero_race_game.c`, `Makefile` (`make race_game`, main loop, 100% test pass) | M1, M2, M3, M4, M5 | PLANNED |
| E2E | E2E Testing Track | `tests/test_race_integration.c`, `TEST_INFRA.md`, `TEST_READY.md` (4-tier verification) | none | IN_PROGRESS |

---

## Interface Contracts

### M1 (Audio) ↔ M6 (Game Integration)
```c
typedef struct ZeroAudioParams {
    float engine_rpm;
    float throttle;
    float tire_slip_ratio;
    float vehicle_speed_kmh;
    bool trigger_shift_pop;
    bool trigger_menu_beep;
    bool trigger_countdown_red;
    bool trigger_countdown_green;
} ZeroAudioParams;

bool zero_audio_init(ZeroAudioEngine *audio, bool force_software);
void zero_audio_shutdown(ZeroAudioEngine *audio);
void zero_audio_update_params(ZeroAudioEngine *audio, const ZeroAudioParams *params);
void zero_audio_synthesize_buffer(ZeroAudioEngine *audio, int16_t *out_samples, uint32_t sample_count);
```

### M2 (Vehicle Physics) ↔ M3 (AI Racers) & M6 (Game)
```c
typedef struct ZeroVehicleEnhanced {
    Vec3 position;
    Vec3 velocity;
    Vec3 angular_velocity;
    Quat orientation;
    Mat4 transform;
    float mass;
    Vec3 inertia;
    Vec3 inv_inertia;
    ZeroWheel wheels[4];
    ZeroTransmission transmission;
    float engine_rpm;
    float drive_wheel_torque;
    float throttle_input;
    float brake_input;
    float steer_input;
    bool handbrake;
    float speed_kmh;
    float drift_angle;
    bool is_drifting;
    int lap_count;
    float current_lap_time;
    float best_lap_time;
    float track_progress_s;
    int race_position;
    float aero_airbrake_active;
} ZeroVehicleEnhanced;

ZeroVehicleEnhanced zero_vehicle_enhanced_create(Vec3 spawn_pos, Quat spawn_rot);
void zero_vehicle_enhanced_update(ZeroVehicleEnhanced *car, float dt, ZeroGroundHeightFn ground_fn);
```

### M3 (AI Racers) ↔ M6 (Game)
```c
#define ZERO_RACE_CAR_COUNT 4

typedef struct ZeroRaceGame {
    ZeroVehicleEnhanced cars[ZERO_RACE_CAR_COUNT]; // Car 0 = Player, Cars 1-3 = AI
    ZeroAIController ai_controllers[3];
    uint64_t tick;
    float race_time;
    bool race_started;
    bool race_finished;
    int total_laps;
    uint32_t winning_car_index;
} ZeroRaceGame;

ZeroRaceGame zero_race_game_create(void);
void zero_race_game_step(ZeroRaceGame *game, float dt);
void zero_race_game_resolve_car_collisions(ZeroRaceGame *game);
void zero_race_game_update_rankings(ZeroRaceGame *game);
```

### M4 (Camera & Track) ↔ M6 (Game)
```c
ZeroRaceCamera zero_race_camera_create(float aspect);
void zero_race_camera_cycle_mode(ZeroRaceCamera *rcam);
void zero_race_camera_update(ZeroRaceCamera *rcam, Vec3 car_position, Quat car_orientation,
                             Vec3 car_velocity, float car_rpm, float steer_input, float dt);
Mat4 zero_race_camera_get_view(const ZeroRaceCamera *rcam);
Mat4 zero_race_camera_get_projection(const ZeroRaceCamera *rcam);

ZeroTrackMeshes zero_track_meshes_generate(ZeroArena *arena);
void zero_track_meshes_draw(const ZeroTrackMeshes *meshes, ZeroShader *pbr_shader);
```

### M5 (UI) ↔ M6 (Game)
```c
bool zero_game_ui_init(ZeroGameUI *gui);
void zero_game_ui_shutdown(ZeroGameUI *gui);
void zero_game_ui_render(ZeroGameUI *gui, const ZeroGameTelemetry *telem,
                         float screen_w, float screen_h,
                         float mouse_x, float mouse_y, bool mouse_down, float dt);
```

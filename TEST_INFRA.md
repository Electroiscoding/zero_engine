# E2E Test Infrastructure & Verification Specification: Zero Drift: Apex

## 1. Executive Summary & Verification Philosophy

"Zero Drift: Apex" is a high-performance, standalone 3D racing game built strictly and exclusively on top of the freestanding Zero Engine (`libzero_engine.a`) without external dependencies beyond `-lX11 -l:libGL.so.1 -lm -lpthread`.

The end-to-end verification strategy enforces a requirement-driven, opaque-box testing paradigm:
1. **Opaque-Box Requirement Derivation**: All test suites derive directly from the requirements in `ORIGINAL_REQUEST.md` and feature contracts in `PROJECT.md`.
2. **Headless & Deterministic Execution**: All gameplay, AI navigation, physics integration, audio synthesis, camera transforms, and UI states execute deterministically at a fixed timestep ($\Delta t = 1/60\text{s}$) in headless environments (without requiring an active X11 display server or audio hardware daemon).
3. **Multi-Tiered Coverage**: Testing is organized into four rigorous tiers:
   - **Tier 1 (Feature Coverage)**: Category-partition verification validating every single feature in the Feature Inventory under standard operating conditions ($\ge 5$ test cases per feature).
   - **Tier 2 (Boundary & Corner Cases)**: Stress and boundary value analysis probing numerical extremes, zero/negative bounds, mathematical wrap-around, and physical limits ($\ge 5$ test cases per feature).
   - **Tier 3 (Cross-Feature Combinations)**: Pairwise combinatorial testing verifying non-trivial interactions between concurrent game subsystems.
   - **Tier 4 (Real-World Application Scenarios)**: Realistic end-to-end race scenarios simulating full multi-car pack races, emergency evasive maneuvers, reverse circuits, and high-speed multi-car collisions.
4. **Zero-Tolerance Quality Gate**: Strict compilation under `gcc -std=c11 -O2 -Wall -Wextra -Werror`. Zero compilation warnings, zero memory leaks, zero placeholders, and zero assertion failures.

---

## 2. Feature Inventory Verification Mapping

The E2E test infrastructure verifies the 6 core functional subsystems defined in `PROJECT.md` and `SCOPE.md`:

| Feature ID | Feature Name | Primary Milestone | Verification Focus | Target File(s) |
|:---:|:---|:---:|:---|:---|
| **F1** | Multi-Car Autonomous AI Navigation | M3 | Pure pursuit spline following, lookahead steering, anticipatory braking, lateral collision avoidance, OBB collision resolution | `zero_ai_racers.h`, `zero_ai_racers.c` |
| **F2** | HUD Telemetry Determinism | M5 / M6 | Digital speedometer, analog arc tachometer, PRND/1-6 gear indicator, lap timer, split delta, deterministic position ranking | `zero_race_ui.h`, `zero_race_ui.c` |
| **F3** | Procedural Audio Synthesizer | M1 | Freestanding tri-backend, 44.1kHz PCM software buffer, pitch-scaled engine roar, tire squeal, wind noise, UI beeps/chimes | `zero_audio.h`, `zero_audio.c` |
| **F4** | Multi-Camera System & Orthonormality | M4 | 4 camera modes (Chase, Hood, Cockpit, TV/Bumper), 2nd-order spring-damper lag, speed-sensitive FOV dilation, view matrix orthonormality | `zero_race_camera.h`, `zero_race_camera.c` |
| **F5** | UI State Machine Transitions | M5 | Main Menu, Car Select, Track Select, Settings, Countdown (3-2-1-GO), Racing HUD, Pause Overlay, Leaderboard Podium | `zero_race_ui.h`, `zero_race_ui.c` |
| **F6** | Build Integrity & Freestanding Standard | M6 | Strict C11 compliance, freestanding link constraints (`-lX11 -l:libGL.so.1 -lm -lpthread`), zero warnings under `-Werror`, regression test parity | `Makefile`, `build/libzero_engine.a` |

---

## 3. Test Architecture & Execution Framework

### 3.1 Test Harness Location & Binary Target
- **Integration Test Source**: `tests/test_race_integration.c`
- **Compiled Test Binary**: `build/test_race_integration`
- **Master Engine Test Suite**: `tests/test_zero_engine.c` (`build/test_zero_engine`)

### 3.2 Compilation & Linkage Commands
The standalone integration test compiles with identical strict optimization and warning flags as the core engine:
```bash
gcc -std=c11 -O2 -Wall -Wextra -Werror -Iinclude -D_POSIX_C_SOURCE=199309L \
    tests/test_race_integration.c build/libzero_engine.a \
    -lX11 -l:libGL.so.1 -lm -lpthread -o build/test_race_integration
```

### 3.3 Test Execution Command
```bash
./build/test_race_integration
```

### 3.4 Pass/Fail Semantics & Exit Codes
- **Return Code `0`**: 100% of test assertions passed with zero failures.
- **Non-Zero Return Code (`1`)**: One or more assertions failed. Detailed error diagnostic with file name, line number, and expression is logged to `stderr`.
- **Diagnostic Format**:
  ```text
  === ZERO DRIFT: APEX - COMPLETE 4-TIER E2E INTEGRATION SUITE ===
  [PASS] Tier 1: F1 - Multi-Car Autonomous AI Navigation (6/6 checks)
  [PASS] Tier 1: F2 - HUD Telemetry Determinism (6/6 checks)
  ...
  [ALL E2E INTEGRATION TESTS PASSED] 110 / 110 checks verified successfully.
  ```

---

## 4. Tier 1: Feature Coverage Specifications ($\ge 5$ Tests Per Feature)

### 4.1 Feature 1: Multi-Car Autonomous AI Navigation (`F1`)

#### `T1-AI-01`: Grid Initialization & Staggered Spacing
- **Description**: Verify that `zero_race_game_create()` initializes all 4 vehicles (Player Car 0, AI Cars 1-3) on the starting straight with proper staggered grid coordinates.
- **Initial Conditions**: Staggered starting line at $x \approx 125.0\text{m}$, heading $+Z$.
- **Verification Logic**:
  - Distance between consecutive cars along track heading: $\Delta z \ge 7.0\text{m}$.
  - Lateral grid offset alternating left/right: $|\Delta x| \ge 2.5\text{m}$.
  - Pairwise Euclidean distance between all 4 vehicle centers: $d_{ij} \ge 5.0\text{m}$ (no initial bounding box overlap).
  - All vehicle linear and angular velocities initialized to $\vec{0}$.

#### `T1-AI-02`: Spline Progression & Waypoint Traversal
- **Description**: Advance simulation for 600 ticks ($\Delta t = 1/60\text{s}$, 10 seconds of simulated racing). Verify that all 3 AI cars make monotonic forward progress along the 120-waypoint spline.
- **Verification Logic**:
  - For each car $k \in \{1, 2, 3\}$:
    - Forward displacement along $+Z$: $\Delta z_{600} > 30.0\text{m}$.
    - Forward spline arc length progress: $s_{600} > s_0$.
    - Current speed: $v_{speed} > 15.0\text{ m/s}$ ($54\text{ km/h}$).

#### `T1-AI-03`: Track Boundary Compliance (Barrier Distance Limit)
- **Description**: Verify that throughout 600 ticks of autonomous navigation, all 4 cars remain strictly within the track boundary corridor.
- **Verification Logic**:
  - Track spline has outer barrier at $D_{barrier} = 11.0\text{m}$.
  - At every tick $t \in [0, 600]$, for each vehicle $k \in \{0, 1, 2, 3\}$:
    - Compute Euclidean distance to closest spline segment: $d_{spline}(pos_k) \le 11.0\text{m}$.
    - Assert no vehicle escapes the track or clips through perimeter barriers.

#### `T1-AI-04`: Upright Dynamic Stability (No Roll/Flip)
- **Description**: Verify that XPBD suspension, anti-roll bars, and tire cornering stiffness maintain chassis upright orientation during full throttle acceleration and cornering.
- **Verification Logic**:
  - Extract vehicle transformed up-vector: $\vec{u} = \text{quat\_rotate}(orientation, (0, 1, 0))$.
  - Assert vertical alignment dot product: $\vec{u} \cdot (0, 1, 0) \ge 0.60$ (body roll and pitch bounded to $< 53^\circ$).
  - Assert vehicle $y$-coordinate remains within realistic suspension ride-height bounds: $y \in [0.1\text{m}, 2.0\text{m}]$ above ground.

#### `T1-AI-05`: Curvature-Aware Anticipatory Corner Braking
- **Description**: Verify that the AI controller evaluates spline curvature ahead of the vehicle and applies brake input prior to entering high-curvature corners.
- **Verification Logic**:
  - When approaching sharp curve ($R < 40\text{m}$, curvature $\kappa > 0.025\text{ m}^{-1}$):
    - AI `throttle_input` drops to $0.0$.
    - AI `brake_input` increases to $\ge 0.40$.
    - Vehicle speed stays below maximum cornering velocity: $v \le \sqrt{\mu g / \kappa} + 5.0\text{ m/s}$.

#### `T1-AI-06`: Forward-Cone AI Overtaking & Lateral Avoidance
- **Description**: Place a slower vehicle directly ahead of an AI racer on a straightaway; verify the trailing AI racer detects the obstacle in its forward cone and steers laterally.
- **Initial Conditions**: Car 1 at $z = 100\text{m}$ ($v = 15\text{ m/s}$); Car 2 at $z = 70\text{m}$ ($v = 30\text{ m/s}$).
- **Verification Logic**:
  - Trailing AI detects lead car within forward cone ($25\text{m}$, $\pm 15^\circ$).
  - AI alters its `current_lateral_offset` by $\pm 3.0\text{m}$.
  - AI completes the pass without colliding: minimum center distance during pass $d_{min} \ge 2.2\text{m}$.

---

### 4.2 Feature 2: HUD Telemetry Determinism (`F2`)

#### `T1-HUD-01`: Speedometer Velocity Tracking
- **Description**: Verify that the HUD digital and analog speedometer readings strictly match the vehicle's physical velocity norm converted to km/h.
- **Verification Logic**:
  - For physical velocity $\vec{v}$: $v_{expected} = \|\vec{v}\| \times 3.6\text{ km/h}$.
  - Assert HUD telemetry value: $|telemetry.speed\_kmh - v_{expected}| \le 0.01\text{ km/h}$.

#### `T1-HUD-02`: Tachometer RPM & Redline Indicator Tracking
- **Description**: Verify that engine RPM accurately reflects drivetrain state and activates the redline warning indicator when exceeding $8000\text{ RPM}$.
- **Verification Logic**:
  - At idle: $RPM \approx 800\text{ RPM}$, `redline_active == false`.
  - Under full acceleration in 1st gear: RPM rises monotonically from $800$ to $> 7500\text{ RPM}$.
  - When $RPM \ge 8000\text{ RPM}$: `telemetry.redline_active == true`.

#### `T1-HUD-03`: Automatic Transmission Gear Indicator
- **Description**: Verify that HUD displays correct gear representation (`P`, `R`, `N`, `1`, `2`, `3`, `4`, `5`, `6`) matching the automatic transmission state.
- **Verification Logic**:
  - As vehicle accelerates from $0$ to $180\text{ km/h}$:
    - Speeds $< 40\text{ km/h}$: displays `1`.
    - Speeds $40 - 75\text{ km/h}$: displays `2`.
    - Speeds $75 - 115\text{ km/h}$: displays `3`.
    - Upshifts occur without skipping gears; gear string matches `1` through `6`.

#### `T1-HUD-04`: Lap Time Accumulator & Lap Counter Increment
- **Description**: Verify that `current_lap_time` advances monotonically by $dt$ each tick and `lap_count` increments upon crossing the start/finish line.
- **Verification Logic**:
  - After $N$ ticks of $\Delta t = 1/60\text{s}$: $t_{lap} \approx N \times (1/60)\text{s} \pm 1e-4\text{s}$.
  - When car crosses start/finish gantry heading $+Z$: `lap_count` increments from $0 \to 1$, `current_lap_time` resets to $0.0$, and `best_lap_time` records previous lap time.

#### `T1-HUD-05`: Real-Time Position Ranking Calculation
- **Description**: Verify that `zero_race_game_update_rankings()` sorts all 4 vehicles deterministically based on total laps completed and spline arc-length progress $s$.
- **Verification Logic**:
  - If Car A has completed 1 lap and Car B has completed 0 laps: Car A rank < Car B rank.
  - If Car A and Car B have identical lap counts: Car with greater spline progress $s$ is ranked ahead.
  - Rankings array contains a strict permutation of $\{0, 1, 2, 3\}$ without duplicates.

#### `T1-HUD-06`: Split-Time Delta Calculation Relative to Leader
- **Description**: Verify that split-time delta reflects positive time gap for trailing cars and zero for the race leader.
- **Verification Logic**:
  - Leader ($1\text{st}$ place): `split_delta == 0.0f`.
  - P2 car: `split_delta > 0.0f` representing distance gap divided by average speed: $\Delta t \approx \Delta s / v_{avg}$.

---

### 4.3 Feature 3: Procedural Audio Synthesizer (`F3`)

#### `T1-AUD-01`: Software Backend Buffer Generation
- **Description**: Verify that `zero_audio_init(&audio, true)` initializes the software ring buffer backend without hardware dependencies and `zero_audio_synthesize_buffer()` produces 16-bit mono PCM chunks at 44.1 kHz.
- **Verification Logic**:
  - Allocation succeeds: `audio.backend == ZERO_AUDIO_BACKEND_SOFTWARE`.
  - Synthesize $4410$ samples ($0.1\text{s}$ at $44.1\text{kHz}$): function completes synchronously without errors.

#### `T1-AUD-02`: Waveform Boundedness & Finite Output Limits
- **Description**: Verify that the synthesized audio buffer contains strictly valid numerical samples: zero NaNs, zero Infs, and zero digital clipping overflow.
- **Verification Logic**:
  - For all $i \in [0, N-1]$:
    - Sample is finite: `isfinite((float)buffer[i]) == true`.
    - Sample value strictly bounded to signed 16-bit range: $-32767 \le buffer[i] \le +32767$.
  - Root Mean Square (RMS) amplitude is non-zero: $RMS = \sqrt{\frac{1}{N} \sum buffer[i]^2} > 100$ (engine audio is active).

#### `T1-AUD-03`: Multi-Harmonic V8 Engine Pitch Scaling with RPM
- **Description**: Verify that engine fundamental pitch $f_0$ scales linearly with RPM: $f_0 = (\text{RPM} / 60) \times 4$.
- **Verification Logic**:
  - At $1200\text{ RPM}$: $f_0 = 80\text{ Hz}$. Measure zero-crossings per second: $80 \pm 4\text{ Hz}$.
  - At $6000\text{ RPM}$: $f_0 = 400\text{ Hz}$. Measure zero-crossings per second: $400 \pm 10\text{ Hz}$.
  - Pitch scales monotonically with RPM.

#### `T1-AUD-04`: Resonant Bandpass Tire Slip Squeal Triggering
- **Description**: Verify that tire squeal procedural sound triggers when tire slip ratio exceeds the adhesion limit ($> 0.15$).
- **Verification Logic**:
  - With `tire_slip_ratio = 0.0`: tire squeal component RMS is $0$.
  - With `tire_slip_ratio = 0.45` and speed $v = 80\text{ km/h}$: tire squeal generates resonant noise around $1350\text{ Hz}$ with $32\text{ Hz}$ stick-slip envelope modulation; output RMS increases significantly.

#### `T1-AUD-05`: Speed-Scaled Aerodynamic Wind Rush Modulation
- **Description**: Verify that aerodynamic wind noise cutoff frequency and acoustic power scale quadratically with vehicle velocity.
- **Verification Logic**:
  - At $v = 0\text{ km/h}$: wind noise amplitude $\approx 0$.
  - At $v = 100\text{ km/h}$ vs $v = 200\text{ km/h}$: acoustic power ratio $P_{200} / P_{100} \approx (200 / 100)^2 = 4.0 \pm 0.5$.

#### `T1-AUD-06`: Procedural UI Beeps & Countdown Tone Chimes
- **Description**: Verify that triggering UI beeps and countdown chimes produces distinct, clean harmonic sine bursts.
- **Verification Logic**:
  - Menu Beep trigger: generates $880\text{ Hz}$ sine burst decaying over $35\text{ ms}$.
  - Countdown Red (3, 2, 1): generates $440\text{ Hz}$ tone for $0.25\text{ s}$.
  - Countdown Green (GO!): generates major triad chord ($880\text{ Hz} + 1100\text{ Hz} + 1320\text{ Hz}$) for $0.65\text{ s}$.

---

### 4.4 Feature 4: Multi-Camera Modes & Orthonormality (`F4`)

#### `T1-CAM-01`: Chase Cam 2nd-Order Spring-Damper Lag & Orthonormality
- **Description**: Verify that `ZERO_CAM_CHASE` tracks vehicle position with 2nd-order critically damped lag and produces a strictly orthonormal view matrix.
- **Verification Logic**:
  - Upper 3x3 rotation submatrix $R$ of view matrix $V$:
    - $\det(R) = 1.000 \pm 0.001$.
    - Orthogonality: $\|R R^T - I_3\|_\infty < 1e-4$.
  - Rapid forward acceleration creates trailing lag distance $\Delta d \in [5.5\text{m}, 8.0\text{m}]$ behind car.

#### `T1-CAM-02`: Hood Cam Rigid Mount & Engine Vibration
- **Description**: Verify that `ZERO_CAM_HOOD` is rigidly anchored to the front hood ($z_{offset} \approx +1.2\text{m}$, $y_{offset} \approx +0.85\text{m}$) and applies high-frequency micro-vibration proportional to RPM.
- **Verification Logic**:
  - Eye position follows car position rigidly: $\|pos_{cam} - (pos_{car} + R_{car} \vec{r}_{hood})\| < 0.05\text{m}$.
  - View matrix upper 3x3 has determinant $\det(R) = 1.000 \pm 0.001$.

#### `T1-CAM-03`: Cockpit Cam Driver Perspective & Steer Lookahead
- **Description**: Verify that `ZERO_CAM_COCKPIT` positions the camera at the driver's head ($x \approx -0.35\text{m}$, $y \approx +1.05\text{m}$, $z \approx +0.20\text{m}$) with dynamic yaw lookahead matching steering input.
- **Verification Logic**:
  - Steer input $\delta = +0.5$ (right turn): camera yaw rotates to the right relative to chassis heading by $\Delta \theta \approx 5.0^\circ - 15.0^\circ$.
  - View matrix satisfies orthonormality criteria.

#### `T1-CAM-04`: Bumper / TV Broadcast Cam Tracking & Orthonormality
- **Description**: Verify that `ZERO_CAM_BUMPER` and `ZERO_CAM_TV_BROADCAST` generate valid orthonormal view matrices and smooth tracking targets.
- **Verification Logic**:
  - Bumper Cam eye height $y \in [0.35\text{m}, 0.55\text{m}]$ skimming ground level.
  - TV Broadcast Cam selects nearest trackside tower and pans smoothly toward car position.
  - Both modes produce $\det(R) = 1.000 \pm 0.001$.

#### `T1-CAM-05`: Sequential Camera Mode Cycling
- **Description**: Verify that `zero_race_camera_cycle_mode()` steps through all 4 modes in sequence (`CHASE` $\to$ `HOOD` $\to$ `COCKPIT` $\to$ `BUMPER/TV` $\to$ `CHASE`).
- **Verification Logic**:
  - Initial mode: `ZERO_CAM_CHASE` (0).
  - 1st cycle: `ZERO_CAM_HOOD` (1).
  - 2nd cycle: `ZERO_CAM_COCKPIT` (2).
  - 3rd cycle: `ZERO_CAM_BUMPER` (3).
  - 4th cycle: wraps around to `ZERO_CAM_CHASE` (0).

#### `T1-CAM-06`: Velocity-Sensitive Dynamic FOV Dilation
- **Description**: Verify that Chase Cam FOV dilates smoothly as vehicle speed increases from standstill to top speed.
- **Verification Logic**:
  - At $v = 0\text{ km/h}$: $FOV = 62.0^\circ \pm 0.5^\circ$.
  - At $v = 100\text{ km/h}$: $FOV \approx 68.0^\circ \pm 1.0^\circ$.
  - At $v = 200\text{ km/h}$: $FOV = 78.0^\circ - 80.0^\circ$.
  - Dilation is monotonic and continuous.

---

### 4.5 Feature 5: UI State Machine Transitions (`F5`)

#### `T1-UI-01`: Main Menu Initialization & Button Layout
- **Description**: Verify that `zero_game_ui_init()` initializes the UI in `ZERO_STATE_MAIN_MENU` with interactive buttons for "Start Race", "Car Select", "Track Select", "Settings", and "Quit".
- **Verification Logic**:
  - Initial state: `gui->state == ZERO_STATE_MAIN_MENU`.
  - Button hitboxes are non-overlapping and within screen bounds $[0, W] \times [0, H]$.

#### `T1-UI-02`: Car Selection Flow & Parameter Mutator
- **Description**: Verify transition to `ZERO_STATE_CAR_SELECT`, car model index switching, and vehicle property updates (Apex GT, Phantom V8, Velocity RS).
- **Verification Logic**:
  - State transitions cleanly from Main Menu to Car Select.
  - Cycling car index mutates selected car configuration (mass, horsepower, handling).
  - Selected car index persists upon returning to Main Menu.

#### `T1-UI-03`: Track Selection Flow & Environmental Configuration
- **Description**: Verify transition to `ZERO_STATE_TRACK_SELECT`, track selection index, and diurnal lighting conditions (Midday, Golden Hour, Twilight).
- **Verification Logic**:
  - State transitions to Track Select.
  - Selected track index mutates time-of-day uniform parameters for analytical sky.

#### `T1-UI-04`: 3-2-1-GO Race Countdown Sequence
- **Description**: Verify that starting a race initiates `ZERO_STATE_COUNTDOWN` and transitions to `ZERO_STATE_RACING` after 3.0 seconds.
- **Verification Logic**:
  - $t \in [0.0\text{s}, 1.0\text{s})$: displays "3", audio triggers Red chime.
  - $t \in [1.0\text{s}, 2.0\text{s})$: displays "2", audio triggers Red chime.
  - $t \in [2.0\text{s}, 3.0\text{s})$: displays "1", audio triggers Red chime.
  - $t \ge 3.0\text{s}$: displays "GO!", triggers Green triad chime, transitions to `ZERO_STATE_RACING`.

#### `T1-UI-05`: In-Game Pause Overlay & Resume Toggle
- **Description**: Verify that pressing Escape or Pause toggles between `ZERO_STATE_RACING` and `ZERO_STATE_PAUSED` without losing race simulation state.
- **Verification Logic**:
  - Racing state transitions to `ZERO_STATE_PAUSED`.
  - Simulation timestep does not advance while paused.
  - Clicking "Resume" returns to `ZERO_STATE_RACING` with identical car positions, velocities, and lap times.

#### `T1-UI-06`: End-Race Podium & Leaderboard Sorting
- **Description**: Verify that when the race finishes, the game transitions to `ZERO_STATE_LEADERBOARD` displaying final rankings, lap times, and podium podium positions.
- **Verification Logic**:
  - Upon completing final lap: `state == ZERO_STATE_LEADERBOARD`.
  - Leaderboard table lists all 4 cars sorted by total race time in ascending order ($T_{1st} \le T_{2nd} \le T_{3rd} \le T_{4th}$).

---

### 4.6 Feature 6: Build Integrity & Freestanding Standard (`F6`)

#### `T1-BLD-01`: Clean Compilation Under Strict Flags
- **Description**: Verify that all engine headers and integration test files compile with zero warnings under `-std=c11 -O2 -Wall -Wextra -Werror`.
- **Verification Logic**:
  - Compiler invocation returns exit code 0.
  - Zero warning output lines emitted.

#### `T1-BLD-02`: Freestanding Link Resolution
- **Description**: Verify that executable links strictly with allowed libraries: `-lX11 -l:libGL.so.1 -lm -lpthread`.
- **Verification Logic**:
  - Binary inspection via `ldd` confirms no unexpected shared library dependencies (specifically zero `-lasound`, `-lSDL`, etc.).

#### `T1-BLD-03`: Static Library Archive Construction
- **Description**: Verify that `build/libzero_engine.a` exists, has non-zero size, and exports all required engine object symbols.
- **Verification Logic**:
  - `ar -t build/libzero_engine.a` contains all 17 core engine objects.

#### `T1-BLD-04`: Backward Compatibility Verification
- **Description**: Verify that all existing 430 engine unit tests in `tests/test_zero_engine.c` continue to pass 100%.
- **Verification Logic**:
  - `make test` execution produces `[ALL TESTS PASSED] 430 / 430 checks verified successfully.` with exit code 0.

#### `T1-BLD-05`: Zero Placeholders, TODOs, and Stubs Audit
- **Description**: Verify that the entire codebase contains zero `// TODO`, `// FIXME`, placeholder stubs, or unfinished mock return values.
- **Verification Logic**:
  - Automated ripgrep audit across `include/` and `src/` yields 0 matches for forbidden tokens.

#### `T1-BLD-06`: Headless Execution Safety & Determinism Over 100 Ticks
- **Description**: Verify that headless race simulation runs without `$DISPLAY` and generates bit-identical vehicle positions across two separate 100-tick runs.
- **Verification Logic**:
  - Run 1 and Run 2 executed from identical initial conditions produce identical vehicle positions: $\Delta pos = \vec{0}$.

---

## 5. Tier 2: Boundary & Corner Case Specifications ($\ge 5$ Tests Per Feature)

### 5.1 AI Navigation Boundaries (`F1`)
- **`T2-AI-01`: Spline Node Index Wrap-Around (Node 119 $\to$ 0)**: Traversal across the start/finish line where waypoint index wraps from 119 back to 0 must calculate correct forward tangent without angular inversion or distance spikes.
- **`T2-AI-02`: Zero-Velocity Launch from Rest**: When cars start from complete rest ($v = 0$), lookahead steering calculation must avoid division by zero ($v + 0.1\text{m/s}$) and produce stable positive throttle.
- **`T2-AI-03`: Redline Velocity Cornering Saturation**: Car driven at maximum speed ($> 240\text{ km/h}$) into sharp corner saturates tire cornering friction ($\sigma \ge 3.0$); verify car slides into barrier safely without NaN coordinates or infinite velocities.
- **`T2-AI-04`: Boundary Barrier Reflection Clamping**: Car positioned at $10.99\text{m}$ lateral distance (track barrier at $11.0\text{m}$) reflects impulse without penetration, tunneling, or getting wedged outside track geometry.
- **`T2-AI-05`: Abrupt $180^\circ$ Lookahead Vector Inversion**: Artificial target teleported $180^\circ$ behind the vehicle; AI steer angle rate-limiter prevents instantaneous angular snapping ($|d\delta/dt| \le 4.5\text{ rad/s}$).
- **`T2-AI-06`: High-Density Multi-Car Jamming**: All 4 vehicles compressed into a tight $3.0\text{m} \times 3.0\text{m}$ cluster; OBB contact resolution resolves overlaps within 8 iterations without numerical explosion.

### 5.2 HUD Telemetry Boundaries (`F2`)
- **`T2-HUD-01`: Zero and Negative Speed Display Clamping**: When vehicle is stationary ($v = 0$) or coasting backward in neutral, speedometer reading displays $0\text{ km/h}$ rather than negative or garbage numbers.
- **`T2-HUD-02`: Rev-Limiter Hard Cutoff & Redline Flash Activation**: Engine revved to extreme limit; RPM clamps at rev limiter ($8500\text{ RPM}$) without integer or floating-point overflow; redline flash duty cycle oscillates at $10\text{ Hz}$.
- **`T2-HUD-03`: Reverse Gear PRND Transition**: Engaging reverse gear (`ZERO_GEAR_REVERSE`) displays `R`; rearward throttle produces positive speedometer magnitude and correct reverse velocity integration.
- **`T2-HUD-04`: Long Race Lap Timer Sub-Millisecond Precision**: Timer evaluated after 1 hour ($3600\text{ s}$, 216,000 ticks); formatting correctly renders `59:59.999` without precision degradation.
- **`T2-HUD-05`: Microsecond Finish-Line Tie-Breaking**: Two vehicles cross the finish line within $0.001\text{ s}$ of each other; sorting algorithm deterministically ranks the leading nose without ranking oscillations.
- **`T2-HUD-06`: Degenerate Screen Dimensions**: Telemetry renderer called with $0\times 0$ resolution or extreme aspect ratio ($32:9$ or $9:32$) computes layout without division-by-zero crashes.

### 5.3 Audio Synthesizer Boundaries (`F3`)
- **`T2-AUD-01`: Idle RPM Acoustic Waveform Stability**: Minimum engine RPM ($800\text{ RPM}$, firing frequency $53.3\text{ Hz}$); synthesizer generates clean saw/pulse wave without DC offset accumulation ($|\mu_{DC}| < 0.01$).
- **`T2-AUD-02`: Maximum RPM Resonant Filter Stability**: Maximum RPM ($8500\text{ RPM}$, filter cutoff $3600\text{ Hz}$); 2-pole resonant filter state variables remain numerically stable without high-frequency limit cycle explosion.
- **`T2-AUD-03`: Extreme Tire Slip Angle & Full Lockup**: Slip ratio set to $1.0$ (handbrake lockup at $100\text{ km/h}$); tire squeal resonant bandpass noise stays strictly bounded within $[-32767, 32767]$ without overflow wrap.
- **`T2-AUD-04`: Zero Velocity Acoustic Fade**: At vehicle speed $0\text{ km/h}$, wind noise amplitude fades to exactly $0.0$; transition from $0 \to 1\text{ km/h}$ occurs without audible step clicks.
- **`T2-AUD-05`: High-Frequency Concurrent SFX Trigger Flooding**: Concurrently triggering 50 menu beeps, shift pops, and countdown chimes within 1 frame; master limiter hyperbolic tangent ($\tanh$) prevents clipping.
- **`T2-AUD-06`: Zero Buffer Length & Large Batch Buffer Synthesis**: Synthesizer called with `sample_count = 0` (returns immediately without error) and `sample_count = 65536` (processes large buffer cleanly).

### 5.4 Camera Modes Boundaries (`F4`)
- **`T2-CAM-01`: Degenerate Aspect Ratio Inversion**: Camera projection computed with aspect ratios $0.01$ and $100.0$; perspective matrix remains invertible with non-zero determinant.
- **`T2-CAM-02`: Frame-by-Frame Rapid Mode Churn**: Camera mode switched every single simulation tick (`CHASE` $\to$ `HOOD` $\to$ `COCKPIT` $\to$ `TV`); spring-damper state resets cleanly without infinite velocity accumulation.
- **`T2-CAM-03`: Rapid $180^\circ$ Vehicle Spin Inertial Damping**: Vehicle performs a rapid spin-out ($180^\circ$ yaw change in $0.2\text{ s}$); chase camera tracks smoothly without gimbal lock or flipping upside down.
- **`T2-CAM-04`: Extreme High-G Deceleration Inertia Clamping**: Vehicle collides at $200\text{ km/h}$ coming to an instant stop; camera spring inertia clamp prevents camera from penetrating vehicle cockpit.
- **`T2-CAM-05`: Near/Far Plane Clipping Depth Precision**: Extreme view frustum with $z_{near} = 0.05\text{m}$ and $z_{far} = 5000.0\text{m}$; depth buffer z-precision preserves depth ordering without z-fighting artifacts.

### 5.5 UI State Machine Boundaries (`F5`)
- **`T2-UI-01`: Rapid State Churn & Key Spamming**: Simulating rapid keypresses toggling Pause, Menu, and Racing states within 10 consecutive ticks; UI state machine remains valid without deadlocks.
- **`T2-UI-02`: Out-of-Bounds & Negative Mouse Coordinates**: Passing mouse coordinates $(-9999, -9999)$ or $(99999, 99999)$ does not trigger spurious button clicks or cause memory faults.
- **`T2-UI-03`: Zero Delta-Time Frame Update**: Updating UI with $\Delta t = 0.0\text{s}$ preserves all animation timers and countdown states without division by zero.
- **`T2-UI-04`: Large Lap Counter Stress**: Simulating a 500-lap race; lap counter text formatting handles multi-digit strings without buffer overflow.
- **`T2-UI-05`: Car & Track Selection Bounds Clamping**: Attempting to set car index to $-1$ or $+99$; UI clamps safely to $[0, \text{CAR\_COUNT}-1]$.

### 5.6 Build Integrity Boundaries (`F6`)
- **`T2-BLD-01`: Completely Unset DISPLAY Environment Variable**: Test runs in an isolated subshell with `DISPLAY=""` and `XAUTHORITY=""`; platform initialiser falls back to headless mode without attempting X11 connection.
- **`T2-BLD-02`: Optimization Flag Invariance**: Verification suite compiles and passes under both `-O0 -g` (debug) and `-O3 -flto` (maximum optimization) with identical assertion results.
- **`T2-BLD-03`: Strict C11 Feature Conformance**: Verification passes with `-std=c11 -pedantic`, confirming absence of non-standard GNU extensions in public headers.
- **`T2-BLD-04`: Arena Memory Exhaustion Handling**: Arena memory allocations exceeding capacity return `NULL` gracefully without memory corruption.
- **`T2-BLD-05`: Concurrency Safety & Mutex Teardown**: Audio engine thread initialization, rapid parameter update streaming, and shutdown tested across 100 cycles without deadlocks or resource leaks.

---

## 6. Tier 3: Cross-Feature Combinations (Pairwise Interaction Specifications)

| Test ID | Subsystem Pair | Cross-Feature Interaction Description | Success Criteria & Assertions |
|:---|:---|:---|:---|
| **`T3-INT-01`** | **Vehicle Drift + Camera Lag + Tire Audio** | Car enters high-speed power slide ($v = 140\text{ km/h}$, handbrake engaged, drift angle $32^\circ$); camera yaw angle lags chassis orientation while FOV dilates to $78^\circ$, and audio synthesizer activates tire squeal. | - Drift detected: `car.is_drifting == true`<br>- Camera yaw lags vehicle heading by $\Delta \theta \ge 8^\circ$<br>- Audio tire squeal RMS $> 500$<br>- View matrix $\det(R) = 1.000 \pm 0.001$ |
| **`T3-INT-02`** | **AI Overtake + HUD Ranking + Split Delta** | AI Car 2 detects leading Car 1, maneuvers laterally to overtake; as Car 2 passes Car 1, HUD race ranking updates from P2 to P1, and split-time delta inverts sign. | - AI lateral offset changes by $\ge 2.5\text{m}$<br>- Rankings update deterministically: Car 2 becomes Rank 1<br>- Split delta sign inverts from $+t \to -t$<br>- Zero collision interpenetration |
| **`T3-INT-03`** | **Rapid UI Navigation + Procedural Audio Beeps** | User navigates UI menus rapidly (Main Menu $\to$ Car Select $\to$ Track Select $\to$ Settings), generating rapid keyboard/mouse navigation events; audio triggers concurrent menu beeps. | - UI state transitions cleanly without lag<br>- Audio synthesizer queues and mixes beeps without digital clipping (samples $\in [-32767, 32767]$)<br>- Zero audio thread deadlocks |
| **`T3-INT-04`** | **Auto Transmission Shift + Engine Pop + HUD Tachometer** | Car reaches upshift RPM threshold ($7200\text{ RPM}$); transmission cuts torque ($40\%$), audio triggers exhaust backfire pop SFX, and HUD tachometer drops to $5000\text{ RPM}$. | - `transmission.is_shifting == true`<br>- `transmission.torque_cut_factor == 0.40`<br>- Shift pop audio SFX triggered<br>- Tachometer reading reflects RPM drop |
| **`T3-INT-05`** | **Multi-Car Collision + OBB Impulse + Camera Jolt** | Two vehicles collide laterally at $120\text{ km/h}$; OBB collision impulse resolves interpenetration, suspension absorbs impulse, camera spring-damper applies momentary shake. | - OBB collision detected and separated<br>- Normal restitution impulse $> 5000\text{ Ns}$<br>- Vehicles maintain upright stability ($\vec{u} \cdot \hat{y} > 0.60$)<br>- Camera recovers smoothly |
| **`T3-INT-06`** | **Race Countdown + Chime Audio + Drivetrain Launch** | UI sequence advances through COUNTDOWN ($3 \to 2 \to 1 \to \text{GO}$); audio plays $440\text{ Hz}$ chimes and $880+1100+1320\text{ Hz}$ triad; at GO, vehicles launch from rest. | - Audio chimes trigger on exact second marks<br>- Throttle input locked to $0$ until GO<br>- At GO, throttle engages, vehicle speed accelerates $> 0\text{ km/h}$<br>- UI transitions to `ZERO_STATE_RACING` |
| **`T3-INT-07`** | **Track Curvature + AI Corner Braking + HUD Radar** | Vehicle approaches sharp track curve near outer barrier; AI applies anticipatory braking; HUD 2D radar reflects car position and orientation along spline curvature. | - AI brake input activates before corner apex<br>- Car stays within $11.0\text{m}$ barrier<br>- HUD radar 2D coordinates map accurately to spline coordinates $(x, z)$ |
| **`T3-INT-08`** | **Pause Menu + Simulation Freeze + Audio Attenuation** | In-game pause triggered during high-speed racing; simulation physics frozen, audio synthesizer attenuates engine master volume by $-12\text{ dB}$, UI renders pause modal overlay. | - Vehicle position and velocity frozen: $\vec{x}_{t+1} = \vec{x}_t$<br>- Audio volume attenuated without buffer click/glitch<br>- UI buttons "Resume" and "Quit" respond to input |

---

## 7. Tier 4: Real-World Application Scenario Specifications

### `T4-SCN-01`: Full Multi-Car 3-Lap Championship Race Simulation
- **Features Exercised**: `F1`, `F2`, `F3`, `F4`, `F5`, `F6`.
- **Simulation Duration**: 1800 ticks (30.0 seconds of racing at $60\text{ Hz}$).
- **Scenario Description**:
  - Full 4-car grid (Player Car + 3 AI rivals) initializes in staggered formation.
  - Race countdown initiates ($3 \to 2 \to 1 \to \text{GO}$) with audio chimes.
  - All 4 vehicles navigate the 120-waypoint closed circuit spline, competing for position with pure pursuit steering and lateral overtaking.
  - HUD telemetry records lap times, positions, and split deltas in real time.
  - After completing designated laps, checkered flag triggers, winning car is determined, and UI transitions to podium leaderboard.
- **Pass/Fail Criteria**:
  - All 4 vehicles complete race simulation without flying off track or escaping $11.0\text{m}$ barrier corridor.
  - Vehicle upright stability maintained throughout: $\vec{u} \cdot \hat{y} > 0.60$ for all cars.
  - Checkered flag triggers deterministically; leaderboard lists 1st through 4th place sorted by total elapsed time.
  - Zero NaNs, Infs, or memory corruption.

### `T4-SCN-02`: Reverse Track Spline Lap & Inverted Tangent Traversal
- **Features Exercised**: `F1`, `F2`, `F4`.
- **Simulation Duration**: 1200 ticks (20.0 seconds).
- **Scenario Description**:
  - Simulation configured in reverse track direction: vehicles traverse spline nodes in decreasing order ($119 \to 118 \to \dots \to 0$).
  - AI pure pursuit lookahead evaluates negative tangents and inverted corner curvature.
- **Pass/Fail Criteria**:
  - AI racers successfully follow the inverted track path.
  - Vehicles maintain track boundary compliance ($d_{spline} \le 11.0\text{m}$).
  - Lap counter detects reverse start/finish line crossing correctly.

### `T4-SCN-03`: Sudden Obstacle & Spin-Out Evasive Maneuver
- **Features Exercised**: `F1`, `F3`, `F4`.
- **Simulation Duration**: 600 ticks (10.0 seconds).
- **Scenario Description**:
  - Lead Car 1 suffers a simulated spin-out and stalls broadside across the track at $z = 180\text{m}$.
  - Cars 2 and 3 approach at $140\text{ km/h}$.
  - Forward-cone detection sensors identify the stationary vehicle, calculate evasive steering offset, apply braking, and steer around the obstacle within track boundaries.
- **Pass/Fail Criteria**:
  - Following cars detect obstacle $\ge 25\text{m}$ prior to impact.
  - Following cars execute lateral avoidance maneuver and clear the obstacle without contact ($d_{min} \ge 2.2\text{m}$).
  - All cars remain within the $11.0\text{m}$ barrier corridor.

### `T4-SCN-04`: High-Speed Pack Collision & Elastic Rebound Recovery
- **Features Exercised**: `F1`, `F3`, `F4`.
- **Simulation Duration**: 600 ticks (10.0 seconds).
- **Scenario Description**:
  - Two cars are steered into an intentional oblique collision at $130\text{ km/h}$ entering a corner.
  - Pairwise OBB collision solver resolves interpenetration, calculates restitution impulses, and applies yaw moments.
- **Pass/Fail Criteria**:
  - Collision detected; OBB solver separates interpenetrating boxes within 4 iterations.
  - Neither car clips through track ground mesh or perimeter barriers.
  - Both vehicles absorb the collision impulse, maintain upright stability ($\vec{u} \cdot \hat{y} > 0.60$), and recover steering trajectory.

### `T4-SCN-05`: Dynamic Multi-Camera Switching Under Heavy Racing Load
- **Features Exercised**: `F1`, `F4`, `F6`.
- **Simulation Duration**: 1200 ticks (20.0 seconds).
- **Scenario Description**:
  - While all 4 cars navigate the circuit at top speed, the camera mode is dynamically cycled every 120 ticks: Chase $\to$ Hood $\to$ Cockpit $\to$ TV/Bumper $\to$ Chase.
  - 2nd-order spring-damper transitions and FOV dilation are continuously evaluated under racing acceleration, braking, and drifting.
- **Pass/Fail Criteria**:
  - Every camera mode produces an orthonormal view matrix: $\det(R) = 1.000 \pm 0.001$.
  - View projection matrices remain strictly invertible without numerical singularites.
  - Camera transitions exhibit continuous interpolation without 1-frame position pops.

### `T4-SCN-06`: Low-Friction Slick Track Surface Simulation
- **Features Exercised**: `F1`, `F3`, `F4`.
- **Simulation Duration**: 900 ticks (15.0 seconds).
- **Scenario Description**:
  - Track surface friction coefficient $\mu$ reduced by $50\%$ ($\mu = 0.45$).
  - AI racers must adapt to reduced adhesion limits, lower cornering speeds, and extended braking distances.
- **Pass/Fail Criteria**:
  - AI racers detect lower friction and reduce target cornering speeds accordingly.
  - Cars exhibit controlled power sliding/drifting around sharp corners without spinning out.
  - Tire squeal procedural audio reflects extended slip conditions.

---

## 8. Pass/Fail Gates, Invalidation Conditions & CI Integration Contract

### 8.1 Verification Gate Thresholds
To achieve certification in `TEST_READY.md`, the integration test suite must satisfy 100% of the following quantitative gates:
1. **Compilation Gate**: `gcc -std=c11 -O2 -Wall -Wextra -Werror` passes with 0 warnings and 0 errors.
2. **Assertion Gate**: 100% of integration checks in `test_race_integration.c` pass (minimum 100 checks).
3. **Regression Gate**: 100% of master engine tests in `test_zero_engine.c` pass (430/430 checks).
4. **Memory Gate**: Zero memory leaks or unbounded arena growth during extended simulation.
5. **Freestanding Link Gate**: Binary links strictly against `-lX11 -l:libGL.so.1 -lm -lpthread`.

### 8.2 Invalidation Conditions
Any of the following conditions immediately invalidates the test run and blocks release:
- Any car escaping the track corridor ($d_{spline} > 11.0\text{m}$).
- Any car overturning ($\vec{u}_{chassis} \cdot \hat{y} < 0.60$).
- Any NaN or Inf in vehicle position, velocity, audio buffer, or camera matrices.
- Any camera view matrix with upper 3x3 determinant deviating from $1.000$ by $> 0.001$.
- Any audio sample clipping outside the signed 16-bit range $[-32767, +32767]$.
- Any non-deterministic output across identical simulation seeds.

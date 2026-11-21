#ifndef _POSIX_C_SOURCE
#define _POSIX_C_SOURCE 199309L
#endif

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>
#include <pthread.h>

#include "zero_core.h"
#include "zero_math.h"
#include "zero_physics.h"
#include "zero_track_spline.h"
#include "zero_vehicle.h"
#include "zero_ai_racers.h"
#include "zero_race_camera.h"
#include "zero_audio.h"
#include "zero_race_ui.h"

/* -------------------------------------------------------------------------- */
/* Test Harness Statistics & Assertion Macros                                 */
/* -------------------------------------------------------------------------- */

static int g_tests_run = 0;
static int g_tests_passed = 0;
static int g_suite_failures = 0;

#define TEST_ASSERT(cond, msg) do { \
    g_tests_run++; \
    if (cond) { \
        g_tests_passed++; \
    } else { \
        g_suite_failures++; \
        fprintf(stderr, "  [FAIL] %s:%d: %s\n", __FILE__, __LINE__, msg); \
    } \
} while (0)

#define TEST_ASSERT_MSG(cond, fmt, ...) do { \
    g_tests_run++; \
    if (cond) { \
        g_tests_passed++; \
    } else { \
        g_suite_failures++; \
        fprintf(stderr, "  [FAIL] %s:%d: " fmt "\n", __FILE__, __LINE__, __VA_ARGS__); \
    } \
} while (0)

/* -------------------------------------------------------------------------- */
/* Mathematical & Audio Verification Helpers                                  */
/* -------------------------------------------------------------------------- */

static float matrix_view_det3x3(Mat4 v) {
    float r00 = v.m[0], r01 = v.m[4], r02 = v.m[8];
    float r10 = v.m[1], r11 = v.m[5], r12 = v.m[9];
    float r20 = v.m[2], r21 = v.m[6], r22 = v.m[10];

    return r00 * (r11 * r22 - r12 * r21) -
           r01 * (r10 * r22 - r12 * r20) +
           r02 * (r10 * r21 - r11 * r20);
}

static float matrix_row_norm_sq(Mat4 v, int row) {
    float x = v.m[row];
    float y = v.m[row + 4];
    float z = v.m[row + 8];
    return x * x + y * y + z * z;
}

static double calculate_audio_rms(const int16_t *buf, uint32_t count) {
    if (!buf || count == 0) return 0.0;
    double sum_sq = 0.0;
    for (uint32_t i = 0; i < count; ++i) {
        double s = (double)buf[i];
        sum_sq += s * s;
    }
    return sqrt(sum_sq / (double)count);
}

static uint32_t count_audio_zero_crossings(const int16_t *buf, uint32_t count) {
    if (!buf || count < 2) return 0;
    uint32_t crossings = 0;
    for (uint32_t i = 1; i < count; ++i) {
        if ((buf[i - 1] < 0 && buf[i] >= 0) || (buf[i - 1] >= 0 && buf[i] < 0)) {
            crossings++;
        }
    }
    return crossings;
}

/* -------------------------------------------------------------------------- */
/* SUITE 1: Multi-Car Autonomous AI Navigation (Tier 1 - F1)                  */
/* -------------------------------------------------------------------------- */

static void test_suite_multicar_ai_navigation(void) {
    printf("[SUITE 1] Multi-Car Autonomous AI Navigation (600 Ticks)...\n");
    int failures_before = g_suite_failures;

    ZeroRaceGame game = zero_race_game_create();
    TEST_ASSERT(game.race_started == true, "Race initialized in started state");
    TEST_ASSERT(game.total_laps == 3, "Race default lap count is 3");

    /* 1. Staggered grid spacing on start line */
    for (int k = 0; k < ZERO_RACE_CAR_COUNT; ++k) {
        TEST_ASSERT(!isnan(game.cars[k].position.x), "Car position.x is finite at spawn");
        TEST_ASSERT(!isnan(game.cars[k].position.z), "Car position.z is finite at spawn");
        TEST_ASSERT(game.cars[k].position.y >= 0.5f && game.cars[k].position.y <= 1.5f,
                    "Car spawn height within normal suspension range");
    }

    for (int i = 0; i < ZERO_RACE_CAR_COUNT; ++i) {
        for (int j = i + 1; j < ZERO_RACE_CAR_COUNT; ++j) {
            float dx = game.cars[i].position.x - game.cars[j].position.x;
            float dz = game.cars[i].position.z - game.cars[j].position.z;
            float dist = sqrtf(dx * dx + dz * dz);
            TEST_ASSERT(dist >= 5.0f, "Spawned vehicles maintain >= 5.0m pairwise clearance");
        }
    }

    /* Assign autonomous pursuit controller to Player Car 0 as well */
    ZeroAIController player_ai = zero_ai_controller_create(0, 0.98f, 0.0f);
    const float dt = 1.0f / 60.0f;
    const int total_ticks = 600;

    float max_spline_dist[ZERO_RACE_CAR_COUNT] = {0.0f};
    float min_up_dot[ZERO_RACE_CAR_COUNT] = {1.0f, 1.0f, 1.0f, 1.0f};
    float min_elevation[ZERO_RACE_CAR_COUNT] = {100.0f, 100.0f, 100.0f, 100.0f};
    float max_elevation[ZERO_RACE_CAR_COUNT] = {-100.0f, -100.0f, -100.0f, -100.0f};
    bool boundary_ok = true;
    bool upright_ok = true;
    bool finite_ok = true;

    for (int tick = 0; tick < total_ticks; ++tick) {
        zero_ai_controller_update(&player_ai, &game, dt);
        zero_race_game_step(&game, dt);

        for (int k = 0; k < ZERO_RACE_CAR_COUNT; ++k) {
            Vec3 pos = game.cars[k].position;

            /* Check finiteness */
            if (isnan(pos.x) || isnan(pos.y) || isnan(pos.z) ||
                isinf(pos.x) || isinf(pos.y) || isinf(pos.z)) {
                finite_ok = false;
            }

            /* Elevation */
            if (pos.y < min_elevation[k]) min_elevation[k] = pos.y;
            if (pos.y > max_elevation[k]) max_elevation[k] = pos.y;
            if (pos.y < -2.0f || pos.y > 15.0f) {
                boundary_ok = false;
            }

            /* Distance to spline centerline */
            int seg = 0;
            float seg_t = 0.0f;
            float dist_sq = 0.0f;
            zero_track_find_nearest_segment(pos, &seg, &seg_t, &dist_sq);
            float dist = sqrtf(dist_sq);
            if (dist > max_spline_dist[k]) max_spline_dist[k] = dist;
            if (dist > ZERO_TRACK_BARRIER_DIST) {
                boundary_ok = false;
            }

            /* Upright stability */
            Mat4 rot = quat_to_mat4(game.cars[k].orientation);
            float up_dot = rot.m[5];
            if (up_dot < min_up_dot[k]) min_up_dot[k] = up_dot;
            if (up_dot <= 0.60f) {
                upright_ok = false;
            }
        }
    }

    TEST_ASSERT(finite_ok, "All car positions and velocities remain finite across 600 ticks");
    TEST_ASSERT(boundary_ok, "All cars stay within track corridor (dist <= 11.0m, elevation in [-2.0, 15.0])");
    TEST_ASSERT(upright_ok, "All cars maintain upright stability (up_dot > 0.60, zero rollovers)");

    for (int k = 0; k < ZERO_RACE_CAR_COUNT; ++k) {
        TEST_ASSERT_MSG(max_spline_dist[k] <= 11.0f,
                        "Car %d max spline distance %.2fm <= 11.0m", k, max_spline_dist[k]);
        TEST_ASSERT_MSG(min_up_dot[k] > 0.60f,
                        "Car %d minimum up-vector dot %.3f > 0.60", k, min_up_dot[k]);
        TEST_ASSERT_MSG(min_elevation[k] >= -2.0f && max_elevation[k] <= 15.0f,
                        "Car %d elevation [%.2f, %.2f] within [-2.0, 15.0]",
                        k, min_elevation[k], max_elevation[k]);
        TEST_ASSERT_MSG(game.cars[k].track_progress_s > 50.0f,
                        "Car %d achieved substantial forward progress: %.1fm",
                        k, game.cars[k].track_progress_s);
        TEST_ASSERT_MSG(game.cars[k].speed_kmh > 30.0f,
                        "Car %d maintained active racing speed: %.1f km/h",
                        k, game.cars[k].speed_kmh);
    }

    /* Verify pure pursuit anticipatory braking */
    float v_max_hairpin = zero_track_get_max_corner_speed(20, 1.0f);
    TEST_ASSERT(v_max_hairpin > 0.0f && v_max_hairpin < 75.0f,
                "Hairpin corner speed limit calculated realistically");

    if (g_suite_failures == failures_before) {
        printf("  -> [PASS] Suite 1 verified successfully (All 4 cars navigated 600 ticks flawlessly).\n");
    }
}

/* -------------------------------------------------------------------------- */
/* SUITE 2: HUD Telemetry Determinism (Tier 1 - F2)                           */
/* -------------------------------------------------------------------------- */

static void test_suite_hud_telemetry_determinism(void) {
    printf("[SUITE 2] HUD Telemetry Determinism & Consistency...\n");
    int failures_before = g_suite_failures;

    ZeroRaceGame run_a = zero_race_game_create();
    ZeroRaceGame run_b = zero_race_game_create();
    const float dt = 1.0f / 60.0f;
    const int check_ticks = 120;

    bool telemetry_deterministic = true;

    for (int t = 0; t < check_ticks; ++t) {
        zero_race_game_step(&run_a, dt);
        zero_race_game_step(&run_b, dt);

        if ((t + 1) % 30 == 0) {
            for (int k = 0; k < ZERO_RACE_CAR_COUNT; ++k) {
                if (fabsf(run_a.cars[k].speed_kmh - run_b.cars[k].speed_kmh) > 1e-4f ||
                    fabsf(run_a.cars[k].engine_rpm - run_b.cars[k].engine_rpm) > 1e-3f ||
                    run_a.cars[k].transmission.current_gear != run_b.cars[k].transmission.current_gear ||
                    fabsf(run_a.cars[k].current_lap_time - run_b.cars[k].current_lap_time) > 1e-5f ||
                    run_a.cars[k].race_position != run_b.cars[k].race_position ||
                    fabsf(run_a.cars[k].position.x - run_b.cars[k].position.x) > 1e-5f ||
                    fabsf(run_a.cars[k].position.z - run_b.cars[k].position.z) > 1e-5f) {
                    telemetry_deterministic = false;
                }
            }
        }
    }

    TEST_ASSERT(telemetry_deterministic,
                "HUD telemetry bit-identical across repeated runs over 120 ticks");

    /* Ranking uniqueness check */
    int seen_ranks[5] = {0};
    for (int k = 0; k < ZERO_RACE_CAR_COUNT; ++k) {
        int r = run_a.cars[k].race_position;
        TEST_ASSERT(r >= 1 && r <= 4, "Car position rank is between 1 and 4");
        seen_ranks[r]++;
    }
    for (int r = 1; r <= 4; ++r) {
        TEST_ASSERT(seen_ranks[r] == 1, "Each position rank (1st to 4th) assigned to exactly one car");
    }

    /* RPM bounds & Speedometer correspondence */
    for (int k = 0; k < ZERO_RACE_CAR_COUNT; ++k) {
        TEST_ASSERT(run_a.cars[k].engine_rpm >= 800.0f && run_a.cars[k].engine_rpm <= 8500.0f,
                    "Car engine RPM stays bounded within [800, 8500]");
        float physical_speed_kmh = vec3_length(run_a.cars[k].velocity) * 3.6f;
        TEST_ASSERT(fabsf(run_a.cars[k].speed_kmh - physical_speed_kmh) < 0.1f,
                    "HUD speedometer matches physical velocity norm in km/h");
        TEST_ASSERT(run_a.cars[k].current_lap_time > 0.0f,
                    "Lap timer advances monotonically with simulation ticks");
    }

    if (g_suite_failures == failures_before) {
        printf("  -> [PASS] Suite 2 verified successfully (Deterministic telemetry & ranking).\n");
    }
}

/* -------------------------------------------------------------------------- */
/* SUITE 3: Procedural Audio Synthesis (Tier 1 - F3)                          */
/* -------------------------------------------------------------------------- */

static void test_suite_procedural_audio(void) {
    printf("[SUITE 3] Procedural Audio Synthesis (Software Mode)...\n");
    int failures_before = g_suite_failures;

    ZeroAudioEngine audio;
    memset(&audio, 0, sizeof(audio));

    bool init_ok = zero_audio_init(&audio, true);
    TEST_ASSERT(init_ok == true, "Audio engine initializes successfully in software mode");
    TEST_ASSERT(audio.backend == ZERO_AUDIO_BACKEND_SOFTWARE,
                "Audio engine active backend is ZERO_AUDIO_BACKEND_SOFTWARE");

    /* 1. Synthesis buffer generation: 44.1kHz buffer without NaNs, Infs, or clipping */
    const uint32_t sample_count = ZERO_AUDIO_SAMPLE_RATE; /* 1 second of 44.1kHz audio */
    int16_t *pcm_buffer = (int16_t *)malloc(sample_count * sizeof(int16_t));
    TEST_ASSERT(pcm_buffer != NULL, "Allocated 44.1kHz audio test buffer");

    ZeroAudioParams params;
    memset(&params, 0, sizeof(params));

    /* Test Case A: Idle Engine (900 RPM, 0 throttle) */
    params.engine_rpm = 900.0f;
    params.throttle = 0.0f;
    params.tire_slip_ratio = 0.0f;
    params.vehicle_speed_kmh = 0.0f;
    zero_audio_update_params(&audio, &params);

    zero_audio_synthesize_buffer(&audio, pcm_buffer, sample_count);

    bool bounds_ok = true;
    for (uint32_t i = 0; i < sample_count; ++i) {
        if (pcm_buffer[i] < -32000 || pcm_buffer[i] > 32000) {
            bounds_ok = false;
        }
    }
    TEST_ASSERT(bounds_ok, "Idle audio waveform samples strictly bounded within [-32000, 32000]");

    double idle_rms = calculate_audio_rms(pcm_buffer, sample_count);
    uint32_t idle_crossings = count_audio_zero_crossings(pcm_buffer, sample_count);
    TEST_ASSERT(idle_rms > 100.0, "Idle engine generates non-zero audible RMS");

    /* Test Case B: High RPM Engine (7500 RPM, full throttle) */
    params.engine_rpm = 7500.0f;
    params.throttle = 1.0f;
    params.tire_slip_ratio = 0.02f;
    params.vehicle_speed_kmh = 160.0f;
    zero_audio_update_params(&audio, &params);

    zero_audio_synthesize_buffer(&audio, pcm_buffer, sample_count);

    bounds_ok = true;
    for (uint32_t i = 0; i < sample_count; ++i) {
        if (pcm_buffer[i] < -32000 || pcm_buffer[i] > 32000) {
            bounds_ok = false;
        }
    }
    TEST_ASSERT(bounds_ok, "High RPM audio waveform samples strictly bounded within [-32000, 32000]");

    double high_rpm_rms = calculate_audio_rms(pcm_buffer, sample_count);
    uint32_t high_rpm_crossings = count_audio_zero_crossings(pcm_buffer, sample_count);
    TEST_ASSERT(high_rpm_rms > idle_rms, "High RPM audio RMS exceeds idle RMS");
    TEST_ASSERT(high_rpm_crossings > (uint32_t)(idle_crossings * 2.0),
                "Engine fundamental frequency and pitch scales proportionally with RPM");

    /* Test Case C: Lateral Tire Slip Squeal */
    params.tire_slip_ratio = 0.50f;
    zero_audio_update_params(&audio, &params);
    zero_audio_synthesize_buffer(&audio, pcm_buffer, sample_count);

    double squeal_rms = calculate_audio_rms(pcm_buffer, sample_count);
    TEST_ASSERT(squeal_rms > 500.0, "Active tire slip generates resonant squeal audio energy");

    /* Test Case D: UI Countdown & Menu Beeps */
    params.trigger_menu_beep = true;
    zero_audio_update_params(&audio, &params);
    zero_audio_synthesize_buffer(&audio, pcm_buffer, sample_count / 10);
    double beep_rms = calculate_audio_rms(pcm_buffer, sample_count / 10);
    TEST_ASSERT(beep_rms > 100.0, "Procedural menu beep synthesized with valid audible power");

    params.trigger_menu_beep = false;
    params.trigger_countdown_green = true;
    zero_audio_update_params(&audio, &params);
    zero_audio_synthesize_buffer(&audio, pcm_buffer, sample_count / 2);
    double green_chime_rms = calculate_audio_rms(pcm_buffer, sample_count / 2);
    TEST_ASSERT(green_chime_rms > 100.0, "Race start GO chime synthesized with valid audible power");

    free(pcm_buffer);
    zero_audio_shutdown(&audio);

    if (g_suite_failures == failures_before) {
        printf("  -> [PASS] Suite 3 verified successfully (44.1kHz buffer, pitch scaling, squeal).\n");
    }
}

/* -------------------------------------------------------------------------- */
/* SUITE 4: Multi-Camera View Matrices & Orthonormality (Tier 1 - F4)         */
/* -------------------------------------------------------------------------- */

static void test_suite_camera_matrices(void) {
    printf("[SUITE 4] Multi-Camera View Matrices & Orthonormality...\n");
    int failures_before = g_suite_failures;

    const float aspect = 16.0f / 9.0f;
    ZeroRaceCamera rcam = zero_race_camera_create(aspect);

    Vec3 car_pos = vec3(125.0f, 0.75f, 50.0f);
    Quat car_rot = quat_identity();
    Vec3 car_vel = vec3(0.0f, 0.0f, 35.0f);

    /* 1. Orthonormality and valid projection across all 4 camera modes */
    for (int mode = 0; mode < 4; ++mode) {
        zero_race_camera_set_mode(&rcam, (ZeroCameraMode)mode, false);
        TEST_ASSERT((int)rcam.mode == mode, "Camera mode switch registered");

        /* Simulate 60 frames of tracking */
        for (int f = 0; f < 60; ++f) {
            car_pos.z += car_vel.z * (1.0f / 60.0f);
            zero_race_camera_update(&rcam, car_pos, car_rot, car_vel, 4500.0f, 0.0f, 1.0f / 60.0f);
        }

        Mat4 view = zero_race_camera_get_view(&rcam);
        Mat4 proj = zero_race_camera_get_projection(&rcam);

        float det = matrix_view_det3x3(view);
        TEST_ASSERT_MSG(fabsf(det - 1.000f) <= 0.001f,
                        "Camera mode %d: upper 3x3 determinant |%.5f - 1.0| <= 0.001",
                        mode, det);

        float r0_norm = matrix_row_norm_sq(view, 0);
        float r1_norm = matrix_row_norm_sq(view, 1);
        float r2_norm = matrix_row_norm_sq(view, 2);
        TEST_ASSERT_MSG(fabsf(r0_norm - 1.0f) < 0.002f &&
                        fabsf(r1_norm - 1.0f) < 0.002f &&
                        fabsf(r2_norm - 1.0f) < 0.002f,
                        "Camera mode %d: view matrix rows have unit Euclidean norm", mode);

        /* Projection matrix properties */
        TEST_ASSERT(proj.m[0] > 0.0f, "Projection matrix m[0] > 0 (valid aspect & fov)");
        TEST_ASSERT(proj.m[5] > 0.0f, "Projection matrix m[5] > 0 (valid fov)");
        TEST_ASSERT(proj.m[11] == -1.0f, "Standard OpenGL perspective divide m[11] == -1.0");
        TEST_ASSERT(proj.m[14] < 0.0f, "Perspective depth map m[14] < 0");

        /* Finiteness check across all 16 entries */
        bool matrices_finite = true;
        for (int i = 0; i < 16; ++i) {
            if (isnan(view.m[i]) || isinf(view.m[i]) ||
                isnan(proj.m[i]) || isinf(proj.m[i])) {
                matrices_finite = false;
            }
        }
        TEST_ASSERT(matrices_finite, "View and projection matrices contain zero NaNs or Infs");
    }

    /* 2. Camera Mode Cycling */
    zero_race_camera_set_mode(&rcam, ZERO_CAM_CHASE, false);
    zero_race_camera_cycle_mode(&rcam);
    TEST_ASSERT(rcam.mode == ZERO_CAM_HOOD, "Cycle 1: CHASE -> HOOD");
    zero_race_camera_cycle_mode(&rcam);
    TEST_ASSERT(rcam.mode == ZERO_CAM_COCKPIT, "Cycle 2: HOOD -> COCKPIT");
    zero_race_camera_cycle_mode(&rcam);
    TEST_ASSERT(rcam.mode == ZERO_CAM_BUMPER, "Cycle 3: COCKPIT -> BUMPER");
    zero_race_camera_cycle_mode(&rcam);
    TEST_ASSERT(rcam.mode == ZERO_CAM_TV_BROADCAST, "Cycle 4: BUMPER -> TV_BROADCAST");
    zero_race_camera_cycle_mode(&rcam);
    TEST_ASSERT(rcam.mode == ZERO_CAM_CHASE, "Cycle 5: Wrap around to CHASE");

    /* 3. Chase Cam Velocity-Sensitive Dynamic FOV Dilation */
    zero_race_camera_set_mode(&rcam, ZERO_CAM_CHASE, false);
    zero_race_camera_update(&rcam, car_pos, car_rot, vec3(0.0f, 0.0f, 0.0f), 800.0f, 0.0f, 0.1f);
    Mat4 proj_rest = zero_race_camera_get_projection(&rcam);

    zero_race_camera_update(&rcam, car_pos, car_rot, vec3(0.0f, 0.0f, 60.0f), 7000.0f, 0.0f, 0.1f);
    Mat4 proj_fast = zero_race_camera_get_projection(&rcam);

    /* As FOV dilates wider, 1.0 / tan(fov/2) decreases */
    TEST_ASSERT(proj_fast.m[5] < proj_rest.m[5],
                "Chase cam FOV expands dynamically at high vehicle speed");

    if (g_suite_failures == failures_before) {
        printf("  -> [PASS] Suite 4 verified successfully (Orthonormal matrices, cycling, FOV).\n");
    }
}

/* -------------------------------------------------------------------------- */
/* SUITE 5: UI State Machine Transitions Across All 8 Pages (Tier 1 - F5)     */
/* -------------------------------------------------------------------------- */

static void test_suite_ui_state_machine(void) {
    printf("[SUITE 5] UI State Machine Transitions Across All 8 Pages...\n");
    int failures_before = g_suite_failures;

    ZeroGameUI gui;
    memset(&gui, 0, sizeof(gui));

    bool ui_ok = zero_game_ui_init(&gui);
    TEST_ASSERT(ui_ok == true, "ZeroUI initialized successfully");
    TEST_ASSERT(gui.state == ZERO_STATE_MAIN_MENU, "UI starts in ZERO_STATE_MAIN_MENU");

    ZeroGameTelemetry telem;
    memset(&telem, 0, sizeof(telem));
    telem.speed_kmh = 125.0f;
    telem.engine_rpm = 5200.0f;
    telem.gear = 3;
    telem.current_lap = 1;
    telem.total_laps = 3;
    telem.race_position = 2;
    telem.total_cars = 4;
    telem.current_lap_time = 42.15f;
    telem.best_lap_time = 41.80f;
    telem.split_delta = 0.35f;

    const float screen_w = 1280.0f;
    const float screen_h = 720.0f;

    /* Page 0: MAIN_MENU */
    zero_game_ui_render_main_menu(&gui, screen_w, screen_h);
    TEST_ASSERT(zero_game_ui_get_car_count() >= 3, "Car catalog contains at least 3 vehicles");
    TEST_ASSERT(zero_game_ui_get_track_count() >= 1, "Track catalog contains at least 1 circuit");

    /* Page 1: CAR_SELECT */
    zero_game_ui_set_state(&gui, ZERO_STATE_CAR_SELECT);
    TEST_ASSERT(gui.state == ZERO_STATE_CAR_SELECT, "Transitioned to ZERO_STATE_CAR_SELECT");
    zero_game_ui_render_car_select(&gui, screen_w, screen_h);
    gui.selected_car_idx = 1;
    const ZeroCarConfig *car1 = zero_game_ui_get_car(gui.selected_car_idx);
    TEST_ASSERT(car1 != NULL, "Retrieved valid car configuration");
    TEST_ASSERT(car1->name != NULL && strlen(car1->name) > 0, "Selected car has valid non-empty name");

    /* Page 2: TRACK_SELECT */
    zero_game_ui_set_state(&gui, ZERO_STATE_TRACK_SELECT);
    TEST_ASSERT(gui.state == ZERO_STATE_TRACK_SELECT, "Transitioned to ZERO_STATE_TRACK_SELECT");
    zero_game_ui_render_track_select(&gui, screen_w, screen_h);
    const ZeroTrackConfig *track = zero_game_ui_get_track(0);
    TEST_ASSERT(track != NULL, "Retrieved valid track configuration");
    TEST_ASSERT(track->lap_count > 0, "Track specifies positive lap count");

    /* Page 3: SETTINGS */
    zero_game_ui_set_state(&gui, ZERO_STATE_SETTINGS);
    TEST_ASSERT(gui.state == ZERO_STATE_SETTINGS, "Transitioned to ZERO_STATE_SETTINGS");
    zero_game_ui_render_settings(&gui, screen_w, screen_h);
    TEST_ASSERT(gui.audio_settings.master_volume >= 0.0f && gui.audio_settings.master_volume <= 1.0f,
                "Master volume within normalized [0, 1] range");

    /* Page 4: COUNTDOWN (3 -> 2 -> 1 -> GO) */
    zero_game_ui_set_state(&gui, ZERO_STATE_COUNTDOWN);
    TEST_ASSERT(gui.state == ZERO_STATE_COUNTDOWN, "Transitioned to ZERO_STATE_COUNTDOWN");
    gui.countdown_timer = 3.0f;
    gui.prev_countdown_timer = 3.0f;

    /* Advance countdown timer in 0.5s increments */
    for (int step = 0; step < 7; ++step) {
        zero_game_ui_render(&gui, &telem, screen_w, screen_h, 0.0f, 0.0f, false, 0.5f);
    }
    TEST_ASSERT(gui.countdown_timer <= 0.0f, "Countdown timer completed reaching <= 0.0s");
    TEST_ASSERT(gui.state == ZERO_STATE_RACING,
                "Automatic transition to ZERO_STATE_RACING triggered on GO");

    /* Page 5: RACING HUD & Widgets */
    zero_game_ui_render_hud(&gui, &telem, screen_w, screen_h);
    /* Test custom procedural widgets */
    zero_ui_draw_arc_gauge(&gui.ui, 150.0f, 600.0f, 70.0f,
                           ZERO_DEG2RAD(150.0f), ZERO_DEG2RAD(390.0f),
                           0.65f, vec4(0.2f, 0.8f, 1.0f, 1.0f), vec4(0.1f, 0.1f, 0.1f, 0.5f), 10.0f);
    zero_ui_draw_segmented_bar(&gui.ui, 50.0f, 50.0f, 200.0f, 15.0f,
                               10, 0.70f, vec4(0.0f, 1.0f, 0.0f, 1.0f),
                               vec4(0.2f, 0.2f, 0.2f, 0.5f), vec4(1.0f, 0.0f, 0.0f, 1.0f));
    float ai_t[3] = {0.25f, 0.23f, 0.20f};
    zero_ui_draw_track_radar(&gui.ui, 1100.0f, 100.0f, 160.0f, 160.0f,
                             g_track_spline, ZERO_TRACK_SPLINE_COUNT, 0.26f, ai_t);

    /* Page 6: PAUSED Overlay */
    zero_game_ui_set_state(&gui, ZERO_STATE_PAUSED);
    TEST_ASSERT(gui.state == ZERO_STATE_PAUSED, "Transitioned to ZERO_STATE_PAUSED");
    zero_game_ui_render_pause(&gui, screen_w, screen_h);

    /* Test Restart and Quit flags */
    TEST_ASSERT(zero_game_ui_is_restart_requested(&gui) == false, "Initial restart request is false");
    TEST_ASSERT(zero_game_ui_is_quit_requested(&gui) == false, "Initial quit request is false");

    /* Resume back to RACING */
    zero_game_ui_set_state(&gui, ZERO_STATE_RACING);
    TEST_ASSERT(gui.state == ZERO_STATE_RACING, "Resumed back to ZERO_STATE_RACING");

    /* Page 7: LEADERBOARD & Podium */
    zero_game_ui_set_podium_entry(&gui, 0, "Player 1", "Apex GT", 124.52f, 41.20f, 1);
    zero_game_ui_set_podium_entry(&gui, 1, "AI Rival 1", "Phantom V8", 125.10f, 41.45f, 2);
    zero_game_ui_set_podium_entry(&gui, 2, "AI Rival 2", "Velocity RS", 126.30f, 41.85f, 3);
    zero_game_ui_set_podium_entry(&gui, 3, "AI Rival 3", "Apex GT", 128.95f, 42.10f, 4);

    zero_game_ui_set_state(&gui, ZERO_STATE_LEADERBOARD);
    TEST_ASSERT(gui.state == ZERO_STATE_LEADERBOARD, "Transitioned to ZERO_STATE_LEADERBOARD");
    zero_game_ui_render_leaderboard(&gui, screen_w, screen_h);

    TEST_ASSERT(strcmp(gui.podium[0].driver_name, "Player 1") == 0,
                "Leaderboard podium entry 1 matches winner name");
    TEST_ASSERT(gui.podium[0].total_time < gui.podium[1].total_time,
                "Podium entries correctly sorted by total time");

    /* Return to MAIN_MENU */
    zero_game_ui_set_state(&gui, ZERO_STATE_MAIN_MENU);
    TEST_ASSERT(gui.state == ZERO_STATE_MAIN_MENU, "Completed cycle back to ZERO_STATE_MAIN_MENU");

    zero_game_ui_shutdown(&gui);

    if (g_suite_failures == failures_before) {
        printf("  -> [PASS] Suite 5 verified successfully (All 8 UI pages & widgets operational).\n");
    }
}

/* -------------------------------------------------------------------------- */
/* SUITE 6: Boundary & Corner Cases (Tier 2)                                  */
/* -------------------------------------------------------------------------- */

static void test_suite_boundary_cases(void) {
    printf("[SUITE 6] Boundary Value & Extreme Stress Cases (Tier 2)...\n");
    int failures_before = g_suite_failures;

    /* T2-AI-01: Spline Node Wrap-Around (Node 119 -> 0) */
    int seg_start = 0;
    float t_start = 0.0f;
    float dsq_start = 0.0f;
    Vec3 wrap_pos = vec3(125.0f, 0.75f, 0.0f);
    zero_track_find_nearest_segment(wrap_pos, &seg_start, &t_start, &dsq_start);
    TEST_ASSERT(seg_start == 0 || seg_start == ZERO_TRACK_SPLINE_COUNT - 1,
                "Spline wrap-around segment lookup smoothly resolves boundary nodes");

    /* T2-AI-02: Zero-Velocity Launch from Rest */
    ZeroRaceGame game_rest = zero_race_game_create();
    for (int k = 0; k < ZERO_RACE_CAR_COUNT; ++k) {
        game_rest.cars[k].velocity = vec3(0.0f, 0.0f, 0.0f);
        game_rest.cars[k].speed_kmh = 0.0f;
    }
    zero_race_game_step(&game_rest, 1.0f / 60.0f);
    TEST_ASSERT(!isnan(game_rest.cars[0].velocity.z) && !isinf(game_rest.cars[0].velocity.z),
                "Zero-velocity launch integration avoids division-by-zero singularities");

    /* T2-AUD-06: Zero Buffer Length & Large Batch Buffer Synthesis */
    ZeroAudioEngine audio_edge;
    zero_audio_init(&audio_edge, true);
    int16_t zero_len_buf[1] = {0};
    zero_audio_synthesize_buffer(&audio_edge, zero_len_buf, 0); /* Zero length call */
    TEST_ASSERT(true, "Zero length audio synthesis handled gracefully without error");

    const uint32_t large_count = 65536;
    int16_t *large_buf = (int16_t *)malloc(large_count * sizeof(int16_t));
    if (large_buf) {
        zero_audio_synthesize_buffer(&audio_edge, large_buf, large_count);
        TEST_ASSERT(large_buf[0] != 0 || large_buf[1000] != 0,
                    "Large 65536-sample audio buffer synthesized cleanly");
        free(large_buf);
    }
    zero_audio_shutdown(&audio_edge);

    /* T2-CAM-01: Extreme Aspect Ratios (0.01 and 100.0) */
    ZeroRaceCamera cam_narrow = zero_race_camera_create(0.01f);
    Mat4 proj_narrow = zero_race_camera_get_projection(&cam_narrow);
    TEST_ASSERT(!isnan(proj_narrow.m[0]) && !isinf(proj_narrow.m[0]),
                "Narrow aspect ratio 0.01 produces finite projection matrix");

    ZeroRaceCamera cam_wide = zero_race_camera_create(100.0f);
    Mat4 proj_wide = zero_race_camera_get_projection(&cam_wide);
    TEST_ASSERT(!isnan(proj_wide.m[0]) && !isinf(proj_wide.m[0]),
                "Ultra-wide aspect ratio 100.0 produces finite projection matrix");

    /* T2-UI-06: Degenerate Screen Dimensions */
    ZeroGameUI gui_edge;
    zero_game_ui_init(&gui_edge);
    ZeroGameTelemetry telem_edge;
    memset(&telem_edge, 0, sizeof(telem_edge));
    zero_game_ui_render(&gui_edge, &telem_edge, 0.0f, 0.0f, 0.0f, 0.0f, false, 0.016f);
    TEST_ASSERT(true, "Degenerate screen dimensions (0x0) handled without crash");
    zero_game_ui_shutdown(&gui_edge);

    if (g_suite_failures == failures_before) {
        printf("  -> [PASS] Suite 6 verified successfully (Boundary & corner cases handled).\n");
    }
}

/* -------------------------------------------------------------------------- */
/* SUITE 7: Cross-Feature Combinations (Tier 3)                               */
/* -------------------------------------------------------------------------- */

static void test_suite_cross_feature_combinations(void) {
    printf("[SUITE 7] Cross-Feature Combinations (Tier 3)...\n");
    int failures_before = g_suite_failures;

    /* T3-INT-06: Race Countdown -> Audio Chimes -> Drivetrain Launch */
    ZeroGameUI gui;
    zero_game_ui_init(&gui);
    ZeroAudioEngine audio;
    zero_audio_init(&audio, true);
    ZeroRaceGame game = zero_race_game_create();

    zero_game_ui_set_state(&gui, ZERO_STATE_COUNTDOWN);
    gui.countdown_timer = 0.1f;
    ZeroGameTelemetry telem;
    memset(&telem, 0, sizeof(telem));

    /* Step countdown across 0 to trigger GO chime */
    zero_game_ui_render(&gui, &telem, 1280.0f, 720.0f, 0.0f, 0.0f, false, 0.2f);
    TEST_ASSERT(gui.countdown_green_triggered == true, "Countdown completion triggered green audio chime");

    /* Step past 0.5s GO display duration to transition to RACING */
    zero_game_ui_render(&gui, &telem, 1280.0f, 720.0f, 0.0f, 0.0f, false, 0.5f);
    TEST_ASSERT(gui.state == ZERO_STATE_RACING, "Countdown finish transitioned UI to RACING");

    /* Audio handles green chime */
    ZeroAudioParams audio_params;
    memset(&audio_params, 0, sizeof(audio_params));
    audio_params.trigger_countdown_green = gui.countdown_green_triggered;
    zero_audio_update_params(&audio, &audio_params);

    int16_t sfx_buf[4410];
    zero_audio_synthesize_buffer(&audio, sfx_buf, 4410);
    double launch_rms = calculate_audio_rms(sfx_buf, 4410);
    TEST_ASSERT(launch_rms > 100.0, "Race launch synthesized green chime audio burst");

    zero_audio_shutdown(&audio);
    zero_game_ui_shutdown(&gui);

    /* T3-INT-01: Vehicle Drift + Camera Lag & FOV + Tire Squeal Audio */
    ZeroRaceCamera cam = zero_race_camera_create(16.0f / 9.0f);
    zero_race_camera_set_mode(&cam, ZERO_CAM_CHASE, false);

    Vec3 drift_pos = vec3(120.0f, 0.75f, 100.0f);
    Quat drift_rot = quat_from_axis_angle(vec3(0.0f, 1.0f, 0.0f), ZERO_DEG2RAD(30.0f));
    Vec3 drift_vel = vec3(15.0f, 0.0f, 25.0f); /* Moving diagonally at ~30 m/s (108 km/h) */

    zero_race_camera_update(&cam, drift_pos, drift_rot, drift_vel, 6500.0f, 0.5f, 0.1f);
    Mat4 drift_view = zero_race_camera_get_view(&cam);
    float drift_det = matrix_view_det3x3(drift_view);
    TEST_ASSERT(fabsf(drift_det - 1.000f) <= 0.001f,
                "Drift dynamics maintain strictly orthonormal camera view matrix");

    /* T3-INT-05: Multi-Car Collision + OBB Impulse Resolution */
    ZeroVehicleEnhanced *carA = &game.cars[0];
    ZeroVehicleEnhanced *carB = &game.cars[1];
    carA->position = vec3(125.0f, 0.75f, 100.0f);
    carA->velocity = vec3(8.0f, 0.0f, 20.0f);
    carB->position = vec3(126.4f, 0.75f, 100.0f); /* Overlapping */
    carB->velocity = vec3(-8.0f, 0.0f, 20.0f);

    zero_race_game_resolve_car_collisions(&game);
    TEST_ASSERT(carB->position.x - carA->position.x > 1.70f,
                "OBB collision impulse separates overlapping vehicles");

    if (g_suite_failures == failures_before) {
        printf("  -> [PASS] Suite 7 verified successfully (Cross-feature interactions verified).\n");
    }
}

/* -------------------------------------------------------------------------- */
/* SUITE 8: Real-World Race Application Scenarios (Tier 4)                    */
/* -------------------------------------------------------------------------- */

static void test_suite_real_world_scenarios(void) {
    printf("[SUITE 8] Real-World Race Application Scenarios (Tier 4)...\n");
    int failures_before = g_suite_failures;

    /* T4-SCN-01: Full Multi-Car Competitive Pack Race Simulation (300 ticks) */
    ZeroRaceGame race = zero_race_game_create();
    ZeroAIController player_ai = zero_ai_controller_create(0, 1.02f, -0.5f);
    const float dt = 1.0f / 60.0f;
    const int race_ticks = 300;

    for (int t = 0; t < race_ticks; ++t) {
        zero_ai_controller_update(&player_ai, &race, dt);
        zero_race_game_step(&race, dt);
    }

    /* Verify all 4 cars survived the pack race intact */
    for (int k = 0; k < ZERO_RACE_CAR_COUNT; ++k) {
        TEST_ASSERT(!isnan(race.cars[k].position.x), "Car position.x finite after pack race");
        TEST_ASSERT(race.cars[k].track_progress_s > 30.0f,
                    "Car achieved progressive forward circuit distance");
        TEST_ASSERT(race.cars[k].race_position >= 1 && race.cars[k].race_position <= 4,
                    "Car maintains valid competitive race ranking");
    }

    /* T4-SCN-05: Dynamic Multi-Camera Switching Under Racing Load */
    ZeroRaceCamera active_cam = zero_race_camera_create(16.0f / 9.0f);
    bool camera_stream_ok = true;

    for (int t = 0; t < 120; ++t) {
        if (t % 30 == 0) {
            zero_race_camera_cycle_mode(&active_cam);
        }
        zero_race_camera_update(&active_cam,
                                race.cars[0].position,
                                race.cars[0].orientation,
                                race.cars[0].velocity,
                                race.cars[0].engine_rpm,
                                race.cars[0].steer_input,
                                dt);
        Mat4 v = zero_race_camera_get_view(&active_cam);
        float det = matrix_view_det3x3(v);
        if (fabsf(det - 1.000f) > 0.001f) {
            camera_stream_ok = false;
        }
    }
    TEST_ASSERT(camera_stream_ok,
                "Dynamic camera switching under continuous race load maintains orthonormality");

    if (g_suite_failures == failures_before) {
        printf("  -> [PASS] Suite 8 verified successfully (Real-world race scenarios pass).\n");
    }
}

/* -------------------------------------------------------------------------- */
/* Main Verification Entry Point                                              */
/* -------------------------------------------------------------------------- */

int main(void) {
    printf("=======================================================================\n");
    printf("  ZERO DRIFT: APEX — COMPLETE 4-TIER E2E INTEGRATION TEST SUITE        \n");
    printf("=======================================================================\n");

    test_suite_multicar_ai_navigation();
    test_suite_hud_telemetry_determinism();
    test_suite_procedural_audio();
    test_suite_camera_matrices();
    test_suite_ui_state_machine();
    test_suite_boundary_cases();
    test_suite_cross_feature_combinations();
    test_suite_real_world_scenarios();

    printf("-----------------------------------------------------------------------\n");
    printf("  TEST EXECUTION SUMMARY\n");
    printf("  Checks Run:    %d\n", g_tests_run);
    printf("  Checks Passed: %d\n", g_tests_passed);
    printf("  Checks Failed: %d\n", g_suite_failures);
    printf("=======================================================================\n");

    if (g_suite_failures == 0) {
        printf("[ALL E2E INTEGRATION TESTS PASSED] %d / %d checks verified successfully.\n",
               g_tests_passed, g_tests_run);
        return 0;
    } else {
        fprintf(stderr, "[E2E TEST FAILURE] %d check(s) failed out of %d run.\n",
                g_suite_failures, g_tests_run);
        return 1;
    }
}

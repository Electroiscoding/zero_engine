#define _POSIX_C_SOURCE 199309L
#include "zero_ai_racers.h"
#include <assert.h>
#include <stdio.h>
#include <math.h>

static void test_spline_parameterization(void) {
    printf("[TEST] Spline Parameterization & Curvature...\n");

    for (int i = 0; i < ZERO_TRACK_SPLINE_COUNT; ++i) {
        float k = zero_track_get_spline_curvature(i);
        assert(k >= 0.0f);
        assert(!isnan(k));
        assert(!isinf(k));

        float v_speed = zero_track_get_max_corner_speed(i, 1.0f);
        assert(v_speed >= 14.0f && v_speed <= 75.0f);

        float v_speed_agg = zero_track_get_max_corner_speed(i, 1.05f);
        assert(v_speed_agg >= v_speed);

        float v_speed_cons = zero_track_get_max_corner_speed(i, 0.88f);
        assert(v_speed_cons <= v_speed);
    }

    // Straight section near start line (node 0)
    float k_start = zero_track_get_spline_curvature(0);
    assert(k_start < 0.05f); // Start straight has low curvature

    // Hairpin/sharp turn section (nodes 15-25)
    float k_turn = zero_track_get_spline_curvature(20);
    assert(k_turn > 0.01f); // Sharp bend has higher curvature

    printf("  -> Spline curvature & corner speeds verified across all 120 nodes.\n");
}

static void test_nearest_segment_search(void) {
    printf("[TEST] Nearest Segment Search...\n");

    // Exact waypoint query (Node 0 at 125.0, 0.0)
    Vec3 p0 = vec3(125.0f, 0.75f, 0.0f);
    int node_idx = -1;
    float t = -1.0f;
    float dist_sq = -1.0f;

    zero_track_find_nearest_segment(p0, &node_idx, &t, &dist_sq);
    assert(node_idx == 0 || node_idx == ZERO_TRACK_SPLINE_COUNT - 1);
    assert(dist_sq < 0.01f);
    assert(t >= 0.0f && t <= 1.0f);

    // Midpoint between node 0 and node 1
    float mid_x = 0.5f * (g_track_spline[0][0] + g_track_spline[1][0]);
    float mid_z = 0.5f * (g_track_spline[0][1] + g_track_spline[1][1]);
    Vec3 p_mid = vec3(mid_x, 0.75f, mid_z);

    zero_track_find_nearest_segment(p_mid, &node_idx, &t, &dist_sq);
    assert(node_idx == 0);
    assert(fabsf(t - 0.5f) < 0.05f);
    assert(dist_sq < 0.01f);

    printf("  -> Nearest segment search projection and distance squared verified.\n");
}

static void test_pure_pursuit_and_braking(void) {
    printf("[TEST] Pure Pursuit Steering & Anticipatory Braking...\n");

    ZeroRaceGame game = zero_race_game_create();
    ZeroAIController *ai = &game.ai_controllers[0]; // AI 1 (Car 1)
    ZeroVehicleEnhanced *car = &game.cars[1];

    // AI at rest on start grid - step for smooth filter convergence
    for (int step = 0; step < 15; ++step) {
        zero_ai_controller_update(ai, &game, 1.0f / 60.0f);
    }

    assert(ai->lookahead_distance >= 9.0f);
    assert(car->throttle_input > 0.5f); // Should request forward throttle
    assert(car->brake_input < 0.05f);   // No braking needed when stationary

    // High speed test heading into a corner (node 15, R ~ 67m where max speed is ~32 m/s)
    car->velocity = vec3(-35.0f, 0.0f, 35.0f); // ~50 m/s > 32 m/s
    car->speed_kmh = 180.0f;
    car->position = vec3(100.0f, 0.75f, 130.0f);

    for (int step = 0; step < 15; ++step) {
        zero_ai_controller_update(ai, &game, 1.0f / 60.0f);
    }

    assert(ai->lookahead_distance > 9.0f + 0.35f * 45.0f);
    // At ~50 m/s approaching turn (max speed ~32 m/s), anticipatory braking must activate
    assert(car->brake_input > 0.3f);
    assert(car->throttle_input < 0.1f);

    printf("  -> Pure pursuit lookahead scaling & anticipatory corner braking verified.\n");
}

static void test_opponent_avoidance(void) {
    printf("[TEST] Opponent Detection & Lateral Avoidance...\n");

    ZeroRaceGame game = zero_race_game_create();
    ZeroAIController *ai = &game.ai_controllers[0]; // AI 1 (Car 1)
    ZeroVehicleEnhanced *car1 = &game.cars[1];
    ZeroVehicleEnhanced *car0 = &game.cars[0]; // Player car

    // Place player car directly 10 meters ahead of AI 1 in the forward cone
    car1->position = vec3(125.0f, 0.75f, 0.0f);
    car1->orientation = quat_identity();
    car1->velocity = vec3(0.0f, 0.0f, 25.0f); // Moving +Z at 25 m/s

    car0->position = vec3(124.5f, 0.75f, 10.0f); // 10m ahead, slightly to left (-X)
    car0->orientation = quat_identity();
    car0->velocity = vec3(0.0f, 0.0f, 12.0f); // Slower car (12 m/s)

    float initial_offset = ai->current_lateral_offset;

    // Update AI for multiple steps to see lateral shift
    for (int step = 0; step < 30; ++step) {
        zero_ai_controller_update(ai, &game, 1.0f / 60.0f);
    }

    // Since car0 is to the left (x_rel < 0), AI 1 must steer right (positive lateral offset shift)
    assert(ai->current_lateral_offset > initial_offset);
    assert(ai->avoidance_timer > 0.0f);

    printf("  -> Forward cone detection (0.5m < z_rel < 22m, |x_rel| < 3.2m) and lateral evasion verified.\n");
}

static void test_obb_collision_resolution(void) {
    printf("[TEST] Pairwise OBB Collision Resolution...\n");

    ZeroRaceGame game = zero_race_game_create();
    ZeroVehicleEnhanced *carA = &game.cars[0];
    ZeroVehicleEnhanced *carB = &game.cars[1];

    // Position two cars overlapping along X (half-width 0.95, overlap 0.30m)
    carA->position = vec3(125.0f, 0.75f, 0.0f);
    carA->velocity = vec3(5.0f, 0.0f, 20.0f); // Moving right (+X)
    carA->angular_velocity = vec3(0.0f, 0.0f, 0.0f);
    carA->orientation = quat_identity();

    carB->position = vec3(126.6f, 0.75f, 0.0f); // Distance 1.6m < 2 * 0.95m = 1.90m
    carB->velocity = vec3(-5.0f, 0.0f, 20.0f); // Moving left (-X)
    carB->angular_velocity = vec3(0.0f, 0.0f, 0.0f);
    carB->orientation = quat_identity();

    float init_dist = carB->position.x - carA->position.x;
    assert(init_dist < 1.90f);

    zero_race_game_resolve_car_collisions(&game);

    float post_dist = carB->position.x - carA->position.x;
    // Symmetrical position separation must have pushed them apart
    assert(post_dist >= 1.85f);

    // Impulse must have reduced or reversed relative normal velocity
    assert(carA->velocity.x < 5.0f);
    assert(carB->velocity.x > -5.0f);

    printf("  -> Symmetrical OBB penetration separation & restitution impulse verified.\n");
}

static void test_race_simulation_determinism(void) {
    printf("[TEST] Deterministic Multi-Car Race Stepping & Rankings...\n");

    ZeroRaceGame game1 = zero_race_game_create();
    ZeroRaceGame game2 = zero_race_game_create();

    const int total_ticks = 240; // 4 seconds at 60 Hz
    const float dt = 1.0f / 60.0f;

    for (int t = 0; t < total_ticks; ++t) {
        game1.cars[0].throttle_input = 1.0f;
        game2.cars[0].throttle_input = 1.0f;
        zero_race_game_step(&game1, dt);
        zero_race_game_step(&game2, dt);
    }

    assert(game1.tick == (uint64_t)total_ticks);
    assert(game2.tick == (uint64_t)total_ticks);
    assert(fabsf(game1.race_time - 4.0f) < 1e-4f);

    // Check exact bitwise determinism between the two runs
    for (int k = 0; k < ZERO_RACE_CAR_COUNT; ++k) {
        assert(game1.cars[k].position.x == game2.cars[k].position.x);
        assert(game1.cars[k].position.y == game2.cars[k].position.y);
        assert(game1.cars[k].position.z == game2.cars[k].position.z);
        assert(game1.cars[k].velocity.x == game2.cars[k].velocity.x);
        assert(game1.cars[k].velocity.z == game2.cars[k].velocity.z);
        assert(game1.cars[k].race_position == game2.cars[k].race_position);
        assert(game1.cars[k].race_position >= 1 && game1.cars[k].race_position <= 4);

        // Cars should have advanced along the track
        assert(vec3_length(game1.cars[k].velocity) > 5.0f);
    }

    // Positions must be unique (1st, 2nd, 3rd, 4th)
    int pos_seen[5] = {0};
    for (int k = 0; k < ZERO_RACE_CAR_COUNT; ++k) {
        int r = game1.cars[k].race_position;
        assert(r >= 1 && r <= 4);
        pos_seen[r]++;
    }
    for (int r = 1; r <= 4; ++r) {
        assert(pos_seen[r] == 1);
    }

    printf("  -> 100%% deterministic simulation across 240 ticks verified. Unique 1st-4th rankings verified.\n");
}

static void test_ai_lap_navigation(void) {
    printf("[TEST] AI Multi-Car Spline Navigation & Track Containment (1200 Ticks)...\n");

    ZeroRaceGame game = zero_race_game_create();
    const int total_ticks = 1200; // 20.0 seconds of race time
    const float dt = 1.0f / 60.0f;

    for (int t = 0; t < total_ticks; ++t) {
        game.cars[0].throttle_input = 1.0f;
        zero_race_game_step(&game, dt);

        // Every 60 ticks (1 sec), assert AI cars remain strictly within track boundaries
        if (t % 60 == 0) {
            for (int k = 1; k < ZERO_RACE_CAR_COUNT; ++k) {
                int near_node = 0;
                float near_t = 0.0f;
                float dist_sq = 0.0f;
                zero_track_find_nearest_segment(game.cars[k].position, &near_node, &near_t, &dist_sq);
                float dist_to_centerline = sqrtf(dist_sq);

                // ZERO_TRACK_BARRIER_DIST = 11.0m
                assert(dist_to_centerline <= ZERO_TRACK_BARRIER_DIST);
                assert(!isnan(game.cars[k].position.x));
                assert(!isnan(game.cars[k].position.z));
            }
        }
    }

    // Verify AI cars completed significant progress around the track
    for (int k = 1; k < ZERO_RACE_CAR_COUNT; ++k) {
        float total_dist = (float)game.cars[k].lap_count * 942.5f + game.cars[k].track_progress_s;
        printf("    Car %d: dist=%.1f m, speed=%.1f km/h, pos=(%.1f, %.1f), laps=%d\n",
               k, total_dist, game.cars[k].speed_kmh, game.cars[k].position.x, game.cars[k].position.z, game.cars[k].lap_count);
        assert(total_dist > 250.0f); // Traveled over 250m in 20s
        assert(game.cars[k].speed_kmh > 30.0f); // Driving at race speeds
    }

    printf("  -> AI cars navigated closed-circuit spline track within boundaries at high speeds.\n");
}

int main(void) {
    printf("=== ZERO ENGINE: MILESTONE M3 AI RACERS TEST SUITE ===\n");
    test_spline_parameterization();
    test_nearest_segment_search();
    test_pure_pursuit_and_braking();
    test_opponent_avoidance();
    test_obb_collision_resolution();
    test_race_simulation_determinism();
    test_ai_lap_navigation();
    printf("[ALL M3 TESTS PASSED] Autonomous AI Racers verified successfully.\n");
    return 0;
}

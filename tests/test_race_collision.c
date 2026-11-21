#define _POSIX_C_SOURCE 199309L
#include "zero_ai_racers.h"
#include "zero_audio.h"
#include "zero_track_mesh.h"
#include "zero_track_spline.h"
#include "zero_vehicle_physics.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <assert.h>

#define TEST_CHECK(cond, msg) do { \
    if (!(cond)) { \
        printf("[FAIL] %s:%d: %s\n", __FILE__, __LINE__, msg); \
        exit(1); \
    } else { \
        printf("[PASS] %s\n", msg); \
    } \
} while (0)

int main(void) {
    printf("=== TESTING CRAZY COLLISION & DEBRIS PHYSICS ===\n");

    /* 1. Test High-Velocity Car-to-Car Impact & Impulse Event */
    ZeroRaceGame game = zero_race_game_create();
    ZeroVehicleEnhanced *carA = &game.cars[0];
    ZeroVehicleEnhanced *carB = &game.cars[1];

    /* Overlapping lateral T-bone / side swipe with high relative velocity */
    carA->position = vec3(125.0f, 0.75f, 50.0f);
    carA->velocity = vec3(18.0f, 0.0f, 20.0f);
    carB->position = vec3(126.2f, 0.75f, 50.0f);
    carB->velocity = vec3(-18.0f, 0.0f, 20.0f);

    zero_race_game_resolve_car_collisions(&game);

    TEST_CHECK(game.collision_event_count > 0, "Car collision event recorded");
    ZeroCollisionEvent *ev = &game.collision_events[0];
    TEST_CHECK(ev->carA_index == 0 && ev->carB_index == 1, "Collision event references correct car indices");
    TEST_CHECK(ev->impulse_magnitude > 5000.0f, "High impulse magnitude generated");
    TEST_CHECK(ev->relative_speed > 30.0f, "Relative speed detected accurately");
    TEST_CHECK(ev->is_severe == true, "High velocity crash flagged as severe");

    /* 2. Test Vertical Pop & Spinout Torque */
    TEST_CHECK(carA->velocity.y > 0.5f, "Car A pops vertically from kinetic impact");
    TEST_CHECK(carB->velocity.y > 0.5f, "Car B pops vertically from kinetic impact");
    TEST_CHECK(vec3_length(carA->angular_velocity) > 0.1f, "Car A receives angular torque impulse");
    TEST_CHECK(vec3_length(carB->angular_velocity) > 0.1f, "Car B receives angular torque impulse");

    /* 3. Test Inviolable Barrier Collision & Reflection */
    ZeroVehicleEnhanced fast_car = zero_vehicle_enhanced_create(vec3(145.0f, 0.75f, 0.0f), quat_identity());
    fast_car.velocity = vec3(35.0f, 0.0f, 10.0f); /* Flying towards outer barrier */

    zero_vehicle_resolve_barrier_collision(&fast_car, NULL);

    int node = 0; float seg_t = 0.0f; float dist_sq = 0.0f;
    zero_track_find_nearest_segment(fast_car.position, &node, &seg_t, &dist_sq);
    float dist = sqrtf(dist_sq);
    TEST_CHECK(dist <= ZERO_TRACK_BARRIER_DIST, "Car position strictly contained inside track barrier");

    /* 4. Test Crash Audio Synthesizer */
    ZeroAudioEngine audio;
    bool audio_ok = zero_audio_init(&audio, true); /* Force software mode */
    TEST_CHECK(audio_ok, "Audio engine initialized in software mode");

    ZeroAudioParams a_params;
    memset(&a_params, 0, sizeof(a_params));
    a_params.trigger_crash = true;
    a_params.crash_intensity = 1.5f;
    zero_audio_update_params(&audio, &a_params);

    int16_t samples[2048];
    zero_audio_synthesize_buffer(&audio, samples, 2048);

    bool has_nonzero_sound = false;
    for (int i = 0; i < 2048; ++i) {
        if (abs(samples[i]) > 100) {
            has_nonzero_sound = true;
            break;
        }
    }
    TEST_CHECK(has_nonzero_sound, "Crash sound synthesized with audible metal crunch waveform");

    zero_audio_shutdown(&audio);

    printf("=== ALL COLLISION & DEBRIS PHYSICS TESTS PASSED ===\n");
    return 0;
}

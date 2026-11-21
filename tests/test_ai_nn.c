#define _POSIX_C_SOURCE 199309L
#include "zero_ai_nn.h"
#include "zero_ai_models.h"
#include "zero_ai_racers.h"
#include "zero_track_spline.h"
#include "zero_vehicle_physics.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define TEST_CHECK(cond, msg) do { \
    if (!(cond)) { \
        printf("[FAIL] %s:%d: %s\n", __FILE__, __LINE__, msg); \
        exit(1); \
    } else { \
        printf("[PASS] %s\n", msg); \
    } \
} while (0)

int main(void) {
    printf("=== ZERO DRIFT: APEX — NEURAL NETWORK AI VERIFICATION SUITE ===\n\n");

    /* 1. Test NN Structure & Activation Bounds */
    ZeroNeuralNet test_net;
    zero_nn_init_zeros(&test_net);
    test_net.b3[0] = 5.0f;  /* tanh(5.0) -> ~0.9999 */
    test_net.b3[1] = 4.0f;  /* sigmoid(4.0) -> ~0.982 */
    test_net.b3[2] = -4.0f; /* sigmoid(-4.0) -> ~0.018 */

    float dummy_in[ZERO_NN_INPUT_DIM] = {0};
    float dummy_out[ZERO_NN_OUTPUT_DIM] = {0};
    zero_nn_forward(&test_net, dummy_in, dummy_out);

    TEST_CHECK(dummy_out[0] >= -1.0f && dummy_out[0] <= 1.0f, "Steer output is bounded within [-1.0, 1.0]");
    TEST_CHECK(dummy_out[1] >= 0.0f && dummy_out[1] <= 1.0f, "Throttle output is bounded within [0.0, 1.0]");
    TEST_CHECK(dummy_out[2] >= 0.0f && dummy_out[2] <= 1.0f, "Brake output is bounded within [0.0, 1.0]");
    TEST_CHECK(dummy_out[0] > 0.95f, "Steer correctly activates to near-maximum tanh");
    TEST_CHECK(dummy_out[1] > 0.90f, "Throttle correctly activates to near-maximum sigmoid");
    TEST_CHECK(dummy_out[2] < 0.10f, "Brake correctly de-activates to near-minimum sigmoid");

    /* 2. Test Pretrained Models Fetching */
    const ZeroNeuralNet *m_apex = zero_ai_get_model(ZERO_AI_APEX_PREDATOR);
    const ZeroNeuralNet *m_prec = zero_ai_get_model(ZERO_AI_PRECISION_MASTER);
    const ZeroNeuralNet *m_drift = zero_ai_get_model(ZERO_AI_DRIFT_KING);

    TEST_CHECK(m_apex != NULL, "Apex Predator model exists");
    TEST_CHECK(m_prec != NULL, "Precision Master model exists");
    TEST_CHECK(m_drift != NULL, "Drift King model exists");
    TEST_CHECK(m_apex != m_prec, "Apex and Precision models have distinct pointers");
    TEST_CHECK(m_prec != m_drift, "Precision and Drift models have distinct pointers");

    /* 3. Test Observation Extraction Vector */
    ZeroRaceGame game = zero_race_game_create();
    float obs[ZERO_NN_INPUT_DIM];
    memset(obs, 0, sizeof(obs));

    zero_ai_extract_observations(&game.cars[1], &game, obs);
    for (int i = 0; i < ZERO_NN_INPUT_DIM; ++i) {
        TEST_CHECK(!isnan(obs[i]) && !isinf(obs[i]), "Observation feature is finite and valid");
    }
    TEST_CHECK(obs[11] >= 0.0f && obs[11] <= 1.5f, "Track barrier clearance feature properly normalized");

    /* 4. Test Multi-Car Multi-Lap Simulation with Distinct Personalities (600 Ticks) */
    printf("\nRunning 600-tick headless multi-personality race simulation...\n");
    float max_speed[4] = {0};
    float max_drift[4] = {0};
    float max_barrier_dist[4] = {0};

    ZeroAIController player_ai = zero_ai_controller_create(0, 0.98f, 0.0f);

    for (int step = 0; step < 1200; ++step) {
        zero_ai_controller_update(&player_ai, &game, 0.016667f);
        zero_race_game_step(&game, 0.016667f);

        if (step >= 420 && step <= 480 && step % 10 == 0) {
            printf("  [Tick %d] finished=%d win=%d Car1: lap=%d spd=%.1f thr=%.2f brk=%.2f Car3: lap=%d spd=%.1f thr=%.2f brk=%.2f\n",
                   step, game.race_finished, game.winning_car_index,
                   game.cars[1].lap_count, game.cars[1].speed_kmh, game.cars[1].throttle_input, game.cars[1].brake_input,
                   game.cars[3].lap_count, game.cars[3].speed_kmh, game.cars[3].throttle_input, game.cars[3].brake_input);
        }

        for (int i = 1; i < 4; ++i) {
            float spd = vec3_length(game.cars[i].velocity) * 3.6f;
            if (spd > max_speed[i]) max_speed[i] = spd;

            if (game.cars[i].drift_angle > max_drift[i]) {
                max_drift[i] = game.cars[i].drift_angle;
            }

            int seg = 0; float seg_t = 0.0f; float dist_sq = 0.0f;
            zero_track_find_nearest_segment(game.cars[i].position, &seg, &seg_t, &dist_sq);
            float dist = sqrtf(dist_sq);
            if (dist > max_barrier_dist[i]) max_barrier_dist[i] = dist;
        }
    }

    printf("  [Telemetry] Car 1 (Apex Predator): Top Speed = %.1f km/h, Max Drift = %.1f deg\n", max_speed[1], max_drift[1]);
    printf("  [Telemetry] Car 2 (Precision Master): Top Speed = %.1f km/h, Max Deviation = %.2f m\n", max_speed[2], max_barrier_dist[2]);
    printf("  [Telemetry] Car 3 (Drift King): Top Speed = %.1f km/h, Max Drift = %.1f deg\n", max_speed[3], max_drift[3]);

    /* Verification assertions */
    for (int i = 1; i < 4; ++i) {
        TEST_CHECK(max_speed[i] > 40.0f, "AI car reaches race velocity (> 40 km/h)");
        TEST_CHECK(max_barrier_dist[i] < ZERO_TRACK_BARRIER_DIST, "AI car remains strictly inside track boundaries");
        TEST_CHECK(game.ai_controllers[i - 1].use_neural_net == true, "AI controller is operating in Neural Network mode");
    }

    TEST_CHECK(max_drift[3] > 8.0f, "Drift King exhibits dynamic slip-angle drifts");
    TEST_CHECK(max_barrier_dist[2] < ZERO_TRACK_BARRIER_DIST - 1.5f, "Precision Master maintains tight track tolerance");

    printf("\n=== ALL NEURAL NETWORK AI RACER CHECKS PASSED ===\n");
    return 0;
}

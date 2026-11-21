#define _POSIX_C_SOURCE 199309L
#include "zero_ai_nn.h"
#include "zero_ai_racers.h"
#include "zero_track_spline.h"
#include "zero_vehicle_physics.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define POPULATION_SIZE 32
#define NUM_GENERATIONS 40
#define EVAL_TICKS 1200
#define FIXED_DT 0.016667f

static float rand_uniform(float min_val, float max_val) {
    float r = (float)rand() / (float)RAND_MAX;
    return min_val + r * (max_val - min_val);
}

static float rand_gaussian(float mean, float stddev) {
    float u1 = (float)rand() / ((float)RAND_MAX + 1.0f);
    float u2 = (float)rand() / ((float)RAND_MAX + 1.0f);
    if (u1 < 1e-7f) u1 = 1e-7f;
    float z0 = sqrtf(-2.0f * logf(u1)) * cosf(2.0f * ZERO_PI * u2);
    return mean + z0 * stddev;
}

static void mutate_net(ZeroNeuralNet *dst, const ZeroNeuralNet *src, float rate, float sigma) {
    *dst = *src;
    float *w_ptr = (float *)dst;
    int num_floats = sizeof(ZeroNeuralNet) / sizeof(float);
    for (int i = 0; i < num_floats; ++i) {
        if (rand_uniform(0.0f, 1.0f) < rate) {
            w_ptr[i] += rand_gaussian(0.0f, sigma);
        }
    }
}

static void extract_car_obs(const ZeroVehicleEnhanced *car, float lateral_offset, float heading_err,
                            float curv_0, float curv_10, float curv_25, float curv_45,
                            float out_obs[ZERO_NN_INPUT_DIM]) {
    float speed = vec3_length(car->velocity);
    Mat4 rot = quat_to_mat4(car->orientation);
    Vec3 fwd = vec3(rot.m[8], rot.m[9], rot.m[10]);
    Vec3 right = vec3(rot.m[0], rot.m[1], rot.m[2]);

    float v_fwd = vec3_dot(car->velocity, fwd);
    float v_lat = vec3_dot(car->velocity, right);

    out_obs[0] = speed / 75.0f;
    out_obs[1] = v_fwd / 75.0f;
    out_obs[2] = v_lat / 20.0f;
    out_obs[3] = car->angular_velocity.y / 4.0f;
    out_obs[4] = car->drift_angle / 45.0f;
    out_obs[5] = lateral_offset / 7.0f;
    out_obs[6] = heading_err / ZERO_PI;
    out_obs[7] = curv_0 * 40.0f;
    out_obs[8] = curv_10 * 40.0f;
    out_obs[9] = curv_25 * 40.0f;
    out_obs[10] = curv_45 * 40.0f;
    out_obs[11] = (ZERO_TRACK_BARRIER_DIST - fabsf(lateral_offset)) / ZERO_TRACK_BARRIER_DIST;
    out_obs[12] = 1.0f; /* Front obstacle clear */
    out_obs[13] = 0.0f; /* Rel closing speed */
    out_obs[14] = 0.0f; /* Left clear */
    out_obs[15] = 0.0f; /* Right clear */
}

typedef enum {
    TRAIN_APEX_PREDATOR,
    TRAIN_PRECISION_MASTER,
    TRAIN_DRIFT_KING
} TrainingStyle;

static float evaluate_genome(const ZeroNeuralNet *net, TrainingStyle style) {
    ZeroVehicleEnhanced car = zero_vehicle_enhanced_create(vec3(125.0f, 0.75f, 0.0f), quat_identity());
    car.velocity = vec3(0.0f, 0.0f, 15.0f);

    float total_distance = 0.0f;
    float avg_speed_accum = 0.0f;
    float centerline_dev_accum = 0.0f;
    float drift_angle_accum = 0.0f;
    float wall_hits = 0.0f;
    int ticks_alive = 0;

    for (int t = 0; t < EVAL_TICKS; ++t) {
        int seg = 0;
        float seg_t = 0.0f;
        float dist_sq = 0.0f;
        zero_track_find_nearest_segment(car.position, &seg, &seg_t, &dist_sq);
        float lat_dist = sqrtf(dist_sq);

        int next_seg = (seg + 1) % ZERO_TRACK_SPLINE_COUNT;
        float tx = g_track_spline[next_seg][0] - g_track_spline[seg][0];
        float tz = g_track_spline[next_seg][1] - g_track_spline[seg][1];
        float spline_yaw = atan2f(tx, tz);

        Mat4 rot = quat_to_mat4(car.orientation);
        Vec3 fwd = vec3(rot.m[8], rot.m[9], rot.m[10]);
        float car_yaw = atan2f(fwd.x, fwd.z);
        float heading_err = car_yaw - spline_yaw;
        while (heading_err > ZERO_PI) heading_err -= 2.0f * ZERO_PI;
        while (heading_err < -ZERO_PI) heading_err += 2.0f * ZERO_PI;

        float c0 = zero_track_get_spline_curvature(seg);
        float c10 = zero_track_get_spline_curvature((seg + 2) % ZERO_TRACK_SPLINE_COUNT);
        float c25 = zero_track_get_spline_curvature((seg + 5) % ZERO_TRACK_SPLINE_COUNT);
        float c45 = zero_track_get_spline_curvature((seg + 9) % ZERO_TRACK_SPLINE_COUNT);

        float obs[ZERO_NN_INPUT_DIM];
        extract_car_obs(&car, lat_dist, heading_err, c0, c10, c25, c45, obs);

        float out[ZERO_NN_OUTPUT_DIM];
        zero_nn_forward(net, obs, out);

        car.steer_input = out[0];
        car.throttle_input = out[1];
        car.brake_input = out[2];

        zero_vehicle_enhanced_update(&car, FIXED_DT, NULL);

        float spd = vec3_length(car.velocity);
        total_distance += spd * FIXED_DT;
        avg_speed_accum += spd;
        centerline_dev_accum += lat_dist;
        drift_angle_accum += car.drift_angle;

        if (lat_dist > ZERO_TRACK_BARRIER_DIST - 1.0f) {
            wall_hits += 1.0f;
        }

        ticks_alive++;
        if (lat_dist > ZERO_TRACK_BARRIER_DIST + 2.0f) {
            break; /* Off track */
        }
    }

    float avg_speed = avg_speed_accum / (float)(ticks_alive + 1);
    float avg_dev = centerline_dev_accum / (float)(ticks_alive + 1);
    float avg_drift = drift_angle_accum / (float)(ticks_alive + 1);

    float score = total_distance * 2.0f + avg_speed * 15.0f - wall_hits * 30.0f - avg_dev * 10.0f;

    if (style == TRAIN_APEX_PREDATOR) {
        score += avg_speed * 25.0f; /* Extreme priority on speed */
    } else if (style == TRAIN_PRECISION_MASTER) {
        score -= avg_dev * 35.0f; /* Extreme priority on clipping line */
    } else if (style == TRAIN_DRIFT_KING) {
        score += avg_drift * 12.0f; /* Priority on controlled slide angles */
    }

    return score;
}

static void init_baseline_model(ZeroNeuralNet *net, TrainingStyle style) {
    zero_nn_init_zeros(net);

    /* Heuristic seed weights mapping input features to steering, throttle, and brake */
    for (int i = 0; i < ZERO_NN_HIDDEN1_DIM; ++i) {
        for (int j = 0; j < ZERO_NN_INPUT_DIM; ++j) {
            net->w1[i][j] = rand_uniform(-0.15f, 0.15f);
        }
        net->b1[i] = 0.05f;
    }

    for (int i = 0; i < ZERO_NN_HIDDEN2_DIM; ++i) {
        for (int j = 0; j < ZERO_NN_HIDDEN1_DIM; ++j) {
            net->w2[i][j] = rand_uniform(-0.15f, 0.15f);
        }
        net->b2[i] = 0.05f;
    }

    /* Steering output: strongly coupled to lateral offset (in[5]) and heading error (in[6]) */
    net->w1[0][5] = -0.75f; /* Counter-steer lateral offset */
    net->w1[0][6] = -1.20f; /* Align with heading error */
    net->w2[0][0] = 1.10f;
    net->w3[0][0] = 1.30f;
    net->b3[0] = 0.0f;

    /* Throttle output: high base bias, scaled by speed ratio */
    net->b3[1] = (style == TRAIN_APEX_PREDATOR) ? 2.5f : (style == TRAIN_DRIFT_KING ? 2.2f : 1.8f);
    net->w1[1][0] = -0.60f; /* Ease throttle at max speed */
    net->w2[1][1] = 0.80f;
    net->w3[1][1] = 0.90f;

    /* Brake output: triggers on high curvature lookahead */
    net->w1[2][8] = (style == TRAIN_APEX_PREDATOR) ? 0.85f : 1.40f;
    net->w1[2][9] = (style == TRAIN_APEX_PREDATOR) ? 0.65f : 1.20f;
    net->w2[2][2] = 1.10f;
    net->w3[2][2] = 1.25f;
    net->b3[2] = (style == TRAIN_APEX_PREDATOR) ? -2.2f : -1.5f;

    if (style == TRAIN_DRIFT_KING) {
        /* Encourage drift slip-angle counter-steering */
        net->w1[3][4] = -0.55f;
        net->w2[3][3] = 0.85f;
        net->w3[0][3] = -0.70f;
    }
}

static ZeroNeuralNet train_style(TrainingStyle style, const char *name) {
    printf("Training %s (Generations: %d, Population: %d)...\n", name, NUM_GENERATIONS, POPULATION_SIZE);
    srand(42 + (int)style * 100);

    ZeroNeuralNet population[POPULATION_SIZE];
    float fitness[POPULATION_SIZE];

    init_baseline_model(&population[0], style);
    for (int i = 1; i < POPULATION_SIZE; ++i) {
        mutate_net(&population[i], &population[0], 0.25f, 0.12f);
    }

    ZeroNeuralNet best_net = population[0];
    float best_fitness = -1e9f;

    for (int gen = 0; gen < NUM_GENERATIONS; ++gen) {
        int gen_best_idx = 0;
        float gen_best_fitness = -1e9f;

        for (int i = 0; i < POPULATION_SIZE; ++i) {
            fitness[i] = evaluate_genome(&population[i], style);
            if (fitness[i] > gen_best_fitness) {
                gen_best_fitness = fitness[i];
                gen_best_idx = i;
            }
        }

        if (gen_best_fitness > best_fitness) {
            best_fitness = gen_best_fitness;
            best_net = population[gen_best_idx];
        }

        if ((gen + 1) % 10 == 0 || gen == NUM_GENERATIONS - 1) {
            printf("  [%s] Gen %2d/%d: Best Fitness = %.1f (All-time: %.1f)\n",
                   name, gen + 1, NUM_GENERATIONS, gen_best_fitness, best_fitness);
        }

        /* Elitism: carry over top 4 */
        ZeroNeuralNet next_pop[POPULATION_SIZE];
        next_pop[0] = best_net;
        next_pop[1] = population[gen_best_idx];

        for (int i = 2; i < POPULATION_SIZE; ++i) {
            int parent = rand() % (POPULATION_SIZE / 4);
            mutate_net(&next_pop[i], &population[parent], 0.18f, 0.08f);
        }

        memcpy(population, next_pop, sizeof(population));
    }

    printf("=> Completed %s: Best Fitness = %.1f\n\n", name, best_fitness);
    return best_net;
}

static void print_net_c_code(FILE *f, const char *var_name, const ZeroNeuralNet *net) {
    fprintf(f, "static const ZeroNeuralNet %s = {\n", var_name);

    /* w1 */
    fprintf(f, "    .w1 = {\n");
    for (int i = 0; i < ZERO_NN_HIDDEN1_DIM; ++i) {
        fprintf(f, "        {");
        for (int j = 0; j < ZERO_NN_INPUT_DIM; ++j) {
            fprintf(f, "%.6ff%s", net->w1[i][j], (j + 1 < ZERO_NN_INPUT_DIM) ? ", " : "");
        }
        fprintf(f, "}%s\n", (i + 1 < ZERO_NN_HIDDEN1_DIM) ? "," : "");
    }
    fprintf(f, "    },\n");

    /* b1 */
    fprintf(f, "    .b1 = {");
    for (int i = 0; i < ZERO_NN_HIDDEN1_DIM; ++i) {
        fprintf(f, "%.6ff%s", net->b1[i], (i + 1 < ZERO_NN_HIDDEN1_DIM) ? ", " : "");
    }
    fprintf(f, "},\n");

    /* w2 */
    fprintf(f, "    .w2 = {\n");
    for (int i = 0; i < ZERO_NN_HIDDEN2_DIM; ++i) {
        fprintf(f, "        {");
        for (int j = 0; j < ZERO_NN_HIDDEN1_DIM; ++j) {
            fprintf(f, "%.6ff%s", net->w2[i][j], (j + 1 < ZERO_NN_HIDDEN1_DIM) ? ", " : "");
        }
        fprintf(f, "}%s\n", (i + 1 < ZERO_NN_HIDDEN2_DIM) ? "," : "");
    }
    fprintf(f, "    },\n");

    /* b2 */
    fprintf(f, "    .b2 = {");
    for (int i = 0; i < ZERO_NN_HIDDEN2_DIM; ++i) {
        fprintf(f, "%.6ff%s", net->b2[i], (i + 1 < ZERO_NN_HIDDEN2_DIM) ? ", " : "");
    }
    fprintf(f, "},\n");

    /* w3 */
    fprintf(f, "    .w3 = {\n");
    for (int i = 0; i < ZERO_NN_OUTPUT_DIM; ++i) {
        fprintf(f, "        {");
        for (int j = 0; j < ZERO_NN_HIDDEN2_DIM; ++j) {
            fprintf(f, "%.6ff%s", net->w3[i][j], (j + 1 < ZERO_NN_HIDDEN2_DIM) ? ", " : "");
        }
        fprintf(f, "}%s\n", (i + 1 < ZERO_NN_OUTPUT_DIM) ? "," : "");
    }
    fprintf(f, "    },\n");

    /* b3 */
    fprintf(f, "    .b3 = {");
    for (int i = 0; i < ZERO_NN_OUTPUT_DIM; ++i) {
        fprintf(f, "%.6ff%s", net->b3[i], (i + 1 < ZERO_NN_OUTPUT_DIM) ? ", " : "");
    }
    fprintf(f, "}\n");

    fprintf(f, "};\n\n");
}

int main(void) {
    printf("=================================================================\n");
    printf("  ZERO ENGINE NEURAL NETWORK RACER TRAINING PIPELINE (PURE C)     \n");
    printf("=================================================================\n\n");

    ZeroNeuralNet apex_predator = train_style(TRAIN_APEX_PREDATOR, "Apex Predator (Car 1)");
    ZeroNeuralNet precision_master = train_style(TRAIN_PRECISION_MASTER, "Precision Master (Car 2)");
    ZeroNeuralNet drift_king = train_style(TRAIN_DRIFT_KING, "Drift King (Car 3)");

    const char *out_path = "include/zero_ai_models.h";
    FILE *f = fopen(out_path, "w");
    if (!f) {
        fprintf(stderr, "Failed to open %s for writing\n", out_path);
        return 1;
    }

    fprintf(f, "#ifndef ZERO_AI_MODELS_H\n");
    fprintf(f, "#define ZERO_AI_MODELS_H\n\n");
    fprintf(f, "#include \"zero_ai_nn.h\"\n\n");
    fprintf(f, "typedef enum ZeroAIPersonality {\n");
    fprintf(f, "    ZERO_AI_APEX_PREDATOR = 0,  /* Aggressive, late braking, high attack speed */\n");
    fprintf(f, "    ZERO_AI_PRECISION_MASTER = 1,/* Geometric line clipping, smooth trail braking */\n");
    fprintf(f, "    ZERO_AI_DRIFT_KING = 2       /* High slip-angle power slides and counter-steering */\n");
    fprintf(f, "} ZeroAIPersonality;\n\n");

    print_net_c_code(f, "g_ai_model_apex_predator", &apex_predator);
    print_net_c_code(f, "g_ai_model_precision_master", &precision_master);
    print_net_c_code(f, "g_ai_model_drift_king", &drift_king);

    fprintf(f, "static inline const ZeroNeuralNet *zero_ai_get_model(ZeroAIPersonality personality) {\n");
    fprintf(f, "    switch (personality) {\n");
    fprintf(f, "        case ZERO_AI_APEX_PREDATOR:    return &g_ai_model_apex_predator;\n");
    fprintf(f, "        case ZERO_AI_PRECISION_MASTER: return &g_ai_model_precision_master;\n");
    fprintf(f, "        case ZERO_AI_DRIFT_KING:       return &g_ai_model_drift_king;\n");
    fprintf(f, "        default:                       return &g_ai_model_apex_predator;\n");
    fprintf(f, "    }\n");
    fprintf(f, "}\n\n");

    fprintf(f, "#endif // ZERO_AI_MODELS_H\n");
    fclose(f);

    printf("Successfully emitted trained model weights to: %s\n", out_path);
    return 0;
}

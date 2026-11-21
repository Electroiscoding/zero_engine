#include "zero_ai_racers.h"
#include <math.h>
#include <string.h>
#include <stdbool.h>

// -----------------------------------------------------------------------------
// Internal Spline Precomputations & Parameterization
// -----------------------------------------------------------------------------

static bool s_spline_initialized = false;
static float s_segment_lengths[ZERO_TRACK_SPLINE_COUNT];
static float s_arc_lengths[ZERO_TRACK_SPLINE_COUNT];
static float s_total_circuit_length = 0.0f;
static Vec3 s_tangents[ZERO_TRACK_SPLINE_COUNT];
static Vec3 s_normals[ZERO_TRACK_SPLINE_COUNT];
static float s_curvatures[ZERO_TRACK_SPLINE_COUNT];
static float s_max_corner_speeds[ZERO_TRACK_SPLINE_COUNT];

static inline Vec3 rotate_vector_by_quat(Quat q, Vec3 v) {
    Vec3 q_vec = vec3(q.x, q.y, q.z);
    Vec3 uv = vec3_cross(q_vec, v);
    Vec3 uuv = vec3_cross(q_vec, uv);
    return vec3_add(v, vec3_scale(vec3_add(vec3_scale(uv, q.w), uuv), 2.0f));
}

static inline Vec3 apply_world_inv_inertia(Quat orientation, Vec3 inv_inertia, Vec3 v) {
    Quat q_conj = quat_conjugate(orientation);
    Vec3 local_v = rotate_vector_by_quat(q_conj, v);
    Vec3 local_res = vec3(local_v.x * inv_inertia.x,
                          local_v.y * inv_inertia.y,
                          local_v.z * inv_inertia.z);
    return rotate_vector_by_quat(orientation, local_res);
}

static void init_spline_tables_once(void) {
    if (s_spline_initialized) return;

    float accum_s = 0.0f;
    for (int i = 0; i < ZERO_TRACK_SPLINE_COUNT; ++i) {
        int next_i = (i + 1) % ZERO_TRACK_SPLINE_COUNT;
        float dx = g_track_spline[next_i][0] - g_track_spline[i][0];
        float dz = g_track_spline[next_i][1] - g_track_spline[i][1];
        float seg_len = sqrtf(dx * dx + dz * dz);
        if (seg_len < 1e-4f) seg_len = 1e-4f;

        s_segment_lengths[i] = seg_len;
        s_arc_lengths[i] = accum_s;
        accum_s += seg_len;
    }
    s_total_circuit_length = accum_s;

    for (int i = 0; i < ZERO_TRACK_SPLINE_COUNT; ++i) {
        int prev_i = (i + ZERO_TRACK_SPLINE_COUNT - 1) % ZERO_TRACK_SPLINE_COUNT;
        int next_i = (i + 1) % ZERO_TRACK_SPLINE_COUNT;

        float tin_x = (g_track_spline[i][0] - g_track_spline[prev_i][0]) / s_segment_lengths[prev_i];
        float tin_z = (g_track_spline[i][1] - g_track_spline[prev_i][1]) / s_segment_lengths[prev_i];
        Vec3 t_in = vec3_normalize(vec3(tin_x, 0.0f, tin_z));

        float tout_x = (g_track_spline[next_i][0] - g_track_spline[i][0]) / s_segment_lengths[i];
        float tout_z = (g_track_spline[next_i][1] - g_track_spline[i][1]) / s_segment_lengths[i];
        Vec3 t_out = vec3_normalize(vec3(tout_x, 0.0f, tout_z));

        Vec3 t_node = vec3_normalize(vec3_add(t_in, t_out));
        s_tangents[i] = t_node;

        // Normal pointing to the right of the track tangent in horizontal plane
        s_normals[i] = vec3(t_node.z, 0.0f, -t_node.x);

        float ds_avg = 0.5f * (s_segment_lengths[prev_i] + s_segment_lengths[i]);
        Vec3 d_tan = vec3_sub(t_out, t_in);
        float d_tan_len = vec3_length(d_tan);
        float curvature = (ds_avg > 1e-4f) ? (d_tan_len / ds_avg) : 0.0f;
        s_curvatures[i] = curvature;

        // Max cornering speed with baseline aggressiveness = 1.0f
        const float mu = 1.45f;
        const float g = 9.81f;
        const float rho = 1.225f;
        const float cl_a = 2.53f;
        const float m = 1150.0f;

        if (curvature < 0.0005f) {
            s_max_corner_speeds[i] = 75.0f; // Straight: max top speed 270 km/h
        } else {
            float r = 1.0f / curvature;
            float k_aero = 0.5f * mu * rho * cl_a / m;
            float denom = 1.0f - k_aero * r;
            if (denom < 0.15f) denom = 0.15f;
            float v_corner = sqrtf((mu * g * r) / denom);
            if (v_corner < 14.0f) v_corner = 14.0f;
            if (v_corner > 75.0f) v_corner = 75.0f;
            s_max_corner_speeds[i] = v_corner;
        }
    }

    s_spline_initialized = true;
}

static Vec3 evaluate_spline_aim_point(float s, float lateral_offset, int *out_node_idx) {
    init_spline_tables_once();

    while (s >= s_total_circuit_length) s -= s_total_circuit_length;
    while (s < 0.0f) s += s_total_circuit_length;

    int seg_idx = 0;
    for (int i = 0; i < ZERO_TRACK_SPLINE_COUNT; ++i) {
        float next_s = (i == ZERO_TRACK_SPLINE_COUNT - 1) ? s_total_circuit_length : s_arc_lengths[i + 1];
        if (s >= s_arc_lengths[i] && s <= next_s) {
            seg_idx = i;
            break;
        }
    }

    if (out_node_idx) *out_node_idx = seg_idx;

    int next_idx = (seg_idx + 1) % ZERO_TRACK_SPLINE_COUNT;
    float seg_len = s_segment_lengths[seg_idx];
    float u = (seg_len > 1e-4f) ? ((s - s_arc_lengths[seg_idx]) / seg_len) : 0.0f;
    if (u < 0.0f) u = 0.0f;
    if (u > 1.0f) u = 1.0f;

    Vec3 p0 = vec3(g_track_spline[seg_idx][0], 0.0f, g_track_spline[seg_idx][1]);
    Vec3 p1 = vec3(g_track_spline[next_idx][0], 0.0f, g_track_spline[next_idx][1]);
    Vec3 center = vec3_add(vec3_scale(p0, 1.0f - u), vec3_scale(p1, u));

    Vec3 n0 = s_normals[seg_idx];
    Vec3 n1 = s_normals[next_idx];
    Vec3 norm = vec3_normalize(vec3_add(vec3_scale(n0, 1.0f - u), vec3_scale(n1, u)));

    return vec3_add(center, vec3_scale(norm, lateral_offset));
}

// -----------------------------------------------------------------------------
// Spline Interface Implementation
// -----------------------------------------------------------------------------

float zero_track_get_spline_curvature(int node_idx) {
    init_spline_tables_once();
    int idx = ((node_idx % ZERO_TRACK_SPLINE_COUNT) + ZERO_TRACK_SPLINE_COUNT) % ZERO_TRACK_SPLINE_COUNT;
    return s_curvatures[idx];
}

float zero_track_get_max_corner_speed(int node_idx, float aggressiveness) {
    init_spline_tables_once();
    int idx = ((node_idx % ZERO_TRACK_SPLINE_COUNT) + ZERO_TRACK_SPLINE_COUNT) % ZERO_TRACK_SPLINE_COUNT;
    float speed = s_max_corner_speeds[idx] * aggressiveness;
    if (speed < 14.0f) speed = 14.0f;
    if (speed > 75.0f) speed = 75.0f;
    return speed;
}

void zero_track_find_nearest_segment(Vec3 pos, int *out_node_idx, float *out_t, float *out_dist_sq) {
    init_spline_tables_once();

    int best_idx = 0;
    float best_t = 0.0f;
    float best_dist_sq = 1e30f;

    for (int i = 0; i < ZERO_TRACK_SPLINE_COUNT; ++i) {
        int next_i = (i + 1) % ZERO_TRACK_SPLINE_COUNT;
        float ax = g_track_spline[i][0];
        float az = g_track_spline[i][1];
        float bx = g_track_spline[next_i][0];
        float bz = g_track_spline[next_i][1];

        float abx = bx - ax;
        float abz = bz - az;
        float seg_len_sq = abx * abx + abz * abz;
        if (seg_len_sq < 1e-8f) continue;

        float apx = pos.x - ax;
        float apz = pos.z - az;
        float t = (apx * abx + apz * abz) / seg_len_sq;
        if (t < 0.0f) t = 0.0f;
        else if (t > 1.0f) t = 1.0f;

        float qx = ax + t * abx;
        float qz = az + t * abz;
        float dx = pos.x - qx;
        float dz = pos.z - qz;
        float dist_sq = dx * dx + dz * dz;

        if (dist_sq < best_dist_sq) {
            best_dist_sq = dist_sq;
            best_idx = i;
            best_t = t;
        }
    }

    if (out_node_idx) *out_node_idx = best_idx;
    if (out_t) *out_t = best_t;
    if (out_dist_sq) *out_dist_sq = best_dist_sq;
}

// -----------------------------------------------------------------------------
// Autonomous AI Opponent Controller
// -----------------------------------------------------------------------------

ZeroAIController zero_ai_controller_create(uint32_t car_index, float aggressiveness, float preferred_offset) {
    init_spline_tables_once();

    ZeroAIController ai;
    memset(&ai, 0, sizeof(ai));
    ai.car_index = car_index;
    ai.aggressiveness = aggressiveness;
    ai.preferred_lateral_offset = preferred_offset;
    ai.current_lateral_offset = preferred_offset;
    ai.lookahead_distance = 12.0f;
    ai.target_spline_node = 0;
    ai.avoidance_timer = 0.0f;
    ai.smooth_steer = 0.0f;
    ai.smooth_throttle = 0.0f;
    ai.smooth_brake = 0.0f;
    return ai;
}

void zero_ai_controller_update(ZeroAIController *ai, ZeroRaceGame *game, float dt) {
    if (!ai || !game || ai->car_index >= ZERO_RACE_CAR_COUNT || dt <= 0.0f) return;
    init_spline_tables_once();

    ZeroVehicleEnhanced *car = &game->cars[ai->car_index];
    float v = vec3_length(car->velocity);

    Mat4 rot_mat = quat_to_mat4(car->orientation);
    Vec3 car_right = vec3(rot_mat.m[0], rot_mat.m[1], rot_mat.m[2]);
    Vec3 car_fwd = vec3(rot_mat.m[8], rot_mat.m[9], rot_mat.m[10]);

    // 1. Forward-cone opponent detection & evasion
    bool opponent_in_cone = false;
    float closest_z = 100.0f;
    float obstacle_x = 0.0f;
    float obstacle_fwd_vel = 0.0f;

    for (uint32_t j = 0; j < ZERO_RACE_CAR_COUNT; ++j) {
        if (j == ai->car_index) continue;
        ZeroVehicleEnhanced *other = &game->cars[j];
        Vec3 d_rel = vec3_sub(other->position, car->position);
        float z_rel = vec3_dot(d_rel, car_fwd);
        float x_rel = vec3_dot(d_rel, car_right);

        if (z_rel > 0.5f && z_rel < 22.0f && fabsf(x_rel) < 3.2f) {
            if (z_rel < closest_z) {
                closest_z = z_rel;
                obstacle_x = x_rel;
                obstacle_fwd_vel = vec3_dot(other->velocity, car_fwd);
                opponent_in_cone = true;
            }
        }
    }

    float target_lateral_offset = ai->preferred_lateral_offset;
    float closing_brake = 0.0f;

    if (opponent_in_cone) {
        float delta_avoid = (obstacle_x <= 0.0f) ? 2.5f : -2.5f;
        target_lateral_offset = ai->preferred_lateral_offset + delta_avoid;
        if (target_lateral_offset < -5.5f) target_lateral_offset = -5.5f;
        if (target_lateral_offset > 5.5f) target_lateral_offset = 5.5f;
        ai->avoidance_timer = 1.2f;

        float car_fwd_vel = vec3_dot(car->velocity, car_fwd);
        if (obstacle_fwd_vel < car_fwd_vel && closest_z < 8.0f) {
            float closing_ratio = (8.0f - closest_z) / 5.0f;
            if (closing_ratio < 0.0f) closing_ratio = 0.0f;
            if (closing_ratio > 1.0f) closing_ratio = 1.0f;
            closing_brake = closing_ratio * 0.85f;
        }
    } else {
        if (ai->avoidance_timer > 0.0f) {
            ai->avoidance_timer -= dt;
        } else {
            target_lateral_offset = ai->preferred_lateral_offset;
        }
    }

    float offset_blend_rate = 4.0f;
    ai->current_lateral_offset += (target_lateral_offset - ai->current_lateral_offset) * fminf(1.0f, dt * offset_blend_rate);

    // 2. Pure pursuit steering controller
    int seg_idx = 0;
    float seg_t = 0.0f;
    float dist_sq = 0.0f;
    zero_track_find_nearest_segment(car->position, &seg_idx, &seg_t, &dist_sq);

    float s_car = s_arc_lengths[seg_idx] + seg_t * s_segment_lengths[seg_idx];
    float l_pursuit = 9.0f + 0.35f * v;
    ai->lookahead_distance = l_pursuit;

    float s_aim = s_car + l_pursuit;
    if (s_aim >= s_total_circuit_length) s_aim -= s_total_circuit_length;

    Vec3 aim_point = evaluate_spline_aim_point(s_aim, ai->current_lateral_offset, &ai->target_spline_node);

    Vec3 delta_aim = vec3_sub(aim_point, car->position);
    float x_local = vec3_dot(delta_aim, car_right);
    float z_local = vec3_dot(delta_aim, car_fwd);
    if (z_local < 0.2f) z_local = 0.2f;

    float l_actual_sq = x_local * x_local + z_local * z_local;
    const float l_wb = 2.90f;
    float delta_steer = atan2f(2.0f * x_local * l_wb, l_actual_sq);

    const float delta_max = ZERO_DEG2RAD(28.0f);
    float raw_steer_input = delta_steer / delta_max;
    if (raw_steer_input < -1.0f) raw_steer_input = -1.0f;
    if (raw_steer_input > 1.0f) raw_steer_input = 1.0f;

    ai->smooth_steer += (raw_steer_input - ai->smooth_steer) * fminf(1.0f, dt * 14.0f);
    car->steer_input = ai->smooth_steer;

    // 3. Anticipatory corner braking
    const float a_brake = 8.5f;
    float d_brake_lookahead = 20.0f + (v * v) / (2.0f * a_brake) + 1.0f * v;
    float v_min_allowable = 75.0f;

    int node_cursor = seg_idx;
    float dist_accum = (1.0f - seg_t) * s_segment_lengths[seg_idx];
    while (dist_accum <= d_brake_lookahead) {
        int next_node = (node_cursor + 1) % ZERO_TRACK_SPLINE_COUNT;
        float v_target_node = zero_track_get_max_corner_speed(next_node, ai->aggressiveness);
        float v_allow = sqrtf(v_target_node * v_target_node + 2.0f * a_brake * dist_accum);
        if (v_allow < v_min_allowable) {
            v_min_allowable = v_allow;
        }
        dist_accum += s_segment_lengths[next_node];
        node_cursor = next_node;
    }

    float v_local_target = zero_track_get_max_corner_speed(seg_idx, ai->aggressiveness);
    float v_desired = fminf(v_min_allowable, v_local_target);

    float raw_throttle = 0.0f;
    float raw_brake = 0.0f;
    if (v > v_desired) {
        raw_throttle = 0.0f;
        raw_brake = (v - v_desired) / 3.0f;
        if (raw_brake < 0.2f) raw_brake = 0.2f;
        if (raw_brake > 1.0f) raw_brake = 1.0f;
    } else {
        raw_brake = 0.0f;
        float v_diff = v_desired - v;
        raw_throttle = v_diff / 2.0f;
        if (raw_throttle < 0.25f) raw_throttle = 0.25f;
        if (raw_throttle > 1.0f) raw_throttle = 1.0f;
    }

    if (closing_brake > raw_brake) {
        raw_brake = closing_brake;
        raw_throttle = 0.0f;
    }

    ai->smooth_throttle += (raw_throttle - ai->smooth_throttle) * fminf(1.0f, dt * 10.0f);
    ai->smooth_brake += (raw_brake - ai->smooth_brake) * fminf(1.0f, dt * 14.0f);
    car->throttle_input = ai->smooth_throttle;
    car->brake_input = ai->smooth_brake;
    car->handbrake = false;
}

// -----------------------------------------------------------------------------
// Car-to-Car OBB Collision Resolution (3D OBB Separating Axis Theorem)
// -----------------------------------------------------------------------------

static bool test_obb_sat(Vec3 posA, Quat rotA, Vec3 posB, Quat rotB,
                         Vec3 half_extents, Vec3 *out_normal, float *out_depth, Vec3 *out_contact) {
    Mat4 rot_mat_A = quat_to_mat4(rotA);
    Vec3 uA[3] = {
        vec3(rot_mat_A.m[0], rot_mat_A.m[1], rot_mat_A.m[2]),
        vec3(rot_mat_A.m[4], rot_mat_A.m[5], rot_mat_A.m[6]),
        vec3(rot_mat_A.m[8], rot_mat_A.m[9], rot_mat_A.m[10])
    };

    Mat4 rot_mat_B = quat_to_mat4(rotB);
    Vec3 uB[3] = {
        vec3(rot_mat_B.m[0], rot_mat_B.m[1], rot_mat_B.m[2]),
        vec3(rot_mat_B.m[4], rot_mat_B.m[5], rot_mat_B.m[6]),
        vec3(rot_mat_B.m[8], rot_mat_B.m[9], rot_mat_B.m[10])
    };

    Vec3 d = vec3_sub(posB, posA);
    float eA[3] = {half_extents.x, half_extents.y, half_extents.z};
    float eB[3] = {half_extents.x, half_extents.y, half_extents.z};

    Vec3 axes[15];
    int axis_count = 0;

    // 3 face normals of A
    axes[axis_count++] = uA[0];
    axes[axis_count++] = uA[1];
    axes[axis_count++] = uA[2];

    // 3 face normals of B
    axes[axis_count++] = uB[0];
    axes[axis_count++] = uB[1];
    axes[axis_count++] = uB[2];

    // 9 edge cross-product axes
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            Vec3 cross = vec3_cross(uA[i], uB[j]);
            if (vec3_length_sq(cross) > 1e-4f) {
                axes[axis_count++] = vec3_normalize(cross);
            }
        }
    }

    float min_penetration = 1e30f;
    Vec3 best_normal = vec3(0.0f, 1.0f, 0.0f);

    for (int k = 0; k < axis_count; ++k) {
        Vec3 axis = vec3_normalize(axes[k]);
        float len_sq = vec3_length_sq(axis);
        if (len_sq < 1e-4f) continue;

        float dist_proj = fabsf(vec3_dot(d, axis));

        float rA = eA[0] * fabsf(vec3_dot(uA[0], axis)) +
                   eA[1] * fabsf(vec3_dot(uA[1], axis)) +
                   eA[2] * fabsf(vec3_dot(uA[2], axis));

        float rB = eB[0] * fabsf(vec3_dot(uB[0], axis)) +
                   eB[1] * fabsf(vec3_dot(uB[1], axis)) +
                   eB[2] * fabsf(vec3_dot(uB[2], axis));

        float overlap = (rA + rB) - dist_proj;
        if (overlap <= 0.0f) {
            return false; // Separating axis exists: no collision
        }

        if (overlap < min_penetration) {
            min_penetration = overlap;
            best_normal = axis;
        }
    }

    // Ensure normal points from A to B
    if (vec3_dot(best_normal, d) < 0.0f) {
        best_normal = vec3_scale(best_normal, -1.0f);
    }

    *out_normal = best_normal;
    *out_depth = min_penetration;

    // Contact point estimation: support points along collision normal
    Vec3 ptA = posA;
    Vec3 ptB = posB;
    for (int i = 0; i < 3; ++i) {
        float signA = (vec3_dot(uA[i], best_normal) >= 0.0f) ? 1.0f : -1.0f;
        ptA = vec3_add(ptA, vec3_scale(uA[i], signA * eA[i]));

        float signB = (vec3_dot(uB[i], best_normal) <= 0.0f) ? 1.0f : -1.0f;
        ptB = vec3_add(ptB, vec3_scale(uB[i], signB * eB[i]));
    }
    *out_contact = vec3_scale(vec3_add(ptA, ptB), 0.5f);

    return true;
}

void zero_race_game_resolve_car_collisions(ZeroRaceGame *game) {
    if (!game) return;

    game->collision_event_count = 0;
    const Vec3 box_half_extents = vec3(0.95f, 0.55f, 2.30f);

    for (int i = 0; i < ZERO_RACE_CAR_COUNT - 1; ++i) {
        for (int j = i + 1; j < ZERO_RACE_CAR_COUNT; ++j) {
            ZeroVehicleEnhanced *carA = &game->cars[i];
            ZeroVehicleEnhanced *carB = &game->cars[j];

            Vec3 diff = vec3_sub(carB->position, carA->position);
            float dist_sq = vec3_length_sq(diff);
            if (dist_sq > 28.0f) continue; // Early-out bounding sphere check

            Vec3 normal;
            float depth = 0.0f;
            Vec3 contact_point;

            bool hit = test_obb_sat(carA->position, carA->orientation,
                                    carB->position, carB->orientation,
                                    box_half_extents, &normal, &depth, &contact_point);
            if (!hit || depth <= 1e-4f) continue;

            // Symmetrical position separation
            Vec3 sep = vec3_scale(normal, depth * 0.52f);
            carA->position = vec3_sub(carA->position, sep);
            carB->position = vec3_add(carB->position, sep);

            // Symmetrical impulse resolution
            Vec3 rA = vec3_sub(contact_point, carA->position);
            Vec3 rB = vec3_sub(contact_point, carB->position);

            Vec3 vA_cp = vec3_add(carA->velocity, vec3_cross(carA->angular_velocity, rA));
            Vec3 vB_cp = vec3_add(carB->velocity, vec3_cross(carB->angular_velocity, rB));
            Vec3 v_rel = vec3_sub(vB_cp, vA_cp);
            float vn = vec3_dot(v_rel, normal);

            if (vn < 0.0f) {
                Vec3 termA = vec3_cross(rA, normal);
                Vec3 termB = vec3_cross(rB, normal);
                Vec3 inv_I_A_term = apply_world_inv_inertia(carA->orientation, carA->inv_inertia, termA);
                Vec3 inv_I_B_term = apply_world_inv_inertia(carB->orientation, carB->inv_inertia, termB);

                float angA = vec3_dot(termA, inv_I_A_term);
                float angB = vec3_dot(termB, inv_I_B_term);

                float inv_mA = (carA->mass > 1.0f) ? (1.0f / carA->mass) : (1.0f / 1150.0f);
                float inv_mB = (carB->mass > 1.0f) ? (1.0f / carB->mass) : (1.0f / 1150.0f);

                float K = inv_mA + inv_mB + angA + angB;
                if (K < 1e-6f) K = inv_mA + inv_mB;

                float v_impact = fabsf(vn);
                float restitution = 0.45f + 0.35f * fminf(1.0f, v_impact / 18.0f);
                float speed_boost = (v_impact > 6.0f) ? 1.35f : 1.0f;
                float J = -(1.0f + restitution) * vn / K * speed_boost;
                if (J < 0.0f) J = 0.0f;

                Vec3 impulse = vec3_scale(normal, J);

                carA->velocity = vec3_sub(carA->velocity, vec3_scale(impulse, inv_mA));
                carB->velocity = vec3_add(carB->velocity, vec3_scale(impulse, inv_mB));

                // Vertical kinetic bounce only on severe high-speed impact (max 0.85 m/s)
                if (v_impact > 15.0f && carA->velocity.y < 0.6f) {
                    float v_pop = fminf(0.85f, 0.52f + (v_impact - 15.0f) * 0.015f);
                    carA->velocity.y = fmaxf(carA->velocity.y, v_pop);
                    carB->velocity.y = fmaxf(carB->velocity.y, v_pop);
                }

                // Pure yaw torque on collisions: realistic horizontal spinout without barrel roll/flipping
                Vec3 torqueA = vec3_cross(rA, vec3_scale(impulse, -1.0f));
                Vec3 torqueB = vec3_cross(rB, impulse);
                torqueA.x = 0.0f;
                torqueA.z = 0.0f;
                torqueB.x = 0.0f;
                torqueB.z = 0.0f;
                torqueA.y = fmaxf(-3500.0f, fminf(3500.0f, torqueA.y));
                torqueB.y = fmaxf(-3500.0f, fminf(3500.0f, torqueB.y));
                carA->angular_velocity = vec3_add(carA->angular_velocity, apply_world_inv_inertia(carA->orientation, carA->inv_inertia, torqueA));
                carB->angular_velocity = vec3_add(carB->angular_velocity, apply_world_inv_inertia(carB->orientation, carB->inv_inertia, torqueB));

                // Dynamic friction impulse with spin torque
                Vec3 vt = vec3_sub(v_rel, vec3_scale(normal, vn));
                float vt_len = vec3_length(vt);
                if (vt_len > 1e-4f) {
                    Vec3 tangent = vec3_scale(vt, 1.0f / vt_len);
                    float Jt = -vt_len / K;
                    float max_Jt = 0.40f * J;
                    if (Jt < -max_Jt) Jt = -max_Jt;
                    if (Jt > max_Jt) Jt = max_Jt;
                    Vec3 friction_impulse = vec3_scale(tangent, Jt);

                    carA->velocity = vec3_sub(carA->velocity, vec3_scale(friction_impulse, inv_mA));
                    carB->velocity = vec3_add(carB->velocity, vec3_scale(friction_impulse, inv_mB));

                    Vec3 f_torqueA = vec3_cross(rA, vec3_scale(friction_impulse, -1.0f));
                    Vec3 f_torqueB = vec3_cross(rB, friction_impulse);
                    f_torqueA.x = 0.0f;
                    f_torqueA.z = 0.0f;
                    f_torqueB.x = 0.0f;
                    f_torqueB.z = 0.0f;
                    carA->angular_velocity = vec3_add(carA->angular_velocity, apply_world_inv_inertia(carA->orientation, carA->inv_inertia, f_torqueA));
                    carB->angular_velocity = vec3_add(carB->angular_velocity, apply_world_inv_inertia(carB->orientation, carB->inv_inertia, f_torqueB));
                }

                // Log collision event for debris, particles, track rupture, and sound
                if (game->collision_event_count < ZERO_MAX_COLLISION_EVENTS) {
                    ZeroCollisionEvent *ev = &game->collision_events[game->collision_event_count++];
                    ev->carA_index = i;
                    ev->carB_index = j;
                    ev->contact_point = contact_point;
                    ev->normal = normal;
                    ev->impulse_magnitude = J;
                    ev->relative_speed = v_impact;
                    ev->is_severe = (v_impact > 7.5f || J > 9000.0f);
                }
            }
        }
    }
}

// -----------------------------------------------------------------------------
// Multi-Car Simulation & Game Logic
// -----------------------------------------------------------------------------

static float default_track_ground_fn(float x, float z, Vec3 *out_normal) {
    (void)x; (void)z;
    if (out_normal) {
        *out_normal = vec3(0.0f, 1.0f, 0.0f);
    }
    return 0.0f;
}

ZeroRaceGame zero_race_game_create(void) {
    init_spline_tables_once();

    ZeroRaceGame game;
    memset(&game, 0, sizeof(game));

    game.tick = 0;
    game.race_time = 0.0f;
    game.race_started = true;
    game.race_finished = false;
    game.total_laps = 3;
    game.winning_car_index = 0;

    // Grid Positions along starting straight (heading +Z)
    Vec3 spawn_positions[ZERO_RACE_CAR_COUNT] = {
        vec3(124.0f, 0.75f, 0.0f),    // Car 0: Player
        vec3(126.2f, 0.75f, -8.0f),   // Car 1: AI 1 ("Apex Predator")
        vec3(123.8f, 0.75f, -16.0f),  // Car 2: AI 2 ("Viper")
        vec3(126.2f, 0.75f, -24.0f)   // Car 3: AI 3 ("Phantom")
    };

    Quat spawn_rot = quat_identity();

    for (int k = 0; k < ZERO_RACE_CAR_COUNT; ++k) {
        game.cars[k] = zero_vehicle_enhanced_create(spawn_positions[k], spawn_rot);
        game.cars[k].race_position = k + 1;
        game.cars[k].lap_count = 0;
        game.cars[k].current_lap_time = 0.0f;
        game.cars[k].best_lap_time = 0.0f;

        int seg = 0;
        float t = 0.0f;
        float dist_sq = 0.0f;
        zero_track_find_nearest_segment(spawn_positions[k], &seg, &t, &dist_sq);
        game.cars[k].track_progress_s = s_arc_lengths[seg] + t * s_segment_lengths[seg];
    }

    game.ai_controllers[0] = zero_ai_controller_create(1, 1.03f, -1.8f);
    game.ai_controllers[1] = zero_ai_controller_create(2, 0.96f, 0.0f);
    game.ai_controllers[2] = zero_ai_controller_create(3, 0.90f, 1.8f);

    return game;
}

void zero_race_game_update_rankings(ZeroRaceGame *game) {
    if (!game) return;
    init_spline_tables_once();

    float score[ZERO_RACE_CAR_COUNT];
    for (int k = 0; k < ZERO_RACE_CAR_COUNT; ++k) {
        score[k] = (float)game->cars[k].lap_count * s_total_circuit_length + game->cars[k].track_progress_s;
    }

    for (int k = 0; k < ZERO_RACE_CAR_COUNT; ++k) {
        int rank = 1;
        for (int j = 0; j < ZERO_RACE_CAR_COUNT; ++j) {
            if (j == k) continue;
            if (score[j] > score[k] || (fabsf(score[j] - score[k]) < 1e-4f && j < k)) {
                rank++;
            }
        }
        game->cars[k].race_position = rank;
        if (rank == 1) {
            if (game->cars[k].lap_count >= game->total_laps && !game->race_finished) {
                game->race_finished = true;
                game->winning_car_index = (uint32_t)k;
            }
        }
    }
}

void zero_race_game_step(ZeroRaceGame *game, float dt) {
    if (!game || dt <= 0.0f) return;
    init_spline_tables_once();

    // 1. Advance AI controllers
    if (game->race_started && !game->race_finished) {
        for (int i = 0; i < 3; ++i) {
            zero_ai_controller_update(&game->ai_controllers[i], game, dt);
        }
    }

    // 2. Advance vehicle physics
    for (int k = 0; k < ZERO_RACE_CAR_COUNT; ++k) {
        zero_vehicle_enhanced_update(&game->cars[k], dt, default_track_ground_fn);
    }

    // 3. Resolve car-to-car collisions
    zero_race_game_resolve_car_collisions(game);

    // 4. Update track progress and lap counters
    for (int k = 0; k < ZERO_RACE_CAR_COUNT; ++k) {
        ZeroVehicleEnhanced *car = &game->cars[k];
        if (game->race_started) {
            car->current_lap_time += dt;
        }

        int seg_idx = 0;
        float seg_t = 0.0f;
        float dist_sq = 0.0f;
        zero_track_find_nearest_segment(car->position, &seg_idx, &seg_t, &dist_sq);

        float new_progress = s_arc_lengths[seg_idx] + seg_t * s_segment_lengths[seg_idx];
        float old_progress = car->track_progress_s;

        // Lap completed when transitioning from end of track (> 70%) to start (< 30%)
        if (old_progress > 0.70f * s_total_circuit_length && new_progress < 0.30f * s_total_circuit_length) {
            car->lap_count++;
            if (car->current_lap_time > 0.0f) {
                if (car->best_lap_time <= 0.0f || car->current_lap_time < car->best_lap_time) {
                    car->best_lap_time = car->current_lap_time;
                }
            }
            car->current_lap_time = 0.0f;
        }
        car->track_progress_s = new_progress;
    }

    // 5. Update rankings
    zero_race_game_update_rankings(game);

    // 6. Tick & race timer
    game->tick++;
    if (game->race_started) {
        game->race_time += dt;
    }
}

// -----------------------------------------------------------------------------
// Weak Fallback Implementation of Vehicle Physics (overridden by M2 when linked)
// -----------------------------------------------------------------------------

__attribute__((weak)) ZeroVehicleEnhanced zero_vehicle_enhanced_create(Vec3 spawn_pos, Quat spawn_rot) {
    ZeroVehicleEnhanced car;
    memset(&car, 0, sizeof(car));

    car.position = spawn_pos;
    float q_mag = quat_dot(spawn_rot, spawn_rot);
    car.orientation = (q_mag > 1e-4f) ? quat_normalize(spawn_rot) : quat_identity();
    car.transform = mat4_identity();

    car.mass = 1150.0f;
    car.inertia = vec3(1400.0f, 950.0f, 750.0f);
    car.inv_inertia = vec3(1.0f / 1400.0f, 1.0f / 950.0f, 1.0f / 750.0f);

    float half_width_f = 0.82f;
    float half_width_r = 0.84f;
    float front_z = 1.35f;
    float rear_z = -1.30f;

    // Wheel 0: FL, Wheel 1: FR, Wheel 2: RL, Wheel 3: RR
    car.wheels[0].hardpoint = vec3(-half_width_f, 0.0f, front_z);
    car.wheels[0].is_front = true;
    car.wheels[0].is_drive = false;

    car.wheels[1].hardpoint = vec3(half_width_f, 0.0f, front_z);
    car.wheels[1].is_front = true;
    car.wheels[1].is_drive = false;

    car.wheels[2].hardpoint = vec3(-half_width_r, 0.0f, rear_z);
    car.wheels[2].is_front = false;
    car.wheels[2].is_drive = true;

    car.wheels[3].hardpoint = vec3(half_width_r, 0.0f, rear_z);
    car.wheels[3].is_front = false;
    car.wheels[3].is_drive = true;

    for (int i = 0; i < 4; ++i) {
        car.wheels[i].rest_length = 0.55f;
        car.wheels[i].spring_stiffness = 44000.0f;
        car.wheels[i].damping_rate = 3600.0f;
        car.wheels[i].radius = 0.37f;
        car.wheels[i].current_length = 0.55f;
    }

    car.anti_roll_stiffness_front = 8500.0f;
    car.anti_roll_stiffness_rear = 5200.0f;

    car.transmission.current_gear = ZERO_GEAR_1;
    car.transmission.target_gear = ZERO_GEAR_1;
    car.transmission.gear_ratios[0] = -3.40f;
    car.transmission.gear_ratios[1] = 3.60f;
    car.transmission.gear_ratios[2] = 2.19f;
    car.transmission.gear_ratios[3] = 1.52f;
    car.transmission.gear_ratios[4] = 1.18f;
    car.transmission.gear_ratios[5] = 0.95f;
    car.transmission.gear_ratios[6] = 0.78f;
    car.transmission.final_drive_ratio = 3.73f;
    car.transmission.torque_cut_factor = 1.0f;

    car.engine_rpm = 1000.0f;

    car.aero_drag_coeff = 0.32f * 2.2f;
    car.aero_front_downforce = 1.15f;
    car.aero_rear_downforce = 1.65f;
    car.aero_diffuser_factor = 0.85f;
    car.aero_airbrake_active = 0.0f;

    return car;
}

__attribute__((weak)) void zero_vehicle_enhanced_update(ZeroVehicleEnhanced *car, float dt, ZeroGroundHeightFn ground_fn) {
    if (!car || dt <= 0.0f) return;

    int sub_steps = 4;
    float sub_dt = dt / (float)sub_steps;

    for (int step = 0; step < sub_steps; ++step) {
        Mat4 rot_mat = quat_to_mat4(car->orientation);
        Vec3 up = vec3(rot_mat.m[4], rot_mat.m[5], rot_mat.m[6]);

        Vec3 total_force = vec3(0.0f, -9.81f * car->mass, 0.0f);
        Vec3 total_torque = vec3(0.0f, 0.0f, 0.0f);

        float speed_kmh_approx = vec3_length(car->velocity) * 3.6f;
        float speed_factor = speed_kmh_approx / 120.0f;
        if (speed_factor > 1.0f) speed_factor = 1.0f;
        float lock_degrees = 24.0f - speed_factor * 12.0f;
        float max_steer = ZERO_DEG2RAD(lock_degrees);
        float current_steer = car->steer_input * max_steer;

        float susp_travel[4] = {0};

        for (int i = 0; i < 4; ++i) {
            ZeroWheel *w = &car->wheels[i];
            w->steer_angle = w->is_front ? current_steer : 0.0f;

            Vec3 r_w = rotate_vector_by_quat(car->orientation, w->hardpoint);
            Vec3 hardpoint_world = vec3_add(car->position, r_w);

            Vec3 ground_norm = vec3(0.0f, 1.0f, 0.0f);
            float ground_y = 0.0f;
            if (ground_fn) {
                ground_y = ground_fn(hardpoint_world.x, hardpoint_world.z, &ground_norm);
            }

            float ray_len = w->rest_length + w->radius;
            float contact_dist = hardpoint_world.y - ground_y;

            if (contact_dist < ray_len) {
                w->is_grounded = true;
                w->current_length = contact_dist - w->radius;
                if (w->current_length < 0.05f) w->current_length = 0.05f;

                float compression = w->rest_length - w->current_length;
                susp_travel[i] = compression;

                Vec3 point_vel = vec3_add(car->velocity, vec3_cross(car->angular_velocity, r_w));
                float susp_vel = vec3_dot(point_vel, up);

                float spring_f = w->spring_stiffness * compression;
                float damper_f = -w->damping_rate * susp_vel;
                float normal_f = spring_f + damper_f;
                if (normal_f < 0.0f) normal_f = 0.0f;

                Quat steer_q = quat_from_axis_angle(vec3(0.0f, 1.0f, 0.0f), w->steer_angle);
                Quat wheel_q = quat_mul(car->orientation, steer_q);
                Mat4 wheel_rot = quat_to_mat4(wheel_q);
                Vec3 wheel_fwd = vec3(wheel_rot.m[8], wheel_rot.m[9], wheel_rot.m[10]);
                Vec3 wheel_right = vec3(wheel_rot.m[0], wheel_rot.m[1], wheel_rot.m[2]);

                float v_lat = vec3_dot(point_vel, wheel_right);
                float v_long = vec3_dot(point_vel, wheel_fwd);

                float cornering_stiff = w->is_front ? 52000.0f : 48000.0f;
                if (car->handbrake && !w->is_front) {
                    cornering_stiff = 5500.0f;
                }
                float lat_force_mag = -v_lat * cornering_stiff;
                float max_lat = normal_f * 2.45f;
                if (lat_force_mag > max_lat) lat_force_mag = max_lat;
                if (lat_force_mag < -max_lat) lat_force_mag = -max_lat;

                float drive_f = 0.0f;
                float brake_f = 0.0f;
                if (w->is_drive) {
                    drive_f = car->throttle_input * 4200.0f;
                }
                if (car->brake_input > 0.01f) {
                    if (v_long > 0.1f) {
                        brake_f = -car->brake_input * 6000.0f;
                    } else if (v_long < -0.1f) {
                        brake_f = car->brake_input * 6000.0f;
                    }
                }
                if (car->transmission.current_gear == ZERO_GEAR_REVERSE && w->is_drive) {
                    drive_f = -car->throttle_input * 3500.0f;
                }
                if (car->handbrake && !w->is_front) {
                    brake_f -= (v_long > 0.1f ? 1.0f : (v_long < -0.1f ? -1.0f : 0.0f)) * 8000.0f;
                }

                Vec3 f_wheel = vec3_add(
                    vec3_scale(up, normal_f),
                    vec3_add(vec3_scale(wheel_right, lat_force_mag), vec3_scale(wheel_fwd, drive_f + brake_f))
                );

                total_force = vec3_add(total_force, f_wheel);
                total_torque = vec3_add(total_torque, vec3_cross(r_w, f_wheel));
            } else {
                w->is_grounded = false;
                w->current_length = w->rest_length;
            }
        }

        // Anti-roll bar torque coupling
        float arb_front_force = car->anti_roll_stiffness_front * (susp_travel[0] - susp_travel[1]);
        float arb_rear_force = car->anti_roll_stiffness_rear * (susp_travel[2] - susp_travel[3]);
        Vec3 arb_front_r = rotate_vector_by_quat(car->orientation, vec3(0.0f, 0.0f, 1.35f));
        Vec3 arb_rear_r = rotate_vector_by_quat(car->orientation, vec3(0.0f, 0.0f, -1.30f));
        total_torque = vec3_add(total_torque, vec3_cross(arb_front_r, vec3_scale(up, arb_front_force)));
        total_torque = vec3_add(total_torque, vec3_cross(arb_rear_r, vec3_scale(up, arb_rear_force)));

        // Aerodynamics
        float speed = vec3_length(car->velocity);
        float rho = 1.225f;
        float dynamic_pressure = 0.5f * rho * speed * speed;

        float target_airbrake = (car->brake_input > 0.3f && speed > 11.0f) ? 1.0f : 0.0f;
        car->aero_airbrake_active += (target_airbrake - car->aero_airbrake_active) * sub_dt * 6.0f;

        float effective_drag_coeff = car->aero_drag_coeff * (1.0f + 0.85f * car->aero_airbrake_active);
        if (speed > 1e-4f) {
            Vec3 drag = vec3_scale(car->velocity, -0.5f * rho * effective_drag_coeff * speed);
            total_force = vec3_add(total_force, drag);
        }

        float f_downforce_front = dynamic_pressure * car->aero_front_downforce;
        Vec3 front_aero_pos = rotate_vector_by_quat(car->orientation, vec3(0.0f, 0.0f, 1.35f));
        Vec3 front_aero_force = vec3_scale(up, -f_downforce_front);
        total_force = vec3_add(total_force, front_aero_force);
        total_torque = vec3_add(total_torque, vec3_cross(front_aero_pos, front_aero_force));

        float f_downforce_rear = dynamic_pressure * car->aero_rear_downforce * (1.0f + 0.65f * car->aero_airbrake_active);
        Vec3 rear_aero_pos = rotate_vector_by_quat(car->orientation, vec3(0.0f, 0.45f, -1.30f));
        Vec3 rear_aero_force = vec3_scale(up, -f_downforce_rear);
        total_force = vec3_add(total_force, rear_aero_force);
        total_torque = vec3_add(total_torque, vec3_cross(rear_aero_pos, rear_aero_force));

        float avg_susp_h = 0.25f * (car->wheels[0].current_length + car->wheels[1].current_length +
                                   car->wheels[2].current_length + car->wheels[3].current_length);
        if (avg_susp_h < 0.08f) avg_susp_h = 0.08f;
        float ground_proximity = 0.35f / avg_susp_h;
        if (ground_proximity > 3.0f) ground_proximity = 3.0f;
        float f_diffuser = dynamic_pressure * car->aero_diffuser_factor * ground_proximity;
        total_force = vec3_add(total_force, vec3_scale(up, -f_diffuser));

        Vec3 aero_rot_damping = vec3_scale(car->angular_velocity, -(280.0f + 15.0f * speed));
        total_torque = vec3_add(total_torque, aero_rot_damping);

        // Integration
        Vec3 accel = vec3_scale(total_force, 1.0f / car->mass);
        car->velocity = vec3_add(car->velocity, vec3_scale(accel, sub_dt));

        Vec3 ang_accel = apply_world_inv_inertia(car->orientation, car->inv_inertia, total_torque);
        car->angular_velocity = vec3_add(car->angular_velocity, vec3_scale(ang_accel, sub_dt));

        car->position = vec3_add(car->position, vec3_scale(car->velocity, sub_dt));

        Quat w_quat = {car->angular_velocity.x, car->angular_velocity.y, car->angular_velocity.z, 0.0f};
        Quat dq = quat_mul(w_quat, car->orientation);
        car->orientation.x += 0.5f * dq.x * sub_dt;
        car->orientation.y += 0.5f * dq.y * sub_dt;
        car->orientation.z += 0.5f * dq.z * sub_dt;
        car->orientation.w += 0.5f * dq.w * sub_dt;
        car->orientation = quat_normalize(car->orientation);

        // Track Spline Barrier Reflection
        int near_idx = 0;
        float near_t = 0.0f;
        float near_dist_sq = 0.0f;
        zero_track_find_nearest_segment(car->position, &near_idx, &near_t, &near_dist_sq);
        float dist_to_center = sqrtf(near_dist_sq);
        const float barrier_radius = ZERO_TRACK_BARRIER_DIST - 1.2f;

        if (dist_to_center > barrier_radius) {
            int next_idx = (near_idx + 1) % ZERO_TRACK_SPLINE_COUNT;
            float qx = g_track_spline[near_idx][0] + near_t * (g_track_spline[next_idx][0] - g_track_spline[near_idx][0]);
            float qz = g_track_spline[near_idx][1] + near_t * (g_track_spline[next_idx][1] - g_track_spline[near_idx][1]);
            Vec3 to_car = vec3(car->position.x - qx, 0.0f, car->position.z - qz);
            Vec3 barrier_normal = vec3_normalize(to_car);

            car->position.x = qx + barrier_normal.x * barrier_radius;
            car->position.z = qz + barrier_normal.z * barrier_radius;

            float vn = vec3_dot(car->velocity, barrier_normal);
            if (vn > 0.0f) {
                Vec3 v_norm = vec3_scale(barrier_normal, vn);
                Vec3 v_tan = vec3_sub(car->velocity, v_norm);
                car->velocity = vec3_add(vec3_scale(v_tan, 0.70f), vec3_scale(v_norm, -0.20f));
            }
        }
    }

    // Telemetry updates
    car->speed_kmh = vec3_length(car->velocity) * 3.6f;
    Mat4 tr = mat4_translate(car->position);
    Mat4 rot = quat_to_mat4(car->orientation);
    car->transform = mat4_mul(tr, rot);

    // Automatic transmission simulation
    if (car->speed_kmh < 35.0f) car->transmission.current_gear = ZERO_GEAR_1;
    else if (car->speed_kmh < 70.0f) car->transmission.current_gear = ZERO_GEAR_2;
    else if (car->speed_kmh < 110.0f) car->transmission.current_gear = ZERO_GEAR_3;
    else if (car->speed_kmh < 160.0f) car->transmission.current_gear = ZERO_GEAR_4;
    else if (car->speed_kmh < 210.0f) car->transmission.current_gear = ZERO_GEAR_5;
    else car->transmission.current_gear = ZERO_GEAR_6;

    float gear_ratio = car->transmission.gear_ratios[(int)car->transmission.current_gear];
    float wheel_rpm = (car->speed_kmh / (3.6f * 0.37f * 2.0f * ZERO_PI)) * 60.0f;
    car->engine_rpm = 1000.0f + fabsf(wheel_rpm * gear_ratio * car->transmission.final_drive_ratio);
    if (car->engine_rpm > 7500.0f) car->engine_rpm = 7500.0f;

    Vec3 fwd = vec3(car->transform.m[8], car->transform.m[9], car->transform.m[10]);
    float forward_speed = vec3_dot(car->velocity, fwd);
    float lateral_speed = vec3_dot(car->velocity, vec3(car->transform.m[0], car->transform.m[1], car->transform.m[2]));
    car->drift_angle = (fabsf(forward_speed) > 1.0f) ? atan2f(lateral_speed, fabsf(forward_speed)) : 0.0f;
    car->is_drifting = (fabsf(car->drift_angle) > ZERO_DEG2RAD(10.0f) && car->speed_kmh > 20.0f);
}

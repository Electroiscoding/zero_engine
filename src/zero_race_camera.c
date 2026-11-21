#include "zero_race_camera.h"
#include "zero_track_spline.h"
#include <math.h>
#include <string.h>

#define ZERO_TV_TOWER_COUNT 8
static const int g_tv_tower_indices[ZERO_TV_TOWER_COUNT] = {0, 15, 30, 45, 60, 75, 90, 105};

static Vec3 zero_race_camera_tv_tower_pos(int tower_idx) {
    if (tower_idx < 0 || tower_idx >= ZERO_TV_TOWER_COUNT) {
        tower_idx = 0;
    }
    int node = g_tv_tower_indices[tower_idx];
    int next = (node + 1) % ZERO_TRACK_SPLINE_COUNT;
    int prev = (node - 1 + ZERO_TRACK_SPLINE_COUNT) % ZERO_TRACK_SPLINE_COUNT;

    float tx = g_track_spline[next][0] - g_track_spline[prev][0];
    float tz = g_track_spline[next][1] - g_track_spline[prev][1];
    float t_len = sqrtf(tx * tx + tz * tz);
    if (t_len > 1e-6f) {
        tx /= t_len;
        tz /= t_len;
    }
    /* Left normal perpendicular to track tangent */
    float nx = -tz;
    float nz = tx;

    float px = g_track_spline[node][0] + nx * 22.0f;
    float py = 8.5f;
    float pz = g_track_spline[node][1] + nz * 22.0f;
    return vec3(px, py, pz);
}

ZeroRaceCamera zero_race_camera_create(float aspect) {
    ZeroRaceCamera rcam;
    memset(&rcam, 0, sizeof(rcam));

    if (aspect <= 0.001f) {
        aspect = 16.0f / 9.0f;
    }

    rcam.mode = ZERO_CAM_CHASE;
    rcam.base_fov_deg = 62.0f;
    rcam.current_fov_deg = 62.0f;
    rcam.fov_dilation_rate = 1.8f;
    rcam.spring_stiffness = 45.0f;
    rcam.damping_ratio = 1.0f;
    rcam.transition_t = 1.0f;
    rcam.active_tv_tower_idx = 0;
    rcam.accum_time = 0.0f;
    rcam.prev_car_velocity = vec3(0.0f, 0.0f, 0.0f);

    Vec3 init_pos = vec3(125.0f, 2.4f, -6.8f);
    Vec3 init_target = vec3(125.0f, 1.1f, 3.5f);

    rcam.base_cam = zero_camera_create(init_pos, ZERO_DEG2RAD(62.0f), aspect, 0.1f, 1000.0f);
    rcam.current_pos = init_pos;
    rcam.current_velocity = vec3(0.0f, 0.0f, 0.0f);
    rcam.current_target = init_target;
    rcam.current_target_vel = vec3(0.0f, 0.0f, 0.0f);
    rcam.transition_start_pos = init_pos;
    rcam.transition_start_target = init_target;

    Vec3 fwd = vec3_normalize(vec3_sub(init_target, init_pos));
    Vec3 right = vec3_normalize(vec3_cross(fwd, vec3(0.0f, 1.0f, 0.0f)));
    Vec3 up = vec3_normalize(vec3_cross(right, fwd));
    rcam.base_cam.forward = fwd;
    rcam.base_cam.right = right;
    rcam.base_cam.up = up;

    return rcam;
}

void zero_race_camera_set_mode(ZeroRaceCamera *rcam, ZeroCameraMode mode, bool smooth_transition) {
    if (!rcam || mode < 0 || mode >= ZERO_CAM_MODE_COUNT) {
        return;
    }
    if (rcam->mode == mode) {
        return;
    }

    if (smooth_transition) {
        rcam->transition_start_pos = rcam->current_pos;
        rcam->transition_start_target = rcam->current_target;
        rcam->transition_t = 0.0f;
    } else {
        rcam->transition_t = 1.0f;
    }
    rcam->mode = mode;
}

void zero_race_camera_cycle_mode(ZeroRaceCamera *rcam) {
    if (!rcam) {
        return;
    }
    ZeroCameraMode next_mode = (ZeroCameraMode)((rcam->mode + 1) % ZERO_CAM_MODE_COUNT);
    zero_race_camera_set_mode(rcam, next_mode, true);
}

void zero_race_camera_update(ZeroRaceCamera *rcam,
                             Vec3 car_position, Quat car_orientation,
                             Vec3 car_velocity, float car_rpm, float steer_input,
                             float dt) {
    if (!rcam) {
        return;
    }

    if (dt <= 0.0f) {
        dt = 0.016667f;
    } else if (dt > 0.1f) {
        dt = 0.1f;
    }

    /* 1. Vehicle orientation and coordinate axes */
    Quat q = quat_normalize(car_orientation);
    Mat4 rot = quat_to_mat4(q);
    Vec3 car_right = vec3_normalize(vec3(rot.m[0], rot.m[1], rot.m[2]));
    Vec3 car_up    = vec3_normalize(vec3(rot.m[4], rot.m[5], rot.m[6]));
    Vec3 car_fwd   = vec3_normalize(vec3(rot.m[8], rot.m[9], rot.m[10]));

    /* 2. Speed and dynamic FOV dilation */
    float speed = vec3_length(car_velocity);
    const float v_max = 72.222f; /* 260 km/h in m/s */
    float speed_ratio = fminf(speed / v_max, 1.0f);
    float fov_dilation = powf(speed_ratio, rcam->fov_dilation_rate);
    float target_fov_deg = rcam->base_fov_deg + fov_dilation * (80.0f - rcam->base_fov_deg);

    /* 3. Vehicle acceleration tracking */
    Vec3 car_accel = vec3(0.0f, 0.0f, 0.0f);
    if (dt > 1e-4f) {
        car_accel = vec3_scale(vec3_sub(car_velocity, rcam->prev_car_velocity), 1.0f / dt);
    }
    bool is_first_update = (rcam->accum_time == 0.0f);
    rcam->prev_car_velocity = car_velocity;
    rcam->accum_time += dt;

    /* 4. Target position, look-at target, and up vector for active mode */
    Vec3 mode_pos;
    Vec3 mode_target;
    Vec3 mode_up = car_up;

    switch (rcam->mode) {
        case ZERO_CAM_CHASE: {
            /* 2nd-order critically damped translation spring and target tracking */
            const float h_chase = 2.4f;
            const float d_chase = 6.8f;
            Vec3 ideal_pos = vec3_sub(
                vec3_add(car_position, vec3_scale(car_up, h_chase)),
                vec3_scale(car_fwd, d_chase)
            );
            Vec3 ideal_target = vec3_add(
                vec3_add(car_position, vec3_scale(car_up, 1.1f)),
                vec3_scale(car_fwd, 3.5f)
            );

            if (is_first_update) {
                rcam->current_pos = ideal_pos;
                rcam->current_target = ideal_target;
                rcam->current_velocity = car_velocity;
                rcam->current_target_vel = car_velocity;
                rcam->transition_start_pos = ideal_pos;
                rcam->transition_start_target = ideal_target;
            }

            /* Lateral G-force tilt */
            float a_lat = vec3_dot(car_accel, car_right);
            float a_lat_eff = a_lat + steer_input * speed * 0.25f;
            float tilt_angle = -fmaxf(-0.12f, fminf(0.12f, a_lat_eff * 0.015f));
            mode_up = vec3_normalize(vec3_add(
                vec3_scale(car_up, cosf(tilt_angle)),
                vec3_scale(car_right, sinf(tilt_angle))
            ));

            /* Critically damped spring integration */
            float k = rcam->spring_stiffness;
            float c = 2.0f * sqrtf(k) * rcam->damping_ratio;

            float sub_dt = dt;
            int steps = 1;
            if (dt > 0.033f) {
                steps = (int)(dt / 0.016f) + 1;
                sub_dt = dt / (float)steps;
            }

            for (int s = 0; s < steps; ++s) {
                /* Position spring */
                Vec3 delta_p = vec3_sub(ideal_pos, rcam->current_pos);
                Vec3 f_spring = vec3_sub(vec3_scale(delta_p, k), vec3_scale(rcam->current_velocity, c));
                rcam->current_velocity = vec3_add(rcam->current_velocity, vec3_scale(f_spring, sub_dt));
                rcam->current_pos = vec3_add(rcam->current_pos, vec3_scale(rcam->current_velocity, sub_dt));

                /* Look-at target spring */
                Vec3 delta_t = vec3_sub(ideal_target, rcam->current_target);
                Vec3 ft_spring = vec3_sub(vec3_scale(delta_t, k * 1.33f), vec3_scale(rcam->current_target_vel, c * 1.15f));
                rcam->current_target_vel = vec3_add(rcam->current_target_vel, vec3_scale(ft_spring, sub_dt));
                rcam->current_target = vec3_add(rcam->current_target, vec3_scale(rcam->current_target_vel, sub_dt));
            }

            mode_pos = rcam->current_pos;
            mode_target = rcam->current_target;
            break;
        }

        case ZERO_CAM_HOOD: {
            /* Rigidly anchored to front hood with RPM-proportional high-frequency vibration */
            float f_rpm = fmaxf(car_rpm / 60.0f, 15.0f);
            float vibe_y = 0.003f * sinf(2.0f * ZERO_PI * f_rpm * rcam->accum_time);

            mode_pos = vec3_add(car_position, vec3_add(
                vec3_scale(car_fwd, 1.6f),
                vec3_scale(car_up, 0.75f + vibe_y)
            ));
            mode_target = vec3_add(mode_pos, vec3_scale(car_fwd, 25.0f));
            mode_up = car_up;

            rcam->current_pos = mode_pos;
            rcam->current_target = mode_target;
            rcam->current_velocity = car_velocity;
            break;
        }

        case ZERO_CAM_COCKPIT: {
            /* Driver head position with dynamic steer lookahead and inertial sway */
            Vec3 head_base = vec3_add(car_position, vec3_add(
                vec3_scale(car_fwd, -0.25f),
                vec3_add(vec3_scale(car_up, 1.15f), vec3_scale(car_right, -0.35f))
            ));

            /* Longitudinal and lateral acceleration sway */
            float a_long = vec3_dot(car_accel, car_fwd);
            float a_lat = vec3_dot(car_accel, car_right);
            float sway_long = -fmaxf(-0.06f, fminf(0.06f, a_long * 0.004f));
            float sway_lat  = -fmaxf(-0.05f, fminf(0.05f, a_lat * 0.004f));

            mode_pos = vec3_add(head_base, vec3_add(
                vec3_scale(car_fwd, sway_long),
                vec3_scale(car_right, sway_lat)
            ));

            /* Steer-lookahead yaw */
            float steer_yaw = -steer_input * 0.28f;
            Vec3 look_fwd = vec3_normalize(vec3_add(
                vec3_scale(car_fwd, cosf(steer_yaw)),
                vec3_scale(car_right, sinf(steer_yaw))
            ));
            mode_target = vec3_add(mode_pos, vec3_scale(look_fwd, 20.0f));
            mode_up = car_up;

            rcam->current_pos = mode_pos;
            rcam->current_target = mode_target;
            rcam->current_velocity = car_velocity;
            break;
        }

        case ZERO_CAM_BUMPER: {
            /* Ground-skimming bumper view */
            mode_pos = vec3_add(car_position, vec3_add(
                vec3_scale(car_fwd, 2.2f),
                vec3_scale(car_up, 0.35f)
            ));
            mode_target = vec3_add(mode_pos, vec3_scale(car_fwd, 30.0f));
            mode_up = car_up;

            rcam->current_pos = mode_pos;
            rcam->current_target = mode_target;
            rcam->current_velocity = car_velocity;
            break;
        }

        case ZERO_CAM_TV_BROADCAST: {
            /* Trackside broadcast tower tracking */
            if (rcam->active_tv_tower_idx < 0 || rcam->active_tv_tower_idx >= ZERO_TV_TOWER_COUNT) {
                rcam->active_tv_tower_idx = 0;
            }

            Vec3 curr_tower = zero_race_camera_tv_tower_pos(rcam->active_tv_tower_idx);
            float curr_dist = vec3_length(vec3_sub(curr_tower, car_position));

            /* Check closest tower with hysteresis */
            int best_tower = rcam->active_tv_tower_idx;
            float best_dist = curr_dist;
            for (int t = 0; t < ZERO_TV_TOWER_COUNT; ++t) {
                Vec3 t_pos = zero_race_camera_tv_tower_pos(t);
                float dist = vec3_length(vec3_sub(t_pos, car_position));
                if (dist < best_dist) {
                    best_dist = dist;
                    best_tower = t;
                }
            }

            if (best_dist < curr_dist * 0.65f || curr_dist > 85.0f) {
                rcam->active_tv_tower_idx = best_tower;
            }

            mode_pos = zero_race_camera_tv_tower_pos(rcam->active_tv_tower_idx);
            mode_target = vec3_add(car_position, vec3(0.0f, 0.9f, 0.0f));
            mode_up = vec3(0.0f, 1.0f, 0.0f);

            float dist_to_car = vec3_length(vec3_sub(mode_target, mode_pos));
            target_fov_deg = fmaxf(18.0f, fminf(52.0f, 350.0f / (dist_to_car + 1e-4f)));

            rcam->current_pos = mode_pos;
            rcam->current_target = mode_target;
            rcam->current_velocity = vec3(0.0f, 0.0f, 0.0f);
            break;
        }

        default:
            mode_pos = rcam->current_pos;
            mode_target = rcam->current_target;
            break;
    }

    /* 5. Smooth mode transition interpolation (over ~0.35s) */
    Vec3 final_pos;
    Vec3 final_target;

    if (rcam->transition_t < 1.0f) {
        rcam->transition_t += dt / 0.35f;
        if (rcam->transition_t > 1.0f) {
            rcam->transition_t = 1.0f;
        }
        float t = rcam->transition_t;
        float s = t * t * (3.0f - 2.0f * t); /* Smoothstep */
        final_pos = vec3_add(
            vec3_scale(rcam->transition_start_pos, 1.0f - s),
            vec3_scale(mode_pos, s)
        );
        final_target = vec3_add(
            vec3_scale(rcam->transition_start_target, 1.0f - s),
            vec3_scale(mode_target, s)
        );
    } else {
        final_pos = mode_pos;
        final_target = mode_target;
    }

    /* Smoothly update FOV */
    float fov_lerp = fminf(10.0f * dt, 1.0f);
    rcam->current_fov_deg += (target_fov_deg - rcam->current_fov_deg) * fov_lerp;

    /* 6. Construct orthonormal basis vectors */
    Vec3 fwd = vec3_sub(final_target, final_pos);
    if (vec3_length_sq(fwd) < 1e-6f) {
        fwd = vec3(0.0f, 0.0f, 1.0f);
    } else {
        fwd = vec3_normalize(fwd);
    }

    Vec3 right = vec3_cross(fwd, mode_up);
    if (vec3_length_sq(right) < 1e-6f) {
        right = vec3_cross(fwd, vec3(0.0f, 1.0f, 0.0f));
    }
    if (vec3_length_sq(right) < 1e-6f) {
        right = vec3(1.0f, 0.0f, 0.0f);
    }
    right = vec3_normalize(right);
    Vec3 up = vec3_normalize(vec3_cross(right, fwd));

    rcam->base_cam.position = final_pos;
    rcam->base_cam.forward = fwd;
    rcam->base_cam.right = right;
    rcam->base_cam.up = up;
    rcam->base_cam.fov = ZERO_DEG2RAD(rcam->current_fov_deg);
}

Mat4 zero_race_camera_get_view(const ZeroRaceCamera *rcam) {
    if (!rcam) {
        return mat4_identity();
    }
    Vec3 target = vec3_add(rcam->base_cam.position, rcam->base_cam.forward);
    return mat4_look_at(rcam->base_cam.position, target, rcam->base_cam.up);
}

Mat4 zero_race_camera_get_projection(const ZeroRaceCamera *rcam) {
    if (!rcam) {
        return mat4_identity();
    }
    return mat4_perspective(rcam->base_cam.fov, rcam->base_cam.aspect, rcam->base_cam.near_z, rcam->base_cam.far_z);
}

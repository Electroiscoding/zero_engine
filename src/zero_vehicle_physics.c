#include "zero_vehicle_physics.h"
#include "zero_track_spline.h"
#include <math.h>
#include <string.h>

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

float zero_transmission_get_gear_ratio(const ZeroTransmission *trans, ZeroGear gear) {
    if (!trans) return 0.0f;
    if (gear == ZERO_GEAR_REVERSE) return trans->gear_ratios[0];
    if (gear == ZERO_GEAR_NEUTRAL) return 0.0f;
    if ((int)gear >= 1 && (int)gear <= 6) return trans->gear_ratios[(int)gear];
    return 0.0f;
}

float zero_vehicle_compute_engine_torque(float rpm) {
    if (rpm < 0.0f) rpm = 0.0f;
    if (rpm >= ZERO_ENGINE_REDLINE_RPM) return 0.0f;

    float effective_rpm = (rpm < ZERO_ENGINE_IDLE_RPM) ? ZERO_ENGINE_IDLE_RPM : rpm;
    float x = (effective_rpm - ZERO_ENGINE_PEAK_RPM) / 3500.0f;
    float curve = 1.0f - 0.70f * (x * x);
    if (curve < 0.25f) curve = 0.25f;

    return ZERO_ENGINE_PEAK_TORQUE * curve;
}

void zero_vehicle_update_transmission(ZeroVehicleEnhanced *car, float dt) {
    if (!car || dt <= 0.0f) return;
    ZeroTransmission *trans = &car->transmission;

    if (trans->shift_lockout_timer > 0.0f) {
        trans->shift_lockout_timer -= dt;
        if (trans->shift_lockout_timer < 0.0f) trans->shift_lockout_timer = 0.0f;
    }

    if (trans->is_shifting) {
        trans->shift_timer -= dt;
        trans->torque_cut_factor = ZERO_TRANS_TORQUE_CUT;
        if (trans->shift_timer <= 0.0f) {
            trans->shift_timer = 0.0f;
            trans->current_gear = trans->target_gear;
            trans->is_shifting = false;
            trans->torque_cut_factor = 1.00f;
        }
    } else {
        trans->torque_cut_factor = 1.00f;
    }

    bool is_stopped = (car->speed_kmh < 2.0f);
    bool brake_held = (car->brake_input > 0.30f);

    if (is_stopped && brake_held && !trans->is_shifting && trans->shift_lockout_timer <= 0.0f) {
        if (trans->current_gear >= ZERO_GEAR_1) {
            trans->reverse_requested = true;
            trans->target_gear = ZERO_GEAR_REVERSE;
            trans->is_shifting = true;
            trans->shift_timer = ZERO_TRANS_SHIFT_TIME;
            trans->shift_lockout_timer = ZERO_TRANS_LOCKOUT_TIME;
            trans->torque_cut_factor = ZERO_TRANS_TORQUE_CUT;
        } else if (trans->current_gear == ZERO_GEAR_REVERSE) {
            trans->reverse_requested = false;
            trans->target_gear = ZERO_GEAR_1;
            trans->is_shifting = true;
            trans->shift_timer = ZERO_TRANS_SHIFT_TIME;
            trans->shift_lockout_timer = ZERO_TRANS_LOCKOUT_TIME;
            trans->torque_cut_factor = ZERO_TRANS_TORQUE_CUT;
        }
    }

    if (!trans->is_shifting && trans->shift_lockout_timer <= 0.0f && trans->current_gear >= ZERO_GEAR_1) {
        if (car->throttle_input > 0.85f && car->engine_rpm < 4500.0f && trans->current_gear > ZERO_GEAR_1) {
            trans->target_gear = (ZeroGear)(trans->current_gear - 1);
            trans->is_shifting = true;
            trans->shift_timer = ZERO_TRANS_SHIFT_TIME;
            trans->shift_lockout_timer = ZERO_TRANS_LOCKOUT_TIME;
            trans->torque_cut_factor = ZERO_TRANS_TORQUE_CUT;
        } else if (car->engine_rpm > 6200.0f && trans->current_gear < ZERO_GEAR_6) {
            trans->target_gear = (ZeroGear)(trans->current_gear + 1);
            trans->is_shifting = true;
            trans->shift_timer = ZERO_TRANS_SHIFT_TIME;
            trans->shift_lockout_timer = ZERO_TRANS_LOCKOUT_TIME;
            trans->torque_cut_factor = ZERO_TRANS_TORQUE_CUT;
        } else if (car->engine_rpm < 2800.0f && trans->current_gear > ZERO_GEAR_1) {
            trans->target_gear = (ZeroGear)(trans->current_gear - 1);
            trans->is_shifting = true;
            trans->shift_timer = ZERO_TRANS_SHIFT_TIME;
            trans->shift_lockout_timer = ZERO_TRANS_LOCKOUT_TIME;
            trans->torque_cut_factor = ZERO_TRANS_TORQUE_CUT;
        }
    }

    Mat4 rot_mat = quat_to_mat4(car->orientation);
    Vec3 fwd = vec3(rot_mat.m[8], rot_mat.m[9], rot_mat.m[10]);
    float v_long = vec3_dot(car->velocity, fwd);
    float gear_ratio = zero_transmission_get_gear_ratio(trans, trans->current_gear);
    float wheel_rpm = (fabsf(v_long) / ZERO_TIRE_RADIUS) * (60.0f / (2.0f * ZERO_PI));
    float trans_rpm = wheel_rpm * fabsf(gear_ratio) * trans->final_drive_ratio;
    float stall_rpm = ZERO_ENGINE_IDLE_RPM + car->throttle_input * 1800.0f;
    float target_rpm = fminf(ZERO_ENGINE_REDLINE_RPM, fmaxf(stall_rpm, trans_rpm));
    float rpm_smooth = fminf(1.0f, dt * 18.0f);
    car->engine_rpm += (target_rpm - car->engine_rpm) * rpm_smooth;
    if (car->engine_rpm < ZERO_ENGINE_IDLE_RPM) car->engine_rpm = ZERO_ENGINE_IDLE_RPM;
    if (car->engine_rpm > ZERO_ENGINE_REDLINE_RPM) car->engine_rpm = ZERO_ENGINE_REDLINE_RPM;

    car->engine_torque = zero_vehicle_compute_engine_torque(car->engine_rpm) * car->throttle_input;
    car->drive_wheel_torque = car->engine_torque * gear_ratio * trans->final_drive_ratio * trans->torque_cut_factor;
}

void zero_vehicle_apply_brush_tire_friction(
    ZeroVehicleEnhanced *car,
    ZeroWheel *w,
    float normal_force,
    float sub_dt,
    Vec3 *out_tire_force
) {
    if (!car || !w || !out_tire_force) return;

    if (!w->is_grounded || normal_force <= 0.0f) {
        *out_tire_force = vec3(0.0f, 0.0f, 0.0f);
        return;
    }

    Quat steer_q = quat_from_axis_angle(vec3(0.0f, 1.0f, 0.0f), w->steer_angle);
    Quat wheel_q = quat_mul(car->orientation, steer_q);
    Mat4 wheel_rot = quat_to_mat4(wheel_q);
    Vec3 wheel_fwd = vec3(wheel_rot.m[8], wheel_rot.m[9], wheel_rot.m[10]);
    Vec3 wheel_right = vec3(wheel_rot.m[0], wheel_rot.m[1], wheel_rot.m[2]);

    Vec3 r_w = rotate_vector_by_quat(car->orientation, w->hardpoint);
    Vec3 v_cp = vec3_add(car->velocity, vec3_cross(car->angular_velocity, r_w));
    float v_long = vec3_dot(v_cp, wheel_fwd);
    float v_lat = vec3_dot(v_cp, wheel_right);

    float drive_torque = 0.0f;
    if (w->is_drive) {
        drive_torque = car->drive_wheel_torque * 0.5f;
    }

    float brake_torque = car->brake_input * (w->is_front ? 3200.0f : 2000.0f);
    bool rear_handbrake = (car->handbrake && !w->is_front);
    if (rear_handbrake) {
        brake_torque += 8000.0f;
        w->angular_velocity = 0.0f;
    }

    float kappa = 0.0f;
    float v_roll = w->angular_velocity * w->radius;
    float denom = fabsf(v_long);
    float roll_abs = fabsf(v_roll);
    if (roll_abs > denom) denom = roll_abs;
    if (denom < ZERO_TIRE_SLIP_EPSILON) denom = ZERO_TIRE_SLIP_EPSILON;

    kappa = (v_roll - v_long) / denom;
    if (kappa > 1.0f) kappa = 1.0f;
    if (kappa < -1.0f) kappa = -1.0f;

    if (rear_handbrake) {
        kappa = (fabsf(v_long) > 0.1f) ? ((v_long > 0.0f) ? -1.0f : 1.0f) : -1.0f;
    }

    float lat_denom = fabsf(v_long) + ZERO_TIRE_SLIP_EPSILON;
    float tan_alpha = -v_lat / lat_denom;
    if (tan_alpha > 11.43f) tan_alpha = 11.43f;
    if (tan_alpha < -11.43f) tan_alpha = -11.43f;

    float c_kappa = ZERO_TIRE_LONG_STIFFNESS;
    float c_alpha = w->is_front ? ZERO_TIRE_LAT_STIFF_FRONT : ZERO_TIRE_LAT_STIFF_REAR;
    float mu = rear_handbrake ? ZERO_TIRE_MU_KINETIC : ZERO_TIRE_MU_STATIC;

    float phi_x = c_kappa * kappa;
    float phi_y = c_alpha * tan_alpha;
    float phi = sqrtf(phi_x * phi_x + phi_y * phi_y);
    float mu_fz = mu * normal_force;
    if (mu_fz < 1.0f) mu_fz = 1.0f;

    float sigma = phi / mu_fz;

    float f_total = 0.0f;
    if (sigma < 3.0f) {
        float s2 = sigma * sigma;
        float s3 = s2 * sigma;
        f_total = mu_fz * (sigma - (1.0f / 3.0f) * s2 + (1.0f / 27.0f) * s3);
    } else {
        f_total = mu_fz;
    }

    float fx = 0.0f;
    float fy = 0.0f;
    if (phi > 1e-4f) {
        fx = f_total * (phi_x / phi);
        fy = f_total * (phi_y / phi);
    }

    float speed = vec3_length(car->velocity);
    float speed_blend = (speed < 0.5f) ? (speed / 0.5f) : 1.0f;
    fx *= speed_blend;
    fy *= speed_blend;

    if (!rear_handbrake) {
        float sign_omega = (w->angular_velocity > 0.01f) ? 1.0f : ((w->angular_velocity < -0.01f) ? -1.0f : 0.0f);
        float net_torque = drive_torque - sign_omega * brake_torque - fx * w->radius;
        float ang_accel = net_torque / ZERO_WHEEL_INERTIA;
        w->angular_velocity += ang_accel * sub_dt;

        if (car->brake_input > 0.05f) {
            if (v_long > 0.1f && w->angular_velocity < 0.0f) w->angular_velocity = 0.0f;
            if (v_long < -0.1f && w->angular_velocity > 0.0f) w->angular_velocity = 0.0f;
        }
    }

    w->rotation_angle += w->angular_velocity * sub_dt;
    *out_tire_force = vec3_add(vec3_scale(wheel_fwd, fx), vec3_scale(wheel_right, fy));
}

void zero_vehicle_apply_aerodynamics(
    ZeroVehicleEnhanced *car,
    float sub_dt,
    Vec3 *inout_total_force,
    Vec3 *inout_total_torque
) {
    if (!car || sub_dt <= 0.0f || !inout_total_force || !inout_total_torque) return;

    Mat4 rot_mat = quat_to_mat4(car->orientation);
    Vec3 up = vec3(rot_mat.m[4], rot_mat.m[5], rot_mat.m[6]);

    float speed = vec3_length(car->velocity);
    float rho = 1.225f;
    float dynamic_pressure = 0.5f * rho * speed * speed;

    float target_airbrake = (car->brake_input > 0.30f && speed > 11.0f) ? 1.0f : 0.0f;
    car->aero_airbrake_active += (target_airbrake - car->aero_airbrake_active) * sub_dt * 6.0f;
    if (car->aero_airbrake_active < 0.0f) car->aero_airbrake_active = 0.0f;
    if (car->aero_airbrake_active > 1.0f) car->aero_airbrake_active = 1.0f;

    float effective_drag_coeff = car->aero_drag_coeff * (1.0f + 0.85f * car->aero_airbrake_active);
    if (speed > 1e-4f) {
        Vec3 drag = vec3_scale(car->velocity, -0.5f * rho * effective_drag_coeff * speed);
        *inout_total_force = vec3_add(*inout_total_force, drag);
    }

    float f_downforce_front = dynamic_pressure * car->aero_front_downforce;
    Vec3 front_aero_pos = rotate_vector_by_quat(car->orientation, vec3(0.0f, 0.0f, 1.35f));
    Vec3 front_aero_force = vec3_scale(up, -f_downforce_front);
    *inout_total_force = vec3_add(*inout_total_force, front_aero_force);
    *inout_total_torque = vec3_add(*inout_total_torque, vec3_cross(front_aero_pos, front_aero_force));

    float wing_coeff = car->aero_rear_downforce * (1.0f + 0.65f * car->aero_airbrake_active);
    float f_downforce_rear = dynamic_pressure * wing_coeff;
    Vec3 rear_aero_pos = rotate_vector_by_quat(car->orientation, vec3(0.0f, 0.45f, -1.30f));
    Vec3 rear_aero_force = vec3_scale(up, -f_downforce_rear);
    *inout_total_force = vec3_add(*inout_total_force, rear_aero_force);
    *inout_total_torque = vec3_add(*inout_total_torque, vec3_cross(rear_aero_pos, rear_aero_force));

    float avg_susp_length = 0.25f * (car->wheels[0].current_length + car->wheels[1].current_length +
                                     car->wheels[2].current_length + car->wheels[3].current_length);
    float h_eff = (avg_susp_length > 0.08f) ? avg_susp_length : 0.08f;
    float diffuser_mult = 0.35f / h_eff;
    if (diffuser_mult > 3.0f) diffuser_mult = 3.0f;
    float f_diffuser = dynamic_pressure * car->aero_diffuser_factor * diffuser_mult;
    Vec3 diffuser_pos = rotate_vector_by_quat(car->orientation, vec3(0.0f, -0.15f, -0.40f));
    Vec3 diffuser_force = vec3_scale(up, -f_diffuser);
    *inout_total_force = vec3_add(*inout_total_force, diffuser_force);
    *inout_total_torque = vec3_add(*inout_total_torque, vec3_cross(diffuser_pos, diffuser_force));

    float damp_factor = 280.0f + speed * 15.0f;
    Vec3 aero_damping = vec3_scale(car->angular_velocity, -damp_factor);
    *inout_total_torque = vec3_add(*inout_total_torque, aero_damping);
}

void zero_vehicle_resolve_barrier_collision(ZeroVehicleEnhanced *car, ZeroGroundHeightFn ground_fn) {
    if (!car) return;

    float min_dsq = 1e9f;
    int best_i = 0;
    for (int i = 0; i < ZERO_TRACK_SPLINE_COUNT; ++i) {
        float dx = car->position.x - g_track_spline[i][0];
        float dz = car->position.z - g_track_spline[i][1];
        float dsq = dx * dx + dz * dz;
        if (dsq < min_dsq) {
            min_dsq = dsq;
            best_i = i;
        }
    }

    if (min_dsq > 1225.0f) return;

    int prev_i = (best_i - 1 + ZERO_TRACK_SPLINE_COUNT) % ZERO_TRACK_SPLINE_COUNT;
    int next_i = (best_i + 1) % ZERO_TRACK_SPLINE_COUNT;

    float best_dist_sq = 1e9f;
    float best_lat_offset = 0.0f;
    float best_nx = 0.0f, best_nz = 0.0f;

    int seg_starts[2] = { prev_i, best_i };
    int seg_ends[2]   = { best_i, next_i };

    for (int s = 0; s < 2; ++s) {
        int i0 = seg_starts[s];
        int i1 = seg_ends[s];
        float sx = g_track_spline[i1][0] - g_track_spline[i0][0];
        float sz = g_track_spline[i1][1] - g_track_spline[i0][1];
        float slen_sq = sx * sx + sz * sz;
        if (slen_sq < 1e-4f) continue;

        float to_x = car->position.x - g_track_spline[i0][0];
        float to_z = car->position.z - g_track_spline[i0][1];
        float t = (to_x * sx + to_z * sz) / slen_sq;
        if (t < 0.0f) t = 0.0f;
        else if (t > 1.0f) t = 1.0f;

        float proj_x = g_track_spline[i0][0] + t * sx;
        float proj_z = g_track_spline[i0][1] + t * sz;

        float dx = car->position.x - proj_x;
        float dz = car->position.z - proj_z;
        float dsq = dx * dx + dz * dz;

        if (dsq < best_dist_sq) {
            best_dist_sq = dsq;
            float slen = sqrtf(slen_sq);
            float tx = sx / slen;
            float tz = sz / slen;
            float nx = -tz;
            float nz = tx;
            best_lat_offset = dx * nx + dz * nz;
            best_nx = nx;
            best_nz = nz;
        }
    }

    float barrier_limit = ZERO_TRACK_BARRIER_DIST - 1.20f;
    float pen = 0.0f;
    float wall_nx = 0.0f, wall_nz = 0.0f;

    if (best_lat_offset > barrier_limit) {
        pen = best_lat_offset - barrier_limit;
        wall_nx = -best_nx;
        wall_nz = -best_nz;
    } else if (best_lat_offset < -barrier_limit) {
        pen = -barrier_limit - best_lat_offset;
        wall_nx = best_nx;
        wall_nz = best_nz;
    }

    if (pen > 0.0f) {
        car->position.x += wall_nx * (pen + 0.30f);
        car->position.z += wall_nz * (pen + 0.30f);

        float vn = car->velocity.x * wall_nx + car->velocity.z * wall_nz;
        float restitution = 0.85f;
        float new_vn = fabsf(vn) * restitution;
        if (new_vn < 5.0f) new_vn = 5.0f;

        float wall_tx = -wall_nz;
        float wall_tz = wall_nx;
        float vt = car->velocity.x * wall_tx + car->velocity.z * wall_tz;
        vt *= 0.98f;

        car->velocity.x = wall_nx * new_vn + wall_tx * vt;
        car->velocity.z = wall_nz * new_vn + wall_tz * vt;

        Mat4 ch_rot = quat_to_mat4(car->orientation);
        Vec3 fwd = vec3(ch_rot.m[8], ch_rot.m[9], ch_rot.m[10]);
        float fwd_dot_wall = fwd.x * wall_nx + fwd.z * wall_nz;
        if (fwd_dot_wall < 0.0f) {
            float deflect_torque = (wall_nx * fwd.z - wall_nz * fwd.x) * 5.5f;
            car->angular_velocity.y = deflect_torque;
        } else {
            car->angular_velocity.y *= 0.70f;
        }
        car->angular_velocity.x *= 0.50f;
        car->angular_velocity.z *= 0.50f;
    }

    float ground_y = 0.0f;
    if (ground_fn) ground_y = ground_fn(car->position.x, car->position.z, NULL);
    if (car->position.y < ground_y + 0.18f) car->position.y = ground_y + 0.18f;
    if (car->position.y > ground_y + 2.00f) car->position.y = ground_y + 2.00f;
    if (car->velocity.y > 4.0f) car->velocity.y = 4.0f;
}

void zero_vehicle_update_drift_telemetry(ZeroVehicleEnhanced *car) {
    if (!car) return;

    Mat4 ch_rot = quat_to_mat4(car->orientation);
    Vec3 ch_fwd = vec3(ch_rot.m[8], ch_rot.m[9], ch_rot.m[10]);
    Vec3 ch_right = vec3(ch_rot.m[0], ch_rot.m[1], ch_rot.m[2]);

    float speed_ms = vec3_length(car->velocity);
    car->speed_kmh = speed_ms * 3.6f;

    if (speed_ms > 2.0f) {
        float v_fwd = vec3_dot(car->velocity, ch_fwd);
        float v_lat = vec3_dot(car->velocity, ch_right);

        float beta = atan2f(v_lat, v_fwd) * (180.0f / ZERO_PI);
        car->drift_angle = fabsf(beta);

        bool high_angle = (car->drift_angle >= 8.0f);
        bool hb_slide = (car->handbrake && car->speed_kmh > 12.0f);
        car->is_drifting = (car->speed_kmh > 15.0f && (high_angle || hb_slide));
    } else {
        car->drift_angle = 0.0f;
        car->is_drifting = false;
    }
}

ZeroVehicleEnhanced zero_vehicle_enhanced_create(Vec3 spawn_pos, Quat spawn_rot) {
    ZeroVehicleEnhanced car;
    memset(&car, 0, sizeof(car));

    car.position = spawn_pos;
    float q_mag = quat_dot(spawn_rot, spawn_rot);
    car.orientation = (q_mag > 1e-4f) ? quat_normalize(spawn_rot) : quat_identity();
    car.transform = mat4_identity();

    car.mass = ZERO_VEHICLE_MASS;
    car.inertia = vec3(1400.0f, 950.0f, 750.0f);
    car.inv_inertia = vec3(1.0f / 1400.0f, 1.0f / 950.0f, 1.0f / 750.0f);

    float half_width_f = 0.82f;
    float half_width_r = 0.84f;
    float front_z = 1.35f;
    float rear_z = -1.30f;

    // Wheel 0: FL
    car.wheels[0].hardpoint = vec3(-half_width_f, 0.0f, front_z);
    car.wheels[0].is_front = true;
    car.wheels[0].is_drive = false;

    // Wheel 1: FR
    car.wheels[1].hardpoint = vec3(half_width_f, 0.0f, front_z);
    car.wheels[1].is_front = true;
    car.wheels[1].is_drive = false;

    // Wheel 2: RL
    car.wheels[2].hardpoint = vec3(-half_width_r, 0.0f, rear_z);
    car.wheels[2].is_front = false;
    car.wheels[2].is_drive = true;

    // Wheel 3: RR
    car.wheels[3].hardpoint = vec3(half_width_r, 0.0f, rear_z);
    car.wheels[3].is_front = false;
    car.wheels[3].is_drive = true;

    for (int i = 0; i < 4; ++i) {
        car.wheels[i].rest_length = ZERO_SUSP_REST_LENGTH;
        car.wheels[i].spring_stiffness = ZERO_SUSP_SPRING_K;
        car.wheels[i].damping_rate = ZERO_SUSP_DAMPER_C;
        car.wheels[i].radius = ZERO_TIRE_RADIUS;
        car.wheels[i].current_length = ZERO_SUSP_REST_LENGTH;
        car.wheels[i].compression = 0.0f;
        car.wheels[i].suspension_velocity = 0.0f;
        car.wheels[i].is_grounded = false;
        car.wheels[i].contact_point = vec3(0.0f, 0.0f, 0.0f);
        car.wheels[i].contact_normal = vec3(0.0f, 1.0f, 0.0f);
        car.wheels[i].steer_angle = 0.0f;
        car.wheels[i].rotation_angle = 0.0f;
        car.wheels[i].angular_velocity = 0.0f;
    }

    car.anti_roll_stiffness_front = ZERO_ARB_STIFFNESS_FRONT;
    car.anti_roll_stiffness_rear = ZERO_ARB_STIFFNESS_REAR;

    car.transmission.current_gear = ZERO_GEAR_1;
    car.transmission.target_gear = ZERO_GEAR_1;
    car.transmission.gear_ratios[0] = -3.40f;
    car.transmission.gear_ratios[1] =  3.60f;
    car.transmission.gear_ratios[2] =  2.19f;
    car.transmission.gear_ratios[3] =  1.52f;
    car.transmission.gear_ratios[4] =  1.18f;
    car.transmission.gear_ratios[5] =  0.95f;
    car.transmission.gear_ratios[6] =  0.78f;
    car.transmission.final_drive_ratio = 3.73f;
    car.transmission.shift_timer = 0.0f;
    car.transmission.shift_lockout_timer = 0.0f;
    car.transmission.torque_cut_factor = 1.0f;
    car.transmission.is_shifting = false;
    car.transmission.reverse_requested = false;

    car.engine_rpm = ZERO_ENGINE_IDLE_RPM;
    car.engine_torque = 0.0f;
    car.drive_wheel_torque = 0.0f;

    car.aero_drag_coeff = 0.32f * 2.2f;
    car.aero_front_downforce = 1.15f;
    car.aero_rear_downforce = 1.65f;
    car.aero_diffuser_factor = 0.85f;
    car.aero_airbrake_active = 0.0f;

    car.race_position = 1;

    return car;
}

void zero_vehicle_enhanced_reset(ZeroVehicleEnhanced *car, Vec3 spawn_pos, Quat spawn_rot) {
    if (!car) return;

    car->position = spawn_pos;
    float q_mag = quat_dot(spawn_rot, spawn_rot);
    car->orientation = (q_mag > 1e-4f) ? quat_normalize(spawn_rot) : quat_identity();
    car->velocity = vec3(0.0f, 0.0f, 0.0f);
    car->angular_velocity = vec3(0.0f, 0.0f, 0.0f);
    car->transform = mat4_identity();

    car->throttle_input = 0.0f;
    car->brake_input = 0.0f;
    car->steer_input = 0.0f;
    car->handbrake = false;

    car->speed_kmh = 0.0f;
    car->drift_angle = 0.0f;
    car->is_drifting = false;

    car->transmission.current_gear = ZERO_GEAR_1;
    car->transmission.target_gear = ZERO_GEAR_1;
    car->transmission.shift_timer = 0.0f;
    car->transmission.shift_lockout_timer = 0.0f;
    car->transmission.torque_cut_factor = 1.0f;
    car->transmission.is_shifting = false;
    car->transmission.reverse_requested = false;

    car->engine_rpm = ZERO_ENGINE_IDLE_RPM;
    car->engine_torque = 0.0f;
    car->drive_wheel_torque = 0.0f;
    car->aero_airbrake_active = 0.0f;

    for (int i = 0; i < 4; ++i) {
        car->wheels[i].current_length = ZERO_SUSP_REST_LENGTH;
        car->wheels[i].compression = 0.0f;
        car->wheels[i].suspension_velocity = 0.0f;
        car->wheels[i].is_grounded = false;
        car->wheels[i].steer_angle = 0.0f;
        car->wheels[i].angular_velocity = 0.0f;
        car->wheels[i].rotation_angle = 0.0f;
    }
}

void zero_vehicle_enhanced_update(ZeroVehicleEnhanced *car, float dt, ZeroGroundHeightFn ground_fn) {
    if (!car || dt <= 0.0f) return;

    zero_vehicle_update_transmission(car, dt);

    const int sub_steps = 4;
    const float sub_dt = dt / (float)sub_steps;

    for (int step = 0; step < sub_steps; ++step) {
        Mat4 rot_mat = quat_to_mat4(car->orientation);
        Vec3 u_right = vec3(rot_mat.m[0], rot_mat.m[1], rot_mat.m[2]);
        Vec3 u_up    = vec3(rot_mat.m[4], rot_mat.m[5], rot_mat.m[6]);
        Vec3 u_fwd   = vec3(rot_mat.m[8], rot_mat.m[9], rot_mat.m[10]);

        float speed_kmh_approx = vec3_length(car->velocity) * 3.6f;
        float speed_factor = speed_kmh_approx / 120.0f;
        if (speed_factor > 1.0f) speed_factor = 1.0f;
        float lock_deg = 24.0f - speed_factor * 12.0f;
        float current_steer = car->steer_input * ZERO_DEG2RAD(lock_deg);

        car->wheels[0].steer_angle = current_steer;
        car->wheels[1].steer_angle = current_steer;
        car->wheels[2].steer_angle = 0.0f;
        car->wheels[3].steer_angle = 0.0f;

        float f_susp_base[4] = {0.0f, 0.0f, 0.0f, 0.0f};

        for (int i = 0; i < 4; ++i) {
            ZeroWheel *w = &car->wheels[i];
            Vec3 r_w = rotate_vector_by_quat(car->orientation, w->hardpoint);
            Vec3 hp_world = vec3_add(car->position, r_w);

            Vec3 ground_norm = vec3(0.0f, 1.0f, 0.0f);
            float ground_y = 0.0f;
            if (ground_fn) {
                ground_y = ground_fn(hp_world.x, hp_world.z, &ground_norm);
            }

            float ray_len = w->rest_length + w->radius;
            float denom_proj = vec3_dot(u_up, ground_norm);
            if (denom_proj < 0.10f) denom_proj = 0.10f;
            float contact_dist = (hp_world.y - ground_y) / denom_proj;

            if (contact_dist < ray_len) {
                w->is_grounded = true;
                float curr_len = contact_dist - w->radius;
                if (curr_len < 0.08f) curr_len = 0.08f;
                if (curr_len > w->rest_length) curr_len = w->rest_length;
                w->current_length = curr_len;

                float compression = w->rest_length - curr_len;
                w->compression = compression;

                Vec3 hp_vel = vec3_add(car->velocity, vec3_cross(car->angular_velocity, r_w));
                float susp_vel = vec3_dot(hp_vel, u_up);
                w->suspension_velocity = susp_vel;

                w->contact_point = vec3_sub(hp_world, vec3_scale(u_up, contact_dist));
                w->contact_normal = ground_norm;

                float r_x = w->hardpoint.x;
                float r_z = w->hardpoint.z;
                float inv_m = 1.0f / car->mass;
                float w_i = inv_m + (r_z * r_z * car->inv_inertia.x) + (r_x * r_x * car->inv_inertia.z);

                float k_term = w->spring_stiffness * sub_dt * sub_dt;
                float c_term = w->damping_rate * sub_dt;
                float xpbd_denom = 1.0f + w_i * (k_term + c_term);

                float raw_force = (w->spring_stiffness * compression) - (w->damping_rate * susp_vel);
                float normal_f = raw_force / xpbd_denom;
                if (normal_f < 0.0f) normal_f = 0.0f;

                f_susp_base[i] = normal_f;
            } else {
                w->is_grounded = false;
                w->current_length = w->rest_length;
                w->compression = 0.0f;
                w->suspension_velocity = 0.0f;
                w->contact_point = vec3_sub(hp_world, vec3_scale(u_up, ray_len));
                w->contact_normal = ground_norm;
                f_susp_base[i] = 0.0f;
            }
        }

        float df_front = car->anti_roll_stiffness_front * (car->wheels[0].compression - car->wheels[1].compression);
        float df_rear  = car->anti_roll_stiffness_rear  * (car->wheels[2].compression - car->wheels[3].compression);

        float f_susp[4];
        f_susp[0] = fmaxf(0.0f, f_susp_base[0] + df_front);
        f_susp[1] = fmaxf(0.0f, f_susp_base[1] - df_front);
        f_susp[2] = fmaxf(0.0f, f_susp_base[2] + df_rear);
        f_susp[3] = fmaxf(0.0f, f_susp_base[3] - df_rear);

        Vec3 total_force = vec3(0.0f, -9.81f * car->mass, 0.0f);
        Vec3 total_torque = vec3(0.0f, 0.0f, 0.0f);

        for (int i = 0; i < 4; ++i) {
            if (!car->wheels[i].is_grounded) continue;
            Vec3 r_w = rotate_vector_by_quat(car->orientation, car->wheels[i].hardpoint);
            Vec3 f_tire = vec3(0.0f, 0.0f, 0.0f);
            zero_vehicle_apply_brush_tire_friction(car, &car->wheels[i], f_susp[i], sub_dt, &f_tire);
            Vec3 f_wheel = vec3_add(vec3_scale(u_up, f_susp[i]), f_tire);
            total_force = vec3_add(total_force, f_wheel);
            total_torque = vec3_add(total_torque, vec3_cross(r_w, f_wheel));
        }

        zero_vehicle_apply_aerodynamics(car, sub_dt, &total_force, &total_torque);

        Vec3 accel = vec3_scale(total_force, 1.0f / car->mass);
        car->velocity = vec3_add(car->velocity, vec3_scale(accel, sub_dt));

        Vec3 ang_accel = apply_world_inv_inertia(car->orientation, car->inv_inertia, total_torque);
        car->angular_velocity = vec3_add(car->angular_velocity, vec3_scale(ang_accel, sub_dt));

        float v_forward = vec3_dot(car->velocity, u_fwd);
        float v_lateral = vec3_dot(car->velocity, u_right);
        float v_vertical = vec3_dot(car->velocity, u_up);

        float lateral_slip_retained = car->handbrake ? 0.88f : 0.12f;
        v_lateral *= powf(lateral_slip_retained, sub_dt * 30.0f);
        car->velocity = vec3_add(
            vec3_scale(u_fwd, v_forward),
            vec3_add(vec3_scale(u_right, v_lateral), vec3_scale(u_up, v_vertical))
        );

        const float wheelbase = 2.65f;
        float target_yaw_rate = (v_forward / wheelbase) * tanf(current_steer) * 0.42f;
        float yaw_blend = car->handbrake ? 0.35f : 0.45f;
        car->angular_velocity.y += (target_yaw_rate - car->angular_velocity.y) * yaw_blend * (sub_dt * 60.0f);

        car->position = vec3_add(car->position, vec3_scale(car->velocity, sub_dt));

        Quat spin = {
            car->angular_velocity.x * 0.5f,
            car->angular_velocity.y * 0.5f,
            car->angular_velocity.z * 0.5f,
            0.0f
        };
        Quat dq = quat_mul(spin, car->orientation);
        car->orientation.x += dq.x * sub_dt;
        car->orientation.y += dq.y * sub_dt;
        car->orientation.z += dq.z * sub_dt;
        car->orientation.w += dq.w * sub_dt;
        car->orientation = quat_normalize(car->orientation);

        zero_vehicle_resolve_barrier_collision(car, ground_fn);
    }

    Mat4 rot_mat = quat_to_mat4(car->orientation);
    Mat4 trans_mat = mat4_translate(car->position);
    car->transform = mat4_mul(trans_mat, rot_mat);

    zero_vehicle_update_drift_telemetry(car);
}

#include "zero_vehicle.h"
#include "zero_track_spline.h"
#include <math.h>
#include <string.h>

ZeroVehicle zero_vehicle_create(Vec3 spawn_pos) {
    ZeroVehicle car;
    memset(&car, 0, sizeof(car));

    car.position = spawn_pos;
    car.orientation = quat_identity();
    car.transform = mat4_identity();

    car.mass = 1150.0f;
    car.inertia = vec3(1400.0f, 950.0f, 750.0f);
    car.inv_inertia = vec3(1.0f / 1400.0f, 1.0f / 950.0f, 1.0f / 750.0f);

    float half_width_f = 0.82f;
    float half_width_r = 0.84f;
    float front_z = 1.35f;
    float rear_z = -1.30f;

    // Wheel 0: Front-Left (-X)
    car.wheels[0].hardpoint = vec3(-half_width_f, 0.0f, front_z);
    car.wheels[0].is_front = true;
    car.wheels[0].is_drive = false;

    // Wheel 1: Front-Right (+X)
    car.wheels[1].hardpoint = vec3(half_width_f, 0.0f, front_z);
    car.wheels[1].is_front = true;
    car.wheels[1].is_drive = false;

    // Wheel 2: Rear-Left (-X)
    car.wheels[2].hardpoint = vec3(-half_width_r, 0.0f, rear_z);
    car.wheels[2].is_front = false;
    car.wheels[2].is_drive = true;

    // Wheel 3: Rear-Right (+X)
    car.wheels[3].hardpoint = vec3(half_width_r, 0.0f, rear_z);
    car.wheels[3].is_front = false;
    car.wheels[3].is_drive = true;

    for (int i = 0; i < 4; ++i) {
        car.wheels[i].rest_length = 0.55f;
        car.wheels[i].spring_stiffness = 40000.0f;
        car.wheels[i].damping_rate = 3400.0f;
        car.wheels[i].radius = 0.37f;
        car.wheels[i].current_length = 0.55f;
    }

    car.gear = 1;
    car.engine_rpm = 1000.0f;

    // Aerodynamics Subsystem Coefficients
    car.aero_drag_coeff = 0.32f * 2.2f;       // CD * Frontal Area
    car.aero_front_downforce = 1.15f;         // Front splitter downforce
    car.aero_rear_downforce = 1.65f;          // Active rear wing downforce
    car.aero_diffuser_factor = 0.85f;         // Ground-effect underbody suction
    car.aero_airbrake_active = 0.0f;          // Retracted initially

    return car;
}

static inline Vec3 rotate_vector_quat(Quat q, Vec3 v) {
    Vec3 q_vec = vec3(q.x, q.y, q.z);
    Vec3 uv = vec3_cross(q_vec, v);
    Vec3 uuv = vec3_cross(q_vec, uv);
    return vec3_add(v, vec3_scale(vec3_add(vec3_scale(uv, q.w), uuv), 2.0f));
}

void zero_vehicle_update(ZeroVehicle *car, float dt, ZeroGroundHeightFn ground_fn) {
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
        float lock_degrees = 24.0f - speed_factor * 12.0f; // 24 deg at low speed, 12 deg at high speed
        float max_steer = ZERO_DEG2RAD(lock_degrees);
        float current_steer = car->steer_input * max_steer;

        int grounded_wheels = 0;

        for (int i = 0; i < 4; ++i) {
            ZeroWheel *w = &car->wheels[i];
            if (w->is_front) {
                w->steer_angle = current_steer;
            } else {
                w->steer_angle = 0.0f;
            }

            Vec3 r_w = rotate_vector_quat(car->orientation, w->hardpoint);
            Vec3 hardpoint_world = vec3_add(car->position, r_w);

            Vec3 ground_norm = vec3(0, 1, 0);
            float ground_y = 0.0f;
            if (ground_fn) {
                ground_y = ground_fn(hardpoint_world.x, hardpoint_world.z, &ground_norm);
            }

            float ray_len = w->rest_length + w->radius;
            float contact_dist = hardpoint_world.y - ground_y;

            if (contact_dist < ray_len) {
                w->is_grounded = true;
                grounded_wheels++;
                w->current_length = contact_dist - w->radius;
                if (w->current_length < 0.05f) w->current_length = 0.05f;

                float compression = w->rest_length - w->current_length;
                Vec3 point_vel = vec3_add(car->velocity, vec3_cross(car->angular_velocity, r_w));
                float susp_vel = vec3_dot(point_vel, up);

                float spring_f = w->spring_stiffness * compression;
                float damper_f = -w->damping_rate * susp_vel;
                float normal_f = spring_f + damper_f;
                if (normal_f < 0.0f) normal_f = 0.0f;

                // Wheel orientation with steer angle
                Quat steer_q = quat_from_axis_angle(vec3(0, 1, 0), w->steer_angle);
                Quat wheel_q = quat_mul(car->orientation, steer_q);
                Mat4 wheel_rot = quat_to_mat4(wheel_q);
                Vec3 wheel_fwd = vec3(wheel_rot.m[8], wheel_rot.m[9], wheel_rot.m[10]);
                Vec3 wheel_right = vec3(wheel_rot.m[0], wheel_rot.m[1], wheel_rot.m[2]);

                float v_lat = vec3_dot(point_vel, wheel_right);
                float v_long = vec3_dot(point_vel, wheel_fwd);

                // Lateral friction with high-grip tarmac tires (eliminates icy sliding)
                float cornering_stiff = w->is_front ? 52000.0f : 48000.0f;
                if (car->handbrake && !w->is_front) {
                    cornering_stiff = 5500.0f; // Induces controlled power slides only with handbrake
                }
                float lat_force_mag = -v_lat * cornering_stiff;
                float max_lat = normal_f * 2.45f;
                if (lat_force_mag > max_lat) lat_force_mag = max_lat;
                if (lat_force_mag < -max_lat) lat_force_mag = -max_lat;

                // Longitudinal drive & brake
                float drive_f = 0.0f;
                float brake_f = 0.0f;
                if (w->is_drive) {
                    drive_f = car->throttle_input * 4200.0f;
                }
                if (car->brake_input > 0.01f) {
                    if (v_long > 0.5f) {
                        brake_f = -car->brake_input * 6000.0f;
                    } else if (w->is_drive) {
                        drive_f = -car->brake_input * 3000.0f; // Reverse gear acceleration
                    }
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

        // Advanced Multi-Element Aerodynamics Simulation
        float speed = vec3_length(car->velocity);
        float rho = 1.225f; // Sea-level air density (kg/m^3)
        float dynamic_pressure = 0.5f * rho * speed * speed;

        // Dynamic Active Airbrake: deploys when braking at speeds > 40 km/h
        float target_airbrake = (car->brake_input > 0.3f && speed > 11.0f) ? 1.0f : 0.0f;
        car->aero_airbrake_active += (target_airbrake - car->aero_airbrake_active) * sub_dt * 6.0f;

        // 1. Aerodynamic Drag Polar (increases when airbrake is deployed)
        float effective_drag_coeff = car->aero_drag_coeff * (1.0f + 0.85f * car->aero_airbrake_active);
        if (speed > 1e-4f) {
            Vec3 drag = vec3_scale(car->velocity, -0.5f * rho * effective_drag_coeff * speed);
            total_force = vec3_add(total_force, drag);
        }

        // 2. Front Splitter Downforce: Applied at front axle (+Z)
        float f_downforce_front = dynamic_pressure * car->aero_front_downforce;
        Vec3 front_aero_pos = rotate_vector_quat(car->orientation, vec3(0.0f, 0.0f, 1.35f));
        Vec3 front_aero_force = vec3_scale(up, -f_downforce_front);
        total_force = vec3_add(total_force, front_aero_force);
        total_torque = vec3_add(total_torque, vec3_cross(front_aero_pos, front_aero_force));

        // 3. Active Rear Wing Downforce: Applied at rear spoiler (-Z), augmented by airbrake angle
        float wing_coeff = car->aero_rear_downforce * (1.0f + 0.65f * car->aero_airbrake_active);
        float f_downforce_rear = dynamic_pressure * wing_coeff;
        Vec3 rear_aero_pos = rotate_vector_quat(car->orientation, vec3(0.0f, 0.45f, -1.30f));
        Vec3 rear_aero_force = vec3_scale(up, -f_downforce_rear);
        total_force = vec3_add(total_force, rear_aero_force);
        total_torque = vec3_add(total_torque, vec3_cross(rear_aero_pos, rear_aero_force));

        // 4. Ground-Effect Diffuser Suction: Scales inversely with underbody ride height
        float avg_susp_length = 0.0f;
        for (int k = 0; k < 4; ++k) avg_susp_length += car->wheels[k].current_length;
        avg_susp_length *= 0.25f;
        float ground_effect_mult = 0.35f / (avg_susp_length > 0.08f ? avg_susp_length : 0.08f);
        if (ground_effect_mult > 3.0f) ground_effect_mult = 3.0f;
        float f_diffuser = dynamic_pressure * car->aero_diffuser_factor * ground_effect_mult;
        total_force = vec3_add(total_force, vec3_scale(up, -f_diffuser));

        // High-Speed Pitch and Yaw Aerodynamic Stabilization
        total_torque = vec3_sub(total_torque, vec3_scale(car->angular_velocity, 280.0f + speed * 15.0f));

        // Integration
        Vec3 accel = vec3_scale(total_force, 1.0f / car->mass);
        car->velocity = vec3_add(car->velocity, vec3_scale(accel, sub_dt));

        // Lateral Grip Constraint (eliminates icy sliding: locks velocity to vehicle heading unless drifting)
        Mat4 ch_rot = quat_to_mat4(car->orientation);
        Vec3 ch_fwd = vec3(ch_rot.m[8], ch_rot.m[9], ch_rot.m[10]);
        Vec3 ch_right = vec3(ch_rot.m[0], ch_rot.m[1], ch_rot.m[2]);
        Vec3 ch_up = vec3(ch_rot.m[4], ch_rot.m[5], ch_rot.m[6]);

        float v_forward = vec3_dot(car->velocity, ch_fwd);
        float v_lateral = vec3_dot(car->velocity, ch_right);
        float v_vertical = vec3_dot(car->velocity, ch_up);

        float lateral_slip_retained = car->handbrake ? 0.88f : 0.08f;
        v_lateral *= powf(lateral_slip_retained, sub_dt * 30.0f);

        // Reconstruct gripped velocity
        car->velocity = vec3_add(
            vec3_scale(ch_fwd, v_forward),
            vec3_add(vec3_scale(ch_right, v_lateral), vec3_scale(ch_up, v_vertical))
        );

        car->position = vec3_add(car->position, vec3_scale(car->velocity, sub_dt));

        // Progressive Ackermann Yaw Steering Authority (smooth, stable handling curve)
        float wheelbase = 2.90f;
        float target_yaw_rate = (v_forward / wheelbase) * tanf(current_steer) * 0.42f;
        float yaw_blend = car->handbrake ? 0.35f : 0.45f;
        car->angular_velocity.y += (target_yaw_rate - car->angular_velocity.y) * yaw_blend;

        Vec3 ang_accel = vec3(
            total_torque.x * car->inv_inertia.x,
            total_torque.y * car->inv_inertia.y * 0.15f,
            total_torque.z * car->inv_inertia.z
        );
        car->angular_velocity = vec3_add(car->angular_velocity, vec3_scale(ang_accel, sub_dt));

        // Integrate orientation quaternion
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
    }

    // Track barrier collision resolution (keeps car on course, prevents launching into sky)
    float min_dist_sq = 1e9f;
    int best_i = 0;
    for (int i = 0; i < ZERO_TRACK_SPLINE_COUNT; ++i) {
        float dx = car->position.x - g_track_spline[i][0];
        float dz = car->position.z - g_track_spline[i][1];
        float dsq = dx * dx + dz * dz;
        if (dsq < min_dist_sq) {
            min_dist_sq = dsq;
            best_i = i;
        }
    }
    // Only resolve collision if car is in vicinity of the circuit barriers (within 35m)
    if (min_dist_sq < 1225.0f) {
        int next_i = (best_i + 1) % ZERO_TRACK_SPLINE_COUNT;
        float seg_dx = g_track_spline[next_i][0] - g_track_spline[best_i][0];
        float seg_dz = g_track_spline[next_i][1] - g_track_spline[best_i][1];
        float seg_len = sqrtf(seg_dx * seg_dx + seg_dz * seg_dz);
        if (seg_len > 1e-4f) {
            float nx = -seg_dz / seg_len;
            float nz = seg_dx / seg_len;
            float to_x = car->position.x - g_track_spline[best_i][0];
            float to_z = car->position.z - g_track_spline[best_i][1];
            float lat_offset = to_x * nx + to_z * nz;
            float max_dist = ZERO_TRACK_BARRIER_DIST - 1.2f;
            if (lat_offset > max_dist) {
                float pen = lat_offset - max_dist;
                car->position.x -= nx * pen;
                car->position.z -= nz * pen;
                float vn = car->velocity.x * nx + car->velocity.z * nz;
                if (vn > 0.0f) {
                    car->velocity.x -= 1.5f * vn * nx;
                    car->velocity.z -= 1.5f * vn * nz;
                }
            } else if (lat_offset < -max_dist) {
                float pen = -max_dist - lat_offset;
                car->position.x += nx * pen;
                car->position.z += nz * pen;
                float vn = car->velocity.x * (-nx) + car->velocity.z * (-nz);
                if (vn > 0.0f) {
                    car->velocity.x -= 1.5f * vn * (-nx);
                    car->velocity.z -= 1.5f * vn * (-nz);
                }
            }
        }
    }

    // Ground clamp to prevent flying into sky
    if (car->position.y < 0.20f) car->position.y = 0.20f;
    if (car->position.y > 1.80f) car->position.y = 1.80f;

    // Update transform matrix
    Mat4 rot_mat = quat_to_mat4(car->orientation);
    Mat4 trans_mat = mat4_translate(car->position);
    car->transform = mat4_mul(trans_mat, rot_mat);

    // Speedometer and gear calculations
    float speed_ms = vec3_length(car->velocity);
    car->speed_kmh = speed_ms * 3.6f;

    if (car->speed_kmh < 35.0f) car->gear = 1;
    else if (car->speed_kmh < 70.0f) car->gear = 2;
    else if (car->speed_kmh < 110.0f) car->gear = 3;
    else if (car->speed_kmh < 160.0f) car->gear = 4;
    else car->gear = 5;

    float base_rpm = 1000.0f + (car->speed_kmh / (float)car->gear) * 65.0f;
    if (base_rpm > 7500.0f) base_rpm = 7500.0f;
    car->engine_rpm = base_rpm;

    // Detect drift angle
    Vec3 fwd = vec3(car->transform.m[8], car->transform.m[9], car->transform.m[10]);
    if (speed_ms > 3.0f) {
        Vec3 vel_norm = vec3_normalize(car->velocity);
        float dot_fwd = vec3_dot(vel_norm, fwd);
        if (dot_fwd < 0.92f && dot_fwd > -0.5f) {
            car->is_drifting = true;
            car->drift_angle = acosf(dot_fwd) * (180.0f / ZERO_PI);
        } else {
            car->is_drifting = false;
            car->drift_angle = 0.0f;
        }
    } else {
        car->is_drifting = false;
        car->drift_angle = 0.0f;
    }
}

void zero_vehicle_reset(ZeroVehicle *car, Vec3 spawn_pos) {
    if (!car) return;
    car->position = spawn_pos;
    car->velocity = vec3(0, 0, 0);
    car->angular_velocity = vec3(0, 0, 0);
    car->orientation = quat_identity();
    car->speed_kmh = 0.0f;
    car->engine_rpm = 1000.0f;
    car->gear = 1;
    car->is_drifting = false;
    car->drift_angle = 0.0f;
    for (int i = 0; i < 4; ++i) {
        car->wheels[i].current_length = car->wheels[i].rest_length;
        car->wheels[i].is_grounded = false;
    }
}

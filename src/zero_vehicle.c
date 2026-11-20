#include "zero_vehicle.h"
#include <math.h>
#include <string.h>

ZeroVehicle zero_vehicle_create(Vec3 spawn_pos) {
    ZeroVehicle car;
    memset(&car, 0, sizeof(car));

    car.position = spawn_pos;
    car.orientation = quat_identity();
    car.transform = mat4_identity();

    car.mass = 1200.0f;
    car.inertia = vec3(1800.0f, 2400.0f, 750.0f);
    car.inv_inertia = vec3(1.0f / 1800.0f, 1.0f / 2400.0f, 1.0f / 750.0f);

    float half_width = 0.85f;
    float front_z = 1.35f;
    float rear_z = -1.25f;

    // Wheel 0: Front-Left
    car.wheels[0].hardpoint = vec3(-half_width, 0.0f, front_z);
    car.wheels[0].is_front = true;
    car.wheels[0].is_drive = false;

    // Wheel 1: Front-Right
    car.wheels[1].hardpoint = vec3(half_width, 0.0f, front_z);
    car.wheels[1].is_front = true;
    car.wheels[1].is_drive = false;

    // Wheel 2: Rear-Left
    car.wheels[2].hardpoint = vec3(-half_width, 0.0f, rear_z);
    car.wheels[2].is_front = false;
    car.wheels[2].is_drive = true;

    // Wheel 3: Rear-Right
    car.wheels[3].hardpoint = vec3(half_width, 0.0f, rear_z);
    car.wheels[3].is_front = false;
    car.wheels[3].is_drive = true;

    for (int i = 0; i < 4; ++i) {
        car.wheels[i].rest_length = 0.55f;
        car.wheels[i].spring_stiffness = 38000.0f;
        car.wheels[i].damping_rate = 3200.0f;
        car.wheels[i].radius = 0.35f;
        car.wheels[i].current_length = 0.55f;
    }

    car.gear = 1;
    car.engine_rpm = 1000.0f;
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

        float max_steer = ZERO_DEG2RAD(34.0f);
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

                // Lateral friction & drift
                float cornering_stiff = 28000.0f;
                if (car->handbrake && !w->is_front) {
                    cornering_stiff = 4500.0f; // Induces power slides
                }
                float lat_force_mag = -v_lat * cornering_stiff;
                float max_lat = normal_f * 1.15f;
                if (lat_force_mag > max_lat) lat_force_mag = max_lat;
                if (lat_force_mag < -max_lat) lat_force_mag = -max_lat;

                // Longitudinal drive & brake
                float drive_f = 0.0f;
                if (w->is_drive) {
                    drive_f = car->throttle_input * 3600.0f;
                }
                float brake_f = 0.0f;
                if (car->brake_input > 0.01f) {
                    float sgn = (v_long > 0.1f) ? -1.0f : ((v_long < -0.1f) ? 1.0f : 0.0f);
                    brake_f = sgn * car->brake_input * 4500.0f;
                }
                if (car->handbrake && !w->is_front) {
                    brake_f -= (v_long > 0.1f ? 1.0f : 0.0f) * 6000.0f;
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

        // Aerodynamic drag & downforce
        float speed = vec3_length(car->velocity);
        Vec3 drag = vec3_scale(car->velocity, -0.5f * 1.225f * 0.32f * 2.2f * speed);
        total_force = vec3_add(total_force, drag);

        Vec3 downforce = vec3_scale(up, -0.5f * 1.225f * 0.85f * speed * speed);
        total_force = vec3_add(total_force, downforce);

        // Angular damping
        total_torque = vec3_sub(total_torque, vec3_scale(car->angular_velocity, 180.0f));

        // Integration
        Vec3 accel = vec3_scale(total_force, 1.0f / car->mass);
        car->velocity = vec3_add(car->velocity, vec3_scale(accel, sub_dt));
        car->position = vec3_add(car->position, vec3_scale(car->velocity, sub_dt));

        Vec3 ang_accel = vec3(
            total_torque.x * car->inv_inertia.x,
            total_torque.y * car->inv_inertia.y,
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

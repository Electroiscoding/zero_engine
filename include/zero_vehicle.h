#ifndef ZERO_VEHICLE_H
#define ZERO_VEHICLE_H

#include "zero_core.h"
#include "zero_math.h"

typedef struct ZeroWheel {
    Vec3 hardpoint;
    float rest_length;
    float spring_stiffness;
    float damping_rate;
    float radius;

    float current_length;
    float compression;
    float suspension_velocity;
    bool is_grounded;
    Vec3 contact_point;
    Vec3 contact_normal;

    float steer_angle;
    bool is_front;
    bool is_drive;
    float rotation_angle;
    float angular_velocity;
} ZeroWheel;

typedef struct ZeroVehicle {
    Vec3 position;
    Vec3 velocity;
    Vec3 angular_velocity;
    Quat orientation;
    Mat4 transform;

    float mass;
    Vec3 inertia;
    Vec3 inv_inertia;

    ZeroWheel wheels[4];

    float throttle_input;
    float brake_input;
    float steer_input;
    bool handbrake;

    float speed_kmh;
    float engine_rpm;
    int gear;
    float drift_angle;
    bool is_drifting;

    // Advanced Aerodynamics Subsystem
    float aero_drag_coeff;       // CD * frontal area (default: 0.32 * 2.2 m^2)
    float aero_front_downforce;  // Front splitter downforce coefficient
    float aero_rear_downforce;   // Active rear wing downforce coefficient
    float aero_diffuser_factor;  // Ground-effect underbody suction
    float aero_airbrake_active;  // Dynamic spoiler airbrake deployment (0.0 to 1.0)
} ZeroVehicle;

typedef float (*ZeroGroundHeightFn)(float x, float z, Vec3 *out_normal);

ZeroVehicle zero_vehicle_create(Vec3 spawn_pos);
void zero_vehicle_update(ZeroVehicle *car, float dt, ZeroGroundHeightFn ground_fn);
void zero_vehicle_reset(ZeroVehicle *car, Vec3 spawn_pos);

#endif

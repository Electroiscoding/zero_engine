#ifndef ZERO_VEHICLE_PHYSICS_H
#define ZERO_VEHICLE_PHYSICS_H

#include "zero_core.h"
#include "zero_math.h"
#include "zero_vehicle.h"

#ifdef __cplusplus
extern "C" {
#endif

// Physical Constants & Vehicle Configuration
#define ZERO_VEHICLE_MASS             1150.0f     // kg
#define ZERO_SUSP_REST_LENGTH         0.55f       // meters
#define ZERO_SUSP_SPRING_K            44000.0f    // N/m
#define ZERO_SUSP_DAMPER_C            3600.0f     // Ns/m
#define ZERO_TIRE_RADIUS              0.37f       // meters
#define ZERO_ARB_STIFFNESS_FRONT      8500.0f     // N/m
#define ZERO_ARB_STIFFNESS_REAR       5200.0f     // N/m

#define ZERO_TIRE_LONG_STIFFNESS      75000.0f    // C_kappa (N/unit slip)
#define ZERO_TIRE_LAT_STIFF_FRONT     50000.0f    // C_alpha front (N/rad)
#define ZERO_TIRE_LAT_STIFF_REAR      46000.0f    // C_alpha rear (N/rad)
#define ZERO_TIRE_MU_STATIC           1.35f       // Tarmac peak friction coefficient
#define ZERO_TIRE_MU_KINETIC          0.35f       // Sliding kinetic friction under handbrake
#define ZERO_WHEEL_INERTIA            1.25f       // kg*m^2
#define ZERO_TIRE_SLIP_EPSILON        0.25f       // Regularization speed (m/s)

#define ZERO_ENGINE_IDLE_RPM          1000.0f
#define ZERO_ENGINE_PEAK_RPM          4500.0f
#define ZERO_ENGINE_PEAK_TORQUE       480.0f      // Nm
#define ZERO_ENGINE_REDLINE_RPM       7500.0f
#define ZERO_TRANS_SHIFT_TIME         0.18f       // seconds
#define ZERO_TRANS_LOCKOUT_TIME       0.50f       // seconds
#define ZERO_TRANS_TORQUE_CUT         0.40f       // Torque cut factor during shifting

typedef enum ZeroGear {
    ZERO_GEAR_REVERSE = -1,
    ZERO_GEAR_NEUTRAL =  0,
    ZERO_GEAR_1       =  1,
    ZERO_GEAR_2       =  2,
    ZERO_GEAR_3       =  3,
    ZERO_GEAR_4       =  4,
    ZERO_GEAR_5       =  5,
    ZERO_GEAR_6       =  6
} ZeroGear;

typedef struct ZeroTransmission {
    ZeroGear current_gear;
    ZeroGear target_gear;
    float gear_ratios[7];        // [0]=R(-3.40), [1]=1st(3.60), [2]=2nd(2.19), [3]=3rd(1.52), [4]=4th(1.18), [5]=5th(0.95), [6]=6th(0.78)
    float final_drive_ratio;     // 3.73
    float shift_timer;           // Countdown during gear shift (0.18s)
    float shift_lockout_timer;   // Prevents rapid shift oscillation (0.50s)
    float torque_cut_factor;     // 0.40 during shift, 1.00 when engaged
    bool is_shifting;
    bool reverse_requested;
} ZeroTransmission;

typedef struct ZeroVehicleEnhanced {
    // 6-DOF Chassis Rigid Body State
    Vec3 position;
    Vec3 velocity;
    Vec3 angular_velocity;
    Quat orientation;
    Mat4 transform;

    float mass;                  // 1150 kg
    Vec3 inertia;                // (1400, 950, 750) kg*m^2
    Vec3 inv_inertia;

    // Suspension & Wheels
    ZeroWheel wheels[4];
    float anti_roll_stiffness_front; // 8500 N/m
    float anti_roll_stiffness_rear;  // 5200 N/m

    // Powertrain & Transmission
    ZeroTransmission transmission;
    float engine_rpm;            // 1000 to 7500 RPM
    float engine_torque;         // Nm
    float drive_wheel_torque;    // Nm delivered to drive axle

    // Driver Controls
    float throttle_input;        // [0.0, 1.0]
    float brake_input;           // [0.0, 1.0]
    float steer_input;           // [-1.0, 1.0]
    bool handbrake;              // Rear wheel lockup

    // Telemetry & Progress
    float speed_kmh;
    float drift_angle;
    bool is_drifting;
    int lap_count;
    float current_lap_time;
    float best_lap_time;
    float track_progress_s;      // Arc length progress along spline circuit
    int race_position;           // 1st to 4th ranking

    // Aerodynamics Subsystem
    float aero_drag_coeff;       // CD * frontal area (nominal 0.704 m^2)
    float aero_front_downforce;  // Front splitter downforce coefficient (1.15 m^2)
    float aero_rear_downforce;   // Active rear wing downforce coefficient (1.65 m^2)
    float aero_diffuser_factor;  // Ground-effect underbody suction (0.85 m^2)
    float aero_airbrake_active;  // Dynamic spoiler airbrake deployment (0.0 to 1.0)
} ZeroVehicleEnhanced;

// Vehicle Lifecycle & Simulation API
ZeroVehicleEnhanced zero_vehicle_enhanced_create(Vec3 spawn_pos, Quat spawn_rot);
void zero_vehicle_enhanced_reset(ZeroVehicleEnhanced *car, Vec3 spawn_pos, Quat spawn_rot);
void zero_vehicle_enhanced_update(ZeroVehicleEnhanced *car, float dt, ZeroGroundHeightFn ground_fn);

// Powertrain & Transmission API
float zero_transmission_get_gear_ratio(const ZeroTransmission *trans, ZeroGear gear);
float zero_vehicle_compute_engine_torque(float rpm);
void zero_vehicle_update_transmission(ZeroVehicleEnhanced *car, float dt);

// Tire Friction Model & Drift Telemetry API
void zero_vehicle_apply_brush_tire_friction(
    ZeroVehicleEnhanced *car,
    ZeroWheel *w,
    float normal_force,
    float sub_dt,
    Vec3 *out_tire_force
);
void zero_vehicle_update_drift_telemetry(ZeroVehicleEnhanced *car);

// Multi-Element Aerodynamics API
void zero_vehicle_apply_aerodynamics(
    ZeroVehicleEnhanced *car,
    float sub_dt,
    Vec3 *inout_total_force,
    Vec3 *inout_total_torque
);

// Spline Barrier Boundary Collision Resolution API
void zero_vehicle_resolve_barrier_collision(ZeroVehicleEnhanced *car, ZeroGroundHeightFn ground_fn);

#ifdef __cplusplus
}
#endif

#endif // ZERO_VEHICLE_PHYSICS_H

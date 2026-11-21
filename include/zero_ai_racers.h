#ifndef ZERO_AI_RACERS_H
#define ZERO_AI_RACERS_H

#include "zero_core.h"
#include "zero_math.h"
#include "zero_physics.h"
#include "zero_track_spline.h"
#include "zero_vehicle.h"

#if __has_include("zero_vehicle_physics.h")
#include "zero_vehicle_physics.h"
#else
#ifndef ZERO_VEHICLE_PHYSICS_H
#define ZERO_VEHICLE_PHYSICS_H

typedef enum ZeroGear {
    ZERO_GEAR_REVERSE = -1,
    ZERO_GEAR_NEUTRAL = 0,
    ZERO_GEAR_1 = 1,
    ZERO_GEAR_2 = 2,
    ZERO_GEAR_3 = 3,
    ZERO_GEAR_4 = 4,
    ZERO_GEAR_5 = 5,
    ZERO_GEAR_6 = 6
} ZeroGear;

typedef struct ZeroTransmission {
    ZeroGear current_gear;
    ZeroGear target_gear;
    float gear_ratios[7];
    float final_drive_ratio;
    float shift_timer;
    float shift_lockout_timer;
    float torque_cut_factor;
    bool is_shifting;
    bool reverse_requested;
} ZeroTransmission;

typedef struct ZeroVehicleEnhanced {
    Vec3 position;
    Vec3 velocity;
    Vec3 angular_velocity;
    Quat orientation;
    Mat4 transform;
    float mass;
    Vec3 inertia;
    Vec3 inv_inertia;
    ZeroWheel wheels[4];
    float anti_roll_stiffness_front;
    float anti_roll_stiffness_rear;
    ZeroTransmission transmission;
    float engine_rpm;
    float engine_torque;
    float drive_wheel_torque;
    float throttle_input;
    float brake_input;
    float steer_input;
    bool handbrake;
    float speed_kmh;
    float drift_angle;
    bool is_drifting;
    int lap_count;
    float current_lap_time;
    float best_lap_time;
    float track_progress_s;
    int race_position;
    float aero_drag_coeff;
    float aero_front_downforce;
    float aero_rear_downforce;
    float aero_diffuser_factor;
    float aero_airbrake_active;
} ZeroVehicleEnhanced;

ZeroVehicleEnhanced zero_vehicle_enhanced_create(Vec3 spawn_pos, Quat spawn_rot);
void zero_vehicle_enhanced_update(ZeroVehicleEnhanced *car, float dt, ZeroGroundHeightFn ground_fn);

#endif // ZERO_VEHICLE_PHYSICS_H
#endif // __has_include("zero_vehicle_physics.h")

#define ZERO_RACE_CAR_COUNT 4

typedef struct ZeroAIController {
    uint32_t car_index;             // Index into cars array (1, 2, or 3)
    float aggressiveness;           // Cornering speed scaling (0.88 to 1.05)
    float preferred_lateral_offset; // Racing line preference (-2.0m to +2.0m)
    float current_lateral_offset;   // Dynamic offset accounting for avoidance
    float lookahead_distance;       // Speed-dependent pursuit distance
    int target_spline_node;         // Index of forward target node on track
    float avoidance_timer;          // Decay timer for avoidance maneuvers
    float smooth_steer;             // Filtered steering angle
    float smooth_throttle;          // Filtered throttle
    float smooth_brake;             // Filtered brake
} ZeroAIController;

typedef struct ZeroRaceGame {
    ZeroVehicleEnhanced cars[ZERO_RACE_CAR_COUNT]; // Car 0 = Player, Cars 1-3 = AI
    ZeroAIController ai_controllers[3];            // AI logic for Cars 1, 2, 3
    uint64_t tick;
    float race_time;
    bool race_started;
    bool race_finished;
    int total_laps;                                // Default 3 Laps
    uint32_t winning_car_index;
} ZeroRaceGame;

// Core Function Prototypes
ZeroAIController zero_ai_controller_create(uint32_t car_index, float aggressiveness, float preferred_offset);
void zero_ai_controller_update(ZeroAIController *ai, ZeroRaceGame *game, float dt);

float zero_track_get_spline_curvature(int node_idx);
float zero_track_get_max_corner_speed(int node_idx, float aggressiveness);
void zero_track_find_nearest_segment(Vec3 pos, int *out_node_idx, float *out_t, float *out_dist_sq);

ZeroRaceGame zero_race_game_create(void);
void zero_race_game_step(ZeroRaceGame *game, float dt);
void zero_race_game_resolve_car_collisions(ZeroRaceGame *game);
void zero_race_game_update_rankings(ZeroRaceGame *game);

#endif // ZERO_AI_RACERS_H

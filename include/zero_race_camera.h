#ifndef ZERO_RACE_CAMERA_H
#define ZERO_RACE_CAMERA_H

#include "zero_camera.h"
#include "zero_math.h"
#include <stdbool.h>

typedef enum ZeroCameraMode {
    ZERO_CAM_CHASE = 0,
    ZERO_CAM_HOOD,
    ZERO_CAM_COCKPIT,
    ZERO_CAM_BUMPER,
    ZERO_CAM_TV_BROADCAST,
    ZERO_CAM_MODE_COUNT
} ZeroCameraMode;

typedef struct ZeroRaceCamera {
    ZeroCamera base_cam;
    ZeroCameraMode mode;

    /* Spring-damper dynamic state */
    Vec3 current_pos;
    Vec3 current_velocity;
    Vec3 current_target;
    Vec3 current_target_vel;

    /* Dynamic parameters */
    float base_fov_deg;
    float current_fov_deg;
    float fov_dilation_rate;
    float spring_stiffness;
    float damping_ratio;

    /* Camera transition state */
    float transition_t;       /* 0.0 to 1.0, 1.0 = transition complete */
    Vec3 transition_start_pos;
    Vec3 transition_start_target;

    /* TV Trackside Camera parameters */
    int active_tv_tower_idx;

    /* Internal simulation state */
    float accum_time;
    Vec3 prev_car_velocity;
} ZeroRaceCamera;

ZeroRaceCamera zero_race_camera_create(float aspect);
void zero_race_camera_set_mode(ZeroRaceCamera *rcam, ZeroCameraMode mode, bool smooth_transition);
void zero_race_camera_cycle_mode(ZeroRaceCamera *rcam);
void zero_race_camera_update(ZeroRaceCamera *rcam,
                             Vec3 car_position, Quat car_orientation,
                             Vec3 car_velocity, float car_rpm, float steer_input,
                             float dt);
Mat4 zero_race_camera_get_view(const ZeroRaceCamera *rcam);
Mat4 zero_race_camera_get_projection(const ZeroRaceCamera *rcam);

#endif /* ZERO_RACE_CAMERA_H */

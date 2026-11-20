#ifndef ZERO_CAMERA_H
#define ZERO_CAMERA_H

#include "zero_core.h"
#include "zero_math.h"

typedef struct ZeroRay {
    Vec3 origin;
    Vec3 direction;
} ZeroRay;

typedef struct ZeroCamera {
    Vec3 position;
    Vec3 forward;
    Vec3 up;
    Vec3 right;
    float pitch;
    float yaw;
    float fov;
    float aspect;
    float near_z;
    float far_z;
} ZeroCamera;

ZeroCamera zero_camera_create(Vec3 pos, float fov, float aspect, float near_z, float far_z);
void zero_camera_update_vectors(ZeroCamera *cam);
void zero_camera_rotate(ZeroCamera *cam, float delta_pitch, float delta_yaw);
void zero_camera_move(ZeroCamera *cam, Vec3 movement);
Mat4 zero_camera_get_view(const ZeroCamera *cam);
Mat4 zero_camera_get_projection(const ZeroCamera *cam);
ZeroRay zero_camera_screen_to_ray(const ZeroCamera *cam, float screen_x, float screen_y, float width, float height);

#endif

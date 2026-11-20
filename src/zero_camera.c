#include "zero_camera.h"
#include <math.h>

ZeroCamera zero_camera_create(Vec3 pos, float fov, float aspect, float near_z, float far_z) {
    ZeroCamera cam;
    cam.position = pos;
    cam.pitch = 0.0f;
    cam.yaw = -90.0f;
    cam.fov = fov;
    cam.aspect = aspect;
    cam.near_z = near_z;
    cam.far_z = far_z;
    zero_camera_update_vectors(&cam);
    return cam;
}

void zero_camera_update_vectors(ZeroCamera *cam) {
    if (!cam) return;

    float rad_pitch = ZERO_DEG2RAD(cam->pitch);
    float rad_yaw = ZERO_DEG2RAD(cam->yaw);

    Vec3 front;
    front.x = cosf(rad_yaw) * cosf(rad_pitch);
    front.y = sinf(rad_pitch);
    front.z = sinf(rad_yaw) * cosf(rad_pitch);

    cam->forward = vec3_normalize(front);
    cam->right = vec3_normalize(vec3_cross(cam->forward, vec3(0.0f, 1.0f, 0.0f)));
    cam->up = vec3_normalize(vec3_cross(cam->right, cam->forward));
}

void zero_camera_rotate(ZeroCamera *cam, float delta_pitch, float delta_yaw) {
    if (!cam) return;

    cam->pitch += delta_pitch;
    cam->yaw += delta_yaw;

    if (cam->pitch > 89.0f) cam->pitch = 89.0f;
    if (cam->pitch < -89.0f) cam->pitch = -89.0f;

    zero_camera_update_vectors(cam);
}

void zero_camera_move(ZeroCamera *cam, Vec3 movement) {
    if (cam) {
        cam->position = vec3_add(cam->position, movement);
    }
}

Mat4 zero_camera_get_view(const ZeroCamera *cam) {
    if (!cam) return mat4_identity();
    Vec3 target = vec3_add(cam->position, cam->forward);
    return mat4_look_at(cam->position, target, cam->up);
}

Mat4 zero_camera_get_projection(const ZeroCamera *cam) {
    if (!cam) return mat4_identity();
    return mat4_perspective(cam->fov, cam->aspect, cam->near_z, cam->far_z);
}

static Vec4 mat4_mul_vec4(Mat4 m, Vec4 v) {
    Vec4 res;
    res.x = m.m[0] * v.x + m.m[4] * v.y + m.m[8]  * v.z + m.m[12] * v.w;
    res.y = m.m[1] * v.x + m.m[5] * v.y + m.m[9]  * v.z + m.m[13] * v.w;
    res.z = m.m[2] * v.x + m.m[6] * v.y + m.m[10] * v.z + m.m[14] * v.w;
    res.w = m.m[3] * v.x + m.m[7] * v.y + m.m[11] * v.z + m.m[15] * v.w;
    return res;
}

ZeroRay zero_camera_screen_to_ray(const ZeroCamera *cam, float screen_x, float screen_y, float width, float height) {
    ZeroRay ray = {vec3(0, 0, 0), vec3(0, 0, -1)};
    if (!cam || width <= 0.0f || height <= 0.0f) {
        return ray;
    }

    float ndc_x = (2.0f * screen_x) / width - 1.0f;
    float ndc_y = 1.0f - (2.0f * screen_y) / height;

    Mat4 view = zero_camera_get_view(cam);
    Mat4 proj = zero_camera_get_projection(cam);
    Mat4 inv_vp = mat4_inverse(mat4_mul(proj, view));

    Vec4 near_clip = vec4(ndc_x, ndc_y, -1.0f, 1.0f);
    Vec4 far_clip  = vec4(ndc_x, ndc_y,  1.0f, 1.0f);

    Vec4 near_world_4 = mat4_mul_vec4(inv_vp, near_clip);
    Vec4 far_world_4  = mat4_mul_vec4(inv_vp, far_clip);

    if (fabsf(near_world_4.w) > 1e-6f && fabsf(far_world_4.w) > 1e-6f) {
        Vec3 near_world = vec3(near_world_4.x / near_world_4.w, near_world_4.y / near_world_4.w, near_world_4.z / near_world_4.w);
        Vec3 far_world  = vec3(far_world_4.x / far_world_4.w, far_world_4.y / far_world_4.w, far_world_4.z / far_world_4.w);

        ray.origin = near_world;
        ray.direction = vec3_normalize(vec3_sub(far_world, near_world));
    }

    return ray;
}

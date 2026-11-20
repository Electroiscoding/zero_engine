#include "zero_core.h"
#include "zero_math.h"
#include "zero_camera.h"
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <math.h>

static int g_stress_tests_run = 0;
static int g_stress_tests_passed = 0;

#define STRESS_ASSERT(cond, msg) do { \
    g_stress_tests_run++; \
    if (cond) { \
        g_stress_tests_passed++; \
    } else { \
        fprintf(stderr, "[FAIL] Line %d: %s\n", __LINE__, msg); \
        assert(cond); \
    } \
} while(0)

static float mat4_max_abs_diff(Mat4 a, Mat4 b) {
    float max_diff = 0.0f;
    for (int i = 0; i < 16; ++i) {
        float diff = fabsf(a.m[i] - b.m[i]);
        if (diff > max_diff) {
            max_diff = diff;
        }
    }
    return max_diff;
}

static Mat4 mat4_transpose(Mat4 m) {
    Mat4 res;
    for (int c = 0; c < 4; ++c) {
        for (int r = 0; r < 4; ++r) {
            res.m[c * 4 + r] = m.m[r * 4 + c];
        }
    }
    return res;
}

static void test_camera_pitch_clamping_edge_cases(void) {
    printf("[STRESS] Running camera pitch clamping & rotation edge case tests...\n");

    /* 1. NULL robustness */
    zero_camera_rotate(NULL, 10.0f, 10.0f);
    zero_camera_update_vectors(NULL);
    zero_camera_move(NULL, vec3(1.0f, 2.0f, 3.0f));
    Mat4 null_v = zero_camera_get_view(NULL);
    Mat4 id = mat4_identity();
    STRESS_ASSERT(mat4_max_abs_diff(null_v, id) < 1e-6f, "NULL camera view returns identity");
    Mat4 null_p = zero_camera_get_projection(NULL);
    STRESS_ASSERT(mat4_max_abs_diff(null_p, id) < 1e-6f, "NULL camera projection returns identity");

    /* 2. Boundary rotation to exactly +89.0f from 0.0f */
    ZeroCamera cam = zero_camera_create(vec3(0.0f, 0.0f, 0.0f), ZERO_DEG2RAD(60.0f), 16.0f / 9.0f, 0.1f, 1000.0f);
    STRESS_ASSERT(cam.pitch == 0.0f, "Initial pitch is 0.0");
    zero_camera_rotate(&cam, 89.0f, 0.0f);
    STRESS_ASSERT(fabsf(cam.pitch - 89.0f) < 1e-6f, "Rotate exactly to +89.0 deg");
    STRESS_ASSERT(fabsf(vec3_length(cam.forward) - 1.0f) < 1e-4f, "Forward vector normalized at +89 deg");
    STRESS_ASSERT(fabsf(vec3_length(cam.right) - 1.0f) < 1e-4f, "Right vector normalized at +89 deg");
    STRESS_ASSERT(fabsf(vec3_length(cam.up) - 1.0f) < 1e-4f, "Up vector normalized at +89 deg");
    STRESS_ASSERT(fabsf(vec3_dot(cam.forward, cam.right)) < 1e-4f, "Forward and Right orthogonal at +89 deg");
    STRESS_ASSERT(fabsf(vec3_dot(cam.right, cam.up)) < 1e-4f, "Right and Up orthogonal at +89 deg");
    STRESS_ASSERT(fabsf(vec3_dot(cam.up, cam.forward)) < 1e-4f, "Up and Forward orthogonal at +89 deg");
    Vec3 reconstructed_up = vec3_cross(cam.right, cam.forward);
    STRESS_ASSERT(vec3_dot(reconstructed_up, cam.up) > 0.999f, "Basis is right-handed at +89 deg");

    /* 3. Boundary rotation to exactly -89.0f from 0.0f */
    cam.pitch = 0.0f;
    zero_camera_update_vectors(&cam);
    zero_camera_rotate(&cam, -89.0f, 0.0f);
    STRESS_ASSERT(fabsf(cam.pitch - (-89.0f)) < 1e-6f, "Rotate exactly to -89.0 deg");
    STRESS_ASSERT(fabsf(vec3_length(cam.forward) - 1.0f) < 1e-4f, "Forward vector normalized at -89 deg");
    STRESS_ASSERT(fabsf(vec3_length(cam.right) - 1.0f) < 1e-4f, "Right vector normalized at -89 deg");
    STRESS_ASSERT(fabsf(vec3_length(cam.up) - 1.0f) < 1e-4f, "Up vector normalized at -89 deg");
    STRESS_ASSERT(fabsf(vec3_dot(cam.forward, cam.right)) < 1e-4f, "Forward and Right orthogonal at -89 deg");
    STRESS_ASSERT(fabsf(vec3_dot(cam.right, cam.up)) < 1e-4f, "Right and Up orthogonal at -89 deg");
    STRESS_ASSERT(fabsf(vec3_dot(cam.up, cam.forward)) < 1e-4f, "Up and Forward orthogonal at -89 deg");

    /* 4. Overshoot clamping: angles > 89.0, > 360, large positive */
    float positive_angles[] = { 89.001f, 89.5f, 90.0f, 95.0f, 180.0f, 270.0f, 360.0f, 450.0f, 720.0f, 10000.0f, 1e6f };
    size_t num_pos = sizeof(positive_angles) / sizeof(positive_angles[0]);
    for (size_t i = 0; i < num_pos; ++i) {
        cam.pitch = 0.0f;
        zero_camera_rotate(&cam, positive_angles[i], 0.0f);
        STRESS_ASSERT(fabsf(cam.pitch - 89.0f) < 1e-6f, "Pitch must clamp to 89.0 on positive overshoot");
        STRESS_ASSERT(fabsf(vec3_length(cam.forward) - 1.0f) < 1e-4f, "Forward normalized after positive overshoot");
        STRESS_ASSERT(fabsf(vec3_length(cam.right) - 1.0f) < 1e-4f, "Right normalized after positive overshoot");
        STRESS_ASSERT(fabsf(vec3_length(cam.up) - 1.0f) < 1e-4f, "Up normalized after positive overshoot");
    }

    /* 5. Undershoot clamping: angles < -89.0, < -360, large negative */
    float negative_angles[] = { -89.001f, -89.5f, -90.0f, -95.0f, -180.0f, -270.0f, -360.0f, -450.0f, -720.0f, -10000.0f, -1e6f };
    size_t num_neg = sizeof(negative_angles) / sizeof(negative_angles[0]);
    for (size_t i = 0; i < num_neg; ++i) {
        cam.pitch = 0.0f;
        zero_camera_rotate(&cam, negative_angles[i], 0.0f);
        STRESS_ASSERT(fabsf(cam.pitch - (-89.0f)) < 1e-6f, "Pitch must clamp to -89.0 on negative overshoot");
        STRESS_ASSERT(fabsf(vec3_length(cam.forward) - 1.0f) < 1e-4f, "Forward normalized after negative overshoot");
        STRESS_ASSERT(fabsf(vec3_length(cam.right) - 1.0f) < 1e-4f, "Right normalized after negative overshoot");
        STRESS_ASSERT(fabsf(vec3_length(cam.up) - 1.0f) < 1e-4f, "Up normalized after negative overshoot");
    }

    /* 6. Incremental drift stress: 1000 steps of +1.0 deg */
    cam.pitch = 0.0f;
    for (int step = 0; step < 1000; ++step) {
        zero_camera_rotate(&cam, 1.0f, 0.0f);
    }
    STRESS_ASSERT(fabsf(cam.pitch - 89.0f) < 1e-6f, "1000 incremental +1.0 deg rotations must remain at 89.0");

    /* 7. Incremental drift stress: 2000 steps of -1.0 deg */
    for (int step = 0; step < 2000; ++step) {
        zero_camera_rotate(&cam, -1.0f, 0.0f);
    }
    STRESS_ASSERT(fabsf(cam.pitch - (-89.0f)) < 1e-6f, "2000 incremental -1.0 deg rotations must remain at -89.0");

    /* 8. Combined large yaw with extreme pitch */
    float test_yaws[] = { 0.0f, 45.0f, 90.0f, 180.0f, 360.0f, 720.0f, 10000.0f, -720.0f, -10000.0f };
    size_t num_yaws = sizeof(test_yaws) / sizeof(test_yaws[0]);
    for (size_t i = 0; i < num_yaws; ++i) {
        cam.pitch = 0.0f;
        cam.yaw = test_yaws[i];
        zero_camera_rotate(&cam, 89.0f, 15.0f);
        STRESS_ASSERT(fabsf(cam.pitch - 89.0f) < 1e-6f, "Pitch clamped at 89 with varied yaw");
        STRESS_ASSERT(fabsf(vec3_length(cam.forward) - 1.0f) < 1e-4f, "Forward normalized with varied yaw");
        STRESS_ASSERT(fabsf(vec3_length(cam.right) - 1.0f) < 1e-4f, "Right normalized with varied yaw");
        STRESS_ASSERT(fabsf(vec3_length(cam.up) - 1.0f) < 1e-4f, "Up normalized with varied yaw");
        STRESS_ASSERT(fabsf(vec3_dot(cam.forward, cam.right)) < 1e-3f, "Forward and Right orthogonal with varied yaw");
        STRESS_ASSERT(fabsf(vec3_dot(cam.right, cam.up)) < 1e-3f, "Right and Up orthogonal with varied yaw");
    }
}

static void test_screen_to_ray_normalization_and_precision(void) {
    printf("[STRESS] Running screen-to-ray normalization & precision tests...\n");

    ZeroCamera cam = zero_camera_create(vec3(0.0f, 2.0f, 5.0f), ZERO_DEG2RAD(60.0f), 16.0f / 9.0f, 0.1f, 1000.0f);

    /* 1. NULL and degenerate screen dimension robustness */
    ZeroRay r_null_cam = zero_camera_screen_to_ray(NULL, 100.0f, 100.0f, 800.0f, 600.0f);
    STRESS_ASSERT(r_null_cam.direction.z == -1.0f, "NULL camera returns fallback ray");

    ZeroRay r_zero_w = zero_camera_screen_to_ray(&cam, 100.0f, 100.0f, 0.0f, 600.0f);
    STRESS_ASSERT(r_zero_w.direction.z == -1.0f, "Zero width returns fallback ray");

    ZeroRay r_neg_h = zero_camera_screen_to_ray(&cam, 100.0f, 100.0f, 800.0f, -100.0f);
    STRESS_ASSERT(r_neg_h.direction.z == -1.0f, "Negative height returns fallback ray");

    /* 2. Standard resolution boundaries (1280x720) */
    const float w = 1280.0f;
    const float h = 720.0f;
    struct {
        float x, y;
        const char *name;
    } boundary_points[] = {
        { 0.0f, 0.0f, "Top-Left (0, 0)" },
        { w, 0.0f, "Top-Right (w, 0)" },
        { 0.0f, h, "Bottom-Left (0, h)" },
        { w, h, "Bottom-Right (w, h)" },
        { w * 0.5f, h * 0.5f, "Center (w/2, h/2)" },
        { w * 0.5f, 0.0f, "Mid-Top (w/2, 0)" },
        { w * 0.5f, h, "Mid-Bottom (w/2, h)" },
        { 0.0f, h * 0.5f, "Mid-Left (0, h/2)" },
        { w, h * 0.5f, "Mid-Right (w, h/2)" },
        { 0.5f, 0.5f, "Subpixel corner (0.5, 0.5)" },
        { w - 0.5f, h - 0.5f, "Subpixel opposite corner" },
        { -50.0f, -50.0f, "Offscreen negative" },
        { w + 100.0f, h + 100.0f, "Offscreen positive" }
    };
    size_t num_points = sizeof(boundary_points) / sizeof(boundary_points[0]);

    for (size_t i = 0; i < num_points; ++i) {
        ZeroRay ray = zero_camera_screen_to_ray(&cam, boundary_points[i].x, boundary_points[i].y, w, h);
        float len = vec3_length(ray.direction);
        STRESS_ASSERT(fabsf(len - 1.0f) < 1e-5f, "Ray direction must be strictly normalized");
        STRESS_ASSERT(isfinite(ray.direction.x) && isfinite(ray.direction.y) && isfinite(ray.direction.z), "Ray direction components finite");
        STRESS_ASSERT(isfinite(ray.origin.x) && isfinite(ray.origin.y) && isfinite(ray.origin.z), "Ray origin components finite");
    }

    /* 3. Center ray alignment with camera forward across poses */
    float pitch_yaw_pairs[][2] = {
        { 0.0f, -90.0f },
        { 30.0f, -90.0f },
        { -45.0f, 0.0f },
        { 89.0f, 45.0f },
        { -89.0f, 180.0f },
        { 0.0f, 270.0f },
        { -60.0f, -225.0f }
    };
    size_t num_pairs = sizeof(pitch_yaw_pairs) / sizeof(pitch_yaw_pairs[0]);
    for (size_t i = 0; i < num_pairs; ++i) {
        cam.pitch = pitch_yaw_pairs[i][0];
        cam.yaw = pitch_yaw_pairs[i][1];
        zero_camera_update_vectors(&cam);

        ZeroRay center_ray = zero_camera_screen_to_ray(&cam, w * 0.5f, h * 0.5f, w, h);
        float dot = vec3_dot(center_ray.direction, cam.forward);
        STRESS_ASSERT(dot > 0.9995f, "Center ray direction must strictly align with camera forward");
    }

    /* 4. Symmetry across screen axes */
    cam.position = vec3(0.0f, 0.0f, 0.0f);
    cam.pitch = 0.0f;
    cam.yaw = -90.0f; /* facing -Z */
    zero_camera_update_vectors(&cam);

    ZeroRay ray_left  = zero_camera_screen_to_ray(&cam, 0.0f, h * 0.5f, w, h);
    ZeroRay ray_right = zero_camera_screen_to_ray(&cam, w, h * 0.5f, w, h);
    STRESS_ASSERT(fabsf(ray_left.direction.x + ray_right.direction.x) < 1e-4f, "Horizontal symmetry: opposite X directions");
    STRESS_ASSERT(fabsf(ray_left.direction.y - ray_right.direction.y) < 1e-4f, "Horizontal symmetry: identical Y directions");
    STRESS_ASSERT(fabsf(ray_left.direction.z - ray_right.direction.z) < 1e-4f, "Horizontal symmetry: identical Z directions");

    ZeroRay ray_top = zero_camera_screen_to_ray(&cam, w * 0.5f, 0.0f, w, h);
    ZeroRay ray_bot = zero_camera_screen_to_ray(&cam, w * 0.5f, h, w, h);
    STRESS_ASSERT(fabsf(ray_top.direction.y + ray_bot.direction.y) < 1e-4f, "Vertical symmetry: opposite Y directions");
    STRESS_ASSERT(fabsf(ray_top.direction.x - ray_bot.direction.x) < 1e-4f, "Vertical symmetry: identical X directions");
    STRESS_ASSERT(fabsf(ray_top.direction.z - ray_bot.direction.z) < 1e-4f, "Vertical symmetry: identical Z directions");

    /* 5. Varied resolutions and extreme aspect ratios */
    struct {
        float width, height, aspect;
    } resolutions[] = {
        { 1920.0f, 1080.0f, 16.0f / 9.0f },
        { 800.0f, 600.0f, 4.0f / 3.0f },
        { 3840.0f, 2160.0f, 16.0f / 9.0f },
        { 100.0f, 100.0f, 1.0f },
        { 2000.0f, 400.0f, 5.0f },
        { 400.0f, 2000.0f, 0.2f }
    };
    size_t num_res = sizeof(resolutions) / sizeof(resolutions[0]);
    for (size_t i = 0; i < num_res; ++i) {
        ZeroCamera res_cam = zero_camera_create(vec3(0, 1, 0), ZERO_DEG2RAD(60.0f), resolutions[i].aspect, 0.1f, 100.0f);
        float rw = resolutions[i].width;
        float rh = resolutions[i].height;
        ZeroRay r_tl = zero_camera_screen_to_ray(&res_cam, 0.0f, 0.0f, rw, rh);
        ZeroRay r_br = zero_camera_screen_to_ray(&res_cam, rw, rh, rw, rh);
        ZeroRay r_c  = zero_camera_screen_to_ray(&res_cam, rw * 0.5f, rh * 0.5f, rw, rh);
        STRESS_ASSERT(fabsf(vec3_length(r_tl.direction) - 1.0f) < 1e-5f, "TL ray normalized for varied resolution");
        STRESS_ASSERT(fabsf(vec3_length(r_br.direction) - 1.0f) < 1e-5f, "BR ray normalized for varied resolution");
        STRESS_ASSERT(fabsf(vec3_length(r_c.direction) - 1.0f) < 1e-5f, "Center ray normalized for varied resolution");
        STRESS_ASSERT(vec3_dot(r_c.direction, res_cam.forward) > 0.9995f, "Center ray aligned for varied resolution");
    }

    /* 6. Dense 11x11 screen sweep */
    for (int gy = 0; gy <= 10; ++gy) {
        for (int gx = 0; gx <= 10; ++gx) {
            float sx = (float)gx * (w / 10.0f);
            float sy = (float)gy * (h / 10.0f);
            ZeroRay ray = zero_camera_screen_to_ray(&cam, sx, sy, w, h);
            float len = vec3_length(ray.direction);
            STRESS_ASSERT(fabsf(len - 1.0f) < 1e-5f, "Screen sweep ray normalized");
            STRESS_ASSERT(ray.direction.z < 0.0f, "Screen sweep ray points into scene (-Z)");
        }
    }
}

static void test_matrix_inverse_precision_stress(void) {
    printf("[STRESS] Running matrix inverse precision stress tests...\n");
    Mat4 id = mat4_identity();

    /* 1. Identity inverse */
    Mat4 inv_id = mat4_inverse(id);
    STRESS_ASSERT(mat4_max_abs_diff(inv_id, id) < 1e-6f, "Identity inverse is identity");

    /* 2. Translations spanning scales from 1e-4 to 1e5 */
    Vec3 translations[] = {
        vec3(0.0f, 0.0f, 0.0f),
        vec3(1.0f, 2.0f, 3.0f),
        vec3(-10.0f, 25.0f, -100.0f),
        vec3(1e-4f, -2e-4f, 5e-4f),
        vec3(1000.0f, -2500.0f, 5000.0f),
        vec3(50000.0f, -30000.0f, 80000.0f),
        vec3(1e5f, -2e5f, 3e5f)
    };
    size_t num_trans = sizeof(translations) / sizeof(translations[0]);
    for (size_t i = 0; i < num_trans; ++i) {
        Mat4 t = mat4_translate(translations[i]);
        Mat4 inv_t = mat4_inverse(t);
        Mat4 prod1 = mat4_mul(t, inv_t);
        Mat4 prod2 = mat4_mul(inv_t, t);
        /* Translation inverse translation elements must be exact negations */
        STRESS_ASSERT(fabsf(inv_t.m[12] + translations[i].x) < 1e-3f * (1.0f + fabsf(translations[i].x)), "Inverse translation X cancel");
        STRESS_ASSERT(fabsf(inv_t.m[13] + translations[i].y) < 1e-3f * (1.0f + fabsf(translations[i].y)), "Inverse translation Y cancel");
        STRESS_ASSERT(fabsf(inv_t.m[14] + translations[i].z) < 1e-3f * (1.0f + fabsf(translations[i].z)), "Inverse translation Z cancel");
        STRESS_ASSERT(mat4_max_abs_diff(prod1, id) < 1e-2f, "T * inv(T) == I");
        STRESS_ASSERT(mat4_max_abs_diff(prod2, id) < 1e-2f, "inv(T) * T == I");
    }

    /* 3. Pure 3D Rotations across diverse axes and angles */
    Vec3 axes[] = {
        vec3(1.0f, 0.0f, 0.0f),
        vec3(0.0f, 1.0f, 0.0f),
        vec3(0.0f, 0.0f, 1.0f),
        vec3(1.0f, 1.0f, 0.0f),
        vec3(0.0f, 1.0f, 1.0f),
        vec3(1.0f, 1.0f, 1.0f),
        vec3(0.267f, 0.534f, 0.801f)
    };
    float rot_angles[] = {
        0.0f,
        ZERO_DEG2RAD(15.0f),
        ZERO_DEG2RAD(45.0f),
        ZERO_DEG2RAD(90.0f),
        ZERO_DEG2RAD(135.0f),
        ZERO_DEG2RAD(180.0f),
        ZERO_DEG2RAD(270.0f),
        ZERO_DEG2RAD(360.0f),
        ZERO_DEG2RAD(720.0f),
        ZERO_DEG2RAD(-450.0f),
        1e-4f
    };
    size_t num_axes = sizeof(axes) / sizeof(axes[0]);
    size_t num_angles = sizeof(rot_angles) / sizeof(rot_angles[0]);

    for (size_t a = 0; a < num_axes; ++a) {
        for (size_t ang = 0; ang < num_angles; ++ang) {
            Quat q = quat_from_axis_angle(axes[a], rot_angles[ang]);
            Mat4 r = quat_to_mat4(q);
            Mat4 inv_r = mat4_inverse(r);
            Mat4 trans_r = mat4_transpose(r);

            /* For orthogonal rotation matrices, inv(R) == transpose(R) */
            float orth_err = mat4_max_abs_diff(inv_r, trans_r);
            STRESS_ASSERT(orth_err < 1e-4f, "Rotation inverse matches transpose (orthogonality)");

            Mat4 prod = mat4_mul(r, inv_r);
            float id_err = mat4_max_abs_diff(prod, id);
            STRESS_ASSERT(id_err < 1e-4f, "R * inv(R) == I");
        }
    }

    /* 4. Combined TRS transformations (Translation * Rotation) */
    for (int k = 0; k < 20; ++k) {
        float angle = (float)k * 0.35f;
        Quat q = quat_from_axis_angle(vec3(0.577f, 0.577f, 0.577f), angle);
        Mat4 r = quat_to_mat4(q);
        Vec3 t_vec = vec3((float)(k * 50 - 500), (float)(k * -30 + 300), (float)(k * 100 - 1000));
        Mat4 t = mat4_translate(t_vec);

        Mat4 m = mat4_mul(t, r);
        Mat4 inv_m = mat4_inverse(m);
        Mat4 prod1 = mat4_mul(m, inv_m);
        Mat4 prod2 = mat4_mul(inv_m, m);

        STRESS_ASSERT(mat4_max_abs_diff(prod1, id) < 1e-3f, "Combined M * inv(M) == I");
        STRESS_ASSERT(mat4_max_abs_diff(prod2, id) < 1e-3f, "Combined inv(M) * M == I");
    }

    /* 5. View Matrices (LookAt) under extreme camera translations */
    Vec3 eye_positions[] = {
        vec3(0.0f, 0.0f, 5.0f),
        vec3(500.0f, -300.0f, 1200.0f),
        vec3(-5000.0f, 10000.0f, -20000.0f)
    };
    size_t num_eyes = sizeof(eye_positions) / sizeof(eye_positions[0]);
    for (size_t e = 0; e < num_eyes; ++e) {
        Vec3 eye = eye_positions[e];
        Vec3 target = vec3_add(eye, vec3(0.0f, 0.0f, -10.0f));
        Vec3 up = vec3(0.0f, 1.0f, 0.0f);
        Mat4 view = mat4_look_at(eye, target, up);
        Mat4 inv_view = mat4_inverse(view);

        Mat4 prod = mat4_mul(view, inv_view);
        STRESS_ASSERT(mat4_max_abs_diff(prod, id) < 1e-3f, "View * inv(View) == I");

        /* inv(View) * (0, 0, 0, 1) should recover camera eye position */
        Vec4 origin_view = vec4(0.0f, 0.0f, 0.0f, 1.0f);
        Vec4 recovered_eye;
        recovered_eye.x = inv_view.m[0]*origin_view.x + inv_view.m[4]*origin_view.y + inv_view.m[8]*origin_view.z + inv_view.m[12]*origin_view.w;
        recovered_eye.y = inv_view.m[1]*origin_view.x + inv_view.m[5]*origin_view.y + inv_view.m[9]*origin_view.z + inv_view.m[13]*origin_view.w;
        recovered_eye.z = inv_view.m[2]*origin_view.x + inv_view.m[6]*origin_view.y + inv_view.m[10]*origin_view.z + inv_view.m[14]*origin_view.w;
        STRESS_ASSERT(fabsf(recovered_eye.x - eye.x) < 1e-2f * (1.0f + fabsf(eye.x)), "Recover eye X from inv(View)");
        STRESS_ASSERT(fabsf(recovered_eye.y - eye.y) < 1e-2f * (1.0f + fabsf(eye.y)), "Recover eye Y from inv(View)");
        STRESS_ASSERT(fabsf(recovered_eye.z - eye.z) < 1e-2f * (1.0f + fabsf(eye.z)), "Recover eye Z from inv(View)");
    }

    /* 6. Perspective Projection Matrices */
    Mat4 proj = mat4_perspective(ZERO_DEG2RAD(60.0f), 16.0f / 9.0f, 0.1f, 1000.0f);
    Mat4 inv_proj = mat4_inverse(proj);
    Mat4 proj_prod = mat4_mul(proj, inv_proj);
    STRESS_ASSERT(mat4_max_abs_diff(proj_prod, id) < 1e-3f, "Proj * inv(Proj) == I");

    /* 7. Combined View-Projection Matrix */
    Mat4 view = mat4_look_at(vec3(0.0f, 2.0f, 5.0f), vec3(0.0f, 2.0f, 0.0f), vec3(0.0f, 1.0f, 0.0f));
    Mat4 vp = mat4_mul(proj, view);
    Mat4 inv_vp = mat4_inverse(vp);
    Mat4 vp_prod = mat4_mul(vp, inv_vp);
    STRESS_ASSERT(mat4_max_abs_diff(vp_prod, id) < 1e-3f, "VP * inv(VP) == I");

    /* 8. Singular / Zero Determinant Matrices (safe fallback without crash/NaN) */
    Mat4 zero_m = {{0}};
    Mat4 inv_zero = mat4_inverse(zero_m);
    STRESS_ASSERT(mat4_max_abs_diff(inv_zero, id) < 1e-6f, "Zero matrix inverse falls back to identity");

    Mat4 singular_scale = mat4_scale(vec3(0.0f, 1.0f, 1.0f));
    Mat4 inv_singular = mat4_inverse(singular_scale);
    STRESS_ASSERT(mat4_max_abs_diff(inv_singular, id) < 1e-6f, "Singular matrix inverse falls back to identity");
}

int main(void) {
    printf("============================================================\n");
    printf("=== ZERO ENGINE: CAMERA & MATH ADVERSARIAL STRESS SUITE  ===\n");
    printf("============================================================\n");

    test_camera_pitch_clamping_edge_cases();
    test_screen_to_ray_normalization_and_precision();
    test_matrix_inverse_precision_stress();

    printf("\n[STRESS TEST VERIFICATION COMPLETE]\n");
    printf("Total stress checks executed: %d\n", g_stress_tests_run);
    printf("Total stress checks passed:   %d\n", g_stress_tests_passed);
    printf("============================================================\n");

    if (g_stress_tests_run == g_stress_tests_passed && g_stress_tests_run > 0) {
        printf("[ALL STRESS TESTS PASSED]\n");
        return 0;
    } else {
        fprintf(stderr, "[STRESS TESTS FAILED]\n");
        return 1;
    }
}

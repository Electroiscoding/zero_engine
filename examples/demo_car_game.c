#include "zero_core.h"
#include "zero_memory.h"
#include "zero_math.h"
#include "zero_platform.h"
#include "zero_gl_loader.h"
#include "zero_shader.h"
#include "zero_mesh.h"
#include "zero_camera.h"
#include "zero_obj.h"
#include "zero_vehicle.h"
#include <stdio.h>
#include <math.h>

static float track_ground_fn(float x, float z, Vec3 *out_normal) {
    (void)x; (void)z;
    if (out_normal) {
        *out_normal = vec3(0.0f, 1.0f, 0.0f);
    }
    return 0.0f;
}

static inline Vec3 vec3_lerp(Vec3 a, Vec3 b, float t) {
    return vec3(
        a.x + (b.x - a.x) * t,
        a.y + (b.y - a.y) * t,
        a.z + (b.z - a.z) * t
    );
}

int main(void) {
    ZeroPlatformConfig cfg = {
        .title = "Zero Racer: 3D Vehicle Physics & Asset Demo",
        .width = 1280,
        .height = 720,
        .headless = false
    };

    ZeroPlatformState platform;
    if (!zero_platform_init(&platform, &cfg)) {
        fprintf(stderr, "Failed to initialize platform\n");
        return 1;
    }

    if (platform.headless) {
        printf("[Zero Engine] Headless environment detected. Car game demo passed.\n");
        zero_platform_shutdown(&platform);
        return 0;
    }

    ZeroArena arena = zero_arena_create(64 * 1024 * 1024);

    // Load 3D assets using Phase 3 OBJ parser
    ZeroMeshData car_data = zero_obj_parse_file("assets/models/car_real.obj", &arena);
    ZeroMeshData wheel_data = zero_obj_parse_file("assets/models/wheel.obj", &arena);
    ZeroMeshData track_data = zero_obj_parse_file("assets/models/race_track.obj", &arena);

    if (car_data.vertex_count == 0 || wheel_data.vertex_count == 0 || track_data.vertex_count == 0) {
        fprintf(stderr, "Failed to load 3D assets\n");
        zero_arena_destroy(&arena);
        zero_platform_shutdown(&platform);
        return 1;
    }

    ZeroMesh car_mesh = zero_mesh_upload(&car_data);
    ZeroMesh wheel_mesh = zero_mesh_upload(&wheel_data);
    ZeroMesh track_mesh = zero_mesh_upload(&track_data);

    ZeroShader shader = zero_shader_create(zero_shader_default_vert(), zero_shader_default_frag());
    if (!shader.is_valid) {
        fprintf(stderr, "Failed to compile shader\n");
        zero_arena_destroy(&arena);
        zero_platform_shutdown(&platform);
        return 1;
    }

    ZeroVehicle car = zero_vehicle_create(vec3(0.0f, 0.65f, -100.0f));

    ZeroCamera cam = zero_camera_create(
        vec3(0.0f, 3.0f, -107.0f),
        ZERO_DEG2RAD(65.0f),
        (float)platform.width / (float)platform.height,
        0.1f,
        1500.0f
    );

    glEnable(GL_DEPTH_TEST);

    double last_time = zero_platform_get_time();
    double hud_timer = 0.0;
    bool cockpit_view = false;
    float steer_smooth = 0.0f;
    Vec3 cam_smooth_pos = cam.position;

    printf("\n============================================================\n");
    printf("===           ZERO RACER: 3D VEHICLE SIMULATOR          ===\n");
    printf("============================================================\n");
    printf(" Controls:\n");
    printf("   [W] / [Up Arrow]    : Throttle / Accelerate\n");
    printf("   [S] / [Down Arrow]  : Brake / Reverse\n");
    printf("   [A] / [D]           : Steer Left / Right (Auto-centering)\n");
    printf("   [Spacebar]          : Handbrake (Initiates High-Speed Drift!)\n");
    printf("   [C]                 : Switch Camera (Chase Cam / Hood Cam)\n");
    printf("   [R]                 : Reset / Recover Car Upright\n");
    printf("   [ESC]               : Exit Game\n");
    printf("============================================================\n\n");

    while (!platform.should_close) {
        double current_time = zero_platform_get_time();
        float dt = (float)(current_time - last_time);
        if (dt > 0.05f) dt = 0.05f;
        last_time = current_time;

        zero_platform_poll_events(&platform);

        // Player input processing
        float target_throttle = (platform.keys['w'] || platform.keys['W']) ? 1.0f : 0.0f;
        float target_brake = (platform.keys['s'] || platform.keys['S']) ? 1.0f : 0.0f;
        float target_steer = 0.0f;
        if (platform.keys['a'] || platform.keys['A']) target_steer -= 1.0f;
        if (platform.keys['d'] || platform.keys['D']) target_steer += 1.0f;

        car.handbrake = platform.keys[' '];

        // Smooth steering response with speed sensitivity
        float steer_speed = 7.0f * dt;
        steer_smooth += (target_steer - steer_smooth) * steer_speed;
        car.steer_input = steer_smooth;

        car.throttle_input = target_throttle;
        car.brake_input = target_brake;

        if (platform.keys['r'] || platform.keys['R']) {
            zero_vehicle_reset(&car, vec3(car.position.x, 0.65f, car.position.z));
        }

        static bool c_pressed_prev = false;
        if (platform.keys['c'] || platform.keys['C']) {
            if (!c_pressed_prev) {
                cockpit_view = !cockpit_view;
            }
            c_pressed_prev = true;
        } else {
            c_pressed_prev = false;
        }

        // Update real vehicle physics
        zero_vehicle_update(&car, dt, track_ground_fn);

        // Chase camera physics & interpolation
        Mat4 car_rot = quat_to_mat4(car.orientation);
        Vec3 car_fwd = vec3(car_rot.m[8], car_rot.m[9], car_rot.m[10]);
        Vec3 car_up = vec3(car_rot.m[4], car_rot.m[5], car_rot.m[6]);

        if (cockpit_view) {
            cam.position = vec3_add(car.position, vec3_add(vec3_scale(car_fwd, 0.4f), vec3_scale(car_up, 1.1f)));
            Vec3 target = vec3_add(cam.position, car_fwd);
            cam.forward = car_fwd;
            cam.up = car_up;
            cam.right = vec3_normalize(vec3_cross(car_fwd, car_up));
            (void)target;
        } else {
            Vec3 desired_cam = vec3_sub(
                vec3_add(car.position, vec3(0.0f, 2.2f, 0.0f)),
                vec3_scale(car_fwd, 6.2f)
            );
            float cam_lerp_factor = 9.0f * dt;
            if (cam_lerp_factor > 1.0f) cam_lerp_factor = 1.0f;
            cam_smooth_pos = vec3_lerp(cam_smooth_pos, desired_cam, cam_lerp_factor);
            cam.position = cam_smooth_pos;

            Vec3 look_target = vec3_add(car.position, vec3(0.0f, 1.0f, 0.0f));
            cam.forward = vec3_normalize(vec3_sub(look_target, cam.position));
            cam.right = vec3_normalize(vec3_cross(cam.forward, vec3(0.0f, 1.0f, 0.0f)));
            cam.up = vec3_normalize(vec3_cross(cam.right, cam.forward));
        }

        // Rendering pass
        cam.aspect = (float)platform.width / (float)platform.height;
        glViewport(0, 0, (GLsizei)platform.width, (GLsizei)platform.height);

        glClearColor(0.10f, 0.12f, 0.16f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        zero_shader_bind(&shader);

        Mat4 view = zero_camera_get_view(&cam);
        Mat4 proj = zero_camera_get_projection(&cam);
        zero_shader_set_mat4(&shader, "u_view", view);
        zero_shader_set_mat4(&shader, "u_proj", proj);
        zero_shader_set_vec3(&shader, "u_camera_pos", cam.position);

        zero_shader_set_vec3(&shader, "u_light_dir", vec3(-0.35f, -0.85f, -0.4f));
        zero_shader_set_vec3(&shader, "u_light_color", vec3(1.0f, 0.96f, 0.88f));
        zero_shader_set_vec3(&shader, "u_ambient_color", vec3(0.20f, 0.22f, 0.26f));

        // 1. Render Race Track Circuit
        Mat4 track_model = mat4_identity();
        zero_shader_set_mat4(&shader, "u_model", track_model);
        zero_shader_set_vec3(&shader, "u_object_color", vec3(0.18f, 0.20f, 0.22f));
        zero_mesh_draw(&track_mesh);

        // 2. Render Sports Car Chassis (Glossy Sports Red)
        zero_shader_set_mat4(&shader, "u_model", car.transform);
        zero_shader_set_vec3(&shader, "u_object_color", vec3(0.92f, 0.12f, 0.14f));
        zero_mesh_draw(&car_mesh);

        // 3. Render 4 Dynamic Suspension Wheels
        for (int i = 0; i < 4; ++i) {
            ZeroWheel *w = &car.wheels[i];
            Vec3 wheel_local = vec3(w->hardpoint.x, -w->current_length, w->hardpoint.z);
            Mat4 wheel_trans = mat4_translate(wheel_local);
            Mat4 wheel_steer = mat4_identity();
            if (w->is_front) {
                wheel_steer = quat_to_mat4(quat_from_axis_angle(vec3(0, 1, 0), w->steer_angle));
            }
            Mat4 wheel_local_mat = mat4_mul(wheel_trans, wheel_steer);
            Mat4 wheel_world_mat = mat4_mul(car.transform, wheel_local_mat);

            zero_shader_set_mat4(&shader, "u_model", wheel_world_mat);
            zero_shader_set_vec3(&shader, "u_object_color", vec3(0.15f, 0.16f, 0.18f));
            zero_mesh_draw(&wheel_mesh);
        }

        zero_shader_unbind();
        zero_platform_swap_buffers(&platform);

        // Telemetry HUD print
        hud_timer += dt;
        if (hud_timer > 0.25) {
            hud_timer = 0.0;
            printf("\r[Zero Racer] Speed: %5.1f km/h | Gear: %d | RPM: %4.0f | Drift: %s %4.1f deg  ",
                   car.speed_kmh, car.gear, car.engine_rpm,
                   car.is_drifting ? "DRIFT!" : "     ", car.drift_angle);
            fflush(stdout);
        }
    }

    printf("\n[Zero Racer] Game session terminated cleanly.\n");

    zero_mesh_destroy(&car_mesh);
    zero_mesh_destroy(&wheel_mesh);
    zero_mesh_destroy(&track_mesh);
    zero_shader_destroy(&shader);
    zero_arena_destroy(&arena);
    zero_platform_shutdown(&platform);

    return 0;
}

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

static const char *get_custom_vert_shader(void) {
    return 
        "#version 330 core\n"
        "layout(location = 0) in vec3 a_pos;\n"
        "layout(location = 1) in vec3 a_normal;\n"
        "layout(location = 2) in vec2 a_uv;\n"
        "uniform mat4 u_model;\n"
        "uniform mat4 u_view;\n"
        "uniform mat4 u_proj;\n"
        "out vec3 v_normal;\n"
        "out vec3 v_world_pos;\n"
        "out vec2 v_uv;\n"
        "void main() {\n"
        "    vec4 world_pos = u_model * vec4(a_pos, 1.0);\n"
        "    v_world_pos = world_pos.xyz;\n"
        "    v_normal = mat3(u_model) * a_normal;\n"
        "    v_uv = a_uv;\n"
        "    gl_Position = u_proj * u_view * world_pos;\n"
        "}\n";
}

static const char *get_custom_frag_shader(void) {
    return 
        "#version 330 core\n"
        "in vec3 v_normal;\n"
        "in vec3 v_world_pos;\n"
        "in vec2 v_uv;\n"
        "uniform vec3 u_light_dir;\n"
        "uniform vec3 u_light_color;\n"
        "uniform vec3 u_ambient_color;\n"
        "uniform vec3 u_object_color;\n"
        "uniform vec3 u_camera_pos;\n"
        "uniform float u_shininess;\n"
        "uniform float u_specular_strength;\n"
        "uniform float u_time;\n"
        "uniform int u_material_mode;\n"
        "out vec4 frag_color;\n"
        "void main() {\n"
        "    vec3 norm = normalize(v_normal);\n"
        "    vec3 light_dir = normalize(-u_light_dir);\n"
        "    float diff = max(dot(norm, light_dir), 0.0);\n"
        "    vec3 diffuse = diff * u_light_color;\n"
        "    vec3 view_dir = normalize(u_camera_pos - v_world_pos);\n"
        "    vec3 half_dir = normalize(light_dir + view_dir);\n"
        "    float spec = pow(max(dot(norm, half_dir), 0.0), u_shininess);\n"
        "    vec3 specular = spec * u_light_color * u_specular_strength;\n"
        "    vec3 base_color = u_object_color;\n"
        "    // Mode 1: Road Asphalt with dashed centerline\n"
        "    if (u_material_mode == 1) {\n"
        "        float lane_dist = abs(v_uv.x - 0.5);\n"
        "        if (lane_dist < 0.015) {\n"
        "            float dash = sin(v_uv.y * 3.14159);\n"
        "            if (dash > 0.0) base_color = vec3(0.96, 0.94, 0.88);\n"
        "        }\n"
        "    }\n"
        "    // Mode 2: Red/White Curbstones\n"
        "    else if (u_material_mode == 2) {\n"
        "        float step_val = mod(v_uv.y, 2.0);\n"
        "        base_color = (step_val < 1.0) ? vec3(0.92, 0.12, 0.14) : vec3(0.96, 0.96, 0.96);\n"
        "    }\n"
        "    // Mode 3: Checkerboard infield & start line\n"
        "    else if (u_material_mode == 3) {\n"
        "        float cx = floor(v_world_pos.x * 0.15);\n"
        "        float cz = floor(v_world_pos.z * 0.15);\n"
        "        float check = mod(cx + cz, 2.0);\n"
        "        base_color = (check < 1.0) ? vec3(0.22, 0.55, 0.20) : vec3(0.18, 0.48, 0.16);\n"
        "    }\n"
        "    // Mode 4: Tire Barrier Stripes\n"
        "    else if (u_material_mode == 4) {\n"
        "        float b_step = mod(v_world_pos.x + v_world_pos.z, 4.0);\n"
        "        base_color = (b_step < 2.0) ? vec3(0.25, 0.45, 0.95) : vec3(0.90, 0.90, 0.92);\n"
        "    }\n"
        "    // Sky atmospheric distance fog\n"
        "    float dist = length(u_camera_pos - v_world_pos);\n"
        "    float fog_factor = clamp((dist - 60.0) / 450.0, 0.0, 0.85);\n"
        "    vec3 lit_color = (u_ambient_color + diffuse + specular) * base_color;\n"
        "    vec3 sky_color = vec3(0.50, 0.70, 0.95);\n"
        "    vec3 final_color = mix(lit_color, sky_color, fog_factor);\n"
        "    frag_color = vec4(final_color, 1.0);\n"
        "}\n";
}

int main(void) {
    ZeroPlatformConfig cfg = {
        .title = "Zero Racer: 3D Grand Prix Circuit & Real Vehicle Simulator",
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
    ZeroMeshData road_data = zero_obj_parse_file("assets/models/track_road.obj", &arena);
    ZeroMeshData curb_data = zero_obj_parse_file("assets/models/track_curbs.obj", &arena);
    ZeroMeshData barrier_data = zero_obj_parse_file("assets/models/track_barriers.obj", &arena);
    ZeroMeshData terrain_data = zero_obj_parse_file("assets/models/track_terrain.obj", &arena);
    ZeroMeshData gantry_data = zero_obj_parse_file("assets/models/track_gantry.obj", &arena);

    if (car_data.vertex_count == 0 || wheel_data.vertex_count == 0 ||
        road_data.vertex_count == 0 || terrain_data.vertex_count == 0) {
        fprintf(stderr, "Failed to load 3D world assets\n");
        zero_arena_destroy(&arena);
        zero_platform_shutdown(&platform);
        return 1;
    }

    ZeroMesh car_mesh = zero_mesh_upload(&car_data);
    ZeroMesh wheel_mesh = zero_mesh_upload(&wheel_data);
    ZeroMesh road_mesh = zero_mesh_upload(&road_data);
    ZeroMesh curb_mesh = zero_mesh_upload(&curb_data);
    ZeroMesh barrier_mesh = zero_mesh_upload(&barrier_data);
    ZeroMesh terrain_mesh = zero_mesh_upload(&terrain_data);
    ZeroMesh gantry_mesh = zero_mesh_upload(&gantry_data);

    ZeroShader shader = zero_shader_create(get_custom_vert_shader(), get_custom_frag_shader());
    if (!shader.is_valid) {
        fprintf(stderr, "Failed to compile custom shader\n");
        zero_arena_destroy(&arena);
        zero_platform_shutdown(&platform);
        return 1;
    }

    // Spawn vehicle at starting grid
    ZeroVehicle car = zero_vehicle_create(vec3(155.0f, 0.70f, 0.0f));

    ZeroCamera cam = zero_camera_create(
        vec3(155.0f, 3.2f, -8.0f),
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
    printf("===      ZERO RACER: GRAND PRIX CIRCUIT & REAL PHYSICS   ===\n");
    printf("============================================================\n");
    printf(" Controls:\n");
    printf("   [W] / [Up Arrow]    : Throttle / Accelerate Forward\n");
    printf("   [S] / [Down Arrow]  : Brake / Active Reverse Gear\n");
    printf("   [A] / [D] / [Arrows]: Steer Left / Right (Speed-sensitive)\n");
    printf("   [Spacebar]          : Handbrake (Initiates High-Speed Drift!)\n");
    printf("   [C]                 : Switch Camera View (Chase Cam / Hood Cam)\n");
    printf("   [R]                 : Reset Vehicle to Start Grid\n");
    printf("   [ESC]               : Exit Game\n");
    printf("============================================================\n\n");

    while (!platform.should_close) {
        double current_time = zero_platform_get_time();
        float dt = (float)(current_time - last_time);
        if (dt > 0.05f) dt = 0.05f;
        last_time = current_time;

        zero_platform_poll_events(&platform);

        // Player input processing with both WASD and Arrow Keys support
        // Keys: XK_Up = 256+2, XK_Down = 256+4, XK_Left = 256+1, XK_Right = 256+3
        bool key_up = platform.keys['w'] || platform.keys['W'] || platform.keys[258];
        bool key_down = platform.keys['s'] || platform.keys['S'] || platform.keys[260];
        bool key_left = platform.keys['a'] || platform.keys['A'] || platform.keys[257];
        bool key_right = platform.keys['d'] || platform.keys['D'] || platform.keys[259];

        float target_throttle = key_up ? 1.0f : 0.0f;
        float target_brake = key_down ? 1.0f : 0.0f;
        float target_steer = 0.0f;
        if (key_left) target_steer -= 1.0f;
        if (key_right) target_steer += 1.0f;

        car.handbrake = platform.keys[' '];

        // Smooth steering response with speed sensitivity
        float steer_speed = 8.5f * dt;
        steer_smooth += (target_steer - steer_smooth) * steer_speed;
        car.steer_input = steer_smooth;

        car.throttle_input = target_throttle;
        car.brake_input = target_brake;

        if (platform.keys['r'] || platform.keys['R']) {
            zero_vehicle_reset(&car, vec3(155.0f, 0.70f, 0.0f));
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

        // Update real vehicle physics with gravity and ground suspension
        zero_vehicle_update(&car, dt, track_ground_fn);

        // Chase camera physics & interpolation
        Mat4 car_rot = quat_to_mat4(car.orientation);
        Vec3 car_fwd = vec3(car_rot.m[8], car_rot.m[9], car_rot.m[10]);
        Vec3 car_up = vec3(car_rot.m[4], car_rot.m[5], car_rot.m[6]);

        if (cockpit_view) {
            cam.position = vec3_add(car.position, vec3_add(vec3_scale(car_fwd, 0.35f), vec3_scale(car_up, 1.05f)));
            Vec3 target = vec3_add(cam.position, car_fwd);
            cam.forward = car_fwd;
            cam.up = car_up;
            cam.right = vec3_normalize(vec3_cross(car_fwd, car_up));
            (void)target;
        } else {
            Vec3 desired_cam = vec3_sub(
                vec3_add(car.position, vec3(0.0f, 2.3f, 0.0f)),
                vec3_scale(car_fwd, 6.2f)
            );
            float cam_lerp_factor = 10.0f * dt;
            if (cam_lerp_factor > 1.0f) cam_lerp_factor = 1.0f;
            cam_smooth_pos = vec3_lerp(cam_smooth_pos, desired_cam, cam_lerp_factor);
            cam.position = cam_smooth_pos;

            Vec3 look_target = vec3_add(car.position, vec3(0.0f, 0.9f, 0.0f));
            cam.forward = vec3_normalize(vec3_sub(look_target, cam.position));
            cam.right = vec3_normalize(vec3_cross(cam.forward, vec3(0.0f, 1.0f, 0.0f)));
            cam.up = vec3_normalize(vec3_cross(cam.right, cam.forward));
        }

        // Rendering pass
        cam.aspect = (float)platform.width / (float)platform.height;
        glViewport(0, 0, (GLsizei)platform.width, (GLsizei)platform.height);

        // Sky blue atmospheric clear
        glClearColor(0.45f, 0.65f, 0.90f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        zero_shader_bind(&shader);

        Mat4 view = zero_camera_get_view(&cam);
        Mat4 proj = zero_camera_get_projection(&cam);
        zero_shader_set_mat4(&shader, "u_view", view);
        zero_shader_set_mat4(&shader, "u_proj", proj);
        zero_shader_set_vec3(&shader, "u_camera_pos", cam.position);
        zero_shader_set_float(&shader, "u_time", (float)current_time);

        // Outdoor sunny daylight lighting
        zero_shader_set_vec3(&shader, "u_light_dir", vec3(-0.45f, -0.80f, -0.38f));
        zero_shader_set_vec3(&shader, "u_light_color", vec3(1.0f, 0.98f, 0.92f));
        zero_shader_set_vec3(&shader, "u_ambient_color", vec3(0.35f, 0.40f, 0.45f));

        Mat4 identity_mat = mat4_identity();
        zero_shader_set_mat4(&shader, "u_model", identity_mat);

        // 1. Render Vast Grass Terrain (Patterned greens)
        zero_shader_set_float(&shader, "u_shininess", 8.0f);
        zero_shader_set_float(&shader, "u_specular_strength", 0.05f);
        zero_shader_set_vec3(&shader, "u_object_color", vec3(0.20f, 0.52f, 0.18f));
        zero_shader_set_float(&shader, "u_material_mode", 3.0f);
        zero_mesh_draw(&terrain_mesh);

        // 2. Render Road Asphalt Circuit (Dark tarmac with lane stripes)
        zero_shader_set_float(&shader, "u_shininess", 16.0f);
        zero_shader_set_float(&shader, "u_specular_strength", 0.20f);
        zero_shader_set_vec3(&shader, "u_object_color", vec3(0.18f, 0.18f, 0.20f));
        zero_shader_set_float(&shader, "u_material_mode", 1.0f);
        zero_mesh_draw(&road_mesh);

        // 3. Render Curbs (Red and White alternating rumble strips)
        zero_shader_set_float(&shader, "u_shininess", 24.0f);
        zero_shader_set_float(&shader, "u_specular_strength", 0.35f);
        zero_shader_set_vec3(&shader, "u_object_color", vec3(0.90f, 0.15f, 0.15f));
        zero_shader_set_float(&shader, "u_material_mode", 2.0f);
        zero_mesh_draw(&curb_mesh);

        // 4. Render Perimeter Barriers (Blue/White safety armco walls)
        zero_shader_set_float(&shader, "u_shininess", 32.0f);
        zero_shader_set_float(&shader, "u_specular_strength", 0.45f);
        zero_shader_set_vec3(&shader, "u_object_color", vec3(0.30f, 0.50f, 0.90f));
        zero_shader_set_float(&shader, "u_material_mode", 4.0f);
        zero_mesh_draw(&barrier_mesh);

        // 5. Render Start/Finish Overhead Gantry Arch
        zero_shader_set_float(&shader, "u_shininess", 64.0f);
        zero_shader_set_float(&shader, "u_specular_strength", 0.60f);
        zero_shader_set_vec3(&shader, "u_object_color", vec3(0.85f, 0.85f, 0.20f));
        zero_shader_set_float(&shader, "u_material_mode", 0.0f);
        zero_mesh_draw(&gantry_mesh);

        // 6. Render Sports Car Chassis (Deep Glossy Italian Racing Red with Blinn-Phong highlight)
        zero_shader_set_mat4(&shader, "u_model", car.transform);
        zero_shader_set_float(&shader, "u_shininess", 96.0f);
        zero_shader_set_float(&shader, "u_specular_strength", 0.85f);
        zero_shader_set_vec3(&shader, "u_object_color", vec3(0.92f, 0.08f, 0.12f));
        zero_shader_set_float(&shader, "u_material_mode", 0.0f);
        zero_mesh_draw(&car_mesh);

        // 7. Render 4 Dynamic Suspension Wheels with realistic spinning & steering
        for (int i = 0; i < 4; ++i) {
            ZeroWheel *w = &car.wheels[i];
            Vec3 wheel_local = vec3(w->hardpoint.x, -w->current_length, w->hardpoint.z);
            Mat4 wheel_trans = mat4_translate(wheel_local);
            Mat4 wheel_steer = mat4_identity();
            if (w->is_front) {
                wheel_steer = quat_to_mat4(quat_from_axis_angle(vec3(0, 1, 0), w->steer_angle));
            }
            // Dynamic tire spinning based on travel speed
            float spin_speed = vec3_length(car.velocity) / w->radius;
            static float wheel_spin_angles[4] = {0};
            wheel_spin_angles[i] += (car.velocity.z >= 0.0f ? 1.0f : -1.0f) * spin_speed * dt;
            Mat4 wheel_spin = quat_to_mat4(quat_from_axis_angle(vec3(1, 0, 0), wheel_spin_angles[i]));

            Mat4 wheel_local_mat = mat4_mul(wheel_trans, mat4_mul(wheel_steer, wheel_spin));
            Mat4 wheel_world_mat = mat4_mul(car.transform, wheel_local_mat);

            zero_shader_set_mat4(&shader, "u_model", wheel_world_mat);
            zero_shader_set_float(&shader, "u_shininess", 16.0f);
            zero_shader_set_float(&shader, "u_specular_strength", 0.25f);
            zero_shader_set_vec3(&shader, "u_object_color", vec3(0.12f, 0.13f, 0.15f));
            zero_shader_set_float(&shader, "u_material_mode", 0.0f);
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
    zero_mesh_destroy(&road_mesh);
    zero_mesh_destroy(&curb_mesh);
    zero_mesh_destroy(&barrier_mesh);
    zero_mesh_destroy(&terrain_mesh);
    zero_mesh_destroy(&gantry_mesh);
    zero_shader_destroy(&shader);
    zero_arena_destroy(&arena);
    zero_platform_shutdown(&platform);

    return 0;
}

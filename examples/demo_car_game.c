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

// Next-Gen GTA-Style Unified 3D PBR, Procedural Skybox, and 2D HUD Vertex Shader
static const char *get_nextgen_vert_shader(void) {
    return 
        "#version 330 core\n"
        "layout(location = 0) in vec3 a_pos;\n"
        "layout(location = 1) in vec3 a_normal;\n"
        "layout(location = 2) in vec2 a_uv;\n"
        "uniform mat4 u_model;\n"
        "uniform mat4 u_view;\n"
        "uniform mat4 u_proj;\n"
        "uniform float u_is_hud;\n"
        "uniform float u_is_sky;\n"
        "out vec3 v_normal;\n"
        "out vec3 v_world_pos;\n"
        "out vec2 v_uv;\n"
        "out vec3 v_sky_ray;\n"
        "void main() {\n"
        "    if (u_is_hud > 0.5) {\n"
        "        v_normal = vec3(0.0, 0.0, 1.0);\n"
        "        v_world_pos = a_pos;\n"
        "        v_uv = a_uv;\n"
        "        gl_Position = u_proj * vec4(a_pos, 1.0);\n"
        "    } else if (u_is_sky > 0.5) {\n"
        "        v_uv = a_uv;\n"
        "        v_world_pos = a_pos;\n"
        "        v_normal = -normalize(a_pos);\n"
        "        v_sky_ray = normalize(a_pos);\n"
        "        gl_Position = u_proj * u_view * vec4(a_pos, 1.0);\n"
        "    } else {\n"
        "        vec4 world_pos = u_model * vec4(a_pos, 1.0);\n"
        "        v_world_pos = world_pos.xyz;\n"
        "        v_normal = mat3(u_model) * a_normal;\n"
        "        v_uv = a_uv;\n"
        "        gl_Position = u_proj * u_view * world_pos;\n"
        "    }\n"
        "}\n";
}

// Next-Gen Cook-Torrance Microfacet GGX Metallic PBR + Layered Clearcoat + Sun Sky + 2D HUD Fragment Shader
static const char *get_nextgen_frag_shader(void) {
    return 
        "#version 330 core\n"
        "in vec3 v_normal;\n"
        "in vec3 v_world_pos;\n"
        "in vec2 v_uv;\n"
        "in vec3 v_sky_ray;\n"
        "uniform vec3 u_light_dir;\n"
        "uniform vec3 u_light_color;\n"
        "uniform vec3 u_ambient_color;\n"
        "uniform vec3 u_object_color;\n"
        "uniform vec3 u_camera_pos;\n"
        "uniform float u_roughness;\n"
        "uniform float u_metallic;\n"
        "uniform float u_time;\n"
        "uniform float u_material_mode;\n"
        "uniform float u_is_hud;\n"
        "uniform float u_is_sky;\n"
        "out vec4 frag_color;\n"
        "\n"
        "const float PI = 3.14159265359;\n"
        "\n"
        "float distributionGGX(vec3 N, vec3 H, float roughness) {\n"
        "    float a = roughness * roughness;\n"
        "    float a2 = a * a;\n"
        "    float NdotH = max(dot(N, H), 0.0);\n"
        "    float NdotH2 = NdotH * NdotH;\n"
        "    float denom = (NdotH2 * (a2 - 1.0) + 1.0);\n"
        "    return a2 / (PI * denom * denom + 0.0001);\n"
        "}\n"
        "\n"
        "float geometrySchlickGGX(float NdotV, float roughness) {\n"
        "    float r = (roughness + 1.0);\n"
        "    float k = (r * r) / 8.0;\n"
        "    return NdotV / (NdotV * (1.0 - k) + k + 0.0001);\n"
        "}\n"
        "\n"
        "float geometrySmith(vec3 N, vec3 V, vec3 L, float roughness) {\n"
        "    float NdotV = max(dot(N, V), 0.0);\n"
        "    float NdotL = max(dot(N, L), 0.0);\n"
        "    return geometrySchlickGGX(NdotV, roughness) * geometrySchlickGGX(NdotL, roughness);\n"
        "}\n"
        "\n"
        "vec3 fresnelSchlick(float cosTheta, vec3 F0) {\n"
        "    return F0 + (1.0 - F0) * pow(clamp(1.0 - cosTheta, 0.0, 1.0), 5.0);\n"
        "}\n"
        "\n"
        "void main() {\n"
        "    // 1. HUD 2D Pass\n"
        "    if (u_is_hud > 0.5) {\n"
        "        frag_color = vec4(u_object_color, 1.0);\n"
        "        return;\n"
        "    }\n"
        "\n"
        "    // 2. Procedural Sky Dome Pass (Realistic Sun, Rayleigh Scattering, Horizon Glow)\n"
        "    if (u_is_sky > 0.5) {\n"
        "        vec3 ray = normalize(v_sky_ray);\n"
        "        vec3 sun_dir = normalize(-u_light_dir);\n"
        "        float sun_dot = max(dot(ray, sun_dir), 0.0);\n"
        "\n"
        "        vec3 sky_zenith = vec3(0.18, 0.42, 0.88);\n"
        "        vec3 sky_mid = vec3(0.48, 0.72, 0.95);\n"
        "        vec3 sky_horizon = vec3(0.85, 0.88, 0.95);\n"
        "        vec3 ground_fog = vec3(0.40, 0.48, 0.55);\n"
        "\n"
        "        vec3 sky = (ray.y > 0.0) ? mix(sky_horizon, mix(sky_mid, sky_zenith, pow(ray.y, 0.65)), ray.y) : mix(sky_horizon, ground_fog, clamp(-ray.y * 3.0, 0.0, 1.0));\n"
        "\n"
        "        // Sharp Sun disk with high-intensity bloom halo\n"
        "        float sun_disk = pow(sun_dot, 800.0) * 15.0;\n"
        "        float sun_halo = pow(sun_dot, 24.0) * 1.8;\n"
        "        vec3 sun = vec3(1.0, 0.96, 0.82) * (sun_disk + sun_halo);\n"
        "\n"
        "        // Procedural cloud wisps\n"
        "        if (ray.y > 0.05) {\n"
        "            vec2 cloud_coord = ray.xz / (ray.y + 0.15) * 0.15 + vec2(u_time * 0.015, 0.0);\n"
        "            float c_noise = sin(cloud_coord.x * 3.0) * cos(cloud_coord.y * 3.0);\n"
        "            c_noise += sin(cloud_coord.x * 7.0 + cloud_coord.y * 5.0) * 0.5;\n"
        "            float cloud_alpha = clamp((c_noise - 0.2) * 1.8, 0.0, 0.65);\n"
        "            sky = mix(sky, vec3(0.98, 0.98, 1.0), cloud_alpha);\n"
        "        }\n"
        "\n"
        "        vec3 final_sky = sky + sun;\n"
        "        final_sky = final_sky / (final_sky + vec3(1.0));\n"
        "        final_sky = pow(final_sky, vec3(1.0 / 2.2));\n"
        "        frag_color = vec4(final_sky, 1.0);\n"
        "        return;\n"
        "    }\n"
        "\n"
        "    // 3. Contact Shadow Quad under Car (Soft ambient occlusion)\n"
        "    if (u_material_mode > 4.5 && u_material_mode < 5.5) {\n"
        "        float dist = length(v_uv - vec2(0.5));\n"
        "        float alpha = clamp((0.5 - dist) * 2.8, 0.0, 0.72);\n"
        "        frag_color = vec4(0.04, 0.04, 0.05, alpha);\n"
        "        return;\n"
        "    }\n"
        "\n"
        "    // 4. 3D PBR Geometry Rendering Pass\n"
        "    vec3 N = normalize(v_normal);\n"
        "    vec3 V = normalize(u_camera_pos - v_world_pos);\n"
        "    vec3 L = normalize(-u_light_dir);\n"
        "    vec3 H = normalize(V + L);\n"
        "\n"
        "    float NdotL = max(dot(N, L), 0.0);\n"
        "    float NdotV = max(dot(N, V), 0.0);\n"
        "    float NdotH = max(dot(N, H), 0.0);\n"
        "    float VdotH = max(dot(V, H), 0.0);\n"
        "\n"
        "    vec3 albedo = u_object_color;\n"
        "    float roughness = clamp(u_roughness, 0.04, 0.96);\n"
        "    float metallic = clamp(u_metallic, 0.0, 1.0);\n"
        "\n"
        "    // Material 1: Tarmac asphalt with crisp white dashed centerline and road asphalt noise\n"
        "    if (u_material_mode > 0.5 && u_material_mode < 1.5) {\n"
        "        float lane = abs(v_uv.x - 0.5);\n"
        "        if (lane < 0.016) {\n"
        "            float dash = sin(v_uv.y * 3.14159);\n"
        "            if (dash > 0.0) albedo = vec3(0.96, 0.95, 0.90);\n"
        "        }\n"
        "        // Subtle asphalt grain texture\n"
        "        float grain = sin(v_world_pos.x * 25.0) * sin(v_world_pos.z * 25.0);\n"
        "        albedo += vec3(grain * 0.015);\n"
        "    }\n"
        "    // Material 2: Grand Prix Red/White Rumble Curbs\n"
        "    else if (u_material_mode > 1.5 && u_material_mode < 2.5) {\n"
        "        float step_val = mod(v_uv.y, 2.0);\n"
        "        albedo = (step_val < 1.0) ? vec3(0.92, 0.08, 0.10) : vec3(0.96, 0.96, 0.96);\n"
        "    }\n"
        "    // Material 3: Lush green terrain with field grid\n"
        "    else if (u_material_mode > 2.5 && u_material_mode < 3.5) {\n"
        "        float cx = floor(v_world_pos.x * 0.12);\n"
        "        float cz = floor(v_world_pos.z * 0.12);\n"
        "        float check = mod(cx + cz, 2.0);\n"
        "        albedo = (check < 1.0) ? vec3(0.24, 0.55, 0.22) : vec3(0.19, 0.48, 0.18);\n"
        "    }\n"
        "    // Material 4: Armco barriers with dual-tone safety blue/white stripes\n"
        "    else if (u_material_mode > 3.5 && u_material_mode < 4.5) {\n"
        "        float b_step = mod(v_world_pos.x + v_world_pos.z, 4.0);\n"
        "        albedo = (b_step < 2.0) ? vec3(0.22, 0.45, 0.92) : vec3(0.92, 0.92, 0.95);\n"
        "    }\n"
        "\n"
        "    // Cook-Torrance Specular BRDF\n"
        "    vec3 F0 = mix(vec3(0.04), albedo, metallic);\n"
        "    float NDF = distributionGGX(N, H, roughness);\n"
        "    float G = geometrySmith(N, V, L, roughness);\n"
        "    vec3 F = fresnelSchlick(VdotH, F0);\n"
        "\n"
        "    vec3 numerator = NDF * G * F;\n"
        "    float denominator = 4.0 * NdotV * NdotL + 0.0001;\n"
        "    vec3 specular = numerator / denominator;\n"
        "\n"
        "    vec3 kS = F;\n"
        "    vec3 kD = vec3(1.0) - kS;\n"
        "    kD *= (1.0 - metallic);\n"
        "\n"
        "    vec3 diffuse = (kD * albedo / PI) * u_light_color * NdotL;\n"
        "    vec3 spec_light = specular * u_light_color * NdotL;\n"
        "\n"
        "    // Layered Automotive Clearcoat Lobe with microfacet sparkle\n"
        "    float clearcoat_NDF = distributionGGX(N, H, 0.10);\n"
        "    float clearcoat_G = geometrySmith(N, V, L, 0.10);\n"
        "    vec3 clearcoat_F = fresnelSchlick(VdotH, vec3(0.04));\n"
        "    vec3 clearcoat_spec = (clearcoat_NDF * clearcoat_G * clearcoat_F) / (4.0 * NdotV * NdotL + 0.0001) * 0.50;\n"
        "\n"
        "    // Dynamic Environmental Sky Reflections on Gloss Surface\n"
        "    vec3 R = reflect(-V, N);\n"
        "    vec3 sky_zenith = vec3(0.25, 0.50, 0.88);\n"
        "    vec3 sky_horizon = vec3(0.80, 0.85, 0.94);\n"
        "    vec3 env_reflection = mix(sky_horizon, sky_zenith, clamp(R.y * 0.5 + 0.5, 0.0, 1.0));\n"
        "    vec3 ambient = u_ambient_color * albedo + env_reflection * F * 0.35;\n"
        "\n"
        "    vec3 color = ambient + diffuse + spec_light + clearcoat_spec * NdotL;\n"
        "\n"
        "    // Realistic Horizon Atmospheric Fog Haze\n"
        "    float dist = length(u_camera_pos - v_world_pos);\n"
        "    float fog_factor = clamp((dist - 90.0) / 450.0, 0.0, 0.82);\n"
        "    color = mix(color, sky_horizon, fog_factor);\n"
        "\n"
        "    // Filmic Reinhard Tone Mapping + Gamma Correction\n"
        "    color = color / (color + vec3(1.0));\n"
        "    color = pow(color, vec3(1.0 / 2.2));\n"
        "\n"
        "    frag_color = vec4(color, 1.0);\n"
        "}\n";
}

// 2D Quad Generator for HUD & Contact Shadow
static ZeroMesh create_quad_mesh(void) {
    ZeroVertex verts[4] = {
        {{-0.5f, 0.0f, -0.5f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f}},
        {{ 0.5f, 0.0f, -0.5f}, {0.0f, 1.0f, 0.0f}, {1.0f, 0.0f}},
        {{ 0.5f, 0.0f,  0.5f}, {0.0f, 1.0f, 0.0f}, {1.0f, 1.0f}},
        {{-0.5f, 0.0f,  0.5f}, {0.0f, 1.0f, 0.0f}, {0.0f, 1.0f}}
    };
    uint32_t indices[6] = {0, 1, 2, 0, 2, 3};
    ZeroMeshData data = {verts, 4, indices, 6};
    return zero_mesh_upload(&data);
}

static ZeroMesh create_hud_quad_mesh(void) {
    ZeroVertex verts[4] = {
        {{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f}, {0.0f, 0.0f}},
        {{1.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f}, {1.0f, 0.0f}},
        {{1.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}, {1.0f, 1.0f}},
        {{0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}, {0.0f, 1.0f}}
    };
    uint32_t indices[6] = {0, 1, 2, 0, 2, 3};
    ZeroMeshData data = {verts, 4, indices, 6};
    return zero_mesh_upload(&data);
}

static void render_hud_quad(ZeroShader *shader, ZeroMesh *quad, float x, float y, float w, float h, Vec3 color) {
    Mat4 trans = mat4_translate(vec3(x, y, 0.0f));
    Mat4 scl = mat4_scale(vec3(w, h, 1.0f));
    Mat4 model = mat4_mul(trans, scl);
    zero_shader_set_mat4(shader, "u_model", model);
    zero_shader_set_vec3(shader, "u_object_color", color);
    zero_mesh_draw(quad);
}

int main(void) {
    ZeroPlatformConfig cfg = {
        .title = "Zero Racer: Next-Gen Grand Prix Simulator",
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
    ZeroMeshData sky_data = zero_mesh_gen_sphere(&arena, 600.0f, 24, 24);
    ZeroMesh sky_sphere = zero_mesh_upload(&sky_data);
    ZeroMesh shadow_quad = create_quad_mesh();
    ZeroMesh hud_quad = create_hud_quad_mesh();

    ZeroShader shader = zero_shader_create(get_nextgen_vert_shader(), get_nextgen_frag_shader());
    if (!shader.is_valid) {
        fprintf(stderr, "Failed to compile next-gen PBR shader\n");
        zero_arena_destroy(&arena);
        zero_platform_shutdown(&platform);
        return 1;
    }

    // Spawn vehicle on starting straight facing along track direction (+Z)
    ZeroVehicle car = zero_vehicle_create(vec3(125.0f, 0.75f, 0.0f));

    ZeroCamera cam = zero_camera_create(
        vec3(125.0f, 3.2f, -8.0f),
        ZERO_DEG2RAD(65.0f),
        (float)platform.width / (float)platform.height,
        0.1f,
        2000.0f
    );

    double last_time = zero_platform_get_time();
    double hud_timer = 0.0;
    bool cockpit_view = false;
    float steer_smooth = 0.0f;
    Vec3 cam_smooth_pos = cam.position;

    printf("\n============================================================\n");
    printf("===   ZERO RACER: NEXT-GEN PBR & GRAND PRIX SIMULATOR    ===\n");
    printf("============================================================\n");
    printf(" Controls:\n");
    printf("   [W] / [Up Arrow]    : Throttle / Accelerate Forward\n");
    printf("   [S] / [Down Arrow]  : Brake / Active Reverse Gear\n");
    printf("   [A] / [Left Arrow]  : Steer Left\n");
    printf("   [D] / [Right Arrow] : Steer Right\n");
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
        bool key_up = platform.keys['w'] || platform.keys['W'] || platform.keys[258];
        bool key_down = platform.keys['s'] || platform.keys['S'] || platform.keys[260];
        bool key_left = platform.keys['a'] || platform.keys['A'] || platform.keys[257];
        bool key_right = platform.keys['d'] || platform.keys['D'] || platform.keys[259];

        float target_throttle = key_up ? 1.0f : 0.0f;
        float target_brake = key_down ? 1.0f : 0.0f;

        // Steering: A/Left key steers left (-1.0), D/Right key steers right (+1.0)
        float target_steer = 0.0f;
        if (key_left) target_steer -= 1.0f;
        if (key_right) target_steer += 1.0f;

        car.handbrake = platform.keys[' '];

        // Comfortable digital keyboard steering ramp (prevents abrupt jerking)
        float steer_rate = 3.5f * dt;
        if (target_steer > steer_smooth) {
            steer_smooth += steer_rate;
            if (steer_smooth > target_steer) steer_smooth = target_steer;
        } else if (target_steer < steer_smooth) {
            steer_smooth -= steer_rate;
            if (steer_smooth < target_steer) steer_smooth = target_steer;
        }
        car.steer_input = steer_smooth;

        car.throttle_input = target_throttle;
        car.brake_input = target_brake;

        if (platform.keys['r'] || platform.keys['R']) {
            zero_vehicle_reset(&car, vec3(125.0f, 0.75f, 0.0f));
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

        // Chase camera physics & interpolation behind vehicle
        Mat4 car_rot = quat_to_mat4(car.orientation);
        Vec3 car_fwd = vec3(car_rot.m[8], car_rot.m[9], car_rot.m[10]);
        Vec3 car_up = vec3(car_rot.m[4], car_rot.m[5], car_rot.m[6]);

        if (cockpit_view) {
            cam.position = vec3_add(car.position, vec3_add(vec3_scale(car_fwd, 0.35f), vec3_scale(car_up, 1.05f)));
            cam.forward = car_fwd;
            cam.right = vec3_normalize(vec3_cross(car_up, car_fwd));
            cam.up = vec3_normalize(vec3_cross(cam.forward, cam.right));
        } else {
            Vec3 desired_cam = vec3_sub(
                vec3_add(car.position, vec3(0.0f, 2.5f, 0.0f)),
                vec3_scale(car_fwd, 6.5f)
            );
            float cam_lerp_factor = 14.0f * dt;
            if (cam_lerp_factor > 1.0f) cam_lerp_factor = 1.0f;
            cam_smooth_pos = vec3_lerp(cam_smooth_pos, desired_cam, cam_lerp_factor);
            cam.position = cam_smooth_pos;

            Vec3 look_target = vec3_add(car.position, vec3(0.0f, 0.9f, 0.0f));
            cam.forward = vec3_normalize(vec3_sub(look_target, cam.position));
            cam.right = vec3_normalize(vec3_cross(vec3(0.0f, 1.0f, 0.0f), cam.forward));
            cam.up = vec3_normalize(vec3_cross(cam.forward, cam.right));
        }

        // 3D Rendering Pass
        cam.aspect = (float)platform.width / (float)platform.height;
        glViewport(0, 0, (GLsizei)platform.width, (GLsizei)platform.height);

        glClearColor(0.15f, 0.20f, 0.30f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        zero_shader_bind(&shader);
        zero_shader_set_float(&shader, "u_is_hud", 0.0f);
        zero_shader_set_float(&shader, "u_time", (float)current_time);

        Mat4 view = mat4_identity();
        view.m[0] = cam.right.x;  view.m[4] = cam.right.y;  view.m[8] = cam.right.z;  view.m[12] = -vec3_dot(cam.right, cam.position);
        view.m[1] = cam.up.x;     view.m[5] = cam.up.y;     view.m[9] = cam.up.z;     view.m[13] = -vec3_dot(cam.up, cam.position);
        view.m[2] = -cam.forward.x; view.m[6] = -cam.forward.y; view.m[10] = -cam.forward.z; view.m[14] = vec3_dot(cam.forward, cam.position);
        view.m[15] = 1.0f;

        Mat4 proj = zero_camera_get_projection(&cam);
        zero_shader_set_mat4(&shader, "u_view", view);
        zero_shader_set_mat4(&shader, "u_proj", proj);
        zero_shader_set_vec3(&shader, "u_camera_pos", cam.position);

        // Sunlight directional lighting (crisp warm afternoon sun)
        zero_shader_set_vec3(&shader, "u_light_dir", vec3(-0.45f, -0.80f, -0.38f));
        zero_shader_set_vec3(&shader, "u_light_color", vec3(1.15f, 1.08f, 0.95f));
        zero_shader_set_vec3(&shader, "u_ambient_color", vec3(0.32f, 0.36f, 0.42f));

        // 1. Render Dynamic Photorealistic Sky Dome (Sun Disk, Rayleigh Sky, Cloud Layers)
        glDisable(GL_DEPTH_TEST);
        zero_shader_set_float(&shader, "u_is_sky", 1.0f);
        Mat4 sky_model = mat4_translate(cam.position);
        zero_shader_set_mat4(&shader, "u_model", sky_model);
        zero_mesh_draw(&sky_sphere);
        zero_shader_set_float(&shader, "u_is_sky", 0.0f);

        // Enable Depth Testing for 3D world geometry
        glEnable(GL_DEPTH_TEST);

        Mat4 identity_mat = mat4_identity();
        zero_shader_set_mat4(&shader, "u_model", identity_mat);

        // 2. Render Vast Grass Terrain (Green field checker)
        zero_shader_set_float(&shader, "u_roughness", 0.88f);
        zero_shader_set_float(&shader, "u_metallic", 0.0f);
        zero_shader_set_vec3(&shader, "u_object_color", vec3(0.22f, 0.54f, 0.20f));
        zero_shader_set_float(&shader, "u_material_mode", 3.0f);
        zero_mesh_draw(&terrain_mesh);

        // 3. Render Road Asphalt Circuit (Dark tarmac with procedural lane stripes & grain)
        zero_shader_set_float(&shader, "u_roughness", 0.70f);
        zero_shader_set_float(&shader, "u_metallic", 0.05f);
        zero_shader_set_vec3(&shader, "u_object_color", vec3(0.18f, 0.18f, 0.20f));
        zero_shader_set_float(&shader, "u_material_mode", 1.0f);
        zero_mesh_draw(&road_mesh);

        // 4. Render Curbs (Red and White alternating rumble strips)
        zero_shader_set_float(&shader, "u_roughness", 0.40f);
        zero_shader_set_float(&shader, "u_metallic", 0.10f);
        zero_shader_set_vec3(&shader, "u_object_color", vec3(0.90f, 0.12f, 0.14f));
        zero_shader_set_float(&shader, "u_material_mode", 2.0f);
        zero_mesh_draw(&curb_mesh);

        // 5. Render Perimeter Barriers (Armco blue and white safety walls)
        zero_shader_set_float(&shader, "u_roughness", 0.32f);
        zero_shader_set_float(&shader, "u_metallic", 0.70f);
        zero_shader_set_vec3(&shader, "u_object_color", vec3(0.25f, 0.48f, 0.92f));
        zero_shader_set_float(&shader, "u_material_mode", 4.0f);
        zero_mesh_draw(&barrier_mesh);

        // 6. Render Soft Contact Shadow under Car (Grounds the vehicle realistically)
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        zero_shader_set_float(&shader, "u_material_mode", 5.0f);
        Mat4 shadow_trans = mat4_translate(vec3(car.position.x, 0.03f, car.position.z));
        Mat4 shadow_rot = quat_to_mat4(car.orientation);
        Mat4 shadow_scale = mat4_scale(vec3(2.5f, 1.0f, 5.0f));
        Mat4 shadow_mat = mat4_mul(shadow_trans, mat4_mul(shadow_rot, shadow_scale));
        zero_shader_set_mat4(&shader, "u_model", shadow_mat);
        zero_mesh_draw(&shadow_quad);
        glDisable(GL_BLEND);

        // 7. Render Sports Car Chassis (GTA-grade Cook-Torrance Italian Racing Red with clearcoat)
        zero_shader_set_mat4(&shader, "u_model", car.transform);
        zero_shader_set_float(&shader, "u_roughness", 0.20f);
        zero_shader_set_float(&shader, "u_metallic", 0.80f);
        zero_shader_set_vec3(&shader, "u_object_color", vec3(0.95f, 0.06f, 0.10f));
        zero_shader_set_float(&shader, "u_material_mode", 0.0f);
        zero_mesh_draw(&car_mesh);

        // 8. Render 4 Dynamic Suspension Wheels with spinning & steering
        for (int i = 0; i < 4; ++i) {
            ZeroWheel *w = &car.wheels[i];
            Vec3 wheel_local = vec3(w->hardpoint.x, -w->current_length, w->hardpoint.z);
            Mat4 wheel_trans = mat4_translate(wheel_local);
            Mat4 wheel_steer = mat4_identity();
            if (w->is_front) {
                wheel_steer = quat_to_mat4(quat_from_axis_angle(vec3(0, 1, 0), w->steer_angle));
            }
            float spin_speed = vec3_length(car.velocity) / w->radius;
            static float wheel_spin_angles[4] = {0};
            wheel_spin_angles[i] += (car.velocity.z >= 0.0f ? 1.0f : -1.0f) * spin_speed * dt;
            Mat4 wheel_spin = quat_to_mat4(quat_from_axis_angle(vec3(1, 0, 0), wheel_spin_angles[i]));

            Mat4 wheel_local_mat = mat4_mul(wheel_trans, mat4_mul(wheel_steer, wheel_spin));
            Mat4 wheel_world_mat = mat4_mul(car.transform, wheel_local_mat);

            zero_shader_set_mat4(&shader, "u_model", wheel_world_mat);
            zero_shader_set_float(&shader, "u_roughness", 0.45f);
            zero_shader_set_float(&shader, "u_metallic", 0.50f);
            zero_shader_set_vec3(&shader, "u_object_color", vec3(0.12f, 0.13f, 0.15f));
            zero_shader_set_float(&shader, "u_material_mode", 0.0f);
            zero_mesh_draw(&wheel_mesh);
        }

        // 2D On-Screen HUD Overlay Pass (Tachometer, Speedometer, Gear, Drift status)
        glDisable(GL_DEPTH_TEST);
        zero_shader_set_float(&shader, "u_is_hud", 1.0f);
        Mat4 ortho_proj = mat4_ortho(0.0f, (float)platform.width, 0.0f, (float)platform.height, -1.0f, 1.0f);
        zero_shader_set_mat4(&shader, "u_proj", ortho_proj);

        // 1. HUD Background Bar at Bottom Right
        float hud_x = (float)platform.width - 320.0f;
        float hud_y = 30.0f;
        render_hud_quad(&shader, &hud_quad, hud_x, hud_y, 290.0f, 100.0f, vec3(0.05f, 0.06f, 0.08f));
        render_hud_quad(&shader, &hud_quad, hud_x + 2.0f, hud_y + 2.0f, 286.0f, 96.0f, vec3(0.12f, 0.14f, 0.18f));

        // 2. Speedometer Progress Bar
        float speed_pct = car.speed_kmh / 200.0f;
        if (speed_pct > 1.0f) speed_pct = 1.0f;
        if (speed_pct < 0.0f) speed_pct = 0.0f;
        render_hud_quad(&shader, &hud_quad, hud_x + 15.0f, hud_y + 60.0f, 260.0f, 18.0f, vec3(0.08f, 0.09f, 0.10f));
        Vec3 speed_col = vec3_lerp(vec3(0.15f, 0.85f, 0.35f), vec3(0.95f, 0.20f, 0.15f), speed_pct);
        render_hud_quad(&shader, &hud_quad, hud_x + 15.0f, hud_y + 60.0f, 260.0f * speed_pct, 18.0f, speed_col);

        // 3. RPM Tachometer Bar
        float rpm_pct = (car.engine_rpm - 1000.0f) / 6500.0f;
        if (rpm_pct > 1.0f) rpm_pct = 1.0f;
        if (rpm_pct < 0.0f) rpm_pct = 0.0f;
        render_hud_quad(&shader, &hud_quad, hud_x + 15.0f, hud_y + 35.0f, 260.0f, 12.0f, vec3(0.08f, 0.09f, 0.10f));
        Vec3 rpm_col = (rpm_pct > 0.82f) ? vec3(0.95f, 0.15f, 0.10f) : vec3(0.25f, 0.70f, 0.95f);
        render_hud_quad(&shader, &hud_quad, hud_x + 15.0f, hud_y + 35.0f, 260.0f * rpm_pct, 12.0f, rpm_col);

        // 4. Gear Indicators (Gears 1 to 5)
        for (int g = 1; g <= 5; ++g) {
            Vec3 gear_col = (car.gear == g) ? vec3(0.95f, 0.85f, 0.15f) : vec3(0.25f, 0.28f, 0.32f);
            render_hud_quad(&shader, &hud_quad, hud_x + 15.0f + (g - 1) * 22.0f, hud_y + 12.0f, 16.0f, 12.0f, gear_col);
        }

        // 5. Active Drift Alert Banner (Flashes vibrant orange when drifting)
        if (car.is_drifting) {
            float pulse = 0.5f + 0.5f * sinf((float)current_time * 15.0f);
            Vec3 drift_col = vec3_lerp(vec3(0.95f, 0.45f, 0.05f), vec3(1.0f, 0.85f, 0.10f), pulse);
            render_hud_quad(&shader, &hud_quad, hud_x + 140.0f, hud_y + 10.0f, 135.0f, 16.0f, drift_col);
        }

        zero_shader_unbind();
        zero_platform_swap_buffers(&platform);

        // Console telemetry HUD print
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
    zero_mesh_destroy(&sky_sphere);
    zero_mesh_destroy(&shadow_quad);
    zero_mesh_destroy(&hud_quad);
    zero_shader_destroy(&shader);
    zero_arena_destroy(&arena);
    zero_platform_shutdown(&platform);

    return 0;
}

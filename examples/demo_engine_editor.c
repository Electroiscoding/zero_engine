#include "zero_core.h"
#include "zero_memory.h"
#include "zero_math.h"
#include "zero_platform.h"
#include "zero_gl_loader.h"
#include "zero_shader.h"
#include "zero_mesh.h"
#include "zero_camera.h"
#include "zero_scene.h"
#include "zero_physics.h"
#include "zero_csm.h"
#include "zero_sky.h"
#include "zero_inspect.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

// Shaders with per-entity u_model transformation
static const char *editor_vert_shader(void) {
    return
        "#version 330 core\n"
        "layout(location = 0) in vec3 a_pos;\n"
        "layout(location = 1) in vec3 a_normal;\n"
        "layout(location = 2) in vec2 a_uv;\n"
        "uniform mat4 u_model;\n"
        "uniform mat4 u_view;\n"
        "uniform mat4 u_proj;\n"
        "out vec3 v_world_pos;\n"
        "out vec3 v_normal;\n"
        "out vec2 v_uv;\n"
        "void main() {\n"
        "    vec4 world_p = u_model * vec4(a_pos, 1.0);\n"
        "    v_world_pos = world_p.xyz;\n"
        "    v_normal = mat3(u_model) * a_normal;\n"
        "    v_uv = a_uv;\n"
        "    gl_Position = u_proj * u_view * world_p;\n"
        "}\n";
}

static const char *editor_frag_shader(void) {
    return
        "#version 330 core\n"
        "in vec3 v_world_pos;\n"
        "in vec3 v_normal;\n"
        "in vec2 v_uv;\n"
        "uniform vec3 u_light_pos;\n"
        "uniform vec3 u_light_color;\n"
        "uniform vec3 u_ambient_color;\n"
        "uniform vec3 u_base_color;\n"
        "uniform float u_is_grid;\n"
        "out vec4 frag_color;\n"
        "void main() {\n"
        "    vec3 N = normalize(v_normal);\n"
        "    vec3 L = normalize(u_light_pos);\n"
        "    float NdotL = max(dot(N, L), 0.0);\n"
        "    vec3 albedo = u_base_color.rgb;\n"
        "    if (u_is_grid > 0.5) {\n"
        "        // Procedural Studio Grid lines\n"
        "        vec2 grid = abs(fract(v_world_pos.xz - 0.5) - 0.5) / fwidth(v_world_pos.xz);\n"
        "        float line = min(grid.x, grid.y);\n"
        "        float c = 1.0 - min(line, 1.0);\n"
        "        albedo = mix(albedo, vec3(0.55, 0.60, 0.70), c * 0.4);\n"
        "        if (abs(v_world_pos.x) < 0.05) albedo = vec3(0.9, 0.2, 0.2); // X-axis (Red)\n"
        "        if (abs(v_world_pos.z) < 0.05) albedo = vec3(0.2, 0.4, 0.9); // Z-axis (Blue)\n"
        "    }\n"
        "    vec3 diffuse = albedo * u_light_color * NdotL;\n"
        "    vec3 ambient = albedo * u_ambient_color;\n"
        "    frag_color = vec4(diffuse + ambient, 1.0);\n"
        "}\n";
}

int main(void) {
    ZeroPlatformConfig cfg = {
        .title = "Zero Engine: Studio Editor & Project Sandbox",
        .width = 1280,
        .height = 720,
        .headless = false
    };

    ZeroPlatformState platform;
    if (!zero_platform_init(&platform, &cfg)) {
        fprintf(stderr, "Failed to initialize Zero Engine platform\n");
        return 1;
    }

    if (platform.headless) {
        printf("[Zero Engine Studio] Headless environment detected. Terminating.\n");
        zero_platform_shutdown(&platform);
        return 0;
    }

    printf("\n============================================================\n");
    printf("===           ZERO ENGINE STUDIO & SANDBOX               ===\n");
    printf("============================================================\n");
    printf(" Interactive Scene & Game Development Studio Controls:\n");
    printf("   [1] : Spawn Dynamic Rigid-Body Sphere\n");
    printf("   [2] : Spawn Dynamic Rigid-Body Cube\n");
    printf("   [3] : Spawn Humanoid AI Agent\n");
    printf("   [4] : Cycle Time-of-Day (Midday -> Sunset -> Twilight)\n");
    printf("   [T] : Print Live Telemetry JSON Perception Snapshot\n");
    printf("   [W/A/S/D] : Fly Free Studio Camera\n");
    printf("   [Right/Left Mouse Drag] : Look / Orbit Camera\n");
    printf("   [R] : Reset Scene & Clear Project\n");
    printf("   [ESC] : Exit Engine Studio\n");
    printf("============================================================\n\n");

    ZeroScene *scene = zero_scene_create("ZeroStudio_Project");
    if (!scene) {
        fprintf(stderr, "Failed to create scene\n");
        zero_platform_shutdown(&platform);
        return 1;
    }

    // Allocate procedural standard meshes for Studio primitives
    ZeroArena mesh_arena = zero_arena_create(16 * 1024 * 1024);
    ZeroMeshData cube_data = zero_mesh_gen_cube(&mesh_arena, 1.0f);
    ZeroMeshData sphere_data = zero_mesh_gen_sphere(&mesh_arena, 0.5f, 32, 32);
    ZeroMeshData plane_data = zero_mesh_gen_plane(&mesh_arena, 60.0f, 60.0f);

    ZeroMesh cube_mesh = zero_mesh_upload(&cube_data);
    ZeroMesh sphere_mesh = zero_mesh_upload(&sphere_data);
    ZeroMesh plane_mesh = zero_mesh_upload(&plane_data);

    // Default Scene Lighting and Grid Plane
    zero_entity_spawn_directional_light(scene, vec3(0.5f, 0.8f, 0.3f), vec3(1.0f, 0.95f, 0.85f));
    ZeroEntity *ground_ent = zero_entity_spawn_plane(scene, "GroundPlane", vec3(0, 0, 0), vec2(60.0f, 60.0f), vec4(0.20f, 0.22f, 0.26f, 1.0f));
    ground_ent->mesh = &plane_mesh;

    // Ground Static Physics Body
    ZeroRigidBody ground_body;
    memset(&ground_body, 0, sizeof(ground_body));
    ground_body.position = vec3(0, -0.5f, 0);
    ground_body.collider.type = ZERO_COLLIDER_BOX;
    ground_body.collider.box.half_extents = vec3(30.0f, 0.5f, 30.0f);
    ground_body.is_static = true;
    zero_physics_add_body(scene->physics, &ground_body);

    // Initial Humanoid Player & Initial Cubes
    ZeroEntity *player = zero_entity_spawn_humanoid(scene, vec3(0.0f, 1.0f, 0.0f));
    player->mesh = &cube_mesh;
    player->color = vec4(0.2f, 0.85f, 0.4f, 1.0f);

    // Pre-populate some starter blocks for game designers
    ZeroEntity *starter_cube = zero_entity_spawn_cube(scene, "Obstacle_1", vec3(-2.5f, 1.0f, 1.0f), vec3(2.0f, 2.0f, 2.0f), vec4(0.9f, 0.5f, 0.1f, 1.0f));
    starter_cube->mesh = &cube_mesh;

    ZeroEntity *starter_sphere = zero_entity_spawn_sphere(scene, "Orb_1", vec3(2.5f, 1.0f, 1.0f), 1.0f, vec4(0.3f, 0.6f, 0.95f, 1.0f));
    starter_sphere->mesh = &sphere_mesh;

    // Studio Camera
    ZeroCamera cam = zero_camera_create(
        vec3(0.0f, 7.0f, 14.0f),
        ZERO_DEG2RAD(60.0f),
        (float)platform.width / (float)platform.height,
        0.1f,
        1000.0f
    );
    cam.pitch = ZERO_DEG2RAD(-22.0f);
    zero_camera_update_vectors(&cam);

    // Viewport Shader
    ZeroShader shader = zero_shader_create(editor_vert_shader(), editor_frag_shader());

    glEnable(GL_DEPTH_TEST);

    double last_time = zero_platform_get_time();
    int last_mouse_x = platform.mouse_x;
    int last_mouse_y = platform.mouse_y;
    float time_of_day = 30.0f; // Noon
    int spawn_counter = 0;

    while (!platform.should_close) {
        zero_platform_poll_events(&platform);

        double current_time = zero_platform_get_time();
        float dt = (float)(current_time - last_time);
        if (dt > 0.1f) dt = 0.1f;
        last_time = current_time;

        // Camera Navigation (W/A/S/D)
        float cam_speed = 12.0f * dt;
        if (platform.keys['w'] || platform.keys['W']) zero_camera_move(&cam, vec3_scale(cam.forward, cam_speed));
        if (platform.keys['s'] || platform.keys['S']) zero_camera_move(&cam, vec3_scale(cam.forward, -cam_speed));
        if (platform.keys['a'] || platform.keys['A']) zero_camera_move(&cam, vec3_scale(cam.right, -cam_speed));
        if (platform.keys['d'] || platform.keys['D']) zero_camera_move(&cam, vec3_scale(cam.right, cam_speed));
        if (platform.keys['q'] || platform.keys['Q']) zero_camera_move(&cam, vec3_scale(cam.up, cam_speed));
        if (platform.keys['e'] || platform.keys['E']) zero_camera_move(&cam, vec3_scale(cam.up, -cam_speed));

        if (platform.mouse_buttons[1] || platform.mouse_buttons[3]) {
            float dx = (float)(platform.mouse_x - last_mouse_x) * 0.003f;
            float dy = (float)(platform.mouse_y - last_mouse_y) * 0.003f;
            zero_camera_rotate(&cam, -dy, -dx);
        }
        last_mouse_x = platform.mouse_x;
        last_mouse_y = platform.mouse_y;

        // Interactive Project Creation Tools
        // [1] Spawn Sphere
        if (platform.keys['1']) {
            platform.keys['1'] = false;
            char name[32];
            snprintf(name, sizeof(name), "Sphere_%d", ++spawn_counter);
            Vec3 spawn_pos = vec3((float)((spawn_counter * 3) % 9) - 4.0f, 9.0f + (float)(spawn_counter * 0.3f), (float)((spawn_counter * 2) % 5) - 2.0f);
            ZeroEntity *ent = zero_entity_spawn_sphere(scene, name, spawn_pos, 0.75f, vec4(0.95f, 0.25f, 0.2f, 1.0f));
            ent->mesh = &sphere_mesh;

            ZeroRigidBody body;
            memset(&body, 0, sizeof(body));
            body.position = spawn_pos;
            body.mass = 5.0f;
            body.collider.type = ZERO_COLLIDER_SPHERE;
            body.collider.sphere.radius = 0.75f;
            body.restitution = 0.55f;
            ZeroRigidBody *rb = zero_physics_add_body(scene->physics, &body);
            if (rb && ent) ent->rigid_body_id = (int32_t)rb->id;
            printf("[Zero Engine Studio] Spawned Entity: %s (Sphere Rigid Body)\n", name);
        }

        // [2] Spawn Cube
        if (platform.keys['2']) {
            platform.keys['2'] = false;
            char name[32];
            snprintf(name, sizeof(name), "Cube_%d", ++spawn_counter);
            Vec3 spawn_pos = vec3((float)((spawn_counter * 3) % 9) - 4.0f, 9.0f + (float)(spawn_counter * 0.3f), (float)((spawn_counter * 2) % 5) - 2.0f);
            ZeroEntity *ent = zero_entity_spawn_cube(scene, name, spawn_pos, vec3(1.2f, 1.2f, 1.2f), vec4(0.2f, 0.6f, 0.95f, 1.0f));
            ent->mesh = &cube_mesh;

            ZeroRigidBody body;
            memset(&body, 0, sizeof(body));
            body.position = spawn_pos;
            body.mass = 8.0f;
            body.collider.type = ZERO_COLLIDER_BOX;
            body.collider.box.half_extents = vec3(0.6f, 0.6f, 0.6f);
            body.restitution = 0.25f;
            ZeroRigidBody *rb = zero_physics_add_body(scene->physics, &body);
            if (rb && ent) ent->rigid_body_id = (int32_t)rb->id;
            printf("[Zero Engine Studio] Spawned Entity: %s (Cube Rigid Body)\n", name);
        }

        // [3] Spawn Humanoid
        if (platform.keys['3']) {
            platform.keys['3'] = false;
            char name[32];
            snprintf(name, sizeof(name), "Humanoid_%d", ++spawn_counter);
            Vec3 spawn_pos = vec3((float)((spawn_counter * 2) % 7) - 3.0f, 1.0f, (float)((spawn_counter * 3) % 5) - 2.0f);
            ZeroEntity *h = zero_entity_spawn_humanoid(scene, spawn_pos);
            h->mesh = &cube_mesh;
            h->color = vec4(0.3f, 0.9f, 0.5f, 1.0f);
            printf("[Zero Engine Studio] Spawned Entity: %s (Humanoid AI Agent)\n", name);
        }

        // [4] Cycle Time of Day
        if (platform.keys['4']) {
            platform.keys['4'] = false;
            time_of_day += 25.0f;
            if (time_of_day > 120.0f) time_of_day = 0.0f;
            Vec3 sun_dir = zero_sky_calc_sun_direction(time_of_day, 120.0f, 0.409f);
            scene->sun_direction = sun_dir;
            ZeroSkyPhase phase = zero_sky_calc_phase(sun_dir.y);
            const char *phase_names[] = {"Midday", "Golden Hour", "Twilight", "Night"};
            printf("[Zero Engine Studio] Diurnal Cycle Time: %.1fs | Solar Phase: %s\n", time_of_day, phase_names[phase]);
        }

        // [T] Print Live Perception Telemetry
        if (platform.keys['t'] || platform.keys['T']) {
            platform.keys['t'] = false;
            platform.keys['T'] = false;
            char json_buf[2048];
            scene->camera = cam;
            zero_engine_inspect_scene(scene, json_buf, sizeof(json_buf));
            printf("\n--- [ZERO ENGINE: LIVE AI AGENT PERCEPTION SNAPSHOT] ---\n%s\n--------------------------------------------------------\n", json_buf);
        }

        // [R] Reset Scene
        if (platform.keys['r'] || platform.keys['R']) {
            platform.keys['r'] = false;
            platform.keys['R'] = false;
            printf("[Zero Engine Studio] Resetting scene to clean template...\n");
            zero_scene_destroy(scene);
            scene = zero_scene_create("ZeroStudio_Project");
            zero_entity_spawn_directional_light(scene, vec3(0.5f, 0.8f, 0.3f), vec3(1.0f, 0.95f, 0.85f));
            ground_ent = zero_entity_spawn_plane(scene, "GroundPlane", vec3(0, 0, 0), vec2(60.0f, 60.0f), vec4(0.20f, 0.22f, 0.26f, 1.0f));
            ground_ent->mesh = &plane_mesh;

            ground_body.position = vec3(0, -0.5f, 0);
            zero_physics_add_body(scene->physics, &ground_body);

            player = zero_entity_spawn_humanoid(scene, vec3(0.0f, 1.0f, 0.0f));
            player->mesh = &cube_mesh;
            player->color = vec4(0.2f, 0.85f, 0.4f, 1.0f);
            spawn_counter = 0;
        }

        // Step physics and hierarchical transforms
        zero_scene_step(scene, dt);

        // Render Frame
        glViewport(0, 0, platform.width, platform.height);
        glClearColor(0.12f, 0.14f, 0.18f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        Mat4 view = zero_camera_get_view(&cam);
        Mat4 proj = zero_camera_get_projection(&cam);

        zero_shader_bind(&shader);
        zero_shader_set_mat4(&shader, "u_view", view);
        zero_shader_set_mat4(&shader, "u_proj", proj);
        zero_shader_set_vec3(&shader, "u_light_pos", scene->sun_direction);
        zero_shader_set_vec3(&shader, "u_light_color", vec3(1.0f, 0.95f, 0.85f));
        zero_shader_set_vec3(&shader, "u_ambient_color", vec3(0.28f, 0.32f, 0.40f));

        // Draw entities with their individual model matrix & base color
        for (uint32_t i = 0; i < ZERO_MAX_SCENE_ENTITIES; ++i) {
            const ZeroEntity *e = &scene->entities[i];
            if (e->is_active && e->is_visible && e->mesh && e->mesh->is_valid) {
                zero_shader_set_mat4(&shader, "u_model", e->world_matrix);
                zero_shader_set_vec3(&shader, "u_base_color", vec3(e->color.x, e->color.y, e->color.z));
                float is_grid = (strcmp(e->name, "GroundPlane") == 0) ? 1.0f : 0.0f;
                zero_shader_set_float(&shader, "u_is_grid", is_grid);
                zero_mesh_draw(e->mesh);
            }
        }

        zero_platform_swap_buffers(&platform);
    }

    zero_scene_destroy(scene);
    zero_mesh_destroy(&cube_mesh);
    zero_mesh_destroy(&sphere_mesh);
    zero_mesh_destroy(&plane_mesh);
    zero_arena_destroy(&mesh_arena);
    zero_shader_destroy(&shader);
    zero_platform_shutdown(&platform);
    printf("[Zero Engine Studio] Session closed cleanly.\n");
    return 0;
}

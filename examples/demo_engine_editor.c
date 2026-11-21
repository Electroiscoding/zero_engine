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
#include "zero_ui.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

// Shaders with per-entity u_model transformation & selection highlight
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
        "uniform float u_selected;\n"
        "out vec4 frag_color;\n"
        "void main() {\n"
        "    vec3 N = normalize(v_normal);\n"
        "    vec3 L = normalize(u_light_pos);\n"
        "    float NdotL = max(dot(N, L), 0.0);\n"
        "    vec3 albedo = u_base_color.rgb;\n"
        "    if (u_is_grid > 0.5) {\n"
        "        vec2 grid = abs(fract(v_world_pos.xz - 0.5) - 0.5) / fwidth(v_world_pos.xz);\n"
        "        float line = min(grid.x, grid.y);\n"
        "        float c = 1.0 - min(line, 1.0);\n"
        "        albedo = mix(albedo, vec3(0.52, 0.56, 0.64), c * 0.45);\n"
        "        if (abs(v_world_pos.x) < 0.05) albedo = vec3(0.95, 0.25, 0.25); // X-axis (Red)\n"
        "        if (abs(v_world_pos.z) < 0.05) albedo = vec3(0.25, 0.50, 0.95); // Z-axis (Blue)\n"
        "    }\n"
        "    vec3 diffuse = albedo * u_light_color * NdotL;\n"
        "    vec3 ambient = albedo * u_ambient_color;\n"
        "    vec3 final_col = diffuse + ambient;\n"
        "    if (u_selected > 0.5) {\n"
        "        final_col = mix(final_col, vec3(1.0, 0.85, 0.2), 0.35);\n"
        "    }\n"
        "    frag_color = vec4(final_col, 1.0);\n"
        "}\n";
}

#define MAX_LOG_LINES 6
static char g_log_lines[MAX_LOG_LINES][128];
static int g_log_head = 0;

static void studio_log(const char *msg) {
    if (!msg) return;
    snprintf(g_log_lines[g_log_head], sizeof(g_log_lines[0]), "%s", msg);
    g_log_head = (g_log_head + 1) % MAX_LOG_LINES;
    printf("[STUDIO] %s\n", msg);
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

    // Allocate procedural standard meshes for Studio primitives
    ZeroArena mesh_arena = zero_arena_create(16 * 1024 * 1024);
    ZeroMeshData cube_data = zero_mesh_gen_cube(&mesh_arena, 1.0f);
    ZeroMeshData sphere_data = zero_mesh_gen_sphere(&mesh_arena, 0.5f, 32, 32);
    ZeroMeshData plane_data = zero_mesh_gen_plane(&mesh_arena, 60.0f, 60.0f);

    ZeroMesh cube_mesh = zero_mesh_upload(&cube_data);
    ZeroMesh sphere_mesh = zero_mesh_upload(&sphere_data);
    ZeroMesh plane_mesh = zero_mesh_upload(&plane_data);

    ZeroScene *scene = zero_scene_create("ZeroStudio_Project");
    if (!scene) {
        fprintf(stderr, "Failed to create scene\n");
        zero_platform_shutdown(&platform);
        return 1;
    }

    // Default Scene Lighting and Grid Plane
    zero_entity_spawn_directional_light(scene, vec3(0.5f, 0.8f, 0.3f), vec3(1.0f, 0.95f, 0.85f));
    ZeroEntity *ground_ent = zero_entity_spawn_plane(scene, "GroundPlane", vec3(0, 0, 0), vec2(60.0f, 60.0f), vec4(0.20f, 0.22f, 0.26f, 1.0f));
    if (ground_ent) ground_ent->mesh = &plane_mesh;

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
    if (player) {
        player->mesh = &cube_mesh;
        player->color = vec4(0.2f, 0.85f, 0.4f, 1.0f);
    }

    ZeroEntity *starter_cube = zero_entity_spawn_cube(scene, "Obstacle_1", vec3(-2.5f, 1.0f, 1.0f), vec3(2.0f, 2.0f, 2.0f), vec4(0.9f, 0.5f, 0.1f, 1.0f));
    if (starter_cube) starter_cube->mesh = &cube_mesh;

    ZeroEntity *starter_sphere = zero_entity_spawn_sphere(scene, "Orb_1", vec3(2.5f, 1.0f, 1.0f), 1.0f, vec4(0.3f, 0.6f, 0.95f, 1.0f));
    if (starter_sphere) starter_sphere->mesh = &sphere_mesh;

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

    // Immediate Mode GUI System
    ZeroUI ui;
    if (!zero_ui_init(&ui)) {
        fprintf(stderr, "Failed to initialize ZeroUI overlay\n");
    }

    studio_log("Zero Engine Studio Editor initialized.");
    studio_log("Ready. Select entities or spawn objects from the toolbar.");

    glEnable(GL_DEPTH_TEST);

    double last_time = zero_platform_get_time();
    int last_mouse_x = platform.mouse_x;
    int last_mouse_y = platform.mouse_y;
    float time_of_day = 30.0f; // Noon
    int spawn_counter = 0;
    int32_t selected_entity_id = -1;
    bool sim_running = true;
    bool step_one_frame = false;
    int fps_display = 60;
    int frame_count = 0;
    double fps_timer = last_time;

    while (!platform.should_close) {
        zero_platform_poll_events(&platform);

        double current_time = zero_platform_get_time();
        float dt = (float)(current_time - last_time);
        if (dt > 0.1f) dt = 0.1f;
        last_time = current_time;

        frame_count++;
        if (current_time - fps_timer >= 1.0) {
            fps_display = frame_count;
            frame_count = 0;
            fps_timer = current_time;
        }

        // Camera Navigation (W/A/S/D)
        float cam_speed = 12.0f * dt;
        if (platform.keys['w'] || platform.keys['W']) zero_camera_move(&cam, vec3_scale(cam.forward, cam_speed));
        if (platform.keys['s'] || platform.keys['S']) zero_camera_move(&cam, vec3_scale(cam.forward, -cam_speed));
        if (platform.keys['a'] || platform.keys['A']) zero_camera_move(&cam, vec3_scale(cam.right, -cam_speed));
        if (platform.keys['d'] || platform.keys['D']) zero_camera_move(&cam, vec3_scale(cam.right, cam_speed));
        if (platform.keys['q'] || platform.keys['Q']) zero_camera_move(&cam, vec3_scale(cam.up, cam_speed));
        if (platform.keys['e'] || platform.keys['E']) zero_camera_move(&cam, vec3_scale(cam.up, -cam_speed));

        // 3D Camera Orbit with mouse (ONLY when mouse is NOT interacting with UI panels)
        if (!ui.mouse_over_ui && (platform.mouse_buttons[1] || platform.mouse_buttons[3])) {
            float dx = (float)(platform.mouse_x - last_mouse_x) * 0.003f;
            float dy = (float)(platform.mouse_y - last_mouse_y) * 0.003f;
            zero_camera_rotate(&cam, -dy, -dx);
        }
        last_mouse_x = platform.mouse_x;
        last_mouse_y = platform.mouse_y;

        // Step physics simulation
        if (sim_running || step_one_frame) {
            zero_scene_step(scene, dt);
            step_one_frame = false;
        }

        // ====================================================================
        // 3D Viewport Render Pass
        // ====================================================================
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

        for (uint32_t i = 0; i < ZERO_MAX_SCENE_ENTITIES; ++i) {
            const ZeroEntity *e = &scene->entities[i];
            if (e->is_active && e->is_visible && e->mesh && e->mesh->is_valid) {
                zero_shader_set_mat4(&shader, "u_model", e->world_matrix);
                zero_shader_set_vec3(&shader, "u_base_color", vec3(e->color.x, e->color.y, e->color.z));
                float is_grid = (strcmp(e->name, "GroundPlane") == 0) ? 1.0f : 0.0f;
                zero_shader_set_float(&shader, "u_is_grid", is_grid);
                float is_sel = ((int32_t)i == selected_entity_id) ? 1.0f : 0.0f;
                zero_shader_set_float(&shader, "u_selected", is_sel);
                zero_mesh_draw(e->mesh);
            }
        }

        // ====================================================================
        // 2D Studio Editor GUI Pass (Immediate-Mode UI)
        // ====================================================================
        float sw = (float)platform.width;
        float sh = (float)platform.height;
        zero_ui_begin(&ui, sw, sh, (float)platform.mouse_x, (float)platform.mouse_y, platform.mouse_buttons[1]);

        // --------------------------------------------------------------------
        // 1. Top Main Menu Bar & Toolbar
        // --------------------------------------------------------------------
        float menu_h = 32.0f;
        zero_ui_draw_rect(&ui, 0, 0, sw, menu_h, vec4(0.15f, 0.16f, 0.20f, 0.98f));
        zero_ui_draw_rect_outline(&ui, 0, 0, sw, menu_h, vec4(0.26f, 0.28f, 0.35f, 1.0f), 1.0f);

        // Logo
        zero_ui_draw_text(&ui, 10.0f, 8.0f, "ZERO ENGINE STUDIO", vec4(0.35f, 0.75f, 1.0f, 1.0f), 1.0f);

        // Quick Spawn Buttons
        float btn_x = 180.0f;
        if (zero_ui_button(&ui, btn_x, 4.0f, 75.0f, 24.0f, "+ Cube") || platform.keys['2']) {
            platform.keys['2'] = false;
            char name[32];
            snprintf(name, sizeof(name), "Cube_%d", ++spawn_counter);
            Vec3 spawn_pos = vec3((float)((spawn_counter * 3) % 9) - 4.0f, 9.0f, (float)((spawn_counter * 2) % 5) - 2.0f);
            ZeroEntity *ent = zero_entity_spawn_cube(scene, name, spawn_pos, vec3(1.2f, 1.2f, 1.2f), vec4(0.2f, 0.6f, 0.95f, 1.0f));
            if (ent) ent->mesh = &cube_mesh;

            ZeroRigidBody body;
            memset(&body, 0, sizeof(body));
            body.position = spawn_pos;
            body.mass = 8.0f;
            body.collider.type = ZERO_COLLIDER_BOX;
            body.collider.box.half_extents = vec3(0.6f, 0.6f, 0.6f);
            body.restitution = 0.25f;
            ZeroRigidBody *rb = zero_physics_add_body(scene->physics, &body);
            if (rb && ent) ent->rigid_body_id = (int32_t)rb->id;
            selected_entity_id = (ent) ? (int32_t)ent->id : -1;

            char buf[128];
            snprintf(buf, sizeof(buf), "Spawned Entity: %s (RigidBody Cube)", name);
            studio_log(buf);
        }
        btn_x += 80.0f;

        if (zero_ui_button(&ui, btn_x, 4.0f, 80.0f, 24.0f, "+ Sphere") || platform.keys['1']) {
            platform.keys['1'] = false;
            char name[32];
            snprintf(name, sizeof(name), "Sphere_%d", ++spawn_counter);
            Vec3 spawn_pos = vec3((float)((spawn_counter * 3) % 9) - 4.0f, 9.0f, (float)((spawn_counter * 2) % 5) - 2.0f);
            ZeroEntity *ent = zero_entity_spawn_sphere(scene, name, spawn_pos, 0.75f, vec4(0.95f, 0.25f, 0.2f, 1.0f));
            if (ent) ent->mesh = &sphere_mesh;

            ZeroRigidBody body;
            memset(&body, 0, sizeof(body));
            body.position = spawn_pos;
            body.mass = 5.0f;
            body.collider.type = ZERO_COLLIDER_SPHERE;
            body.collider.sphere.radius = 0.75f;
            body.restitution = 0.55f;
            ZeroRigidBody *rb = zero_physics_add_body(scene->physics, &body);
            if (rb && ent) ent->rigid_body_id = (int32_t)rb->id;
            selected_entity_id = (ent) ? (int32_t)ent->id : -1;

            char buf[128];
            snprintf(buf, sizeof(buf), "Spawned Entity: %s (RigidBody Sphere)", name);
            studio_log(buf);
        }
        btn_x += 85.0f;

        if (zero_ui_button(&ui, btn_x, 4.0f, 95.0f, 24.0f, "+ Humanoid") || platform.keys['3']) {
            platform.keys['3'] = false;
            char name[32];
            snprintf(name, sizeof(name), "Humanoid_%d", ++spawn_counter);
            Vec3 spawn_pos = vec3((float)((spawn_counter * 2) % 7) - 3.0f, 1.0f, (float)((spawn_counter * 3) % 5) - 2.0f);
            ZeroEntity *h = zero_entity_spawn_humanoid(scene, spawn_pos);
            if (h) {
                h->mesh = &cube_mesh;
                h->color = vec4(0.3f, 0.9f, 0.5f, 1.0f);
                selected_entity_id = (int32_t)h->id;
            }
            char buf[128];
            snprintf(buf, sizeof(buf), "Spawned Entity: %s (Humanoid Agent)", name);
            studio_log(buf);
        }
        btn_x += 105.0f;

        // Play / Pause Simulation
        const char *sim_txt = sim_running ? "❚❚ Pause" : "▶ Play";
        if (zero_ui_button_toggle(&ui, btn_x, 4.0f, 75.0f, 24.0f, sim_txt, sim_running)) {
            sim_running = !sim_running;
            studio_log(sim_running ? "Physics simulation resumed." : "Physics simulation paused.");
        }
        btn_x += 80.0f;

        if (!sim_running) {
            if (zero_ui_button(&ui, btn_x, 4.0f, 65.0f, 24.0f, "↻ Step")) {
                step_one_frame = true;
                studio_log("Stepped simulation 1 tick.");
            }
            btn_x += 70.0f;
        }

        // Reset Scene
        if (zero_ui_button(&ui, btn_x, 4.0f, 70.0f, 24.0f, "⟲ Reset") || platform.keys['r'] || platform.keys['R']) {
            platform.keys['r'] = false;
            platform.keys['R'] = false;
            zero_scene_destroy(scene);
            scene = zero_scene_create("ZeroStudio_Project");
            zero_entity_spawn_directional_light(scene, vec3(0.5f, 0.8f, 0.3f), vec3(1.0f, 0.95f, 0.85f));
            ground_ent = zero_entity_spawn_plane(scene, "GroundPlane", vec3(0, 0, 0), vec2(60.0f, 60.0f), vec4(0.20f, 0.22f, 0.26f, 1.0f));
            if (ground_ent) ground_ent->mesh = &plane_mesh;

            ground_body.position = vec3(0, -0.5f, 0);
            zero_physics_add_body(scene->physics, &ground_body);

            player = zero_entity_spawn_humanoid(scene, vec3(0.0f, 1.0f, 0.0f));
            if (player) {
                player->mesh = &cube_mesh;
                player->color = vec4(0.2f, 0.85f, 0.4f, 1.0f);
            }
            spawn_counter = 0;
            selected_entity_id = -1;
            studio_log("Reset scene to clean project template.");
        }
        btn_x += 75.0f;

        // Cycle Sky Lighting
        if (zero_ui_button(&ui, btn_x, 4.0f, 75.0f, 24.0f, "☀ Sky") || platform.keys['4']) {
            platform.keys['4'] = false;
            time_of_day += 25.0f;
            if (time_of_day > 120.0f) time_of_day = 0.0f;
            Vec3 sun_dir = zero_sky_calc_sun_direction(time_of_day, 120.0f, 0.409f);
            scene->sun_direction = sun_dir;
            ZeroSkyPhase phase = zero_sky_calc_phase(sun_dir.y);
            const char *phase_names[] = {"Midday", "Golden Hour", "Twilight", "Night"};
            char buf[128];
            snprintf(buf, sizeof(buf), "Sun Diurnal Cycle: %.1fs | Solar Phase: %s", time_of_day, phase_names[phase]);
            studio_log(buf);
        }
        btn_x += 80.0f;

        // AI Perception Inspect
        if (zero_ui_button(&ui, btn_x, 4.0f, 90.0f, 24.0f, "👁 AI JSON") || platform.keys['t'] || platform.keys['T']) {
            platform.keys['t'] = false;
            platform.keys['T'] = false;
            char json_buf[2048];
            scene->camera = cam;
            zero_engine_inspect_scene(scene, json_buf, sizeof(json_buf));
            printf("\n--- [ZERO ENGINE: LIVE AI AGENT PERCEPTION SNAPSHOT] ---\n%s\n--------------------------------------------------------\n", json_buf);
            studio_log("Exported AI perception telemetry JSON to console.");
        }

        // --------------------------------------------------------------------
        // 2. Left Panel: Scene Hierarchy
        // --------------------------------------------------------------------
        float h_x = 10.0f;
        float h_y = 40.0f;
        float h_w = 230.0f;
        float h_h = sh - 210.0f;
        zero_ui_panel(&ui, h_x, h_y, h_w, h_h, "SCENE HIERARCHY");

        float item_y = h_y + 30.0f;
        for (uint32_t i = 0; i < ZERO_MAX_SCENE_ENTITIES; ++i) {
            ZeroEntity *e = &scene->entities[i];
            if (e->is_active) {
                if (item_y + 24.0f > h_y + h_h - 10.0f) break;
                char label[64];
                const char *type_tag = (e->type == ZERO_ENTITY_TYPE_LIGHT_DIRECTIONAL) ? "LGT" :
                                       (e->rigid_body_id > 0) ? "RB" :
                                       (e->mesh == &sphere_mesh) ? "SPH" :
                                       (e->mesh == &plane_mesh) ? "PLN" : "BOX";
                snprintf(label, sizeof(label), "%-14s [%s]", e->name, type_tag);
                bool is_sel = (selected_entity_id == (int32_t)i);
                if (zero_ui_menu_item(&ui, h_x + 6.0f, item_y, h_w - 12.0f, 22.0f, label, is_sel)) {
                    selected_entity_id = (int32_t)i;
                    char buf[128];
                    snprintf(buf, sizeof(buf), "Selected entity: %s (ID: %u)", e->name, i);
                    studio_log(buf);
                }
                item_y += 24.0f;
            }
        }

        // --------------------------------------------------------------------
        // 3. Right Panel: Inspector & Properties
        // --------------------------------------------------------------------
        float ins_w = 280.0f;
        float ins_x = sw - ins_w - 10.0f;
        float ins_y = 40.0f;
        float ins_h = sh - 210.0f;
        zero_ui_panel(&ui, ins_x, ins_y, ins_w, ins_h, "INSPECTOR");

        if (selected_entity_id >= 0 && selected_entity_id < (int32_t)ZERO_MAX_SCENE_ENTITIES &&
            scene->entities[selected_entity_id].is_active) {
            ZeroEntity *sel = &scene->entities[selected_entity_id];

            float py = ins_y + 32.0f;
            char header_str[64];
            snprintf(header_str, sizeof(header_str), "Entity: %s", sel->name);
            zero_ui_draw_text(&ui, ins_x + 10.0f, py, header_str, vec4(0.95f, 0.95f, 1.0f, 1.0f), 1.0f);
            py += 24.0f;

            zero_ui_draw_text(&ui, ins_x + 10.0f, py, "--- TRANSFORM ---", vec4(0.55f, 0.65f, 0.85f, 1.0f), 0.9f);
            py += 20.0f;

            char pos_str[64];
            snprintf(pos_str, sizeof(pos_str), "Pos: (%.2f, %.2f, %.2f)", sel->world_pos.x, sel->world_pos.y, sel->world_pos.z);
            zero_ui_draw_text(&ui, ins_x + 10.0f, py, pos_str, vec4(0.85f, 0.88f, 0.92f, 1.0f), 0.9f);
            py += 20.0f;

            // Transform Nudge Buttons
            if (zero_ui_button(&ui, ins_x + 10.0f, py, 38.0f, 20.0f, "X-")) {
                sel->local_pos.x -= 0.5f;
                sel->is_dirty = true;
            }
            if (zero_ui_button(&ui, ins_x + 52.0f, py, 38.0f, 20.0f, "X+")) {
                sel->local_pos.x += 0.5f;
                sel->is_dirty = true;
            }
            if (zero_ui_button(&ui, ins_x + 98.0f, py, 38.0f, 20.0f, "Y-")) {
                sel->local_pos.y -= 0.5f;
                sel->is_dirty = true;
            }
            if (zero_ui_button(&ui, ins_x + 140.0f, py, 38.0f, 20.0f, "Y+")) {
                sel->local_pos.y += 0.5f;
                sel->is_dirty = true;
            }
            if (zero_ui_button(&ui, ins_x + 186.0f, py, 38.0f, 20.0f, "Z-")) {
                sel->local_pos.z -= 0.5f;
                sel->is_dirty = true;
            }
            if (zero_ui_button(&ui, ins_x + 228.0f, py, 38.0f, 20.0f, "Z+")) {
                sel->local_pos.z += 0.5f;
                sel->is_dirty = true;
            }
            py += 28.0f;

            // Material Color Preset Buttons & Swatch
            zero_ui_draw_text(&ui, ins_x + 10.0f, py, "--- MATERIAL COLOR ---", vec4(0.55f, 0.65f, 0.85f, 1.0f), 0.9f);
            py += 20.0f;

            // Color swatch preview box
            zero_ui_draw_rect(&ui, ins_x + 10.0f, py, 36.0f, 22.0f, sel->color);
            zero_ui_draw_rect_outline(&ui, ins_x + 10.0f, py, 36.0f, 22.0f, vec4(0.9f, 0.9f, 0.9f, 1.0f), 1.0f);

            if (zero_ui_button(&ui, ins_x + 52.0f, py, 50.0f, 22.0f, "Orange")) sel->color = vec4(0.95f, 0.55f, 0.15f, 1.0f);
            if (zero_ui_button(&ui, ins_x + 106.0f, py, 50.0f, 22.0f, "Cyan")) sel->color = vec4(0.15f, 0.80f, 0.95f, 1.0f);
            if (zero_ui_button(&ui, ins_x + 160.0f, py, 50.0f, 22.0f, "Green")) sel->color = vec4(0.25f, 0.90f, 0.45f, 1.0f);
            if (zero_ui_button(&ui, ins_x + 214.0f, py, 50.0f, 22.0f, "Red")) sel->color = vec4(0.95f, 0.25f, 0.25f, 1.0f);
            py += 32.0f;

            // XPBD Physics Body info
            zero_ui_draw_text(&ui, ins_x + 10.0f, py, "--- XPBD RIGID BODY ---", vec4(0.55f, 0.65f, 0.85f, 1.0f), 0.9f);
            py += 20.0f;

            if (sel->rigid_body_id > 0 && scene->physics) {
                uint32_t b_idx = (uint32_t)(sel->rigid_body_id - 1);
                if (b_idx < (uint32_t)scene->physics->body_count) {
                    ZeroRigidBody *b = &scene->physics->bodies[b_idx];
                    char rb_str[64];
                    snprintf(rb_str, sizeof(rb_str), "Mass: %.1f kg | Restitution: %.2f", b->mass, b->restitution);
                    zero_ui_draw_text(&ui, ins_x + 10.0f, py, rb_str, vec4(0.85f, 0.88f, 0.92f, 1.0f), 0.85f);
                    py += 24.0f;

                    // Sliders
                    float m = b->mass;
                    if (zero_ui_slider(&ui, ins_x + 10.0f, py, ins_w - 20.0f, 16.0f, "Mass", &m, 0.5f, 30.0f)) {
                        b->mass = m;
                        b->inv_mass = 1.0f / m;
                    }
                    py += 28.0f;

                    float rest = b->restitution;
                    if (zero_ui_slider(&ui, ins_x + 10.0f, py, ins_w - 20.0f, 16.0f, "Restitution", &rest, 0.0f, 1.0f)) {
                        b->restitution = rest;
                    }
                    py += 28.0f;
                }
            } else {
                zero_ui_draw_text(&ui, ins_x + 10.0f, py, "No active rigid body collider.", vec4(0.6f, 0.6f, 0.7f, 1.0f), 0.85f);
                py += 24.0f;
            }

            // Entity Actions
            if (zero_ui_button(&ui, ins_x + 10.0f, py, 120.0f, 26.0f, "Delete Entity")) {
                studio_log("Deleted entity.");
                zero_entity_destroy(scene, (uint32_t)selected_entity_id);
                selected_entity_id = -1;
            }
            if (zero_ui_button(&ui, ins_x + 140.0f, py, 120.0f, 26.0f, "Deselect")) {
                selected_entity_id = -1;
            }
        } else {
            zero_ui_draw_text(&ui, ins_x + 12.0f, ins_y + 40.0f, "No entity selected.", vec4(0.7f, 0.75f, 0.85f, 1.0f), 1.0f);
            zero_ui_draw_text(&ui, ins_x + 12.0f, ins_y + 65.0f, "Click an item in the", vec4(0.55f, 0.58f, 0.65f, 1.0f), 0.9f);
            zero_ui_draw_text(&ui, ins_x + 12.0f, ins_y + 85.0f, "Hierarchy panel to", vec4(0.55f, 0.58f, 0.65f, 1.0f), 0.9f);
            zero_ui_draw_text(&ui, ins_x + 12.0f, ins_y + 105.0f, "inspect its properties.", vec4(0.55f, 0.58f, 0.65f, 1.0f), 0.9f);
        }

        // --------------------------------------------------------------------
        // 4. Bottom Panel: Studio Console & Telemetry
        // --------------------------------------------------------------------
        float c_x = 10.0f;
        float c_y = sh - 160.0f;
        float c_w = sw - 20.0f;
        float c_h = 150.0f;
        zero_ui_panel(&ui, c_x, c_y, c_w, c_h, "STUDIO CONSOLE & ENGINE TELEMETRY");

        // Telemetry Stats Left Box
        float stat_y = c_y + 32.0f;
        char stat1[128], stat2[128], stat3[128];
        snprintf(stat1, sizeof(stat1), "FPS: %d  |  Entities: %u / %d  |  Bodies: %d",
                 fps_display, scene->entity_count, ZERO_MAX_SCENE_ENTITIES,
                 scene->physics ? scene->physics->body_count : 0);
        snprintf(stat2, sizeof(stat2), "Camera Pos: (%.1f, %.1f, %.1f) | Yaw: %.1f | Pitch: %.1f",
                 cam.position.x, cam.position.y, cam.position.z,
                 ZERO_RAD2DEG(cam.yaw), ZERO_RAD2DEG(cam.pitch));
        snprintf(stat3, sizeof(stat3), "Simulation State: %s  |  Tick: %llu  |  Time: %.2fs",
                 sim_running ? "RUNNING (XPBD 60Hz)" : "PAUSED",
                 (unsigned long long)scene->tick, scene->time);

        zero_ui_draw_text(&ui, c_x + 12.0f, stat_y, stat1, vec4(0.25f, 0.85f, 0.95f, 1.0f), 0.9f);
        zero_ui_draw_text(&ui, c_x + 12.0f, stat_y + 20.0f, stat2, vec4(0.85f, 0.88f, 0.95f, 1.0f), 0.85f);
        zero_ui_draw_text(&ui, c_x + 12.0f, stat_y + 40.0f, stat3, vec4(0.85f, 0.88f, 0.95f, 1.0f), 0.85f);

        // Event Log Window Right Box
        float log_x = c_x + 480.0f;
        float log_y = c_y + 30.0f;
        zero_ui_draw_text(&ui, log_x, log_y, "[EVENT LOG]", vec4(0.95f, 0.8f, 0.2f, 1.0f), 0.85f);
        log_y += 18.0f;

        for (int l = 0; l < 4; ++l) {
            int idx = (g_log_head - 1 - l + MAX_LOG_LINES) % MAX_LOG_LINES;
            if (g_log_lines[idx][0] != '\0') {
                zero_ui_draw_text(&ui, log_x, log_y, g_log_lines[idx], vec4(0.80f, 0.82f, 0.88f, 1.0f), 0.85f);
                log_y += 18.0f;
            }
        }

        // Render UI Elements to Screen
        zero_ui_end(&ui);

        zero_platform_swap_buffers(&platform);
    }

    zero_ui_shutdown(&ui);
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

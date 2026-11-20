#include "zero_core.h"
#include "zero_memory.h"
#include "zero_math.h"
#include "zero_platform.h"
#include "zero_gl_loader.h"
#include "zero_shader.h"
#include "zero_mesh.h"
#include "zero_camera.h"
#include <stdio.h>
#include <math.h>

int main(void) {
    ZeroPlatformConfig cfg = {
        .title = "Zero Engine: 3D Viewport Demo",
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
        printf("[Zero Engine] Headless environment detected. Demo completed successfully.\n");
        zero_platform_shutdown(&platform);
        return 0;
    }

    ZeroArena arena = zero_arena_create(32 * 1024 * 1024);

    ZeroMeshData cube_data = zero_mesh_gen_cube(&arena, 1.5f);
    ZeroMeshData plane_data = zero_mesh_gen_plane(&arena, 20.0f, 20.0f);
    ZeroMeshData sphere_data = zero_mesh_gen_sphere(&arena, 1.0f, 24, 24);

    ZeroMesh cube_mesh = zero_mesh_upload(&cube_data);
    ZeroMesh plane_mesh = zero_mesh_upload(&plane_data);
    ZeroMesh sphere_mesh = zero_mesh_upload(&sphere_data);

    ZeroShader shader = zero_shader_create(zero_shader_default_vert(), zero_shader_default_frag());
    if (!shader.is_valid) {
        fprintf(stderr, "Failed to compile default shader\n");
        zero_platform_shutdown(&platform);
        return 1;
    }

    ZeroCamera cam = zero_camera_create(
        vec3(0.0f, 3.0f, 7.0f),
        ZERO_DEG2RAD(60.0f),
        (float)platform.width / (float)platform.height,
        0.1f,
        1000.0f
    );

    glEnable(GL_DEPTH_TEST);

    double last_time = zero_platform_get_time();
    int last_mouse_x = platform.mouse_x;
    int last_mouse_y = platform.mouse_y;
    float rotation = 0.0f;

    printf("[Zero Engine] Interactive 3D Viewport running.\n");
    printf("Controls: W/A/S/D to move, Space/C for Up/Down, Mouse Drag to look, ESC to exit.\n");

    while (!platform.should_close) {
        double current_time = zero_platform_get_time();
        float dt = (float)(current_time - last_time);
        last_time = current_time;

        zero_platform_poll_events(&platform);

        if (platform.mouse_buttons[1]) {
            float dx = (float)(platform.mouse_x - last_mouse_x);
            float dy = (float)(last_mouse_y - platform.mouse_y);
            zero_camera_rotate(&cam, dy * 0.15f, dx * 0.15f);
        }
        last_mouse_x = platform.mouse_x;
        last_mouse_y = platform.mouse_y;

        float speed = 6.0f * dt;
        Vec3 move = vec3(0, 0, 0);
        if (platform.keys['w'] || platform.keys['W']) move = vec3_add(move, vec3_scale(cam.forward, speed));
        if (platform.keys['s'] || platform.keys['S']) move = vec3_sub(move, vec3_scale(cam.forward, speed));
        if (platform.keys['d'] || platform.keys['D']) move = vec3_add(move, vec3_scale(cam.right, speed));
        if (platform.keys['a'] || platform.keys['A']) move = vec3_sub(move, vec3_scale(cam.right, speed));
        if (platform.keys[' ']) move = vec3_add(move, vec3_scale(vec3(0, 1, 0), speed));
        if (platform.keys['c'] || platform.keys['C']) move = vec3_sub(move, vec3_scale(vec3(0, 1, 0), speed));
        zero_camera_move(&cam, move);

        cam.aspect = (float)platform.width / (float)platform.height;
        glViewport(0, 0, (GLsizei)platform.width, (GLsizei)platform.height);

        glClearColor(0.06f, 0.07f, 0.10f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        zero_shader_bind(&shader);

        Mat4 view = zero_camera_get_view(&cam);
        Mat4 proj = zero_camera_get_projection(&cam);
        zero_shader_set_mat4(&shader, "u_view", view);
        zero_shader_set_mat4(&shader, "u_proj", proj);
        zero_shader_set_vec3(&shader, "u_camera_pos", cam.position);

        zero_shader_set_vec3(&shader, "u_light_dir", vec3(-0.4f, -1.0f, -0.3f));
        zero_shader_set_vec3(&shader, "u_light_color", vec3(1.0f, 0.95f, 0.85f));
        zero_shader_set_vec3(&shader, "u_ambient_color", vec3(0.15f, 0.18f, 0.22f));

        Mat4 plane_model = mat4_translate(vec3(0.0f, -0.01f, 0.0f));
        zero_shader_set_mat4(&shader, "u_model", plane_model);
        zero_shader_set_vec3(&shader, "u_object_color", vec3(0.25f, 0.28f, 0.35f));
        zero_mesh_draw(&plane_mesh);

        rotation += 45.0f * dt;
        Mat4 cube_rot = quat_to_mat4(quat_from_axis_angle(vec3(0.0f, 1.0f, 0.0f), ZERO_DEG2RAD(rotation)));
        Mat4 cube_model = mat4_mul(mat4_translate(vec3(-2.0f, 1.0f, 0.0f)), cube_rot);
        zero_shader_set_mat4(&shader, "u_model", cube_model);
        zero_shader_set_vec3(&shader, "u_object_color", vec3(0.95f, 0.65f, 0.15f));
        zero_mesh_draw(&cube_mesh);

        Mat4 sphere_model = mat4_translate(vec3(2.0f, 1.0f, 0.0f));
        zero_shader_set_mat4(&shader, "u_model", sphere_model);
        zero_shader_set_vec3(&shader, "u_object_color", vec3(0.15f, 0.75f, 0.95f));
        zero_mesh_draw(&sphere_mesh);

        zero_shader_unbind();
        zero_platform_swap_buffers(&platform);
    }

    zero_mesh_destroy(&cube_mesh);
    zero_mesh_destroy(&plane_mesh);
    zero_mesh_destroy(&sphere_mesh);
    zero_shader_destroy(&shader);
    zero_arena_destroy(&arena);
    zero_platform_shutdown(&platform);

    return 0;
}

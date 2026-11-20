#include "zero_core.h"
#include "zero_memory.h"
#include "zero_math.h"
#include "zero_platform.h"
#include "zero_gl_loader.h"
#include "zero_shader.h"
#include "zero_mesh.h"
#include "zero_camera.h"
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <string.h>

static int g_tests_run = 0;
static int g_tests_passed = 0;

#define TEST_ASSERT(cond, msg) do { \
    g_tests_run++; \
    if (cond) { \
        g_tests_passed++; \
    } else { \
        fprintf(stderr, "[FAIL] Line %d: %s\n", __LINE__, msg); \
        assert(cond); \
    } \
} while(0)

static void test_memory_arena(void) {
    ZeroArena arena = zero_arena_create(1024 * 1024);
    TEST_ASSERT(arena.buffer != NULL, "Arena buffer must not be NULL");
    TEST_ASSERT(arena.capacity >= 1024 * 1024, "Arena capacity must be at least requested size");
    TEST_ASSERT(arena.offset == 0, "Initial offset must be 0");

    void *ptr1 = zero_arena_alloc(&arena, 128, 64);
    TEST_ASSERT(ptr1 != NULL, "Allocation 1 must succeed");
    TEST_ASSERT(((uintptr_t)ptr1 % 64) == 0, "Pointer 1 must be 64-byte aligned");

    void *ptr2 = zero_arena_alloc(&arena, 256, 64);
    TEST_ASSERT(ptr2 != NULL, "Allocation 2 must succeed");
    TEST_ASSERT(((uintptr_t)ptr2 % 64) == 0, "Pointer 2 must be 64-byte aligned");
    TEST_ASSERT((uint8_t *)ptr2 >= (uint8_t *)ptr1 + 128, "Pointer 2 must not overlap Pointer 1");

    void *huge_ptr = zero_arena_alloc(&arena, arena.capacity * 2, 64);
    TEST_ASSERT(huge_ptr == NULL, "Allocation larger than capacity must fail gracefully");

    zero_arena_reset(&arena);
    TEST_ASSERT(arena.offset == 0, "Offset after reset must be 0");

    zero_scratch_init(1024 * 1024);
    ZeroScratch s1 = zero_scratch_get(NULL);
    TEST_ASSERT(s1.arena != NULL, "Scratch arena 1 must be valid");
    void *s1_ptr = zero_arena_alloc(s1.arena, 512, 64);
    TEST_ASSERT(s1_ptr != NULL, "Scratch allocation must succeed");

    ZeroScratch s2 = zero_scratch_get(s1.arena);
    TEST_ASSERT(s2.arena != s1.arena, "Conflicting scratch must return alternate arena");

    zero_scratch_release(s2);
    zero_scratch_release(s1);
    zero_scratch_shutdown();

    zero_arena_destroy(&arena);
    TEST_ASSERT(arena.buffer == NULL, "Destroyed arena buffer must be NULL");
}

static void test_math_vectors_matrices(void) {
    Vec3 a = vec3(1.0f, 2.0f, 3.0f);
    Vec3 b = vec3(4.0f, 5.0f, 6.0f);

    Vec3 add = vec3_add(a, b);
    TEST_ASSERT(add.x == 5.0f && add.y == 7.0f && add.z == 9.0f, "Vec3 addition");

    float dot = vec3_dot(a, b);
    TEST_ASSERT(dot == 32.0f, "Vec3 dot product");

    Vec3 cross = vec3_cross(vec3(1.0f, 0.0f, 0.0f), vec3(0.0f, 1.0f, 0.0f));
    TEST_ASSERT(fabsf(cross.x) < 1e-6f && fabsf(cross.y) < 1e-6f && fabsf(cross.z - 1.0f) < 1e-6f, "Vec3 cross product");

    Vec3 norm = vec3_normalize(vec3(0.0f, 3.0f, 4.0f));
    TEST_ASSERT(fabsf(vec3_length(norm) - 1.0f) < 1e-6f, "Vec3 normalization");

    Mat4 id = mat4_identity();
    Mat4 tr = mat4_translate(vec3(10.0f, 20.0f, 30.0f));
    Mat4 mul = mat4_mul(id, tr);
    TEST_ASSERT(mul.m[12] == 10.0f && mul.m[13] == 20.0f && mul.m[14] == 30.0f, "Mat4 translation multiply");

    Mat4 inv = mat4_inverse(tr);
    Mat4 ident_check = mat4_mul(tr, inv);
    TEST_ASSERT(fabsf(ident_check.m[0] - 1.0f) < 1e-4f, "Mat4 inverse element 0");
    TEST_ASSERT(fabsf(ident_check.m[5] - 1.0f) < 1e-4f, "Mat4 inverse element 5");
    TEST_ASSERT(fabsf(ident_check.m[10] - 1.0f) < 1e-4f, "Mat4 inverse element 10");
    TEST_ASSERT(fabsf(ident_check.m[15] - 1.0f) < 1e-4f, "Mat4 inverse element 15");
    TEST_ASSERT(fabsf(ident_check.m[12]) < 1e-4f, "Mat4 inverse translation x cancel");
    TEST_ASSERT(fabsf(ident_check.m[13]) < 1e-4f, "Mat4 inverse translation y cancel");
    TEST_ASSERT(fabsf(ident_check.m[14]) < 1e-4f, "Mat4 inverse translation z cancel");

    Mat4 proj = mat4_perspective(ZERO_DEG2RAD(60.0f), 16.0f / 9.0f, 0.1f, 1000.0f);
    TEST_ASSERT(proj.m[11] == -1.0f, "Perspective projection W divide factor");
}

static void test_math_dual_quaternions(void) {
    Quat q_rot = quat_from_axis_angle(vec3(0.0f, 1.0f, 0.0f), ZERO_DEG2RAD(90.0f));
    Vec3 trans = vec3(5.0f, 0.0f, 0.0f);

    DualQuat dq = dual_quat_create(q_rot, trans);
    dq = dual_quat_normalize(dq);

    Vec3 pt = vec3(1.0f, 0.0f, 0.0f);
    Vec3 transformed = dual_quat_transform_point(dq, pt);

    TEST_ASSERT(fabsf(transformed.x - 5.0f) < 1e-4f, "DualQuat rotated & translated X");
    TEST_ASSERT(fabsf(transformed.y - 0.0f) < 1e-4f, "DualQuat rotated & translated Y");
    TEST_ASSERT(fabsf(transformed.z - (-1.0f)) < 1e-4f, "DualQuat rotated & translated Z");

    DualQuat dq2 = dual_quat_create(quat_identity(), vec3(0.0f, 0.0f, 0.0f));
    DualQuat blended = dual_quat_blend(dq, dq2, 0.5f);
    TEST_ASSERT(fabsf(quat_dot(blended.real, blended.real) - 1.0f) < 1e-5f, "DualQuat blend normalized");
}

static void test_procedural_meshes(void) {
    ZeroArena arena = zero_arena_create(2 * 1024 * 1024);

    ZeroMeshData cube = zero_mesh_gen_cube(&arena, 2.0f);
    TEST_ASSERT(cube.vertex_count == 24, "Cube vertex count must be 24");
    TEST_ASSERT(cube.index_count == 36, "Cube index count must be 36");

    ZeroMeshData plane = zero_mesh_gen_plane(&arena, 10.0f, 10.0f);
    TEST_ASSERT(plane.vertex_count == 4, "Plane vertex count must be 4");
    TEST_ASSERT(plane.index_count == 6, "Plane index count must be 6");

    ZeroMeshData sphere = zero_mesh_gen_sphere(&arena, 1.0f, 16, 16);
    TEST_ASSERT(sphere.vertex_count == 17 * 17, "Sphere vertex count check");
    TEST_ASSERT(sphere.index_count == 16 * 16 * 6, "Sphere index count check");

    for (uint32_t i = 0; i < sphere.vertex_count; ++i) {
        float len = vec3_length(sphere.vertices[i].normal);
        TEST_ASSERT(fabsf(len - 1.0f) < 1e-3f, "Sphere vertex normals must be normalized");
    }

    zero_arena_destroy(&arena);
}

static void test_camera_system(void) {
    ZeroCamera cam = zero_camera_create(vec3(0.0f, 2.0f, 5.0f), ZERO_DEG2RAD(60.0f), 16.0f / 9.0f, 0.1f, 1000.0f);
    TEST_ASSERT(cam.position.x == 0.0f && cam.position.y == 2.0f && cam.position.z == 5.0f, "Camera position");
    TEST_ASSERT(fabsf(cam.forward.z - (-1.0f)) < 1e-4f, "Initial camera forward must face -Z");

    zero_camera_rotate(&cam, 100.0f, 0.0f);
    TEST_ASSERT(cam.pitch == 89.0f, "Pitch must be clamped to 89 degrees");

    cam.pitch = 0.0f;
    cam.yaw = -90.0f;
    zero_camera_update_vectors(&cam);

    ZeroRay center_ray = zero_camera_screen_to_ray(&cam, 640.0f, 360.0f, 1280.0f, 720.0f);
    TEST_ASSERT(fabsf(center_ray.direction.z - (-1.0f)) < 1e-2f, "Center screen ray must point along camera forward");

    Mat4 view = zero_camera_get_view(&cam);
    TEST_ASSERT(fabsf(view.m[0] - 1.0f) < 1e-4f, "View matrix X basis");
    TEST_ASSERT(fabsf(view.m[5] - 1.0f) < 1e-4f, "View matrix Y basis");
}

static void test_shader_definitions(void) {
    const char *vs = zero_shader_default_vert();
    const char *fs = zero_shader_default_frag();
    TEST_ASSERT(vs != NULL && strlen(vs) > 50, "Default vertex shader must be non-empty");
    TEST_ASSERT(fs != NULL && strlen(fs) > 50, "Default fragment shader must be non-empty");
    TEST_ASSERT(strstr(vs, "u_model") != NULL, "Vertex shader must contain u_model");
    TEST_ASSERT(strstr(fs, "u_light_dir") != NULL, "Fragment shader must contain u_light_dir");
}

static void test_platform_headless(void) {
    ZeroPlatformConfig cfg = {
        .title = "Zero Engine Test",
        .width = 800,
        .height = 600,
        .headless = true
    };
    ZeroPlatformState state;
    bool ok = zero_platform_init(&state, &cfg);
    TEST_ASSERT(ok == true, "Headless platform init must succeed");
    TEST_ASSERT(state.headless == true, "Platform state must indicate headless mode");

    double t0 = zero_platform_get_time();
    TEST_ASSERT(t0 > 0.0, "Monotonic time must be greater than zero");

    zero_platform_poll_events(&state);
    zero_platform_swap_buffers(&state);
    zero_platform_shutdown(&state);
}

int main(int argc, char **argv) {
    bool run_windowed = false;
    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "--windowed") == 0) {
            run_windowed = true;
        }
    }

    printf("=== ZERO ENGINE: PHASE 1 & PHASE 2 VERIFICATION SUITE ===\n");
    test_memory_arena();
    test_math_vectors_matrices();
    test_math_dual_quaternions();
    test_procedural_meshes();
    test_camera_system();
    test_shader_definitions();
    test_platform_headless();

    if (run_windowed) {
        printf("Running windowed platform verification...\n");
        ZeroPlatformConfig cfg = {
            .title = "Zero Engine Phase 2 Test",
            .width = 640,
            .height = 480,
            .headless = false
        };
        ZeroPlatformState state;
        if (zero_platform_init(&state, &cfg)) {
            printf("Window initialized. Display: %p, Context: %p, Headless: %d\n",
                   (void *)state.display, (void *)state.gl_context, state.headless);
            double start = zero_platform_get_time();
            while (!state.should_close && (zero_platform_get_time() - start < 1.0)) {
                zero_platform_poll_events(&state);
                zero_platform_swap_buffers(&state);
            }
            zero_platform_shutdown(&state);
        }
    }

    printf("\n[ALL TESTS PASSED] %d / %d checks verified successfully.\n", g_tests_passed, g_tests_run);
    return 0;
}

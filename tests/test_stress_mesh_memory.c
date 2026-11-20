#define _GNU_SOURCE
#include "zero_core.h"
#include "zero_memory.h"
#include "zero_math.h"
#include "zero_mesh.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>
#include <unistd.h>
#include <sys/wait.h>

static int g_stress_tests_run = 0;
static int g_stress_tests_passed = 0;
static int g_stress_tests_failed = 0;

#define STRESS_CHECK(cond, msg) do { \
    g_stress_tests_run++; \
    if (cond) { \
        g_stress_tests_passed++; \
    } else { \
        g_stress_tests_failed++; \
        fprintf(stderr, "[FAIL] Line %d: %s\n", __LINE__, msg); \
    } \
} while(0)

static void test_sphere_rejections_and_boundaries(void) {
    ZeroArena arena = zero_arena_create(1024 * 1024);

    ZeroMeshData d0 = zero_mesh_gen_sphere(&arena, 1.0f, 0, 0);
    STRESS_CHECK(d0.vertices == NULL && d0.indices == NULL, "Sphere (0,0) must return empty mesh");
    STRESS_CHECK(d0.vertex_count == 0 && d0.index_count == 0, "Sphere (0,0) count must be 0");

    ZeroMeshData d1 = zero_mesh_gen_sphere(&arena, 1.0f, 1, 1);
    STRESS_CHECK(d1.vertices == NULL && d1.indices == NULL, "Sphere (1,1) must return empty mesh");
    STRESS_CHECK(d1.vertex_count == 0 && d1.index_count == 0, "Sphere (1,1) count must be 0");

    ZeroMeshData d2 = zero_mesh_gen_sphere(&arena, 1.0f, 2, 2);
    STRESS_CHECK(d2.vertices == NULL && d2.indices == NULL, "Sphere (2,2) with sectors < 3 must return empty mesh");
    STRESS_CHECK(d2.vertex_count == 0 && d2.index_count == 0, "Sphere (2,2) count must be 0");

    ZeroMeshData d3 = zero_mesh_gen_sphere(&arena, 1.0f, 1, 10);
    STRESS_CHECK(d3.vertices == NULL && d3.indices == NULL, "Sphere (1,10) with rings < 2 must return empty mesh");
    STRESS_CHECK(d3.vertex_count == 0 && d3.index_count == 0, "Sphere (1,10) count must be 0");

    zero_arena_destroy(&arena);
}

static void test_sphere_tessellation_sweep(void) {
    const size_t arena_cap = 16 * 1024 * 1024;
    ZeroArena arena = zero_arena_create(arena_cap);
    STRESS_CHECK(arena.buffer != NULL, "Arena allocation for sphere sweep must succeed");

    struct {
        uint32_t rings;
        uint32_t sectors;
        float radius;
    } configs[] = {
        {2, 3, 1.0f},
        {3, 3, 2.5f},
        {4, 4, 0.5f},
        {8, 8, 10.0f},
        {16, 16, 1.0f},
        {32, 32, 5.0f},
        {64, 64, 1.0f},
        {128, 128, 3.14f},
        {256, 256, 1.0f},
        {2, 256, 0.75f},
        {256, 3, 1.25f},
        {17, 73, 2.0f},
        {128, 4, 3.0f},
        {5, 120, 4.0f}
    };

    uint32_t num_configs = ZERO_ARRAY_COUNT(configs);
    int winding_failures = 0;

    for (uint32_t c = 0; c < num_configs; ++c) {
        zero_arena_reset(&arena);
        uint32_t rings = configs[c].rings;
        uint32_t sectors = configs[c].sectors;
        float radius = configs[c].radius;

        ZeroMeshData sphere = zero_mesh_gen_sphere(&arena, radius, rings, sectors);
        STRESS_CHECK(sphere.vertices != NULL, "Sphere vertices pointer must not be NULL");
        STRESS_CHECK(sphere.indices != NULL, "Sphere indices pointer must not be NULL");

        uint32_t expected_vcount = (rings + 1) * (sectors + 1);
        uint32_t expected_icount = rings * sectors * 6;
        STRESS_CHECK(sphere.vertex_count == expected_vcount, "Sphere vertex count mismatch");
        STRESS_CHECK(sphere.index_count == expected_icount, "Sphere index count mismatch");

        bool indices_valid = true;
        for (uint32_t i = 0; i < sphere.index_count; ++i) {
            if (sphere.indices[i] >= sphere.vertex_count) {
                indices_valid = false;
                break;
            }
        }
        STRESS_CHECK(indices_valid, "All sphere indices within [0, vertex_count-1]");

        bool normals_normalized = true;
        bool positions_valid = true;
        bool uvs_valid = true;
        for (uint32_t v = 0; v < sphere.vertex_count; ++v) {
            ZeroVertex vert = sphere.vertices[v];
            float n_len = vec3_length(vert.normal);
            if (fabsf(n_len - 1.0f) >= 1e-4f) normals_normalized = false;

            float p_len = vec3_length(vert.pos);
            if (fabsf(p_len - radius) >= 1e-3f * radius + 1e-4f) positions_valid = false;

            if (vert.uv.x < -1e-5f || vert.uv.x > 1.0f + 1e-5f ||
                vert.uv.y < -1e-5f || vert.uv.y > 1.0f + 1e-5f) {
                uvs_valid = false;
            }
        }
        STRESS_CHECK(normals_normalized, "All sphere vertex normals normalized");
        STRESS_CHECK(positions_valid, "All sphere vertex positions match radius");
        STRESS_CHECK(uvs_valid, "All sphere UV coordinates in [0, 1]");

        uint32_t num_triangles = sphere.index_count / 3;
        for (uint32_t t = 0; t < num_triangles; ++t) {
            uint32_t i0 = sphere.indices[3 * t + 0];
            uint32_t i1 = sphere.indices[3 * t + 1];
            uint32_t i2 = sphere.indices[3 * t + 2];

            Vec3 p0 = sphere.vertices[i0].pos;
            Vec3 p1 = sphere.vertices[i1].pos;
            Vec3 p2 = sphere.vertices[i2].pos;

            Vec3 e1 = vec3_sub(p1, p0);
            Vec3 e2 = vec3_sub(p2, p0);
            Vec3 fn = vec3_cross(e1, e2);
            float fn_len = vec3_length(fn);

            if (fn_len > 1e-6f) {
                Vec3 center = vec3_scale(vec3_add(p0, vec3_add(p1, p2)), 1.0f / 3.0f);
                float dot = vec3_dot(fn, center);
                if (dot <= 0.0f) {
                    winding_failures++;
                }
            }
        }
    }

    if (winding_failures > 0) {
        fprintf(stderr, "[DEFECT OBSERVED] Sphere triangles have INVERTED winding order: %d triangles face INWARD (CW instead of CCW)\n", winding_failures);
    }
    STRESS_CHECK(winding_failures == 0, "Sphere triangle face normal must point outwards (CCW winding)");

    zero_arena_destroy(&arena);
}

static void test_sphere_exhaustion_resilience(void) {
    ZeroArena tiny_arena = zero_arena_create(256);
    ZeroMeshData sphere = zero_mesh_gen_sphere(&tiny_arena, 1.0f, 256, 256);
    STRESS_CHECK(sphere.vertices == NULL || sphere.indices == NULL,
                 "Sphere generation with insufficient arena memory must fail allocation gracefully");

    if (sphere.vertices == NULL && sphere.vertex_count != 0) {
        fprintf(stderr, "[DEFECT OBSERVED] Allocation failure leaves vertex_count=%u while vertices pointer is NULL\n",
                sphere.vertex_count);
    }
    zero_arena_destroy(&tiny_arena);
}

static void test_cube_winding_and_normals(void) {
    ZeroArena arena = zero_arena_create(1024 * 1024);
    float size = 2.0f;
    float h = size * 0.5f;

    ZeroMeshData cube = zero_mesh_gen_cube(&arena, size);
    STRESS_CHECK(cube.vertex_count == 24, "Cube vertex count must be 24");
    STRESS_CHECK(cube.index_count == 36, "Cube index count must be 36");
    STRESS_CHECK(cube.vertices != NULL && cube.indices != NULL, "Cube pointers must be valid");

    static const Vec3 expected_face_normals[6] = {
        { 0.0f,  0.0f,  1.0f},
        { 0.0f,  0.0f, -1.0f},
        {-1.0f,  0.0f,  0.0f},
        { 1.0f,  0.0f,  0.0f},
        { 0.0f,  1.0f,  0.0f},
        { 0.0f, -1.0f,  0.0f}
    };

    float total_area = 0.0f;
    bool normals_ok = true;
    bool bounds_ok = true;
    bool winding_ok = true;

    for (int face = 0; face < 6; ++face) {
        Vec3 nominal_n = expected_face_normals[face];
        uint32_t v_base = face * 4;
        uint32_t i_base = face * 6;

        for (int v = 0; v < 4; ++v) {
            ZeroVertex vert = cube.vertices[v_base + v];
            if (fabsf(vert.normal.x - nominal_n.x) > 1e-5f ||
                fabsf(vert.normal.y - nominal_n.y) > 1e-5f ||
                fabsf(vert.normal.z - nominal_n.z) > 1e-5f) {
                normals_ok = false;
            }

            if (fabsf(vert.pos.x) > h + 1e-5f ||
                fabsf(vert.pos.y) > h + 1e-5f ||
                fabsf(vert.pos.z) > h + 1e-5f) {
                bounds_ok = false;
            }
        }

        for (int tri = 0; tri < 2; ++tri) {
            uint32_t i0 = cube.indices[i_base + tri * 3 + 0];
            uint32_t i1 = cube.indices[i_base + tri * 3 + 1];
            uint32_t i2 = cube.indices[i_base + tri * 3 + 2];

            Vec3 p0 = cube.vertices[i0].pos;
            Vec3 p1 = cube.vertices[i1].pos;
            Vec3 p2 = cube.vertices[i2].pos;

            Vec3 fn = vec3_cross(vec3_sub(p1, p0), vec3_sub(p2, p0));
            float dot = vec3_dot(fn, nominal_n);
            if (dot <= 0.0f) {
                winding_ok = false;
            }

            float tri_area = 0.5f * vec3_length(fn);
            total_area += tri_area;
        }
    }

    STRESS_CHECK(normals_ok, "All cube vertex normals match expected face normals");
    STRESS_CHECK(bounds_ok, "All cube vertex positions are within bounds");
    STRESS_CHECK(winding_ok, "All cube triangles have CCW winding facing outwards");

    float expected_total_area = 6.0f * (size * size);
    STRESS_CHECK(fabsf(total_area - expected_total_area) < 1e-3f, "Total cube surface area matches 6 * size^2");

    zero_arena_destroy(&arena);
}

static void test_plane_winding_and_normals(void) {
    ZeroArena arena = zero_arena_create(1024 * 1024);
    float width = 14.0f;
    float depth = 8.0f;

    ZeroMeshData plane = zero_mesh_gen_plane(&arena, width, depth);
    STRESS_CHECK(plane.vertex_count == 4, "Plane vertex count must be 4");
    STRESS_CHECK(plane.index_count == 6, "Plane index count must be 6");
    STRESS_CHECK(plane.vertices != NULL && plane.indices != NULL, "Plane pointers must be valid");

    Vec3 up = vec3(0.0f, 1.0f, 0.0f);
    bool normals_ok = true;
    for (uint32_t v = 0; v < plane.vertex_count; ++v) {
        ZeroVertex vert = plane.vertices[v];
        if (fabsf(vert.normal.x) > 1e-5f ||
            fabsf(vert.normal.y - 1.0f) > 1e-5f ||
            fabsf(vert.normal.z) > 1e-5f ||
            fabsf(vert.pos.y) > 1e-5f) {
            normals_ok = false;
        }
    }
    STRESS_CHECK(normals_ok, "Plane vertex normals point +Y and y=0");

    float total_plane_area = 0.0f;
    bool winding_ok = true;
    for (int tri = 0; tri < 2; ++tri) {
        uint32_t i0 = plane.indices[tri * 3 + 0];
        uint32_t i1 = plane.indices[tri * 3 + 1];
        uint32_t i2 = plane.indices[tri * 3 + 2];

        Vec3 p0 = plane.vertices[i0].pos;
        Vec3 p1 = plane.vertices[i1].pos;
        Vec3 p2 = plane.vertices[i2].pos;

        Vec3 fn = vec3_cross(vec3_sub(p1, p0), vec3_sub(p2, p0));
        float dot = vec3_dot(fn, up);
        if (dot <= 0.0f) {
            winding_ok = false;
        }

        total_plane_area += 0.5f * vec3_length(fn);
    }
    STRESS_CHECK(winding_ok, "Plane triangles have CCW winding with normal pointing +Y");

    float expected_plane_area = width * depth;
    STRESS_CHECK(fabsf(total_plane_area - expected_plane_area) < 1e-3f, "Total plane surface area matches width * depth");

    zero_arena_destroy(&arena);
}

static void test_scratch_arena_reentrancy_and_stress(void) {
    zero_scratch_init(4 * 1024 * 1024);

    ZeroScratch s1 = zero_scratch_get(NULL);
    STRESS_CHECK(s1.arena != NULL, "Primary scratch arena must be non-NULL");
    size_t base_offset1 = s1.offset;

    ZeroScratch s2 = zero_scratch_get(s1.arena);
    STRESS_CHECK(s2.arena != NULL, "Secondary scratch arena must be non-NULL");
    STRESS_CHECK(s2.arena != s1.arena, "Conflicting scratch arena must be distinct");
    size_t base_offset2 = s2.offset;

    ZeroScratch s3 = zero_scratch_get(s2.arena);
    STRESS_CHECK(s3.arena == s1.arena, "Alternating scratch must return primary arena");
    STRESS_CHECK(s3.offset == s1.arena->offset, "Nested scratch offset must match parent arena current offset");

    zero_scratch_release(s3);
    zero_scratch_release(s2);
    zero_scratch_release(s1);

    for (int iter = 0; iter < 100000; ++iter) {
        ZeroArena *conflict = (iter % 2 == 0) ? NULL : s1.arena;
        ZeroScratch scratch = zero_scratch_get(conflict);
        size_t alloc_size = ((size_t)iter % 1024) + 1;
        size_t align = ((iter % 4) == 0) ? 64 : (((iter % 4) == 1) ? 32 : 16);

        uint32_t *data = (uint32_t *)zero_arena_alloc(scratch.arena, alloc_size, align);
        if (!data || ((uintptr_t)data % align) != 0) {
            STRESS_CHECK(false, "Stress loop allocation failure");
            zero_scratch_release(scratch);
            break;
        }

        data[0] = 0xDEADBEEF;
        data[(alloc_size / sizeof(uint32_t)) > 0 ? (alloc_size / sizeof(uint32_t)) - 1 : 0] = 0xCAFEBABE;

        zero_scratch_release(scratch);
    }
    STRESS_CHECK(true, "Completed 100,000 scratch allocation iterations");

    STRESS_CHECK(s1.arena->offset == base_offset1, "Primary scratch arena must not leak memory after 100k loop");
    STRESS_CHECK(s2.arena->offset == base_offset2, "Secondary scratch arena must not leak memory after 100k loop");

    bool nested_ok = true;
    for (int iter = 0; iter < 10000; ++iter) {
        ZeroScratch l1 = zero_scratch_get(NULL);
        uint8_t *b1 = (uint8_t *)zero_arena_alloc(l1.arena, 128, 64);
        b1[0] = 1;

        ZeroScratch l2 = zero_scratch_get(l1.arena);
        uint8_t *b2 = (uint8_t *)zero_arena_alloc(l2.arena, 256, 64);
        b2[0] = 2;

        ZeroScratch l3 = zero_scratch_get(l2.arena);
        uint8_t *b3 = (uint8_t *)zero_arena_alloc(l3.arena, 512, 64);
        b3[0] = 3;

        ZeroScratch l4 = zero_scratch_get(l3.arena);
        uint8_t *b4 = (uint8_t *)zero_arena_alloc(l4.arena, 1024, 64);
        b4[0] = 4;

        if (b1[0] != 1 || b2[0] != 2 || b3[0] != 3 || b4[0] != 4) {
            nested_ok = false;
        }

        zero_scratch_release(l4);
        zero_scratch_release(l3);
        zero_scratch_release(l2);
        zero_scratch_release(l1);
    }
    STRESS_CHECK(nested_ok, "Nested scratch allocation payloads remained intact across 10k iterations");

    STRESS_CHECK(s1.arena->offset == base_offset1, "Primary scratch arena offset clean after 10k nested runs");
    STRESS_CHECK(s2.arena->offset == base_offset2, "Secondary scratch arena offset clean after 10k nested runs");

    ZeroScratch s_over = zero_scratch_get(NULL);
    void *huge = zero_arena_alloc(s_over.arena, 10 * 1024 * 1024, 64);
    STRESS_CHECK(huge == NULL, "Oversized allocation in scratch arena must return NULL");
    zero_scratch_release(s_over);

    size_t alignments[] = {1, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024, 2048, 4096};
    ZeroScratch s_align = zero_scratch_get(NULL);
    bool align_ok = true;
    for (size_t a = 0; a < ZERO_ARRAY_COUNT(alignments); ++a) {
        size_t align = alignments[a];
        void *p = zero_arena_alloc(s_align.arena, 128, align);
        if (!p || ((uintptr_t)p % align) != 0) {
            align_ok = false;
        }
    }
    STRESS_CHECK(align_ok, "Power-of-two alignment allocations (1 to 4096 bytes) all succeeded and aligned");
    zero_scratch_release(s_align);

    zero_scratch_shutdown();
}

static void test_arena_integer_overflow_vulnerability(void) {
    pid_t pid = fork();
    if (pid == 0) {
        ZeroArena arena = zero_arena_create(1024 * 1024);
        (void)zero_arena_alloc(&arena, 64, 64);
        void *p = zero_arena_alloc(&arena, (size_t)-1, 64);
        if (p == NULL) {
            _exit(0);
        }
        _exit(1);
    } else if (pid > 0) {
        int status = 0;
        waitpid(pid, &status, 0);
        if (WIFSIGNALED(status) && (WTERMSIG(status) == SIGSEGV || WTERMSIG(status) == SIGBUS)) {
            fprintf(stderr, "[DEFECT OBSERVED] zero_arena_alloc crashes with SIGSEGV on integer overflow size ((size_t)-1)\n");
            STRESS_CHECK(false, "zero_arena_alloc must guard against integer overflow on size");
        } else if (WIFEXITED(status) && WEXITSTATUS(status) == 0) {
            STRESS_CHECK(true, "zero_arena_alloc handles (size_t)-1 safely");
        } else {
            STRESS_CHECK(false, "zero_arena_alloc unexpected exit on integer overflow test");
        }
    }
}

int main(void) {
    printf("=== ZERO ENGINE: CHALLENGER M1-2 MESH & MEMORY STRESS HARNESS ===\n");

    test_sphere_rejections_and_boundaries();
    printf("[CHECK] Sphere rejection & minimum boundaries evaluated.\n");

    test_sphere_tessellation_sweep();
    printf("[CHECK] Sphere extreme tessellation (2x3 up to 256x256), normals, UVs & winding evaluated.\n");

    test_sphere_exhaustion_resilience();
    printf("[CHECK] Sphere arena exhaustion resilience evaluated.\n");

    test_cube_winding_and_normals();
    printf("[CHECK] Cube 24v/36i winding order, surface area & normals evaluated.\n");

    test_plane_winding_and_normals();
    printf("[CHECK] Plane 4v/6i winding order, surface area & normals evaluated.\n");

    test_scratch_arena_reentrancy_and_stress();
    printf("[CHECK] Scratch arena re-entrancy, conflict handling & 100k-iteration zero-leak evaluated.\n");

    test_arena_integer_overflow_vulnerability();
    printf("[CHECK] Arena integer overflow on extreme size evaluated.\n");

    printf("\n=== STRESS TEST SUMMARY ===\n");
    printf("Total Checks Run: %d\n", g_stress_tests_run);
    printf("Passed: %d\n", g_stress_tests_passed);
    printf("Failed: %d\n", g_stress_tests_failed);

    if (g_stress_tests_failed > 0) {
        printf("VERDICT: REQUEST_CHANGES (Defects confirmed empirically)\n");
        return 1;
    }

    printf("VERDICT: APPROVE (All empirical stress tests passed)\n");
    return 0;
}

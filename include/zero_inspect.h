#ifndef ZERO_INSPECT_H
#define ZERO_INSPECT_H

#include "zero_core.h"
#include "zero_math.h"
#include "zero_scene.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <assert.h>

// Frustum plane indexing
typedef enum ZeroFrustumPlaneIndex {
    ZERO_FRUSTUM_PLANE_LEFT = 0,
    ZERO_FRUSTUM_PLANE_RIGHT = 1,
    ZERO_FRUSTUM_PLANE_BOTTOM = 2,
    ZERO_FRUSTUM_PLANE_TOP = 3,
    ZERO_FRUSTUM_PLANE_NEAR = 4,
    ZERO_FRUSTUM_PLANE_FAR = 5
} ZeroFrustumPlaneIndex;

// Normalized 3D frustum plane equation: dot(normal, p) + distance >= 0
typedef struct ZeroFrustumPlane {
    Vec3 normal;
    float distance;
} ZeroFrustumPlane;

typedef struct ZeroFrustum {
    ZeroFrustumPlane planes[6];
} ZeroFrustum;

// Visible entity metadata projected for AI perception
typedef struct ZeroEntityVisibility {
    const ZeroEntity *entity;
    float distance;
    Vec2 screen_coord;
    Vec2 screen_bbox_min;
    Vec2 screen_bbox_max;
} ZeroEntityVisibility;

// 1. Frustum extraction from View-Projection matrix (Gribb-Hartmann 6 planes normalized)
ZeroFrustum zero_frustum_from_matrix(Mat4 vp);

// 2. Entity world AABB frustum culling (true = visible/intersecting, false = fully outside)
bool zero_frustum_contains_aabb(const ZeroFrustum *frustum, ZeroAABB aabb);

// 3. 3D point projection to screen space [0, 1] x [0, 1] with near-plane safety
bool zero_project_to_screen(Vec3 world_pos, Mat4 vp, Vec2 *out_screen, float *out_depth);

// 4. Screen-space bounding box calculation from 8 AABB world corners with near-plane clipping
bool zero_project_aabb_to_screen(ZeroAABB aabb, Mat4 vp, Vec2 *out_min, Vec2 *out_max);

// 5. Stable Euclidean distance sorting of visible entities
void zero_inspect_sort_visible(ZeroEntityVisibility *entities, int count);

// 6. JSON serialization matching exact ARCHITECTURE.md §6.2 schema
// Adheres strictly to POSIX snprintf semantics: safe truncation, '\0' termination, returns total required chars
int zero_engine_inspect_scene(const ZeroScene *scene, char *buffer, size_t size);

// Automated gameplay assertion harness macro dumping JSON telemetry on failure
#ifndef ZERO_ASSERT_SCENE
#define ZERO_ASSERT_SCENE(cond, scene, msg) do { \
    bool _zero_assert_cond = (bool)(cond); \
    if (!_zero_assert_cond) { \
        fprintf(stderr, "\n=======================================================\n"); \
        fprintf(stderr, "[ZERO ASSERTION FAILED] %s\n", (msg)); \
        fprintf(stderr, "Location: %s:%d\n", __FILE__, __LINE__); \
        if (scene) { \
            char _inspect_buf[4096]; \
            zero_engine_inspect_scene((scene), _inspect_buf, sizeof(_inspect_buf)); \
            fprintf(stderr, "Scene State Dump:\n%s\n", _inspect_buf); \
        } else { \
            fprintf(stderr, "Scene State Dump: <Scene pointer is NULL>\n"); \
        } \
        fprintf(stderr, "=======================================================\n"); \
        fflush(stderr); \
        assert(_zero_assert_cond); \
    } \
} while(0)
#endif

#endif // ZERO_INSPECT_H

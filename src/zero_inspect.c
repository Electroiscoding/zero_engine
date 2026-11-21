#include "zero_inspect.h"
#include <stdio.h>
#include <stdarg.h>
#include <math.h>
#include <string.h>

// -----------------------------------------------------------------------------
// Safe JSON Buffer Accumulator (POSIX snprintf semantics)
// -----------------------------------------------------------------------------
typedef struct ZeroJsonBuffer {
    char *data;
    size_t capacity;
    size_t count; // Total characters that would have been written
} ZeroJsonBuffer;

static void json_buf_init(ZeroJsonBuffer *b, char *buffer, size_t size) {
    b->data = buffer;
    b->capacity = size;
    b->count = 0;
    if (b->data && b->capacity > 0) {
        b->data[0] = '\0';
    }
}

static void json_buf_printf(ZeroJsonBuffer *b, const char *fmt, ...) {
    va_list ap;
    va_start(ap, fmt);

    size_t remaining = 0;
    char *dst = NULL;
    if (b->data && b->capacity > 0) {
        if (b->count < b->capacity) {
            dst = b->data + b->count;
            remaining = b->capacity - b->count;
        }
    }

    va_list ap_copy;
    va_copy(ap_copy, ap);
    int written = vsnprintf(dst, remaining, fmt, ap_copy);
    va_end(ap_copy);

    if (written > 0) {
        b->count += (size_t)written;
    }
    va_end(ap);
}

static void json_buf_finish(ZeroJsonBuffer *b) {
    if (b->data && b->capacity > 0) {
        if (b->count < b->capacity) {
            b->data[b->count] = '\0';
        } else {
            b->data[b->capacity - 1] = '\0';
        }
    }
}

// -----------------------------------------------------------------------------
// 1. Frustum Extraction from VP Matrix (Gribb-Hartmann Method)
// -----------------------------------------------------------------------------
ZeroFrustum zero_frustum_from_matrix(Mat4 vp) {
    ZeroFrustum f;

    // Mat4 column-major: row r, col c -> vp.m[c * 4 + r]
    // Row 0: m[0], m[4], m[8],  m[12]
    // Row 1: m[1], m[5], m[9],  m[13]
    // Row 2: m[2], m[6], m[10], m[14]
    // Row 3: m[3], m[7], m[11], m[15]

    float a[6] = {
        vp.m[3] + vp.m[0],  // Left
        vp.m[3] - vp.m[0],  // Right
        vp.m[3] + vp.m[1],  // Bottom
        vp.m[3] - vp.m[1],  // Top
        vp.m[3] + vp.m[2],  // Near
        vp.m[3] - vp.m[2]   // Far
    };
    float b[6] = {
        vp.m[7] + vp.m[4],
        vp.m[7] - vp.m[4],
        vp.m[7] + vp.m[5],
        vp.m[7] - vp.m[5],
        vp.m[7] + vp.m[6],
        vp.m[7] - vp.m[6]
    };
    float c[6] = {
        vp.m[11] + vp.m[8],
        vp.m[11] - vp.m[8],
        vp.m[11] + vp.m[9],
        vp.m[11] - vp.m[9],
        vp.m[11] + vp.m[10],
        vp.m[11] - vp.m[10]
    };
    float d[6] = {
        vp.m[15] + vp.m[12],
        vp.m[15] - vp.m[12],
        vp.m[15] + vp.m[13],
        vp.m[15] - vp.m[13],
        vp.m[15] + vp.m[14],
        vp.m[15] - vp.m[14]
    };

    for (int i = 0; i < 6; ++i) {
        float len = sqrtf(a[i] * a[i] + b[i] * b[i] + c[i] * c[i]);
        if (len > 1e-8f) {
            float inv = 1.0f / len;
            f.planes[i].normal = vec3(a[i] * inv, b[i] * inv, c[i] * inv);
            f.planes[i].distance = d[i] * inv;
        } else {
            f.planes[i].normal = vec3(0.0f, 0.0f, 0.0f);
            f.planes[i].distance = 0.0f;
        }
    }

    return f;
}

// -----------------------------------------------------------------------------
// 2. Frustum vs AABB Culling (p-vertex test)
// -----------------------------------------------------------------------------
bool zero_frustum_contains_aabb(const ZeroFrustum *frustum, ZeroAABB aabb) {
    if (!frustum) return true;

    // Sanitize min/max if inverted
    Vec3 box_min = aabb.min;
    Vec3 box_max = aabb.max;
    if (box_min.x > box_max.x) { float t = box_min.x; box_min.x = box_max.x; box_max.x = t; }
    if (box_min.y > box_max.y) { float t = box_min.y; box_min.y = box_max.y; box_max.y = t; }
    if (box_min.z > box_max.z) { float t = box_min.z; box_min.z = box_max.z; box_max.z = t; }

    for (int i = 0; i < 6; ++i) {
        const ZeroFrustumPlane *pl = &frustum->planes[i];

        // p-vertex: furthest corner in normal direction
        Vec3 p;
        p.x = (pl->normal.x > 0.0f) ? box_max.x : box_min.x;
        p.y = (pl->normal.y > 0.0f) ? box_max.y : box_min.y;
        p.z = (pl->normal.z > 0.0f) ? box_max.z : box_min.z;

        float dist = pl->normal.x * p.x + pl->normal.y * p.y + pl->normal.z * p.z + pl->distance;
        if (dist < 0.0f) {
            return false; // Entire box is outside this plane
        }
    }
    return true;
}

// -----------------------------------------------------------------------------
// 3. 3D Point Screen Projection with Near-Plane Safety
// -----------------------------------------------------------------------------
bool zero_project_to_screen(Vec3 world_pos, Mat4 vp, Vec2 *out_screen, float *out_depth) {
    float xc = vp.m[0] * world_pos.x + vp.m[4] * world_pos.y + vp.m[8]  * world_pos.z + vp.m[12];
    float yc = vp.m[1] * world_pos.x + vp.m[5] * world_pos.y + vp.m[9]  * world_pos.z + vp.m[13];
    float zc = vp.m[2] * world_pos.x + vp.m[6] * world_pos.y + vp.m[10] * world_pos.z + vp.m[14];
    float wc = vp.m[3] * world_pos.x + vp.m[7] * world_pos.y + vp.m[11] * world_pos.z + vp.m[15];

    (void)zc;

    // Safety: behind or on camera near plane
    if (wc <= 1e-4f) {
        if (out_depth) *out_depth = -1.0f;
        if (out_screen) *out_screen = vec2(-1.0f, -1.0f);
        return false;
    }

    float inv_w = 1.0f / wc;
    float ndc_x = xc * inv_w;
    float ndc_y = yc * inv_w;

    // Map NDC [-1, 1] to screen [0, 1] with top-left origin
    if (out_screen) {
        out_screen->x = (ndc_x + 1.0f) * 0.5f;
        out_screen->y = (1.0f - ndc_y) * 0.5f;
    }
    if (out_depth) {
        *out_depth = wc; // Camera view depth
    }
    return true;
}

// -----------------------------------------------------------------------------
// 4. Screen-Space AABB Projection with 12-Edge Near Plane Clipping
// -----------------------------------------------------------------------------
bool zero_project_aabb_to_screen(ZeroAABB aabb, Mat4 vp, Vec2 *out_min, Vec2 *out_max) {
    Vec3 corners[8] = {
        {aabb.min.x, aabb.min.y, aabb.min.z},
        {aabb.max.x, aabb.min.y, aabb.min.z},
        {aabb.min.x, aabb.max.y, aabb.min.z},
        {aabb.max.x, aabb.max.y, aabb.min.z},
        {aabb.min.x, aabb.min.y, aabb.max.z},
        {aabb.max.x, aabb.min.y, aabb.max.z},
        {aabb.min.x, aabb.max.y, aabb.max.z},
        {aabb.max.x, aabb.max.y, aabb.max.z}
    };

    Vec4 clip[8];
    int in_front_count = 0;
    const float near_eps = 1e-4f;

    for (int i = 0; i < 8; ++i) {
        clip[i].x = vp.m[0]*corners[i].x + vp.m[4]*corners[i].y + vp.m[8]*corners[i].z + vp.m[12];
        clip[i].y = vp.m[1]*corners[i].x + vp.m[5]*corners[i].y + vp.m[9]*corners[i].z + vp.m[13];
        clip[i].z = vp.m[2]*corners[i].x + vp.m[6]*corners[i].y + vp.m[10]*corners[i].z + vp.m[14];
        clip[i].w = vp.m[3]*corners[i].x + vp.m[7]*corners[i].y + vp.m[11]*corners[i].z + vp.m[15];
        if (clip[i].w > near_eps) {
            in_front_count++;
        }
    }

    // All corners behind near plane => culled from screen
    if (in_front_count == 0) {
        if (out_min) *out_min = vec2(-1.0f, -1.0f);
        if (out_max) *out_max = vec2(-1.0f, -1.0f);
        return false;
    }

    float min_u = 1e9f, min_v = 1e9f;
    float max_u = -1e9f, max_v = -1e9f;

    // Project valid front corners
    for (int i = 0; i < 8; ++i) {
        if (clip[i].w > near_eps) {
            float inv_w = 1.0f / clip[i].w;
            float u = (clip[i].x * inv_w + 1.0f) * 0.5f;
            float v = (1.0f - clip[i].y * inv_w) * 0.5f;
            if (u < min_u) min_u = u;
            if (u > max_u) max_u = u;
            if (v < min_v) min_v = v;
            if (v > max_v) max_v = v;
        }
    }

    // If straddling the near plane, clip all 12 cube edges against w = near_eps
    if (in_front_count < 8) {
        static const int edges[12][2] = {
            {0,1}, {2,3}, {4,5}, {6,7}, // X edges
            {0,2}, {1,3}, {4,6}, {5,7}, // Y edges
            {0,4}, {1,5}, {2,6}, {3,7}  // Z edges
        };

        for (int e = 0; e < 12; ++e) {
            int i0 = edges[e][0];
            int i1 = edges[e][1];
            bool f0 = (clip[i0].w > near_eps);
            bool f1 = (clip[i1].w > near_eps);
            if (f0 != f1) {
                float t = (near_eps - clip[i0].w) / (clip[i1].w - clip[i0].w);
                float x = clip[i0].x + t * (clip[i1].x - clip[i0].x);
                float y = clip[i0].y + t * (clip[i1].y - clip[i0].y);
                float u = (x / near_eps + 1.0f) * 0.5f;
                float v = (1.0f - y / near_eps) * 0.5f;
                if (u < min_u) min_u = u;
                if (u > max_u) max_u = u;
                if (v < min_v) min_v = v;
                if (v > max_v) max_v = v;
            }
        }
    }

    // Clamp bounds to visible screen viewport [0, 1]^2
    if (min_u < 0.0f) min_u = 0.0f;
    if (max_u > 1.0f) max_u = 1.0f;
    if (min_v < 0.0f) min_v = 0.0f;
    if (max_v > 1.0f) max_v = 1.0f;

    if (out_min) { out_min->x = min_u; out_min->y = min_v; }
    if (out_max) { out_max->x = max_u; out_max->y = max_v; }
    return true;
}

// -----------------------------------------------------------------------------
// 5. Stable Euclidean Distance Sorting
// -----------------------------------------------------------------------------
void zero_inspect_sort_visible(ZeroEntityVisibility *entities, int count) {
    if (!entities || count <= 1) return;

    for (int i = 1; i < count; ++i) {
        ZeroEntityVisibility key = entities[i];
        int j = i - 1;
        while (j >= 0) {
            bool swap_needed = false;
            if (entities[j].distance > key.distance + 1e-4f) {
                swap_needed = true;
            } else if (fabsf(entities[j].distance - key.distance) <= 1e-4f) {
                // Deterministic tie-breaker on unique entity ID
                if (entities[j].entity->id > key.entity->id) {
                    swap_needed = true;
                }
            }

            if (!swap_needed) break;
            entities[j + 1] = entities[j];
            j--;
        }
        entities[j + 1] = key;
    }
}

// -----------------------------------------------------------------------------
// 6. JSON Serialization for zero_engine_inspect_scene
// -----------------------------------------------------------------------------
int zero_engine_inspect_scene(const ZeroScene *scene, char *buffer, size_t size) {
    if (!scene) return 0;

    ZeroJsonBuffer b;
    json_buf_init(&b, buffer, size);

    // Compute Camera VP Matrix
    Mat4 view = zero_camera_get_view(&scene->camera);
    Mat4 proj = zero_camera_get_projection(&scene->camera);
    Mat4 vp   = mat4_mul(proj, view);
    ZeroFrustum frustum = zero_frustum_from_matrix(vp);

    // Collect Visible Entities
    ZeroEntityVisibility visible[ZERO_MAX_SCENE_ENTITIES];
    int visible_count = 0;

    for (uint32_t i = 0; i < ZERO_MAX_SCENE_ENTITIES; ++i) {
        const ZeroEntity *e = &scene->entities[i];
        if (!e->is_active || !e->is_visible) continue;
        if (e->type == ZERO_ENTITY_TYPE_CAMERA || e->type == ZERO_ENTITY_TYPE_LIGHT_DIRECTIONAL) continue;

        if (zero_frustum_contains_aabb(&frustum, e->world_aabb)) {
            ZeroEntityVisibility vis;
            vis.entity = e;
            vis.distance = vec3_length(vec3_sub(e->world_pos, scene->camera.position));
            float depth = 0.0f;
            zero_project_to_screen(e->world_pos, vp, &vis.screen_coord, &depth);
            zero_project_aabb_to_screen(e->world_aabb, vp, &vis.screen_bbox_min, &vis.screen_bbox_max);
            visible[visible_count++] = vis;
        }
    }

    // Sort Visible Entities deterministically
    zero_inspect_sort_visible(visible, visible_count);

    // Serialize JSON
    json_buf_printf(&b, "{\n");
    json_buf_printf(&b, "  \"tick\": %llu,\n", (unsigned long long)scene->tick);

    // Camera object matching ARCHITECTURE.md §6.2 + camera frustum state
    json_buf_printf(&b, "  \"camera\": {\n");
    json_buf_printf(&b, "    \"pos\": [%.2f, %.2f, %.2f],\n", scene->camera.position.x, scene->camera.position.y, scene->camera.position.z);
    json_buf_printf(&b, "    \"forward\": [%.2f, %.2f, %.2f],\n", scene->camera.forward.x, scene->camera.forward.y, scene->camera.forward.z);
    json_buf_printf(&b, "    \"up\": [%.2f, %.2f, %.2f],\n", scene->camera.up.x, scene->camera.up.y, scene->camera.up.z);
    json_buf_printf(&b, "    \"fov\": %.3f,\n", scene->camera.fov);
    json_buf_printf(&b, "    \"near_z\": %.2f,\n", scene->camera.near_z);
    json_buf_printf(&b, "    \"far_z\": %.2f,\n", scene->camera.far_z);
    json_buf_printf(&b, "    \"aspect\": %.3f\n", scene->camera.aspect);
    json_buf_printf(&b, "  },\n");

    // Visible Entities array
    if (visible_count > 0) {
        json_buf_printf(&b, "  \"visible_entities\": [\n");
        for (int i = 0; i < visible_count; ++i) {
            const ZeroEntityVisibility *v = &visible[i];
            const ZeroEntity *e = v->entity;
            const char *id_str = (e->name[0] != '\0') ? e->name : "entity";
            const char *state_str = (e->state[0] != '\0') ? e->state : "grounded_idle";

            json_buf_printf(&b, "    {\n");
            json_buf_printf(&b, "      \"id\": \"%s\",\n", id_str);
            json_buf_printf(&b, "      \"distance\": %.2f,\n", v->distance);
            json_buf_printf(&b, "      \"screen_coord\": [%.2f, %.2f],\n", v->screen_coord.x, v->screen_coord.y);
            json_buf_printf(&b, "      \"screen_bbox\": [%.2f, %.2f, %.2f, %.2f],\n",
                            v->screen_bbox_min.x, v->screen_bbox_min.y, v->screen_bbox_max.x, v->screen_bbox_max.y);
            json_buf_printf(&b, "      \"state\": \"%s\",\n", state_str);
            json_buf_printf(&b, "      \"velocity\": [%.2f, %.2f, %.2f],\n", e->velocity.x, e->velocity.y, e->velocity.z);
            json_buf_printf(&b, "      \"aabb_min\": [%.2f, %.2f, %.2f],\n", e->world_aabb.min.x, e->world_aabb.min.y, e->world_aabb.min.z);
            json_buf_printf(&b, "      \"aabb_max\": [%.2f, %.2f, %.2f]\n", e->world_aabb.max.x, e->world_aabb.max.y, e->world_aabb.max.z);
            json_buf_printf(&b, "    }%s\n", (i + 1 < visible_count) ? "," : "");
        }
        json_buf_printf(&b, "  ],\n");
    } else {
        json_buf_printf(&b, "  \"visible_entities\": [],\n");
    }

    // Active Contacts array from attached physics world
    int contact_count = 0;
    if (scene->physics && scene->physics->manifold_count > 0) {
        for (int m = 0; m < scene->physics->manifold_count; ++m) {
            if (scene->physics->manifolds[m].point_count > 0) {
                contact_count++;
            }
        }
    }

    if (contact_count > 0 && scene->physics) {
        json_buf_printf(&b, "  \"active_contacts\": [\n");
        int emitted = 0;
        for (int m = 0; m < scene->physics->manifold_count; ++m) {
            const ZeroManifold *man = &scene->physics->manifolds[m];
            if (man->point_count <= 0) continue;

            // Resolve entity names for body_a and body_b
            char name_a[64];
            char name_b[64];
            snprintf(name_a, sizeof(name_a), "body_%u", man->body_a_id);
            snprintf(name_b, sizeof(name_b), "body_%u", man->body_b_id);

            for (uint32_t e = 0; e < ZERO_MAX_SCENE_ENTITIES; ++e) {
                if (scene->entities[e].is_active && scene->entities[e].rigid_body_id == (int32_t)man->body_a_id) {
                    if (scene->entities[e].name[0] != '\0') {
                        snprintf(name_a, sizeof(name_a), "%s", scene->entities[e].name);
                    }
                }
                if (scene->entities[e].is_active && scene->entities[e].rigid_body_id == (int32_t)man->body_b_id) {
                    if (scene->entities[e].name[0] != '\0') {
                        snprintf(name_b, sizeof(name_b), "%s", scene->entities[e].name);
                    }
                }
            }

            const ZeroContactPoint *cp = &man->points[0];
            emitted++;
            json_buf_printf(&b, "    {\"entity_a\": \"%s\", \"entity_b\": \"%s\", \"normal\": [%.2f, %.2f, %.2f], \"depth\": %.3f}%s\n",
                            name_a, name_b, cp->normal.x, cp->normal.y, cp->normal.z, cp->penetration,
                            (emitted < contact_count) ? "," : "");
        }
        json_buf_printf(&b, "  ]\n");
    } else {
        json_buf_printf(&b, "  \"active_contacts\": []\n");
    }

    json_buf_printf(&b, "}\n");
    json_buf_finish(&b);

    return (int)b.count;
}

#include "zero_scene.h"
#include <string.h>
#include <stdio.h>
#include <math.h>

// ---------------------------------------------------------------------------
// Internal Transform & Box Math Helpers
// ---------------------------------------------------------------------------

static ZERO_FORCE_INLINE Mat4 mat4_from_trs(Vec3 pos, Quat q, Vec3 s) {
    float x2 = q.x + q.x, y2 = q.y + q.y, z2 = q.z + q.z;
    float xx = q.x * x2,  xy = q.x * y2,  xz = q.x * z2;
    float yy = q.y * y2,  yz = q.y * z2,  zz = q.z * z2;
    float wx = q.w * x2,  wy = q.w * y2,  wz = q.w * z2;

    Mat4 m;
    m.m[0]  = (1.0f - (yy + zz)) * s.x;
    m.m[1]  = (xy + wz) * s.x;
    m.m[2]  = (xz - wy) * s.x;
    m.m[3]  = 0.0f;

    m.m[4]  = (xy - wz) * s.y;
    m.m[5]  = (1.0f - (xx + zz)) * s.y;
    m.m[6]  = (yz + wx) * s.y;
    m.m[7]  = 0.0f;

    m.m[8]  = (xz + wy) * s.z;
    m.m[9]  = (yz - wx) * s.z;
    m.m[10] = (1.0f - (xx + yy)) * s.z;
    m.m[11] = 0.0f;

    m.m[12] = pos.x;
    m.m[13] = pos.y;
    m.m[14] = pos.z;
    m.m[15] = 1.0f;
    return m;
}

static ZERO_FORCE_INLINE Vec3 mat4_transform_point(const Mat4 *m, Vec3 p) {
    return vec3(
        m->m[0] * p.x + m->m[4] * p.y + m->m[8]  * p.z + m->m[12],
        m->m[1] * p.x + m->m[5] * p.y + m->m[9]  * p.z + m->m[13],
        m->m[2] * p.x + m->m[6] * p.y + m->m[10] * p.z + m->m[14]
    );
}

static ZeroAABB aabb_transform_by_matrix(ZeroAABB local, const Mat4 *m) {
    Vec3 corners[8] = {
        vec3(local.min.x, local.min.y, local.min.z),
        vec3(local.max.x, local.min.y, local.min.z),
        vec3(local.min.x, local.max.y, local.min.z),
        vec3(local.max.x, local.max.y, local.min.z),
        vec3(local.min.x, local.min.y, local.max.z),
        vec3(local.max.x, local.min.y, local.max.z),
        vec3(local.min.x, local.max.y, local.max.z),
        vec3(local.max.x, local.max.y, local.max.z)
    };

    Vec3 first = mat4_transform_point(m, corners[0]);
    Vec3 min_pt = first;
    Vec3 max_pt = first;

    for (int i = 1; i < 8; ++i) {
        Vec3 t = mat4_transform_point(m, corners[i]);
        min_pt.x = fminf(min_pt.x, t.x);
        min_pt.y = fminf(min_pt.y, t.y);
        min_pt.z = fminf(min_pt.z, t.z);
        max_pt.x = fmaxf(max_pt.x, t.x);
        max_pt.y = fmaxf(max_pt.y, t.y);
        max_pt.z = fmaxf(max_pt.z, t.z);
    }
    return zero_aabb_create(min_pt, max_pt);
}

// ---------------------------------------------------------------------------
// Entity Pool Free-List Management
// ---------------------------------------------------------------------------

static void scene_init_pool(ZeroScene *scene) {
    scene->entity_count = 0;
    scene->capacity = ZERO_MAX_SCENE_ENTITIES;
    scene->free_list_head = 0;

    for (uint32_t i = 0; i < ZERO_MAX_SCENE_ENTITIES; ++i) {
        ZeroEntity *e = &scene->entities[i];
        memset(e, 0, sizeof(ZeroEntity));
        e->id = i;
        e->is_active = false;
        e->parent_id = -1;
        e->first_child = -1;
        e->next_sibling = -1;
        e->prev_sibling = -1;
        e->rigid_body_id = -1;
        e->next_free = (i + 1 < ZERO_MAX_SCENE_ENTITIES) ? (int32_t)(i + 1) : -1;
    }
}

static ZeroEntity *entity_alloc(ZeroScene *scene) {
    if (!scene || scene->free_list_head < 0 || scene->entity_count >= ZERO_MAX_SCENE_ENTITIES) {
        return NULL;
    }

    int32_t idx = scene->free_list_head;
    ZeroEntity *e = &scene->entities[idx];
    scene->free_list_head = e->next_free;
    scene->entity_count++;

    // Clear and initialize fields
    memset(e->name, 0, sizeof(e->name));
    memset(e->state, 0, sizeof(e->state));
    e->type = ZERO_ENTITY_TYPE_GENERIC;
    e->is_active = true;
    e->is_visible = true;
    e->is_dirty = true;

    e->parent_id = -1;
    e->first_child = -1;
    e->next_sibling = -1;
    e->prev_sibling = -1;

    e->local_pos = vec3(0.0f, 0.0f, 0.0f);
    e->local_rot = quat_identity();
    e->local_scale = vec3(1.0f, 1.0f, 1.0f);
    e->local_matrix = mat4_identity();

    e->world_pos = vec3(0.0f, 0.0f, 0.0f);
    e->world_rot = quat_identity();
    e->world_scale = vec3(1.0f, 1.0f, 1.0f);
    e->world_matrix = mat4_identity();

    e->local_aabb = zero_aabb_create(vec3(-0.5f, -0.5f, -0.5f), vec3(0.5f, 0.5f, 0.5f));
    e->world_aabb = e->local_aabb;

    e->velocity = vec3(0.0f, 0.0f, 0.0f);
    e->color = vec4(1.0f, 1.0f, 1.0f, 1.0f);
    e->mesh = NULL;
    e->rigid_body_id = -1;
    e->next_free = -1;

    return e;
}

static void entity_unlink_from_parent(ZeroScene *scene, ZeroEntity *child) {
    if (!scene || !child || child->parent_id < 0) return;

    ZeroEntity *parent = &scene->entities[child->parent_id];
    if (child->prev_sibling >= 0) {
        scene->entities[child->prev_sibling].next_sibling = child->next_sibling;
    } else {
        parent->first_child = child->next_sibling;
    }

    if (child->next_sibling >= 0) {
        scene->entities[child->next_sibling].prev_sibling = child->prev_sibling;
    }

    child->parent_id = -1;
    child->prev_sibling = -1;
    child->next_sibling = -1;
}

// ---------------------------------------------------------------------------
// Scene Lifecycle
// ---------------------------------------------------------------------------

ZeroScene *zero_scene_create(const char *name) {
    size_t alloc_size = sizeof(ZeroScene) + 64 * 1024;
    ZeroArena arena = zero_arena_create(alloc_size);
    if (!arena.buffer) {
        return NULL;
    }

    ZeroScene *scene = (ZeroScene *)zero_arena_alloc(&arena, sizeof(ZeroScene), 64);
    if (!scene) {
        zero_arena_destroy(&arena);
        return NULL;
    }

    memset(scene, 0, sizeof(ZeroScene));
    scene->arena_storage = arena;
    scene->arena = &scene->arena_storage;
    scene->owns_arena = true;

    if (name && name[0] != '\0') {
        snprintf(scene->name, sizeof(scene->name), "%s", name);
    } else {
        snprintf(scene->name, sizeof(scene->name), "default_scene");
    }

    scene_init_pool(scene);

    scene->tick = 0;
    scene->time = 0.0f;
    scene->time_step = 1.0f / 60.0f;

    scene->camera = zero_camera_create(vec3(0.0f, 2.0f, -5.0f), ZERO_DEG2RAD(60.0f), 16.0f / 9.0f, 0.1f, 1000.0f);
    scene->physics = NULL;
    scene->owns_physics = false;

    scene->sun_direction = vec3_normalize(vec3(-0.5f, -0.8f, 0.3f));
    scene->sun_color = vec3(1.0f, 0.95f, 0.85f);

    return scene;
}

ZeroScene *zero_scene_create_with_arena(ZeroArena *arena, const char *name) {
    if (!arena) {
        return zero_scene_create(name);
    }

    ZeroScene *scene = (ZeroScene *)zero_arena_alloc(arena, sizeof(ZeroScene), 64);
    if (!scene) return NULL;

    memset(scene, 0, sizeof(ZeroScene));
    scene->arena = arena;
    scene->owns_arena = false;

    if (name && name[0] != '\0') {
        snprintf(scene->name, sizeof(scene->name), "%s", name);
    } else {
        snprintf(scene->name, sizeof(scene->name), "default_scene");
    }

    scene_init_pool(scene);

    scene->tick = 0;
    scene->time = 0.0f;
    scene->time_step = 1.0f / 60.0f;

    scene->camera = zero_camera_create(vec3(0.0f, 2.0f, -5.0f), ZERO_DEG2RAD(60.0f), 16.0f / 9.0f, 0.1f, 1000.0f);
    scene->physics = NULL;
    scene->owns_physics = false;

    scene->sun_direction = vec3_normalize(vec3(-0.5f, -0.8f, 0.3f));
    scene->sun_color = vec3(1.0f, 0.95f, 0.85f);

    return scene;
}

void zero_scene_destroy(ZeroScene *scene) {
    if (!scene) return;

    if (scene->owns_physics && scene->physics) {
        zero_physics_world_destroy(scene->physics);
        scene->physics = NULL;
    }

    if (scene->owns_arena) {
        ZeroArena arena_copy = scene->arena_storage;
        zero_arena_destroy(&arena_copy);
    }
}

// ---------------------------------------------------------------------------
// Entity Spawning API
// ---------------------------------------------------------------------------

ZeroEntity *zero_entity_spawn_empty(ZeroScene *scene, const char *name) {
    ZeroEntity *e = entity_alloc(scene);
    if (!e) return NULL;

    if (name && name[0] != '\0') {
        snprintf(e->name, sizeof(e->name), "%s", name);
    } else {
        snprintf(e->name, sizeof(e->name), "entity_%u", e->id);
    }
    return e;
}

ZeroEntity *zero_entity_spawn_directional_light(ZeroScene *scene, Vec3 dir, Vec3 color) {
    ZeroEntity *e = zero_entity_spawn_empty(scene, "sun_light");
    if (!e) return NULL;

    e->type = ZERO_ENTITY_TYPE_LIGHT_DIRECTIONAL;
    Vec3 norm_dir = vec3_normalize(dir);
    scene->sun_direction = norm_dir;
    scene->sun_color = color;
    e->color = vec4(color.x, color.y, color.z, 1.0f);
    e->is_visible = false;
    return e;
}

ZeroEntity *zero_entity_spawn_point_light(ZeroScene *scene, const char *name, Vec3 pos, Vec3 color, float radius, float intensity) {
    ZeroEntity *e = zero_entity_spawn_empty(scene, name ? name : "point_light");
    if (!e) return NULL;

    e->type = ZERO_ENTITY_TYPE_LIGHT_POINT;
    e->local_pos = pos;
    e->world_pos = pos;
    e->color = vec4(color.x, color.y, color.z, intensity);
    e->local_aabb = zero_aabb_create(vec3(-radius, -radius, -radius), vec3(radius, radius, radius));
    e->world_aabb = zero_aabb_create(vec3_sub(pos, vec3(radius, radius, radius)), vec3_add(pos, vec3(radius, radius, radius)));
    e->is_dirty = true;
    return e;
}

ZeroEntity *zero_entity_spawn_humanoid(ZeroScene *scene, Vec3 pos) {
    ZeroEntity *e = zero_entity_spawn_empty(scene, "player");
    if (!e) return NULL;

    e->type = ZERO_ENTITY_TYPE_HUMANOID;
    snprintf(e->state, sizeof(e->state), "grounded_idle");

    e->local_pos = pos;
    e->world_pos = pos;
    // Dimensions: 0.6m wide, 1.8m high (matches ARCHITECTURE.md §6.2)
    e->local_aabb = zero_aabb_create(vec3(-0.3f, 0.0f, -0.3f), vec3(0.3f, 1.8f, 0.3f));
    e->world_aabb = zero_aabb_create(vec3(pos.x - 0.3f, pos.y, pos.z - 0.3f),
                                     vec3(pos.x + 0.3f, pos.y + 1.8f, pos.z + 0.3f));
    e->is_dirty = true;
    return e;
}

ZeroEntity *zero_entity_spawn_mesh(ZeroScene *scene, const char *name, const ZeroMesh *mesh, Vec3 pos) {
    ZeroEntity *e = zero_entity_spawn_empty(scene, name ? name : "mesh_entity");
    if (!e) return NULL;

    e->type = ZERO_ENTITY_TYPE_MESH;
    e->mesh = mesh;
    e->local_pos = pos;
    e->world_pos = pos;
    e->is_dirty = true;
    return e;
}

ZeroEntity *zero_entity_spawn_cube(ZeroScene *scene, const char *name, Vec3 pos, Vec3 size, Vec4 color) {
    ZeroEntity *e = zero_entity_spawn_empty(scene, name ? name : "cube");
    if (!e) return NULL;

    e->type = ZERO_ENTITY_TYPE_MESH;
    e->local_pos = pos;
    e->world_pos = pos;
    e->local_scale = size;
    e->world_scale = size;
    e->color = color;
    Vec3 half = vec3_scale(size, 0.5f);
    e->local_aabb = zero_aabb_create(vec3(-half.x, -half.y, -half.z), vec3(half.x, half.y, half.z));
    e->world_aabb = zero_aabb_create(vec3_sub(pos, half), vec3_add(pos, half));
    e->is_dirty = true;
    return e;
}

ZeroEntity *zero_entity_spawn_sphere(ZeroScene *scene, const char *name, Vec3 pos, float radius, Vec4 color) {
    ZeroEntity *e = zero_entity_spawn_empty(scene, name ? name : "sphere");
    if (!e) return NULL;

    e->type = ZERO_ENTITY_TYPE_MESH;
    e->local_pos = pos;
    e->world_pos = pos;
    e->local_scale = vec3(radius * 2.0f, radius * 2.0f, radius * 2.0f);
    e->world_scale = e->local_scale;
    e->color = color;
    e->local_aabb = zero_aabb_create(vec3(-radius, -radius, -radius), vec3(radius, radius, radius));
    e->world_aabb = zero_aabb_create(vec3_sub(pos, vec3(radius, radius, radius)), vec3_add(pos, vec3(radius, radius, radius)));
    e->is_dirty = true;
    return e;
}

ZeroEntity *zero_entity_spawn_plane(ZeroScene *scene, const char *name, Vec3 pos, Vec2 size, Vec4 color) {
    ZeroEntity *e = zero_entity_spawn_empty(scene, name ? name : "plane");
    if (!e) return NULL;

    e->type = ZERO_ENTITY_TYPE_MESH;
    e->local_pos = pos;
    e->world_pos = pos;
    e->color = color;
    float hx = size.x * 0.5f;
    float hz = size.y * 0.5f;
    e->local_aabb = zero_aabb_create(vec3(-hx, -0.01f, -hz), vec3(hx, 0.01f, hz));
    e->world_aabb = zero_aabb_create(vec3(pos.x - hx, pos.y - 0.01f, pos.z - hz),
                                     vec3(pos.x + hx, pos.y + 0.01f, pos.z + hz));
    e->is_dirty = true;
    return e;
}

ZeroEntity *zero_entity_spawn_rigid_body(ZeroScene *scene, const char *name, const ZeroRigidBody *body_def) {
    ZeroEntity *e = zero_entity_spawn_empty(scene, name ? name : "rigid_body");
    if (!e) return NULL;

    e->type = ZERO_ENTITY_TYPE_RIGID_BODY;
    if (body_def) {
        if (scene->physics) {
            // Check if this body is already present in scene's physics world
            if (body_def->id > 0 && (int)body_def->id <= scene->physics->body_count) {
                const ZeroRigidBody *b = &scene->physics->bodies[body_def->id - 1];
                e->rigid_body_id = (int32_t)b->id;
                e->local_pos = b->position;
                e->world_pos = b->position;
                e->local_rot = b->orientation;
                e->world_rot = b->orientation;
                e->local_aabb = b->aabb;
                e->world_aabb = b->aabb;
                e->velocity = b->linear_velocity;
            } else {
                ZeroRigidBody *b = zero_physics_add_body(scene->physics, body_def);
                if (b) {
                    e->rigid_body_id = (int32_t)b->id;
                    e->local_pos = b->position;
                    e->world_pos = b->position;
                    e->local_rot = b->orientation;
                    e->world_rot = b->orientation;
                    e->local_aabb = b->aabb;
                    e->world_aabb = b->aabb;
                    e->velocity = b->linear_velocity;
                }
            }
        } else {
            e->rigid_body_id = (int32_t)body_def->id;
            e->local_pos = body_def->position;
            e->world_pos = body_def->position;
            e->local_rot = body_def->orientation;
            e->world_rot = body_def->orientation;
            e->local_aabb = body_def->aabb;
            e->world_aabb = body_def->aabb;
            e->velocity = body_def->linear_velocity;
        }
    }
    e->is_dirty = true;
    return e;
}

void zero_entity_destroy(ZeroScene *scene, uint32_t entity_id) {
    if (!scene || entity_id >= ZERO_MAX_SCENE_ENTITIES) return;

    ZeroEntity *e = &scene->entities[entity_id];
    if (!e->is_active) return;

    // Unlink from parent
    entity_unlink_from_parent(scene, e);

    // Recursively destroy active children
    int32_t child_id = e->first_child;
    while (child_id >= 0) {
        int32_t next = scene->entities[child_id].next_sibling;
        zero_entity_destroy(scene, (uint32_t)child_id);
        child_id = next;
    }

    // Reset entity and return to free-list
    e->is_active = false;
    e->is_visible = false;
    e->first_child = -1;
    e->next_sibling = -1;
    e->prev_sibling = -1;
    e->parent_id = -1;

    e->next_free = scene->free_list_head;
    scene->free_list_head = (int32_t)entity_id;
    scene->entity_count--;
}

// ---------------------------------------------------------------------------
// Hierarchy & Transforms
// ---------------------------------------------------------------------------

bool zero_entity_attach_child(ZeroScene *scene, uint32_t parent_id, uint32_t child_id) {
    if (!scene) return false;
    if (parent_id >= ZERO_MAX_SCENE_ENTITIES || child_id >= ZERO_MAX_SCENE_ENTITIES) return false;
    if (parent_id == child_id) return false;

    ZeroEntity *parent = &scene->entities[parent_id];
    ZeroEntity *child = &scene->entities[child_id];
    if (!parent->is_active || !child->is_active) return false;

    // Cycle Detection: walk up parent's ancestor chain to ensure parent is not a descendant of child
    int32_t curr = (int32_t)parent_id;
    int guard = 0;
    while (curr >= 0 && guard < ZERO_MAX_SCENE_ENTITIES) {
        if (curr == (int32_t)child_id) {
            return false; // Cycle detected!
        }
        curr = scene->entities[curr].parent_id;
        guard++;
    }

    // If already attached to this parent, do nothing
    if (child->parent_id == (int32_t)parent_id) return true;

    // Unlink from previous parent if attached elsewhere
    if (child->parent_id >= 0) {
        entity_unlink_from_parent(scene, child);
    }

    // Attach to parent: prepend to parent's first_child list
    child->parent_id = (int32_t)parent_id;
    child->prev_sibling = -1;
    child->next_sibling = parent->first_child;
    if (parent->first_child >= 0) {
        scene->entities[parent->first_child].prev_sibling = (int32_t)child_id;
    }
    parent->first_child = (int32_t)child_id;

    child->is_dirty = true;
    return true;
}

bool zero_entity_detach_child(ZeroScene *scene, uint32_t child_id) {
    if (!scene || child_id >= ZERO_MAX_SCENE_ENTITIES) return false;

    ZeroEntity *child = &scene->entities[child_id];
    if (!child->is_active || child->parent_id < 0) return false;

    // Detach preserving world transform:
    // When becoming a root entity, local TRS must equal current cached world TRS
    child->local_pos = child->world_pos;
    child->local_rot = child->world_rot;
    child->local_scale = child->world_scale;
    child->local_matrix = child->world_matrix;
    child->is_dirty = false;

    entity_unlink_from_parent(scene, child);
    return true;
}

void zero_entity_set_local_transform(ZeroEntity *entity, Vec3 pos, Quat rot, Vec3 scale) {
    if (!entity) return;
    entity->local_pos = pos;
    entity->local_rot = rot;
    entity->local_scale = scale;
    entity->is_dirty = true;
}

void zero_entity_set_position(ZeroEntity *entity, Vec3 pos) {
    if (!entity) return;
    entity->local_pos = pos;
    entity->is_dirty = true;
}

void zero_entity_set_rotation(ZeroEntity *entity, Quat rot) {
    if (!entity) return;
    entity->local_rot = rot;
    entity->is_dirty = true;
}

void zero_entity_set_scale(ZeroEntity *entity, Vec3 scale) {
    if (!entity) return;
    entity->local_scale = scale;
    entity->is_dirty = true;
}

// ---------------------------------------------------------------------------
// Cascading Transform Update Algorithm
// ---------------------------------------------------------------------------

static void update_entity_transforms_recursive(ZeroScene *scene, int32_t entity_id, bool parent_dirty, int depth) {
    if (entity_id < 0 || entity_id >= ZERO_MAX_SCENE_ENTITIES || depth >= ZERO_MAX_SCENE_ENTITIES) return;

    ZeroEntity *entity = &scene->entities[entity_id];
    if (!entity->is_active) return;

    bool needs_update = entity->is_dirty || parent_dirty;

    if (needs_update) {
        // 1. Recompute local matrix from local TRS
        entity->local_matrix = mat4_from_trs(entity->local_pos, entity->local_rot, entity->local_scale);

        // 2. Recompute world matrix and world rotation
        if (entity->parent_id >= 0) {
            const ZeroEntity *parent = &scene->entities[entity->parent_id];
            entity->world_matrix = mat4_mul(parent->world_matrix, entity->local_matrix);
            entity->world_rot = quat_normalize(quat_mul(parent->world_rot, entity->local_rot));
        } else {
            entity->world_matrix = entity->local_matrix;
            entity->world_rot = entity->local_rot;
        }

        // 3. Extract world position
        entity->world_pos = vec3(entity->world_matrix.m[12],
                                 entity->world_matrix.m[13],
                                 entity->world_matrix.m[14]);

        // 4. Extract world scale from matrix column lengths
        float sx = vec3_length(vec3(entity->world_matrix.m[0], entity->world_matrix.m[1], entity->world_matrix.m[2]));
        float sy = vec3_length(vec3(entity->world_matrix.m[4], entity->world_matrix.m[5], entity->world_matrix.m[6]));
        float sz = vec3_length(vec3(entity->world_matrix.m[8], entity->world_matrix.m[9], entity->world_matrix.m[10]));
        entity->world_scale = vec3(sx, sy, sz);

        // 5. Transform bounding AABB to world space
        entity->world_aabb = aabb_transform_by_matrix(entity->local_aabb, &entity->world_matrix);

        // 6. Clear local dirty flag
        entity->is_dirty = false;
    }

    // 7. Cascade dirty propagation down through all children
    int32_t child_id = entity->first_child;
    while (child_id >= 0) {
        int32_t next = scene->entities[child_id].next_sibling;
        update_entity_transforms_recursive(scene, child_id, needs_update, depth + 1);
        child_id = next;
    }
}

void zero_scene_update_transforms(ZeroScene *scene) {
    if (!scene) return;

    for (uint32_t i = 0; i < ZERO_MAX_SCENE_ENTITIES; ++i) {
        ZeroEntity *entity = &scene->entities[i];
        if (entity->is_active && entity->parent_id < 0) {
            update_entity_transforms_recursive(scene, (int32_t)i, false, 0);
        }
    }
}

// ---------------------------------------------------------------------------
// Stepping & Rendering
// ---------------------------------------------------------------------------

static void zero_scene_sync_physics_transforms(ZeroScene *scene) {
    if (!scene || !scene->physics) return;

    ZeroPhysicsWorld *pw = scene->physics;

    for (uint32_t i = 0; i < scene->capacity; ++i) {
        ZeroEntity *e = &scene->entities[i];
        if (!e->is_active || e->rigid_body_id <= 0) continue;

        uint32_t body_idx = (uint32_t)(e->rigid_body_id - 1);
        if (body_idx >= (uint32_t)pw->body_count) continue;

        const ZeroRigidBody *b = &pw->bodies[body_idx];
        if (!b->is_active) continue;

        if (e->parent_id < 0) {
            // Root entity: local transform directly reflects world transform
            e->local_pos = b->position;
            e->local_rot = b->orientation;
        } else {
            // Child entity: transform world physics pos/rot into parent local space
            const ZeroEntity *parent = &scene->entities[e->parent_id];
            Mat4 inv_parent_mat = mat4_inverse(parent->world_matrix);

            e->local_pos = vec3(
                inv_parent_mat.m[0] * b->position.x + inv_parent_mat.m[4] * b->position.y + inv_parent_mat.m[8]  * b->position.z + inv_parent_mat.m[12],
                inv_parent_mat.m[1] * b->position.x + inv_parent_mat.m[5] * b->position.y + inv_parent_mat.m[9]  * b->position.z + inv_parent_mat.m[13],
                inv_parent_mat.m[2] * b->position.x + inv_parent_mat.m[6] * b->position.y + inv_parent_mat.m[10] * b->position.z + inv_parent_mat.m[14]
            );

            Quat inv_parent_rot = quat_conjugate(parent->world_rot);
            e->local_rot = quat_normalize(quat_mul(inv_parent_rot, b->orientation));
        }

        e->velocity = b->linear_velocity;
        e->is_dirty = true;
    }
}

void zero_scene_step(ZeroScene *scene, float dt) {
    if (!scene || dt <= 0.0f || isnan(dt) || isinf(dt)) return;

    // 1. Advance discrete simulation tick
    scene->tick++;

    // 2. Advance continuous simulation clock (deterministic multiplication from tick)
    scene->time = (float)scene->tick * dt;

    // 3. Step physics subsystem if attached
    if (scene->physics) {
        zero_physics_step(scene->physics, dt);

        // 4. Synchronize physics bodies -> scene entity transforms
        zero_scene_sync_physics_transforms(scene);
    }

    // 5. Cascade updated transforms through the spatial hierarchy & refresh world AABBs
    zero_scene_update_transforms(scene);
}

void zero_scene_render(ZeroScene *scene, const Mat4 *view, const Mat4 *proj) {
    if (!scene) return;

    Mat4 v = view ? *view : zero_camera_get_view(&scene->camera);
    Mat4 p = proj ? *proj : zero_camera_get_projection(&scene->camera);
    (void)v; (void)p;

    // Render active visible entities with valid meshes
    for (uint32_t i = 0; i < ZERO_MAX_SCENE_ENTITIES; ++i) {
        const ZeroEntity *e = &scene->entities[i];
        if (e->is_active && e->is_visible && e->mesh && e->mesh->is_valid) {
            zero_mesh_draw(e->mesh);
        }
    }
}

// ---------------------------------------------------------------------------
// Entity Queries
// ---------------------------------------------------------------------------

ZeroEntity *zero_scene_get_entity(ZeroScene *scene, uint32_t entity_id) {
    if (!scene || entity_id >= ZERO_MAX_SCENE_ENTITIES) return NULL;
    ZeroEntity *e = &scene->entities[entity_id];
    return e->is_active ? e : NULL;
}

ZeroEntity *zero_scene_find_entity_by_name(const ZeroScene *scene, const char *name) {
    if (!scene || !name) return NULL;
    for (uint32_t i = 0; i < ZERO_MAX_SCENE_ENTITIES; ++i) {
        if (scene->entities[i].is_active && strcmp(scene->entities[i].name, name) == 0) {
            return (ZeroEntity *)&scene->entities[i];
        }
    }
    return NULL;
}

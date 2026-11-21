#ifndef ZERO_SCENE_H
#define ZERO_SCENE_H

#include "zero_core.h"
#include "zero_math.h"
#include "zero_memory.h"
#include "zero_mesh.h"
#include "zero_camera.h"
#include "zero_physics.h"
#include <stdbool.h>
#include <stdint.h>

#define ZERO_MAX_SCENE_ENTITIES 512
#define ZERO_ENTITY_NAME_MAX 64
#define ZERO_ENTITY_STATE_MAX 32

typedef enum ZeroEntityType {
    ZERO_ENTITY_TYPE_GENERIC = 0,
    ZERO_ENTITY_TYPE_LIGHT_DIRECTIONAL,
    ZERO_ENTITY_TYPE_LIGHT_POINT,
    ZERO_ENTITY_TYPE_HUMANOID,
    ZERO_ENTITY_TYPE_MESH,
    ZERO_ENTITY_TYPE_RIGID_BODY,
    ZERO_ENTITY_TYPE_CAMERA
} ZeroEntityType;

typedef struct ZeroEntity {
    uint32_t id;
    char name[ZERO_ENTITY_NAME_MAX];
    char state[ZERO_ENTITY_STATE_MAX];
    ZeroEntityType type;
    bool is_active;
    bool is_visible;
    bool is_dirty;

    // Hierarchy links (Left-Child Right-Sibling + Prev-Sibling)
    int32_t parent_id;
    int32_t first_child;
    int32_t next_sibling;
    int32_t prev_sibling;

    // Local spatial transforms
    Vec3 local_pos;
    Quat local_rot;
    Vec3 local_scale;
    Mat4 local_matrix;

    // Cached world spatial transforms
    Vec3 world_pos;
    Quat world_rot;
    Vec3 world_scale;
    Mat4 world_matrix;

    // Bounding volumes
    ZeroAABB local_aabb;
    ZeroAABB world_aabb;

    // Dynamics & Rendering
    Vec3 velocity;
    Vec4 color;
    const ZeroMesh *mesh;
    int32_t rigid_body_id; // -1 if not linked to physics body

    // Embedded free-list link
    int32_t next_free;
} ZeroEntity;

typedef struct ZeroScene {
    char name[ZERO_ENTITY_NAME_MAX];
    ZeroArena *arena;
    ZeroArena arena_storage;
    bool owns_arena;

    // Preallocated entity pool
    ZeroEntity entities[ZERO_MAX_SCENE_ENTITIES];
    uint32_t entity_count;
    uint32_t capacity;
    int32_t free_list_head;

    // Simulation clock
    uint64_t tick;
    float time;
    float time_step;

    // Camera and physics integration
    ZeroCamera camera;
    ZeroPhysicsWorld *physics;
    bool owns_physics;

    // Lighting uniforms (for CSM and Sky integration)
    Vec3 sun_direction;
    Vec3 sun_color;
} ZeroScene;

// Declarative Scene Lifecycle
ZeroScene *zero_scene_create(const char *name);
ZeroScene *zero_scene_create_with_arena(ZeroArena *arena, const char *name);
void zero_scene_destroy(ZeroScene *scene);

// Entity Spawning Declarative API
ZeroEntity *zero_entity_spawn_empty(ZeroScene *scene, const char *name);
ZeroEntity *zero_entity_spawn_directional_light(ZeroScene *scene, Vec3 dir, Vec3 color);
ZeroEntity *zero_entity_spawn_point_light(ZeroScene *scene, const char *name, Vec3 pos, Vec3 color, float radius, float intensity);
ZeroEntity *zero_entity_spawn_humanoid(ZeroScene *scene, Vec3 pos);
ZeroEntity *zero_entity_spawn_mesh(ZeroScene *scene, const char *name, const ZeroMesh *mesh, Vec3 pos);
ZeroEntity *zero_entity_spawn_cube(ZeroScene *scene, const char *name, Vec3 pos, Vec3 size, Vec4 color);
ZeroEntity *zero_entity_spawn_sphere(ZeroScene *scene, const char *name, Vec3 pos, float radius, Vec4 color);
ZeroEntity *zero_entity_spawn_plane(ZeroScene *scene, const char *name, Vec3 pos, Vec2 size, Vec4 color);
ZeroEntity *zero_entity_spawn_rigid_body(ZeroScene *scene, const char *name, const ZeroRigidBody *body_def);
void zero_entity_destroy(ZeroScene *scene, uint32_t entity_id);

// Hierarchy & Spatial Transforms
bool zero_entity_attach_child(ZeroScene *scene, uint32_t parent_id, uint32_t child_id);
bool zero_entity_detach_child(ZeroScene *scene, uint32_t child_id);
void zero_entity_set_local_transform(ZeroEntity *entity, Vec3 pos, Quat rot, Vec3 scale);
void zero_entity_set_position(ZeroEntity *entity, Vec3 pos);
void zero_entity_set_rotation(ZeroEntity *entity, Quat rot);
void zero_entity_set_scale(ZeroEntity *entity, Vec3 scale);
void zero_scene_update_transforms(ZeroScene *scene);

// Rendering & Stepping
void zero_scene_render(ZeroScene *scene, const Mat4 *view, const Mat4 *proj);
void zero_scene_step(ZeroScene *scene, float dt);

// Query Helpers
ZeroEntity *zero_scene_find_entity_by_name(const ZeroScene *scene, const char *name);
ZeroEntity *zero_scene_get_entity(ZeroScene *scene, uint32_t entity_id);

#endif // ZERO_SCENE_H

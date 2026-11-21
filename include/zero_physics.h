#ifndef ZERO_PHYSICS_H
#define ZERO_PHYSICS_H

#include "zero_core.h"
#include "zero_math.h"
#include <stdbool.h>
#include <stdint.h>

#define ZERO_MAX_RIGID_BODIES 256
#define ZERO_MAX_MANIFOLDS 1024
#define ZERO_MAX_CONTACT_POINTS 4
#define ZERO_MAX_GJK_ITERATIONS 32
#define ZERO_MAX_EPA_ITERATIONS 32
#define ZERO_MAX_EPA_VERTS 64
#define ZERO_MAX_EPA_FACES 128

typedef struct ZeroAABB {
    Vec3 min;
    Vec3 max;
} ZeroAABB;

static inline ZeroAABB zero_aabb_create(Vec3 min, Vec3 max) {
    ZeroAABB aabb = {min, max};
    return aabb;
}

static inline bool zero_aabb_intersects(ZeroAABB a, ZeroAABB b) {
    return (a.min.x <= b.max.x && a.max.x >= b.min.x) &&
           (a.min.y <= b.max.y && a.max.y >= b.min.y) &&
           (a.min.z <= b.max.z && a.max.z >= b.min.z);
}

static inline ZeroAABB zero_aabb_union(ZeroAABB a, ZeroAABB b) {
    ZeroAABB res;
    res.min.x = fminf(a.min.x, b.min.x);
    res.min.y = fminf(a.min.y, b.min.y);
    res.min.z = fminf(a.min.z, b.min.z);
    res.max.x = fmaxf(a.max.x, b.max.x);
    res.max.y = fmaxf(a.max.y, b.max.y);
    res.max.z = fmaxf(a.max.z, b.max.z);
    return res;
}

static inline float zero_aabb_surface_area(ZeroAABB aabb) {
    Vec3 d = vec3_sub(aabb.max, aabb.min);
    return 2.0f * (d.x * d.y + d.y * d.z + d.z * d.x);
}

typedef enum ZeroColliderType {
    ZERO_COLLIDER_SPHERE = 0,
    ZERO_COLLIDER_BOX = 1,
    ZERO_COLLIDER_CAPSULE = 2
} ZeroColliderType;

typedef struct ZeroCollider {
    ZeroColliderType type;
    union {
        struct {
            float radius;
        } sphere;
        struct {
            Vec3 half_extents;
        } box;
        struct {
            float radius;
            float half_height;
        } capsule;
    };
} ZeroCollider;

typedef struct ZeroRigidBody {
    uint32_t id;
    bool is_static;
    bool is_active;

    Vec3 position;
    Vec3 linear_velocity;
    Vec3 force;

    Quat orientation;
    Vec3 angular_velocity;
    Vec3 torque;

    float mass;
    float inv_mass;
    Vec3 inertia_diag;
    Vec3 inv_inertia_diag;

    float restitution;
    float friction;
    float linear_damping;
    float angular_damping;

    ZeroCollider collider;
    ZeroAABB aabb;
    int aabb_node_id;
} ZeroRigidBody;

typedef struct ZeroContactPoint {
    Vec3 position;
    Vec3 normal; // Points from body A to body B
    float penetration;
    float normal_impulse;
    float tangent_impulse[2];
    Vec3 rA;
    Vec3 rB;
    float normal_mass;
    float tangent_mass[2];
} ZeroContactPoint;

typedef struct ZeroManifold {
    uint32_t body_a_id;
    uint32_t body_b_id;
    ZeroContactPoint points[ZERO_MAX_CONTACT_POINTS];
    int point_count;
} ZeroManifold;

// Broadphase Dynamic AABB Tree Node
typedef struct ZeroAABBNode {
    ZeroAABB aabb;
    int parent;
    int left;
    int right;
    int height;
    int body_id; // -1 for internal branch nodes
} ZeroAABBNode;

typedef struct ZeroDynamicAABBTree {
    ZeroAABBNode *nodes;
    int root;
    int count;       // Total active nodes (leaves + internal)
    int leaf_count;  // Total active leaf nodes
    int capacity;
    int free_list;
} ZeroDynamicAABBTree;

typedef struct ZeroPhysicsWorld {
    ZeroRigidBody bodies[ZERO_MAX_RIGID_BODIES];
    int body_count;

    ZeroDynamicAABBTree broadphase;
    ZeroManifold manifolds[ZERO_MAX_MANIFOLDS];
    int manifold_count;

    Vec3 gravity;
    int velocity_iterations;
    int position_iterations;
    float allowed_penetration;
    float bias_factor;
} ZeroPhysicsWorld;

// Support Mapping Function Interface for GJK & EPA
Vec3 zero_collider_support(const ZeroCollider *collider, Vec3 position, Quat orientation, Vec3 dir);

// Dynamic AABB Tree Functions
ZeroDynamicAABBTree zero_aabb_tree_create(int initial_capacity);
void zero_aabb_tree_destroy(ZeroDynamicAABBTree *tree);
int zero_aabb_tree_insert(ZeroDynamicAABBTree *tree, ZeroAABB aabb, int body_id);
void zero_aabb_tree_remove(ZeroDynamicAABBTree *tree, int node_id);
void zero_aabb_tree_update(ZeroDynamicAABBTree *tree, int node_id, ZeroAABB new_aabb, Vec3 velocity);

// Narrowphase: GJK & EPA
bool zero_gjk_intersection(const ZeroCollider *colA, Vec3 posA, Quat rotA,
                          const ZeroCollider *colB, Vec3 posB, Quat rotB,
                          Vec3 *out_simplex, int *out_simplex_count);

bool zero_epa_penetration(const ZeroCollider *colA, Vec3 posA, Quat rotA,
                          const ZeroCollider *colB, Vec3 posB, Quat rotB,
                          Vec3 *simplex, int simplex_count,
                          Vec3 *out_normal, float *out_depth, Vec3 *out_contact_point);

// Physics World Simulation API
ZeroPhysicsWorld zero_physics_world_create(void);
void zero_physics_world_destroy(ZeroPhysicsWorld *world);
ZeroRigidBody *zero_physics_add_body(ZeroPhysicsWorld *world, const ZeroRigidBody *def);
void zero_physics_step(ZeroPhysicsWorld *world, float dt);

// Continuous Collision Detection (Raycast / Swept Sphere)
bool zero_physics_raycast(const ZeroPhysicsWorld *world, Vec3 origin, Vec3 dir, float max_dist,
                          float *out_hit_dist, Vec3 *out_hit_normal, uint32_t *out_hit_body_id);

// AI Semantic Inspection API
int zero_physics_inspect_scene(const ZeroPhysicsWorld *world, char *buffer, int buffer_size);

#endif

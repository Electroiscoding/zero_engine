#include "zero_physics.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

static inline Vec3 quat_rotate_vec3(Quat q, Vec3 v) {
    Vec3 qv = vec3(q.x, q.y, q.z);
    Vec3 uv = vec3_cross(qv, v);
    Vec3 uuv = vec3_cross(qv, uv);
    return vec3_add(v, vec3_scale(vec3_add(vec3_scale(uv, q.w), uuv), 2.0f));
}

static inline Vec3 quat_unrotate_vec3(Quat q, Vec3 v) {
    Quat inv_q = quat_conjugate(q);
    return quat_rotate_vec3(inv_q, v);
}

Vec3 zero_collider_support(const ZeroCollider *collider, Vec3 position, Quat orientation, Vec3 dir) {
    Vec3 local_dir = quat_unrotate_vec3(orientation, dir);
    Vec3 local_support = vec3(0, 0, 0);

    switch (collider->type) {
        case ZERO_COLLIDER_SPHERE: {
            float len = vec3_length(local_dir);
            if (len > 1e-6f) {
                local_support = vec3_scale(local_dir, collider->sphere.radius / len);
            }
            break;
        }
        case ZERO_COLLIDER_BOX: {
            local_support.x = (local_dir.x >= 0.0f) ? collider->box.half_extents.x : -collider->box.half_extents.x;
            local_support.y = (local_dir.y >= 0.0f) ? collider->box.half_extents.y : -collider->box.half_extents.y;
            local_support.z = (local_dir.z >= 0.0f) ? collider->box.half_extents.z : -collider->box.half_extents.z;
            break;
        }
        case ZERO_COLLIDER_CAPSULE: {
            float len = vec3_length(local_dir);
            Vec3 sphere_part = vec3(0, 0, 0);
            if (len > 1e-6f) {
                sphere_part = vec3_scale(local_dir, collider->capsule.radius / len);
            }
            float hh = collider->capsule.half_height;
            local_support = (local_dir.y >= 0.0f) ? vec3(0, hh, 0) : vec3(0, -hh, 0);
            local_support = vec3_add(local_support, sphere_part);
            break;
        }
    }

    return vec3_add(position, quat_rotate_vec3(orientation, local_support));
}

// -----------------------------------------------------------------------------
// Dynamic AABB Tree
// -----------------------------------------------------------------------------

ZeroDynamicAABBTree zero_aabb_tree_create(int initial_capacity) {
    ZeroDynamicAABBTree tree;
    if (initial_capacity < 16) initial_capacity = 16;
    tree.capacity = initial_capacity;
    tree.count = 0;
    tree.root = -1;
    tree.nodes = (ZeroAABBNode *)calloc(tree.capacity, sizeof(ZeroAABBNode));
    for (int i = 0; i < tree.capacity - 1; ++i) {
        tree.nodes[i].parent = i + 1;
        tree.nodes[i].height = -1;
    }
    tree.nodes[tree.capacity - 1].parent = -1;
    tree.nodes[tree.capacity - 1].height = -1;
    tree.free_list = 0;
    return tree;
}

void zero_aabb_tree_destroy(ZeroDynamicAABBTree *tree) {
    if (tree && tree->nodes) {
        free(tree->nodes);
        tree->nodes = NULL;
        tree->capacity = 0;
        tree->count = 0;
        tree->root = -1;
        tree->free_list = -1;
    }
}

static int aabb_tree_alloc_node(ZeroDynamicAABBTree *tree) {
    if (tree->free_list == -1) {
        int old_cap = tree->capacity;
        tree->capacity *= 2;
        tree->nodes = (ZeroAABBNode *)realloc(tree->nodes, tree->capacity * sizeof(ZeroAABBNode));
        memset(tree->nodes + old_cap, 0, old_cap * sizeof(ZeroAABBNode));
        for (int i = old_cap; i < tree->capacity - 1; ++i) {
            tree->nodes[i].parent = i + 1;
            tree->nodes[i].height = -1;
        }
        tree->nodes[tree->capacity - 1].parent = -1;
        tree->nodes[tree->capacity - 1].height = -1;
        tree->free_list = old_cap;
    }
    int id = tree->free_list;
    tree->free_list = tree->nodes[id].parent;
    tree->nodes[id].parent = -1;
    tree->nodes[id].left = -1;
    tree->nodes[id].right = -1;
    tree->nodes[id].height = 0;
    tree->nodes[id].body_id = -1;
    tree->count++;
    return id;
}

static void aabb_tree_free_node(ZeroDynamicAABBTree *tree, int node_id) {
    tree->nodes[node_id].parent = tree->free_list;
    tree->nodes[node_id].height = -1;
    tree->free_list = node_id;
    tree->count--;
}

int zero_aabb_tree_insert(ZeroDynamicAABBTree *tree, ZeroAABB aabb, int body_id) {
    int leaf = aabb_tree_alloc_node(tree);
    tree->nodes[leaf].aabb = aabb;
    tree->nodes[leaf].body_id = body_id;
    tree->nodes[leaf].height = 0;
    tree->leaf_count++;

    if (tree->root == -1) {
        tree->root = leaf;
        return leaf;
    }

    // Find best sibling using Surface Area Heuristic
    ZeroAABB leaf_aabb = aabb;
    int index = tree->root;
    while (tree->nodes[index].left != -1) {
        int left = tree->nodes[index].left;
        int right = tree->nodes[index].right;

        float area = zero_aabb_surface_area(tree->nodes[index].aabb);
        ZeroAABB combined = zero_aabb_union(tree->nodes[index].aabb, leaf_aabb);
        float combined_area = zero_aabb_surface_area(combined);

        float cost = 2.0f * combined_area;
        float inherit_cost = 2.0f * (combined_area - area);

        float cost_left;
        if (tree->nodes[left].left == -1) {
            cost_left = zero_aabb_surface_area(zero_aabb_union(tree->nodes[left].aabb, leaf_aabb)) + inherit_cost;
        } else {
            float old_a = zero_aabb_surface_area(tree->nodes[left].aabb);
            float new_a = zero_aabb_surface_area(zero_aabb_union(tree->nodes[left].aabb, leaf_aabb));
            cost_left = (new_a - old_a) + inherit_cost;
        }

        float cost_right;
        if (tree->nodes[right].left == -1) {
            cost_right = zero_aabb_surface_area(zero_aabb_union(tree->nodes[right].aabb, leaf_aabb)) + inherit_cost;
        } else {
            float old_a = zero_aabb_surface_area(tree->nodes[right].aabb);
            float new_a = zero_aabb_surface_area(zero_aabb_union(tree->nodes[right].aabb, leaf_aabb));
            cost_right = (new_a - old_a) + inherit_cost;
        }

        if (cost < cost_left && cost < cost_right) break;
        index = (cost_left < cost_right) ? left : right;
    }

    int sibling = index;
    int old_parent = tree->nodes[sibling].parent;
    int new_parent = aabb_tree_alloc_node(tree);
    tree->nodes[new_parent].parent = old_parent;
    tree->nodes[new_parent].aabb = zero_aabb_union(leaf_aabb, tree->nodes[sibling].aabb);
    tree->nodes[new_parent].height = tree->nodes[sibling].height + 1;
    tree->nodes[new_parent].body_id = -1;

    if (old_parent != -1) {
        if (tree->nodes[old_parent].left == sibling) {
            tree->nodes[old_parent].left = new_parent;
        } else {
            tree->nodes[old_parent].right = new_parent;
        }
    } else {
        tree->root = new_parent;
    }

    tree->nodes[new_parent].left = sibling;
    tree->nodes[new_parent].right = leaf;
    tree->nodes[sibling].parent = new_parent;
    tree->nodes[leaf].parent = new_parent;

    // Fix heights and AABBs upward
    index = tree->nodes[leaf].parent;
    while (index != -1) {
        int l = tree->nodes[index].left;
        int r = tree->nodes[index].right;
        tree->nodes[index].height = 1 + (tree->nodes[l].height > tree->nodes[r].height ? tree->nodes[l].height : tree->nodes[r].height);
        tree->nodes[index].aabb = zero_aabb_union(tree->nodes[l].aabb, tree->nodes[r].aabb);
        index = tree->nodes[index].parent;
    }

    return leaf;
}

void zero_aabb_tree_remove(ZeroDynamicAABBTree *tree, int node_id) {
    if (node_id == tree->root) {
        tree->root = -1;
        aabb_tree_free_node(tree, node_id);
        return;
    }

    int parent = tree->nodes[node_id].parent;
    int grand_parent = tree->nodes[parent].parent;
    int sibling = (tree->nodes[parent].left == node_id) ? tree->nodes[parent].right : tree->nodes[parent].left;

    if (grand_parent != -1) {
        if (tree->nodes[grand_parent].left == parent) {
            tree->nodes[grand_parent].left = sibling;
        } else {
            tree->nodes[grand_parent].right = sibling;
        }
        tree->nodes[sibling].parent = grand_parent;
        aabb_tree_free_node(tree, parent);

        int index = grand_parent;
        while (index != -1) {
            int l = tree->nodes[index].left;
            int r = tree->nodes[index].right;
            tree->nodes[index].height = 1 + (tree->nodes[l].height > tree->nodes[r].height ? tree->nodes[l].height : tree->nodes[r].height);
            tree->nodes[index].aabb = zero_aabb_union(tree->nodes[l].aabb, tree->nodes[r].aabb);
            index = tree->nodes[index].parent;
        }
    } else {
        tree->root = sibling;
        tree->nodes[sibling].parent = -1;
        aabb_tree_free_node(tree, parent);
    }

    aabb_tree_free_node(tree, node_id);
    tree->leaf_count--;
}

void zero_aabb_tree_update(ZeroDynamicAABBTree *tree, int node_id, ZeroAABB new_aabb, Vec3 velocity) {
    // Fat AABB velocity expansion
    Vec3 margin = vec3(0.1f, 0.1f, 0.1f);
    if (velocity.x < 0.0f) new_aabb.min.x += velocity.x * 2.0f; else new_aabb.max.x += velocity.x * 2.0f;
    if (velocity.y < 0.0f) new_aabb.min.y += velocity.y * 2.0f; else new_aabb.max.y += velocity.y * 2.0f;
    if (velocity.z < 0.0f) new_aabb.min.z += velocity.z * 2.0f; else new_aabb.max.z += velocity.z * 2.0f;
    new_aabb.min = vec3_sub(new_aabb.min, margin);
    new_aabb.max = vec3_add(new_aabb.max, margin);

    int body_id = tree->nodes[node_id].body_id;
    zero_aabb_tree_remove(tree, node_id);
    zero_aabb_tree_insert(tree, new_aabb, body_id);
}

// -----------------------------------------------------------------------------
// GJK (Gilbert-Johnson-Keerthi) & EPA (Expanding Polytope Algorithm)
// -----------------------------------------------------------------------------

typedef struct Simplex {
    Vec3 points[4];
    int count;
} Simplex;

static inline Vec3 minkowski_support(const ZeroCollider *colA, Vec3 posA, Quat rotA,
                                     const ZeroCollider *colB, Vec3 posB, Quat rotB,
                                     Vec3 dir) {
    Vec3 sA = zero_collider_support(colA, posA, rotA, dir);
    Vec3 sB = zero_collider_support(colB, posB, rotB, vec3_scale(dir, -1.0f));
    return vec3_sub(sA, sB);
}

static bool update_simplex(Simplex *s, Vec3 *d) {
    switch (s->count) {
        case 2: { // Line segment [B, A] where A is latest point
            Vec3 a = s->points[1];
            Vec3 b = s->points[0];
            Vec3 ab = vec3_sub(b, a);
            Vec3 ao = vec3_scale(a, -1.0f);
            if (vec3_dot(ab, ao) > 0.0f) {
                *d = vec3_cross(vec3_cross(ab, ao), ab);
                if (vec3_length_sq(*d) < 1e-6f) {
                    *d = vec3_cross(ab, vec3(1, 0, 0));
                    if (vec3_length_sq(*d) < 1e-6f) *d = vec3_cross(ab, vec3(0, 1, 0));
                }
            } else {
                s->points[0] = a;
                s->count = 1;
                *d = ao;
            }
            return false;
        }
        case 3: { // Triangle [C, B, A] where A is latest
            Vec3 a = s->points[2];
            Vec3 b = s->points[1];
            Vec3 c = s->points[0];
            Vec3 ab = vec3_sub(b, a);
            Vec3 ac = vec3_sub(c, a);
            Vec3 ao = vec3_scale(a, -1.0f);
            Vec3 abc = vec3_cross(ab, ac);

            Vec3 ab_perp = vec3_cross(ab, abc);
            if (vec3_dot(ab_perp, ao) > 0.0f) {
                s->points[0] = b;
                s->points[1] = a;
                s->count = 2;
                *d = vec3_cross(vec3_cross(ab, ao), ab);
                return false;
            }

            Vec3 ac_perp = vec3_cross(abc, ac);
            if (vec3_dot(ac_perp, ao) > 0.0f) {
                s->points[0] = c;
                s->points[1] = a;
                s->count = 2;
                *d = vec3_cross(vec3_cross(ac, ao), ac);
                return false;
            }

            if (vec3_dot(abc, ao) > 0.0f) {
                *d = abc;
            } else {
                s->points[0] = b;
                s->points[1] = c;
                *d = vec3_scale(abc, -1.0f);
            }
            return false;
        }
        case 4: { // Tetrahedron [D, C, B, A]
            Vec3 a = s->points[3];
            Vec3 b = s->points[2];
            Vec3 c = s->points[1];
            Vec3 pt_d = s->points[0];
            Vec3 ab = vec3_sub(b, a);
            Vec3 ac = vec3_sub(c, a);
            Vec3 ad = vec3_sub(pt_d, a);
            Vec3 ao = vec3_scale(a, -1.0f);

            Vec3 abc = vec3_cross(ab, ac);
            Vec3 acd = vec3_cross(ac, ad);
            Vec3 adb = vec3_cross(ad, ab);

            if (vec3_dot(abc, ao) > 0.0f) {
                s->points[0] = c;
                s->points[1] = b;
                s->points[2] = a;
                s->count = 3;
                *d = abc;
                return false;
            }
            if (vec3_dot(acd, ao) > 0.0f) {
                s->points[0] = pt_d;
                s->points[1] = c;
                s->points[2] = a;
                s->count = 3;
                *d = acd;
                return false;
            }
            if (vec3_dot(adb, ao) > 0.0f) {
                s->points[0] = b;
                s->points[1] = pt_d;
                s->points[2] = a;
                s->count = 3;
                *d = adb;
                return false;
            }

            return true; // Origin is enclosed within tetrahedron!
        }
    }
    return false;
}

bool zero_gjk_intersection(const ZeroCollider *colA, Vec3 posA, Quat rotA,
                          const ZeroCollider *colB, Vec3 posB, Quat rotB,
                          Vec3 *out_simplex, int *out_simplex_count) {
    Vec3 d = vec3_sub(posB, posA);
    if (vec3_length_sq(d) < 1e-6f) d = vec3(0, 1, 0);

    Simplex s;
    s.count = 0;
    Vec3 a = minkowski_support(colA, posA, rotA, colB, posB, rotB, d);
    s.points[0] = a;
    s.count = 1;
    d = vec3_scale(a, -1.0f);

    for (int iter = 0; iter < ZERO_MAX_GJK_ITERATIONS; ++iter) {
        if (vec3_length_sq(d) < 1e-6f) {
            d = vec3(0, 1, 0);
        }
        a = minkowski_support(colA, posA, rotA, colB, posB, rotB, d);
        if (vec3_dot(a, d) <= 0.0f) {
            return false; // No intersection
        }
        s.points[s.count++] = a;
        if (update_simplex(&s, &d)) {
            if (out_simplex && out_simplex_count) {
                *out_simplex_count = s.count;
                for (int i = 0; i < s.count; ++i) out_simplex[i] = s.points[i];
            }
            return true;
        }
    }
    return false;
}

typedef struct EPAFace {
    int a, b, c;
    Vec3 normal;
    float dist;
} EPAFace;

typedef struct EPAEdge {
    int a, b;
} EPAEdge;

bool zero_epa_penetration(const ZeroCollider *colA, Vec3 posA, Quat rotA,
                          const ZeroCollider *colB, Vec3 posB, Quat rotB,
                          Vec3 *simplex, int simplex_count,
                          Vec3 *out_normal, float *out_depth, Vec3 *out_contact_point) {
    if (simplex_count < 4) {
        *out_normal = vec3(0, 1, 0);
        *out_depth = 0.01f;
        *out_contact_point = posA;
        return true;
    }

    Vec3 verts[ZERO_MAX_EPA_VERTS];
    int num_verts = 4;
    for (int i = 0; i < 4; ++i) verts[i] = simplex[i];

    EPAFace faces[ZERO_MAX_EPA_FACES];
    int num_faces = 4;
    int init_indices[4][3] = {
        {0, 1, 2},
        {0, 3, 1},
        {0, 2, 3},
        {1, 3, 2}
    };

    for (int i = 0; i < 4; ++i) {
        faces[i].a = init_indices[i][0];
        faces[i].b = init_indices[i][1];
        faces[i].c = init_indices[i][2];
        Vec3 va = verts[faces[i].a];
        Vec3 vb = verts[faces[i].b];
        Vec3 vc = verts[faces[i].c];
        Vec3 ab = vec3_sub(vb, va);
        Vec3 ac = vec3_sub(vc, va);
        Vec3 n = vec3_normalize(vec3_cross(ab, ac));
        float d = vec3_dot(n, va);
        if (d < 0.0f) {
            n = vec3_scale(n, -1.0f);
            d = -d;
            int tmp = faces[i].b;
            faces[i].b = faces[i].c;
            faces[i].c = tmp;
        }
        faces[i].normal = n;
        faces[i].dist = d;
    }

    int best_face_idx = 0;
    for (int iter = 0; iter < ZERO_MAX_EPA_ITERATIONS; ++iter) {
        int min_idx = 0;
        float min_dist = faces[0].dist;
        for (int i = 1; i < num_faces; ++i) {
            if (faces[i].dist < min_dist) {
                min_dist = faces[i].dist;
                min_idx = i;
            }
        }
        best_face_idx = min_idx;

        Vec3 search_dir = faces[min_idx].normal;
        Vec3 p = minkowski_support(colA, posA, rotA, colB, posB, rotB, search_dir);
        float p_dist = vec3_dot(search_dir, p);

        if (p_dist - min_dist < 0.005f || num_verts >= ZERO_MAX_EPA_VERTS - 2 || num_faces >= ZERO_MAX_EPA_FACES - 12) {
            break;
        }

        int p_idx = num_verts++;
        verts[p_idx] = p;

        EPAEdge edges[ZERO_MAX_EPA_FACES];
        int num_edges = 0;

        for (int i = num_faces - 1; i >= 0; --i) {
            Vec3 va = verts[faces[i].a];
            if (vec3_dot(faces[i].normal, vec3_sub(p, va)) > 0.0f) {
                int f_edges[3][2] = {
                    {faces[i].a, faces[i].b},
                    {faces[i].b, faces[i].c},
                    {faces[i].c, faces[i].a}
                };
                for (int e = 0; e < 3; ++e) {
                    int ea = f_edges[e][0];
                    int eb = f_edges[e][1];
                    bool found = false;
                    for (int k = 0; k < num_edges; ++k) {
                        if (edges[k].a == eb && edges[k].b == ea) {
                            edges[k] = edges[--num_edges];
                            found = true;
                            break;
                        }
                    }
                    if (!found && num_edges < ZERO_MAX_EPA_FACES) {
                        edges[num_edges].a = ea;
                        edges[num_edges].b = eb;
                        num_edges++;
                    }
                }
                faces[i] = faces[--num_faces];
            }
        }

        for (int k = 0; k < num_edges && num_faces < ZERO_MAX_EPA_FACES; ++k) {
            faces[num_faces].a = edges[k].a;
            faces[num_faces].b = edges[k].b;
            faces[num_faces].c = p_idx;
            Vec3 va = verts[faces[num_faces].a];
            Vec3 vb = verts[faces[num_faces].b];
            Vec3 vc = verts[faces[num_faces].c];
            Vec3 ab = vec3_sub(vb, va);
            Vec3 ac = vec3_sub(vc, va);
            Vec3 n = vec3_normalize(vec3_cross(ab, ac));
            float d = vec3_dot(n, va);
            if (d < 0.0f) {
                n = vec3_scale(n, -1.0f);
                d = -d;
                int tmp = faces[num_faces].b;
                faces[num_faces].b = faces[num_faces].c;
                faces[num_faces].c = tmp;
            }
            faces[num_faces].normal = n;
            faces[num_faces].dist = d;
            num_faces++;
        }
    }

    *out_normal = faces[best_face_idx].normal;
    *out_depth = faces[best_face_idx].dist;
    *out_contact_point = vec3_add(posA, vec3_scale(faces[best_face_idx].normal, faces[best_face_idx].dist * 0.5f));
    return true;
}

// -----------------------------------------------------------------------------
// Continuous Collision Detection (Raycast against colliders)
// -----------------------------------------------------------------------------

bool zero_physics_raycast(const ZeroPhysicsWorld *world, Vec3 origin, Vec3 dir, float max_dist,
                          float *out_hit_dist, Vec3 *out_hit_normal, uint32_t *out_hit_body_id) {
    if (!world) return false;
    dir = vec3_normalize(dir);

    float closest_t = max_dist;
    Vec3 best_n = vec3(0, 1, 0);
    uint32_t best_id = 0;
    bool hit_any = false;

    for (int i = 0; i < world->body_count; ++i) {
        const ZeroRigidBody *b = &world->bodies[i];
        if (!b->is_active) continue;

        switch (b->collider.type) {
            case ZERO_COLLIDER_SPHERE: {
                Vec3 m = vec3_sub(origin, b->position);
                float b_coeff = vec3_dot(m, dir);
                float c = vec3_dot(m, m) - b->collider.sphere.radius * b->collider.sphere.radius;
                if (c > 0.0f && b_coeff > 0.0f) continue;
                float discr = b_coeff * b_coeff - c;
                if (discr < 0.0f) continue;
                float t = -b_coeff - sqrtf(discr);
                if (t < 0.0f) t = 0.0f;
                if (t < closest_t) {
                    closest_t = t;
                    best_n = vec3_normalize(vec3_sub(vec3_add(origin, vec3_scale(dir, t)), b->position));
                    best_id = b->id;
                    hit_any = true;
                }
                break;
            }
            case ZERO_COLLIDER_BOX: {
                // Slab raycast in local oriented bounding box space
                Vec3 p = quat_unrotate_vec3(b->orientation, vec3_sub(origin, b->position));
                Vec3 d = quat_unrotate_vec3(b->orientation, dir);
                Vec3 h = b->collider.box.half_extents;

                float tmin = 0.0f;
                float tmax = closest_t;
                Vec3 n_hit = vec3(0, 1, 0);

                for (int axis = 0; axis < 3; ++axis) {
                    float p_val = (axis == 0) ? p.x : ((axis == 1) ? p.y : p.z);
                    float d_val = (axis == 0) ? d.x : ((axis == 1) ? d.y : d.z);
                    float h_val = (axis == 0) ? h.x : ((axis == 1) ? h.y : h.z);

                    if (fabsf(d_val) < 1e-6f) {
                        if (p_val < -h_val || p_val > h_val) goto next_body;
                    } else {
                        float ood = 1.0f / d_val;
                        float t1 = (-h_val - p_val) * ood;
                        float t2 = (h_val - p_val) * ood;
                        Vec3 axis_n = (axis == 0) ? vec3(-1, 0, 0) : ((axis == 1) ? vec3(0, -1, 0) : vec3(0, 0, -1));
                        if (t1 > t2) {
                            float tmp = t1; t1 = t2; t2 = tmp;
                            axis_n = vec3_scale(axis_n, -1.0f);
                        }
                        if (t1 > tmin) {
                            tmin = t1;
                            n_hit = axis_n;
                        }
                        if (t2 < tmax) tmax = t2;
                        if (tmin > tmax) goto next_body;
                    }
                }

                if (tmin < closest_t) {
                    closest_t = tmin;
                    best_n = quat_rotate_vec3(b->orientation, n_hit);
                    best_id = b->id;
                    hit_any = true;
                }
                break;
            }
            default:
                break;
        }
next_body:;
    }

    if (hit_any) {
        if (out_hit_dist) *out_hit_dist = closest_t;
        if (out_hit_normal) *out_hit_normal = best_n;
        if (out_hit_body_id) *out_hit_body_id = best_id;
        return true;
    }
    return false;
}

// -----------------------------------------------------------------------------
// Physics World Implementation
// -----------------------------------------------------------------------------

ZeroPhysicsWorld zero_physics_world_create(void) {
    ZeroPhysicsWorld world;
    memset(&world, 0, sizeof(world));
    world.broadphase = zero_aabb_tree_create(256);
    world.gravity = vec3(0.0f, -9.81f, 0.0f);
    world.velocity_iterations = 8;
    world.position_iterations = 4;
    world.allowed_penetration = 0.01f;
    world.bias_factor = 0.2f;
    return world;
}

void zero_physics_world_destroy(ZeroPhysicsWorld *world) {
    if (world) {
        zero_aabb_tree_destroy(&world->broadphase);
        world->body_count = 0;
        world->manifold_count = 0;
    }
}

ZeroRigidBody *zero_physics_add_body(ZeroPhysicsWorld *world, const ZeroRigidBody *def) {
    if (!world || world->body_count >= ZERO_MAX_RIGID_BODIES) return NULL;
    int idx = world->body_count++;
    ZeroRigidBody *b = &world->bodies[idx];
    *b = *def;
    b->id = (uint32_t)(idx + 1);
    b->is_active = true;

    if (b->is_static) {
        b->mass = 0.0f;
        b->inv_mass = 0.0f;
        b->inertia_diag = vec3(0, 0, 0);
        b->inv_inertia_diag = vec3(0, 0, 0);
    } else {
        if (b->mass <= 0.0f) b->mass = 1.0f;
        b->inv_mass = 1.0f / b->mass;
        if (vec3_length_sq(b->inertia_diag) < 1e-4f) {
            b->inertia_diag = vec3(b->mass * 0.4f, b->mass * 0.4f, b->mass * 0.4f);
        }
        b->inv_inertia_diag = vec3(1.0f / b->inertia_diag.x, 1.0f / b->inertia_diag.y, 1.0f / b->inertia_diag.z);
    }

    if (b->linear_damping < 1e-4f) b->linear_damping = 0.02f;
    if (b->angular_damping < 1e-4f) b->angular_damping = 0.05f;
    if (b->friction < 1e-4f) b->friction = 0.6f;

    // Compute bounding AABB
    Vec3 extents = vec3(1, 1, 1);
    switch (b->collider.type) {
        case ZERO_COLLIDER_SPHERE:
            extents = vec3(b->collider.sphere.radius, b->collider.sphere.radius, b->collider.sphere.radius);
            break;
        case ZERO_COLLIDER_BOX:
            extents = b->collider.box.half_extents;
            break;
        case ZERO_COLLIDER_CAPSULE:
            extents = vec3(b->collider.capsule.radius, b->collider.capsule.half_height + b->collider.capsule.radius, b->collider.capsule.radius);
            break;
    }
    b->aabb = zero_aabb_create(vec3_sub(b->position, extents), vec3_add(b->position, extents));
    b->aabb_node_id = zero_aabb_tree_insert(&world->broadphase, b->aabb, b->id);
    return b;
}

void zero_physics_step(ZeroPhysicsWorld *world, float dt) {
    if (!world || dt <= 0.0f) return;

    // 1. Semi-implicit Euler integration of forces and velocity damping
    for (int i = 0; i < world->body_count; ++i) {
        ZeroRigidBody *b = &world->bodies[i];
        if (!b->is_active || b->is_static) continue;

        Vec3 total_force = vec3_add(b->force, vec3_scale(world->gravity, b->mass));
        Vec3 accel = vec3_scale(total_force, b->inv_mass);
        b->linear_velocity = vec3_add(b->linear_velocity, vec3_scale(accel, dt));
        b->linear_velocity = vec3_scale(b->linear_velocity, powf(1.0f - b->linear_damping, dt));

        Vec3 ang_accel = vec3(
            b->torque.x * b->inv_inertia_diag.x,
            b->torque.y * b->inv_inertia_diag.y,
            b->torque.z * b->inv_inertia_diag.z
        );
        b->angular_velocity = vec3_add(b->angular_velocity, vec3_scale(ang_accel, dt));
        b->angular_velocity = vec3_scale(b->angular_velocity, powf(1.0f - b->angular_damping, dt));

        b->force = vec3(0, 0, 0);
        b->torque = vec3(0, 0, 0);
    }

    // 2. Broadphase update and narrowphase contact generation
    world->manifold_count = 0;
    for (int i = 0; i < world->body_count; ++i) {
        ZeroRigidBody *bA = &world->bodies[i];
        if (!bA->is_active) continue;

        // Update fat AABB in broadphase
        Vec3 extents = vec3(1, 1, 1);
        switch (bA->collider.type) {
            case ZERO_COLLIDER_SPHERE:
                extents = vec3(bA->collider.sphere.radius, bA->collider.sphere.radius, bA->collider.sphere.radius);
                break;
            case ZERO_COLLIDER_BOX:
                extents = bA->collider.box.half_extents;
                break;
            case ZERO_COLLIDER_CAPSULE:
                extents = vec3(bA->collider.capsule.radius, bA->collider.capsule.half_height + bA->collider.capsule.radius, bA->collider.capsule.radius);
                break;
        }
        bA->aabb = zero_aabb_create(vec3_sub(bA->position, extents), vec3_add(bA->position, extents));

        for (int j = i + 1; j < world->body_count; ++j) {
            ZeroRigidBody *bB = &world->bodies[j];
            if (!bB->is_active || (bA->is_static && bB->is_static)) continue;

            if (!zero_aabb_intersects(bA->aabb, bB->aabb)) continue;

            // GJK Narrowphase
            Vec3 simplex[4];
            int simplex_count = 0;
            if (zero_gjk_intersection(&bA->collider, bA->position, bA->orientation,
                                     &bB->collider, bB->position, bB->orientation,
                                     simplex, &simplex_count)) {
                // EPA for penetration and contact manifold
                Vec3 normal, contact_pt;
                float depth = 0.0f;
                if (zero_epa_penetration(&bA->collider, bA->position, bA->orientation,
                                         &bB->collider, bB->position, bB->orientation,
                                         simplex, simplex_count,
                                         &normal, &depth, &contact_pt)) {
                    if (world->manifold_count < ZERO_MAX_MANIFOLDS) {
                        ZeroManifold *m = &world->manifolds[world->manifold_count++];
                        m->body_a_id = bA->id;
                        m->body_b_id = bB->id;
                        m->point_count = 1;
                        m->points[0].normal = normal;
                        m->points[0].penetration = depth;
                        m->points[0].position = contact_pt;
                        m->points[0].rA = vec3_sub(contact_pt, bA->position);
                        m->points[0].rB = vec3_sub(contact_pt, bB->position);
                        m->points[0].normal_impulse = 0.0f;
                        m->points[0].tangent_impulse[0] = 0.0f;
                        m->points[0].tangent_impulse[1] = 0.0f;

                        // Calculate effective mass
                        float inv_m_sum = bA->inv_mass + bB->inv_mass;
                        m->points[0].normal_mass = (inv_m_sum > 0.0f) ? 1.0f / inv_m_sum : 0.0f;
                    }
                }
            }
        }
    }

    // 3. Sequential Impulse Solver: Velocity Iterations
    for (int iter = 0; iter < world->velocity_iterations; ++iter) {
        for (int m_idx = 0; m_idx < world->manifold_count; ++m_idx) {
            ZeroManifold *m = &world->manifolds[m_idx];
            ZeroRigidBody *bA = &world->bodies[m->body_a_id - 1];
            ZeroRigidBody *bB = &world->bodies[m->body_b_id - 1];

            for (int p_idx = 0; p_idx < m->point_count; ++p_idx) {
                ZeroContactPoint *cp = &m->points[p_idx];
                Vec3 n = cp->normal;

                Vec3 vA = vec3_add(bA->linear_velocity, vec3_cross(bA->angular_velocity, cp->rA));
                Vec3 vB = vec3_add(bB->linear_velocity, vec3_cross(bB->angular_velocity, cp->rB));
                Vec3 rel_v = vec3_sub(vB, vA);

                float normal_vel = vec3_dot(rel_v, n);
                float restitution = fmaxf(bA->restitution, bB->restitution);
                float desired_vel = -normal_vel * restitution;

                float impulse = -(normal_vel - desired_vel) * cp->normal_mass;
                float old_impulse = cp->normal_impulse;
                cp->normal_impulse = fmaxf(old_impulse + impulse, 0.0f);
                float delta_impulse = cp->normal_impulse - old_impulse;

                Vec3 p = vec3_scale(n, delta_impulse);
                if (!bA->is_static) {
                    bA->linear_velocity = vec3_sub(bA->linear_velocity, vec3_scale(p, bA->inv_mass));
                    Vec3 tA = vec3_cross(cp->rA, p);
                    bA->angular_velocity = vec3_sub(bA->angular_velocity, vec3(tA.x * bA->inv_inertia_diag.x, tA.y * bA->inv_inertia_diag.y, tA.z * bA->inv_inertia_diag.z));
                }
                if (!bB->is_static) {
                    bB->linear_velocity = vec3_add(bB->linear_velocity, vec3_scale(p, bB->inv_mass));
                    Vec3 tB = vec3_cross(cp->rB, p);
                    bB->angular_velocity = vec3_add(bB->angular_velocity, vec3(tB.x * bB->inv_inertia_diag.x, tB.y * bB->inv_inertia_diag.y, tB.z * bB->inv_inertia_diag.z));
                }
            }
        }
    }

    // 4. Integrate positions and orientations
    for (int i = 0; i < world->body_count; ++i) {
        ZeroRigidBody *b = &world->bodies[i];
        if (!b->is_active || b->is_static) continue;

        b->position = vec3_add(b->position, vec3_scale(b->linear_velocity, dt));

        // Quaternion spin integration
        Quat spin = {
            b->angular_velocity.x * 0.5f,
            b->angular_velocity.y * 0.5f,
            b->angular_velocity.z * 0.5f,
            0.0f
        };
        Quat dq = quat_mul(spin, b->orientation);
        b->orientation.x += dq.x * dt;
        b->orientation.y += dq.y * dt;
        b->orientation.z += dq.z * dt;
        b->orientation.w += dq.w * dt;
        b->orientation = quat_normalize(b->orientation);
    }

    // 5. XPBD Position correction loop (eliminates penetration sinking)
    for (int iter = 0; iter < world->position_iterations; ++iter) {
        for (int m_idx = 0; m_idx < world->manifold_count; ++m_idx) {
            ZeroManifold *m = &world->manifolds[m_idx];
            ZeroRigidBody *bA = &world->bodies[m->body_a_id - 1];
            ZeroRigidBody *bB = &world->bodies[m->body_b_id - 1];

            for (int p_idx = 0; p_idx < m->point_count; ++p_idx) {
                ZeroContactPoint *cp = &m->points[p_idx];
                float pen = cp->penetration - world->allowed_penetration;
                if (pen <= 0.0f) continue;

                float correction_mag = pen * world->bias_factor;
                Vec3 corr = vec3_scale(cp->normal, correction_mag);
                float inv_m_sum = bA->inv_mass + bB->inv_mass;
                if (inv_m_sum > 0.0f) {
                    if (!bA->is_static) bA->position = vec3_sub(bA->position, vec3_scale(corr, bA->inv_mass / inv_m_sum));
                    if (!bB->is_static) bB->position = vec3_add(bB->position, vec3_scale(corr, bB->inv_mass / inv_m_sum));
                }
            }
        }
    }
}

int zero_physics_inspect_scene(const ZeroPhysicsWorld *world, char *buffer, int buffer_size) {
    if (!world || !buffer || buffer_size <= 0) return 0;
    int offset = 0;
    offset += snprintf(buffer + offset, buffer_size - offset, "{\n  \"body_count\": %d,\n  \"manifold_count\": %d,\n  \"bodies\": [\n", world->body_count, world->manifold_count);

    for (int i = 0; i < world->body_count; ++i) {
        const ZeroRigidBody *b = &world->bodies[i];
        offset += snprintf(buffer + offset, buffer_size - offset,
            "    {\"id\": %u, \"static\": %s, \"pos\": [%.2f, %.2f, %.2f], \"vel\": [%.2f, %.2f, %.2f]}%s\n",
            b->id, b->is_static ? "true" : "false",
            b->position.x, b->position.y, b->position.z,
            b->linear_velocity.x, b->linear_velocity.y, b->linear_velocity.z,
            (i + 1 < world->body_count) ? "," : "");
    }
    offset += snprintf(buffer + offset, buffer_size - offset, "  ]\n}\n");
    return offset;
}

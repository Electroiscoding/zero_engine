#include "zero_track_mesh.h"
#include "zero_track_spline.h"
#include "zero_gl_loader.h"
#include <math.h>
#include <string.h>

static void add_box(ZeroVertex *vertices, uint32_t *indices,
                    uint32_t *v_count, uint32_t *i_count,
                    Vec3 min_p, Vec3 max_p) {
    uint32_t base_v = *v_count;
    uint32_t base_i = *i_count;

    static const Vec3 face_normals[6] = {
        { 0.0f,  0.0f,  1.0f}, /* +Z Front */
        { 0.0f,  0.0f, -1.0f}, /* -Z Back */
        {-1.0f,  0.0f,  0.0f}, /* -X Left */
        { 1.0f,  0.0f,  0.0f}, /* +X Right */
        { 0.0f,  1.0f,  0.0f}, /* +Y Top */
        { 0.0f, -1.0f,  0.0f}  /* -Y Bottom */
    };

    float x0 = min_p.x, y0 = min_p.y, z0 = min_p.z;
    float x1 = max_p.x, y1 = max_p.y, z1 = max_p.z;

    float corners[6][4][3] = {
        {{x0, y0, z1}, {x1, y0, z1}, {x1, y1, z1}, {x0, y1, z1}}, /* +Z */
        {{x1, y0, z0}, {x0, y0, z0}, {x0, y1, z0}, {x1, y1, z0}}, /* -Z */
        {{x0, y0, z0}, {x0, y0, z1}, {x0, y1, z1}, {x0, y1, z0}}, /* -X */
        {{x1, y0, z1}, {x1, y0, z0}, {x1, y1, z0}, {x1, y1, z1}}, /* +X */
        {{x0, y1, z1}, {x1, y1, z1}, {x1, y1, z0}, {x0, y1, z0}}, /* +Y */
        {{x0, y0, z0}, {x1, y0, z0}, {x1, y0, z1}, {x0, y0, z1}}  /* -Y */
    };

    static const Vec2 uvs[4] = {
        {0.0f, 0.0f}, {1.0f, 0.0f}, {1.0f, 1.0f}, {0.0f, 1.0f}
    };

    for (int face = 0; face < 6; ++face) {
        uint32_t face_base = base_v + (uint32_t)(face * 4);
        for (int v = 0; v < 4; ++v) {
            vertices[face_base + v].pos = vec3(corners[face][v][0], corners[face][v][1], corners[face][v][2]);
            vertices[face_base + v].normal = face_normals[face];
            vertices[face_base + v].uv = uvs[v];
        }

        indices[base_i + face * 6 + 0] = face_base + 0;
        indices[base_i + face * 6 + 1] = face_base + 1;
        indices[base_i + face * 6 + 2] = face_base + 2;
        indices[base_i + face * 6 + 3] = face_base + 0;
        indices[base_i + face * 6 + 4] = face_base + 2;
        indices[base_i + face * 6 + 5] = face_base + 3;
    }

    *v_count += 24;
    *i_count += 36;
}

ZeroTrackMeshes zero_track_meshes_generate(ZeroArena *arena) {
    ZeroTrackMeshes meshes;
    memset(&meshes, 0, sizeof(meshes));

    bool created_local_arena = false;
    ZeroArena local_arena;
    if (!arena) {
        local_arena = zero_arena_create(1024 * 1024);
        arena = &local_arena;
        created_local_arena = true;
    }

    const int N = ZERO_TRACK_SPLINE_COUNT;
    const int rings = N + 1;

    /* 1. Calculate cumulative arc length and node tangents/normals */
    float cum_dist[ZERO_TRACK_SPLINE_COUNT + 1];
    cum_dist[0] = 0.0f;
    for (int i = 0; i < N; ++i) {
        int next = (i + 1) % N;
        float dx = g_track_spline[next][0] - g_track_spline[i][0];
        float dz = g_track_spline[next][1] - g_track_spline[i][1];
        float seg_len = sqrtf(dx * dx + dz * dz);
        cum_dist[i + 1] = cum_dist[i] + seg_len;
    }

    float normals[ZERO_TRACK_SPLINE_COUNT][2];
    for (int i = 0; i < N; ++i) {
        int next = (i + 1) % N;
        int prev = (i - 1 + N) % N;
        float tx = g_track_spline[next][0] - g_track_spline[prev][0];
        float tz = g_track_spline[next][1] - g_track_spline[prev][1];
        float t_len = sqrtf(tx * tx + tz * tz);
        if (t_len > 1e-6f) {
            tx /= t_len;
            tz /= t_len;
        }
        normals[i][0] = -tz; /* Left normal */
        normals[i][1] = tx;
    }

    /* 2. Generate Road Mesh (Asphalt extrusion, W = 16.0m) */
    {
        ZeroMeshData road_data;
        road_data.vertex_count = (uint32_t)(rings * 2);
        road_data.index_count = (uint32_t)(N * 6);
        road_data.vertices = (ZeroVertex *)zero_arena_alloc(arena, road_data.vertex_count * sizeof(ZeroVertex), 64);
        road_data.indices = (uint32_t *)zero_arena_alloc(arena, road_data.index_count * sizeof(uint32_t), 64);

        if (road_data.vertices && road_data.indices) {
            const float half_w = ZERO_TRACK_WIDTH * 0.5f; /* 8.0m */
            for (int r = 0; r < rings; ++r) {
                int node = r % N;
                float px = g_track_spline[node][0];
                float pz = g_track_spline[node][1];
                float nx = normals[node][0];
                float nz = normals[node][1];
                float s = cum_dist[r];

                uint32_t v_idx = (uint32_t)(r * 2);
                /* Left vertex (UV.x = 0.0) */
                road_data.vertices[v_idx + 0].pos = vec3(px + nx * half_w, 0.02f, pz + nz * half_w);
                road_data.vertices[v_idx + 0].normal = vec3(0.0f, 1.0f, 0.0f);
                road_data.vertices[v_idx + 0].uv = vec2(0.0f, s / 4.0f);

                /* Right vertex (UV.x = 1.0) */
                road_data.vertices[v_idx + 1].pos = vec3(px - nx * half_w, 0.02f, pz - nz * half_w);
                road_data.vertices[v_idx + 1].normal = vec3(0.0f, 1.0f, 0.0f);
                road_data.vertices[v_idx + 1].uv = vec2(1.0f, s / 4.0f);
            }

            for (int i = 0; i < N; ++i) {
                uint32_t v0 = (uint32_t)(i * 2 + 0);
                uint32_t v1 = (uint32_t)(i * 2 + 1);
                uint32_t v2 = (uint32_t)((i + 1) * 2 + 0);
                uint32_t v3 = (uint32_t)((i + 1) * 2 + 1);
                uint32_t i_base = (uint32_t)(i * 6);

                road_data.indices[i_base + 0] = v0;
                road_data.indices[i_base + 1] = v2;
                road_data.indices[i_base + 2] = v1;
                road_data.indices[i_base + 3] = v1;
                road_data.indices[i_base + 4] = v2;
                road_data.indices[i_base + 5] = v3;
            }

            meshes.road_mesh = zero_mesh_upload(&road_data);
        }
    }

    /* 3. Generate Curbs Mesh (Alternating red/white rumble strips, W = 1.6m, h = 0.08m) */
    {
        /* Each ring has:
           Left curb: 3 vertices (inner edge, outer top, outer drop)
           Right curb: 3 vertices (inner edge, outer top, outer drop)
           Total 6 vertices per ring */
        ZeroMeshData curbs_data;
        curbs_data.vertex_count = (uint32_t)(rings * 6);
        curbs_data.index_count = (uint32_t)(N * 24); /* 4 quads * 6 = 24 indices per segment */
        curbs_data.vertices = (ZeroVertex *)zero_arena_alloc(arena, curbs_data.vertex_count * sizeof(ZeroVertex), 64);
        curbs_data.indices = (uint32_t *)zero_arena_alloc(arena, curbs_data.index_count * sizeof(uint32_t), 64);

        if (curbs_data.vertices && curbs_data.indices) {
            const float half_w = ZERO_TRACK_WIDTH * 0.5f;
            const float curb_w = 1.6f;
            const float curb_h = 0.08f;

            for (int r = 0; r < rings; ++r) {
                int node = r % N;
                float px = g_track_spline[node][0];
                float pz = g_track_spline[node][1];
                float nx = normals[node][0];
                float nz = normals[node][1];
                float s = cum_dist[r];

                uint32_t base_v = (uint32_t)(r * 6);

                /* Left Curb Top surface normal: angled slightly inwards */
                Vec3 l_top_n = vec3_normalize(vec3(-nx * 0.06f, 1.6f, -nz * 0.06f));
                /* Left Curb Lip normal: outward */
                Vec3 l_lip_n = vec3(nx, 0.0f, nz);

                /* 0: Left curb inner */
                curbs_data.vertices[base_v + 0].pos = vec3(px + nx * half_w, 0.02f, pz + nz * half_w);
                curbs_data.vertices[base_v + 0].normal = l_top_n;
                curbs_data.vertices[base_v + 0].uv = vec2(0.0f, s / 2.0f);

                /* 1: Left curb outer top */
                curbs_data.vertices[base_v + 1].pos = vec3(px + nx * (half_w + curb_w), curb_h, pz + nz * (half_w + curb_w));
                curbs_data.vertices[base_v + 1].normal = l_top_n;
                curbs_data.vertices[base_v + 1].uv = vec2(1.0f, s / 2.0f);

                /* 2: Left curb outer bottom lip */
                curbs_data.vertices[base_v + 2].pos = vec3(px + nx * (half_w + curb_w), 0.00f, pz + nz * (half_w + curb_w));
                curbs_data.vertices[base_v + 2].normal = l_lip_n;
                curbs_data.vertices[base_v + 2].uv = vec2(1.0f, s / 2.0f);

                /* Right Curb Top surface normal */
                Vec3 r_top_n = vec3_normalize(vec3(nx * 0.06f, 1.6f, nz * 0.06f));
                Vec3 r_lip_n = vec3(-nx, 0.0f, -nz);

                /* 3: Right curb inner */
                curbs_data.vertices[base_v + 3].pos = vec3(px - nx * half_w, 0.02f, pz - nz * half_w);
                curbs_data.vertices[base_v + 3].normal = r_top_n;
                curbs_data.vertices[base_v + 3].uv = vec2(0.0f, s / 2.0f);

                /* 4: Right curb outer top */
                curbs_data.vertices[base_v + 4].pos = vec3(px - nx * (half_w + curb_w), curb_h, pz - nz * (half_w + curb_w));
                curbs_data.vertices[base_v + 4].normal = r_top_n;
                curbs_data.vertices[base_v + 4].uv = vec2(1.0f, s / 2.0f);

                /* 5: Right curb outer bottom lip */
                curbs_data.vertices[base_v + 5].pos = vec3(px - nx * (half_w + curb_w), 0.00f, pz - nz * (half_w + curb_w));
                curbs_data.vertices[base_v + 5].normal = r_lip_n;
                curbs_data.vertices[base_v + 5].uv = vec2(1.0f, s / 2.0f);
            }

            for (int i = 0; i < N; ++i) {
                uint32_t r0 = (uint32_t)(i * 6);
                uint32_t r1 = (uint32_t)((i + 1) * 6);
                uint32_t i_base = (uint32_t)(i * 24);

                /* Left curb top quad: r0+0, r1+0, r0+1, r1+1 */
                curbs_data.indices[i_base + 0] = r0 + 0;
                curbs_data.indices[i_base + 1] = r1 + 0;
                curbs_data.indices[i_base + 2] = r0 + 1;
                curbs_data.indices[i_base + 3] = r0 + 1;
                curbs_data.indices[i_base + 4] = r1 + 0;
                curbs_data.indices[i_base + 5] = r1 + 1;

                /* Left curb lip quad: r0+1, r1+1, r0+2, r1+2 */
                curbs_data.indices[i_base + 6]  = r0 + 1;
                curbs_data.indices[i_base + 7]  = r1 + 1;
                curbs_data.indices[i_base + 8]  = r0 + 2;
                curbs_data.indices[i_base + 9]  = r0 + 2;
                curbs_data.indices[i_base + 10] = r1 + 1;
                curbs_data.indices[i_base + 11] = r1 + 2;

                /* Right curb top quad: r0+4, r1+4, r0+3, r1+3 */
                curbs_data.indices[i_base + 12] = r0 + 4;
                curbs_data.indices[i_base + 13] = r1 + 4;
                curbs_data.indices[i_base + 14] = r0 + 3;
                curbs_data.indices[i_base + 15] = r0 + 3;
                curbs_data.indices[i_base + 16] = r1 + 4;
                curbs_data.indices[i_base + 17] = r1 + 3;

                /* Right curb lip quad: r0+5, r1+5, r0+4, r1+4 */
                curbs_data.indices[i_base + 18] = r0 + 5;
                curbs_data.indices[i_base + 19] = r1 + 5;
                curbs_data.indices[i_base + 20] = r0 + 4;
                curbs_data.indices[i_base + 21] = r0 + 4;
                curbs_data.indices[i_base + 22] = r1 + 5;
                curbs_data.indices[i_base + 23] = r1 + 4;
            }

            meshes.curbs_mesh = zero_mesh_upload(&curbs_data);
        }
    }

    /* 4. Generate Barriers Mesh (Armco corrugated barriers, D = 11.0m, h = 1.0m) */
    {
        /* Each ring has 3 vertices for left barrier and 3 for right barrier = 6 vertices */
        ZeroMeshData barrier_data;
        barrier_data.vertex_count = (uint32_t)(rings * 6);
        barrier_data.index_count = (uint32_t)(N * 24); /* 4 quads * 6 = 24 indices */
        barrier_data.vertices = (ZeroVertex *)zero_arena_alloc(arena, barrier_data.vertex_count * sizeof(ZeroVertex), 64);
        barrier_data.indices = (uint32_t *)zero_arena_alloc(arena, barrier_data.index_count * sizeof(uint32_t), 64);

        if (barrier_data.vertices && barrier_data.indices) {
            const float bar_d = ZERO_TRACK_BARRIER_DIST; /* 11.0m */
            const float y_bot = 0.15f;
            const float y_mid = 0.60f;
            const float y_top = 1.05f;
            const float crease = 0.08f; /* Protrusion toward road */

            for (int r = 0; r < rings; ++r) {
                int node = r % N;
                float px = g_track_spline[node][0];
                float pz = g_track_spline[node][1];
                float nx = normals[node][0];
                float nz = normals[node][1];
                float s = cum_dist[r];

                uint32_t base_v = (uint32_t)(r * 6);

                /* Left barrier: faces toward -N (inward to road) */
                Vec3 l_norm = vec3(-nx, 0.0f, -nz);
                barrier_data.vertices[base_v + 0].pos = vec3(px + nx * bar_d, y_bot, pz + nz * bar_d);
                barrier_data.vertices[base_v + 0].normal = l_norm;
                barrier_data.vertices[base_v + 0].uv = vec2(0.0f, s / 2.0f);

                barrier_data.vertices[base_v + 1].pos = vec3(px + nx * (bar_d - crease), y_mid, pz + nz * (bar_d - crease));
                barrier_data.vertices[base_v + 1].normal = l_norm;
                barrier_data.vertices[base_v + 1].uv = vec2(0.5f, s / 2.0f);

                barrier_data.vertices[base_v + 2].pos = vec3(px + nx * bar_d, y_top, pz + nz * bar_d);
                barrier_data.vertices[base_v + 2].normal = l_norm;
                barrier_data.vertices[base_v + 2].uv = vec2(1.0f, s / 2.0f);

                /* Right barrier: faces toward +N (inward to road) */
                Vec3 r_norm = vec3(nx, 0.0f, nz);
                barrier_data.vertices[base_v + 3].pos = vec3(px - nx * bar_d, y_bot, pz - nz * bar_d);
                barrier_data.vertices[base_v + 3].normal = r_norm;
                barrier_data.vertices[base_v + 3].uv = vec2(0.0f, s / 2.0f);

                barrier_data.vertices[base_v + 4].pos = vec3(px - nx * (bar_d - crease), y_mid, pz - nz * (bar_d - crease));
                barrier_data.vertices[base_v + 4].normal = r_norm;
                barrier_data.vertices[base_v + 4].uv = vec2(0.5f, s / 2.0f);

                barrier_data.vertices[base_v + 5].pos = vec3(px - nx * bar_d, y_top, pz - nz * bar_d);
                barrier_data.vertices[base_v + 5].normal = r_norm;
                barrier_data.vertices[base_v + 5].uv = vec2(1.0f, s / 2.0f);
            }

            for (int i = 0; i < N; ++i) {
                uint32_t r0 = (uint32_t)(i * 6);
                uint32_t r1 = (uint32_t)((i + 1) * 6);
                uint32_t i_base = (uint32_t)(i * 24);

                /* Left barrier bottom panel (r0+0 -> r0+1) */
                barrier_data.indices[i_base + 0] = r0 + 0;
                barrier_data.indices[i_base + 1] = r0 + 1;
                barrier_data.indices[i_base + 2] = r1 + 0;
                barrier_data.indices[i_base + 3] = r1 + 0;
                barrier_data.indices[i_base + 4] = r0 + 1;
                barrier_data.indices[i_base + 5] = r1 + 1;

                /* Left barrier top panel (r0+1 -> r0+2) */
                barrier_data.indices[i_base + 6]  = r0 + 1;
                barrier_data.indices[i_base + 7]  = r0 + 2;
                barrier_data.indices[i_base + 8]  = r1 + 1;
                barrier_data.indices[i_base + 9]  = r1 + 1;
                barrier_data.indices[i_base + 10] = r0 + 2;
                barrier_data.indices[i_base + 11] = r1 + 2;

                /* Right barrier bottom panel (r0+3 -> r0+4) */
                barrier_data.indices[i_base + 12] = r0 + 3;
                barrier_data.indices[i_base + 13] = r1 + 3;
                barrier_data.indices[i_base + 14] = r0 + 4;
                barrier_data.indices[i_base + 15] = r0 + 4;
                barrier_data.indices[i_base + 16] = r1 + 3;
                barrier_data.indices[i_base + 17] = r1 + 4;

                /* Right barrier top panel (r0+4 -> r0+5) */
                barrier_data.indices[i_base + 18] = r0 + 4;
                barrier_data.indices[i_base + 19] = r1 + 4;
                barrier_data.indices[i_base + 20] = r0 + 5;
                barrier_data.indices[i_base + 21] = r0 + 5;
                barrier_data.indices[i_base + 22] = r1 + 4;
                barrier_data.indices[i_base + 23] = r1 + 5;
            }

            meshes.barriers_mesh = zero_mesh_upload(&barrier_data);
        }
    }

    /* 5. Generate Start/Finish Overhead Gantry (near x ~ 145-164) */
    {
        /* 3 cuboids: Left Pillar, Right Pillar, Overhead Crossbar = 3 * 24 = 72 verts, 3 * 36 = 108 indices */
        ZeroMeshData gantry_data;
        gantry_data.vertex_count = 72;
        gantry_data.index_count = 108;
        gantry_data.vertices = (ZeroVertex *)zero_arena_alloc(arena, gantry_data.vertex_count * sizeof(ZeroVertex), 64);
        gantry_data.indices = (uint32_t *)zero_arena_alloc(arena, gantry_data.index_count * sizeof(uint32_t), 64);

        if (gantry_data.vertices && gantry_data.indices) {
            uint32_t v_curr = 0;
            uint32_t i_curr = 0;

            /* Left Pillar: x in [145.35, 146.35], y in [0.0, 6.5], z in [-4.1, -3.1] */
            add_box(gantry_data.vertices, gantry_data.indices, &v_curr, &i_curr,
                    vec3(145.35f, 0.0f, -4.1f), vec3(146.35f, 6.5f, -3.1f));

            /* Right Pillar: x in [163.34, 164.34], y in [0.0, 6.5], z in [-4.8, -3.8] */
            add_box(gantry_data.vertices, gantry_data.indices, &v_curr, &i_curr,
                    vec3(163.34f, 0.0f, -4.8f), vec3(164.34f, 6.5f, -3.8f));

            /* Overhead Crossbar: x in [145.35, 164.34], y in [5.2, 6.5], z in [-4.3, -3.6] */
            add_box(gantry_data.vertices, gantry_data.indices, &v_curr, &i_curr,
                    vec3(145.35f, 5.2f, -4.3f), vec3(164.34f, 6.5f, -3.6f));

            meshes.gantry_mesh = zero_mesh_upload(&gantry_data);
        }
    }

    if (created_local_arena) {
        zero_arena_destroy(&local_arena);
    }

    meshes.is_initialized = true;
    return meshes;
}

void zero_track_meshes_draw(const ZeroTrackMeshes *meshes, ZeroShader *pbr_shader) {
    if (!meshes || !meshes->is_initialized) {
        return;
    }
    if (!zgl.glBindVertexArray) {
        return;
    }

    if (pbr_shader && pbr_shader->is_valid) {
        zero_shader_bind(pbr_shader);
        zero_shader_set_mat4(pbr_shader, "u_model", mat4_identity());
    }

    zero_mesh_draw(&meshes->road_mesh);
    zero_mesh_draw(&meshes->curbs_mesh);
    zero_mesh_draw(&meshes->barriers_mesh);
    zero_mesh_draw(&meshes->gantry_mesh);
}

void zero_track_meshes_draw_shadow(const ZeroTrackMeshes *meshes, ZeroShader *shadow_shader) {
    if (!meshes || !meshes->is_initialized) {
        return;
    }
    if (!zgl.glBindVertexArray) {
        return;
    }

    if (shadow_shader && shadow_shader->is_valid) {
        zero_shader_bind(shadow_shader);
        zero_shader_set_mat4(shadow_shader, "u_model", mat4_identity());
    }

    zero_mesh_draw(&meshes->road_mesh);
    zero_mesh_draw(&meshes->curbs_mesh);
    zero_mesh_draw(&meshes->barriers_mesh);
    zero_mesh_draw(&meshes->gantry_mesh);
}

void zero_track_meshes_destroy(ZeroTrackMeshes *meshes) {
    if (!meshes || !meshes->is_initialized) {
        return;
    }
    zero_mesh_destroy(&meshes->road_mesh);
    zero_mesh_destroy(&meshes->curbs_mesh);
    zero_mesh_destroy(&meshes->barriers_mesh);
    zero_mesh_destroy(&meshes->gantry_mesh);
    meshes->is_initialized = false;
}

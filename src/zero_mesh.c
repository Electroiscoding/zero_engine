#include "zero_mesh.h"
#include "zero_gl_loader.h"
#include <math.h>
#include <string.h>

ZeroMesh zero_mesh_upload(const ZeroMeshData *data) {
    ZeroMesh mesh = {0, 0, 0, 0, 0, false};
    if (!data || data->vertex_count == 0 || !data->vertices) {
        return mesh;
    }

    if (!zgl.glGenVertexArrays || !zgl.glGenBuffers) {
        mesh.vertex_count = data->vertex_count;
        mesh.index_count = data->index_count;
        mesh.is_valid = true;
        return mesh;
    }

    zgl.glGenVertexArrays(1, &mesh.vao);
    zgl.glBindVertexArray(mesh.vao);

    zgl.glGenBuffers(1, &mesh.vbo);
    zgl.glBindBuffer(GL_ARRAY_BUFFER, mesh.vbo);
    zgl.glBufferData(GL_ARRAY_BUFFER, (GLsizeiptr)(data->vertex_count * sizeof(ZeroVertex)), data->vertices, GL_STATIC_DRAW);

    if (data->index_count > 0 && data->indices) {
        zgl.glGenBuffers(1, &mesh.ebo);
        zgl.glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mesh.ebo);
        zgl.glBufferData(GL_ELEMENT_ARRAY_BUFFER, (GLsizeiptr)(data->index_count * sizeof(uint32_t)), data->indices, GL_STATIC_DRAW);
    }

    zgl.glEnableVertexAttribArray(0);
    zgl.glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, (GLsizei)sizeof(ZeroVertex), (const void *)offsetof(ZeroVertex, pos));

    zgl.glEnableVertexAttribArray(1);
    zgl.glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, (GLsizei)sizeof(ZeroVertex), (const void *)offsetof(ZeroVertex, normal));

    zgl.glEnableVertexAttribArray(2);
    zgl.glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, (GLsizei)sizeof(ZeroVertex), (const void *)offsetof(ZeroVertex, uv));

    zgl.glBindVertexArray(0);

    mesh.vertex_count = data->vertex_count;
    mesh.index_count = data->index_count;
    mesh.is_valid = true;
    return mesh;
}

void zero_mesh_destroy(ZeroMesh *mesh) {
    if (!mesh || !mesh->is_valid) {
        return;
    }

    if (zgl.glDeleteVertexArrays && mesh->vao) {
        zgl.glDeleteVertexArrays(1, &mesh->vao);
        mesh->vao = 0;
    }
    if (zgl.glDeleteBuffers) {
        if (mesh->vbo) {
            zgl.glDeleteBuffers(1, &mesh->vbo);
            mesh->vbo = 0;
        }
        if (mesh->ebo) {
            zgl.glDeleteBuffers(1, &mesh->ebo);
            mesh->ebo = 0;
        }
    }
    mesh->vertex_count = 0;
    mesh->index_count = 0;
    mesh->is_valid = false;
}

void zero_mesh_draw(const ZeroMesh *mesh) {
    if (!mesh || !mesh->is_valid) {
        return;
    }

    if (zgl.glBindVertexArray && mesh->vao) {
        zgl.glBindVertexArray(mesh->vao);
        if (mesh->index_count > 0) {
            glDrawElements(GL_TRIANGLES, (GLsizei)mesh->index_count, GL_UNSIGNED_INT, NULL);
        } else {
            glDrawArrays(GL_TRIANGLES, 0, (GLsizei)mesh->vertex_count);
        }
        zgl.glBindVertexArray(0);
    }
}

ZeroMeshData zero_mesh_gen_cube(ZeroArena *arena, float size) {
    ZeroMeshData data = {0};
    float h = size * 0.5f;

    static const Vec3 normals[6] = {
        { 0.0f,  0.0f,  1.0f},
        { 0.0f,  0.0f, -1.0f},
        {-1.0f,  0.0f,  0.0f},
        { 1.0f,  0.0f,  0.0f},
        { 0.0f,  1.0f,  0.0f},
        { 0.0f, -1.0f,  0.0f}
    };

    static const float corners[6][4][3] = {
        {{-1, -1,  1}, { 1, -1,  1}, { 1,  1,  1}, {-1,  1,  1}},
        {{ 1, -1, -1}, {-1, -1, -1}, {-1,  1, -1}, { 1,  1, -1}},
        {{-1, -1, -1}, {-1, -1,  1}, {-1,  1,  1}, {-1,  1, -1}},
        {{ 1, -1,  1}, { 1, -1, -1}, { 1,  1, -1}, { 1,  1,  1}},
        {{-1,  1,  1}, { 1,  1,  1}, { 1,  1, -1}, {-1,  1, -1}},
        {{-1, -1, -1}, { 1, -1, -1}, { 1, -1,  1}, {-1, -1,  1}}
    };

    static const Vec2 uvs[4] = {
        {0.0f, 0.0f}, {1.0f, 0.0f}, {1.0f, 1.0f}, {0.0f, 1.0f}
    };

    data.vertex_count = 24;
    data.index_count = 36;
    data.vertices = (ZeroVertex *)zero_arena_alloc(arena, data.vertex_count * sizeof(ZeroVertex), 64);
    data.indices = (uint32_t *)zero_arena_alloc(arena, data.index_count * sizeof(uint32_t), 64);

    if (!data.vertices || !data.indices) {
        return (ZeroMeshData){0};
    }

    uint32_t v_idx = 0;
    uint32_t i_idx = 0;

    for (int face = 0; face < 6; ++face) {
        uint32_t base = v_idx;
        for (int v = 0; v < 4; ++v) {
            data.vertices[v_idx].pos = vec3(corners[face][v][0] * h, corners[face][v][1] * h, corners[face][v][2] * h);
            data.vertices[v_idx].normal = normals[face];
            data.vertices[v_idx].uv = uvs[v];
            v_idx++;
        }

        data.indices[i_idx++] = base + 0;
        data.indices[i_idx++] = base + 1;
        data.indices[i_idx++] = base + 2;
        data.indices[i_idx++] = base + 2;
        data.indices[i_idx++] = base + 3;
        data.indices[i_idx++] = base + 0;
    }

    return data;
}

ZeroMeshData zero_mesh_gen_plane(ZeroArena *arena, float width, float depth) {
    ZeroMeshData data = {0};
    float hw = width * 0.5f;
    float hd = depth * 0.5f;

    data.vertex_count = 4;
    data.index_count = 6;
    data.vertices = (ZeroVertex *)zero_arena_alloc(arena, data.vertex_count * sizeof(ZeroVertex), 64);
    data.indices = (uint32_t *)zero_arena_alloc(arena, data.index_count * sizeof(uint32_t), 64);

    if (!data.vertices || !data.indices) {
        return (ZeroMeshData){0};
    }

    Vec3 normal = vec3(0.0f, 1.0f, 0.0f);
    data.vertices[0] = (ZeroVertex){vec3(-hw, 0.0f, -hd), normal, vec2(0.0f, 0.0f)};
    data.vertices[1] = (ZeroVertex){vec3( hw, 0.0f, -hd), normal, vec2(1.0f, 0.0f)};
    data.vertices[2] = (ZeroVertex){vec3( hw, 0.0f,  hd), normal, vec2(1.0f, 1.0f)};
    data.vertices[3] = (ZeroVertex){vec3(-hw, 0.0f,  hd), normal, vec2(0.0f, 1.0f)};

    data.indices[0] = 0; data.indices[1] = 2; data.indices[2] = 1;
    data.indices[3] = 0; data.indices[4] = 3; data.indices[5] = 2;

    return data;
}

ZeroMeshData zero_mesh_gen_sphere(ZeroArena *arena, float radius, uint32_t rings, uint32_t sectors) {
    ZeroMeshData data = {0};
    if (rings < 2 || sectors < 3) {
        return data;
    }

    data.vertex_count = (rings + 1) * (sectors + 1);
    data.index_count = rings * sectors * 6;
    data.vertices = (ZeroVertex *)zero_arena_alloc(arena, data.vertex_count * sizeof(ZeroVertex), 64);
    data.indices = (uint32_t *)zero_arena_alloc(arena, data.index_count * sizeof(uint32_t), 64);

    if (!data.vertices || !data.indices) {
        return (ZeroMeshData){0};
    }

    uint32_t v_idx = 0;
    for (uint32_t r = 0; r <= rings; ++r) {
        float phi = ZERO_PI * (float)r / (float)rings;
        float y = cosf(phi);
        float sin_phi = (r == rings) ? 0.0f : sinf(phi);

        for (uint32_t s = 0; s <= sectors; ++s) {
            float theta = 2.0f * ZERO_PI * (float)s / (float)sectors;
            float x = sin_phi * cosf(theta);
            float z = sin_phi * sinf(theta);

            data.vertices[v_idx].pos = vec3(x * radius, y * radius, z * radius);
            data.vertices[v_idx].normal = vec3(x, y, z);
            data.vertices[v_idx].uv = vec2((float)s / (float)sectors, (float)r / (float)rings);
            v_idx++;
        }
    }

    uint32_t i_idx = 0;
    for (uint32_t r = 0; r < rings; ++r) {
        for (uint32_t s = 0; s < sectors; ++s) {
            uint32_t first = r * (sectors + 1) + s;
            uint32_t second = first + sectors + 1;

            data.indices[i_idx++] = first;
            data.indices[i_idx++] = first + 1;
            data.indices[i_idx++] = second;

            data.indices[i_idx++] = second;
            data.indices[i_idx++] = first + 1;
            data.indices[i_idx++] = second + 1;
        }
    }

    return data;
}

#ifndef ZERO_MESH_H
#define ZERO_MESH_H

#include "zero_core.h"
#include "zero_math.h"
#include "zero_memory.h"
#include "zero_gl_defs.h"

typedef struct ZeroVertex {
    Vec3 pos;
    Vec3 normal;
    Vec2 uv;
} ZeroVertex;

typedef struct ZeroMeshData {
    ZeroVertex *vertices;
    uint32_t vertex_count;
    uint32_t *indices;
    uint32_t index_count;
} ZeroMeshData;

typedef struct ZeroMesh {
    GLuint vao;
    GLuint vbo;
    GLuint ebo;
    uint32_t vertex_count;
    uint32_t index_count;
    bool is_valid;
} ZeroMesh;

ZeroMesh zero_mesh_upload(const ZeroMeshData *data);
void zero_mesh_destroy(ZeroMesh *mesh);
void zero_mesh_draw(const ZeroMesh *mesh);

ZeroMeshData zero_mesh_gen_cube(ZeroArena *arena, float size);
ZeroMeshData zero_mesh_gen_plane(ZeroArena *arena, float width, float depth);
ZeroMeshData zero_mesh_gen_sphere(ZeroArena *arena, float radius, uint32_t rings, uint32_t sectors);

#endif

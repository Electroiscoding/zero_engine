#ifndef ZERO_ZMESH_H
#define ZERO_ZMESH_H

#include "zero_core.h"
#include "zero_mesh.h"
#include "zero_memory.h"

#define ZERO_ZMESH_MAGIC 0x48534D5A /* "ZMSH" */
#define ZERO_ZMESH_VERSION 1

typedef struct ZERO_ALIGNAS(64) ZeroZmeshHeader {
    uint32_t magic;
    uint32_t version;
    uint32_t vertex_count;
    uint32_t index_count;
    uint32_t vertex_data_offset;
    uint32_t index_data_offset;
    Vec3 aabb_min;
    Vec3 aabb_max;
    Vec3 sphere_center;
    float sphere_radius;
    uint8_t reserved[8];
} ZeroZmeshHeader;

typedef struct ZeroZmeshMap {
    void *mmap_ptr;
    size_t file_size;
    int fd;
    const ZeroZmeshHeader *header;
    const ZeroVertex *vertices;
    const uint32_t *indices;
    bool is_valid;
} ZeroZmeshMap;

bool zero_zmesh_save_file(const char *filepath, const ZeroMeshData *mesh);
ZeroMeshData zero_zmesh_load_file(const char *filepath, ZeroArena *arena);

ZeroZmeshMap zero_zmesh_mmap_open(const char *filepath);
void zero_zmesh_mmap_close(ZeroZmeshMap *map);

#endif

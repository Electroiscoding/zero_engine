#define _GNU_SOURCE
#include "zero_zmesh.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <float.h>

static void compute_mesh_bounds(const ZeroVertex *verts, uint32_t count, Vec3 *out_min, Vec3 *out_max, Vec3 *out_center, float *out_radius) {
    Vec3 bmin = vec3(FLT_MAX, FLT_MAX, FLT_MAX);
    Vec3 bmax = vec3(-FLT_MAX, -FLT_MAX, -FLT_MAX);

    for (uint32_t i = 0; i < count; ++i) {
        Vec3 p = verts[i].pos;
        if (p.x < bmin.x) bmin.x = p.x;
        if (p.y < bmin.y) bmin.y = p.y;
        if (p.z < bmin.z) bmin.z = p.z;

        if (p.x > bmax.x) bmax.x = p.x;
        if (p.y > bmax.y) bmax.y = p.y;
        if (p.z > bmax.z) bmax.z = p.z;
    }

    Vec3 center = vec3_scale(vec3_add(bmin, bmax), 0.5f);
    float max_dist_sq = 0.0f;

    for (uint32_t i = 0; i < count; ++i) {
        Vec3 diff = vec3_sub(verts[i].pos, center);
        float dist_sq = vec3_dot(diff, diff);
        if (dist_sq > max_dist_sq) {
            max_dist_sq = dist_sq;
        }
    }

    *out_min = bmin;
    *out_max = bmax;
    *out_center = center;
    *out_radius = sqrtf(max_dist_sq);
}

bool zero_zmesh_save_file(const char *filepath, const ZeroMeshData *mesh) {
    if (!filepath || !mesh || mesh->vertex_count == 0 || !mesh->vertices) {
        return false;
    }

    FILE *f = fopen(filepath, "wb");
    if (!f) return false;

    ZeroZmeshHeader hdr;
    memset(&hdr, 0, sizeof(hdr));
    hdr.magic = ZERO_ZMESH_MAGIC;
    hdr.version = ZERO_ZMESH_VERSION;
    hdr.vertex_count = mesh->vertex_count;
    hdr.index_count = mesh->index_count;
    hdr.vertex_data_offset = (uint32_t)sizeof(ZeroZmeshHeader);
    hdr.index_data_offset = hdr.vertex_data_offset + (uint32_t)(mesh->vertex_count * sizeof(ZeroVertex));

    compute_mesh_bounds(mesh->vertices, mesh->vertex_count, &hdr.aabb_min, &hdr.aabb_max, &hdr.sphere_center, &hdr.sphere_radius);

    if (fwrite(&hdr, sizeof(hdr), 1, f) != 1) {
        fclose(f);
        return false;
    }

    if (fwrite(mesh->vertices, sizeof(ZeroVertex), mesh->vertex_count, f) != mesh->vertex_count) {
        fclose(f);
        return false;
    }

    if (mesh->index_count > 0 && mesh->indices) {
        if (fwrite(mesh->indices, sizeof(uint32_t), mesh->index_count, f) != mesh->index_count) {
            fclose(f);
            return false;
        }
    }

    fclose(f);
    return true;
}

ZeroMeshData zero_zmesh_load_file(const char *filepath, ZeroArena *arena) {
    ZeroMeshData result = {0};
    if (!filepath || !arena) return result;

    FILE *f = fopen(filepath, "rb");
    if (!f) return result;

    ZeroZmeshHeader hdr;
    if (fread(&hdr, sizeof(hdr), 1, f) != 1) {
        fclose(f);
        return result;
    }

    if (hdr.magic != ZERO_ZMESH_MAGIC || hdr.version != ZERO_ZMESH_VERSION) {
        fclose(f);
        return result;
    }

    ZeroVertex *verts = (ZeroVertex *)zero_arena_alloc(arena, hdr.vertex_count * sizeof(ZeroVertex), 64);
    uint32_t *indices = NULL;
    if (hdr.index_count > 0) {
        indices = (uint32_t *)zero_arena_alloc(arena, hdr.index_count * sizeof(uint32_t), 64);
    }

    if (!verts || (hdr.index_count > 0 && !indices)) {
        fclose(f);
        return result;
    }

    fseek(f, (long)hdr.vertex_data_offset, SEEK_SET);
    if (fread(verts, sizeof(ZeroVertex), hdr.vertex_count, f) != hdr.vertex_count) {
        fclose(f);
        return result;
    }

    if (hdr.index_count > 0) {
        fseek(f, (long)hdr.index_data_offset, SEEK_SET);
        if (fread(indices, sizeof(uint32_t), hdr.index_count, f) != hdr.index_count) {
            fclose(f);
            return result;
        }
    }

    fclose(f);

    result.vertices = verts;
    result.vertex_count = hdr.vertex_count;
    result.indices = indices;
    result.index_count = hdr.index_count;
    return result;
}

ZeroZmeshMap zero_zmesh_mmap_open(const char *filepath) {
    ZeroZmeshMap map = {0};
    if (!filepath) return map;

    int fd = open(filepath, O_RDONLY);
    if (fd < 0) return map;

    struct stat st;
    if (fstat(fd, &st) < 0 || st.st_size < (off_t)sizeof(ZeroZmeshHeader)) {
        close(fd);
        return map;
    }

    void *ptr = mmap(NULL, (size_t)st.st_size, PROT_READ, MAP_SHARED, fd, 0);
    if (ptr == MAP_FAILED) {
        close(fd);
        return map;
    }

    madvise(ptr, (size_t)st.st_size, MADV_WILLNEED);

    const ZeroZmeshHeader *hdr = (const ZeroZmeshHeader *)ptr;
    if (hdr->magic != ZERO_ZMESH_MAGIC || hdr->version != ZERO_ZMESH_VERSION) {
        munmap(ptr, (size_t)st.st_size);
        close(fd);
        return map;
    }

    map.mmap_ptr = ptr;
    map.file_size = (size_t)st.st_size;
    map.fd = fd;
    map.header = hdr;
    map.vertices = (const ZeroVertex *)((const uint8_t *)ptr + hdr->vertex_data_offset);
    if (hdr->index_count > 0) {
        map.indices = (const uint32_t *)((const uint8_t *)ptr + hdr->index_data_offset);
    }
    map.is_valid = true;
    return map;
}

void zero_zmesh_mmap_close(ZeroZmeshMap *map) {
    if (map && map->is_valid) {
        if (map->mmap_ptr && map->file_size > 0) {
            munmap(map->mmap_ptr, map->file_size);
        }
        if (map->fd >= 0) {
            close(map->fd);
        }
        memset(map, 0, sizeof(*map));
    }
}

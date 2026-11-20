#include "zero_obj.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

typedef struct ObjIndex {
    int v, vt, vn;
} ObjIndex;

static inline const char *skip_whitespace(const char *p, const char *end) {
    while (p < end && (*p == ' ' || *p == '\t' || *p == '\r')) {
        p++;
    }
    return p;
}

static inline const char *skip_to_eol(const char *p, const char *end) {
    while (p < end && *p != '\n') {
        p++;
    }
    if (p < end && *p == '\n') {
        p++;
    }
    return p;
}

static inline int fix_index(int idx, int count) {
    if (idx > 0) return idx - 1;
    if (idx < 0) return count + idx;
    return -1;
}

static ObjIndex parse_face_token(const char **p_ptr, const char *end, int v_count, int vt_count, int vn_count) {
    ObjIndex idx = {-1, -1, -1};
    const char *p = *p_ptr;

    char *next_p = NULL;
    int v = (int)strtol(p, &next_p, 10);
    idx.v = fix_index(v, v_count);
    p = next_p;

    if (p < end && *p == '/') {
        p++;
        if (p < end && *p != '/') {
            int vt = (int)strtol(p, &next_p, 10);
            idx.vt = fix_index(vt, vt_count);
            p = next_p;
        }
        if (p < end && *p == '/') {
            p++;
            int vn = (int)strtol(p, &next_p, 10);
            idx.vn = fix_index(vn, vn_count);
            p = next_p;
        }
    }

    *p_ptr = p;
    return idx;
}

ZeroMeshData zero_obj_parse_memory(const char *text, size_t length, ZeroArena *arena) {
    ZeroMeshData result = {0};
    if (!text || length == 0 || !arena) {
        return result;
    }

    // Pass 1: Count elements for exact allocation
    uint32_t count_v = 0, count_vt = 0, count_vn = 0, count_triangles = 0;
    const char *p = text;
    const char *end = text + length;

    while (p < end) {
        p = skip_whitespace(p, end);
        if (p >= end) break;

        if (*p == '#') {
            p = skip_to_eol(p, end);
            continue;
        }

        if (p[0] == 'v' && (p[1] == ' ' || p[1] == '\t')) {
            count_v++;
            p = skip_to_eol(p, end);
        } else if (p[0] == 'v' && p[1] == 't') {
            count_vt++;
            p = skip_to_eol(p, end);
        } else if (p[0] == 'v' && p[1] == 'n') {
            count_vn++;
            p = skip_to_eol(p, end);
        } else if (p[0] == 'f' && (p[1] == ' ' || p[1] == '\t')) {
            p += 2;
            int face_verts = 0;
            while (p < end && *p != '\n' && *p != '\r') {
                p = skip_whitespace(p, end);
                if (p >= end || *p == '\n' || *p == '\r') break;
                face_verts++;
                while (p < end && *p != ' ' && *p != '\t' && *p != '\n' && *p != '\r') {
                    p++;
                }
            }
            if (face_verts >= 3) {
                count_triangles += (uint32_t)(face_verts - 2);
            }
            p = skip_to_eol(p, end);
        } else {
            p = skip_to_eol(p, end);
        }
    }

    if (count_triangles == 0) {
        return result;
    }

    ZeroScratch scratch = zero_scratch_get(arena);
    Vec3 *temp_v = (Vec3 *)zero_arena_alloc(scratch.arena, (count_v + 1) * sizeof(Vec3), 16);
    Vec2 *temp_vt = (Vec2 *)zero_arena_alloc(scratch.arena, (count_vt + 1) * sizeof(Vec2), 16);
    Vec3 *temp_vn = (Vec3 *)zero_arena_alloc(scratch.arena, (count_vn + 1) * sizeof(Vec3), 16);

    uint32_t total_out_verts = count_triangles * 3;
    ZeroVertex *out_vertices = (ZeroVertex *)zero_arena_alloc(arena, total_out_verts * sizeof(ZeroVertex), 64);
    uint32_t *out_indices = (uint32_t *)zero_arena_alloc(arena, total_out_verts * sizeof(uint32_t), 64);

    if (!temp_v || !temp_vt || !temp_vn || !out_vertices || !out_indices) {
        zero_scratch_release(scratch);
        return result;
    }

    int v_count = 0, vt_count = 0, vn_count = 0;
    uint32_t total_verts = 0;
    uint32_t total_indices = 0;

    p = text;
    while (p < end) {
        p = skip_whitespace(p, end);
        if (p >= end) break;

        if (*p == '#') {
            p = skip_to_eol(p, end);
            continue;
        }

        if (p[0] == 'v' && (p[1] == ' ' || p[1] == '\t')) {
            p += 2;
            char *next_p = NULL;
            float x = strtof(p, &next_p); p = next_p;
            float y = strtof(p, &next_p); p = next_p;
            float z = strtof(p, &next_p); p = next_p;
            temp_v[v_count++] = vec3(x, y, z);
            p = skip_to_eol(p, end);
        } else if (p[0] == 'v' && p[1] == 't') {
            p += 2;
            char *next_p = NULL;
            float u = strtof(p, &next_p); p = next_p;
            float v = strtof(p, &next_p); p = next_p;
            temp_vt[vt_count++] = vec2(u, v);
            p = skip_to_eol(p, end);
        } else if (p[0] == 'v' && p[1] == 'n') {
            p += 2;
            char *next_p = NULL;
            float nx = strtof(p, &next_p); p = next_p;
            float ny = strtof(p, &next_p); p = next_p;
            float nz = strtof(p, &next_p); p = next_p;
            temp_vn[vn_count++] = vec3_normalize(vec3(nx, ny, nz));
            p = skip_to_eol(p, end);
        } else if (p[0] == 'f' && (p[1] == ' ' || p[1] == '\t')) {
            p += 2;
            ObjIndex face_indices[64];
            int face_count = 0;

            while (p < end && *p != '\n' && *p != '\r') {
                p = skip_whitespace(p, end);
                if (p >= end || *p == '\n' || *p == '\r') break;

                if (face_count < 64) {
                    face_indices[face_count++] = parse_face_token(&p, end, v_count, vt_count, vn_count);
                } else {
                    p = skip_to_eol(p, end);
                    break;
                }
            }

            for (int i = 1; i + 1 < face_count; ++i) {
                ObjIndex tri[3] = {face_indices[0], face_indices[i], face_indices[i + 1]};

                for (int t = 0; t < 3; ++t) {
                    ZeroVertex vert = {0};
                    if (tri[t].v >= 0 && tri[t].v < v_count) {
                        vert.pos = temp_v[tri[t].v];
                    }
                    if (tri[t].vt >= 0 && tri[t].vt < vt_count) {
                        vert.uv = temp_vt[tri[t].vt];
                    }
                    if (tri[t].vn >= 0 && tri[t].vn < vn_count) {
                        vert.normal = temp_vn[tri[t].vn];
                    } else {
                        vert.normal = vec3(0.0f, 1.0f, 0.0f);
                    }

                    out_vertices[total_verts] = vert;
                    out_indices[total_indices] = total_verts;
                    total_verts++;
                    total_indices++;
                }
            }
            p = skip_to_eol(p, end);
        } else {
            p = skip_to_eol(p, end);
        }
    }

    result.vertices = out_vertices;
    result.vertex_count = total_verts;
    result.indices = out_indices;
    result.index_count = total_indices;

    zero_scratch_release(scratch);
    return result;
}

ZeroMeshData zero_obj_parse_file(const char *filepath, ZeroArena *arena) {
    ZeroMeshData result = {0};
    if (!filepath || !arena) return result;

    FILE *f = fopen(filepath, "rb");
    if (!f) return result;

    fseek(f, 0, SEEK_END);
    long fsize = ftell(f);
    fseek(f, 0, SEEK_SET);

    if (fsize <= 0) {
        fclose(f);
        return result;
    }

    ZeroScratch scratch = zero_scratch_get(arena);
    char *buffer = (char *)zero_arena_alloc(scratch.arena, (size_t)fsize + 1, 16);
    if (!buffer) {
        fclose(f);
        zero_scratch_release(scratch);
        return result;
    }

    size_t read_bytes = fread(buffer, 1, (size_t)fsize, f);
    fclose(f);
    buffer[read_bytes] = '\0';

    result = zero_obj_parse_memory(buffer, read_bytes, arena);
    zero_scratch_release(scratch);
    return result;
}

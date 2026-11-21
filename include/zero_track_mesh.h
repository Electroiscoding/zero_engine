#ifndef ZERO_TRACK_MESH_H
#define ZERO_TRACK_MESH_H

#include "zero_mesh.h"
#include "zero_shader.h"
#include "zero_memory.h"
#include "zero_csm.h"
#include "zero_sky.h"
#include "zero_camera.h"
#include <stdbool.h>

typedef struct ZeroTrackMeshes {
    ZeroMesh road_mesh;
    ZeroMesh curbs_mesh;
    ZeroMesh barriers_mesh;
    ZeroMesh gantry_mesh;
    bool is_initialized;
} ZeroTrackMeshes;

ZeroTrackMeshes zero_track_meshes_generate(ZeroArena *arena);
void zero_track_meshes_draw(const ZeroTrackMeshes *meshes, ZeroShader *pbr_shader);
void zero_track_meshes_draw_shadow(const ZeroTrackMeshes *meshes, ZeroShader *shadow_shader);
void zero_track_meshes_destroy(ZeroTrackMeshes *meshes);

#endif /* ZERO_TRACK_MESH_H */

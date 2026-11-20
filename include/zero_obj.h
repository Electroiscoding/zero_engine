#ifndef ZERO_OBJ_H
#define ZERO_OBJ_H

#include "zero_core.h"
#include "zero_mesh.h"
#include "zero_memory.h"

ZeroMeshData zero_obj_parse_memory(const char *text, size_t length, ZeroArena *arena);
ZeroMeshData zero_obj_parse_file(const char *filepath, ZeroArena *arena);

#endif

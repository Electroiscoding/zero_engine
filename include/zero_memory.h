#ifndef ZERO_MEMORY_H
#define ZERO_MEMORY_H

#include "zero_core.h"

typedef struct ZeroArena {
    uint8_t *buffer;
    size_t capacity;
    size_t offset;
    size_t committed;
} ZeroArena;

typedef struct ZeroScratch {
    ZeroArena *arena;
    size_t offset;
} ZeroScratch;

ZeroArena zero_arena_create(size_t capacity);
void zero_arena_destroy(ZeroArena *arena);
void *zero_arena_alloc(ZeroArena *arena, size_t size, size_t alignment);
void zero_arena_reset(ZeroArena *arena);

void zero_scratch_init(size_t capacity);
void zero_scratch_shutdown(void);
ZeroScratch zero_scratch_get(ZeroArena *conflict);
void zero_scratch_release(ZeroScratch scratch);

#endif

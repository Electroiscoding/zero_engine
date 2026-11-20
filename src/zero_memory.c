#define _GNU_SOURCE
#include "zero_memory.h"
#include <sys/mman.h>
#include <unistd.h>
#include <string.h>

static ZeroArena g_scratch_arenas[2];
static bool g_scratch_initialized = false;

ZeroArena zero_arena_create(size_t capacity) {
    ZeroArena arena;
    size_t page_aligned_capacity = ZERO_ALIGN_UP(capacity, ZERO_PAGE_SIZE);
    
    arena.buffer = (uint8_t *)mmap(
        NULL,
        page_aligned_capacity,
        PROT_READ | PROT_WRITE,
        MAP_PRIVATE | MAP_ANONYMOUS,
        -1,
        0
    );

    if (arena.buffer == MAP_FAILED) {
        arena.buffer = NULL;
        arena.capacity = 0;
        arena.offset = 0;
        arena.committed = 0;
        return arena;
    }

    arena.capacity = page_aligned_capacity;
    arena.offset = 0;
    arena.committed = page_aligned_capacity;
    return arena;
}

void zero_arena_destroy(ZeroArena *arena) {
    if (arena && arena->buffer) {
        munmap(arena->buffer, arena->capacity);
        arena->buffer = NULL;
        arena->capacity = 0;
        arena->offset = 0;
        arena->committed = 0;
    }
}

void *zero_arena_alloc(ZeroArena *arena, size_t size, size_t alignment) {
    if (!arena || !arena->buffer || size == 0) {
        return NULL;
    }

    if (alignment == 0) {
        alignment = ZERO_CACHE_LINE;
    }

    uintptr_t current_ptr = (uintptr_t)arena->buffer + arena->offset;
    uintptr_t aligned_ptr = ZERO_ALIGN_UP(current_ptr, alignment);
    size_t new_offset = (aligned_ptr - (uintptr_t)arena->buffer) + size;

    if (new_offset > arena->capacity) {
        return NULL;
    }

    arena->offset = new_offset;
    void *ptr = (void *)aligned_ptr;
    memset(ptr, 0, size);
    return ptr;
}

void zero_arena_reset(ZeroArena *arena) {
    if (arena) {
        arena->offset = 0;
    }
}

void zero_scratch_init(size_t capacity) {
    if (!g_scratch_initialized) {
        g_scratch_arenas[0] = zero_arena_create(capacity);
        g_scratch_arenas[1] = zero_arena_create(capacity);
        g_scratch_initialized = true;
    }
}

void zero_scratch_shutdown(void) {
    if (g_scratch_initialized) {
        zero_arena_destroy(&g_scratch_arenas[0]);
        zero_arena_destroy(&g_scratch_arenas[1]);
        g_scratch_initialized = false;
    }
}

ZeroScratch zero_scratch_get(ZeroArena *conflict) {
    ZeroScratch scratch;
    if (!g_scratch_initialized) {
        zero_scratch_init(32 * 1024 * 1024);
    }

    ZeroArena *selected = &g_scratch_arenas[0];
    if (conflict == selected) {
        selected = &g_scratch_arenas[1];
    }

    scratch.arena = selected;
    scratch.offset = selected->offset;
    return scratch;
}

void zero_scratch_release(ZeroScratch scratch) {
    if (scratch.arena && scratch.offset <= scratch.arena->offset) {
        scratch.arena->offset = scratch.offset;
    }
}

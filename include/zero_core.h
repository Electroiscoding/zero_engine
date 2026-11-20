#ifndef ZERO_CORE_H
#define ZERO_CORE_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#define ZERO_ALIGNAS(x) __attribute__((aligned(x)))
#define ZERO_FORCE_INLINE inline __attribute__((always_inline))

#define ZERO_CACHE_LINE 64
#define ZERO_PAGE_SIZE 4096

#define ZERO_ARRAY_COUNT(arr) (sizeof(arr) / sizeof((arr)[0]))
#define ZERO_ALIGN_UP(size, align) (((size) + ((align) - 1)) & ~((align) - 1))

#endif

#ifndef ZERO_IMAGE_H
#define ZERO_IMAGE_H

#include "zero_core.h"
#include "zero_memory.h"
#include "zero_gl_defs.h"

typedef struct ZeroImage {
    uint32_t width;
    uint32_t height;
    uint32_t channels;
    uint8_t *pixels;
    bool is_valid;
} ZeroImage;

ZeroImage zero_image_load_tga_memory(const uint8_t *data, size_t size, ZeroArena *arena);
ZeroImage zero_image_load_tga_file(const char *filepath, ZeroArena *arena);

ZeroImage zero_image_load_bmp_memory(const uint8_t *data, size_t size, ZeroArena *arena);
ZeroImage zero_image_load_bmp_file(const char *filepath, ZeroArena *arena);

GLuint zero_image_upload_texture(const ZeroImage *image);
void zero_image_destroy_texture(GLuint texture_id);

#endif

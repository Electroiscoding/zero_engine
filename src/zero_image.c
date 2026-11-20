#include "zero_image.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

ZeroImage zero_image_load_tga_memory(const uint8_t *data, size_t size, ZeroArena *arena) {
    ZeroImage img = {0, 0, 0, NULL, false};
    if (!data || size < 18 || !arena) {
        return img;
    }

    uint8_t id_len = data[0];
    uint8_t img_type = data[2];

    if (img_type != 2) {
        return img;
    }

    uint32_t width = (uint32_t)data[12] | ((uint32_t)data[13] << 8);
    uint32_t height = (uint32_t)data[14] | ((uint32_t)data[15] << 8);
    uint8_t bpp = data[16];
    uint8_t descriptor = data[17];

    if (width == 0 || height == 0 || (bpp != 24 && bpp != 32)) {
        return img;
    }

    uint32_t channels = bpp / 8;
    size_t pixel_data_offset = 18 + id_len;
    size_t expected_size = pixel_data_offset + (size_t)width * height * channels;

    if (size < expected_size) {
        return img;
    }

    size_t total_bytes = (size_t)width * height * channels;
    uint8_t *pixels = (uint8_t *)zero_arena_alloc(arena, total_bytes, 64);
    if (!pixels) {
        return img;
    }

    bool flip_vertically = !(descriptor & 0x20);
    const uint8_t *src_pixels = data + pixel_data_offset;

    for (uint32_t y = 0; y < height; ++y) {
        uint32_t src_y = flip_vertically ? (height - 1 - y) : y;
        const uint8_t *src_row = src_pixels + (size_t)src_y * width * channels;
        uint8_t *dst_row = pixels + (size_t)y * width * channels;

        for (uint32_t x = 0; x < width; ++x) {
            uint32_t px_offset = x * channels;
            dst_row[px_offset + 0] = src_row[px_offset + 2];
            dst_row[px_offset + 1] = src_row[px_offset + 1];
            dst_row[px_offset + 2] = src_row[px_offset + 0];
            if (channels == 4) {
                dst_row[px_offset + 3] = src_row[px_offset + 3];
            }
        }
    }

    img.width = width;
    img.height = height;
    img.channels = channels;
    img.pixels = pixels;
    img.is_valid = true;
    return img;
}

ZeroImage zero_image_load_bmp_memory(const uint8_t *data, size_t size, ZeroArena *arena) {
    ZeroImage img = {0, 0, 0, NULL, false};
    if (!data || size < 54 || !arena) {
        return img;
    }

    if (data[0] != 'B' || data[1] != 'M') {
        return img;
    }

    uint32_t pixel_offset = (uint32_t)data[10] | ((uint32_t)data[11] << 8) |
                            ((uint32_t)data[12] << 16) | ((uint32_t)data[13] << 24);

    uint32_t dib_size = (uint32_t)data[14] | ((uint32_t)data[15] << 8) |
                        ((uint32_t)data[16] << 16) | ((uint32_t)data[17] << 24);

    if (dib_size < 40) {
        return img;
    }

    int32_t raw_width = (int32_t)((uint32_t)data[18] | ((uint32_t)data[19] << 8) |
                                  ((uint32_t)data[20] << 16) | ((uint32_t)data[21] << 24));
    int32_t raw_height = (int32_t)((uint32_t)data[22] | ((uint32_t)data[23] << 8) |
                                   ((uint32_t)data[24] << 16) | ((uint32_t)data[25] << 24));

    uint16_t bpp = (uint16_t)data[28] | ((uint16_t)data[29] << 8);
    uint32_t compression = (uint32_t)data[30] | ((uint32_t)data[31] << 8) |
                           ((uint32_t)data[32] << 16) | ((uint32_t)data[33] << 24);

    if (raw_width <= 0 || raw_height == 0 || compression != 0 || (bpp != 24 && bpp != 32)) {
        return img;
    }

    uint32_t width = (uint32_t)raw_width;
    bool top_down = (raw_height < 0);
    uint32_t height = top_down ? (uint32_t)(-raw_height) : (uint32_t)raw_height;
    uint32_t channels = bpp / 8;

    size_t row_stride = ((size_t)width * channels + 3) & ~3;
    if (size < pixel_offset + row_stride * height) {
        return img;
    }

    size_t total_bytes = (size_t)width * height * channels;
    uint8_t *pixels = (uint8_t *)zero_arena_alloc(arena, total_bytes, 64);
    if (!pixels) {
        return img;
    }

    const uint8_t *src_pixels = data + pixel_offset;

    for (uint32_t y = 0; y < height; ++y) {
        uint32_t src_y = top_down ? y : (height - 1 - y);
        const uint8_t *src_row = src_pixels + (size_t)src_y * row_stride;
        uint8_t *dst_row = pixels + (size_t)y * width * channels;

        for (uint32_t x = 0; x < width; ++x) {
            uint32_t px_offset = x * channels;
            dst_row[px_offset + 0] = src_row[px_offset + 2];
            dst_row[px_offset + 1] = src_row[px_offset + 1];
            dst_row[px_offset + 2] = src_row[px_offset + 0];
            if (channels == 4) {
                dst_row[px_offset + 3] = src_row[px_offset + 3];
            }
        }
    }

    img.width = width;
    img.height = height;
    img.channels = channels;
    img.pixels = pixels;
    img.is_valid = true;
    return img;
}

static uint8_t *read_entire_file(const char *filepath, size_t *out_size, ZeroArena *arena) {
    if (!filepath || !out_size || !arena) return NULL;
    FILE *f = fopen(filepath, "rb");
    if (!f) return NULL;

    fseek(f, 0, SEEK_END);
    long fsize = ftell(f);
    fseek(f, 0, SEEK_SET);

    if (fsize <= 0) {
        fclose(f);
        return NULL;
    }

    uint8_t *buffer = (uint8_t *)zero_arena_alloc(arena, (size_t)fsize, 64);
    if (!buffer) {
        fclose(f);
        return NULL;
    }

    size_t read_bytes = fread(buffer, 1, (size_t)fsize, f);
    fclose(f);

    if (read_bytes != (size_t)fsize) {
        return NULL;
    }

    *out_size = (size_t)fsize;
    return buffer;
}

ZeroImage zero_image_load_tga_file(const char *filepath, ZeroArena *arena) {
    ZeroScratch scratch = zero_scratch_get(arena);
    size_t file_size = 0;
    uint8_t *file_data = read_entire_file(filepath, &file_size, scratch.arena);
    ZeroImage img = {0};
    if (file_data) {
        img = zero_image_load_tga_memory(file_data, file_size, arena);
    }
    zero_scratch_release(scratch);
    return img;
}

ZeroImage zero_image_load_bmp_file(const char *filepath, ZeroArena *arena) {
    ZeroScratch scratch = zero_scratch_get(arena);
    size_t file_size = 0;
    uint8_t *file_data = read_entire_file(filepath, &file_size, scratch.arena);
    ZeroImage img = {0};
    if (file_data) {
        img = zero_image_load_bmp_memory(file_data, file_size, arena);
    }
    zero_scratch_release(scratch);
    return img;
}

GLuint zero_image_upload_texture(const ZeroImage *image) {
    if (!image || !image->is_valid || !image->pixels) {
        return 0;
    }

    GLuint tex = 0;
    glGenTextures(1, &tex);
    if (!tex) return 0;

    glBindTexture(GL_TEXTURE_2D, tex);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);

    GLenum format = (image->channels == 4) ? GL_RGBA : GL_RGB;
    glTexImage2D(GL_TEXTURE_2D, 0, (GLint)format, (GLsizei)image->width, (GLsizei)image->height, 0, format, GL_UNSIGNED_BYTE, image->pixels);
    glBindTexture(GL_TEXTURE_2D, 0);

    return tex;
}

void zero_image_destroy_texture(GLuint texture_id) {
    if (texture_id) {
        glDeleteTextures(1, &texture_id);
    }
}

#ifndef ZERO_SHADER_H
#define ZERO_SHADER_H

#include "zero_core.h"
#include "zero_math.h"
#include "zero_gl_defs.h"

typedef struct ZeroShader {
    GLuint program_id;
    bool is_valid;
} ZeroShader;

ZeroShader zero_shader_create(const char *vert_src, const char *frag_src);
void zero_shader_destroy(ZeroShader *shader);
void zero_shader_bind(ZeroShader *shader);
void zero_shader_unbind(void);

void zero_shader_set_mat4(ZeroShader *shader, const char *name, Mat4 mat);
void zero_shader_set_vec3(ZeroShader *shader, const char *name, Vec3 v);
void zero_shader_set_float(ZeroShader *shader, const char *name, float val);

const char *zero_shader_default_vert(void);
const char *zero_shader_default_frag(void);

#endif

#ifndef ZERO_CSM_H
#define ZERO_CSM_H

#include "zero_core.h"
#include "zero_math.h"
#include "zero_camera.h"
#include "zero_shader.h"
#include "zero_gl_defs.h"

#define ZERO_CSM_CASCADE_COUNT 3
#define ZERO_CSM_DEFAULT_RESOLUTION 1024
#define ZERO_CSM_DEFAULT_MAX_DISTANCE 150.0f
#define ZERO_CSM_DEFAULT_LAMBDA 0.75f
#define ZERO_CSM_DEFAULT_CASTER_MARGIN 50.0f

typedef struct ZeroCSMConfig {
    uint32_t resolution;        /* Shadow map resolution per cascade (e.g. 1024) */
    float max_distance;        /* Maximum shadow distance in world units (e.g. 150.0f) */
    float split_lambda;        /* Logarithmic-uniform split weight [0.0, 1.0] */
    float caster_depth_margin; /* Extrusion distance along light ray for shadow casters */
} ZeroCSMConfig;

typedef struct ZeroCSM {
    ZeroCSMConfig config;
    GLuint fbos[ZERO_CSM_CASCADE_COUNT];
    GLuint depth_textures[ZERO_CSM_CASCADE_COUNT];
    float split_depths[ZERO_CSM_CASCADE_COUNT];
    Mat4 light_view_proj[ZERO_CSM_CASCADE_COUNT];
    Vec3 light_dir;
    bool is_initialized;
} ZeroCSM;

static ZERO_FORCE_INLINE ZeroCSMConfig zero_csm_default_config(void) {
    ZeroCSMConfig cfg;
    cfg.resolution = ZERO_CSM_DEFAULT_RESOLUTION;
    cfg.max_distance = ZERO_CSM_DEFAULT_MAX_DISTANCE;
    cfg.split_lambda = ZERO_CSM_DEFAULT_LAMBDA;
    cfg.caster_depth_margin = ZERO_CSM_DEFAULT_CASTER_MARGIN;
    return cfg;
}

/* Lifecycle */
bool zero_csm_init(ZeroCSM *csm, const ZeroCSMConfig *config);
void zero_csm_destroy(ZeroCSM *csm);

/* Mathematical Frustum & Projection Routines (100% Headless Safe) */
void zero_csm_calculate_splits(float near_z, float max_dist, float lambda, float *out_splits);
void zero_csm_calculate_bounding_sphere(const ZeroCamera *cam, float slice_near, float slice_far, Vec3 *out_center, float *out_radius);
Mat4 zero_csm_calculate_light_matrix(Vec3 center, float radius, Vec3 light_dir, uint32_t resolution, float caster_depth_margin);
void zero_csm_update(ZeroCSM *csm, const ZeroCamera *camera, Vec3 light_dir);

/* Pass Execution & Multi-Texturing */
void zero_csm_bind_cascade_fbo(const ZeroCSM *csm, uint32_t cascade_index);
void zero_csm_unbind_fbo(uint32_t screen_width, uint32_t screen_height);
void zero_csm_bind_textures(const ZeroCSM *csm, uint32_t start_texture_slot);
void zero_csm_set_shader_uniforms(const ZeroCSM *csm, ZeroShader *shader, uint32_t start_texture_slot);

/* Built-in Depth Shader Sources */
const char *zero_csm_depth_vert(void);
const char *zero_csm_depth_frag(void);

#endif /* ZERO_CSM_H */

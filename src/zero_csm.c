#include "zero_csm.h"
#include "zero_gl_loader.h"
#include <string.h>
#include <math.h>

void zero_csm_calculate_splits(float near_z, float max_dist, float lambda, float *out_splits) {
    if (!out_splits) return;

    float n = (near_z > 0.001f) ? near_z : 0.001f;
    float f = (max_dist > n + 0.1f) ? max_dist : (n + 0.1f);
    float l = lambda;
    if (l < 0.0f) l = 0.0f;
    if (l > 1.0f) l = 1.0f;

    for (int i = 1; i <= ZERO_CSM_CASCADE_COUNT; ++i) {
        float p = (float)i / (float)ZERO_CSM_CASCADE_COUNT;
        float c_log = n * powf(f / n, p);
        float c_uni = n + (f - n) * p;
        float c = l * c_log + (1.0f - l) * c_uni;
        out_splits[i - 1] = c;
    }
    out_splits[ZERO_CSM_CASCADE_COUNT - 1] = f;
}

void zero_csm_calculate_bounding_sphere(const ZeroCamera *cam, float slice_near, float slice_far, Vec3 *out_center, float *out_radius) {
    if (!cam) return;

    float zn = (slice_near > 0.0001f) ? slice_near : 0.0001f;
    float zf = (slice_far > zn + 0.0001f) ? slice_far : (zn + 0.0001f);

    float tan_half_fov = tanf(cam->fov * 0.5f);
    float hn = zn * tan_half_fov;
    float wn = hn * cam->aspect;
    float hf = zf * tan_half_fov;
    float wf = hf * cam->aspect;

    Vec3 fwd = vec3_normalize(cam->forward);
    Vec3 right = vec3_normalize(cam->right);
    Vec3 up = vec3_normalize(cam->up);

    Vec3 pn = vec3_add(cam->position, vec3_scale(fwd, zn));
    Vec3 pf = vec3_add(cam->position, vec3_scale(fwd, zf));
    Vec3 center = vec3_scale(vec3_add(pn, pf), 0.5f);

    Vec3 corners[8];
    corners[0] = vec3_add(pn, vec3_add(vec3_scale(right,  wn), vec3_scale(up,  hn)));
    corners[1] = vec3_add(pn, vec3_add(vec3_scale(right, -wn), vec3_scale(up,  hn)));
    corners[2] = vec3_add(pn, vec3_add(vec3_scale(right,  wn), vec3_scale(up, -hn)));
    corners[3] = vec3_add(pn, vec3_add(vec3_scale(right, -wn), vec3_scale(up, -hn)));
    corners[4] = vec3_add(pf, vec3_add(vec3_scale(right,  wf), vec3_scale(up,  hf)));
    corners[5] = vec3_add(pf, vec3_add(vec3_scale(right, -wf), vec3_scale(up,  hf)));
    corners[6] = vec3_add(pf, vec3_add(vec3_scale(right,  wf), vec3_scale(up, -hf)));
    corners[7] = vec3_add(pf, vec3_add(vec3_scale(right, -wf), vec3_scale(up, -hf)));

    float max_r2 = 0.0f;
    for (int i = 0; i < 8; ++i) {
        Vec3 d = vec3_sub(corners[i], center);
        float r2 = vec3_dot(d, d);
        if (r2 > max_r2) {
            max_r2 = r2;
        }
    }
    float radius = sqrtf(max_r2);

    if (out_center) *out_center = center;
    if (out_radius) *out_radius = radius;
}

Mat4 zero_csm_calculate_light_matrix(Vec3 center, float radius, Vec3 light_dir, uint32_t resolution, float caster_depth_margin) {
    float len = vec3_length(light_dir);
    Vec3 L = (len > 1e-4f) ? vec3_scale(light_dir, 1.0f / len) : vec3(0.0f, -1.0f, 0.0f);

    Vec3 up_temp = (fabsf(L.y) > 0.99f) ? vec3(0.0f, 0.0f, 1.0f) : vec3(0.0f, 1.0f, 0.0f);
    Vec3 right_light = vec3_normalize(vec3_cross(L, up_temp));
    Vec3 up_light = vec3_cross(right_light, L);

    uint32_t res = (resolution >= 256) ? resolution : 1024;
    float rad = (radius > 0.001f) ? radius : 1.0f;
    float texel_size = (2.0f * rad) / (float)res;

    float cx = vec3_dot(center, right_light);
    float cy = vec3_dot(center, up_light);

    float snap_cx = floorf(cx / texel_size) * texel_size;
    float snap_cy = floorf(cy / texel_size) * texel_size;

    Vec3 snapped_center = vec3_add(center,
        vec3_add(vec3_scale(right_light, snap_cx - cx),
                 vec3_scale(up_light, snap_cy - cy)));

    float margin = (caster_depth_margin > 0.0f) ? caster_depth_margin : 50.0f;
    Vec3 eye = vec3_sub(snapped_center, vec3_scale(L, rad + margin));

    Mat4 v_light = mat4_look_at(eye, snapped_center, up_light);
    Mat4 p_light = mat4_ortho(-rad, rad, -rad, rad, 0.0f, 2.0f * rad + margin);
    return mat4_mul(p_light, v_light);
}

bool zero_csm_init(ZeroCSM *csm, const ZeroCSMConfig *config) {
    if (!csm) return false;
    memset(csm, 0, sizeof(*csm));

    if (config) {
        csm->config = *config;
    } else {
        csm->config = zero_csm_default_config();
    }

    if (csm->config.resolution < 256) csm->config.resolution = ZERO_CSM_DEFAULT_RESOLUTION;
    if (csm->config.max_distance <= 0.0f) csm->config.max_distance = ZERO_CSM_DEFAULT_MAX_DISTANCE;
    if (csm->config.split_lambda < 0.0f || csm->config.split_lambda > 1.0f) {
        csm->config.split_lambda = ZERO_CSM_DEFAULT_LAMBDA;
    }
    if (csm->config.caster_depth_margin <= 0.0f) {
        csm->config.caster_depth_margin = ZERO_CSM_DEFAULT_CASTER_MARGIN;
    }

    /* Initialize default split depths with near=0.1f */
    zero_csm_calculate_splits(0.1f, csm->config.max_distance, csm->config.split_lambda, csm->split_depths);

    /* Headless check: if OpenGL FBO functions are not loaded, safely succeed in CPU mode */
    if (!zgl.glGenFramebuffers || !zgl.glBindFramebuffer || !zgl.glFramebufferTexture2D) {
        csm->is_initialized = true;
        return true;
    }

    /* Hardware OpenGL FBO & Texture Setup */
    static const GLfloat border_color[4] = {1.0f, 1.0f, 1.0f, 1.0f};

    for (uint32_t i = 0; i < ZERO_CSM_CASCADE_COUNT; ++i) {
        glGenTextures(1, &csm->depth_textures[i]);
        glBindTexture(GL_TEXTURE_2D, csm->depth_textures[i]);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT24,
                     (GLsizei)csm->config.resolution, (GLsizei)csm->config.resolution,
                     0, GL_DEPTH_COMPONENT, GL_FLOAT, NULL);

        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);

        if (zgl.glTexParameterfv) {
            zgl.glTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, border_color);
        } else {
            glTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, border_color);
        }

        zgl.glGenFramebuffers(1, &csm->fbos[i]);
        zgl.glBindFramebuffer(GL_FRAMEBUFFER, csm->fbos[i]);
        zgl.glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, csm->depth_textures[i], 0);

        if (zgl.glDrawBuffer) {
            zgl.glDrawBuffer(GL_NONE);
        } else if (zgl.glDrawBuffers) {
            GLenum none = GL_NONE;
            zgl.glDrawBuffers(1, &none);
        }
        if (zgl.glReadBuffer) {
            zgl.glReadBuffer(GL_NONE);
        }

        GLenum status = zgl.glCheckFramebufferStatus(GL_FRAMEBUFFER);
        if (status != GL_FRAMEBUFFER_COMPLETE) {
            zero_csm_destroy(csm);
            return false;
        }
    }

    zgl.glBindFramebuffer(GL_FRAMEBUFFER, 0);
    csm->is_initialized = true;
    return true;
}

void zero_csm_destroy(ZeroCSM *csm) {
    if (!csm || !csm->is_initialized) return;

    for (uint32_t i = 0; i < ZERO_CSM_CASCADE_COUNT; ++i) {
        if (zgl.glDeleteFramebuffers && csm->fbos[i]) {
            zgl.glDeleteFramebuffers(1, &csm->fbos[i]);
            csm->fbos[i] = 0;
        }
        if (csm->depth_textures[i]) {
            glDeleteTextures(1, &csm->depth_textures[i]);
            csm->depth_textures[i] = 0;
        }
    }

    csm->is_initialized = false;
}

void zero_csm_update(ZeroCSM *csm, const ZeroCamera *camera, Vec3 light_dir) {
    if (!csm || !camera) return;

    csm->light_dir = light_dir;
    zero_csm_calculate_splits(camera->near_z, csm->config.max_distance, csm->config.split_lambda, csm->split_depths);

    for (uint32_t i = 0; i < ZERO_CSM_CASCADE_COUNT; ++i) {
        float slice_near = (i == 0) ? camera->near_z : csm->split_depths[i - 1];
        float slice_far = csm->split_depths[i];

        Vec3 center;
        float radius;
        zero_csm_calculate_bounding_sphere(camera, slice_near, slice_far, &center, &radius);

        csm->light_view_proj[i] = zero_csm_calculate_light_matrix(
            center, radius, light_dir, csm->config.resolution, csm->config.caster_depth_margin);
    }
}

void zero_csm_bind_cascade_fbo(const ZeroCSM *csm, uint32_t cascade_index) {
    if (!csm || !csm->is_initialized || cascade_index >= ZERO_CSM_CASCADE_COUNT) return;
    if (!zgl.glBindFramebuffer || csm->fbos[cascade_index] == 0) return;

    zgl.glBindFramebuffer(GL_FRAMEBUFFER, csm->fbos[cascade_index]);
    glViewport(0, 0, (GLsizei)csm->config.resolution, (GLsizei)csm->config.resolution);
    glClear(GL_DEPTH_BUFFER_BIT);
}

void zero_csm_unbind_fbo(uint32_t screen_width, uint32_t screen_height) {
    if (!zgl.glBindFramebuffer) return;
    zgl.glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glViewport(0, 0, (GLsizei)screen_width, (GLsizei)screen_height);
}

void zero_csm_bind_textures(const ZeroCSM *csm, uint32_t start_texture_slot) {
    if (!csm || !csm->is_initialized || !zgl.glActiveTexture) return;

    for (uint32_t i = 0; i < ZERO_CSM_CASCADE_COUNT; ++i) {
        if (csm->depth_textures[i]) {
            zgl.glActiveTexture(GL_TEXTURE0 + start_texture_slot + i);
            glBindTexture(GL_TEXTURE_2D, csm->depth_textures[i]);
        }
    }
}

void zero_csm_set_shader_uniforms(const ZeroCSM *csm, ZeroShader *shader, uint32_t start_texture_slot) {
    if (!csm || !shader || !shader->is_valid) return;

    zero_shader_set_mat4(shader, "u_light_view_proj[0]", csm->light_view_proj[0]);
    zero_shader_set_mat4(shader, "u_light_view_proj[1]", csm->light_view_proj[1]);
    zero_shader_set_mat4(shader, "u_light_view_proj[2]", csm->light_view_proj[2]);
    zero_shader_set_mat4(shader, "u_light_view_proj_0", csm->light_view_proj[0]);
    zero_shader_set_mat4(shader, "u_light_view_proj_1", csm->light_view_proj[1]);
    zero_shader_set_mat4(shader, "u_light_view_proj_2", csm->light_view_proj[2]);

    if (zgl.glUniform1fv && zgl.glGetUniformLocation) {
        GLint loc = zgl.glGetUniformLocation(shader->program_id, "u_cascade_splits");
        if (loc >= 0) {
            zgl.glUniform1fv(loc, ZERO_CSM_CASCADE_COUNT, csm->split_depths);
        }
    }
    zero_shader_set_float(shader, "u_cascade_splits[0]", csm->split_depths[0]);
    zero_shader_set_float(shader, "u_cascade_splits[1]", csm->split_depths[1]);
    zero_shader_set_float(shader, "u_cascade_splits[2]", csm->split_depths[2]);
    zero_shader_set_float(shader, "u_cascade_split_0", csm->split_depths[0]);
    zero_shader_set_float(shader, "u_cascade_split_1", csm->split_depths[1]);
    zero_shader_set_float(shader, "u_cascade_split_2", csm->split_depths[2]);

    if (zgl.glUniform1i && zgl.glGetUniformLocation) {
        GLint s0 = zgl.glGetUniformLocation(shader->program_id, "u_shadow_map[0]");
        if (s0 >= 0) zgl.glUniform1i(s0, (GLint)(start_texture_slot + 0));
        GLint s1 = zgl.glGetUniformLocation(shader->program_id, "u_shadow_map[1]");
        if (s1 >= 0) zgl.glUniform1i(s1, (GLint)(start_texture_slot + 1));
        GLint s2 = zgl.glGetUniformLocation(shader->program_id, "u_shadow_map[2]");
        if (s2 >= 0) zgl.glUniform1i(s2, (GLint)(start_texture_slot + 2));

        GLint s0_alt = zgl.glGetUniformLocation(shader->program_id, "u_shadow_map_0");
        if (s0_alt >= 0) zgl.glUniform1i(s0_alt, (GLint)(start_texture_slot + 0));
        GLint s1_alt = zgl.glGetUniformLocation(shader->program_id, "u_shadow_map_1");
        if (s1_alt >= 0) zgl.glUniform1i(s1_alt, (GLint)(start_texture_slot + 1));
        GLint s2_alt = zgl.glGetUniformLocation(shader->program_id, "u_shadow_map_2");
        if (s2_alt >= 0) zgl.glUniform1i(s2_alt, (GLint)(start_texture_slot + 2));
    }

    zero_shader_set_vec3(shader, "u_light_dir", csm->light_dir);
}

const char *zero_csm_depth_vert(void) {
    return
        "#version 330 core\n"
        "layout(location = 0) in vec3 a_pos;\n"
        "uniform mat4 u_model;\n"
        "uniform mat4 u_light_view_proj;\n"
        "void main() {\n"
        "    gl_Position = u_light_view_proj * u_model * vec4(a_pos, 1.0);\n"
        "}\n";
}

const char *zero_csm_depth_frag(void) {
    return
        "#version 330 core\n"
        "void main() {\n"
        "    // Depth automatically written to gl_FragCoord.z\n"
        "}\n";
}

#include "zero_shader.h"
#include "zero_gl_loader.h"
#include <stdio.h>

static GLuint compile_shader_stage(GLenum type, const char *source) {
    if (!zgl.glCreateShader || !source) {
        return 0;
    }

    GLuint shader = zgl.glCreateShader(type);
    if (!shader) {
        return 0;
    }

    zgl.glShaderSource(shader, 1, &source, NULL);
    zgl.glCompileShader(shader);

    GLint success = 0;
    zgl.glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
    if (!success) {
        char info_log[512];
        zgl.glGetShaderInfoLog(shader, sizeof(info_log), NULL, info_log);
        fprintf(stderr, "[ZeroShader Compile Error]: %s\n", info_log);
        zgl.glDeleteShader(shader);
        return 0;
    }
    return shader;
}

ZeroShader zero_shader_create(const char *vert_src, const char *frag_src) {
    ZeroShader s = {0, false};
    if (!zgl.glCreateProgram) {
        return s;
    }

    GLuint vs = compile_shader_stage(GL_VERTEX_SHADER, vert_src);
    if (!vs) {
        return s;
    }

    GLuint fs = compile_shader_stage(GL_FRAGMENT_SHADER, frag_src);
    if (!fs) {
        zgl.glDeleteShader(vs);
        return s;
    }

    GLuint program = zgl.glCreateProgram();
    zgl.glAttachShader(program, vs);
    zgl.glAttachShader(program, fs);
    zgl.glLinkProgram(program);

    GLint success = 0;
    zgl.glGetProgramiv(program, GL_LINK_STATUS, &success);
    if (!success) {
        char info_log[512];
        zgl.glGetProgramInfoLog(program, sizeof(info_log), NULL, info_log);
        fprintf(stderr, "[ZeroShader Link Error]: %s\n", info_log);
        zgl.glDeleteShader(vs);
        zgl.glDeleteShader(fs);
        zgl.glDeleteProgram(program);
        return s;
    }

    zgl.glDeleteShader(vs);
    zgl.glDeleteShader(fs);

    s.program_id = program;
    s.is_valid = true;
    return s;
}

void zero_shader_destroy(ZeroShader *shader) {
    if (shader && shader->is_valid && zgl.glDeleteProgram) {
        zgl.glDeleteProgram(shader->program_id);
        shader->program_id = 0;
        shader->is_valid = false;
    }
}

void zero_shader_bind(ZeroShader *shader) {
    if (shader && shader->is_valid && zgl.glUseProgram) {
        zgl.glUseProgram(shader->program_id);
    }
}

void zero_shader_unbind(void) {
    if (zgl.glUseProgram) {
        zgl.glUseProgram(0);
    }
}

void zero_shader_set_mat4(ZeroShader *shader, const char *name, Mat4 mat) {
    if (shader && shader->is_valid && zgl.glGetUniformLocation && zgl.glUniformMatrix4fv) {
        GLint loc = zgl.glGetUniformLocation(shader->program_id, name);
        if (loc >= 0) {
            zgl.glUniformMatrix4fv(loc, 1, GL_FALSE, mat.m);
        }
    }
}

void zero_shader_set_vec3(ZeroShader *shader, const char *name, Vec3 v) {
    if (shader && shader->is_valid && zgl.glGetUniformLocation && zgl.glUniform3f) {
        GLint loc = zgl.glGetUniformLocation(shader->program_id, name);
        if (loc >= 0) {
            zgl.glUniform3f(loc, v.x, v.y, v.z);
        }
    }
}

void zero_shader_set_float(ZeroShader *shader, const char *name, float val) {
    if (shader && shader->is_valid && zgl.glGetUniformLocation && zgl.glUniform1f) {
        GLint loc = zgl.glGetUniformLocation(shader->program_id, name);
        if (loc >= 0) {
            zgl.glUniform1f(loc, val);
        }
    }
}

const char *zero_shader_default_vert(void) {
    return 
        "#version 330 core\n"
        "layout(location = 0) in vec3 a_pos;\n"
        "layout(location = 1) in vec3 a_normal;\n"
        "layout(location = 2) in vec2 a_uv;\n"
        "uniform mat4 u_model;\n"
        "uniform mat4 u_view;\n"
        "uniform mat4 u_proj;\n"
        "out vec3 v_normal;\n"
        "out vec3 v_world_pos;\n"
        "out vec2 v_uv;\n"
        "void main() {\n"
        "    vec4 world_pos = u_model * vec4(a_pos, 1.0);\n"
        "    v_world_pos = world_pos.xyz;\n"
        "    v_normal = mat3(u_model) * a_normal;\n"
        "    v_uv = a_uv;\n"
        "    gl_Position = u_proj * u_view * world_pos;\n"
        "}\n";
}

const char *zero_shader_default_frag(void) {
    return 
        "#version 330 core\n"
        "in vec3 v_normal;\n"
        "in vec3 v_world_pos;\n"
        "in vec2 v_uv;\n"
        "uniform vec3 u_light_dir;\n"
        "uniform vec3 u_light_color;\n"
        "uniform vec3 u_ambient_color;\n"
        "uniform vec3 u_object_color;\n"
        "uniform vec3 u_camera_pos;\n"
        "out vec4 frag_color;\n"
        "void main() {\n"
        "    vec3 norm = normalize(v_normal);\n"
        "    vec3 light_dir = normalize(-u_light_dir);\n"
        "    float diff = max(dot(norm, light_dir), 0.0);\n"
        "    vec3 diffuse = diff * u_light_color;\n"
        "    vec3 view_dir = normalize(u_camera_pos - v_world_pos);\n"
        "    vec3 half_dir = normalize(light_dir + view_dir);\n"
        "    float spec = pow(max(dot(norm, half_dir), 0.0), 32.0);\n"
        "    vec3 specular = spec * u_light_color * 0.3;\n"
        "    vec3 color = (u_ambient_color + diffuse + specular) * u_object_color;\n"
        "    frag_color = vec4(color, 1.0);\n"
        "}\n";
}

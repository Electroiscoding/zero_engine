#include "zero_ui.h"
#include "zero_gl_loader.h"
#include "zero_shader.h"
#include "zero_font_atlas.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

static const char *ui_vert_src(void) {
    return
        "#version 330 core\n"
        "layout(location = 0) in vec2 a_pos;\n"
        "layout(location = 1) in vec2 a_uv;\n"
        "layout(location = 2) in vec4 a_color;\n"
        "layout(location = 3) in float a_mode;\n"
        "uniform vec3 u_screen;\n"
        "out vec2 v_uv;\n"
        "out vec4 v_color;\n"
        "out float v_mode;\n"
        "void main() {\n"
        "    vec2 ndc = (a_pos / u_screen.xy) * 2.0 - 1.0;\n"
        "    ndc.y = -ndc.y;\n"
        "    gl_Position = vec4(ndc, 0.0, 1.0);\n"
        "    v_uv = a_uv;\n"
        "    v_color = a_color;\n"
        "    v_mode = a_mode;\n"
        "}\n";
}

static const char *ui_frag_src(void) {
    return
        "#version 330 core\n"
        "in vec2 v_uv;\n"
        "in vec4 v_color;\n"
        "in float v_mode;\n"
        "uniform sampler2D u_font;\n"
        "out vec4 frag_color;\n"
        "void main() {\n"
        "    if (v_mode > 0.5) {\n"
        "        float a = texture(u_font, v_uv).r;\n"
        "        frag_color = vec4(v_color.rgb, v_color.a * a);\n"
        "    } else {\n"
        "        frag_color = v_color;\n"
        "    }\n"
        "}\n";
}

bool zero_ui_init(ZeroUI *ui) {
    if (!ui) return false;
    memset(ui, 0, sizeof(*ui));

    ui->vertices = (ZeroUIVertex *)malloc(ZERO_UI_MAX_VERTS * sizeof(ZeroUIVertex));
    ui->indices = (uint32_t *)malloc(ZERO_UI_MAX_INDICES * sizeof(uint32_t));
    if (!ui->vertices || !ui->indices) {
        zero_ui_shutdown(ui);
        return false;
    }

    if (!zgl.glGenVertexArrays) {
        return true;
    }

    ZeroShader s = zero_shader_create(ui_vert_src(), ui_frag_src());
    if (!s.is_valid) {
        zero_ui_shutdown(ui);
        return false;
    }
    ui->program = s.program_id;
    ui->u_screen_loc = zgl.glGetUniformLocation(ui->program, "u_screen");

    zgl.glGenVertexArrays(1, &ui->vao);
    zgl.glGenBuffers(1, &ui->vbo);
    zgl.glGenBuffers(1, &ui->ibo);

    zgl.glBindVertexArray(ui->vao);

    zgl.glBindBuffer(GL_ARRAY_BUFFER, ui->vbo);
    zgl.glBufferData(GL_ARRAY_BUFFER, ZERO_UI_MAX_VERTS * sizeof(ZeroUIVertex), NULL, GL_DYNAMIC_DRAW);

    zgl.glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ui->ibo);
    zgl.glBufferData(GL_ELEMENT_ARRAY_BUFFER, ZERO_UI_MAX_INDICES * sizeof(uint32_t), NULL, GL_DYNAMIC_DRAW);

    GLsizei stride = (GLsizei)sizeof(ZeroUIVertex);
    zgl.glEnableVertexAttribArray(0);
    zgl.glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, stride, (void *)offsetof(ZeroUIVertex, pos));

    zgl.glEnableVertexAttribArray(1);
    zgl.glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, stride, (void *)offsetof(ZeroUIVertex, uv));

    zgl.glEnableVertexAttribArray(2);
    zgl.glVertexAttribPointer(2, 4, GL_FLOAT, GL_FALSE, stride, (void *)offsetof(ZeroUIVertex, color));

    zgl.glEnableVertexAttribArray(3);
    zgl.glVertexAttribPointer(3, 1, GL_FLOAT, GL_FALSE, stride, (void *)offsetof(ZeroUIVertex, mode));

    zgl.glBindVertexArray(0);

    glGenTextures(1, &ui->font_tex);
    glBindTexture(GL_TEXTURE_2D, ui->font_tex);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RED, 128, 128, 0, GL_RED, GL_UNSIGNED_BYTE, zero_font_atlas);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glBindTexture(GL_TEXTURE_2D, 0);

    return true;
}

void zero_ui_shutdown(ZeroUI *ui) {
    if (!ui) return;
    if (ui->program && zgl.glDeleteProgram) {
        zgl.glDeleteProgram(ui->program);
        ui->program = 0;
    }
    if (ui->vao && zgl.glDeleteVertexArrays) {
        zgl.glDeleteVertexArrays(1, &ui->vao);
        ui->vao = 0;
    }
    if (ui->vbo && zgl.glDeleteBuffers) {
        zgl.glDeleteBuffers(1, &ui->vbo);
        ui->vbo = 0;
    }
    if (ui->ibo && zgl.glDeleteBuffers) {
        zgl.glDeleteBuffers(1, &ui->ibo);
        ui->ibo = 0;
    }
    if (ui->font_tex) {
        glDeleteTextures(1, &ui->font_tex);
        ui->font_tex = 0;
    }
    if (ui->vertices) {
        free(ui->vertices);
        ui->vertices = NULL;
    }
    if (ui->indices) {
        free(ui->indices);
        ui->indices = NULL;
    }
}

void zero_ui_begin(ZeroUI *ui, float screen_w, float screen_h, float mouse_x, float mouse_y, bool mouse_down) {
    if (!ui) return;
    ui->screen_w = screen_w;
    ui->screen_h = screen_h;
    ui->mouse_x = mouse_x;
    ui->mouse_y = mouse_y;
    ui->mouse_down = mouse_down;
    ui->mouse_clicked = (mouse_down && !ui->prev_mouse_down);
    ui->prev_mouse_down = mouse_down;

    ui->vertex_count = 0;
    ui->index_count = 0;
    ui->mouse_over_ui = false;
}

static void push_quad(ZeroUI *ui, Vec2 p0, Vec2 p1, Vec2 p2, Vec2 p3,
                      Vec2 uv0, Vec2 uv1, Vec2 uv2, Vec2 uv3,
                      Vec4 color, float mode) {
    if (ui->vertex_count + 4 > ZERO_UI_MAX_VERTS || ui->index_count + 6 > ZERO_UI_MAX_INDICES) {
        return;
    }

    uint32_t base = ui->vertex_count;
    ui->vertices[base + 0] = (ZeroUIVertex){ p0, uv0, color, mode };
    ui->vertices[base + 1] = (ZeroUIVertex){ p1, uv1, color, mode };
    ui->vertices[base + 2] = (ZeroUIVertex){ p2, uv2, color, mode };
    ui->vertices[base + 3] = (ZeroUIVertex){ p3, uv3, color, mode };
    ui->vertex_count += 4;

    ui->indices[ui->index_count++] = base + 0;
    ui->indices[ui->index_count++] = base + 1;
    ui->indices[ui->index_count++] = base + 2;
    ui->indices[ui->index_count++] = base + 2;
    ui->indices[ui->index_count++] = base + 3;
    ui->indices[ui->index_count++] = base + 0;
}

void zero_ui_draw_rect(ZeroUI *ui, float x, float y, float w, float h, Vec4 color) {
    if (!ui || w <= 0.0f || h <= 0.0f) return;
    Vec2 p0 = vec2(x, y);
    Vec2 p1 = vec2(x + w, y);
    Vec2 p2 = vec2(x + w, y + h);
    Vec2 p3 = vec2(x, y + h);
    Vec2 uv = vec2(0, 0);
    push_quad(ui, p0, p1, p2, p3, uv, uv, uv, uv, color, 0.0f);
}

void zero_ui_draw_rect_outline(ZeroUI *ui, float x, float y, float w, float h, Vec4 color, float thickness) {
    if (!ui || w <= 0.0f || h <= 0.0f) return;
    zero_ui_draw_rect(ui, x, y, w, thickness, color);
    zero_ui_draw_rect(ui, x, y + h - thickness, w, thickness, color);
    zero_ui_draw_rect(ui, x, y + thickness, thickness, h - 2.0f * thickness, color);
    zero_ui_draw_rect(ui, x + w - thickness, y + thickness, thickness, h - 2.0f * thickness, color);
}

void zero_ui_draw_text(ZeroUI *ui, float x, float y, const char *text, Vec4 color, float scale) {
    if (!ui || !text || scale <= 0.0f) return;
    float cur_x = floorf(x);
    float cur_y = floorf(y);
    float char_adv = floorf(7.0f * scale);
    float glyph_w = floorf(8.0f * scale);
    float glyph_h = floorf(16.0f * scale);

    for (const char *p = text; *p; ++p) {
        char c = *p;
        if (c == '\n') {
            cur_x = floorf(x);
            cur_y += glyph_h;
            continue;
        }
        if ((unsigned char)c < 32 || (unsigned char)c > 126) {
            c = ' ';
        }
        uint32_t idx = (uint32_t)((unsigned char)c - 32);
        uint32_t col = idx % 16;
        uint32_t row = idx / 16;

        float u0 = (float)(col * 8) / 128.0f;
        float v0 = (float)(row * 16) / 128.0f;
        float u1 = (float)((col + 1) * 8) / 128.0f;
        float v1 = (float)((row + 1) * 16) / 128.0f;

        Vec2 p0 = vec2(cur_x, cur_y);
        Vec2 p1 = vec2(cur_x + glyph_w, cur_y);
        Vec2 p2 = vec2(cur_x + glyph_w, cur_y + glyph_h);
        Vec2 p3 = vec2(cur_x, cur_y + glyph_h);

        Vec2 uv0 = vec2(u0, v0);
        Vec2 uv1 = vec2(u1, v0);
        Vec2 uv2 = vec2(u1, v1);
        Vec2 uv3 = vec2(u0, v1);

        push_quad(ui, p0, p1, p2, p3, uv0, uv1, uv2, uv3, color, 1.0f);
        cur_x += char_adv;
    }
}

static bool point_in_rect(float px, float py, float x, float y, float w, float h) {
    return (px >= x && px <= x + w && py >= y && py <= y + h);
}

bool zero_ui_button(ZeroUI *ui, float x, float y, float w, float h, const char *label) {
    if (!ui) return false;
    bool hovered = point_in_rect(ui->mouse_x, ui->mouse_y, x, y, w, h);
    if (hovered) ui->mouse_over_ui = true;

    Vec4 bg = vec4(0.20f, 0.22f, 0.28f, 0.95f);
    Vec4 border = vec4(0.32f, 0.35f, 0.44f, 1.0f);
    Vec4 text_col = vec4(0.88f, 0.90f, 0.95f, 1.0f);

    if (hovered) {
        if (ui->mouse_down) {
            bg = vec4(0.12f, 0.38f, 0.65f, 1.0f);
            border = vec4(0.25f, 0.55f, 0.85f, 1.0f);
        } else {
            bg = vec4(0.28f, 0.31f, 0.40f, 1.0f);
            border = vec4(0.45f, 0.50f, 0.62f, 1.0f);
        }
    }

    zero_ui_draw_rect(ui, x, y, w, h, bg);
    zero_ui_draw_rect_outline(ui, x, y, w, h, border, 1.0f);

    if (label) {
        size_t len = strlen(label);
        float tw = (float)len * 7.0f;
        float th = 16.0f;
        float tx = floorf(x + (w - tw) * 0.5f);
        float ty = floorf(y + (h - th) * 0.5f);
        zero_ui_draw_text(ui, tx, ty, label, text_col, 1.0f);
    }

    return (hovered && ui->mouse_clicked);
}

bool zero_ui_button_toggle(ZeroUI *ui, float x, float y, float w, float h, const char *label, bool active) {
    if (!ui) return false;
    bool hovered = point_in_rect(ui->mouse_x, ui->mouse_y, x, y, w, h);
    if (hovered) ui->mouse_over_ui = true;

    Vec4 bg = active ? vec4(0.18f, 0.48f, 0.78f, 1.0f) : vec4(0.18f, 0.20f, 0.25f, 0.95f);
    Vec4 border = active ? vec4(0.35f, 0.68f, 0.98f, 1.0f) : vec4(0.30f, 0.32f, 0.38f, 1.0f);
    Vec4 text_col = active ? vec4(1.0f, 1.0f, 1.0f, 1.0f) : vec4(0.75f, 0.78f, 0.85f, 1.0f);

    if (hovered && !active) {
        bg = vec4(0.25f, 0.28f, 0.35f, 1.0f);
    }

    zero_ui_draw_rect(ui, x, y, w, h, bg);
    zero_ui_draw_rect_outline(ui, x, y, w, h, border, 1.0f);

    if (label) {
        size_t len = strlen(label);
        float tw = (float)len * 7.0f;
        float th = 16.0f;
        float tx = floorf(x + (w - tw) * 0.5f);
        float ty = floorf(y + (h - th) * 0.5f);
        zero_ui_draw_text(ui, tx, ty, label, text_col, 1.0f);
    }

    return (hovered && ui->mouse_clicked);
}

bool zero_ui_menu_item(ZeroUI *ui, float x, float y, float w, float h, const char *label, bool selected) {
    if (!ui) return false;
    bool hovered = point_in_rect(ui->mouse_x, ui->mouse_y, x, y, w, h);
    if (hovered) ui->mouse_over_ui = true;

    Vec4 bg = vec4(0, 0, 0, 0);
    Vec4 text_col = vec4(0.82f, 0.85f, 0.90f, 1.0f);

    if (selected) {
        bg = vec4(0.15f, 0.42f, 0.72f, 0.9f);
        text_col = vec4(1.0f, 1.0f, 1.0f, 1.0f);
    } else if (hovered) {
        bg = vec4(0.25f, 0.28f, 0.35f, 0.8f);
    }

    if (bg.w > 0.0f) {
        zero_ui_draw_rect(ui, x, y, w, h, bg);
    }

    if (label) {
        zero_ui_draw_text(ui, x + 8.0f, floorf(y + (h - 16.0f) * 0.5f), label, text_col, 1.0f);
    }

    return (hovered && ui->mouse_clicked);
}

bool zero_ui_slider(ZeroUI *ui, float x, float y, float w, float h, const char *label, float *value, float min, float max) {
    if (!ui || !value || max <= min) return false;
    bool hovered = point_in_rect(ui->mouse_x, ui->mouse_y, x, y, w, h);
    if (hovered) ui->mouse_over_ui = true;

    bool changed = false;
    if (hovered && ui->mouse_down) {
        float rel = (ui->mouse_x - x) / w;
        if (rel < 0.0f) rel = 0.0f;
        if (rel > 1.0f) rel = 1.0f;
        *value = min + rel * (max - min);
        changed = true;
    }

    zero_ui_draw_rect(ui, x, y + h * 0.35f, w, h * 0.3f, vec4(0.12f, 0.14f, 0.18f, 1.0f));

    float norm = (*value - min) / (max - min);
    if (norm < 0.0f) norm = 0.0f;
    if (norm > 1.0f) norm = 1.0f;
    zero_ui_draw_rect(ui, x, y + h * 0.35f, w * norm, h * 0.3f, vec4(0.18f, 0.52f, 0.88f, 1.0f));

    float thumb_x = x + norm * (w - 10.0f);
    zero_ui_draw_rect(ui, thumb_x, y, 10.0f, h, vec4(0.85f, 0.88f, 0.95f, 1.0f));

    if (label) {
        char buf[64];
        snprintf(buf, sizeof(buf), "%s: %.2f", label, *value);
        zero_ui_draw_text(ui, x + 4.0f, y - 14.0f, buf, vec4(0.85f, 0.88f, 0.92f, 1.0f), 0.85f);
    }

    return changed;
}

void zero_ui_panel(ZeroUI *ui, float x, float y, float w, float h, const char *title) {
    if (!ui) return;
    if (point_in_rect(ui->mouse_x, ui->mouse_y, x, y, w, h)) {
        ui->mouse_over_ui = true;
    }

    zero_ui_draw_rect(ui, x, y, w, h, vec4(0.13f, 0.14f, 0.18f, 0.92f));
    zero_ui_draw_rect_outline(ui, x, y, w, h, vec4(0.24f, 0.26f, 0.32f, 1.0f), 1.0f);

    float bar_h = 24.0f;
    zero_ui_draw_rect(ui, x, y, w, bar_h, vec4(0.18f, 0.20f, 0.26f, 1.0f));
    zero_ui_draw_rect_outline(ui, x, y, w, bar_h, vec4(0.28f, 0.30f, 0.38f, 1.0f), 1.0f);

    if (title) {
        zero_ui_draw_text(ui, x + 8.0f, y + 4.0f, title, vec4(0.92f, 0.94f, 0.98f, 1.0f), 1.0f);
    }
}

void zero_ui_label(ZeroUI *ui, float x, float y, const char *text, Vec4 color) {
    if (!ui || !text) return;
    zero_ui_draw_text(ui, x, y, text, color, 1.0f);
}

void zero_ui_end(ZeroUI *ui) {
    if (!ui || ui->vertex_count == 0 || ui->index_count == 0) return;
    if (!zgl.glBindVertexArray) return;

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDisable(GL_DEPTH_TEST);

    zgl.glBindBuffer(GL_ARRAY_BUFFER, ui->vbo);
    zgl.glBufferData(GL_ARRAY_BUFFER, ui->vertex_count * sizeof(ZeroUIVertex), ui->vertices, GL_DYNAMIC_DRAW);

    zgl.glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ui->ibo);
    zgl.glBufferData(GL_ELEMENT_ARRAY_BUFFER, ui->index_count * sizeof(uint32_t), ui->indices, GL_DYNAMIC_DRAW);

    zgl.glUseProgram(ui->program);
    zgl.glUniform3f(ui->u_screen_loc, ui->screen_w, ui->screen_h, 0.0f);

    GLint u_font_loc = zgl.glGetUniformLocation(ui->program, "u_font");
    zgl.glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, ui->font_tex);
    if (u_font_loc >= 0) {
        zgl.glUniform1i(u_font_loc, 0);
    }

    zgl.glBindVertexArray(ui->vao);
    glDrawElements(GL_TRIANGLES, (GLsizei)ui->index_count, GL_UNSIGNED_INT, 0);
    zgl.glBindVertexArray(0);

    glEnable(GL_DEPTH_TEST);
}

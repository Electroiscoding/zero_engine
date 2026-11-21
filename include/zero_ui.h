#ifndef ZERO_UI_H
#define ZERO_UI_H

#include "zero_core.h"
#include "zero_math.h"
#include <stdbool.h>
#include <stdint.h>

#define ZERO_UI_MAX_VERTS 65536
#define ZERO_UI_MAX_INDICES 98304

typedef struct ZeroUIVertex {
    Vec2 pos;
    Vec2 uv;
    Vec4 color;
    float mode; // 0.0 = solid color quad, 1.0 = textured font glyph
} ZeroUIVertex;

typedef struct ZeroUI {
    uint32_t vao;
    uint32_t vbo;
    uint32_t ibo;
    uint32_t font_tex;
    uint32_t program;
    int32_t u_screen_loc;

    ZeroUIVertex *vertices;
    uint32_t *indices;
    uint32_t vertex_count;
    uint32_t index_count;

    float screen_w;
    float screen_h;
    float mouse_x;
    float mouse_y;
    bool mouse_down;
    bool mouse_clicked;
    bool prev_mouse_down;

    bool mouse_over_ui;
} ZeroUI;

bool zero_ui_init(ZeroUI *ui);
void zero_ui_shutdown(ZeroUI *ui);

void zero_ui_begin(ZeroUI *ui, float screen_w, float screen_h, float mouse_x, float mouse_y, bool mouse_down);
void zero_ui_end(ZeroUI *ui);

void zero_ui_draw_rect(ZeroUI *ui, float x, float y, float w, float h, Vec4 color);
void zero_ui_draw_rect_outline(ZeroUI *ui, float x, float y, float w, float h, Vec4 color, float thickness);
void zero_ui_draw_text(ZeroUI *ui, float x, float y, const char *text, Vec4 color, float scale);

bool zero_ui_button(ZeroUI *ui, float x, float y, float w, float h, const char *label);
bool zero_ui_button_toggle(ZeroUI *ui, float x, float y, float w, float h, const char *label, bool active);
bool zero_ui_menu_item(ZeroUI *ui, float x, float y, float w, float h, const char *label, bool selected);
bool zero_ui_slider(ZeroUI *ui, float x, float y, float w, float h, const char *label, float *value, float min, float max);
void zero_ui_panel(ZeroUI *ui, float x, float y, float w, float h, const char *title);
void zero_ui_label(ZeroUI *ui, float x, float y, const char *text, Vec4 color);

#endif // ZERO_UI_H

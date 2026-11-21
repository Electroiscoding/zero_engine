#include "zero_race_ui.h"
#include "zero_track_spline.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* --------------------------------------------------------------------------
 * Static Catalogs: Cars & Tracks
 * -------------------------------------------------------------------------- */

static const ZeroCarConfig s_car_catalog[4] = {
    {
        .name = "Apex GT",
        .tagline = "Balanced Grand Touring Specialist",
        .max_speed_kmh = 285.0f,
        .acceleration_rating = 0.85f,
        .handling_rating = 0.88f,
        .body_color = { 0.95f, 0.15f, 0.10f },
        .mass = 1250.0f,
        .engine_power = 580.0f
    },
    {
        .name = "Phantom R",
        .tagline = "High-Downforce Aero Interceptor",
        .max_speed_kmh = 310.0f,
        .acceleration_rating = 0.92f,
        .handling_rating = 0.78f,
        .body_color = { 0.12f, 0.16f, 0.28f },
        .mass = 1380.0f,
        .engine_power = 650.0f
    },
    {
        .name = "Drift King",
        .tagline = "Rear-Drive Slip-Angle Legend",
        .max_speed_kmh = 265.0f,
        .acceleration_rating = 0.80f,
        .handling_rating = 0.95f,
        .body_color = { 0.98f, 0.72f, 0.05f },
        .mass = 1180.0f,
        .engine_power = 520.0f
    },
    {
        .name = "Horizon Turbo",
        .tagline = "Twin-Turbo All-Wheel Beast",
        .max_speed_kmh = 325.0f,
        .acceleration_rating = 0.96f,
        .handling_rating = 0.82f,
        .body_color = { 0.05f, 0.82f, 0.78f },
        .mass = 1320.0f,
        .engine_power = 700.0f
    }
};

static const ZeroTrackConfig s_track_catalog[3] = {
    {
        .name = "Apex Grand Prix",
        .location = "Mount Apex Circuit",
        .lap_count = 3,
        .length_meters = 3420.0f,
        .time_of_day_hours = 14.5f
    },
    {
        .name = "Sunset Ridge",
        .location = "Pacific Coastline Highway",
        .lap_count = 2,
        .length_meters = 4150.0f,
        .time_of_day_hours = 18.0f
    },
    {
        .name = "Midnight Sprint",
        .location = "Neo-Tokyo Expressway",
        .lap_count = 4,
        .length_meters = 2880.0f,
        .time_of_day_hours = 23.5f
    }
};

int zero_game_ui_get_car_count(void) {
    return 4;
}

const ZeroCarConfig *zero_game_ui_get_car(int index) {
    if (index < 0 || index >= 4) return &s_car_catalog[0];
    return &s_car_catalog[index];
}

int zero_game_ui_get_track_count(void) {
    return 3;
}

const ZeroTrackConfig *zero_game_ui_get_track(int index) {
    if (index < 0 || index >= 3) return &s_track_catalog[0];
    return &s_track_catalog[index];
}

/* --------------------------------------------------------------------------
 * Navigation & State Accessors
 * -------------------------------------------------------------------------- */

void zero_game_ui_set_state(ZeroGameUI *gui, ZeroGameFlowState state) {
    if (!gui) return;
    gui->prev_state = gui->state;
    gui->state = state;
}

bool zero_game_ui_is_restart_requested(const ZeroGameUI *gui) {
    return gui ? gui->restart_requested : false;
}

void zero_game_ui_clear_restart_request(ZeroGameUI *gui) {
    if (gui) gui->restart_requested = false;
}

bool zero_game_ui_is_quit_requested(const ZeroGameUI *gui) {
    return gui ? gui->quit_requested : false;
}

void zero_game_ui_clear_quit_request(ZeroGameUI *gui) {
    if (gui) gui->quit_requested = false;
}

void zero_game_ui_set_podium_entry(ZeroGameUI *gui, int index, const char *driver,
                                   const char *car, float total_time, float best_lap, int pos) {
    if (!gui || index < 0 || index >= 4) return;
    if (driver) {
        snprintf(gui->podium[index].driver_name, sizeof(gui->podium[index].driver_name), "%s", driver);
    }
    if (car) {
        snprintf(gui->podium[index].car_name, sizeof(gui->podium[index].car_name), "%s", car);
    }
    gui->podium[index].total_time = total_time;
    gui->podium[index].best_lap = best_lap;
    gui->podium[index].finish_position = pos;
}

/* --------------------------------------------------------------------------
 * Internal Geometry & Drawing Helpers
 * -------------------------------------------------------------------------- */

static void push_colored_quad(ZeroUI *ui, Vec2 p0, Vec2 p1, Vec2 p2, Vec2 p3, Vec4 color) {
    if (!ui || !ui->vertices || !ui->indices) return;
    if (ui->vertex_count + 4 > ZERO_UI_MAX_VERTS || ui->index_count + 6 > ZERO_UI_MAX_INDICES) return;

    uint32_t base = ui->vertex_count;
    Vec2 uv = vec2(0.0f, 0.0f);
    ui->vertices[base + 0] = (ZeroUIVertex){ p0, uv, color, 0.0f };
    ui->vertices[base + 1] = (ZeroUIVertex){ p1, uv, color, 0.0f };
    ui->vertices[base + 2] = (ZeroUIVertex){ p2, uv, color, 0.0f };
    ui->vertices[base + 3] = (ZeroUIVertex){ p3, uv, color, 0.0f };
    ui->vertex_count += 4;

    ui->indices[ui->index_count++] = base + 0;
    ui->indices[ui->index_count++] = base + 1;
    ui->indices[ui->index_count++] = base + 2;
    ui->indices[ui->index_count++] = base + 2;
    ui->indices[ui->index_count++] = base + 3;
    ui->indices[ui->index_count++] = base + 0;
}

static void format_lap_time(float seconds, char *buf, size_t size) {
    if (seconds < 0.0f) seconds = 0.0f;
    int minutes = (int)(seconds / 60.0f);
    float rem = seconds - (float)(minutes * 60);
    int sec = (int)rem;
    int ms = (int)((rem - (float)sec) * 1000.0f);
    if (ms > 999) ms = 999;
    snprintf(buf, size, "%02d:%02d.%03d", minutes, sec, ms);
}

static void sample_spline_point(const float spline[][2], int count, float t, float *out_x, float *out_z) {
    if (count <= 0) {
        *out_x = 0.0f; *out_z = 0.0f;
        return;
    }
    if (count == 1) {
        *out_x = spline[0][0]; *out_z = spline[0][1];
        return;
    }

    float idx_f = t;
    if (idx_f <= 1.0001f && idx_f >= 0.0f) {
        idx_f = t * (float)count;
    }
    while (idx_f < 0.0f) idx_f += (float)count;
    while (idx_f >= (float)count) idx_f -= (float)count;

    int i0 = (int)floorf(idx_f);
    if (i0 < 0) i0 = 0;
    if (i0 >= count) i0 = count - 1;
    int i1 = (i0 + 1) % count;
    float frac = idx_f - floorf(idx_f);

    *out_x = spline[i0][0] + (spline[i1][0] - spline[i0][0]) * frac;
    *out_z = spline[i0][1] + (spline[i1][1] - spline[i0][1]) * frac;
}

/* --------------------------------------------------------------------------
 * Procedural Widgets: Arc Gauge, Segmented Bar, Track Radar
 * -------------------------------------------------------------------------- */

void zero_ui_draw_arc_gauge(ZeroUI *ui, float center_x, float center_y, float radius,
                            float start_angle_rad, float end_angle_rad,
                            float value_pct, Vec4 bar_color, Vec4 bg_color, float thickness) {
    if (!ui || radius <= 0.0f || thickness <= 0.0f) return;
    if (value_pct < 0.0f) value_pct = 0.0f;
    if (value_pct > 1.0f) value_pct = 1.0f;

    float r_in = radius - thickness * 0.5f;
    if (r_in < 1.0f) r_in = 1.0f;
    float r_out = radius + thickness * 0.5f;

    const int SEGMENTS = 48;
    float total_arc = end_angle_rad - start_angle_rad;

    /* Background Arc */
    if (bg_color.w > 0.001f) {
        for (int i = 0; i < SEGMENTS; ++i) {
            float a0 = start_angle_rad + total_arc * ((float)i / (float)SEGMENTS);
            float a1 = start_angle_rad + total_arc * ((float)(i + 1) / (float)SEGMENTS);

            Vec2 p0 = vec2(center_x + cosf(a0) * r_in, center_y + sinf(a0) * r_in);
            Vec2 p1 = vec2(center_x + cosf(a0) * r_out, center_y + sinf(a0) * r_out);
            Vec2 p2 = vec2(center_x + cosf(a1) * r_out, center_y + sinf(a1) * r_out);
            Vec2 p3 = vec2(center_x + cosf(a1) * r_in, center_y + sinf(a1) * r_in);

            push_colored_quad(ui, p0, p1, p2, p3, bg_color);
        }
    }

    /* Active Value Arc */
    if (value_pct > 0.001f && bar_color.w > 0.001f) {
        int active_segs = (int)ceilf((float)SEGMENTS * value_pct);
        if (active_segs > SEGMENTS) active_segs = SEGMENTS;

        for (int i = 0; i < active_segs; ++i) {
            float f0 = (float)i / (float)SEGMENTS;
            float f1 = (float)(i + 1) / (float)SEGMENTS;
            if (f1 > value_pct) f1 = value_pct;
            if (f1 <= f0) break;

            float a0 = start_angle_rad + total_arc * f0;
            float a1 = start_angle_rad + total_arc * f1;

            Vec2 p0 = vec2(center_x + cosf(a0) * r_in, center_y + sinf(a0) * r_in);
            Vec2 p1 = vec2(center_x + cosf(a0) * r_out, center_y + sinf(a0) * r_out);
            Vec2 p2 = vec2(center_x + cosf(a1) * r_out, center_y + sinf(a1) * r_out);
            Vec2 p3 = vec2(center_x + cosf(a1) * r_in, center_y + sinf(a1) * r_in);

            push_colored_quad(ui, p0, p1, p2, p3, bar_color);
        }
    }
}

void zero_ui_draw_segmented_bar(ZeroUI *ui, float x, float y, float w, float h,
                                int num_segments, float value_pct,
                                Vec4 active_color, Vec4 inactive_color, Vec4 redline_color) {
    if (!ui || num_segments <= 0 || w <= 0.0f || h <= 0.0f) return;
    if (value_pct < 0.0f) value_pct = 0.0f;
    if (value_pct > 1.0f) value_pct = 1.0f;

    float gap = (w / (float)num_segments > 5.0f) ? 2.0f : 1.0f;
    float total_gap = gap * (float)(num_segments - 1);
    float seg_w = (w - total_gap) / (float)num_segments;
    if (seg_w < 1.0f) seg_w = 1.0f;

    float filled = value_pct * (float)num_segments;
    int redline_start = (int)floorf((float)num_segments * 0.75f);

    for (int i = 0; i < num_segments; ++i) {
        float sx = x + (float)i * (seg_w + gap);
        Vec4 col;
        if ((float)i + 0.5f <= filled) {
            col = (i >= redline_start) ? redline_color : active_color;
        } else {
            col = inactive_color;
        }
        zero_ui_draw_rect(ui, sx, y, seg_w, h, col);
    }
}

void zero_ui_draw_track_radar(ZeroUI *ui, float x, float y, float w, float h,
                              const float spline[][2], int point_count,
                              float player_t, const float ai_t[3]) {
    if (!ui || w <= 10.0f || h <= 10.0f || !spline || point_count < 2) return;

    /* Radar background panel */
    zero_ui_draw_rect(ui, x, y, w, h, vec4(0.08f, 0.10f, 0.14f, 0.88f));
    zero_ui_draw_rect_outline(ui, x, y, w, h, vec4(0.25f, 0.30f, 0.42f, 0.90f), 1.0f);
    zero_ui_draw_text(ui, x + 6.0f, y + 4.0f, "CIRCUIT RADAR", vec4(0.70f, 0.75f, 0.85f, 0.90f), 0.70f);

    /* Find bounding box */
    float min_x = spline[0][0], max_x = spline[0][0];
    float min_z = spline[0][1], max_z = spline[0][1];
    for (int i = 1; i < point_count; ++i) {
        if (spline[i][0] < min_x) min_x = spline[i][0];
        if (spline[i][0] > max_x) max_x = spline[i][0];
        if (spline[i][1] < min_z) min_z = spline[i][1];
        if (spline[i][1] > max_z) max_z = spline[i][1];
    }

    float track_w = max_x - min_x;
    float track_h = max_z - min_z;
    if (track_w < 1.0f) track_w = 1.0f;
    if (track_h < 1.0f) track_h = 1.0f;

    float center_tx = (min_x + max_x) * 0.5f;
    float center_tz = (min_z + max_z) * 0.5f;

    float pad = 12.0f;
    float top_pad = 18.0f;
    float avail_w = w - 2.0f * pad;
    float avail_h = h - pad - top_pad;
    if (avail_w < 1.0f) avail_w = 1.0f;
    if (avail_h < 1.0f) avail_h = 1.0f;

    float scale_x = avail_w / track_w;
    float scale_z = avail_h / track_h;
    float scale = (scale_x < scale_z) ? scale_x : scale_z;

    float radar_cx = x + w * 0.5f;
    float radar_cy = y + top_pad + avail_h * 0.5f;

    /* Draw track outline segments */
    Vec4 line_col = vec4(0.38f, 0.44f, 0.55f, 0.85f);
    for (int i = 0; i < point_count; ++i) {
        int next = (i + 1) % point_count;
        float p0x = radar_cx + (spline[i][0] - center_tx) * scale;
        float p0y = radar_cy + (spline[i][1] - center_tz) * scale;
        float p1x = radar_cx + (spline[next][0] - center_tx) * scale;
        float p1y = radar_cy + (spline[next][1] - center_tz) * scale;

        float dx = p1x - p0x;
        float dy = p1y - p0y;
        float len = sqrtf(dx * dx + dy * dy);
        if (len > 0.001f) {
            float nx = -dy / len * 1.0f;
            float ny = dx / len * 1.0f;
            Vec2 q0 = vec2(p0x + nx, p0y + ny);
            Vec2 q1 = vec2(p1x + nx, p1y + ny);
            Vec2 q2 = vec2(p1x - nx, p1y - ny);
            Vec2 q3 = vec2(p0x - nx, p0y - ny);
            push_colored_quad(ui, q0, q1, q2, q3, line_col);
        }
    }

    /* Start / finish line marker */
    float s0x = radar_cx + (spline[0][0] - center_tx) * scale;
    float s0y = radar_cy + (spline[0][1] - center_tz) * scale;
    zero_ui_draw_rect(ui, s0x - 2.0f, s0y - 2.0f, 5.0f, 5.0f, vec4(1.0f, 1.0f, 1.0f, 0.95f));

    /* Draw AI cars */
    if (ai_t) {
        for (int k = 0; k < 3; ++k) {
            float ax, az;
            sample_spline_point(spline, point_count, ai_t[k], &ax, &az);
            float sx = radar_cx + (ax - center_tx) * scale;
            float sy = radar_cy + (az - center_tz) * scale;
            zero_ui_draw_rect(ui, sx - 2.5f, sy - 2.5f, 5.0f, 5.0f, vec4(1.0f, 0.28f, 0.22f, 1.0f));
        }
    }

    /* Draw Player car */
    float px, pz;
    sample_spline_point(spline, point_count, player_t, &px, &pz);
    float psx = radar_cx + (px - center_tx) * scale;
    float psy = radar_cy + (pz - center_tz) * scale;
    zero_ui_draw_rect(ui, psx - 4.0f, psy - 4.0f, 8.0f, 8.0f, vec4(0.0f, 0.95f, 1.0f, 1.0f));
    zero_ui_draw_rect(ui, psx - 1.5f, psy - 1.5f, 3.0f, 3.0f, vec4(1.0f, 1.0f, 1.0f, 1.0f));
}

/* --------------------------------------------------------------------------
 * Lifecycle Implementation
 * -------------------------------------------------------------------------- */

bool zero_game_ui_init(ZeroGameUI *gui) {
    if (!gui) return false;
    memset(gui, 0, sizeof(*gui));

    if (!zero_ui_init(&gui->ui)) {
        return false;
    }

    gui->state = ZERO_STATE_MAIN_MENU;
    gui->prev_state = ZERO_STATE_MAIN_MENU;
    gui->selected_car_idx = 0;
    gui->selected_track_idx = 0;
    gui->countdown_timer = 3.0f;
    gui->prev_countdown_timer = 3.0f;
    gui->show_minimap = true;
    gui->active_camera_mode = 0;

    gui->audio_settings.master_volume = 0.85f;
    gui->audio_settings.engine_volume = 0.80f;
    gui->audio_settings.sfx_volume = 0.90f;
    gui->audio_settings.is_muted = false;

    /* Preset initial podium records */
    const char *default_drivers[4] = { "Player", "Apex AI 1", "Apex AI 2", "Apex AI 3" };
    const char *default_cars[4] = { "Apex GT", "Phantom R", "Drift King", "Horizon Turbo" };
    float default_totals[4] = { 246.320f, 248.150f, 251.840f, 255.400f };
    float default_bests[4] = { 81.420f, 82.010f, 83.150f, 84.620f };

    for (int i = 0; i < 4; ++i) {
        snprintf(gui->podium[i].driver_name, sizeof(gui->podium[i].driver_name), "%s", default_drivers[i]);
        snprintf(gui->podium[i].car_name, sizeof(gui->podium[i].car_name), "%s", default_cars[i]);
        gui->podium[i].total_time = default_totals[i];
        gui->podium[i].best_lap = default_bests[i];
        gui->podium[i].finish_position = i + 1;
    }

    return true;
}

void zero_game_ui_shutdown(ZeroGameUI *gui) {
    if (!gui) return;
    zero_ui_shutdown(&gui->ui);
    memset(gui, 0, sizeof(*gui));
}

/* --------------------------------------------------------------------------
 * Page Renderers
 * -------------------------------------------------------------------------- */

void zero_game_ui_render_main_menu(ZeroGameUI *gui, float screen_w, float screen_h) {
    if (!gui) return;

    /* Subtle backdrop tint */
    zero_ui_draw_rect(&gui->ui, 0, 0, screen_w, screen_h, vec4(0.04f, 0.05f, 0.08f, 0.65f));

    /* Title banner */
    float title_y = screen_h * 0.16f;
    zero_ui_draw_text(&gui->ui, screen_w * 0.5f - 240.0f, title_y,
                      "ZERO DRIFT: APEX", vec4(0.98f, 0.78f, 0.12f, 1.0f), 3.0f);
    zero_ui_draw_text(&gui->ui, screen_w * 0.5f - 200.0f, title_y + 56.0f,
                      "HIGH-PERFORMANCE 3D RACING SIMULATION", vec4(0.70f, 0.75f, 0.85f, 0.90f), 1.0f);

    /* Menu panel & buttons */
    float btn_w = 260.0f;
    float btn_h = 42.0f;
    float btn_x = screen_w * 0.5f - btn_w * 0.5f;
    float start_y = title_y + 115.0f;
    float spacing = 52.0f;

    if (zero_ui_button(&gui->ui, btn_x, start_y + spacing * 0.0f, btn_w, btn_h, "START RACE")) {
        gui->menu_beep_triggered = true;
        gui->countdown_timer = 3.0f;
        gui->prev_countdown_timer = 3.0f;
        gui->state = ZERO_STATE_COUNTDOWN;
    }

    if (zero_ui_button(&gui->ui, btn_x, start_y + spacing * 1.0f, btn_w, btn_h, "CAR SELECTION")) {
        gui->menu_beep_triggered = true;
        gui->state = ZERO_STATE_CAR_SELECT;
    }

    if (zero_ui_button(&gui->ui, btn_x, start_y + spacing * 2.0f, btn_w, btn_h, "TRACK SELECTION")) {
        gui->menu_beep_triggered = true;
        gui->state = ZERO_STATE_TRACK_SELECT;
    }

    if (zero_ui_button(&gui->ui, btn_x, start_y + spacing * 3.0f, btn_w, btn_h, "SETTINGS")) {
        gui->menu_beep_triggered = true;
        gui->prev_state = ZERO_STATE_MAIN_MENU;
        gui->state = ZERO_STATE_SETTINGS;
    }

    if (zero_ui_button(&gui->ui, btn_x, start_y + spacing * 4.0f, btn_w, btn_h, "QUIT")) {
        gui->menu_beep_triggered = true;
        gui->quit_requested = true;
    }

    /* Selected configuration preview card */
    const ZeroCarConfig *car = zero_game_ui_get_car(gui->selected_car_idx);
    const ZeroTrackConfig *track = zero_game_ui_get_track(gui->selected_track_idx);

    float card_w = 420.0f;
    float card_h = 70.0f;
    float card_x = screen_w * 0.5f - card_w * 0.5f;
    float card_y = screen_h - card_h - 25.0f;

    zero_ui_draw_rect(&gui->ui, card_x, card_y, card_w, card_h, vec4(0.10f, 0.12f, 0.16f, 0.85f));
    zero_ui_draw_rect_outline(&gui->ui, card_x, card_y, card_w, card_h, vec4(0.25f, 0.30f, 0.40f, 0.90f), 1.0f);

    char car_buf[80];
    snprintf(car_buf, sizeof(car_buf), "ACTIVE CAR: %s (%s)", car->name, car->tagline);
    zero_ui_draw_text(&gui->ui, card_x + 12.0f, card_y + 12.0f, car_buf, vec4(0.92f, 0.92f, 0.95f, 1.0f), 0.90f);

    char trk_buf[80];
    snprintf(trk_buf, sizeof(trk_buf), "CIRCUIT:    %s - %d Laps (%.0fm)", track->name, track->lap_count, track->length_meters);
    zero_ui_draw_text(&gui->ui, card_x + 12.0f, card_y + 38.0f, trk_buf, vec4(0.65f, 0.75f, 0.88f, 1.0f), 0.90f);
}

void zero_game_ui_render_car_select(ZeroGameUI *gui, float screen_w, float screen_h) {
    if (!gui) return;

    /* Backdrop */
    zero_ui_draw_rect(&gui->ui, 0, 0, screen_w, screen_h, vec4(0.04f, 0.05f, 0.08f, 0.85f));

    /* Header */
    zero_ui_draw_text(&gui->ui, screen_w * 0.5f - 140.0f, 35.0f,
                      "CAR SELECTION", vec4(0.95f, 0.95f, 0.98f, 1.0f), 2.0f);

    const ZeroCarConfig *car = zero_game_ui_get_car(gui->selected_car_idx);

    /* Spec Card Panel */
    float panel_w = 540.0f;
    float panel_h = 340.0f;
    float panel_x = screen_w * 0.5f - panel_w * 0.5f;
    float panel_y = 90.0f;

    zero_ui_panel(&gui->ui, panel_x, panel_y, panel_w, panel_h, car->name);

    /* Tagline */
    zero_ui_draw_text(&gui->ui, panel_x + 20.0f, panel_y + 40.0f,
                      car->tagline, vec4(0.70f, 0.75f, 0.88f, 1.0f), 1.0f);

    /* Body Color preview swatch */
    zero_ui_draw_text(&gui->ui, panel_x + 20.0f, panel_y + 70.0f, "Livery Tone:", vec4(0.85f, 0.85f, 0.90f, 1.0f), 0.90f);
    Vec4 col_preview = vec4(car->body_color.x, car->body_color.y, car->body_color.z, 1.0f);
    zero_ui_draw_rect(&gui->ui, panel_x + 125.0f, panel_y + 68.0f, 40.0f, 18.0f, col_preview);
    zero_ui_draw_rect_outline(&gui->ui, panel_x + 125.0f, panel_y + 68.0f, 40.0f, 18.0f, vec4(0.8f, 0.8f, 0.8f, 1.0f), 1.0f);

    /* Numerical Specs */
    char buf[64];
    snprintf(buf, sizeof(buf), "Top Speed:    %.0f km/h", car->max_speed_kmh);
    zero_ui_draw_text(&gui->ui, panel_x + 20.0f, panel_y + 105.0f, buf, vec4(0.90f, 0.90f, 0.95f, 1.0f), 1.0f);

    snprintf(buf, sizeof(buf), "Engine Power: %.0f HP", car->engine_power);
    zero_ui_draw_text(&gui->ui, panel_x + 20.0f, panel_y + 130.0f, buf, vec4(0.90f, 0.90f, 0.95f, 1.0f), 1.0f);

    snprintf(buf, sizeof(buf), "Curb Weight:  %.0f kg", car->mass);
    zero_ui_draw_text(&gui->ui, panel_x + 20.0f, panel_y + 155.0f, buf, vec4(0.90f, 0.90f, 0.95f, 1.0f), 1.0f);

    /* Performance Rating Bars */
    zero_ui_draw_text(&gui->ui, panel_x + 20.0f, panel_y + 195.0f, "ACCELERATION", vec4(0.80f, 0.85f, 0.95f, 1.0f), 0.85f);
    zero_ui_draw_segmented_bar(&gui->ui, panel_x + 20.0f, panel_y + 215.0f, panel_w - 40.0f, 16.0f,
                               20, car->acceleration_rating,
                               vec4(0.18f, 0.65f, 0.95f, 1.0f), vec4(0.18f, 0.20f, 0.26f, 1.0f), vec4(0.95f, 0.25f, 0.20f, 1.0f));

    zero_ui_draw_text(&gui->ui, panel_x + 20.0f, panel_y + 245.0f, "HANDLING / GRIP", vec4(0.80f, 0.85f, 0.95f, 1.0f), 0.85f);
    zero_ui_draw_segmented_bar(&gui->ui, panel_x + 20.0f, panel_y + 265.0f, panel_w - 40.0f, 16.0f,
                               20, car->handling_rating,
                               vec4(0.18f, 0.85f, 0.45f, 1.0f), vec4(0.18f, 0.20f, 0.26f, 1.0f), vec4(0.95f, 0.75f, 0.15f, 1.0f));

    /* Quick Car Selection Buttons (Tabs) */
    float tab_w = 120.0f;
    float tab_h = 32.0f;
    float tab_start_x = screen_w * 0.5f - (tab_w * 4.0f + 30.0f) * 0.5f;
    float tab_y = panel_y + panel_h + 18.0f;

    for (int i = 0; i < 4; ++i) {
        float tx = tab_start_x + (float)i * (tab_w + 10.0f);
        const ZeroCarConfig *c = zero_game_ui_get_car(i);
        if (zero_ui_button_toggle(&gui->ui, tx, tab_y, tab_w, tab_h, c->name, (gui->selected_car_idx == i))) {
            gui->menu_beep_triggered = true;
            gui->selected_car_idx = i;
        }
    }

    /* Navigation Buttons */
    float nav_y = tab_y + 55.0f;
    float btn_w = 150.0f;
    float btn_h = 38.0f;

    if (zero_ui_button(&gui->ui, screen_w * 0.5f - 245.0f, nav_y, btn_w, btn_h, "< PREV CAR")) {
        gui->menu_beep_triggered = true;
        gui->selected_car_idx = (gui->selected_car_idx + 3) % 4;
    }

    if (zero_ui_button(&gui->ui, screen_w * 0.5f - 75.0f, nav_y, btn_w, btn_h, "NEXT CAR >")) {
        gui->menu_beep_triggered = true;
        gui->selected_car_idx = (gui->selected_car_idx + 1) % 4;
    }

    if (zero_ui_button(&gui->ui, screen_w * 0.5f + 95.0f, nav_y, btn_w, btn_h, "CONFIRM")) {
        gui->menu_beep_triggered = true;
        gui->countdown_timer = 3.0f;
        gui->prev_countdown_timer = 3.0f;
        gui->state = ZERO_STATE_COUNTDOWN;
    }

    if (zero_ui_button(&gui->ui, screen_w * 0.5f - 75.0f, nav_y + 48.0f, btn_w, btn_h, "BACK")) {
        gui->menu_beep_triggered = true;
        gui->state = ZERO_STATE_MAIN_MENU;
    }
}

void zero_game_ui_render_track_select(ZeroGameUI *gui, float screen_w, float screen_h) {
    if (!gui) return;

    /* Backdrop */
    zero_ui_draw_rect(&gui->ui, 0, 0, screen_w, screen_h, vec4(0.04f, 0.05f, 0.08f, 0.85f));

    /* Header */
    zero_ui_draw_text(&gui->ui, screen_w * 0.5f - 160.0f, 35.0f,
                      "TRACK SELECTION", vec4(0.95f, 0.95f, 0.98f, 1.0f), 2.0f);

    const ZeroTrackConfig *track = zero_game_ui_get_track(gui->selected_track_idx);

    /* Track Info Card */
    float panel_w = 540.0f;
    float panel_h = 320.0f;
    float panel_x = screen_w * 0.5f - panel_w * 0.5f;
    float panel_y = 90.0f;

    zero_ui_panel(&gui->ui, panel_x, panel_y, panel_w, panel_h, track->name);

    zero_ui_draw_text(&gui->ui, panel_x + 20.0f, panel_y + 40.0f,
                      track->location, vec4(0.70f, 0.75f, 0.88f, 1.0f), 1.0f);

    char buf[64];
    snprintf(buf, sizeof(buf), "Lap Count:     %d Laps", track->lap_count);
    zero_ui_draw_text(&gui->ui, panel_x + 20.0f, panel_y + 75.0f, buf, vec4(0.90f, 0.90f, 0.95f, 1.0f), 1.0f);

    snprintf(buf, sizeof(buf), "Circuit Length: %.0f meters", track->length_meters);
    zero_ui_draw_text(&gui->ui, panel_x + 20.0f, panel_y + 105.0f, buf, vec4(0.90f, 0.90f, 0.95f, 1.0f), 1.0f);

    snprintf(buf, sizeof(buf), "Time of Day:   %.1f Hours (Sun angle)", track->time_of_day_hours);
    zero_ui_draw_text(&gui->ui, panel_x + 20.0f, panel_y + 135.0f, buf, vec4(0.90f, 0.90f, 0.95f, 1.0f), 1.0f);

    /* Mini Track Preview inside the panel */
    float radar_x = panel_x + panel_w - 200.0f;
    float radar_y = panel_y + 45.0f;
    float radar_size = 180.0f;
    float dummy_ai[3] = { 0.25f, 0.50f, 0.75f };
    zero_ui_draw_track_radar(&gui->ui, radar_x, radar_y, radar_size, radar_size,
                             g_track_spline, ZERO_TRACK_SPLINE_COUNT, 0.0f, dummy_ai);

    /* Quick Track Selection Buttons (Tabs) */
    float tab_w = 160.0f;
    float tab_h = 32.0f;
    float tab_start_x = screen_w * 0.5f - (tab_w * 3.0f + 20.0f) * 0.5f;
    float tab_y = panel_y + panel_h + 18.0f;

    for (int i = 0; i < 3; ++i) {
        float tx = tab_start_x + (float)i * (tab_w + 10.0f);
        const ZeroTrackConfig *t = zero_game_ui_get_track(i);
        if (zero_ui_button_toggle(&gui->ui, tx, tab_y, tab_w, tab_h, t->name, (gui->selected_track_idx == i))) {
            gui->menu_beep_triggered = true;
            gui->selected_track_idx = i;
        }
    }

    /* Navigation Buttons */
    float nav_y = tab_y + 55.0f;
    float btn_w = 150.0f;
    float btn_h = 38.0f;

    if (zero_ui_button(&gui->ui, screen_w * 0.5f - 245.0f, nav_y, btn_w, btn_h, "< PREV TRACK")) {
        gui->menu_beep_triggered = true;
        gui->selected_track_idx = (gui->selected_track_idx + 2) % 3;
    }

    if (zero_ui_button(&gui->ui, screen_w * 0.5f - 75.0f, nav_y, btn_w, btn_h, "NEXT TRACK >")) {
        gui->menu_beep_triggered = true;
        gui->selected_track_idx = (gui->selected_track_idx + 1) % 3;
    }

    if (zero_ui_button(&gui->ui, screen_w * 0.5f + 95.0f, nav_y, btn_w, btn_h, "CONFIRM")) {
        gui->menu_beep_triggered = true;
        gui->countdown_timer = 3.0f;
        gui->prev_countdown_timer = 3.0f;
        gui->state = ZERO_STATE_COUNTDOWN;
    }

    if (zero_ui_button(&gui->ui, screen_w * 0.5f - 75.0f, nav_y + 48.0f, btn_w, btn_h, "BACK")) {
        gui->menu_beep_triggered = true;
        gui->state = ZERO_STATE_MAIN_MENU;
    }
}

void zero_game_ui_render_settings(ZeroGameUI *gui, float screen_w, float screen_h) {
    if (!gui) return;

    /* Backdrop */
    zero_ui_draw_rect(&gui->ui, 0, 0, screen_w, screen_h, vec4(0.04f, 0.05f, 0.08f, 0.85f));

    /* Header */
    zero_ui_draw_text(&gui->ui, screen_w * 0.5f - 180.0f, 35.0f,
                      "AUDIO & SETTINGS", vec4(0.95f, 0.95f, 0.98f, 1.0f), 2.0f);

    float panel_w = 480.0f;
    float panel_h = 320.0f;
    float panel_x = screen_w * 0.5f - panel_w * 0.5f;
    float panel_y = 90.0f;

    zero_ui_panel(&gui->ui, panel_x, panel_y, panel_w, panel_h, "AUDIO CONFIGURATION");

    float slider_w = 360.0f;
    float slider_h = 24.0f;
    float slider_x = panel_x + 60.0f;
    float cur_y = panel_y + 60.0f;

    zero_ui_slider(&gui->ui, slider_x, cur_y, slider_w, slider_h, "Master Volume",
                   &gui->audio_settings.master_volume, 0.0f, 1.0f);

    cur_y += 55.0f;
    zero_ui_slider(&gui->ui, slider_x, cur_y, slider_w, slider_h, "Engine Rev Volume",
                   &gui->audio_settings.engine_volume, 0.0f, 1.0f);

    cur_y += 55.0f;
    zero_ui_slider(&gui->ui, slider_x, cur_y, slider_w, slider_h, "SFX & Tire Slip",
                   &gui->audio_settings.sfx_volume, 0.0f, 1.0f);

    cur_y += 50.0f;
    const char *mute_label = gui->audio_settings.is_muted ? "AUDIO MUTED: [ON]" : "AUDIO MUTED: [OFF]";
    if (zero_ui_button_toggle(&gui->ui, slider_x, cur_y, 170.0f, 32.0f, mute_label, gui->audio_settings.is_muted)) {
        gui->menu_beep_triggered = true;
        gui->audio_settings.is_muted = !gui->audio_settings.is_muted;
    }

    const char *map_label = gui->show_minimap ? "MINIMAP: [ON]" : "MINIMAP: [OFF]";
    if (zero_ui_button_toggle(&gui->ui, slider_x + 190.0f, cur_y, 170.0f, 32.0f, map_label, gui->show_minimap)) {
        gui->menu_beep_triggered = true;
        gui->show_minimap = !gui->show_minimap;
    }

    /* Back button */
    float btn_w = 180.0f;
    float btn_h = 38.0f;
    if (zero_ui_button(&gui->ui, screen_w * 0.5f - btn_w * 0.5f, panel_y + panel_h + 30.0f, btn_w, btn_h, "BACK")) {
        gui->menu_beep_triggered = true;
        gui->state = gui->prev_state;
    }
}

void zero_game_ui_render_countdown(ZeroGameUI *gui, float screen_w, float screen_h) {
    if (!gui) return;

    /* Centered prominent countdown banner */
    float banner_w = 320.0f;
    float banner_h = 130.0f;
    float banner_x = screen_w * 0.5f - banner_w * 0.5f;
    float banner_y = screen_h * 0.35f;

    zero_ui_draw_rect(&gui->ui, banner_x, banner_y, banner_w, banner_h, vec4(0.06f, 0.08f, 0.12f, 0.85f));
    zero_ui_draw_rect_outline(&gui->ui, banner_x, banner_y, banner_w, banner_h, vec4(0.35f, 0.40f, 0.55f, 0.95f), 2.0f);

    zero_ui_draw_text(&gui->ui, banner_x + 85.0f, banner_y + 16.0f, "STARTING GRID", vec4(0.70f, 0.75f, 0.85f, 0.9f), 0.90f);

    float text_y = banner_y + 44.0f;
    if (gui->countdown_timer > 2.0f) {
        zero_ui_draw_text(&gui->ui, screen_w * 0.5f - 22.0f, text_y, "3", vec4(1.0f, 0.25f, 0.20f, 1.0f), 5.0f);
    } else if (gui->countdown_timer > 1.0f) {
        zero_ui_draw_text(&gui->ui, screen_w * 0.5f - 22.0f, text_y, "2", vec4(1.0f, 0.65f, 0.15f, 1.0f), 5.0f);
    } else if (gui->countdown_timer > 0.0f) {
        zero_ui_draw_text(&gui->ui, screen_w * 0.5f - 22.0f, text_y, "1", vec4(1.0f, 0.95f, 0.20f, 1.0f), 5.0f);
    } else {
        zero_ui_draw_text(&gui->ui, screen_w * 0.5f - 60.0f, text_y, "GO!", vec4(0.15f, 0.98f, 0.35f, 1.0f), 5.0f);
    }
}

void zero_game_ui_render_hud(ZeroGameUI *gui, const ZeroGameTelemetry *telem, float screen_w, float screen_h) {
    if (!gui) return;

    /* Fallback default telemetry if NULL */
    ZeroGameTelemetry default_telem;
    if (!telem) {
        memset(&default_telem, 0, sizeof(default_telem));
        default_telem.speed_kmh = 0.0f;
        default_telem.engine_rpm = 950.0f;
        default_telem.gear = 1;
        default_telem.is_automatic = true;
        default_telem.current_lap = 1;
        default_telem.total_laps = 3;
        default_telem.race_position = 1;
        default_telem.total_cars = 4;
        telem = &default_telem;
    }

    /* 1. Tachometer & Analog/Digital RPM (Bottom-Right) */
    float tach_cx = screen_w - 140.0f;
    float tach_cy = screen_h - 130.0f;
    float tach_radius = 80.0f;
    float start_angle = 0.75f * (float)M_PI; /* 135 deg */
    float end_angle = 2.25f * (float)M_PI;   /* 405 deg */
    float rpm_val = telem->engine_rpm;
    if (rpm_val < 0.0f) rpm_val = 0.0f;
    float rpm_pct = rpm_val / 9000.0f;
    if (rpm_pct > 1.0f) rpm_pct = 1.0f;

    Vec4 active_rpm_col = (rpm_val >= 7000.0f) ? vec4(1.0f, 0.20f, 0.15f, 1.0f) : vec4(0.15f, 0.85f, 0.95f, 1.0f);
    Vec4 bg_arc_col = vec4(0.12f, 0.14f, 0.18f, 0.75f);
    zero_ui_draw_arc_gauge(&gui->ui, tach_cx, tach_cy, tach_radius, start_angle, end_angle,
                           rpm_pct, active_rpm_col, bg_arc_col, 12.0f);

    /* Segmented Rev Bar under Tachometer */
    zero_ui_draw_segmented_bar(&gui->ui, tach_cx - 65.0f, tach_cy + 42.0f, 130.0f, 8.0f,
                               16, rpm_pct,
                               vec4(0.15f, 0.85f, 0.95f, 1.0f), vec4(0.12f, 0.14f, 0.18f, 0.9f), vec4(1.0f, 0.20f, 0.15f, 1.0f));

    /* RPM Numerical readout */
    char rpm_buf[32];
    snprintf(rpm_buf, sizeof(rpm_buf), "%4.0f RPM", telem->engine_rpm);
    zero_ui_draw_text(&gui->ui, tach_cx - 36.0f, tach_cy + 22.0f, rpm_buf, vec4(0.85f, 0.90f, 0.95f, 0.95f), 0.95f);

    /* 2. Digital Speedometer & Gear Indicator (Bottom-Right / Centered on gauge) */
    char speed_buf[32];
    snprintf(speed_buf, sizeof(speed_buf), "%3.0f", telem->speed_kmh);
    zero_ui_draw_text(&gui->ui, tach_cx - 45.0f, tach_cy - 40.0f, speed_buf, vec4(0.98f, 0.98f, 1.0f, 1.0f), 3.0f);
    zero_ui_draw_text(&gui->ui, tach_cx - 18.0f, tach_cy + 4.0f, "KM/H", vec4(0.65f, 0.72f, 0.85f, 0.90f), 0.90f);

    /* Gear indicator badge */
    char gear_str[8];
    if (telem->gear == -1) {
        snprintf(gear_str, sizeof(gear_str), "R");
    } else if (telem->gear == 0) {
        snprintf(gear_str, sizeof(gear_str), "N");
    } else {
        snprintf(gear_str, sizeof(gear_str), "%d", telem->gear);
    }
    Vec4 gear_col = (telem->gear == -1) ? vec4(1.0f, 0.3f, 0.2f, 1.0f) :
                    (telem->gear == 0) ? vec4(1.0f, 0.85f, 0.2f, 1.0f) : vec4(0.2f, 0.95f, 0.4f, 1.0f);

    zero_ui_draw_rect(&gui->ui, tach_cx - 88.0f, tach_cy - 12.0f, 26.0f, 26.0f, vec4(0.08f, 0.10f, 0.14f, 0.85f));
    zero_ui_draw_rect_outline(&gui->ui, tach_cx - 88.0f, tach_cy - 12.0f, 26.0f, 26.0f, gear_col, 1.0f);
    zero_ui_draw_text(&gui->ui, tach_cx - 81.0f, tach_cy - 7.0f, gear_str, gear_col, 1.4f);

    if (telem->is_automatic) {
        zero_ui_draw_text(&gui->ui, tach_cx - 96.0f, tach_cy + 18.0f, "AUTO", vec4(0.65f, 0.70f, 0.80f, 0.85f), 0.70f);
    }

    /* 3. Lap & Position Card (Top-Left) */
    float pos_card_w = 190.0f;
    float pos_card_h = 75.0f;
    float pos_card_x = 20.0f;
    float pos_card_y = 20.0f;

    zero_ui_draw_rect(&gui->ui, pos_card_x, pos_card_y, pos_card_w, pos_card_h, vec4(0.08f, 0.10f, 0.14f, 0.85f));
    zero_ui_draw_rect_outline(&gui->ui, pos_card_x, pos_card_y, pos_card_w, pos_card_h, vec4(0.25f, 0.30f, 0.40f, 0.90f), 1.0f);

    const char *pos_suffix = (telem->race_position == 1) ? "1st" :
                             (telem->race_position == 2) ? "2nd" :
                             (telem->race_position == 3) ? "3rd" : "4th";
    Vec4 pos_col = (telem->race_position == 1) ? vec4(1.0f, 0.84f, 0.10f, 1.0f) :
                   (telem->race_position == 2) ? vec4(0.80f, 0.85f, 0.92f, 1.0f) :
                   (telem->race_position == 3) ? vec4(0.85f, 0.55f, 0.25f, 1.0f) : vec4(0.65f, 0.70f, 0.80f, 1.0f);

    char pos_buf[32];
    snprintf(pos_buf, sizeof(pos_buf), "POS: %s / %d", pos_suffix, telem->total_cars > 0 ? telem->total_cars : 4);
    zero_ui_draw_text(&gui->ui, pos_card_x + 12.0f, pos_card_y + 12.0f, pos_buf, pos_col, 1.3f);

    char lap_buf[32];
    snprintf(lap_buf, sizeof(lap_buf), "LAP: %d / %d", telem->current_lap, telem->total_laps > 0 ? telem->total_laps : 3);
    zero_ui_draw_text(&gui->ui, pos_card_x + 12.0f, pos_card_y + 44.0f, lap_buf, vec4(0.85f, 0.88f, 0.95f, 1.0f), 1.1f);

    /* 4. Lap Times & Split Delta Card (Top-Center) */
    float time_card_w = 280.0f;
    float time_card_h = 75.0f;
    float time_card_x = screen_w * 0.5f - time_card_w * 0.5f;
    float time_card_y = 20.0f;

    zero_ui_draw_rect(&gui->ui, time_card_x, time_card_y, time_card_w, time_card_h, vec4(0.08f, 0.10f, 0.14f, 0.85f));
    zero_ui_draw_rect_outline(&gui->ui, time_card_x, time_card_y, time_card_w, time_card_h, vec4(0.25f, 0.30f, 0.40f, 0.90f), 1.0f);

    char cur_time_str[32];
    format_lap_time(telem->current_lap_time, cur_time_str, sizeof(cur_time_str));
    char cur_time_buf[48];
    snprintf(cur_time_buf, sizeof(cur_time_buf), "TIME: %s", cur_time_str);
    zero_ui_draw_text(&gui->ui, time_card_x + 14.0f, time_card_y + 10.0f, cur_time_buf, vec4(0.95f, 0.95f, 0.98f, 1.0f), 1.1f);

    char best_time_str[32];
    format_lap_time(telem->best_lap_time, best_time_str, sizeof(best_time_str));
    char best_time_buf[48];
    snprintf(best_time_buf, sizeof(best_time_buf), "BEST: %s", best_time_str);
    zero_ui_draw_text(&gui->ui, time_card_x + 14.0f, time_card_y + 32.0f, best_time_buf, vec4(0.85f, 0.75f, 0.20f, 1.0f), 0.95f);

    char split_buf[32];
    Vec4 split_col = vec4(0.85f, 0.88f, 0.92f, 1.0f);
    if (telem->split_delta < -0.001f) {
        snprintf(split_buf, sizeof(split_buf), "SPLIT: %.2fs", telem->split_delta);
        split_col = vec4(0.20f, 0.95f, 0.40f, 1.0f); /* Green = ahead */
    } else if (telem->split_delta > 0.001f) {
        snprintf(split_buf, sizeof(split_buf), "SPLIT: +%.2fs", telem->split_delta);
        split_col = vec4(1.0f, 0.30f, 0.25f, 1.0f); /* Red = behind */
    } else {
        snprintf(split_buf, sizeof(split_buf), "SPLIT:  0.00s");
    }
    zero_ui_draw_text(&gui->ui, time_card_x + 14.0f, time_card_y + 52.0f, split_buf, split_col, 0.95f);

    /* 5. Drift Banner (Center screen when drifting) */
    if (telem->is_drifting) {
        float drift_w = 230.0f;
        float drift_h = 36.0f;
        float drift_x = screen_w * 0.5f - drift_w * 0.5f;
        float drift_y = 105.0f;

        zero_ui_draw_rect(&gui->ui, drift_x, drift_y, drift_w, drift_h, vec4(0.92f, 0.45f, 0.08f, 0.90f));
        zero_ui_draw_rect_outline(&gui->ui, drift_x, drift_y, drift_w, drift_h, vec4(1.0f, 0.85f, 0.25f, 1.0f), 2.0f);

        char drift_buf[48];
        snprintf(drift_buf, sizeof(drift_buf), "DRIFT! %.1f DEG", fabsf(telem->drift_angle));
        zero_ui_draw_text(&gui->ui, drift_x + 18.0f, drift_y + 8.0f, drift_buf, vec4(1.0f, 1.0f, 1.0f, 1.0f), 1.2f);
    }

    /* 6. Minimap / Track Radar (Bottom-Left) */
    if (gui->show_minimap) {
        float radar_w = 180.0f;
        float radar_h = 180.0f;
        float radar_x = 20.0f;
        float radar_y = screen_h - radar_h - 20.0f;
        zero_ui_draw_track_radar(&gui->ui, radar_x, radar_y, radar_w, radar_h,
                                 g_track_spline, ZERO_TRACK_SPLINE_COUNT,
                                 telem->player_spline_t, telem->ai_spline_t);
    }

    /* 7. Controls Hint Guide (Top-Right) */
    zero_ui_draw_text(&gui->ui, screen_w - 310.0f, 20.0f,
                      "[ESC] PAUSE  [C] CAM  [R] RESET", vec4(0.65f, 0.70f, 0.80f, 0.85f), 0.80f);
}

void zero_game_ui_render_pause(ZeroGameUI *gui, float screen_w, float screen_h) {
    if (!gui) return;

    /* Dark translucent overlay covering the full screen */
    zero_ui_draw_rect(&gui->ui, 0, 0, screen_w, screen_h, vec4(0.02f, 0.03f, 0.05f, 0.78f));

    /* Pause Panel */
    float panel_w = 360.0f;
    float panel_h = 310.0f;
    float panel_x = screen_w * 0.5f - panel_w * 0.5f;
    float panel_y = screen_h * 0.5f - panel_h * 0.5f;

    zero_ui_panel(&gui->ui, panel_x, panel_y, panel_w, panel_h, "RACE PAUSED");

    float btn_w = 260.0f;
    float btn_h = 40.0f;
    float btn_x = screen_w * 0.5f - btn_w * 0.5f;
    float start_y = panel_y + 55.0f;
    float spacing = 52.0f;

    if (zero_ui_button(&gui->ui, btn_x, start_y + spacing * 0.0f, btn_w, btn_h, "RESUME")) {
        gui->menu_beep_triggered = true;
        gui->state = ZERO_STATE_RACING;
    }

    if (zero_ui_button(&gui->ui, btn_x, start_y + spacing * 1.0f, btn_w, btn_h, "RESTART")) {
        gui->menu_beep_triggered = true;
        gui->restart_requested = true;
        gui->countdown_timer = 3.0f;
        gui->prev_countdown_timer = 3.0f;
        gui->state = ZERO_STATE_COUNTDOWN;
    }

    if (zero_ui_button(&gui->ui, btn_x, start_y + spacing * 2.0f, btn_w, btn_h, "SETTINGS")) {
        gui->menu_beep_triggered = true;
        gui->prev_state = ZERO_STATE_PAUSED;
        gui->state = ZERO_STATE_SETTINGS;
    }

    if (zero_ui_button(&gui->ui, btn_x, start_y + spacing * 3.0f, btn_w, btn_h, "QUIT TO MAIN MENU")) {
        gui->menu_beep_triggered = true;
        gui->state = ZERO_STATE_MAIN_MENU;
    }
}

void zero_game_ui_render_leaderboard(ZeroGameUI *gui, float screen_w, float screen_h) {
    if (!gui) return;

    /* Dark backdrop */
    zero_ui_draw_rect(&gui->ui, 0, 0, screen_w, screen_h, vec4(0.04f, 0.05f, 0.08f, 0.88f));

    /* Panel */
    float panel_w = 640.0f;
    float panel_h = 380.0f;
    float panel_x = screen_w * 0.5f - panel_w * 0.5f;
    float panel_y = screen_h * 0.5f - panel_h * 0.5f - 20.0f;

    zero_ui_panel(&gui->ui, panel_x, panel_y, panel_w, panel_h, "RACE FINISHED - FINAL STANDINGS");

    /* Table Column Headers */
    float cur_y = panel_y + 45.0f;
    zero_ui_draw_rect(&gui->ui, panel_x + 15.0f, cur_y, panel_w - 30.0f, 24.0f, vec4(0.16f, 0.18f, 0.24f, 1.0f));
    zero_ui_draw_text(&gui->ui, panel_x + 25.0f, cur_y + 4.0f, "POS", vec4(0.70f, 0.75f, 0.85f, 1.0f), 0.85f);
    zero_ui_draw_text(&gui->ui, panel_x + 85.0f, cur_y + 4.0f, "DRIVER", vec4(0.70f, 0.75f, 0.85f, 1.0f), 0.85f);
    zero_ui_draw_text(&gui->ui, panel_x + 225.0f, cur_y + 4.0f, "VEHICLE", vec4(0.70f, 0.75f, 0.85f, 1.0f), 0.85f);
    zero_ui_draw_text(&gui->ui, panel_x + 375.0f, cur_y + 4.0f, "TOTAL TIME", vec4(0.70f, 0.75f, 0.85f, 1.0f), 0.85f);
    zero_ui_draw_text(&gui->ui, panel_x + 505.0f, cur_y + 4.0f, "BEST LAP", vec4(0.70f, 0.75f, 0.85f, 1.0f), 0.85f);

    /* Rows */
    cur_y += 30.0f;
    for (int i = 0; i < 4; ++i) {
        Vec4 row_bg = (i % 2 == 0) ? vec4(0.10f, 0.12f, 0.16f, 0.85f) : vec4(0.13f, 0.15f, 0.20f, 0.85f);
        zero_ui_draw_rect(&gui->ui, panel_x + 15.0f, cur_y, panel_w - 30.0f, 40.0f, row_bg);

        Vec4 rank_col = (gui->podium[i].finish_position == 1) ? vec4(1.0f, 0.84f, 0.10f, 1.0f) :
                        (gui->podium[i].finish_position == 2) ? vec4(0.80f, 0.85f, 0.92f, 1.0f) :
                        (gui->podium[i].finish_position == 3) ? vec4(0.85f, 0.55f, 0.25f, 1.0f) :
                                                                vec4(0.65f, 0.70f, 0.80f, 1.0f);

        char pos_str[16];
        snprintf(pos_str, sizeof(pos_str), "%d", gui->podium[i].finish_position);
        zero_ui_draw_text(&gui->ui, panel_x + 32.0f, cur_y + 12.0f, pos_str, rank_col, 1.1f);

        zero_ui_draw_text(&gui->ui, panel_x + 85.0f, cur_y + 12.0f, gui->podium[i].driver_name, rank_col, 1.0f);
        zero_ui_draw_text(&gui->ui, panel_x + 225.0f, cur_y + 12.0f, gui->podium[i].car_name, vec4(0.85f, 0.88f, 0.95f, 1.0f), 0.95f);

        char tot_str[32];
        format_lap_time(gui->podium[i].total_time, tot_str, sizeof(tot_str));
        zero_ui_draw_text(&gui->ui, panel_x + 375.0f, cur_y + 12.0f, tot_str, vec4(0.90f, 0.92f, 0.98f, 1.0f), 0.95f);

        char best_str[32];
        format_lap_time(gui->podium[i].best_lap, best_str, sizeof(best_str));
        zero_ui_draw_text(&gui->ui, panel_x + 505.0f, cur_y + 12.0f, best_str, vec4(0.85f, 0.75f, 0.20f, 1.0f), 0.95f);

        cur_y += 46.0f;
    }

    /* Buttons */
    float btn_w = 200.0f;
    float btn_h = 38.0f;
    float nav_y = panel_y + panel_h + 18.0f;

    if (zero_ui_button(&gui->ui, screen_w * 0.5f - btn_w - 15.0f, nav_y, btn_w, btn_h, "RESTART RACE")) {
        gui->menu_beep_triggered = true;
        gui->restart_requested = true;
        gui->countdown_timer = 3.0f;
        gui->prev_countdown_timer = 3.0f;
        gui->state = ZERO_STATE_COUNTDOWN;
    }

    if (zero_ui_button(&gui->ui, screen_w * 0.5f + 15.0f, nav_y, btn_w, btn_h, "CONTINUE TO MENU")) {
        gui->menu_beep_triggered = true;
        gui->state = ZERO_STATE_MAIN_MENU;
    }
}

/* --------------------------------------------------------------------------
 * Main Per-Frame Orchestrator
 * -------------------------------------------------------------------------- */

void zero_game_ui_render(ZeroGameUI *gui, const ZeroGameTelemetry *telem,
                         float screen_w, float screen_h,
                         float mouse_x, float mouse_y, bool mouse_down, float dt) {
    if (!gui) return;

    /* Clear per-frame sound triggers */
    gui->menu_beep_triggered = false;
    gui->countdown_red_triggered = false;
    gui->countdown_green_triggered = false;

    zero_ui_begin(&gui->ui, screen_w, screen_h, mouse_x, mouse_y, mouse_down);

    /* Update Countdown state timing & sound cues */
    if (gui->state == ZERO_STATE_COUNTDOWN) {
        float old_t = gui->countdown_timer;
        gui->countdown_timer -= dt;
        float new_t = gui->countdown_timer;

        /* Red sound cues on integer crossing 3, 2, 1 */
        if (old_t > 3.0f && new_t <= 3.0f) gui->countdown_red_triggered = true;
        if (old_t > 2.0f && new_t <= 2.0f) gui->countdown_red_triggered = true;
        if (old_t > 1.0f && new_t <= 1.0f) gui->countdown_red_triggered = true;

        /* Green chime on GO! */
        if (old_t > 0.0f && new_t <= 0.0f) gui->countdown_green_triggered = true;

        /* After brief GO display (0.5s), transition to RACING */
        if (new_t <= -0.50f) {
            gui->state = ZERO_STATE_RACING;
        }
        gui->prev_countdown_timer = new_t;
    }

    /* Render current state page */
    switch (gui->state) {
        case ZERO_STATE_MAIN_MENU:
            zero_game_ui_render_main_menu(gui, screen_w, screen_h);
            break;

        case ZERO_STATE_CAR_SELECT:
            zero_game_ui_render_car_select(gui, screen_w, screen_h);
            break;

        case ZERO_STATE_TRACK_SELECT:
            zero_game_ui_render_track_select(gui, screen_w, screen_h);
            break;

        case ZERO_STATE_SETTINGS:
            zero_game_ui_render_settings(gui, screen_w, screen_h);
            break;

        case ZERO_STATE_COUNTDOWN:
            /* In countdown, render HUD behind countdown banner */
            zero_game_ui_render_hud(gui, telem, screen_w, screen_h);
            zero_game_ui_render_countdown(gui, screen_w, screen_h);
            break;

        case ZERO_STATE_RACING:
            zero_game_ui_render_hud(gui, telem, screen_w, screen_h);
            break;

        case ZERO_STATE_PAUSED:
            /* In pause, render HUD behind pause dialog */
            zero_game_ui_render_hud(gui, telem, screen_w, screen_h);
            zero_game_ui_render_pause(gui, screen_w, screen_h);
            break;

        case ZERO_STATE_LEADERBOARD:
            zero_game_ui_render_leaderboard(gui, screen_w, screen_h);
            break;

        default:
            break;
    }

    zero_ui_end(&gui->ui);
}

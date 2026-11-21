#ifndef ZERO_RACE_UI_H
#define ZERO_RACE_UI_H

#include "zero_ui.h"
#include "zero_math.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum ZeroGameFlowState {
    ZERO_STATE_MAIN_MENU = 0,
    ZERO_STATE_CAR_SELECT,
    ZERO_STATE_TRACK_SELECT,
    ZERO_STATE_SETTINGS,
    ZERO_STATE_COUNTDOWN,
    ZERO_STATE_RACING,
    ZERO_STATE_PAUSED,
    ZERO_STATE_LEADERBOARD
} ZeroGameFlowState;

typedef struct ZeroCarConfig {
    const char *name;
    const char *tagline;
    float max_speed_kmh;
    float acceleration_rating; /* 0.0 to 1.0 */
    float handling_rating;     /* 0.0 to 1.0 */
    Vec3 body_color;
    float mass;
    float engine_power;
} ZeroCarConfig;

typedef struct ZeroTrackConfig {
    const char *name;
    const char *location;
    int lap_count;
    float length_meters;
    float time_of_day_hours; /* 0.0 to 24.0, e.g. 14.5 = 2:30 PM */
} ZeroTrackConfig;

typedef struct ZeroGameTelemetry {
    float speed_kmh;
    float engine_rpm;
    int gear; /* -1 = R, 0 = N, 1-6 */
    bool is_automatic;
    int current_lap;
    int total_laps;
    int race_position; /* 1 to 4 */
    int total_cars;
    float current_lap_time;
    float best_lap_time;
    float split_delta; /* Negative = ahead, positive = behind */
    bool is_drifting;
    float drift_angle;
    float player_spline_t;
    float ai_spline_t[3];
} ZeroGameTelemetry;

typedef struct ZeroAudioSettings {
    float master_volume; /* 0.0 to 1.0 */
    float engine_volume;
    float sfx_volume;
    bool is_muted;
} ZeroAudioSettings;

typedef struct ZeroPodiumRecord {
    char driver_name[32];
    char car_name[32];
    float total_time;
    float best_lap;
    int finish_position;
} ZeroPodiumRecord;

typedef struct ZeroGameUI {
    ZeroUI ui;
    ZeroGameFlowState state;
    ZeroGameFlowState prev_state;
    int selected_car_idx;
    int selected_track_idx;
    ZeroAudioSettings audio_settings;
    float countdown_timer; /* 3.0 -> 0.0 */
    float prev_countdown_timer;
    bool show_minimap;
    int active_camera_mode;

    /* Leaderboard podium records */
    struct {
        char driver_name[32];
        char car_name[32];
        float total_time;
        float best_lap;
        int finish_position;
    } podium[4];

    /* Navigation & event notification flags */
    bool restart_requested;
    bool quit_requested;
    bool menu_beep_triggered;
    bool countdown_red_triggered;
    bool countdown_green_triggered;
} ZeroGameUI;

/* Core Lifecycle & Frame Rendering */
bool zero_game_ui_init(ZeroGameUI *gui);
void zero_game_ui_shutdown(ZeroGameUI *gui);
void zero_game_ui_render(ZeroGameUI *gui, const ZeroGameTelemetry *telem,
                         float screen_w, float screen_h,
                         float mouse_x, float mouse_y, bool mouse_down, float dt);

/* Specific Page Renderers */
void zero_game_ui_render_main_menu(ZeroGameUI *gui, float screen_w, float screen_h);
void zero_game_ui_render_car_select(ZeroGameUI *gui, float screen_w, float screen_h);
void zero_game_ui_render_track_select(ZeroGameUI *gui, float screen_w, float screen_h);
void zero_game_ui_render_settings(ZeroGameUI *gui, float screen_w, float screen_h);
void zero_game_ui_render_countdown(ZeroGameUI *gui, float screen_w, float screen_h);
void zero_game_ui_render_hud(ZeroGameUI *gui, const ZeroGameTelemetry *telem, float screen_w, float screen_h);
void zero_game_ui_render_pause(ZeroGameUI *gui, float screen_w, float screen_h);
void zero_game_ui_render_leaderboard(ZeroGameUI *gui, float screen_w, float screen_h);

/* Custom Procedural Widgets */
void zero_ui_draw_arc_gauge(ZeroUI *ui, float center_x, float center_y, float radius,
                            float start_angle_rad, float end_angle_rad,
                            float value_pct, Vec4 bar_color, Vec4 bg_color, float thickness);

void zero_ui_draw_segmented_bar(ZeroUI *ui, float x, float y, float w, float h,
                                int num_segments, float value_pct,
                                Vec4 active_color, Vec4 inactive_color, Vec4 redline_color);

void zero_ui_draw_track_radar(ZeroUI *ui, float x, float y, float w, float h,
                              const float spline[][2], int point_count,
                              float player_t, const float ai_t[3]);

/* Catalog Accessors & Helpers */
int zero_game_ui_get_car_count(void);
const ZeroCarConfig *zero_game_ui_get_car(int index);
int zero_game_ui_get_track_count(void);
const ZeroTrackConfig *zero_game_ui_get_track(int index);

void zero_game_ui_set_state(ZeroGameUI *gui, ZeroGameFlowState state);
bool zero_game_ui_is_restart_requested(const ZeroGameUI *gui);
void zero_game_ui_clear_restart_request(ZeroGameUI *gui);
bool zero_game_ui_is_quit_requested(const ZeroGameUI *gui);
void zero_game_ui_clear_quit_request(ZeroGameUI *gui);

void zero_game_ui_set_podium_entry(ZeroGameUI *gui, int index, const char *driver,
                                   const char *car, float total_time, float best_lap, int pos);

#ifdef __cplusplus
}
#endif

#endif /* ZERO_RACE_UI_H */

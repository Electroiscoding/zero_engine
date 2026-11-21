#ifndef ZERO_AUDIO_H
#define ZERO_AUDIO_H

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <pthread.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ZERO_AUDIO_SAMPLE_RATE 44100
#define ZERO_AUDIO_RING_BUFFER_SIZE 32768

#define ZERO_SFX_NONE 0
#define ZERO_SFX_MENU_BEEP 1
#define ZERO_SFX_COUNTDOWN_RED 2
#define ZERO_SFX_COUNTDOWN_GREEN 3

typedef enum ZeroAudioBackend {
    ZERO_AUDIO_BACKEND_ALSA = 0,
    ZERO_AUDIO_BACKEND_PIPE,
    ZERO_AUDIO_BACKEND_SOFTWARE
} ZeroAudioBackend;

typedef struct ZeroAudioParams {
    float engine_rpm;
    float throttle;
    float tire_slip_ratio;
    float vehicle_speed_kmh;
    bool trigger_shift_pop;
    bool trigger_menu_beep;
    bool trigger_countdown_red;
    bool trigger_countdown_green;
} ZeroAudioParams;

typedef struct ZeroAudioEngine {
    ZeroAudioBackend backend;
    bool is_running;
    pthread_t thread;
    pthread_mutex_t lock;
    void *alsa_handle;
    void *alsa_lib;
    FILE *pipe_file;
    int16_t ring_buffer[ZERO_AUDIO_RING_BUFFER_SIZE];
    uint32_t ring_write_pos;
    uint32_t ring_read_pos;
    float engine_phase;
    float engine_lpf_state;
    uint32_t noise_seed;
    float tire_bpf_x1, tire_bpf_x2, tire_bpf_y1, tire_bpf_y2;
    float wind_lpf_state;
    float sfx_phase;
    float sfx_timer;
    int sfx_type;
    float pop_timer;
    float pop_phase;
    ZeroAudioParams params;
    float master_volume;
    bool is_muted;
} ZeroAudioEngine;

bool zero_audio_init(ZeroAudioEngine *audio, bool force_software);
void zero_audio_shutdown(ZeroAudioEngine *audio);
void zero_audio_update_params(ZeroAudioEngine *audio, const ZeroAudioParams *params);
void zero_audio_synthesize_buffer(ZeroAudioEngine *audio, int16_t *out_samples, uint32_t sample_count);

#ifdef __cplusplus
}
#endif

#endif /* ZERO_AUDIO_H */

#ifndef _POSIX_C_SOURCE
#define _POSIX_C_SOURCE 200809L
#endif

#include "zero_audio.h"

#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <dlfcn.h>
#include <time.h>

#define ZERO_PI 3.14159265358979323846f
#define ZERO_TWO_PI 6.28318530717958647692f

/* Dynamic ALSA opaque types and constants (self-contained, no asoundlib.h) */
typedef void snd_pcm_t;
typedef long snd_pcm_sframes_t;
typedef unsigned long snd_pcm_uframes_t;

#define SND_PCM_STREAM_PLAYBACK 0
#define SND_PCM_ACCESS_RW_INTERLEAVED 3
#define SND_PCM_FORMAT_S16_LE 2

typedef int (*snd_pcm_open_t)(snd_pcm_t **pcm, const char *name, int stream, int mode);
typedef int (*snd_pcm_set_params_t)(snd_pcm_t *pcm, int format, int access, unsigned int channels, unsigned int rate, int soft_resample, unsigned int latency);
typedef snd_pcm_sframes_t (*snd_pcm_writei_t)(snd_pcm_t *pcm, const void *buffer, snd_pcm_uframes_t size);
typedef int (*snd_pcm_recover_t)(snd_pcm_t *pcm, int err, int silent);
typedef int (*snd_pcm_close_t)(snd_pcm_t *pcm);
typedef const char *(*snd_strerror_t)(int errnum);

static snd_pcm_open_t alsa_pcm_open = NULL;
static snd_pcm_set_params_t alsa_pcm_set_params = NULL;
static snd_pcm_writei_t alsa_pcm_writei = NULL;
static snd_pcm_recover_t alsa_pcm_recover = NULL;
static snd_pcm_close_t alsa_pcm_close = NULL;
static snd_strerror_t alsa_strerror = NULL;

/* Precomputed BPF filter coefficients for tire squeal (fc = 1350 Hz, Q = 6.0, fs = 44100 Hz) */
static float s_tire_b0 = 0.0f;
static float s_tire_a1 = 0.0f;
static float s_tire_a2 = 0.0f;
static bool s_tire_coeffs_ready = false;

static void zero_audio_init_tire_coeffs(void) {
    if (s_tire_coeffs_ready) return;
    float omega = ZERO_TWO_PI * 1350.0f / (float)ZERO_AUDIO_SAMPLE_RATE;
    float sin_w = sinf(omega);
    float cos_w = cosf(omega);
    float alpha = sin_w / (2.0f * 6.0f);
    float a0 = 1.0f + alpha;

    s_tire_b0 = (sin_w * 0.5f) / a0;
    s_tire_a1 = (-2.0f * cos_w) / a0;
    s_tire_a2 = (1.0f - alpha) / a0;
    s_tire_coeffs_ready = true;
}

static inline float zero_audio_xorshift32(uint32_t *state) {
    uint32_t x = *state;
    if (x == 0) x = 0x12345678;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    *state = x;
    return ((float)(int32_t)x) * (1.0f / 2147483648.0f);
}

static inline float zero_audio_smoothstep(float edge0, float edge1, float x) {
    if (x <= edge0) return 0.0f;
    if (x >= edge1) return 1.0f;
    float t = (x - edge0) / (edge1 - edge0);
    return t * t * (3.0f - 2.0f * t);
}

/* Background audio playback thread for ALSA and PIPE streaming */
static void *zero_audio_playback_thread(void *user_data) {
    ZeroAudioEngine *audio = (ZeroAudioEngine *)user_data;
    const uint32_t chunk_size = 1024;
    int16_t chunk[1024];

    while (audio->is_running) {
        zero_audio_synthesize_buffer(audio, chunk, chunk_size);

        if (audio->backend == ZERO_AUDIO_BACKEND_ALSA && audio->alsa_handle && alsa_pcm_writei) {
            snd_pcm_sframes_t frames = alsa_pcm_writei(audio->alsa_handle, chunk, chunk_size);
            if (frames < 0) {
                if (alsa_pcm_recover) {
                    alsa_pcm_recover(audio->alsa_handle, (int)frames, 1);
                }
            }
        } else if (audio->backend == ZERO_AUDIO_BACKEND_PIPE && audio->pipe_file) {
            size_t written = fwrite(chunk, sizeof(int16_t), chunk_size, audio->pipe_file);
            if (written < chunk_size) {
                break;
            }
            fflush(audio->pipe_file);
        } else {
            struct timespec ts;
            ts.tv_sec = 0;
            ts.tv_nsec = 20000000L; /* 20 ms */
            nanosleep(&ts, NULL);
        }
    }
    return NULL;
}

bool zero_audio_init(ZeroAudioEngine *audio, bool force_software) {
    if (!audio) return false;
    memset(audio, 0, sizeof(ZeroAudioEngine));

    if (pthread_mutex_init(&audio->lock, NULL) != 0) {
        return false;
    }

    zero_audio_init_tire_coeffs();

    audio->noise_seed = 0x12345678;
    audio->master_volume = 1.0f;
    audio->is_muted = false;
    audio->params.engine_rpm = 1000.0f;
    audio->params.throttle = 0.0f;
    audio->params.tire_slip_ratio = 0.0f;
    audio->params.vehicle_speed_kmh = 0.0f;
    audio->is_running = true;

    if (force_software) {
        audio->backend = ZERO_AUDIO_BACKEND_SOFTWARE;
        return true;
    }

    /* Primary: Dynamic ALSA */
    void *alsa_lib = dlopen("libasound.so.2", RTLD_LAZY);
    if (alsa_lib) {
        alsa_pcm_open = (snd_pcm_open_t)dlsym(alsa_lib, "snd_pcm_open");
        alsa_pcm_set_params = (snd_pcm_set_params_t)dlsym(alsa_lib, "snd_pcm_set_params");
        alsa_pcm_writei = (snd_pcm_writei_t)dlsym(alsa_lib, "snd_pcm_writei");
        alsa_pcm_recover = (snd_pcm_recover_t)dlsym(alsa_lib, "snd_pcm_recover");
        alsa_pcm_close = (snd_pcm_close_t)dlsym(alsa_lib, "snd_pcm_close");
        alsa_strerror = (snd_strerror_t)dlsym(alsa_lib, "snd_strerror");

        if (alsa_pcm_open && alsa_pcm_set_params && alsa_pcm_writei &&
            alsa_pcm_recover && alsa_pcm_close) {
            snd_pcm_t *pcm = NULL;
            int err = alsa_pcm_open(&pcm, "default", SND_PCM_STREAM_PLAYBACK, 0);
            if (err == 0 && pcm) {
                err = alsa_pcm_set_params(pcm, SND_PCM_FORMAT_S16_LE, SND_PCM_ACCESS_RW_INTERLEAVED,
                                         1, ZERO_AUDIO_SAMPLE_RATE, 1, 50000);
                if (err == 0) {
                    audio->backend = ZERO_AUDIO_BACKEND_ALSA;
                    audio->alsa_handle = (void *)pcm;
                    audio->alsa_lib = alsa_lib;
                    if (pthread_create(&audio->thread, NULL, zero_audio_playback_thread, audio) == 0) {
                        return true;
                    }
                    alsa_pcm_close(pcm);
                    audio->alsa_handle = NULL;
                } else {
                    alsa_pcm_close(pcm);
                }
            }
        }
        dlclose(alsa_lib);
        alsa_lib = NULL;
    }

    /* Secondary: Direct Audio Pipe fallback (aplay) */
    FILE *pipe = popen("aplay -q -r 44100 -f S16_LE -c 1 -t raw", "w");
    if (pipe) {
        audio->backend = ZERO_AUDIO_BACKEND_PIPE;
        audio->pipe_file = pipe;
        if (pthread_create(&audio->thread, NULL, zero_audio_playback_thread, audio) == 0) {
            return true;
        }
        pclose(pipe);
        audio->pipe_file = NULL;
    }

    /* Tertiary: Software Ring Buffer */
    audio->backend = ZERO_AUDIO_BACKEND_SOFTWARE;
    return true;
}

void zero_audio_shutdown(ZeroAudioEngine *audio) {
    if (!audio) return;

    if (audio->is_running) {
        audio->is_running = false;

        if (audio->backend == ZERO_AUDIO_BACKEND_ALSA || audio->backend == ZERO_AUDIO_BACKEND_PIPE) {
            if (audio->thread) {
                pthread_join(audio->thread, NULL);
                audio->thread = 0;
            }
        }

        if (audio->alsa_handle && alsa_pcm_close) {
            alsa_pcm_close(audio->alsa_handle);
            audio->alsa_handle = NULL;
        }

        if (audio->alsa_lib) {
            dlclose(audio->alsa_lib);
            audio->alsa_lib = NULL;
        }

        if (audio->pipe_file) {
            pclose(audio->pipe_file);
            audio->pipe_file = NULL;
        }

        pthread_mutex_destroy(&audio->lock);
    }
}

void zero_audio_update_params(ZeroAudioEngine *audio, const ZeroAudioParams *params) {
    if (!audio || !audio->is_running || !params) return;

    pthread_mutex_lock(&audio->lock);
    audio->params.engine_rpm = params->engine_rpm;
    audio->params.throttle = params->throttle;
    audio->params.tire_slip_ratio = params->tire_slip_ratio;
    audio->params.vehicle_speed_kmh = params->vehicle_speed_kmh;

    if (params->trigger_shift_pop) {
        audio->params.trigger_shift_pop = true;
    }
    if (params->trigger_menu_beep) {
        audio->params.trigger_menu_beep = true;
    }
    if (params->trigger_countdown_red) {
        audio->params.trigger_countdown_red = true;
    }
    if (params->trigger_countdown_green) {
        audio->params.trigger_countdown_green = true;
    }
    if (params->trigger_crash) {
        audio->params.trigger_crash = true;
        audio->params.crash_intensity = params->crash_intensity;
    }
    pthread_mutex_unlock(&audio->lock);
}

void zero_audio_synthesize_buffer(ZeroAudioEngine *audio, int16_t *out_samples, uint32_t sample_count) {
    if (!audio || !audio->is_running || sample_count == 0) return;

    pthread_mutex_lock(&audio->lock);

    /* Latch one-shot audio triggers */
    if (audio->params.trigger_shift_pop) {
        audio->pop_timer = 0.12f;
        audio->pop_phase = 0.0f;
        audio->params.trigger_shift_pop = false;
    }
    if (audio->params.trigger_crash) {
        audio->crash_timer = 0.38f;
        audio->crash_phase = 0.0f;
        audio->crash_gain = (audio->params.crash_intensity > 0.05f) ? audio->params.crash_intensity : 1.0f;
        if (audio->crash_gain > 2.0f) audio->crash_gain = 2.0f;
        audio->params.trigger_crash = false;
    }
    if (audio->params.trigger_menu_beep) {
        audio->sfx_type = ZERO_SFX_MENU_BEEP;
        audio->sfx_timer = 0.08f;
        audio->sfx_phase = 0.0f;
        audio->params.trigger_menu_beep = false;
    }
    if (audio->params.trigger_countdown_red) {
        audio->sfx_type = ZERO_SFX_COUNTDOWN_RED;
        audio->sfx_timer = 0.25f;
        audio->sfx_phase = 0.0f;
        audio->params.trigger_countdown_red = false;
    }
    if (audio->params.trigger_countdown_green) {
        audio->sfx_type = ZERO_SFX_COUNTDOWN_GREEN;
        audio->sfx_timer = 0.65f;
        audio->sfx_phase = 0.0f;
        audio->params.trigger_countdown_green = false;
    }

    const float sample_rate = (float)ZERO_AUDIO_SAMPLE_RATE;
    const float dt = 1.0f / sample_rate;

    /* Snapshot continuous parameters */
    float rpm = audio->params.engine_rpm;
    if (rpm < 500.0f) rpm = 500.0f;
    if (rpm > 10000.0f) rpm = 10000.0f;

    float throttle = audio->params.throttle;
    if (throttle < 0.0f) throttle = 0.0f;
    if (throttle > 1.0f) throttle = 1.0f;

    float tire_slip = fabsf(audio->params.tire_slip_ratio);
    float speed_kmh = audio->params.vehicle_speed_kmh;
    if (speed_kmh < 0.0f) speed_kmh = 0.0f;

    float master_vol = audio->master_volume;
    bool is_muted = audio->is_muted;

    /* Engine model constants: f_fire = (RPM / 60) * 4 for V8 */
    float f_fire = (rpm / 60.0f) * 4.0f;
    float dphi_engine = f_fire * dt;

    /* Engine throttle-modulated resonant lowpass cutoff */
    float fc_engine = 320.0f + 3280.0f * throttle + 800.0f * (rpm / 7500.0f);
    if (fc_engine < 100.0f) fc_engine = 100.0f;
    if (fc_engine > 18000.0f) fc_engine = 18000.0f;
    float wc_eng = ZERO_TWO_PI * fc_engine * dt;
    float alpha_eng = wc_eng / (1.0f + wc_eng);

    /* Wind model parameters: fc = 180 + v_kmh * 4.5 */
    float fc_wind = 180.0f + speed_kmh * 4.5f;
    if (fc_wind < 100.0f) fc_wind = 100.0f;
    if (fc_wind > 18000.0f) fc_wind = 18000.0f;
    float wc_wind = ZERO_TWO_PI * fc_wind * dt;
    float alpha_wind = wc_wind / (1.0f + wc_wind);

    float wind_norm = speed_kmh / 220.0f;
    float wind_power = wind_norm * wind_norm;
    if (wind_power > 1.0f) wind_power = 1.0f;
    float wind_gain = wind_power * 0.35f;

    /* Tire squeal modulation */
    float tire_slip_gain = zero_audio_smoothstep(0.12f, 0.55f, tire_slip);
    float speed_ms = speed_kmh * (1.0f / 3.6f);
    float speed_factor = (speed_ms >= 4.0f) ? 1.0f : (speed_ms / 4.0f);
    float tire_base_gain = tire_slip_gain * speed_factor;

    for (uint32_t i = 0; i < sample_count; ++i) {
        /* 1. Multi-harmonic V8 Engine Roar */
        audio->engine_phase += dphi_engine;
        if (audio->engine_phase >= 2.0f) {
            audio->engine_phase -= 2.0f;
        }

        float phi1 = fmodf(audio->engine_phase, 1.0f);
        float w_saw1 = 2.0f * phi1 - 1.0f;

        float phi2 = fmodf(audio->engine_phase * 2.0f, 1.0f);
        float w_pulse = (phi2 < 0.38f) ? 1.0f : -1.0f;

        float phi3 = fmodf(audio->engine_phase * 3.0f, 1.0f);
        float w_saw3 = 2.0f * phi3 - 1.0f;

        float phi_sub = audio->engine_phase * 0.5f;
        float w_sub = 2.0f * phi_sub - 1.0f;

        float s_eng_raw = 0.45f * w_saw1 + 0.25f * w_pulse + 0.18f * w_saw3 + 0.12f * w_sub;

        audio->engine_lpf_state += alpha_eng * (s_eng_raw - audio->engine_lpf_state);
        if (isnan(audio->engine_lpf_state) || isinf(audio->engine_lpf_state)) {
            audio->engine_lpf_state = 0.0f;
        }
        float s_engine = audio->engine_lpf_state;

        /* 2. Lateral Tire Slip Squeal */
        float x_tire = zero_audio_xorshift32(&audio->noise_seed);
        float y_tire = s_tire_b0 * (x_tire - audio->tire_bpf_x2)
                     - s_tire_a1 * audio->tire_bpf_y1
                     - s_tire_a2 * audio->tire_bpf_y2;

        audio->tire_bpf_x2 = audio->tire_bpf_x1;
        audio->tire_bpf_x1 = x_tire;
        audio->tire_bpf_y2 = audio->tire_bpf_y1;
        audio->tire_bpf_y1 = y_tire;

        if (isnan(y_tire) || isinf(y_tire)) {
            y_tire = 0.0f;
            audio->tire_bpf_y1 = 0.0f;
            audio->tire_bpf_y2 = 0.0f;
        }

        float tremolo_t = (float)(audio->ring_write_pos % ZERO_AUDIO_SAMPLE_RATE) * dt;
        float tremolo = 1.0f + 0.25f * sinf(ZERO_TWO_PI * 32.0f * tremolo_t);
        float s_tire = y_tire * tire_base_gain * tremolo;

        /* 3. Aerodynamic Wind Noise */
        float x_wind = zero_audio_xorshift32(&audio->noise_seed);
        audio->wind_lpf_state += alpha_wind * (x_wind - audio->wind_lpf_state);
        if (isnan(audio->wind_lpf_state) || isinf(audio->wind_lpf_state)) {
            audio->wind_lpf_state = 0.0f;
        }
        float s_wind = audio->wind_lpf_state * wind_gain;

        /* 4. Gear Shift Pop / Backfire */
        float s_pop = 0.0f;
        if (audio->pop_timer > 0.0f) {
            float t_pop = 0.12f - audio->pop_timer;
            float env_pop = expf(-t_pop / 0.025f);
            audio->pop_phase += ZERO_TWO_PI * 85.0f * dt;
            if (audio->pop_phase >= ZERO_TWO_PI) {
                audio->pop_phase -= ZERO_TWO_PI;
            }
            float pop_noise = zero_audio_xorshift32(&audio->noise_seed);
            s_pop = env_pop * (sinf(audio->pop_phase) * 0.6f + pop_noise * 0.4f) * 0.8f;

            audio->pop_timer -= dt;
            if (audio->pop_timer <= 0.0f) {
                audio->pop_timer = 0.0f;
            }
        }

        /* 5. UI Countdown Chimes and Menu Beeps */
        float s_sfx = 0.0f;
        if (audio->sfx_timer > 0.0f) {
            if (audio->sfx_type == ZERO_SFX_MENU_BEEP) {
                float t_sfx = 0.08f - audio->sfx_timer;
                float env = expf(-t_sfx / 0.035f);
                s_sfx = sinf(audio->sfx_phase) * env * 0.5f;
                audio->sfx_phase += ZERO_TWO_PI * 880.0f * dt;
            } else if (audio->sfx_type == ZERO_SFX_COUNTDOWN_RED) {
                float t_sfx = 0.25f - audio->sfx_timer;
                float env = expf(-t_sfx / 0.08f) * (1.0f - (t_sfx / 0.25f));
                if (env < 0.0f) env = 0.0f;
                s_sfx = sinf(audio->sfx_phase) * env * 0.6f;
                audio->sfx_phase += ZERO_TWO_PI * 440.0f * dt;
            } else if (audio->sfx_type == ZERO_SFX_COUNTDOWN_GREEN) {
                float t_sfx = 0.65f - audio->sfx_timer;
                float env = expf(-t_sfx / 0.18f) * (1.0f - (t_sfx / 0.65f));
                if (env < 0.0f) env = 0.0f;
                float p1 = audio->sfx_phase;
                float triad = sinf(p1) + 0.8f * sinf(p1 * 1.25f) + 0.7f * sinf(p1 * 1.5f);
                s_sfx = (triad * (1.0f / 2.5f)) * env * 0.7f;
                audio->sfx_phase += ZERO_TWO_PI * 880.0f * dt;
            }

            if (audio->sfx_phase >= ZERO_TWO_PI * 4.0f) {
                audio->sfx_phase -= ZERO_TWO_PI * 4.0f;
            }

            audio->sfx_timer -= dt;
            if (audio->sfx_timer <= 0.0f) {
                audio->sfx_timer = 0.0f;
                audio->sfx_type = ZERO_SFX_NONE;
            }
        }

        /* 6. Dynamic Collision Metal Crunch / Grind Synthesizer */
        float s_crash = 0.0f;
        if (audio->crash_timer > 0.0f) {
            float t_crash = 0.38f - audio->crash_timer;
            float env_thud = expf(-t_crash / 0.060f);
            float env_grind = expf(-t_crash / 0.160f);
            audio->crash_phase += ZERO_TWO_PI * (80.0f + 1600.0f * env_thud) * dt;
            if (audio->crash_phase >= ZERO_TWO_PI * 4.0f) {
                audio->crash_phase -= ZERO_TWO_PI * 4.0f;
            }
            float c_noise = zero_audio_xorshift32(&audio->noise_seed);
            float thud = sinf(audio->crash_phase) * env_thud * 0.90f;
            float grind = c_noise * env_grind * 0.80f;
            s_crash = (thud + grind) * audio->crash_gain;

            audio->crash_timer -= dt;
            if (audio->crash_timer <= 0.0f) {
                audio->crash_timer = 0.0f;
            }
        }

        /* 7. Master Mixer with Hyperbolic Tangent Soft-Clipping Saturation */
        float mix = s_engine * 0.55f
                  + s_tire * 0.45f
                  + s_wind * 0.35f
                  + s_pop * 0.50f
                  + s_sfx * 0.60f
                  + s_crash * 0.75f;

        float sat = tanhf(mix);
        if (is_muted) {
            sat = 0.0f;
        }

        float final_sample = sat * master_vol * 32000.0f;
        if (final_sample > 32767.0f) final_sample = 32767.0f;
        if (final_sample < -32767.0f) final_sample = -32767.0f;
        int16_t sample_i16 = (int16_t)final_sample;

        if (out_samples) {
            out_samples[i] = sample_i16;
        }

        audio->ring_buffer[audio->ring_write_pos] = sample_i16;
        audio->ring_write_pos = (audio->ring_write_pos + 1) % ZERO_AUDIO_RING_BUFFER_SIZE;
    }

    pthread_mutex_unlock(&audio->lock);
}

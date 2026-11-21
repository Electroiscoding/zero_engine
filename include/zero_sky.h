#ifndef ZERO_SKY_H
#define ZERO_SKY_H

#include "zero_core.h"
#include "zero_math.h"
#include "zero_camera.h"
#include "zero_shader.h"
#include "zero_mesh.h"
#include "zero_memory.h"
#include "zero_gl_defs.h"

/* Astronomical day phases based on solar elevation */
typedef enum ZeroSkyPhase {
    ZERO_SKY_MIDDAY = 0,
    ZERO_SKY_GOLDEN_HOUR,
    ZERO_SKY_TWILIGHT,
    ZERO_SKY_NIGHT
} ZeroSkyPhase;

/* Atmospheric configuration parameters */
typedef struct ZeroSkyConfig {
    float day_duration_seconds; /* Full diurnal cycle period in seconds (e.g. 120.0f) */
    float sun_inclination_rad;  /* Solar orbit plane tilt in radians (default ~23.44 deg = 0.409f) */
    float turbidity;            /* Aerosol turbidity factor (default 2.5f) */
    float rayleigh_factor;      /* Rayleigh scattering scale factor (default 1.0f) */
    float mie_factor;           /* Mie scattering scale factor (default 1.0f) */
    float mie_g;                /* Henyey-Greenstein asymmetry factor (default 0.76f) */
} ZeroSkyConfig;

/* Live atmospheric state evaluated per frame */
typedef struct ZeroSkyState {
    Vec3 sun_direction;         /* Normalized vector pointing towards the sun */
    Vec3 sun_color;             /* Direct sunlight color and intensity */
    Vec3 ambient_color;         /* Ambient sky bounce illumination */
    Vec3 sky_zenith_color;      /* Zenith sky color */
    Vec3 sky_horizon_color;     /* Horizon sky color */
    float sun_elevation;        /* Sin of sun altitude angle [-1.0, 1.0] */
    ZeroSkyPhase phase;         /* Current astronomical phase */
} ZeroSkyState;

/* Atmosphere renderer object */
typedef struct ZeroSky {
    ZeroSkyConfig config;
    ZeroSkyState state;
    ZeroMesh dome_mesh;
    ZeroShader shader;
    bool is_initialized;
} ZeroSky;

/* Helper for default configuration */
static ZERO_FORCE_INLINE ZeroSkyConfig zero_sky_default_config(void) {
    ZeroSkyConfig cfg;
    cfg.day_duration_seconds = 120.0f;
    cfg.sun_inclination_rad = 0.4090615f;
    cfg.turbidity = 2.5f;
    cfg.rayleigh_factor = 1.0f;
    cfg.mie_factor = 1.0f;
    cfg.mie_g = 0.76f;
    return cfg;
}

/* Lifecycle */
bool zero_sky_init(ZeroSky *sky, ZeroArena *arena, const ZeroSkyConfig *config);
void zero_sky_destroy(ZeroSky *sky);

/* Per-frame simulation and rendering */
void zero_sky_update(ZeroSky *sky, float scene_time);
void zero_sky_render(const ZeroSky *sky, const ZeroCamera *camera);

/* Uniform forwarding to scene / PBR shaders */
void zero_sky_forward_uniforms(const ZeroSky *sky, ZeroShader *target_shader);

/* Pure mathematical calculation functions (callable without ZeroSky or OpenGL context) */
Vec3 zero_sky_calc_sun_direction(float scene_time, float day_duration, float inclination_rad);
ZeroSkyPhase zero_sky_calc_phase(float sun_elevation);
void zero_sky_calc_colors(float sun_elevation, Vec3 *out_sun_color, Vec3 *out_ambient, Vec3 *out_zenith, Vec3 *out_horizon);

/* Shader source access */
const char *zero_sky_shader_vert(void);
const char *zero_sky_shader_frag(void);

#endif /* ZERO_SKY_H */

#include "zero_sky.h"
#include "zero_gl_loader.h"
#include <string.h>
#include <math.h>

static ZERO_FORCE_INLINE float sky_smoothstep(float edge0, float edge1, float x) {
    if (edge1 <= edge0) return 0.0f;
    float t = (x - edge0) / (edge1 - edge0);
    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;
    return t * t * (3.0f - 2.0f * t);
}

static ZERO_FORCE_INLINE Vec3 sky_vec3_lerp(Vec3 a, Vec3 b, float t) {
    return vec3(
        a.x + (b.x - a.x) * t,
        a.y + (b.y - a.y) * t,
        a.z + (b.z - a.z) * t
    );
}

Vec3 zero_sky_calc_sun_direction(float scene_time, float day_duration, float inclination_rad) {
    float period = (day_duration > 0.001f) ? day_duration : 120.0f;
    float norm_t = fmodf(scene_time, period);
    if (norm_t < 0.0f) {
        norm_t += period;
    }
    float phi = (norm_t / period) * (2.0f * ZERO_PI);

    /* Diurnal celestial circle tilted by inclination_rad:
       phi = 0      -> Sunrise in the East (+X)
       phi = pi/2   -> Midday peak (+Y tilted by inclination into +Z)
       phi = pi     -> Sunset in the West (-X)
       phi = 3pi/2  -> Midnight nadir (-Y tilted into -Z) */
    Vec3 dir = vec3(
        cosf(phi),
        sinf(phi) * cosf(inclination_rad),
        sinf(phi) * sinf(inclination_rad)
    );
    return vec3_normalize(dir);
}

ZeroSkyPhase zero_sky_calc_phase(float sun_elevation) {
    if (sun_elevation > 0.35f) {
        return ZERO_SKY_MIDDAY;
    } else if (sun_elevation > 0.05f) {
        return ZERO_SKY_GOLDEN_HOUR;
    } else if (sun_elevation >= -0.15f) {
        return ZERO_SKY_TWILIGHT;
    } else {
        return ZERO_SKY_NIGHT;
    }
}

void zero_sky_calc_colors(float sun_elevation, Vec3 *out_sun_color, Vec3 *out_ambient, Vec3 *out_zenith, Vec3 *out_horizon) {
    /* 4 Canonical Astronomical Color States */
    const Vec3 sun_night    = vec3(0.00f, 0.00f, 0.00f);
    const Vec3 amb_night    = vec3(0.02f, 0.025f, 0.045f);
    const Vec3 zen_night    = vec3(0.01f, 0.015f, 0.035f);
    const Vec3 hor_night    = vec3(0.02f, 0.025f, 0.045f);

    const Vec3 sun_twilight = vec3(0.04f, 0.015f, 0.01f);
    const Vec3 amb_twilight = vec3(0.08f, 0.09f, 0.18f);
    const Vec3 zen_twilight = vec3(0.05f, 0.08f, 0.25f);
    const Vec3 hor_twilight = vec3(0.45f, 0.16f, 0.10f);

    const Vec3 sun_golden   = vec3(1.00f, 0.68f, 0.28f);
    const Vec3 amb_golden   = vec3(0.30f, 0.22f, 0.20f);
    const Vec3 zen_golden   = vec3(0.15f, 0.32f, 0.68f);
    const Vec3 hor_golden   = vec3(0.85f, 0.45f, 0.15f);

    const Vec3 sun_midday   = vec3(1.20f, 1.18f, 1.10f);
    const Vec3 amb_midday   = vec3(0.22f, 0.28f, 0.38f);
    const Vec3 zen_midday   = vec3(0.18f, 0.38f, 0.78f);
    const Vec3 hor_midday   = vec3(0.65f, 0.75f, 0.88f);

    Vec3 sun, amb, zen, hor;

    if (sun_elevation < -0.05f) {
        /* Blend between Night and Twilight: [-0.20, -0.05] */
        float t = sky_smoothstep(-0.20f, -0.05f, sun_elevation);
        sun = sky_vec3_lerp(sun_night, sun_twilight, t);
        amb = sky_vec3_lerp(amb_night, amb_twilight, t);
        zen = sky_vec3_lerp(zen_night, zen_twilight, t);
        hor = sky_vec3_lerp(hor_night, hor_twilight, t);
    } else if (sun_elevation < 0.15f) {
        /* Blend between Twilight and Golden Hour: [-0.05, 0.15] */
        float t = sky_smoothstep(-0.05f, 0.15f, sun_elevation);
        sun = sky_vec3_lerp(sun_twilight, sun_golden, t);
        amb = sky_vec3_lerp(amb_twilight, amb_golden, t);
        zen = sky_vec3_lerp(zen_twilight, zen_golden, t);
        hor = sky_vec3_lerp(hor_twilight, hor_golden, t);
    } else {
        /* Blend between Golden Hour and Midday: [0.15, 0.45] */
        float t = sky_smoothstep(0.15f, 0.45f, sun_elevation);
        sun = sky_vec3_lerp(sun_golden, sun_midday, t);
        amb = sky_vec3_lerp(amb_golden, amb_midday, t);
        zen = sky_vec3_lerp(zen_golden, zen_midday, t);
        hor = sky_vec3_lerp(hor_golden, hor_midday, t);
    }

    if (out_sun_color) *out_sun_color = sun;
    if (out_ambient)   *out_ambient = amb;
    if (out_zenith)    *out_zenith = zen;
    if (out_horizon)   *out_horizon = hor;
}

bool zero_sky_init(ZeroSky *sky, ZeroArena *arena, const ZeroSkyConfig *config) {
    if (!sky) return false;
    memset(sky, 0, sizeof(*sky));

    if (config) {
        sky->config = *config;
    } else {
        sky->config = zero_sky_default_config();
    }

    /* Guard and sanitize config values */
    if (sky->config.day_duration_seconds <= 0.001f) {
        sky->config.day_duration_seconds = 120.0f;
    }
    if (sky->config.turbidity <= 0.001f) {
        sky->config.turbidity = 2.5f;
    }
    if (sky->config.rayleigh_factor <= 0.001f) {
        sky->config.rayleigh_factor = 1.0f;
    }
    if (sky->config.mie_factor <= 0.001f) {
        sky->config.mie_factor = 1.0f;
    }
    if (sky->config.mie_g <= 0.001f || sky->config.mie_g >= 1.0f) {
        sky->config.mie_g = 0.76f;
    }

    /* Initialize state at t = 0 */
    zero_sky_update(sky, 0.0f);

    /* Allocate dome mesh if arena provided */
    if (arena) {
        ZeroMeshData dome_data = zero_mesh_gen_sphere(arena, 1.0f, 16, 32);
        sky->dome_mesh = zero_mesh_upload(&dome_data);
    }

    /* Compile atmospheric shader */
    sky->shader = zero_shader_create(zero_sky_shader_vert(), zero_sky_shader_frag());

    sky->is_initialized = true;
    return true;
}

void zero_sky_destroy(ZeroSky *sky) {
    if (!sky || !sky->is_initialized) return;
    zero_mesh_destroy(&sky->dome_mesh);
    zero_shader_destroy(&sky->shader);
    sky->is_initialized = false;
}

void zero_sky_update(ZeroSky *sky, float scene_time) {
    if (!sky) return;

    sky->state.sun_direction = zero_sky_calc_sun_direction(
        scene_time,
        sky->config.day_duration_seconds,
        sky->config.sun_inclination_rad
    );

    sky->state.sun_elevation = sky->state.sun_direction.y;
    sky->state.phase = zero_sky_calc_phase(sky->state.sun_elevation);

    zero_sky_calc_colors(
        sky->state.sun_elevation,
        &sky->state.sun_color,
        &sky->state.ambient_color,
        &sky->state.sky_zenith_color,
        &sky->state.sky_horizon_color
    );
}

void zero_sky_render(const ZeroSky *sky, const ZeroCamera *camera) {
    if (!sky || !sky->is_initialized || !camera) return;
    if (!sky->shader.is_valid || !sky->dome_mesh.is_valid) return;
    if (!zgl.glUseProgram || !zgl.glBindVertexArray) return;

    /* Render sky at far plane behind all geometry */
    glDepthMask(GL_FALSE);
    glDepthFunc(GL_LEQUAL);
    glDisable(GL_CULL_FACE);

    zero_shader_bind((ZeroShader *)&sky->shader);

    Mat4 view = zero_camera_get_view(camera);
    Mat4 proj = zero_camera_get_projection(camera);

    zero_shader_set_mat4((ZeroShader *)&sky->shader, "u_view", view);
    zero_shader_set_mat4((ZeroShader *)&sky->shader, "u_proj", proj);
    zero_shader_set_vec3((ZeroShader *)&sky->shader, "u_sun_dir", sky->state.sun_direction);
    zero_shader_set_vec3((ZeroShader *)&sky->shader, "u_sun_color", sky->state.sun_color);
    zero_shader_set_vec3((ZeroShader *)&sky->shader, "u_ambient_color", sky->state.ambient_color);

    Vec3 beta_r = vec3(
        5.802e-3f * sky->config.rayleigh_factor,
        13.558e-3f * sky->config.rayleigh_factor,
        33.100e-3f * sky->config.rayleigh_factor
    );
    zero_shader_set_vec3((ZeroShader *)&sky->shader, "u_beta_rayleigh", beta_r);

    float mie_base = 4.0e-3f * (sky->config.turbidity / 2.0f) * sky->config.mie_factor;
    Vec3 beta_m = vec3(mie_base, mie_base, mie_base);
    zero_shader_set_vec3((ZeroShader *)&sky->shader, "u_beta_mie", beta_m);

    zero_shader_set_float((ZeroShader *)&sky->shader, "u_mie_g", sky->config.mie_g);
    zero_shader_set_float((ZeroShader *)&sky->shader, "u_sun_elevation", sky->state.sun_elevation);
    zero_shader_set_vec3((ZeroShader *)&sky->shader, "u_sky_zenith", sky->state.sky_zenith_color);
    zero_shader_set_vec3((ZeroShader *)&sky->shader, "u_sky_horizon", sky->state.sky_horizon_color);

    zero_mesh_draw(&sky->dome_mesh);

    zero_shader_unbind();

    /* Restore depth and culling states */
    glDepthMask(GL_TRUE);
    glDepthFunc(GL_LESS);
    glEnable(GL_CULL_FACE);
}

void zero_sky_forward_uniforms(const ZeroSky *sky, ZeroShader *target_shader) {
    if (!sky || !target_shader || !target_shader->is_valid) return;

    Vec3 sun_dir = sky->state.sun_direction;
    Vec3 ray_dir = vec3_scale(sun_dir, -1.0f);

    /* Forward both convention names for universal shader compatibility */
    zero_shader_set_vec3(target_shader, "u_sun_dir", sun_dir);
    zero_shader_set_vec3(target_shader, "u_light_dir", ray_dir);
    zero_shader_set_vec3(target_shader, "u_sun_color", sky->state.sun_color);
    zero_shader_set_vec3(target_shader, "u_light_color", sky->state.sun_color);
    zero_shader_set_vec3(target_shader, "u_ambient_color", sky->state.ambient_color);
}

const char *zero_sky_shader_vert(void) {
    return
        "#version 330 core\n"
        "layout(location = 0) in vec3 a_pos;\n"
        "layout(location = 1) in vec3 a_normal;\n"
        "layout(location = 2) in vec2 a_uv;\n"
        "\n"
        "uniform mat4 u_view;\n"
        "uniform mat4 u_proj;\n"
        "\n"
        "out vec3 v_ray_dir;\n"
        "\n"
        "void main() {\n"
        "    v_ray_dir = a_pos;\n"
        "    mat4 view_rot = mat4(mat3(u_view));\n"
        "    vec4 clip_pos = u_proj * view_rot * vec4(a_pos, 1.0);\n"
        "    gl_Position = clip_pos.xyww;\n"
        "}\n";
}

const char *zero_sky_shader_frag(void) {
    return
        "#version 330 core\n"
        "in vec3 v_ray_dir;\n"
        "out vec4 frag_color;\n"
        "\n"
        "uniform vec3 u_sun_dir;\n"
        "uniform vec3 u_sun_color;\n"
        "uniform vec3 u_ambient_color;\n"
        "uniform vec3 u_beta_rayleigh;\n"
        "uniform vec3 u_beta_mie;\n"
        "uniform float u_mie_g;\n"
        "uniform float u_sun_elevation;\n"
        "uniform vec3 u_sky_zenith;\n"
        "uniform vec3 u_sky_horizon;\n"
        "\n"
        "const float PI = 3.14159265358979323846;\n"
        "\n"
        "float phase_rayleigh(float cos_theta) {\n"
        "    return (3.0 / (16.0 * PI)) * (1.0 + cos_theta * cos_theta);\n"
        "}\n"
        "\n"
        "float phase_mie(float cos_theta, float g) {\n"
        "    float g2 = g * g;\n"
        "    float denom = 1.0 + g2 - 2.0 * g * cos_theta;\n"
        "    return (1.0 / (4.0 * PI)) * (1.0 - g2) / max(pow(denom, 1.5), 1e-4);\n"
        "}\n"
        "\n"
        "float optical_depth(float cos_zenith) {\n"
        "    float y = max(cos_zenith, 0.0);\n"
        "    return 1.0 / (y + 0.05);\n"
        "}\n"
        "\n"
        "void main() {\n"
        "    vec3 V = normalize(v_ray_dir);\n"
        "    vec3 L = normalize(u_sun_dir);\n"
        "    float cos_theta = dot(V, L);\n"
        "    float cos_zenith = V.y;\n"
        "\n"
        "    float s_view = optical_depth(cos_zenith);\n"
        "    float s_sun = optical_depth(max(L.y, 0.0));\n"
        "\n"
        "    float p_rayleigh = phase_rayleigh(cos_theta);\n"
        "    float p_mie = phase_mie(cos_theta, u_mie_g);\n"
        "\n"
        "    vec3 extinction_sun = exp(-(u_beta_rayleigh + u_beta_mie) * s_sun * 8.0);\n"
        "    vec3 extinction_view = exp(-(u_beta_rayleigh + u_beta_mie) * s_view * 2.0);\n"
        "\n"
        "    vec3 in_scatter = (u_beta_rayleigh * p_rayleigh + u_beta_mie * p_mie) *\n"
        "                      (vec3(1.0) - extinction_view) / max(u_beta_rayleigh + u_beta_mie, vec3(1e-4));\n"
        "    vec3 sky_radiance = in_scatter * u_sun_color * extinction_sun * 20.0;\n"
        "\n"
        "    float h_factor = pow(1.0 - max(cos_zenith, 0.0), 3.0);\n"
        "    vec3 base_sky = mix(u_sky_zenith, u_sky_horizon, h_factor);\n"
        "    sky_radiance += base_sky * (u_ambient_color + 0.1);\n"
        "\n"
        "    if (cos_zenith > -0.05 && L.y > -0.15) {\n"
        "        float sun_disc = smoothstep(0.9996, 0.9999, cos_theta) * 30.0;\n"
        "        sky_radiance += sun_disc * u_sun_color * extinction_sun;\n"
        "    }\n"
        "\n"
        "    if (cos_zenith < 0.0) {\n"
        "        float ground_blend = clamp(-cos_zenith * 5.0, 0.0, 1.0);\n"
        "        vec3 ground_color = u_sky_horizon * 0.35 * u_ambient_color;\n"
        "        sky_radiance = mix(sky_radiance, ground_color, ground_blend);\n"
        "    }\n"
        "\n"
        "    vec3 mapped = sky_radiance / (sky_radiance + vec3(1.0));\n"
        "    mapped = pow(mapped, vec3(1.0 / 2.2));\n"
        "    frag_color = vec4(mapped, 1.0);\n"
        "}\n";
}

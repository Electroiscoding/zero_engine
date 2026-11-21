#include "zero_core.h"
#include "zero_memory.h"
#include "zero_math.h"
#include "zero_platform.h"
#include "zero_gl_loader.h"
#include "zero_shader.h"
#include "zero_mesh.h"
#include "zero_camera.h"
#include "zero_obj.h"
#include "zero_csm.h"
#include "zero_sky.h"
#include "zero_audio.h"
#include "zero_track_mesh.h"
#include "zero_vehicle_physics.h"
#include "zero_ai_racers.h"
#include "zero_race_camera.h"
#include "zero_race_ui.h"
#include "zero_track_spline.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define MAX_PARTICLES 2048

typedef struct ZeroParticle {
    Vec3 pos;
    Vec3 vel;
    Vec4 color;
    float size;
    float life;
    float max_life;
    float drag;
    bool has_gravity;
    bool active;
} ZeroParticle;

typedef struct ZeroParticleSystem {
    ZeroParticle particles[MAX_PARTICLES];
    uint32_t active_count;
    ZeroMesh quad_mesh;
} ZeroParticleSystem;

#define MAX_TRACK_DEBRIS 128

typedef enum ZeroDebrisType {
    ZERO_DEBRIS_FRONT_SPLITTER = 0,
    ZERO_DEBRIS_REAR_WING,
    ZERO_DEBRIS_HOOD,
    ZERO_DEBRIS_WHEEL,
    ZERO_DEBRIS_SHARD
} ZeroDebrisType;

typedef struct ZeroTrackDebris {
    ZeroDebrisType type;
    Vec3 pos;
    Vec3 vel;
    Vec3 ang_vel;
    Quat rot;
    Vec3 color;
    float life;
    float max_life;
    float radius;
    float restitution;
    bool active;
    bool on_ground;
} ZeroTrackDebris;

#define MAX_TRACK_RUPTURES 64

typedef struct ZeroTrackRupture {
    Vec3 pos;
    float radius;
    float intensity;
    float age;
    bool active;
} ZeroTrackRupture;

typedef struct ZeroCarDamage {
    bool lost_front_bumper;
    bool lost_rear_wing;
    bool lost_hood;
} ZeroCarDamage;

static void particle_system_init(ZeroParticleSystem *ps) {
    memset(ps, 0, sizeof(*ps));
    ZeroVertex verts[4] = {
        {{-0.5f, -0.5f, 0.0f}, {0.0f, 0.0f, 1.0f}, {0.0f, 0.0f}},
        {{ 0.5f, -0.5f, 0.0f}, {0.0f, 0.0f, 1.0f}, {1.0f, 0.0f}},
        {{ 0.5f,  0.5f, 0.0f}, {0.0f, 0.0f, 1.0f}, {1.0f, 1.0f}},
        {{-0.5f,  0.5f, 0.0f}, {0.0f, 0.0f, 1.0f}, {0.0f, 1.0f}}
    };
    uint32_t indices[6] = {0, 1, 2, 0, 2, 3};
    ZeroMeshData data = {verts, 4, indices, 6};
    ps->quad_mesh = zero_mesh_upload(&data);
}

static void particle_spawn_ext(ZeroParticleSystem *ps, Vec3 pos, Vec3 vel, Vec4 color, float size, float max_life, bool has_gravity, float drag) {
    for (int i = 0; i < MAX_PARTICLES; ++i) {
        if (!ps->particles[i].active) {
            ps->particles[i].pos = pos;
            ps->particles[i].vel = vel;
            ps->particles[i].color = color;
            ps->particles[i].size = size;
            ps->particles[i].life = max_life;
            ps->particles[i].max_life = max_life;
            ps->particles[i].has_gravity = has_gravity;
            ps->particles[i].drag = drag;
            ps->particles[i].active = true;
            return;
        }
    }
}

static void particle_spawn(ZeroParticleSystem *ps, Vec3 pos, Vec3 vel, Vec4 color, float size, float max_life) {
    particle_spawn_ext(ps, pos, vel, color, size, max_life, false, 0.94f);
}

static void particle_system_update(ZeroParticleSystem *ps, float dt) {
    for (int i = 0; i < MAX_PARTICLES; ++i) {
        if (!ps->particles[i].active) continue;
        ZeroParticle *p = &ps->particles[i];
        p->life -= dt;
        if (p->life <= 0.0f) {
            p->active = false;
            continue;
        }
        if (p->has_gravity) {
            p->vel.y -= 19.0f * dt;
        }
        p->pos = vec3_add(p->pos, vec3_scale(p->vel, dt));
        p->vel = vec3_scale(p->vel, p->drag);

        if (p->has_gravity && p->pos.y < 0.04f) {
            p->pos.y = 0.04f;
            if (p->vel.y < -0.4f) {
                p->vel.y = -p->vel.y * 0.52f;
                p->vel.x *= 0.72f;
                p->vel.z *= 0.72f;
            } else {
                p->vel.y = 0.0f;
            }
        }
    }
}

static void debris_spawn(ZeroTrackDebris *pool, ZeroDebrisType type, Vec3 pos, Vec3 vel, Vec3 ang_vel, Vec3 color, float radius, float max_life) {
    for (int i = 0; i < MAX_TRACK_DEBRIS; ++i) {
        if (!pool[i].active) {
            pool[i].type = type;
            pool[i].pos = pos;
            pool[i].vel = vel;
            pool[i].ang_vel = ang_vel;
            pool[i].rot = quat_identity();
            pool[i].color = color;
            pool[i].life = max_life;
            pool[i].max_life = max_life;
            pool[i].radius = radius;
            pool[i].restitution = (type == ZERO_DEBRIS_WHEEL) ? 0.70f : 0.40f;
            pool[i].active = true;
            pool[i].on_ground = false;
            return;
        }
    }
}

static void debris_system_update(ZeroTrackDebris *pool, ZeroParticleSystem *ps, ZeroRaceGame *game, float dt) {
    for (int i = 0; i < MAX_TRACK_DEBRIS; ++i) {
        if (!pool[i].active) continue;
        ZeroTrackDebris *d = &pool[i];
        d->life -= dt;
        if (d->life <= 0.0f) {
            d->active = false;
            continue;
        }

        d->vel.y -= 18.0f * dt;
        d->vel = vec3_scale(d->vel, 1.0f - 0.22f * dt);
        d->pos = vec3_add(d->pos, vec3_scale(d->vel, dt));

        float ang_speed = vec3_length(d->ang_vel);
        if (ang_speed > 1e-4f) {
            Vec3 axis = vec3_scale(d->ang_vel, 1.0f / ang_speed);
            Quat dq = quat_from_axis_angle(axis, ang_speed * dt);
            d->rot = quat_normalize(quat_mul(dq, d->rot));
        }

        float ground_y = 0.0f;
        if (d->pos.y < ground_y + d->radius) {
            d->pos.y = ground_y + d->radius;
            if (d->vel.y < -0.8f) {
                d->vel.y = -d->vel.y * d->restitution;
                d->vel.x *= 0.70f;
                d->vel.z *= 0.70f;
                d->ang_vel = vec3_scale(d->ang_vel, 0.60f);

                if (d->type == ZERO_DEBRIS_FRONT_SPLITTER || d->type == ZERO_DEBRIS_SHARD) {
                    for (int s = 0; s < 3; ++s) {
                        Vec3 spk_v = vec3(
                            ((float)rand() / (float)RAND_MAX - 0.5f) * 4.0f,
                            1.0f + ((float)rand() / (float)RAND_MAX) * 2.5f,
                            ((float)rand() / (float)RAND_MAX - 0.5f) * 4.0f
                        );
                        particle_spawn_ext(ps, d->pos, spk_v, vec4(1.0f, 0.85f, 0.2f, 0.95f), 0.18f, 0.25f, true, 0.95f);
                    }
                }
            } else {
                d->vel.y = 0.0f;
                d->vel.x *= 0.88f;
                d->vel.z *= 0.88f;
                d->ang_vel = vec3_scale(d->ang_vel, 0.82f);
                d->on_ground = true;
            }
        }

        int b_node = 0; float b_t = 0.0f; float b_dist_sq = 0.0f;
        zero_track_find_nearest_segment(d->pos, &b_node, &b_t, &b_dist_sq);
        float b_dist = sqrtf(b_dist_sq);
        if (b_dist > (ZERO_TRACK_BARRIER_DIST - 1.20f)) {
            Vec3 to_track = vec3_sub(vec3(g_track_spline[b_node][0], 0, g_track_spline[b_node][1]), d->pos);
            to_track.y = 0.0f;
            float t_len = vec3_length(to_track);
            if (t_len > 1e-4f) {
                Vec3 in_norm = vec3_scale(to_track, 1.0f / t_len);
                float vn = vec3_dot(d->vel, in_norm);
                if (vn < 0.0f) {
                    d->vel = vec3_sub(d->vel, vec3_scale(in_norm, 1.6f * vn));
                    d->vel.y += 2.0f;
                }
            }
        }

        for (int k = 0; k < ZERO_RACE_CAR_COUNT; ++k) {
            ZeroVehicleEnhanced *car = &game->cars[k];
            Vec3 c_diff = vec3_sub(d->pos, car->position);
            c_diff.y = 0.0f;
            float c_dist = vec3_length(c_diff);
            if (c_dist < (d->radius + 1.25f)) {
                d->vel = vec3_add(d->vel, vec3_scale(car->velocity, 0.65f));
                d->vel.y += 5.5f + ((float)rand() / (float)RAND_MAX) * 3.5f;
                d->ang_vel = vec3(
                    ((float)rand() / (float)RAND_MAX - 0.5f) * 22.0f,
                    ((float)rand() / (float)RAND_MAX - 0.5f) * 25.0f,
                    ((float)rand() / (float)RAND_MAX - 0.5f) * 22.0f
                );
                car->velocity.y += 0.35f;
                for (int sp = 0; sp < 6; ++sp) {
                    Vec3 spk_v = vec3(
                        ((float)rand() / (float)RAND_MAX - 0.5f) * 6.0f,
                        2.0f + ((float)rand() / (float)RAND_MAX) * 3.0f,
                        ((float)rand() / (float)RAND_MAX - 0.5f) * 6.0f
                    );
                    particle_spawn_ext(ps, d->pos, spk_v, vec4(1.0f, 0.8f, 0.15f, 0.95f), 0.22f, 0.35f, true, 0.95f);
                }
            }
        }
    }
}

static void track_rupture_spawn(ZeroTrackRupture *ruptures, Vec3 pos, float radius) {
    for (int i = 0; i < MAX_TRACK_RUPTURES; ++i) {
        if (!ruptures[i].active) {
            ruptures[i].pos = pos;
            ruptures[i].pos.y = 0.015f;
            ruptures[i].radius = radius;
            ruptures[i].intensity = 1.0f;
            ruptures[i].age = 0.0f;
            ruptures[i].active = true;
            return;
        }
    }
}

static void track_rupture_update(ZeroTrackRupture *ruptures, float dt) {
    for (int i = 0; i < MAX_TRACK_RUPTURES; ++i) {
        if (!ruptures[i].active) continue;
        ruptures[i].age += dt;
        ruptures[i].intensity = expf(-ruptures[i].age / 14.0f);
    }
}

static const char *get_race_vert_shader(void) {
    return
        "#version 330 core\n"
        "layout(location = 0) in vec3 a_pos;\n"
        "layout(location = 1) in vec3 a_normal;\n"
        "layout(location = 2) in vec2 a_uv;\n"
        "uniform mat4 u_model;\n"
        "uniform mat4 u_view;\n"
        "uniform mat4 u_proj;\n"
        "uniform float u_is_sky;\n"
        "out vec3 v_normal;\n"
        "out vec3 v_world_pos;\n"
        "out vec2 v_uv;\n"
        "out vec3 v_sky_ray;\n"
        "void main() {\n"
        "    if (u_is_sky > 0.5) {\n"
        "        v_uv = a_uv;\n"
        "        v_world_pos = a_pos;\n"
        "        v_normal = -normalize(a_pos);\n"
        "        v_sky_ray = normalize(a_pos);\n"
        "        mat4 rot_view = mat4(mat3(u_view));\n"
        "        gl_Position = (u_proj * rot_view * vec4(a_pos, 1.0)).xyww;\n"
        "    } else {\n"
        "        vec4 world_pos = u_model * vec4(a_pos, 1.0);\n"
        "        v_world_pos = world_pos.xyz;\n"
        "        v_normal = mat3(u_model) * a_normal;\n"
        "        v_uv = a_uv;\n"
        "        gl_Position = u_proj * u_view * world_pos;\n"
        "    }\n"
        "}\n";
}

static const char *get_race_frag_shader(void) {
    return
        "#version 330 core\n"
        "in vec3 v_normal;\n"
        "in vec3 v_world_pos;\n"
        "in vec2 v_uv;\n"
        "in vec3 v_sky_ray;\n"
        "uniform vec3 u_light_dir;\n"
        "uniform vec3 u_light_color;\n"
        "uniform vec3 u_ambient_color;\n"
        "uniform vec3 u_object_color;\n"
        "uniform vec3 u_camera_pos;\n"
        "uniform float u_roughness;\n"
        "uniform float u_metallic;\n"
        "uniform float u_material_mode;\n"
        "uniform float u_is_sky;\n"
        "uniform float u_time;\n"
        "out vec4 frag_color;\n"
        "\n"
        "const float PI = 3.14159265359;\n"
        "\n"
        "float distributionGGX(vec3 N, vec3 H, float roughness) {\n"
        "    float a = roughness * roughness;\n"
        "    float a2 = a * a;\n"
        "    float NdotH = max(dot(N, H), 0.0);\n"
        "    float NdotH2 = NdotH * NdotH;\n"
        "    float denom = (NdotH2 * (a2 - 1.0) + 1.0);\n"
        "    return a2 / (PI * denom * denom + 0.0001);\n"
        "}\n"
        "\n"
        "float geometrySchlickGGX(float NdotV, float roughness) {\n"
        "    float r = (roughness + 1.0);\n"
        "    float k = (r * r) / 8.0;\n"
        "    return NdotV / (NdotV * (1.0 - k) + k);\n"
        "}\n"
        "\n"
        "float geometrySmith(vec3 N, vec3 V, vec3 L, float roughness) {\n"
        "    float NdotV = max(dot(N, V), 0.0);\n"
        "    float NdotL = max(dot(N, L), 0.0);\n"
        "    return geometrySchlickGGX(NdotV, roughness) * geometrySchlickGGX(NdotL, roughness);\n"
        "}\n"
        "\n"
        "vec3 fresnelSchlick(float cosTheta, vec3 F0) {\n"
        "    return F0 + (1.0 - F0) * pow(clamp(1.0 - cosTheta, 0.0, 1.0), 5.0);\n"
        "}\n"
        "\n"
        "void main() {\n"
        "    // 1. Procedural Diurnal Sky Dome Pass (Radiant Sun, Lens Flare, Azure Atmosphere, Dynamic Clouds)\n"
        "    if (u_is_sky > 0.5) {\n"
        "        vec3 ray = normalize(v_sky_ray);\n"
        "        vec3 sun_d = normalize(-u_light_dir);\n"
        "        float sun_dot = max(dot(ray, sun_d), 0.0);\n"
        "\n"
        "        vec3 sky_zenith = vec3(0.08, 0.35, 0.88);\n"
        "        vec3 sky_mid = vec3(0.35, 0.65, 0.98);\n"
        "        vec3 sky_horizon = vec3(0.72, 0.85, 0.98);\n"
        "        vec3 ground_color = vec3(0.25, 0.35, 0.42);\n"
        "\n"
        "        vec3 sky = (ray.y > 0.0) ? mix(sky_horizon, mix(sky_mid, sky_zenith, pow(ray.y, 0.50)), ray.y)\n"
        "                                 : mix(sky_horizon, ground_color, clamp(-ray.y * 3.0, 0.0, 1.0));\n"
        "\n"
        "        // Radiant Blazing Sun Disk with Multiphase Solar Flare\n"
        "        float sun_disk = pow(sun_dot, 750.0) * 35.0;\n"
        "        float sun_corona = pow(sun_dot, 28.0) * 4.5;\n"
        "        float sun_flare = pow(sun_dot, 6.0) * 1.2;\n"
        "        vec3 sun = vec3(1.0, 0.95, 0.82) * sun_disk + vec3(1.0, 0.82, 0.45) * sun_corona + vec3(0.95, 0.60, 0.25) * sun_flare;\n"
        "\n"
        "        // Drifting Procedural Clouds\n"
        "        if (ray.y > 0.02) {\n"
        "            vec2 cloud_coord = ray.xz / (ray.y + 0.10) * 0.14 + vec2(u_time * 0.012, 0.0);\n"
        "            float c_noise = sin(cloud_coord.x * 3.0) * cos(cloud_coord.y * 3.0);\n"
        "            c_noise += sin(cloud_coord.x * 6.5 + cloud_coord.y * 4.5) * 0.5;\n"
        "            float cloud_alpha = clamp((c_noise - 0.12) * 2.2, 0.0, 0.75);\n"
        "            sky = mix(sky, vec3(0.98, 0.99, 1.0), cloud_alpha);\n"
        "        }\n"
        "\n"
        "        vec3 final_sky = sky + sun;\n"
        "        final_sky = final_sky / (final_sky + vec3(1.0));\n"
        "        final_sky = pow(final_sky, vec3(1.0 / 2.2));\n"
        "        frag_color = vec4(final_sky, 1.0);\n"
        "        return;\n"
        "    }\n"
        "\n"
        "    // 2. Soft Contact Drop Shadow Quad\n"
        "    if (u_material_mode > 4.5 && u_material_mode < 5.5) {\n"
        "        float dist = length(v_uv - vec2(0.5));\n"
        "        float alpha = clamp((0.5 - dist) * 2.8, 0.0, 0.75);\n"
        "        frag_color = vec4(0.02, 0.02, 0.03, alpha);\n"
        "        return;\n"
        "    }\n"
        "\n"
        "    // 3. Billboarding Particles (Tire Smoke, Barrier Sparks, Exhaust Fire, Glass Shards)\n"
        "    if (u_material_mode > 8.5 && u_material_mode < 9.5) {\n"
        "        float dist = length(v_uv - vec2(0.5));\n"
        "        float alpha = clamp((0.5 - dist) * 2.0, 0.0, 1.0) * u_roughness;\n"
        "        vec3 col = u_object_color;\n"
        "        if (dist < 0.20) {\n"
        "            col += vec3(0.6, 0.5, 0.4) * (0.20 - dist) * 5.0;\n"
        "        }\n"
        "        frag_color = vec4(col, alpha);\n"
        "        return;\n"
        "    }\n"
        "\n"
        "    // 4. 3D Cook-Torrance Specular PBR + Clearcoat\n"
        "    vec3 N = normalize(v_normal);\n"
        "    vec3 V = normalize(u_camera_pos - v_world_pos);\n"
        "    vec3 L = normalize(-u_light_dir);\n"
        "    vec3 H = normalize(V + L);\n"
        "\n"
        "    float NdotL = max(dot(N, L), 0.0);\n"
        "    float NdotV = max(dot(N, V), 0.001);\n"
        "    float NdotH = max(dot(N, H), 0.0);\n"
        "    float VdotH = max(dot(V, H), 0.0);\n"
        "\n"
        "    vec3 albedo = u_object_color;\n"
        "    float roughness = clamp(u_roughness, 0.04, 0.96);\n"
        "    float metallic = clamp(u_metallic, 0.0, 1.0);\n"
        "    vec3 emissive = vec3(0.0);\n"
        "\n"
        "    // Mode 0: Sports Car Chassis (Full Sub-Material Decoding)\n"
        "    if (u_material_mode < 0.5) {\n"
        "        float sub_mat = v_uv.x;\n"
        "        if (sub_mat > 0.5 && sub_mat < 1.5) {\n"
        "            // Tinted Glass Canopy\n"
        "            albedo = vec3(0.03, 0.04, 0.06);\n"
        "            roughness = 0.03;\n"
        "            metallic = 0.15;\n"
        "        } else if (sub_mat > 1.5 && sub_mat < 2.5) {\n"
        "            // Carbon Fiber Aero Diffuser / Spoiler\n"
        "            albedo = vec3(0.05, 0.05, 0.06);\n"
        "            roughness = 0.35;\n"
        "            metallic = 0.30;\n"
        "        } else if (sub_mat > 2.5 && sub_mat < 3.5) {\n"
        "            // Gloss Black Trim & Bumpers\n"
        "            albedo = vec3(0.02, 0.02, 0.02);\n"
        "            roughness = 0.08;\n"
        "            metallic = 0.45;\n"
        "        } else if (sub_mat > 3.5 && sub_mat < 4.5) {\n"
        "            // Emissive Taillight Neon Bar\n"
        "            albedo = vec3(0.98, 0.02, 0.02);\n"
        "            emissive = vec3(3.5, 0.1, 0.1);\n"
        "            roughness = 0.15;\n"
        "        } else if (sub_mat > 4.5 && sub_mat < 5.5) {\n"
        "            // Emissive Xenon Headlights\n"
        "            albedo = vec3(0.96, 0.98, 1.0);\n"
        "            emissive = vec3(3.8, 3.8, 4.0);\n"
        "            roughness = 0.08;\n"
        "        } else if (sub_mat > 5.5 && sub_mat < 6.5) {\n"
        "            // Chrome Quad Exhaust Tips\n"
        "            albedo = vec3(0.88, 0.90, 0.92);\n"
        "            roughness = 0.12;\n"
        "            metallic = 0.95;\n"
        "        }\n"
        "    }\n"
        "    // Mode 1: Asphalt Road (Clean white dashes and border lines)\n"
        "    else if (u_material_mode > 0.5 && u_material_mode < 1.5) {\n"
        "        float u_track = v_uv.x;\n"
        "        float v_dist = v_uv.y;\n"
        "        float center_dist = abs(u_track - 0.5);\n"
        "        if (center_dist < 0.012) {\n"
        "            float dash = mod(v_dist * 8.0, 2.0);\n"
        "            if (dash < 1.0) albedo = vec3(0.98, 0.98, 0.98);\n"
        "        }\n"
        "        if (abs(u_track - 0.05) < 0.010 || abs(u_track - 0.95) < 0.010) {\n"
        "            albedo = vec3(0.95, 0.95, 0.92);\n"
        "        }\n"
        "    }\n"
        "    // Mode 2: Rumble Curbs (High-visibility red and white alternating stripes)\n"
        "    else if (u_material_mode > 1.5 && u_material_mode < 2.5) {\n"
        "        float curb_step = mod(v_uv.y * 30.0, 2.0);\n"
        "        albedo = (curb_step < 1.0) ? vec3(0.95, 0.06, 0.08) : vec3(0.98, 0.98, 0.98);\n"
        "    }\n"
        "    // Mode 3: Lush Green Terrain Checkerboard\n"
        "    else if (u_material_mode > 2.5 && u_material_mode < 3.5) {\n"
        "        float cx = floor(v_world_pos.x * 0.10);\n"
        "        float cz = floor(v_world_pos.z * 0.10);\n"
        "        float chk = mod(cx + cz, 2.0);\n"
        "        albedo = (chk < 1.0) ? vec3(0.24, 0.56, 0.22) : vec3(0.19, 0.46, 0.18);\n"
        "    }\n"
        "    // Mode 4: Armco Barriers with Bright Safety Stripes\n"
        "    else if (u_material_mode > 3.5 && u_material_mode < 4.5) {\n"
        "        float b_step = mod(v_world_pos.x + v_world_pos.z, 4.0);\n"
        "        albedo = (b_step < 2.0) ? vec3(0.20, 0.48, 0.95) : vec3(0.95, 0.95, 0.98);\n"
        "    }\n"
        "    // Mode 6: Wheels (Rubber Tires, Alloy Rims, Red Brembo Calipers)\n"
        "    else if (u_material_mode > 5.5 && u_material_mode < 6.5) {\n"
        "        float w_mat = v_uv.x;\n"
        "        if (w_mat > 0.5 && w_mat < 1.5) {\n"
        "            // Matte Black Rubber Tire\n"
        "            albedo = vec3(0.10, 0.11, 0.12);\n"
        "            roughness = 0.75;\n"
        "            metallic = 0.05;\n"
        "        } else if (w_mat > 1.5 && w_mat < 2.5) {\n"
        "            // Brushed Silver Alloy Rim\n"
        "            albedo = vec3(0.88, 0.90, 0.94);\n"
        "            roughness = 0.15;\n"
        "            metallic = 0.95;\n"
        "        } else if (w_mat > 2.5 && w_mat < 3.5) {\n"
        "            // Brembo Red Brake Caliper\n"
        "            albedo = vec3(0.92, 0.06, 0.08);\n"
        "            roughness = 0.20;\n"
        "            metallic = 0.50;\n"
        "        }\n"
        "    }\n"
        "    // Mode 7: Grandstand Audience Spectator Crowd (Multi-colored shirts)\n"
        "    else if (u_material_mode > 6.5 && u_material_mode < 7.5) {\n"
        "        float cid = floor(v_uv.x * 24.0) + floor(v_uv.y * 12.0);\n"
        "        float r_val = fract(sin(cid * 91.345) * 47453.545);\n"
        "        if (r_val < 0.20) albedo = vec3(0.92, 0.12, 0.14);\n"
        "        else if (r_val < 0.40) albedo = vec3(0.15, 0.48, 0.95);\n"
        "        else if (r_val < 0.60) albedo = vec3(0.96, 0.84, 0.08);\n"
        "        else if (r_val < 0.80) albedo = vec3(0.95, 0.95, 0.95);\n"
        "        else albedo = vec3(0.10, 0.88, 0.55);\n"
        "        roughness = 0.55;\n"
        "        metallic = 0.05;\n"
        "    }\n"
        "    // Mode 8: Sponsor Billboards (Pirelli, Brembo, Shell, Apex)\n"
        "    else if (u_material_mode > 7.5 && u_material_mode < 8.5) {\n"
        "        float b_idx = floor(v_uv.x * 4.0);\n"
        "        if (b_idx < 1.0) albedo = vec3(0.94, 0.08, 0.10);\n"
        "        else if (b_idx < 2.0) albedo = vec3(0.98, 0.86, 0.06);\n"
        "        else if (b_idx < 3.0) albedo = vec3(0.08, 0.48, 0.98);\n"
        "        else albedo = vec3(0.05, 0.05, 0.06);\n"
        "        roughness = 0.25;\n"
        "        metallic = 0.20;\n"
        "    }\n"
        "    // Mode 10: Ruptured Asphalt Battle Crater\n"
        "    else if (u_material_mode > 9.5 && u_material_mode < 10.5) {\n"
        "        vec2 p = v_uv * 2.0 - 1.0;\n"
        "        float r = length(p);\n"
        "        if (r > 1.0) discard;\n"
        "        float angle = atan(p.y, p.x);\n"
        "        float fissure = abs(sin(angle * 7.0 + r * 12.0)) * 0.22;\n"
        "        if (r + fissure > 1.0) discard;\n"
        "        albedo = vec3(0.03, 0.03, 0.035);\n"
        "        roughness = 0.92;\n"
        "        metallic = 0.02;\n"
        "        float ember = clamp(1.0 - r * 2.5, 0.0, 1.0);\n"
        "        emissive = vec3(3.5, 1.1, 0.18) * pow(ember, 2.2) * u_roughness;\n"
        "    }\n"
        "    // Mode 11: Carbon-Fiber Shrapnel & Jagged Debris\n"
        "    else if (u_material_mode > 10.5 && u_material_mode < 11.5) {\n"
        "        float weave = mod(floor(v_world_pos.x * 50.0) + floor(v_world_pos.z * 50.0), 2.0);\n"
        "        albedo = (weave < 1.0) ? u_object_color : vec3(0.06, 0.06, 0.07);\n"
        "        roughness = 0.20;\n"
        "        metallic = 0.75;\n"
        "    }\n"
        "\n"
        "    vec3 F0 = mix(vec3(0.04), albedo, metallic);\n"
        "    float NDF = distributionGGX(N, H, roughness);\n"
        "    float G = geometrySmith(N, V, L, roughness);\n"
        "    vec3 F = fresnelSchlick(VdotH, F0);\n"
        "\n"
        "    vec3 numerator = NDF * G * F;\n"
        "    float denominator = 4.0 * NdotV * NdotL + 0.0001;\n"
        "    vec3 specular = numerator / denominator;\n"
        "\n"
        "    vec3 kS = F;\n"
        "    vec3 kD = (vec3(1.0) - kS) * (1.0 - metallic);\n"
        "\n"
        "    vec3 diffuse = (kD * albedo / PI) * u_light_color * NdotL;\n"
        "    vec3 spec_light = specular * u_light_color * NdotL;\n"
        "\n"
        "    // Secondary Automotive Clearcoat Highlight\n"
        "    float cc_NDF = distributionGGX(N, H, 0.06);\n"
        "    float cc_G = geometrySmith(N, V, L, 0.06);\n"
        "    vec3 cc_F = fresnelSchlick(VdotH, vec3(0.04));\n"
        "    vec3 cc_spec = (cc_NDF * cc_G * cc_F) / (4.0 * NdotV * NdotL + 0.0001) * 0.75;\n"
        "\n"
        "    // Environmental Dynamic Sky Reflections\n"
        "    vec3 R = reflect(-V, N);\n"
        "    vec3 sky_zenith = vec3(0.12, 0.38, 0.88);\n"
        "    vec3 sky_horizon = vec3(0.78, 0.86, 0.96);\n"
        "    vec3 env_reflection = mix(sky_horizon, sky_zenith, clamp(R.y * 0.5 + 0.5, 0.0, 1.0));\n"
        "\n"
        "    vec3 ambient = u_ambient_color * albedo + env_reflection * F * 0.55;\n"
        "    vec3 color = ambient + diffuse + spec_light + cc_spec * NdotL + emissive;\n"
        "\n"
        "    // Long-distance atmospheric haze (starts past 500m)\n"
        "    float dist = length(u_camera_pos - v_world_pos);\n"
        "    float fog_factor = clamp((dist - 500.0) / 1000.0, 0.0, 0.50);\n"
        "    color = mix(color, sky_horizon, fog_factor);\n"
        "\n"
        "    // Filmic Reinhard Tone Mapping + Gamma Correction\n"
        "    color = color / (color + vec3(1.0));\n"
        "    color = pow(color, vec3(1.0 / 2.2));\n"
        "\n"
        "    frag_color = vec4(color, 1.0);\n"
        "}\n";
}

static ZeroMesh create_shadow_quad(void) {
    ZeroVertex verts[4] = {
        {{-0.5f, 0.0f, -0.5f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f}},
        {{ 0.5f, 0.0f, -0.5f}, {0.0f, 1.0f, 0.0f}, {1.0f, 0.0f}},
        {{ 0.5f, 0.0f,  0.5f}, {0.0f, 1.0f, 0.0f}, {1.0f, 1.0f}},
        {{-0.5f, 0.0f,  0.5f}, {0.0f, 1.0f, 0.0f}, {0.0f, 1.0f}}
    };
    uint32_t indices[6] = {0, 1, 2, 0, 2, 3};
    ZeroMeshData data = {verts, 4, indices, 6};
    return zero_mesh_upload(&data);
}

static ZeroMesh create_rupture_quad(void) {
    ZeroVertex verts[4] = {
        {{-0.5f, 0.0f, -0.5f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f}},
        {{ 0.5f, 0.0f, -0.5f}, {0.0f, 1.0f, 0.0f}, {1.0f, 0.0f}},
        {{ 0.5f, 0.0f,  0.5f}, {0.0f, 1.0f, 0.0f}, {1.0f, 1.0f}},
        {{-0.5f, 0.0f,  0.5f}, {0.0f, 1.0f, 0.0f}, {0.0f, 1.0f}}
    };
    uint32_t indices[6] = {0, 1, 2, 0, 2, 3};
    ZeroMeshData data = {verts, 4, indices, 6};
    return zero_mesh_upload(&data);
}

static ZeroMesh create_shard_mesh(void) {
    ZeroVertex verts[6] = {
        {{-0.25f,  0.02f, -0.20f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f}},
        {{ 0.30f,  0.03f, -0.15f}, {0.0f, 1.0f, 0.0f}, {1.0f, 0.0f}},
        {{ 0.05f, -0.01f,  0.28f}, {0.0f, 1.0f, 0.0f}, {0.5f, 1.0f}},
        {{-0.25f, -0.04f, -0.20f}, {0.0f, -1.0f, 0.0f}, {0.0f, 0.0f}},
        {{ 0.30f, -0.03f, -0.15f}, {0.0f, -1.0f, 0.0f}, {1.0f, 0.0f}},
        {{ 0.05f, -0.05f,  0.28f}, {0.0f, -1.0f, 0.0f}, {0.5f, 1.0f}}
    };
    uint32_t indices[24] = {
        0, 1, 2,  3, 5, 4,
        0, 3, 4,  0, 4, 1,
        1, 4, 5,  1, 5, 2,
        2, 5, 3,  2, 3, 0
    };
    ZeroMeshData data = {verts, 6, indices, 24};
    return zero_mesh_upload(&data);
}

static ZeroMesh create_front_splitter_mesh(void) {
    ZeroVertex verts[16] = {
        {{-0.95f, -0.04f, -0.22f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f}},
        {{ 0.95f, -0.04f, -0.22f}, {0.0f, 1.0f, 0.0f}, {1.0f, 0.0f}},
        {{ 0.92f, -0.03f,  0.25f}, {0.0f, 1.0f, 0.0f}, {1.0f, 1.0f}},
        {{-0.92f, -0.03f,  0.25f}, {0.0f, 1.0f, 0.0f}, {0.0f, 1.0f}},
        {{-0.95f,  0.03f, -0.22f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f}},
        {{ 0.95f,  0.03f, -0.22f}, {0.0f, 1.0f, 0.0f}, {1.0f, 0.0f}},
        {{ 0.92f,  0.04f,  0.25f}, {0.0f, 1.0f, 0.0f}, {1.0f, 1.0f}},
        {{-0.92f,  0.04f,  0.25f}, {0.0f, 1.0f, 0.0f}, {0.0f, 1.0f}},
        {{-0.95f, -0.04f, -0.15f}, {-1.0f, 0.0f, 0.0f}, {0.0f, 0.0f}},
        {{-0.95f,  0.22f, -0.10f}, {-1.0f, 0.0f, 0.0f}, {0.0f, 1.0f}},
        {{-0.92f,  0.24f,  0.22f}, {-1.0f, 0.0f, 0.0f}, {1.0f, 1.0f}},
        {{-0.92f, -0.03f,  0.25f}, {-1.0f, 0.0f, 0.0f}, {1.0f, 0.0f}},
        {{ 0.95f, -0.04f, -0.15f}, {1.0f, 0.0f, 0.0f}, {0.0f, 0.0f}},
        {{ 0.95f,  0.22f, -0.10f}, {1.0f, 0.0f, 0.0f}, {0.0f, 1.0f}},
        {{ 0.92f,  0.24f,  0.22f}, {1.0f, 0.0f, 0.0f}, {1.0f, 1.0f}},
        {{ 0.92f, -0.03f,  0.25f}, {1.0f, 0.0f, 0.0f}, {1.0f, 0.0f}}
    };
    uint32_t indices[48] = {
        4, 5, 6,  4, 6, 7,
        0, 2, 1,  0, 3, 2,
        0, 1, 5,  0, 5, 4,
        1, 2, 6,  1, 6, 5,
        2, 3, 7,  2, 7, 6,
        3, 0, 4,  3, 4, 7,
        8, 9, 10,  8, 10, 11,
        12, 14, 13,  12, 15, 14
    };
    ZeroMeshData data = {verts, 16, indices, 48};
    return zero_mesh_upload(&data);
}

static ZeroMesh create_rear_wing_mesh(void) {
    ZeroVertex verts[20] = {
        {{-0.92f,  0.00f, -0.18f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f}},
        {{ 0.92f,  0.00f, -0.18f}, {0.0f, 1.0f, 0.0f}, {1.0f, 0.0f}},
        {{ 0.92f,  0.04f,  0.18f}, {0.0f, 1.0f, 0.0f}, {1.0f, 1.0f}},
        {{-0.92f,  0.04f,  0.18f}, {0.0f, 1.0f, 0.0f}, {0.0f, 1.0f}},
        {{-0.92f, -0.04f, -0.18f}, {0.0f, -1.0f, 0.0f}, {0.0f, 0.0f}},
        {{ 0.92f, -0.04f, -0.18f}, {0.0f, -1.0f, 0.0f}, {1.0f, 0.0f}},
        {{ 0.92f,  0.00f,  0.18f}, {0.0f, -1.0f, 0.0f}, {1.0f, 1.0f}},
        {{-0.92f,  0.00f,  0.18f}, {0.0f, -1.0f, 0.0f}, {0.0f, 1.0f}},
        {{-0.93f, -0.15f, -0.22f}, {-1.0f, 0.0f, 0.0f}, {0.0f, 0.0f}},
        {{-0.93f,  0.22f, -0.22f}, {-1.0f, 0.0f, 0.0f}, {0.0f, 1.0f}},
        {{-0.93f,  0.22f,  0.22f}, {-1.0f, 0.0f, 0.0f}, {1.0f, 1.0f}},
        {{-0.93f, -0.15f,  0.22f}, {-1.0f, 0.0f, 0.0f}, {1.0f, 0.0f}},
        {{ 0.93f, -0.15f, -0.22f}, {1.0f, 0.0f, 0.0f}, {0.0f, 0.0f}},
        {{ 0.93f,  0.22f, -0.22f}, {1.0f, 0.0f, 0.0f}, {0.0f, 1.0f}},
        {{ 0.93f,  0.22f,  0.22f}, {1.0f, 0.0f, 0.0f}, {1.0f, 1.0f}},
        {{ 0.93f, -0.15f,  0.22f}, {1.0f, 0.0f, 0.0f}, {1.0f, 0.0f}},
        {{-0.42f, -0.45f,  0.00f}, {0.0f, 0.0f, 1.0f}, {0.0f, 0.0f}},
        {{-0.42f,  0.00f,  0.00f}, {0.0f, 0.0f, 1.0f}, {0.0f, 1.0f}},
        {{ 0.42f, -0.45f,  0.00f}, {0.0f, 0.0f, 1.0f}, {1.0f, 0.0f}},
        {{ 0.42f,  0.00f,  0.00f}, {0.0f, 0.0f, 1.0f}, {1.0f, 1.0f}}
    };
    uint32_t indices[48] = {
        0, 1, 2,  0, 2, 3,
        4, 6, 5,  4, 7, 6,
        0, 4, 5,  0, 5, 1,
        2, 6, 7,  2, 7, 3,
        8, 9, 10,  8, 10, 11,
        12, 14, 13,  12, 15, 14,
        16, 17, 19,  16, 19, 18,
        0, 3, 7,  0, 7, 4
    };
    ZeroMeshData data = {verts, 20, indices, 48};
    return zero_mesh_upload(&data);
}

static ZeroMesh create_hood_mesh(void) {
    ZeroVertex verts[8] = {
        {{-0.75f,  0.00f, -0.65f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f}},
        {{ 0.75f,  0.00f, -0.65f}, {0.0f, 1.0f, 0.0f}, {1.0f, 0.0f}},
        {{ 0.65f, -0.10f,  0.70f}, {0.0f, 1.0f, 0.0f}, {1.0f, 1.0f}},
        {{-0.65f, -0.10f,  0.70f}, {0.0f, 1.0f, 0.0f}, {0.0f, 1.0f}},
        {{-0.75f, -0.05f, -0.65f}, {0.0f, -1.0f, 0.0f}, {0.0f, 0.0f}},
        {{ 0.75f, -0.05f, -0.65f}, {0.0f, -1.0f, 0.0f}, {1.0f, 0.0f}},
        {{ 0.65f, -0.15f,  0.70f}, {0.0f, -1.0f, 0.0f}, {1.0f, 1.0f}},
        {{-0.65f, -0.15f,  0.70f}, {0.0f, -1.0f, 0.0f}, {0.0f, 1.0f}}
    };
    uint32_t indices[36] = {
        0, 1, 2,  0, 2, 3,
        4, 6, 5,  4, 7, 6,
        0, 4, 5,  0, 5, 1,
        1, 5, 6,  1, 6, 2,
        2, 6, 7,  2, 7, 3,
        3, 7, 4,  3, 4, 0
    };
    ZeroMeshData data = {verts, 8, indices, 36};
    return zero_mesh_upload(&data);
}

static ZeroMesh create_unit_sphere(void) {
    const int stacks = 16;
    const int slices = 32;
    int v_count = (stacks + 1) * (slices + 1);
    int i_count = stacks * slices * 6;

    ZeroVertex *verts = malloc(sizeof(ZeroVertex) * v_count);
    uint32_t *indices = malloc(sizeof(uint32_t) * i_count);

    int v_idx = 0;
    for (int i = 0; i <= stacks; ++i) {
        float phi = ZERO_PI * (float)i / (float)stacks;
        for (int j = 0; j <= slices; ++j) {
            float theta = 2.0f * ZERO_PI * (float)j / (float)slices;
            float x = sinf(phi) * cosf(theta);
            float y = cosf(phi);
            float z = sinf(phi) * sinf(theta);

            verts[v_idx].pos = vec3(x * 600.0f, y * 600.0f, z * 600.0f);
            verts[v_idx].normal = vec3(x, y, z);
            verts[v_idx].uv = vec2((float)j / (float)slices, (float)i / (float)stacks);
            v_idx++;
        }
    }

    int i_idx = 0;
    for (int i = 0; i < stacks; ++i) {
        for (int j = 0; j < slices; ++j) {
            int first = i * (slices + 1) + j;
            int second = first + slices + 1;
            indices[i_idx++] = first;
            indices[i_idx++] = second;
            indices[i_idx++] = first + 1;
            indices[i_idx++] = second;
            indices[i_idx++] = second + 1;
            indices[i_idx++] = first + 1;
        }
    }

    ZeroMeshData data = {verts, (uint32_t)v_count, indices, (uint32_t)i_count};
    ZeroMesh mesh = zero_mesh_upload(&data);
    free(verts);
    free(indices);
    return mesh;
}

// Procedural Grandstands with Audience Spectators & Roof Canopy
static ZeroMesh create_grandstands_mesh(void) {
    const int num_sections = 8;
    const float section_len = 15.0f;
    const int tiers = 6;
    const float tier_height = 0.65f;
    const float tier_depth = 1.10f;
    const float start_x = 108.0f;
    const float start_z = -20.0f;

    int max_verts = num_sections * (tiers * 16 + 24);
    int max_indices = num_sections * (tiers * 24 + 36);
    ZeroVertex *verts = malloc(sizeof(ZeroVertex) * max_verts);
    uint32_t *indices = malloc(sizeof(uint32_t) * max_indices);

    int v_count = 0;
    int i_count = 0;

    for (int s = 0; s < num_sections; ++s) {
        float z0 = start_z + (float)s * section_len;
        float z1 = z0 + section_len - 0.5f;

        for (int t = 0; t < tiers; ++t) {
            float x0 = start_x + (float)t * tier_depth;
            float x1 = x0 + tier_depth;
            float y0 = (float)t * tier_height;
            float y1 = y0 + tier_height;

            // Riser face
            int b0 = v_count;
            verts[v_count++] = (ZeroVertex){vec3(x0, y0, z0), vec3(-1.0f, 0.0f, 0.0f), vec2(0.0f, 0.0f)};
            verts[v_count++] = (ZeroVertex){vec3(x0, y0, z1), vec3(-1.0f, 0.0f, 0.0f), vec2(1.0f, 0.0f)};
            verts[v_count++] = (ZeroVertex){vec3(x0, y1, z1), vec3(-1.0f, 0.0f, 0.0f), vec2(1.0f, 1.0f)};
            verts[v_count++] = (ZeroVertex){vec3(x0, y1, z0), vec3(-1.0f, 0.0f, 0.0f), vec2(0.0f, 1.0f)};
            indices[i_count++] = b0; indices[i_count++] = b0 + 1; indices[i_count++] = b0 + 2;
            indices[i_count++] = b0; indices[i_count++] = b0 + 2; indices[i_count++] = b0 + 3;

            // Tread face (Cheering audience spectator crowd)
            int b1 = v_count;
            verts[v_count++] = (ZeroVertex){vec3(x0, y1, z0), vec3(0.0f, 1.0f, 0.0f), vec2((float)s * 0.125f, (float)t * 0.16f)};
            verts[v_count++] = (ZeroVertex){vec3(x1, y1, z0), vec3(0.0f, 1.0f, 0.0f), vec2((float)(s+1) * 0.125f, (float)t * 0.16f)};
            verts[v_count++] = (ZeroVertex){vec3(x1, y1, z1), vec3(0.0f, 1.0f, 0.0f), vec2((float)(s+1) * 0.125f, (float)(t+1) * 0.16f)};
            verts[v_count++] = (ZeroVertex){vec3(x0, y1, z1), vec3(0.0f, 1.0f, 0.0f), vec2((float)s * 0.125f, (float)(t+1) * 0.16f)};
            indices[i_count++] = b1; indices[i_count++] = b1 + 1; indices[i_count++] = b1 + 2;
            indices[i_count++] = b1; indices[i_count++] = b1 + 2; indices[i_count++] = b1 + 3;
        }

        // Roof Canopy
        float rx0 = start_x - 1.5f;
        float rx1 = start_x + (float)tiers * tier_depth + 2.5f;
        float ry = (float)tiers * tier_height + 4.5f;
        int b_roof = v_count;
        verts[v_count++] = (ZeroVertex){vec3(rx0, ry + 0.8f, z0), vec3(0.2f, 1.0f, 0.0f), vec2(0.0f, 0.0f)};
        verts[v_count++] = (ZeroVertex){vec3(rx1, ry, z0), vec3(0.2f, 1.0f, 0.0f), vec2(1.0f, 0.0f)};
        verts[v_count++] = (ZeroVertex){vec3(rx1, ry, z1), vec3(0.2f, 1.0f, 0.0f), vec2(1.0f, 1.0f)};
        verts[v_count++] = (ZeroVertex){vec3(rx0, ry + 0.8f, z1), vec3(0.2f, 1.0f, 0.0f), vec2(0.0f, 1.0f)};
        indices[i_count++] = b_roof; indices[i_count++] = b_roof + 1; indices[i_count++] = b_roof + 2;
        indices[i_count++] = b_roof; indices[i_count++] = b_roof + 2; indices[i_count++] = b_roof + 3;
    }

    ZeroMeshData data = {verts, (uint32_t)v_count, indices, (uint32_t)i_count};
    ZeroMesh mesh = zero_mesh_upload(&data);
    free(verts);
    free(indices);
    return mesh;
}

// Procedural Trackside Trees (Alpine forest perimeter)
static ZeroMesh create_forest_mesh(void) {
    const int num_trees = 40;
    int max_verts = num_trees * 24;
    int max_indices = num_trees * 36;
    ZeroVertex *verts = malloc(sizeof(ZeroVertex) * max_verts);
    uint32_t *indices = malloc(sizeof(uint32_t) * max_indices);

    int v_count = 0;
    int i_count = 0;

    for (int i = 0; i < num_trees; ++i) {
        float angle = (float)i / (float)num_trees * 2.0f * ZERO_PI;
        int spline_idx = (i * 3) % ZERO_TRACK_SPLINE_COUNT;
        float tx = g_track_spline[spline_idx][0];
        float tz = g_track_spline[spline_idx][1];

        float offset = 18.0f + (float)(i % 5) * 4.0f;
        float px = tx + cosf(angle) * offset;
        float pz = tz + sinf(angle) * offset;
        float h = 7.0f + (float)(i % 4) * 2.5f;
        float r = 2.4f + (float)(i % 3) * 0.6f;

        int b0 = v_count;
        verts[v_count++] = (ZeroVertex){vec3(px - 0.3f, 0.0f, pz), vec3(0.0f, 0.0f, 1.0f), vec2(0.0f, 0.0f)};
        verts[v_count++] = (ZeroVertex){vec3(px + 0.3f, 0.0f, pz), vec3(0.0f, 0.0f, 1.0f), vec2(1.0f, 0.0f)};
        verts[v_count++] = (ZeroVertex){vec3(px + 0.3f, h * 0.4f, pz), vec3(0.0f, 0.0f, 1.0f), vec2(1.0f, 1.0f)};
        verts[v_count++] = (ZeroVertex){vec3(px - 0.3f, h * 0.4f, pz), vec3(0.0f, 0.0f, 1.0f), vec2(0.0f, 1.0f)};
        indices[i_count++] = b0; indices[i_count++] = b0 + 1; indices[i_count++] = b0 + 2;
        indices[i_count++] = b0; indices[i_count++] = b0 + 2; indices[i_count++] = b0 + 3;

        int b1 = v_count;
        verts[v_count++] = (ZeroVertex){vec3(px - r, h * 0.25f, pz), vec3(0.0f, 0.7f, 0.7f), vec2(0.0f, 0.0f)};
        verts[v_count++] = (ZeroVertex){vec3(px + r, h * 0.25f, pz), vec3(0.0f, 0.7f, 0.7f), vec2(1.0f, 0.0f)};
        verts[v_count++] = (ZeroVertex){vec3(px, h * 0.70f, pz), vec3(0.0f, 0.7f, 0.7f), vec2(0.5f, 1.0f)};
        indices[i_count++] = b1; indices[i_count++] = b1 + 1; indices[i_count++] = b1 + 2;

        int b2 = v_count;
        verts[v_count++] = (ZeroVertex){vec3(px - r * 0.75f, h * 0.50f, pz), vec3(0.0f, 0.7f, 0.7f), vec2(0.0f, 0.0f)};
        verts[v_count++] = (ZeroVertex){vec3(px + r * 0.75f, h * 0.50f, pz), vec3(0.0f, 0.7f, 0.7f), vec2(1.0f, 0.0f)};
        verts[v_count++] = (ZeroVertex){vec3(px, h, pz), vec3(0.0f, 0.7f, 0.7f), vec2(0.5f, 1.0f)};
        indices[i_count++] = b2; indices[i_count++] = b2 + 1; indices[i_count++] = b2 + 2;
    }

    ZeroMeshData data = {verts, (uint32_t)v_count, indices, (uint32_t)i_count};
    ZeroMesh mesh = zero_mesh_upload(&data);
    free(verts);
    free(indices);
    return mesh;
}

int main(void) {
    ZeroPlatformConfig cfg = {
        .title = "Zero Drift: Apex",
        .width = 1280,
        .height = 720,
        .headless = false
    };

    ZeroPlatformState platform;
    if (!zero_platform_init(&platform, &cfg)) {
        fprintf(stderr, "Failed to initialize platform\n");
        return 1;
    }

    if (platform.headless) {
        printf("[Zero Drift: Apex] Headless environment verified.\n");
        zero_platform_shutdown(&platform);
        return 0;
    }

    ZeroArena arena = zero_arena_create(96 * 1024 * 1024);

    // 1. Shaders & Meshes
    ZeroShader shader = zero_shader_create(get_race_vert_shader(), get_race_frag_shader());
    ZeroMesh sky_sphere = create_unit_sphere();
    ZeroMesh shadow_quad = create_shadow_quad();
    ZeroMesh grandstands_mesh = create_grandstands_mesh();
    ZeroMesh forest_mesh = create_forest_mesh();

    // 2. Load 3D World Geometry
    ZeroMeshData car_data = zero_obj_parse_file("assets/models/car_real.obj", &arena);
    ZeroMeshData wheel_data = zero_obj_parse_file("assets/models/wheel_real.obj", &arena);
    if (wheel_data.vertex_count == 0) {
        wheel_data = zero_obj_parse_file("assets/models/wheel.obj", &arena);
    }
    ZeroMeshData road_data = zero_obj_parse_file("assets/models/track_road.obj", &arena);
    ZeroMeshData curb_data = zero_obj_parse_file("assets/models/track_curbs.obj", &arena);
    ZeroMeshData barrier_data = zero_obj_parse_file("assets/models/track_barriers.obj", &arena);
    ZeroMeshData terrain_data = zero_obj_parse_file("assets/models/track_terrain.obj", &arena);
    ZeroMeshData gantry_data = zero_obj_parse_file("assets/models/track_gantry.obj", &arena);

    ZeroMesh car_mesh = (car_data.vertex_count > 0) ? zero_mesh_upload(&car_data) : create_shadow_quad();
    ZeroMesh wheel_mesh = (wheel_data.vertex_count > 0) ? zero_mesh_upload(&wheel_data) : create_shadow_quad();
    ZeroMesh road_mesh = (road_data.vertex_count > 0) ? zero_mesh_upload(&road_data) : create_shadow_quad();
    ZeroMesh curb_mesh = (curb_data.vertex_count > 0) ? zero_mesh_upload(&curb_data) : create_shadow_quad();
    ZeroMesh barrier_mesh = (barrier_data.vertex_count > 0) ? zero_mesh_upload(&barrier_data) : create_shadow_quad();
    ZeroMesh terrain_mesh = (terrain_data.vertex_count > 0) ? zero_mesh_upload(&terrain_data) : create_shadow_quad();
    ZeroMesh gantry_mesh = (gantry_data.vertex_count > 0) ? zero_mesh_upload(&gantry_data) : create_shadow_quad();

    ZeroMesh rupture_quad = create_rupture_quad();
    ZeroMesh shard_mesh = create_shard_mesh();
    ZeroMesh front_splitter_mesh = create_front_splitter_mesh();
    ZeroMesh rear_wing_mesh = create_rear_wing_mesh();
    ZeroMesh hood_mesh = create_hood_mesh();

    ZeroTrackDebris debris_pool[MAX_TRACK_DEBRIS];
    memset(debris_pool, 0, sizeof(debris_pool));

    ZeroTrackRupture ruptures[MAX_TRACK_RUPTURES];
    memset(ruptures, 0, sizeof(ruptures));

    ZeroCarDamage car_damage[ZERO_RACE_CAR_COUNT];
    memset(car_damage, 0, sizeof(car_damage));

    float camera_shake_trauma = 0.0f;

    // 3. Subsystems: Particles, Audio, Camera, UI, Simulation
    ZeroParticleSystem ps;
    particle_system_init(&ps);

    ZeroAudioEngine audio;
    zero_audio_init(&audio, false);

    ZeroGameUI gui;
    zero_game_ui_init(&gui);

    ZeroRaceCamera rcam = zero_race_camera_create((float)platform.width / (float)platform.height);
    ZeroRaceGame game = zero_race_game_create();

    const Vec3 car_palette[4] = {
        {0.95f, 0.08f, 0.10f}, // Apex Red
        {0.10f, 0.20f, 0.55f}, // Midnight Blue
        {0.98f, 0.75f, 0.06f}, // Amber Gold
        {0.05f, 0.88f, 0.82f}  // Neon Teal
    };

    float wheel_spin_angles[4][4] = {{0}};
    double last_time = zero_platform_get_time();
    bool c_key_prev = false;
    bool p_key_prev = false;

    // Main Game Loop
    while (!platform.should_close && !zero_game_ui_is_quit_requested(&gui)) {
        zero_platform_poll_events(&platform);

        double cur_time = zero_platform_get_time();
        float dt = (float)(cur_time - last_time);
        last_time = cur_time;
        if (dt > 0.05f) dt = 0.05f;
        if (dt <= 0.0f) dt = 1.0f / 60.0f;

        // Camera Cycle (Key C)
        if (platform.keys['c'] || platform.keys['C']) {
            if (!c_key_prev) {
                zero_race_camera_cycle_mode(&rcam);
                gui.active_camera_mode = (int)rcam.mode;
            }
            c_key_prev = true;
        } else {
            c_key_prev = false;
        }

        // Pause (ESC or P)
        if (platform.keys[27] || platform.keys['p'] || platform.keys['P']) {
            if (!p_key_prev) {
                if (gui.state == ZERO_STATE_RACING) {
                    gui.state = ZERO_STATE_PAUSED;
                } else if (gui.state == ZERO_STATE_PAUSED) {
                    gui.state = ZERO_STATE_RACING;
                }
            }
            p_key_prev = true;
        } else {
            p_key_prev = false;
        }

        // Quick Track Reset / Respawn (Key R)
        if (platform.keys['r'] || platform.keys['R']) {
            platform.keys['r'] = false;
            platform.keys['R'] = false;
            ZeroVehicleEnhanced *pcar = &game.cars[0];
            int seg = 0;
            float seg_t = 0.0f;
            zero_track_find_nearest_segment(pcar->position, &seg, &seg_t, NULL);
            int next_seg = (seg + 1) % ZERO_TRACK_SPLINE_COUNT;
            float cx = g_track_spline[seg][0] + seg_t * (g_track_spline[next_seg][0] - g_track_spline[seg][0]);
            float cz = g_track_spline[seg][1] + seg_t * (g_track_spline[next_seg][1] - g_track_spline[seg][1]);
            float tx = g_track_spline[next_seg][0] - g_track_spline[seg][0];
            float tz = g_track_spline[next_seg][1] - g_track_spline[seg][1];
            float yaw = atan2f(tx, tz);

            pcar->position = vec3(cx, 0.90f, cz);
            pcar->velocity = vec3(0.0f, 0.0f, 0.0f);
            pcar->angular_velocity = vec3(0.0f, 0.0f, 0.0f);
            pcar->orientation = quat_from_axis_angle(vec3(0.0f, 1.0f, 0.0f), yaw);
            pcar->speed_kmh = 0.0f;
            pcar->transmission.current_gear = ZERO_GEAR_1;
            pcar->steer_input = 0.0f;
            pcar->throttle_input = 0.0f;
            pcar->brake_input = 0.0f;
            camera_shake_trauma = 0.0f;
        }

        if (zero_game_ui_is_restart_requested(&gui)) {
            zero_game_ui_clear_restart_request(&gui);
            game = zero_race_game_create();
            memset(car_damage, 0, sizeof(car_damage));
            memset(debris_pool, 0, sizeof(debris_pool));
            memset(ruptures, 0, sizeof(ruptures));
            camera_shake_trauma = 0.0f;
            gui.countdown_timer = 3.0f;
            gui.prev_countdown_timer = 3.0f;
            gui.state = ZERO_STATE_COUNTDOWN;
        }

        ZeroAudioParams audio_params;
        memset(&audio_params, 0, sizeof(audio_params));

        // Responsive Driving Controls with Immediate Punchy Reverse
        if (gui.state == ZERO_STATE_RACING) {
            ZeroVehicleEnhanced *pcar = &game.cars[0];

            Mat4 ch_rot = quat_to_mat4(pcar->orientation);
            Vec3 car_fwd = vec3(ch_rot.m[8], ch_rot.m[9], ch_rot.m[10]);
            Vec3 car_right = vec3(ch_rot.m[0], ch_rot.m[1], ch_rot.m[2]);
            float v_fwd = vec3_dot(pcar->velocity, car_fwd);

            // Progressive Steering
            float target_steer = 0.0f;
            if (platform.keys['a'] || platform.keys['A'] || platform.keys[257]) target_steer += 1.0f;
            if (platform.keys['d'] || platform.keys['D'] || platform.keys[259]) target_steer -= 1.0f;
            pcar->steer_input += (target_steer - pcar->steer_input) * fminf(1.0f, dt * 12.0f);

            bool key_accel = platform.keys['w'] || platform.keys['W'] || platform.keys[258];
            bool key_decel = platform.keys['s'] || platform.keys['S'] || platform.keys[260];
            pcar->handbrake = platform.keys[' '];

            if (key_accel) {
                if (v_fwd < -1.0f) {
                    // Moving in reverse: W brakes forcefully to stop
                    pcar->brake_input = 1.0f;
                    pcar->throttle_input = 0.0f;
                } else {
                    // Forward Drive in 1st-6th gear
                    if (pcar->transmission.current_gear <= ZERO_GEAR_NEUTRAL) {
                        pcar->transmission.current_gear = ZERO_GEAR_1;
                    }
                    pcar->throttle_input = 1.0f;
                    pcar->brake_input = 0.0f;

                    // Dual Exhaust Flame Particles during hard acceleration
                    if (pcar->engine_rpm > 5500.0f) {
                        Vec3 exh_l = vec3_sub(pcar->position, vec3_add(vec3_scale(car_fwd, 2.2f), vec3_scale(car_right, 0.45f)));
                        Vec3 exh_r = vec3_add(vec3_sub(pcar->position, vec3_scale(car_fwd, 2.2f)), vec3_scale(car_right, 0.45f));
                        exh_l.y += 0.32f; exh_r.y += 0.32f;
                        Vec3 f_vel = vec3_add(vec3_scale(car_fwd, -4.0f), vec3(0.0f, 0.5f, 0.0f));
                        particle_spawn(&ps, exh_l, f_vel, vec4(0.2f, 0.6f, 1.0f, 0.9f), 0.35f, 0.15f);
                        particle_spawn(&ps, exh_r, f_vel, vec4(1.0f, 0.4f, 0.1f, 0.9f), 0.35f, 0.15f);
                    }
                }
            } else if (key_decel) {
                if (v_fwd > 2.0f) {
                    // Moving forward: S applies high-performance disc brakes
                    pcar->brake_input = 1.0f;
                    pcar->throttle_input = 0.0f;
                } else {
                    // Stopped or reversing: S acts as immediate punchy reverse drive!
                    pcar->transmission.current_gear = ZERO_GEAR_REVERSE;
                    pcar->throttle_input = 1.0f;
                    pcar->brake_input = 0.0f;
                    // Provide instant reverse impulse so car immediately backs away from walls
                    pcar->velocity = vec3_sub(pcar->velocity, vec3_scale(car_fwd, 6.0f * dt));
                }
            } else {
                pcar->throttle_input = 0.0f;
                pcar->brake_input = 0.0f;
            }

            // Tire Drift Smoke Particles
            if (pcar->is_drifting || (pcar->handbrake && pcar->speed_kmh > 15.0f)) {
                for (int w = 2; w < 4; ++w) {
                    Vec3 tire_pos = pcar->position;
                    tire_pos.y += 0.15f;
                    Vec3 side_offset = (w == 2) ? vec3_scale(car_right, -0.85f) : vec3_scale(car_right, 0.85f);
                    tire_pos = vec3_add(vec3_sub(tire_pos, vec3_scale(car_fwd, 1.3f)), side_offset);
                    Vec3 smk_vel = vec3(
                        ((float)rand() / (float)RAND_MAX - 0.5f) * 1.5f,
                        0.8f + (float)rand() / (float)RAND_MAX * 1.2f,
                        ((float)rand() / (float)RAND_MAX - 0.5f) * 1.5f
                    );
                    particle_spawn(&ps, tire_pos, smk_vel, vec4(0.88f, 0.88f, 0.90f, 0.65f), 0.70f, 0.65f);
                }
            }

            // Advance simulation step (XPBD chassis, tire friction, barrier deflection, AI pursuit, car collisions)
            zero_race_game_step(&game, dt);

            // Dynamic Collision Events: Flying Parts, Track Ruptures, Incandescent Sparks, Screech Sound & Trauma
            for (int e = 0; e < game.collision_event_count; ++e) {
                ZeroCollisionEvent *ev = &game.collision_events[e];
                Vec3 cp = ev->contact_point;
                float v_impact = ev->relative_speed;

                // 1. Audio Crash SFX Trigger
                audio_params.trigger_crash = true;
                audio_params.crash_intensity = fminf(2.0f, 0.6f + v_impact / 10.0f);

                // 2. Camera Screen Shake Trauma
                if (ev->carA_index == 0 || ev->carB_index == 0) {
                    float trauma_add = 0.45f + fminf(0.55f, v_impact / 14.0f);
                    camera_shake_trauma = fminf(1.0f, camera_shake_trauma + trauma_add);
                }

                // 3. Track Rupture Cratering on Asphalt
                if (ev->is_severe) {
                    track_rupture_spawn(ruptures, cp, 1.5f + fminf(2.0f, v_impact * 0.08f));
                }

                // 4. Incandescent Spark Shower (35 to 80 sparks)
                int spark_count = (int)(35.0f + fminf(50.0f, v_impact * 2.8f));
                for (int sp = 0; sp < spark_count; ++sp) {
                    Vec3 spk_v = vec3(
                        ((float)rand() / (float)RAND_MAX - 0.5f) * (8.0f + v_impact * 0.45f),
                        2.5f + ((float)rand() / (float)RAND_MAX) * (5.0f + v_impact * 0.35f),
                        ((float)rand() / (float)RAND_MAX - 0.5f) * (8.0f + v_impact * 0.45f)
                    );
                    Vec4 spk_col = ((rand() % 4) == 0) ? vec4(1.0f, 1.0f, 0.95f, 1.0f) : vec4(1.0f, 0.72f, 0.12f, 0.95f);
                    particle_spawn_ext(&ps, cp, spk_v, spk_col, 0.20f + ((float)rand() / (float)RAND_MAX) * 0.15f, 0.35f + ((float)rand() / (float)RAND_MAX) * 0.35f, true, 0.96f);
                }

                // 5. Fiery Explosion Flash & Friction Smoke
                for (int fl = 0; fl < 8; ++fl) {
                    Vec3 fl_v = vec3(
                        ((float)rand() / (float)RAND_MAX - 0.5f) * 4.0f,
                        1.5f + ((float)rand() / (float)RAND_MAX) * 3.5f,
                        ((float)rand() / (float)RAND_MAX - 0.5f) * 4.0f
                    );
                    particle_spawn_ext(&ps, cp, fl_v, vec4(1.0f, 0.45f, 0.08f, 0.90f), 0.75f + fl * 0.12f, 0.22f, false, 0.90f);
                }
                for (int sm = 0; sm < 14; ++sm) {
                    Vec3 sm_v = vec3(
                        ((float)rand() / (float)RAND_MAX - 0.5f) * 3.5f,
                        0.8f + ((float)rand() / (float)RAND_MAX) * 2.2f,
                        ((float)rand() / (float)RAND_MAX - 0.5f) * 3.5f
                    );
                    particle_spawn_ext(&ps, cp, sm_v, vec4(0.85f, 0.85f, 0.88f, 0.65f), 0.95f, 0.85f, false, 0.92f);
                }

                // 6. Flying Shards (Carbon Fiber & Metal Debris)
                int shard_count = 5 + (int)(v_impact * 0.5f);
                if (shard_count > 16) shard_count = 16;
                for (int sh = 0; sh < shard_count; ++sh) {
                    Vec3 sh_v = vec3(
                        ((float)rand() / (float)RAND_MAX - 0.5f) * (10.0f + v_impact * 0.5f),
                        3.5f + ((float)rand() / (float)RAND_MAX) * (6.0f + v_impact * 0.4f),
                        ((float)rand() / (float)RAND_MAX - 0.5f) * (10.0f + v_impact * 0.5f)
                    );
                    Vec3 sh_ang = vec3(
                        ((float)rand() / (float)RAND_MAX - 0.5f) * 30.0f,
                        ((float)rand() / (float)RAND_MAX - 0.5f) * 30.0f,
                        ((float)rand() / (float)RAND_MAX - 0.5f) * 30.0f
                    );
                    debris_spawn(debris_pool, ZERO_DEBRIS_SHARD, cp, sh_v, sh_ang, vec3(0.12f, 0.12f, 0.14f), 0.25f, 20.0f);
                }

                // 7. Dynamic Part Detachments (Front Splitter, Rear GT Wing, Hood Bonnet, Wheel)
                int car_indices[2] = { ev->carA_index, ev->carB_index };
                for (int c_idx = 0; c_idx < 2; ++c_idx) {
                    int c = car_indices[c_idx];
                    if (c < 0 || c >= ZERO_RACE_CAR_COUNT) continue;
                    ZeroVehicleEnhanced *car = &game.cars[c];
                    Mat4 car_rot = quat_to_mat4(car->orientation);
                    Vec3 c_fwd = vec3(car_rot.m[8], car_rot.m[9], car_rot.m[10]);
                    Vec3 to_cp = vec3_sub(cp, car->position);
                    float dot_fwd = vec3_dot(to_cp, c_fwd);

                    int col_id = (c == 0) ? gui.selected_car_idx : c;
                    if (col_id < 0 || col_id >= 4) col_id = c % 4;
                    Vec3 c_color = car_palette[col_id];

                    // Front hit -> detach front splitter
                    if (dot_fwd > 0.5f && !car_damage[c].lost_front_bumper) {
                        car_damage[c].lost_front_bumper = true;
                        Vec3 d_pos = vec3_add(car->position, vec3_scale(c_fwd, 2.2f));
                        d_pos.y += 0.35f;
                        Vec3 d_vel = vec3_add(car->velocity, vec3_scale(c_fwd, 6.0f + ((float)rand() / (float)RAND_MAX) * 4.0f));
                        d_vel.y += 4.5f + ((float)rand() / (float)RAND_MAX) * 3.0f;
                        Vec3 d_ang = vec3(
                            ((float)rand() / (float)RAND_MAX - 0.5f) * 16.0f,
                            ((float)rand() / (float)RAND_MAX - 0.5f) * 18.0f,
                            ((float)rand() / (float)RAND_MAX - 0.5f) * 14.0f
                        );
                        debris_spawn(debris_pool, ZERO_DEBRIS_FRONT_SPLITTER, d_pos, d_vel, d_ang, c_color, 0.95f, 45.0f);
                    }

                    // Rear hit -> rip off rear GT wing
                    if (dot_fwd < -0.5f && !car_damage[c].lost_rear_wing) {
                        car_damage[c].lost_rear_wing = true;
                        Vec3 d_pos = vec3_add(car->position, vec3_scale(c_fwd, -2.1f));
                        d_pos.y += 0.95f;
                        Vec3 d_vel = vec3_add(car->velocity, vec3_scale(c_fwd, -5.0f));
                        d_vel.y += 6.5f + ((float)rand() / (float)RAND_MAX) * 4.0f;
                        Vec3 d_ang = vec3(
                            ((float)rand() / (float)RAND_MAX - 0.5f) * 20.0f,
                            ((float)rand() / (float)RAND_MAX - 0.5f) * 20.0f,
                            ((float)rand() / (float)RAND_MAX - 0.5f) * 18.0f
                        );
                        debris_spawn(debris_pool, ZERO_DEBRIS_REAR_WING, d_pos, d_vel, d_ang, c_color, 0.92f, 45.0f);
                    }

                    // High-energy impact -> eject hood panel
                    if (ev->is_severe && v_impact > 10.0f && !car_damage[c].lost_hood) {
                        car_damage[c].lost_hood = true;
                        Vec3 d_pos = vec3_add(car->position, vec3_scale(c_fwd, 0.8f));
                        d_pos.y += 0.65f;
                        Vec3 d_vel = vec3_add(car->velocity, vec3(0.0f, 7.5f + ((float)rand() / (float)RAND_MAX) * 3.5f, 0.0f));
                        Vec3 d_ang = vec3(
                            ((float)rand() / (float)RAND_MAX - 0.5f) * 15.0f,
                            ((float)rand() / (float)RAND_MAX - 0.5f) * 25.0f,
                            ((float)rand() / (float)RAND_MAX - 0.5f) * 15.0f
                        );
                        debris_spawn(debris_pool, ZERO_DEBRIS_HOOD, d_pos, d_vel, d_ang, c_color, 0.85f, 45.0f);
                    }

                    // Extreme hyper impact -> wheel detachment
                    if (v_impact > 18.0f && (rand() % 3 == 0)) {
                        Vec3 d_pos = car->position;
                        d_pos.y += 0.35f;
                        Vec3 d_vel = vec3_add(car->velocity, vec3(
                            ((float)rand() / (float)RAND_MAX - 0.5f) * 8.0f,
                            4.5f + ((float)rand() / (float)RAND_MAX) * 3.0f,
                            ((float)rand() / (float)RAND_MAX - 0.5f) * 8.0f
                        ));
                        Vec3 d_ang = vec3(35.0f, 0.0f, 0.0f);
                        debris_spawn(debris_pool, ZERO_DEBRIS_WHEEL, d_pos, d_vel, d_ang, vec3(0.2f, 0.2f, 0.2f), 0.35f, 45.0f);
                    }
                }
            }

            // Barrier Collision Spark Particles & High-Speed Wall Smashes
            int p_node = 0; float p_t = 0.0f; float p_dist_sq = 0.0f;
            zero_track_find_nearest_segment(pcar->position, &p_node, &p_t, &p_dist_sq);
            float p_dist = sqrtf(p_dist_sq);
            if (p_dist > (ZERO_TRACK_BARRIER_DIST - 1.45f) && pcar->speed_kmh > 20.0f) {
                float b_spd = pcar->speed_kmh / 3.6f;
                int b_sparks = (int)(12.0f + b_spd * 1.5f);
                if (b_sparks > 45) b_sparks = 45;
                for (int sp = 0; sp < b_sparks; ++sp) {
                    Vec3 spk_pos = pcar->position;
                    spk_pos.y += 0.25f + ((float)rand() / (float)RAND_MAX) * 0.45f;
                    Vec3 spk_vel = vec3(
                        ((float)rand() / (float)RAND_MAX - 0.5f) * 8.0f,
                        2.0f + (float)rand() / (float)RAND_MAX * 4.5f,
                        ((float)rand() / (float)RAND_MAX - 0.5f) * 8.0f
                    );
                    particle_spawn_ext(&ps, spk_pos, spk_vel, vec4(1.0f, 0.80f, 0.15f, 0.95f), 0.22f, 0.38f, true, 0.96f);
                }
                if (b_spd > 12.0f) {
                    audio_params.trigger_crash = true;
                    audio_params.crash_intensity = fminf(1.8f, b_spd / 14.0f);
                    camera_shake_trauma = fminf(1.0f, camera_shake_trauma + 0.35f);
                    track_rupture_spawn(ruptures, pcar->position, 1.4f);
                }
            }

            // Debris and Track Rupture Simulation Update
            debris_system_update(debris_pool, &ps, &game, dt);
            track_rupture_update(ruptures, dt);

            // Check race completion
            if (game.race_finished) {
                for (int k = 0; k < ZERO_RACE_CAR_COUNT; ++k) {
                    int pos = game.cars[k].race_position;
                    const char *drv = (k == 0) ? "Player" :
                                      (k == 1) ? "Apex AI 1" :
                                      (k == 2) ? "Apex AI 2" : "Apex AI 3";
                    const char *car_name = zero_game_ui_get_car(k)->name;
                    zero_game_ui_set_podium_entry(&gui, pos - 1, drv, car_name,
                                                  game.race_time, game.cars[k].best_lap_time, pos);
                }
                gui.state = ZERO_STATE_LEADERBOARD;
            }
        } else if (gui.state == ZERO_STATE_COUNTDOWN) {
            float throttle = 0.0f;
            if (platform.keys['w'] || platform.keys['W'] || platform.keys[258]) throttle = 1.0f;
            game.cars[0].throttle_input = throttle;
            game.cars[0].brake_input = 1.0f;
            if (throttle > 0.0f) {
                game.cars[0].engine_rpm += throttle * 4500.0f * dt;
                if (game.cars[0].engine_rpm > 7500.0f) game.cars[0].engine_rpm = 7500.0f;
            } else {
                game.cars[0].engine_rpm -= 3000.0f * dt;
                if (game.cars[0].engine_rpm < 800.0f) game.cars[0].engine_rpm = 800.0f;
            }
        }

        // Particle System Simulation
        particle_system_update(&ps, dt);

        // Camera Update
        rcam.base_cam.aspect = (float)platform.width / (float)platform.height;
        zero_race_camera_update(&rcam,
                                game.cars[0].position,
                                game.cars[0].orientation,
                                game.cars[0].velocity,
                                game.cars[0].engine_rpm,
                                game.cars[0].steer_input,
                                dt);
        Mat4 view = zero_race_camera_get_view(&rcam);
        if (camera_shake_trauma > 0.001f) {
            float sq_trauma = camera_shake_trauma * camera_shake_trauma;
            float shake_x = (sinf((float)cur_time * 52.0f) * 0.35f + sinf((float)cur_time * 83.0f) * 0.15f) * sq_trauma;
            float shake_y = (cosf((float)cur_time * 61.0f) * 0.28f + cosf((float)cur_time * 94.0f) * 0.12f) * sq_trauma;
            float shake_rot = sinf((float)cur_time * 73.0f) * 0.06f * sq_trauma;
            Mat4 shake_t = mat4_translate(vec3(shake_x, shake_y, 0.0f));
            Mat4 shake_r = quat_to_mat4(quat_from_axis_angle(vec3(0, 0, 1), shake_rot));
            view = mat4_mul(shake_r, mat4_mul(shake_t, view));
            camera_shake_trauma -= dt * 1.6f;
            if (camera_shake_trauma < 0.0f) camera_shake_trauma = 0.0f;
        }
        Mat4 proj = zero_race_camera_get_projection(&rcam);

        // Audio Update
        audio_params.engine_rpm = game.cars[0].engine_rpm;
        audio_params.throttle = game.cars[0].throttle_input;
        audio_params.vehicle_speed_kmh = game.cars[0].speed_kmh;
        audio_params.tire_slip_ratio = (game.cars[0].drift_angle > 8.0f) ? (game.cars[0].drift_angle / 30.0f) : 0.0f;
        if (audio_params.tire_slip_ratio > 1.0f) audio_params.tire_slip_ratio = 1.0f;
        audio_params.trigger_menu_beep = gui.menu_beep_triggered;
        audio_params.trigger_countdown_red = gui.countdown_red_triggered;
        audio_params.trigger_countdown_green = gui.countdown_green_triggered;
        audio.master_volume = gui.audio_settings.master_volume;
        audio.is_muted = gui.audio_settings.is_muted;
        zero_audio_update_params(&audio, &audio_params);

        // 3D Rendering Pass
        glViewport(0, 0, (GLsizei)platform.width, (GLsizei)platform.height);
        glClearColor(0.08f, 0.12f, 0.20f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        zero_shader_bind(&shader);
        zero_shader_set_mat4(&shader, "u_view", view);
        zero_shader_set_mat4(&shader, "u_proj", proj);
        zero_shader_set_vec3(&shader, "u_camera_pos", rcam.current_pos);
        zero_shader_set_vec3(&shader, "u_light_dir", vec3(-0.45f, -0.75f, -0.45f));
        zero_shader_set_vec3(&shader, "u_light_color", vec3(1.35f, 1.30f, 1.15f));
        zero_shader_set_vec3(&shader, "u_ambient_color", vec3(0.38f, 0.42f, 0.48f));
        zero_shader_set_float(&shader, "u_time", (float)cur_time);

        // 1. Radiant Diurnal Sky Dome
        glDisable(GL_DEPTH_TEST);
        zero_shader_set_float(&shader, "u_is_sky", 1.0f);
        Mat4 sky_model = mat4_translate(rcam.current_pos);
        zero_shader_set_mat4(&shader, "u_model", sky_model);
        zero_mesh_draw(&sky_sphere);
        zero_shader_set_float(&shader, "u_is_sky", 0.0f);

        // 2. Track & Environment Geometry
        glEnable(GL_DEPTH_TEST);
        Mat4 id_mat = mat4_identity();
        zero_shader_set_mat4(&shader, "u_model", id_mat);

        // Vibrant Green Terrain
        zero_shader_set_float(&shader, "u_roughness", 0.88f);
        zero_shader_set_float(&shader, "u_metallic", 0.0f);
        zero_shader_set_vec3(&shader, "u_object_color", vec3(0.24f, 0.56f, 0.22f));
        zero_shader_set_float(&shader, "u_material_mode", 3.0f);
        zero_mesh_draw(&terrain_mesh);

        // Clean Asphalt Road
        zero_shader_set_float(&shader, "u_roughness", 0.70f);
        zero_shader_set_float(&shader, "u_metallic", 0.05f);
        zero_shader_set_vec3(&shader, "u_object_color", vec3(0.18f, 0.18f, 0.20f));
        zero_shader_set_float(&shader, "u_material_mode", 1.0f);
        zero_mesh_draw(&road_mesh);

        // Rumble Curbs
        zero_shader_set_float(&shader, "u_roughness", 0.35f);
        zero_shader_set_float(&shader, "u_metallic", 0.10f);
        zero_shader_set_vec3(&shader, "u_object_color", vec3(0.95f, 0.06f, 0.08f));
        zero_shader_set_float(&shader, "u_material_mode", 2.0f);
        zero_mesh_draw(&curb_mesh);

        // Perimeter Barriers
        zero_shader_set_float(&shader, "u_roughness", 0.30f);
        zero_shader_set_float(&shader, "u_metallic", 0.75f);
        zero_shader_set_vec3(&shader, "u_object_color", vec3(0.20f, 0.48f, 0.95f));
        zero_shader_set_float(&shader, "u_material_mode", 4.0f);
        zero_mesh_draw(&barrier_mesh);

        // Stadium Grandstands with Spectator Crowds
        zero_shader_set_float(&shader, "u_roughness", 0.50f);
        zero_shader_set_float(&shader, "u_metallic", 0.10f);
        zero_shader_set_vec3(&shader, "u_object_color", vec3(0.75f, 0.76f, 0.80f));
        zero_shader_set_float(&shader, "u_material_mode", 7.0f);
        zero_mesh_draw(&grandstands_mesh);

        // Forest Perimeter Trees
        zero_shader_set_float(&shader, "u_roughness", 0.75f);
        zero_shader_set_float(&shader, "u_metallic", 0.0f);
        zero_shader_set_vec3(&shader, "u_object_color", vec3(0.15f, 0.38f, 0.16f));
        zero_shader_set_float(&shader, "u_material_mode", 3.0f);
        zero_mesh_draw(&forest_mesh);

        // Starting Line Gantry & Sponsor Signage
        zero_shader_set_float(&shader, "u_roughness", 0.35f);
        zero_shader_set_float(&shader, "u_metallic", 0.80f);
        zero_shader_set_vec3(&shader, "u_object_color", vec3(0.40f, 0.42f, 0.46f));
        zero_shader_set_float(&shader, "u_material_mode", 8.0f);
        zero_mesh_draw(&gantry_mesh);

        // 3. Vehicles & Wheels (4 High-Poly Cars with Metallic Clearcoat & Sub-materials)
        for (int k = 0; k < ZERO_RACE_CAR_COUNT; ++k) {
            ZeroVehicleEnhanced *car = &game.cars[k];

            // Contact Drop Shadow
            glEnable(GL_BLEND);
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
            zero_shader_set_float(&shader, "u_material_mode", 5.0f);
            Mat4 shadow_t = mat4_translate(vec3(car->position.x, 0.03f, car->position.z));
            Mat4 shadow_r = quat_to_mat4(car->orientation);
            Mat4 shadow_s = mat4_scale(vec3(2.5f, 1.0f, 5.0f));
            Mat4 shadow_m = mat4_mul(shadow_t, mat4_mul(shadow_r, shadow_s));
            zero_shader_set_mat4(&shader, "u_model", shadow_m);
            zero_mesh_draw(&shadow_quad);
            glDisable(GL_BLEND);

            // Car Body with Metallic Paint & Sub-materials
            int color_idx = (k == 0) ? gui.selected_car_idx : k;
            if (color_idx < 0 || color_idx >= 4) color_idx = k % 4;
            Vec3 body_color = car_palette[color_idx];

            zero_shader_set_mat4(&shader, "u_model", car->transform);
            zero_shader_set_float(&shader, "u_roughness", 0.12f);
            zero_shader_set_float(&shader, "u_metallic", 0.92f);
            zero_shader_set_vec3(&shader, "u_object_color", body_color);
            zero_shader_set_float(&shader, "u_material_mode", 0.0f);
            zero_mesh_draw(&car_mesh);

            // Dynamic Aero Components (drawn only if still intact)
            if (!car_damage[k].lost_front_bumper) {
                Mat4 sp_mat = mat4_mul(car->transform, mat4_translate(vec3(0.0f, -0.05f, 2.15f)));
                zero_shader_set_mat4(&shader, "u_model", sp_mat);
                zero_shader_set_float(&shader, "u_material_mode", 11.0f);
                zero_shader_set_vec3(&shader, "u_object_color", body_color);
                zero_mesh_draw(&front_splitter_mesh);
            }
            if (!car_damage[k].lost_rear_wing) {
                Mat4 rw_mat = mat4_mul(car->transform, mat4_translate(vec3(0.0f, 0.85f, -1.95f)));
                zero_shader_set_mat4(&shader, "u_model", rw_mat);
                zero_shader_set_float(&shader, "u_material_mode", 11.0f);
                zero_shader_set_vec3(&shader, "u_object_color", body_color);
                zero_mesh_draw(&rear_wing_mesh);
            }
            if (!car_damage[k].lost_hood) {
                Mat4 hd_mat = mat4_mul(car->transform, mat4_translate(vec3(0.0f, 0.48f, 1.10f)));
                zero_shader_set_mat4(&shader, "u_model", hd_mat);
                zero_shader_set_float(&shader, "u_material_mode", 0.0f);
                zero_shader_set_vec3(&shader, "u_object_color", body_color);
                zero_mesh_draw(&hood_mesh);
            }

            // 4 Wheels with Rubber Tires, Alloy Rims, Red Calipers
            for (int w = 0; w < 4; ++w) {
                ZeroWheel *wh = &car->wheels[w];
                Vec3 wh_local = vec3(wh->hardpoint.x, -wh->current_length, wh->hardpoint.z);
                Mat4 wh_t = mat4_translate(wh_local);
                Mat4 wh_s = mat4_identity();
                if (wh->is_front) {
                    wh_s = quat_to_mat4(quat_from_axis_angle(vec3(0, 1, 0), wh->steer_angle));
                }
                float spin_spd = vec3_length(car->velocity) / wh->radius;
                wheel_spin_angles[k][w] += (car->velocity.z >= 0.0f ? 1.0f : -1.0f) * spin_spd * dt;
                Mat4 wh_spin = quat_to_mat4(quat_from_axis_angle(vec3(1, 0, 0), wheel_spin_angles[k][w]));
                Mat4 wh_mat = mat4_mul(car->transform, mat4_mul(wh_t, mat4_mul(wh_s, wh_spin)));

                zero_shader_set_mat4(&shader, "u_model", wh_mat);
                zero_shader_set_float(&shader, "u_roughness", 0.35f);
                zero_shader_set_float(&shader, "u_metallic", 0.85f);
                zero_shader_set_vec3(&shader, "u_object_color", vec3(0.88f, 0.90f, 0.94f));
                zero_shader_set_float(&shader, "u_material_mode", 6.0f);
                zero_mesh_draw(&wheel_mesh);
            }
        }

        // 4. Render Dynamic Track Debris Littered on Track (Bumpers, Wings, Hoods, Wheels, Shards)
        for (int i = 0; i < MAX_TRACK_DEBRIS; ++i) {
            if (!debris_pool[i].active) continue;
            ZeroTrackDebris *d = &debris_pool[i];
            Mat4 d_t = mat4_translate(d->pos);
            Mat4 d_r = quat_to_mat4(d->rot);
            Mat4 d_mat = mat4_mul(d_t, d_r);
            zero_shader_set_mat4(&shader, "u_model", d_mat);

            if (d->type == ZERO_DEBRIS_FRONT_SPLITTER) {
                zero_shader_set_float(&shader, "u_material_mode", 11.0f);
                zero_shader_set_vec3(&shader, "u_object_color", d->color);
                zero_mesh_draw(&front_splitter_mesh);
            } else if (d->type == ZERO_DEBRIS_REAR_WING) {
                zero_shader_set_float(&shader, "u_material_mode", 11.0f);
                zero_shader_set_vec3(&shader, "u_object_color", d->color);
                zero_mesh_draw(&rear_wing_mesh);
            } else if (d->type == ZERO_DEBRIS_HOOD) {
                zero_shader_set_float(&shader, "u_material_mode", 0.0f);
                zero_shader_set_vec3(&shader, "u_object_color", d->color);
                zero_mesh_draw(&hood_mesh);
            } else if (d->type == ZERO_DEBRIS_WHEEL) {
                zero_shader_set_float(&shader, "u_material_mode", 6.0f);
                zero_shader_set_vec3(&shader, "u_object_color", vec3(0.88f, 0.90f, 0.94f));
                zero_mesh_draw(&wheel_mesh);
            } else if (d->type == ZERO_DEBRIS_SHARD) {
                zero_shader_set_float(&shader, "u_material_mode", 11.0f);
                zero_shader_set_vec3(&shader, "u_object_color", d->color);
                zero_mesh_draw(&shard_mesh);
            }
        }

        // 5. Render Track Rupture Ground Decal Craters
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        zero_shader_set_float(&shader, "u_material_mode", 10.0f);
        for (int i = 0; i < MAX_TRACK_RUPTURES; ++i) {
            if (!ruptures[i].active) continue;
            Mat4 r_t = mat4_translate(ruptures[i].pos);
            Mat4 r_s = mat4_scale(vec3(ruptures[i].radius * 2.0f, 1.0f, ruptures[i].radius * 2.0f));
            Mat4 r_mat = mat4_mul(r_t, r_s);
            zero_shader_set_mat4(&shader, "u_model", r_mat);
            zero_shader_set_float(&shader, "u_roughness", ruptures[i].intensity);
            zero_mesh_draw(&rupture_quad);
        }
        glDisable(GL_BLEND);

        // 4. Render Particle System (Camera-facing billboard quads with blending)
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE); // Additive blending for fiery glow
        glDepthMask(GL_FALSE);
        zero_shader_set_float(&shader, "u_material_mode", 9.0f);

        Mat4 inv_cam_view = mat4_identity();
        inv_cam_view.m[0] = view.m[0]; inv_cam_view.m[1] = view.m[4]; inv_cam_view.m[2] = view.m[8];
        inv_cam_view.m[4] = view.m[1]; inv_cam_view.m[5] = view.m[5]; inv_cam_view.m[6] = view.m[9];
        inv_cam_view.m[8] = view.m[2]; inv_cam_view.m[9] = view.m[6]; inv_cam_view.m[10] = view.m[10];

        for (int i = 0; i < MAX_PARTICLES; ++i) {
            if (!ps.particles[i].active) continue;
            ZeroParticle *p = &ps.particles[i];
            float alpha = (p->life / p->max_life) * p->color.w;
            Mat4 p_trans = mat4_translate(p->pos);
            Mat4 p_scale = mat4_scale(vec3(p->size, p->size, 1.0f));
            Mat4 p_mat = mat4_mul(p_trans, mat4_mul(inv_cam_view, p_scale));

            zero_shader_set_mat4(&shader, "u_model", p_mat);
            zero_shader_set_vec3(&shader, "u_object_color", vec3(p->color.x, p->color.y, p->color.z));
            zero_shader_set_float(&shader, "u_roughness", alpha); // Pass alpha via roughness uniform
            zero_mesh_draw(&ps.quad_mesh);
        }
        glDepthMask(GL_TRUE);
        glDisable(GL_BLEND);

        // 5. UI Telemetry & Full State Machine Overlay Pass
        ZeroGameTelemetry telem;
        memset(&telem, 0, sizeof(telem));
        telem.speed_kmh = game.cars[0].speed_kmh;
        telem.engine_rpm = game.cars[0].engine_rpm;
        telem.gear = (int)game.cars[0].transmission.current_gear;
        telem.is_automatic = true;
        telem.current_lap = game.cars[0].lap_count + 1;
        if (telem.current_lap > game.total_laps) telem.current_lap = game.total_laps;
        telem.total_laps = game.total_laps;
        telem.race_position = game.cars[0].race_position;
        telem.total_cars = ZERO_RACE_CAR_COUNT;
        telem.current_lap_time = game.cars[0].current_lap_time;
        telem.best_lap_time = game.cars[0].best_lap_time;
        telem.is_drifting = game.cars[0].is_drifting;
        telem.drift_angle = game.cars[0].drift_angle;

        int p_seg = 0;
        float p_seg_t = 0.0f;
        zero_track_find_nearest_segment(game.cars[0].position, &p_seg, &p_seg_t, NULL);
        telem.player_spline_t = ((float)p_seg + p_seg_t) / (float)ZERO_TRACK_SPLINE_COUNT;

        for (int j = 1; j < ZERO_RACE_CAR_COUNT; ++j) {
            int ai_seg = 0;
            float ai_seg_t = 0.0f;
            zero_track_find_nearest_segment(game.cars[j].position, &ai_seg, &ai_seg_t, NULL);
            telem.ai_spline_t[j - 1] = ((float)ai_seg + ai_seg_t) / (float)ZERO_TRACK_SPLINE_COUNT;
        }

        zero_game_ui_render(&gui, &telem,
                            (float)platform.width, (float)platform.height,
                            (float)platform.mouse_x, (float)platform.mouse_y,
                            platform.mouse_buttons[1], dt);

        zero_platform_swap_buffers(&platform);
    }

    // Cleanup & Shutdown
    zero_game_ui_shutdown(&gui);
    zero_audio_shutdown(&audio);
    zero_mesh_destroy(&car_mesh);
    zero_mesh_destroy(&wheel_mesh);
    zero_mesh_destroy(&road_mesh);
    zero_mesh_destroy(&curb_mesh);
    zero_mesh_destroy(&barrier_mesh);
    zero_mesh_destroy(&terrain_mesh);
    zero_mesh_destroy(&gantry_mesh);
    zero_mesh_destroy(&grandstands_mesh);
    zero_mesh_destroy(&forest_mesh);
    zero_mesh_destroy(&sky_sphere);
    zero_mesh_destroy(&shadow_quad);
    zero_mesh_destroy(&ps.quad_mesh);
    zero_mesh_destroy(&rupture_quad);
    zero_mesh_destroy(&shard_mesh);
    zero_mesh_destroy(&front_splitter_mesh);
    zero_mesh_destroy(&rear_wing_mesh);
    zero_mesh_destroy(&hood_mesh);
    zero_shader_destroy(&shader);
    zero_arena_destroy(&arena);
    zero_platform_shutdown(&platform);

    return 0;
}

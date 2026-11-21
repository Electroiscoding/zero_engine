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
        "    // 1. Procedural Diurnal Sky Dome Pass (Sun Disk, Solar Flare, Rayleigh Sky, Clouds)\n"
        "    if (u_is_sky > 0.5) {\n"
        "        vec3 ray = normalize(v_sky_ray);\n"
        "        vec3 sun_d = normalize(-u_light_dir);\n"
        "        float sun_dot = max(dot(ray, sun_d), 0.0);\n"
        "\n"
        "        vec3 sky_zenith = vec3(0.12, 0.38, 0.88);\n"
        "        vec3 sky_mid = vec3(0.42, 0.68, 0.95);\n"
        "        vec3 sky_horizon = vec3(0.78, 0.86, 0.96);\n"
        "        vec3 ground_fog = vec3(0.32, 0.40, 0.48);\n"
        "\n"
        "        vec3 sky = (ray.y > 0.0) ? mix(sky_horizon, mix(sky_mid, sky_zenith, pow(ray.y, 0.55)), ray.y)\n"
        "                                 : mix(sky_horizon, ground_fog, clamp(-ray.y * 3.0, 0.0, 1.0));\n"
        "\n"
        "        // High-intensity Sun disk + Golden Corona Bloom\n"
        "        float sun_disk = pow(sun_dot, 850.0) * 22.0;\n"
        "        float sun_halo = pow(sun_dot, 24.0) * 3.5;\n"
        "        vec3 sun = vec3(1.0, 0.94, 0.82) * (sun_disk + sun_halo);\n"
        "\n"
        "        // Procedural cumulus cloud wisps\n"
        "        if (ray.y > 0.02) {\n"
        "            vec2 cloud_coord = ray.xz / (ray.y + 0.12) * 0.16 + vec2(u_time * 0.015, 0.0);\n"
        "            float c_noise = sin(cloud_coord.x * 3.2) * cos(cloud_coord.y * 3.2);\n"
        "            c_noise += sin(cloud_coord.x * 7.4 + cloud_coord.y * 5.2) * 0.5;\n"
        "            float cloud_alpha = clamp((c_noise - 0.15) * 2.0, 0.0, 0.70);\n"
        "            sky = mix(sky, vec3(0.98, 0.98, 1.0), cloud_alpha);\n"
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
        "        frag_color = vec4(0.04, 0.04, 0.05, alpha);\n"
        "        return;\n"
        "    }\n"
        "\n"
        "    // 3. 3D Cook-Torrance Specular PBR + Automotive Clearcoat\n"
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
        "            albedo = vec3(0.04, 0.05, 0.07);\n"
        "            roughness = 0.04;\n"
        "            metallic = 0.10;\n"
        "        } else if (sub_mat > 1.5 && sub_mat < 2.5) {\n"
        "            // Carbon Fiber Aero Splitters & Wing\n"
        "            albedo = vec3(0.06, 0.06, 0.07);\n"
        "            roughness = 0.38;\n"
        "            metallic = 0.25;\n"
        "        } else if (sub_mat > 2.5 && sub_mat < 3.5) {\n"
        "            // Gloss Black Trim & Pillars\n"
        "            albedo = vec3(0.02, 0.02, 0.02);\n"
        "            roughness = 0.12;\n"
        "            metallic = 0.40;\n"
        "        } else if (sub_mat > 3.5 && sub_mat < 4.5) {\n"
        "            // Emissive Taillight Neon Bar\n"
        "            albedo = vec3(0.95, 0.04, 0.04);\n"
        "            emissive = vec3(2.5, 0.1, 0.1);\n"
        "            roughness = 0.20;\n"
        "        } else if (sub_mat > 4.5 && sub_mat < 5.5) {\n"
        "            // Emissive Xenon Headlights\n"
        "            albedo = vec3(0.95, 0.95, 1.0);\n"
        "            emissive = vec3(2.8, 2.8, 3.0);\n"
        "            roughness = 0.10;\n"
        "        } else if (sub_mat > 5.5 && sub_mat < 6.5) {\n"
        "            // Chrome / Metallic Exhaust Tips\n"
        "            albedo = vec3(0.85, 0.88, 0.90);\n"
        "            roughness = 0.15;\n"
        "            metallic = 0.95;\n"
        "        }\n"
        "    }\n"
        "    // Mode 1: Asphalt Road (Centerline dashed striping & edge bounds)\n"
        "    else if (u_material_mode > 0.5 && u_material_mode < 1.5) {\n"
        "        float u_track = v_uv.x;\n"
        "        float v_dist = v_uv.y;\n"
        "        float center_dist = abs(u_track - 0.5);\n"
        "        if (center_dist < 0.012) {\n"
        "            float dash = mod(v_dist * 8.0, 2.0);\n"
        "            if (dash < 1.0) albedo = vec3(0.96, 0.96, 0.96);\n"
        "        }\n"
        "        if (abs(u_track - 0.05) < 0.010 || abs(u_track - 0.95) < 0.010) {\n"
        "            albedo = vec3(0.92, 0.90, 0.88);\n"
        "        }\n"
        "    }\n"
        "    // Mode 2: Rumble Curbs (Alternating red and white curb strips)\n"
        "    else if (u_material_mode > 1.5 && u_material_mode < 2.5) {\n"
        "        float curb_step = mod(v_uv.y * 30.0, 2.0);\n"
        "        albedo = (curb_step < 1.0) ? vec3(0.92, 0.08, 0.10) : vec3(0.96, 0.96, 0.96);\n"
        "    }\n"
        "    // Mode 3: Terrain checkerboard green fields\n"
        "    else if (u_material_mode > 2.5 && u_material_mode < 3.5) {\n"
        "        float cx = floor(v_world_pos.x * 0.10);\n"
        "        float cz = floor(v_world_pos.z * 0.10);\n"
        "        float chk = mod(cx + cz, 2.0);\n"
        "        albedo = (chk < 1.0) ? vec3(0.22, 0.52, 0.20) : vec3(0.18, 0.44, 0.16);\n"
        "    }\n"
        "    // Mode 4: Armco barriers with dual-tone safety stripes\n"
        "    else if (u_material_mode > 3.5 && u_material_mode < 4.5) {\n"
        "        float b_step = mod(v_world_pos.x + v_world_pos.z, 4.0);\n"
        "        albedo = (b_step < 2.0) ? vec3(0.22, 0.45, 0.92) : vec3(0.92, 0.92, 0.95);\n"
        "    }\n"
        "    // Mode 6: Wheels (Rubber Tire, Alloy Rim, Red Calipers)\n"
        "    else if (u_material_mode > 5.5 && u_material_mode < 6.5) {\n"
        "        float w_mat = v_uv.x;\n"
        "        if (w_mat > 0.5 && w_mat < 1.5) {\n"
        "            // Black Rubber Tire\n"
        "            albedo = vec3(0.12, 0.13, 0.14);\n"
        "            roughness = 0.70;\n"
        "            metallic = 0.05;\n"
        "        } else if (w_mat > 1.5 && w_mat < 2.5) {\n"
        "            // Brushed Silver Alloy Rim\n"
        "            albedo = vec3(0.85, 0.88, 0.92);\n"
        "            roughness = 0.18;\n"
        "            metallic = 0.92;\n"
        "        } else if (w_mat > 2.5 && w_mat < 3.5) {\n"
        "            // Sport Red Brake Caliper\n"
        "            albedo = vec3(0.88, 0.08, 0.10);\n"
        "            roughness = 0.25;\n"
        "            metallic = 0.45;\n"
        "        }\n"
        "    }\n"
        "    // Mode 7: Grandstand Audience Spectator Crowd (Multi-colored shirts)\n"
        "    else if (u_material_mode > 6.5 && u_material_mode < 7.5) {\n"
        "        float cid = floor(v_uv.x * 24.0) + floor(v_uv.y * 12.0);\n"
        "        float r_val = fract(sin(cid * 91.345) * 47453.545);\n"
        "        if (r_val < 0.20) albedo = vec3(0.92, 0.12, 0.14);\n"
        "        else if (r_val < 0.40) albedo = vec3(0.15, 0.45, 0.92);\n"
        "        else if (r_val < 0.60) albedo = vec3(0.96, 0.82, 0.10);\n"
        "        else if (r_val < 0.80) albedo = vec3(0.95, 0.95, 0.95);\n"
        "        else albedo = vec3(0.12, 0.85, 0.55);\n"
        "        roughness = 0.60;\n"
        "        metallic = 0.05;\n"
        "    }\n"
        "    // Mode 8: Sponsor Billboards\n"
        "    else if (u_material_mode > 7.5 && u_material_mode < 8.5) {\n"
        "        float b_idx = floor(v_uv.x * 4.0);\n"
        "        if (b_idx < 1.0) albedo = vec3(0.92, 0.08, 0.10); // Red (Brembo/Shell)\n"
        "        else if (b_idx < 2.0) albedo = vec3(0.96, 0.85, 0.08); // Yellow (Pirelli)\n"
        "        else if (b_idx < 3.0) albedo = vec3(0.08, 0.45, 0.95); // Blue (Apex Motors)\n"
        "        else albedo = vec3(0.05, 0.05, 0.06); // Dark Carbon (Zero Drift)\n"
        "        roughness = 0.30;\n"
        "        metallic = 0.15;\n"
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
        "    // Automotive Clearcoat Highlight\n"
        "    float cc_NDF = distributionGGX(N, H, 0.08);\n"
        "    float cc_G = geometrySmith(N, V, L, 0.08);\n"
        "    vec3 cc_F = fresnelSchlick(VdotH, vec3(0.04));\n"
        "    vec3 cc_spec = (cc_NDF * cc_G * cc_F) / (4.0 * NdotV * NdotL + 0.0001) * 0.65;\n"
        "\n"
        "    // Sky environmental reflections\n"
        "    vec3 R = reflect(-V, N);\n"
        "    vec3 sky_zenith = vec3(0.12, 0.38, 0.88);\n"
        "    vec3 sky_horizon = vec3(0.78, 0.86, 0.96);\n"
        "    vec3 env_reflection = mix(sky_horizon, sky_zenith, clamp(R.y * 0.5 + 0.5, 0.0, 1.0));\n"
        "\n"
        "    vec3 ambient = u_ambient_color * albedo + env_reflection * F * 0.50;\n"
        "    vec3 color = ambient + diffuse + spec_light + cc_spec * NdotL + emissive;\n"
        "\n"
        "    // Atmospheric distance fog\n"
        "    float dist = length(u_camera_pos - v_world_pos);\n"
        "    float fog_factor = clamp((dist - 140.0) / 600.0, 0.0, 0.65);\n"
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

// Procedural Grandstands with Audience Spectator Crowds & Roof Canopy
static ZeroMesh create_grandstands_mesh(void) {
    const int num_sections = 8;
    const float section_len = 15.0f;
    const int tiers = 6;
    const float tier_height = 0.65f;
    const float tier_depth = 1.10f;
    const float start_x = 108.0f;
    const float start_z = -20.0f;

    // Allocate geometry buffers
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

            // Tread face (Audience seating)
            int b1 = v_count;
            verts[v_count++] = (ZeroVertex){vec3(x0, y1, z0), vec3(0.0f, 1.0f, 0.0f), vec2((float)s * 0.125f, (float)t * 0.16f)};
            verts[v_count++] = (ZeroVertex){vec3(x1, y1, z0), vec3(0.0f, 1.0f, 0.0f), vec2((float)(s+1) * 0.125f, (float)t * 0.16f)};
            verts[v_count++] = (ZeroVertex){vec3(x1, y1, z1), vec3(0.0f, 1.0f, 0.0f), vec2((float)(s+1) * 0.125f, (float)(t+1) * 0.16f)};
            verts[v_count++] = (ZeroVertex){vec3(x0, y1, z1), vec3(0.0f, 1.0f, 0.0f), vec2((float)s * 0.125f, (float)(t+1) * 0.16f)};
            indices[i_count++] = b1; indices[i_count++] = b1 + 1; indices[i_count++] = b1 + 2;
            indices[i_count++] = b1; indices[i_count++] = b1 + 2; indices[i_count++] = b1 + 3;
        }

        // Roof Canopy over Grandstand
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

// Procedural Trackside Trees (Pine / Conifer trees for realistic depth and horizon)
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

        // Place outside the barrier
        float offset = 18.0f + (float)(i % 5) * 4.0f;
        float px = tx + cosf(angle) * offset;
        float pz = tz + sinf(angle) * offset;
        float h = 7.0f + (float)(i % 4) * 2.5f;
        float r = 2.4f + (float)(i % 3) * 0.6f;

        // Trunk (Quad cross)
        int b0 = v_count;
        verts[v_count++] = (ZeroVertex){vec3(px - 0.3f, 0.0f, pz), vec3(0.0f, 0.0f, 1.0f), vec2(0.0f, 0.0f)};
        verts[v_count++] = (ZeroVertex){vec3(px + 0.3f, 0.0f, pz), vec3(0.0f, 0.0f, 1.0f), vec2(1.0f, 0.0f)};
        verts[v_count++] = (ZeroVertex){vec3(px + 0.3f, h * 0.4f, pz), vec3(0.0f, 0.0f, 1.0f), vec2(1.0f, 1.0f)};
        verts[v_count++] = (ZeroVertex){vec3(px - 0.3f, h * 0.4f, pz), vec3(0.0f, 0.0f, 1.0f), vec2(0.0f, 1.0f)};
        indices[i_count++] = b0; indices[i_count++] = b0 + 1; indices[i_count++] = b0 + 2;
        indices[i_count++] = b0; indices[i_count++] = b0 + 2; indices[i_count++] = b0 + 3;

        // Pine Foliage Cone 1 (Lower)
        int b1 = v_count;
        verts[v_count++] = (ZeroVertex){vec3(px - r, h * 0.25f, pz), vec3(0.0f, 0.7f, 0.7f), vec2(0.0f, 0.0f)};
        verts[v_count++] = (ZeroVertex){vec3(px + r, h * 0.25f, pz), vec3(0.0f, 0.7f, 0.7f), vec2(1.0f, 0.0f)};
        verts[v_count++] = (ZeroVertex){vec3(px, h * 0.70f, pz), vec3(0.0f, 0.7f, 0.7f), vec2(0.5f, 1.0f)};
        indices[i_count++] = b1; indices[i_count++] = b1 + 1; indices[i_count++] = b1 + 2;

        // Pine Foliage Cone 2 (Upper)
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

    // 3. Subsystems: Audio, Camera, UI, Simulation
    ZeroAudioEngine audio;
    zero_audio_init(&audio, false);

    ZeroGameUI gui;
    zero_game_ui_init(&gui);

    ZeroRaceCamera rcam = zero_race_camera_create((float)platform.width / (float)platform.height);
    ZeroRaceGame game = zero_race_game_create();

    const Vec3 car_palette[4] = {
        {0.92f, 0.08f, 0.10f}, // Apex Red
        {0.12f, 0.18f, 0.45f}, // Midnight Blue
        {0.96f, 0.72f, 0.08f}, // Amber Gold
        {0.06f, 0.85f, 0.80f}  // Neon Teal
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

        // Camera Mode Cycling (Key C)
        if (platform.keys['c'] || platform.keys['C']) {
            if (!c_key_prev) {
                zero_race_camera_cycle_mode(&rcam);
                gui.active_camera_mode = (int)rcam.mode;
            }
            c_key_prev = true;
        } else {
            c_key_prev = false;
        }

        // Pause Menu (ESC or P)
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
        }

        if (zero_game_ui_is_restart_requested(&gui)) {
            zero_game_ui_clear_restart_request(&gui);
            game = zero_race_game_create();
            gui.countdown_timer = 3.0f;
            gui.prev_countdown_timer = 3.0f;
            gui.state = ZERO_STATE_COUNTDOWN;
        }

        // Responsive Driving Controls with Automatic Reverse
        if (gui.state == ZERO_STATE_RACING) {
            ZeroVehicleEnhanced *pcar = &game.cars[0];

            Mat4 ch_rot = quat_to_mat4(pcar->orientation);
            Vec3 car_fwd = vec3(ch_rot.m[8], ch_rot.m[9], ch_rot.m[10]);
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
                    // Moving in reverse: W brakes to stop
                    pcar->brake_input = 1.0f;
                    pcar->throttle_input = 0.0f;
                } else {
                    // Forward Drive in 1st-6th gear
                    if (pcar->transmission.current_gear <= ZERO_GEAR_NEUTRAL) {
                        pcar->transmission.current_gear = ZERO_GEAR_1;
                    }
                    pcar->throttle_input = 1.0f;
                    pcar->brake_input = 0.0f;
                }
            } else if (key_decel) {
                if (v_fwd > 1.5f) {
                    // Moving forward: S brakes to slow down
                    pcar->brake_input = 1.0f;
                    pcar->throttle_input = 0.0f;
                } else {
                    // Stopped or reversing: S accelerates backwards in Reverse gear!
                    pcar->transmission.current_gear = ZERO_GEAR_REVERSE;
                    pcar->throttle_input = 1.0f;
                    pcar->brake_input = 0.0f;
                }
            } else {
                pcar->throttle_input = 0.0f;
                pcar->brake_input = 0.0f;
            }

            // Advance simulation step (XPBD chassis, tire friction, barrier deflection, AI pursuit, car collisions)
            zero_race_game_step(&game, dt);

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
        Mat4 proj = zero_race_camera_get_projection(&rcam);

        // Audio Update
        ZeroAudioParams audio_params;
        memset(&audio_params, 0, sizeof(audio_params));
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
        glClearColor(0.10f, 0.14f, 0.22f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        zero_shader_bind(&shader);
        zero_shader_set_mat4(&shader, "u_view", view);
        zero_shader_set_mat4(&shader, "u_proj", proj);
        zero_shader_set_vec3(&shader, "u_camera_pos", rcam.current_pos);
        zero_shader_set_vec3(&shader, "u_light_dir", vec3(-0.45f, -0.75f, -0.45f));
        zero_shader_set_vec3(&shader, "u_light_color", vec3(1.20f, 1.15f, 1.00f));
        zero_shader_set_vec3(&shader, "u_ambient_color", vec3(0.35f, 0.38f, 0.44f));
        zero_shader_set_float(&shader, "u_time", (float)cur_time);

        // 1. Realistic Sky Dome with Sun Disk and Clouds
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

        // Green Terrain
        zero_shader_set_float(&shader, "u_roughness", 0.88f);
        zero_shader_set_float(&shader, "u_metallic", 0.0f);
        zero_shader_set_vec3(&shader, "u_object_color", vec3(0.22f, 0.52f, 0.20f));
        zero_shader_set_float(&shader, "u_material_mode", 3.0f);
        zero_mesh_draw(&terrain_mesh);

        // Asphalt Road
        zero_shader_set_float(&shader, "u_roughness", 0.70f);
        zero_shader_set_float(&shader, "u_metallic", 0.05f);
        zero_shader_set_vec3(&shader, "u_object_color", vec3(0.18f, 0.18f, 0.20f));
        zero_shader_set_float(&shader, "u_material_mode", 1.0f);
        zero_mesh_draw(&road_mesh);

        // Rumble Curbs
        zero_shader_set_float(&shader, "u_roughness", 0.40f);
        zero_shader_set_float(&shader, "u_metallic", 0.10f);
        zero_shader_set_vec3(&shader, "u_object_color", vec3(0.92f, 0.08f, 0.10f));
        zero_shader_set_float(&shader, "u_material_mode", 2.0f);
        zero_mesh_draw(&curb_mesh);

        // Barriers
        zero_shader_set_float(&shader, "u_roughness", 0.32f);
        zero_shader_set_float(&shader, "u_metallic", 0.70f);
        zero_shader_set_vec3(&shader, "u_object_color", vec3(0.22f, 0.45f, 0.92f));
        zero_shader_set_float(&shader, "u_material_mode", 4.0f);
        zero_mesh_draw(&barrier_mesh);

        // Grandstand Stadium Bleachers with Spectator Crowds
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

        // Starting Line Gantry
        zero_shader_set_float(&shader, "u_roughness", 0.40f);
        zero_shader_set_float(&shader, "u_metallic", 0.75f);
        zero_shader_set_vec3(&shader, "u_object_color", vec3(0.35f, 0.38f, 0.42f));
        zero_shader_set_float(&shader, "u_material_mode", 8.0f);
        zero_mesh_draw(&gantry_mesh);

        // 3. Vehicles & Wheels (4 High-Poly Cars with PBR Clearcoat & Sub-materials)
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
            zero_shader_set_float(&shader, "u_roughness", 0.16f);
            zero_shader_set_float(&shader, "u_metallic", 0.88f);
            zero_shader_set_vec3(&shader, "u_object_color", body_color);
            zero_shader_set_float(&shader, "u_material_mode", 0.0f);
            zero_mesh_draw(&car_mesh);

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
                zero_shader_set_vec3(&shader, "u_object_color", vec3(0.85f, 0.88f, 0.92f));
                zero_shader_set_float(&shader, "u_material_mode", 6.0f);
                zero_mesh_draw(&wheel_mesh);
            }
        }

        // 4. UI Telemetry & Full State Machine Overlay Pass
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
    zero_shader_destroy(&shader);
    zero_arena_destroy(&arena);
    zero_platform_shutdown(&platform);

    return 0;
}

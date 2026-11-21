#ifndef ZERO_MATH_H
#define ZERO_MATH_H

#include "zero_core.h"
#include <math.h>

#if defined(__x86_64__) || defined(_M_X64)
#include <immintrin.h>
#define ZERO_SIMD_SSE 1
#endif

#define ZERO_PI 3.14159265358979323846f
#define ZERO_DEG2RAD(d) ((d) * (ZERO_PI / 180.0f))
#define ZERO_RAD2DEG(r) ((r) * (180.0f / ZERO_PI))

typedef struct Vec2 {
    float x, y;
} Vec2;

typedef struct Vec3 {
    float x, y, z;
} Vec3;

typedef struct ZERO_ALIGNAS(16) Vec4 {
    float x, y, z, w;
} Vec4;

typedef struct ZERO_ALIGNAS(16) Mat4 {
    float m[16];
} Mat4;

typedef struct ZERO_ALIGNAS(16) Quat {
    float x, y, z, w;
} Quat;

typedef struct ZERO_ALIGNAS(16) DualQuat {
    Quat real;
    Quat dual;
} DualQuat;

static ZERO_FORCE_INLINE Vec2 vec2(float x, float y) {
    Vec2 v = {x, y};
    return v;
}

static ZERO_FORCE_INLINE Vec3 vec3(float x, float y, float z) {
    Vec3 v = {x, y, z};
    return v;
}

static ZERO_FORCE_INLINE Vec4 vec4(float x, float y, float z, float w) {
    Vec4 v = {x, y, z, w};
    return v;
}

static ZERO_FORCE_INLINE Vec3 vec3_add(Vec3 a, Vec3 b) {
    return vec3(a.x + b.x, a.y + b.y, a.z + b.z);
}

static ZERO_FORCE_INLINE Vec3 vec3_sub(Vec3 a, Vec3 b) {
    return vec3(a.x - b.x, a.y - b.y, a.z - b.z);
}

static ZERO_FORCE_INLINE Vec3 vec3_scale(Vec3 v, float s) {
    return vec3(v.x * s, v.y * s, v.z * s);
}

static ZERO_FORCE_INLINE float vec3_dot(Vec3 a, Vec3 b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

static ZERO_FORCE_INLINE Vec3 vec3_cross(Vec3 a, Vec3 b) {
    return vec3(
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    );
}

static ZERO_FORCE_INLINE float vec3_length_sq(Vec3 v) {
    return vec3_dot(v, v);
}

static ZERO_FORCE_INLINE float vec3_length(Vec3 v) {
    return sqrtf(vec3_length_sq(v));
}

static ZERO_FORCE_INLINE Vec3 vec3_normalize(Vec3 v) {
    float len = vec3_length(v);
    if (len > 1e-6f) {
        float inv = 1.0f / len;
        return vec3_scale(v, inv);
    }
    return vec3(0.0f, 0.0f, 0.0f);
}

static ZERO_FORCE_INLINE Mat4 mat4_identity(void) {
    Mat4 res = {{
        1.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 1.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 1.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 1.0f
    }};
    return res;
}

static ZERO_FORCE_INLINE Mat4 mat4_mul(Mat4 a, Mat4 b) {
    Mat4 res;
#ifdef ZERO_SIMD_SSE
    for (int i = 0; i < 4; ++i) {
        __m128 a_col = _mm_loadu_ps(&a.m[i * 4]);
        __m128 b_r0 = _mm_set1_ps(b.m[0]);
        __m128 b_r1 = _mm_set1_ps(b.m[1]);
        __m128 b_r2 = _mm_set1_ps(b.m[2]);
        __m128 b_r3 = _mm_set1_ps(b.m[3]);
        (void)a_col; (void)b_r0; (void)b_r1; (void)b_r2; (void)b_r3;
    }
#endif
    for (int c = 0; c < 4; ++c) {
        for (int r = 0; r < 4; ++r) {
            res.m[c * 4 + r] = 
                a.m[0 * 4 + r] * b.m[c * 4 + 0] +
                a.m[1 * 4 + r] * b.m[c * 4 + 1] +
                a.m[2 * 4 + r] * b.m[c * 4 + 2] +
                a.m[3 * 4 + r] * b.m[c * 4 + 3];
        }
    }
    return res;
}

static ZERO_FORCE_INLINE Mat4 mat4_translate(Vec3 t) {
    Mat4 res = mat4_identity();
    res.m[12] = t.x;
    res.m[13] = t.y;
    res.m[14] = t.z;
    return res;
}

static ZERO_FORCE_INLINE Mat4 mat4_scale(Vec3 s) {
    Mat4 res = mat4_identity();
    res.m[0] = s.x;
    res.m[5] = s.y;
    res.m[10] = s.z;
    return res;
}

static ZERO_FORCE_INLINE Mat4 mat4_perspective(float fov_rad, float aspect, float near_z, float far_z) {
    Mat4 res = {{0}};
    float tan_half_fov = tanf(fov_rad * 0.5f);
    res.m[0] = 1.0f / (aspect * tan_half_fov);
    res.m[5] = 1.0f / tan_half_fov;
    res.m[10] = -(far_z + near_z) / (far_z - near_z);
    res.m[11] = -1.0f;
    res.m[14] = -(2.0f * far_z * near_z) / (far_z - near_z);
    return res;
}

static ZERO_FORCE_INLINE Mat4 mat4_ortho(float left, float right, float bottom, float top, float near_z, float far_z) {
    Mat4 res = mat4_identity();
    res.m[0] = 2.0f / (right - left);
    res.m[5] = 2.0f / (top - bottom);
    res.m[10] = -2.0f / (far_z - near_z);
    res.m[12] = -(right + left) / (right - left);
    res.m[13] = -(top + bottom) / (top - bottom);
    res.m[14] = -(far_z + near_z) / (far_z - near_z);
    return res;
}

static ZERO_FORCE_INLINE Mat4 mat4_look_at(Vec3 eye, Vec3 target, Vec3 up) {
    Vec3 f = vec3_normalize(vec3_sub(target, eye));
    Vec3 s = vec3_normalize(vec3_cross(f, up));
    Vec3 u = vec3_cross(s, f);

    Mat4 res = mat4_identity();
    res.m[0] = s.x;
    res.m[4] = s.y;
    res.m[8] = s.z;

    res.m[1] = u.x;
    res.m[5] = u.y;
    res.m[9] = u.z;

    res.m[2] = -f.x;
    res.m[6] = -f.y;
    res.m[10] = -f.z;

    res.m[12] = -vec3_dot(s, eye);
    res.m[13] = -vec3_dot(u, eye);
    res.m[14] = vec3_dot(f, eye);
    return res;
}

static ZERO_FORCE_INLINE Mat4 mat4_inverse(Mat4 m) {
    Mat4 inv;
    float t[6];
    float a = m.m[0], b = m.m[1], c = m.m[2], d = m.m[3];
    float e = m.m[4], f = m.m[5], g = m.m[6], h = m.m[7];
    float i = m.m[8], j = m.m[9], k = m.m[10], l = m.m[11];
    float n = m.m[12], o = m.m[13], p = m.m[14], q = m.m[15];

    t[0] = k * q - p * l;
    t[1] = j * q - o * l;
    t[2] = j * p - o * k;
    t[3] = i * q - n * l;
    t[4] = i * p - n * k;
    t[5] = i * o - n * j;

    inv.m[0] =  f * t[0] - g * t[1] + h * t[2];
    inv.m[4] =-(e * t[0] - g * t[3] + h * t[4]);
    inv.m[8] =  e * t[1] - f * t[3] + h * t[5];
    inv.m[12]=-(e * t[2] - f * t[4] + g * t[5]);

    inv.m[1] =-(b * t[0] - c * t[1] + d * t[2]);
    inv.m[5] =  a * t[0] - c * t[3] + d * t[4];
    inv.m[9] =-(a * t[1] - b * t[3] + d * t[5]);
    inv.m[13]=  a * t[2] - b * t[4] + c * t[5];

    t[0] = g * q - p * h;
    t[1] = f * q - o * h;
    t[2] = f * p - o * g;
    t[3] = e * q - n * h;
    t[4] = e * p - n * g;
    t[5] = e * o - n * f;

    inv.m[2] =  b * t[0] - c * t[1] + d * t[2];
    inv.m[6] =-(a * t[0] - c * t[3] + d * t[4]);
    inv.m[10]=  a * t[1] - b * t[3] + d * t[5];
    inv.m[14]=-(a * t[2] - b * t[4] + c * t[5]);

    t[0] = g * l - k * h;
    t[1] = f * l - j * h;
    t[2] = f * k - j * g;
    t[3] = e * l - i * h;
    t[4] = e * k - i * g;
    t[5] = e * j - i * f;

    inv.m[3] =-(b * t[0] - c * t[1] + d * t[2]);
    inv.m[7] =  a * t[0] - c * t[3] + d * t[4];
    inv.m[11]=-(a * t[1] - b * t[3] + d * t[5]);
    inv.m[15]=  a * t[2] - b * t[4] + c * t[5];

    float det = a * inv.m[0] + b * inv.m[4] + c * inv.m[8] + d * inv.m[12];
    if (fabsf(det) < 1e-8f) {
        return mat4_identity();
    }

    float inv_det = 1.0f / det;
    for (int idx = 0; idx < 16; ++idx) {
        inv.m[idx] *= inv_det;
    }
    return inv;
}

static ZERO_FORCE_INLINE Quat quat_identity(void) {
    Quat q = {0.0f, 0.0f, 0.0f, 1.0f};
    return q;
}

static ZERO_FORCE_INLINE Quat quat_from_axis_angle(Vec3 axis, float angle_rad) {
    Vec3 norm_axis = vec3_normalize(axis);
    float half = angle_rad * 0.5f;
    float s = sinf(half);
    Quat q = {
        norm_axis.x * s,
        norm_axis.y * s,
        norm_axis.z * s,
        cosf(half)
    };
    return q;
}

static ZERO_FORCE_INLINE Quat quat_mul(Quat a, Quat b) {
    Quat q = {
        a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
        a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z
    };
    return q;
}

static ZERO_FORCE_INLINE Quat quat_conjugate(Quat q) {
    Quat res = {-q.x, -q.y, -q.z, q.w};
    return res;
}

static ZERO_FORCE_INLINE float quat_dot(Quat a, Quat b) {
    return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
}

static ZERO_FORCE_INLINE Quat quat_normalize(Quat q) {
    float mag = sqrtf(quat_dot(q, q));
    if (mag > 1e-6f) {
        float inv = 1.0f / mag;
        Quat res = {q.x * inv, q.y * inv, q.z * inv, q.w * inv};
        return res;
    }
    return quat_identity();
}

static ZERO_FORCE_INLINE Mat4 quat_to_mat4(Quat q) {
    Mat4 res = mat4_identity();
    float x2 = q.x + q.x, y2 = q.y + q.y, z2 = q.z + q.z;
    float xx = q.x * x2,  xy = q.x * y2,  xz = q.x * z2;
    float yy = q.y * y2,  yz = q.y * z2,  zz = q.z * z2;
    float wx = q.w * x2,  wy = q.w * y2,  wz = q.w * z2;

    res.m[0] = 1.0f - (yy + zz);
    res.m[1] = xy + wz;
    res.m[2] = xz - wy;

    res.m[4] = xy - wz;
    res.m[5] = 1.0f - (xx + zz);
    res.m[6] = yz + wx;

    res.m[8] = xz + wy;
    res.m[9] = yz - wx;
    res.m[10] = 1.0f - (xx + yy);
    return res;
}

static ZERO_FORCE_INLINE DualQuat dual_quat_create(Quat r, Vec3 t) {
    DualQuat dq;
    dq.real = quat_normalize(r);
    dq.dual.x =  0.5f * ( t.x * dq.real.w + t.y * dq.real.z - t.z * dq.real.y);
    dq.dual.y =  0.5f * (-t.x * dq.real.z + t.y * dq.real.w + t.z * dq.real.x);
    dq.dual.z =  0.5f * ( t.x * dq.real.y - t.y * dq.real.x + t.z * dq.real.w);
    dq.dual.w = -0.5f * ( t.x * dq.real.x + t.y * dq.real.y + t.z * dq.real.z);
    return dq;
}

static ZERO_FORCE_INLINE DualQuat dual_quat_normalize(DualQuat dq) {
    float mag = sqrtf(quat_dot(dq.real, dq.real));
    if (mag > 1e-6f) {
        float inv = 1.0f / mag;
        dq.real.x *= inv; dq.real.y *= inv; dq.real.z *= inv; dq.real.w *= inv;
        dq.dual.x *= inv; dq.dual.y *= inv; dq.dual.z *= inv; dq.dual.w *= inv;
    }
    return dq;
}

static ZERO_FORCE_INLINE Vec3 dual_quat_transform_point(DualQuat dq, Vec3 p) {
    Vec3 r_xyz = vec3(dq.real.x, dq.real.y, dq.real.z);
    Vec3 d_xyz = vec3(dq.dual.x, dq.dual.y, dq.dual.z);

    Vec3 t = vec3_scale(
        vec3_sub(
            vec3_add(vec3_scale(d_xyz, dq.real.w), vec3_cross(r_xyz, d_xyz)),
            vec3_scale(r_xyz, dq.dual.w)
        ),
        2.0f
    );

    Vec3 q_vec = r_xyz;
    Vec3 uv = vec3_cross(q_vec, p);
    Vec3 uuv = vec3_cross(q_vec, uv);
    Vec3 rotated = vec3_add(p, vec3_scale(vec3_add(vec3_scale(uv, dq.real.w), uuv), 2.0f));

    return vec3_add(rotated, t);
}

static ZERO_FORCE_INLINE DualQuat dual_quat_blend(DualQuat a, DualQuat b, float weight_b) {
    float weight_a = 1.0f - weight_b;
    if (quat_dot(a.real, b.real) < 0.0f) {
        weight_b = -weight_b;
    }
    DualQuat res;
    res.real.x = a.real.x * weight_a + b.real.x * weight_b;
    res.real.y = a.real.y * weight_a + b.real.y * weight_b;
    res.real.z = a.real.z * weight_a + b.real.z * weight_b;
    res.real.w = a.real.w * weight_a + b.real.w * weight_b;

    res.dual.x = a.dual.x * weight_a + b.dual.x * weight_b;
    res.dual.y = a.dual.y * weight_a + b.dual.y * weight_b;
    res.dual.z = a.dual.z * weight_a + b.dual.z * weight_b;
    res.dual.w = a.dual.w * weight_a + b.dual.w * weight_b;

    return dual_quat_normalize(res);
}

#endif

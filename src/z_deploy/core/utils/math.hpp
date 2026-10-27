#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <string>
#include <vector>

namespace math
{

// Component positions of [w, x, y, z] inside a quaternion vector laid out
// as `order` ("wxyz" or "xyzw").
inline std::array<int, 4> quat_layout(const std::string &order)
{
    return order == "xyzw" ? std::array<int, 4>{3, 0, 1, 2}
                           : std::array<int, 4>{0, 1, 2, 3};
}

// Hamilton product a * b; the result keeps the inputs' component order.
inline std::vector<float> quat_multiply(const std::vector<float> &a,
                                        const std::vector<float> &b,
                                        const std::string &order = "wxyz")
{
    if (a.size() != 4 || b.size() != 4)
        return {};

    const auto l = quat_layout(order);
    const float aw = a[l[0]], ax = a[l[1]], ay = a[l[2]], az = a[l[3]];
    const float bw = b[l[0]], bx = b[l[1]], by = b[l[2]], bz = b[l[3]];

    std::vector<float> out(4);
    out[l[0]] = aw * bw - ax * bx - ay * by - az * bz;
    out[l[1]] = aw * bx + ax * bw + ay * bz - az * by;
    out[l[2]] = aw * by - ax * bz + ay * bw + az * bx;
    out[l[3]] = aw * bz + ax * by - ay * bx + az * bw;
    return out;
}

// quaternion yaw angle, radians
inline float yaw_from_quat(const std::vector<float> &q, const std::string &order = "wxyz")
{
    if (q.size() != 4)
        return 0.0F;

    const auto l = quat_layout(order);
    const float w = q[l[0]], x = q[l[1]], y = q[l[2]], z = q[l[3]];
    return std::atan2(2.0F * (w * z + x * y), 1.0F - 2.0F * (y * y + z * z));
}

// euler(roll, pitch, yaw) -> quaternion, radians; output in `order`.
inline std::vector<float> euler_to_quat(const std::vector<float> &euler,
                                        const std::string &order = "wxyz")
{
    if (euler.size() != 3)
        return {};

    const float half_roll = 0.5F * euler[0];
    const float half_pitch = 0.5F * euler[1];
    const float half_yaw = 0.5F * euler[2];

    const float cr = std::cos(half_roll);
    const float sr = std::sin(half_roll);
    const float cp = std::cos(half_pitch);
    const float sp = std::sin(half_pitch);
    const float cy = std::cos(half_yaw);
    const float sy = std::sin(half_yaw);

    const float w = cr * cp * cy + sr * sp * sy;
    const float x = sr * cp * cy - cr * sp * sy;
    const float y = cr * sp * cy + sr * cp * sy;
    const float z = cr * cp * sy - sr * sp * cy;

    const auto l = quat_layout(order);
    std::vector<float> out(4);
    out[l[0]] = w;
    out[l[1]] = x;
    out[l[2]] = y;
    out[l[3]] = z;
    return out;
}

// Rotate vector by inverse quaternion: q^{-1} * v * q
inline std::vector<float> quat_rotate_inverse(const std::vector<float> &quaternion,
                                              const std::vector<float> &vector,
                                              const std::string &order = "wxyz")
{
    if (quaternion.size() != 4 || vector.size() != 3)
        return {0.0F, 0.0F, 0.0F};

    const auto l = quat_layout(order);
    const float w = quaternion[l[0]], x = quaternion[l[1]];
    const float y = quaternion[l[2]], z = quaternion[l[3]];

    const float v_x = vector[0];
    const float v_y = vector[1];
    const float v_z = vector[2];

    const float a_x = v_x * (2.0F * w * w - 1.0F);
    const float a_y = v_y * (2.0F * w * w - 1.0F);
    const float a_z = v_z * (2.0F * w * w - 1.0F);

    const float cross_x = y * v_z - z * v_y;
    const float cross_y = z * v_x - x * v_z;
    const float cross_z = x * v_y - y * v_x;

    const float b_x = cross_x * w * 2.0F;
    const float b_y = cross_y * w * 2.0F;
    const float b_z = cross_z * w * 2.0F;

    const float dot = x * v_x + y * v_y + z * v_z;
    const float c_x = x * dot * 2.0F;
    const float c_y = y * dot * 2.0F;
    const float c_z = z * dot * 2.0F;

    return {a_x - b_x + c_x, a_y - b_y + c_y, a_z - b_z + c_z};
}

// Project world gravity [0, 0, -1] into body frame.
inline std::vector<float> gravity_from_quat(const std::vector<float> &quaternion,
                                            const std::string &order = "wxyz")
{
    return quat_rotate_inverse(quaternion, {0.0F, 0.0F, -1.0F}, order);
}

// First two rows of the row-major rotation matrix, matching rl_sar's
// MatrixFirstTwoColumns helper ordering: R00, R01, R10, R11, R20, R21.
inline std::vector<float> quat_to_ori_6d(const std::vector<float> &quaternion,
                                         const std::string &order = "wxyz")
{
    if (quaternion.size() != 4)
        return {0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F};

    const auto l = quat_layout(order);
    const float w = quaternion[l[0]], x = quaternion[l[1]];
    const float y = quaternion[l[2]], z = quaternion[l[3]];
    return {
        1.0F - 2.0F * (y * y + z * z),
        2.0F * (x * y - w * z),
        2.0F * (x * y + w * z),
        1.0F - 2.0F * (x * x + z * z),
        2.0F * (x * z - w * y),
        2.0F * (y * z + w * x)};
}

// Linear interpolation: (1 - t) * a + t * b
inline float lerp(float a, float b, float t)
{
    return a + t * (b - a);
}

inline std::vector<float> lerp(const std::vector<float> &a,
                               const std::vector<float> &b,
                               float t)
{
    const std::size_t n = std::min(a.size(), b.size());
    std::vector<float> out(n);
    for (std::size_t i = 0; i < n; ++i)
        out[i] = lerp(a[i], b[i], t);
    return out;
}

// Normalize a quaternion vector; component order is preserved.
inline std::vector<float> normalize(const std::vector<float> &q)
{
    if (q.size() != 4)
        return q;

    const float n = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    if (n < 1e-8F)
        return q;
    return {q[0] / n, q[1] / n, q[2] / n, q[3] / n};
}

// q* (w unchanged, xyz negated); component order is preserved.
inline std::vector<float> conjugate(const std::vector<float> &q,
                                    const std::string &order = "wxyz")
{
    if (q.size() != 4)
        return {};

    const auto l = quat_layout(order);
    std::vector<float> out = q;
    out[l[1]] = -q[l[1]];
    out[l[2]] = -q[l[2]];
    out[l[3]] = -q[l[3]];
    return out;
}

// Spherical linear interpolation between unit quaternions; same order in/out.
inline std::vector<float> slerp(const std::vector<float> &a,
                                const std::vector<float> &b,
                                float t,
                                const std::string &order = "wxyz")
{
    if (a.size() != 4 || b.size() != 4)
        return {};

    const auto l = quat_layout(order);
    const auto q0 = normalize(a);
    const auto q1 = normalize(b);

    float q0w = q0[l[0]], q0x = q0[l[1]], q0y = q0[l[2]], q0z = q0[l[3]];
    float q1w = q1[l[0]], q1x = q1[l[1]], q1y = q1[l[2]], q1z = q1[l[3]];

    float dot = q0w * q1w + q0x * q1x + q0y * q1y + q0z * q1z;

    // Take the shortest path on the quaternion sphere.
    if (dot < 0.0F)
    {
        q1w = -q1w;
        q1x = -q1x;
        q1y = -q1y;
        q1z = -q1z;
        dot = -dot;
    }

    // Near-parallel: fall back to linear interpolation.
    if (dot > 0.9995F)
    {
        std::vector<float> out(4);
        out[l[0]] = lerp(q0w, q1w, t);
        out[l[1]] = lerp(q0x, q1x, t);
        out[l[2]] = lerp(q0y, q1y, t);
        out[l[3]] = lerp(q0z, q1z, t);
        return normalize(out);
    }

    const float theta_0 = std::acos(std::clamp(dot, -1.0F, 1.0F));
    const float sin_theta_0 = std::sin(theta_0);
    const float theta = theta_0 * t;
    const float s0 = std::sin(theta_0 - theta) / sin_theta_0;
    const float s1 = std::sin(theta) / sin_theta_0;

    std::vector<float> out(4);
    out[l[0]] = s0 * q0w + s1 * q1w;
    out[l[1]] = s0 * q0x + s1 * q1x;
    out[l[2]] = s0 * q0y + s1 * q1y;
    out[l[3]] = s0 * q0z + s1 * q1z;
    return out;
}

} // namespace math

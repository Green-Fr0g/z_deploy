#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace utils
{

// empty + default -> fill; size == 1 -> broadcast to num; otherwise keep as-is.
inline void expand_to_num(std::vector<float> &values,
                          std::size_t num,
                          std::optional<float> default_value = std::nullopt)
{
    if (num == 0)
        return;
    if (values.empty())
    {
        if (default_value.has_value())
            values.assign(num, *default_value);
        return;
    }
    if (values.size() == 1)
        values.assign(num, values.front());
}

// If empty, fill with identity sequence [0, 1, ..., num-1].
inline void fill_identity(std::vector<int> &values, std::size_t num)
{
    if (!values.empty() || num == 0)
        return;
    values.resize(num);
    for (std::size_t i = 0; i < num; ++i)
        values[i] = static_cast<int>(i);
}

// Place policy-ordered values onto hardware indices: out[mapping[i]] = values[i].
template <typename T>
inline std::vector<T> scatter(const std::vector<T> &values,
                              const std::vector<int> &mapping,
                              std::size_t size)
{
    std::vector<T> out(size);
    const auto count = std::min(values.size(), mapping.size());
    for (std::size_t i = 0; i < count; ++i)
    {
        const int hardware_index = mapping[i];
        if (hardware_index < 0 || static_cast<std::size_t>(hardware_index) >= size)
            continue;
        out[static_cast<std::size_t>(hardware_index)] = values[i];
    }
    return out;
}

// Reorder values by composing input_mapping and output_mapping.
template <typename T>
inline std::vector<T> remap(const std::vector<T> &input_data,
                            const std::vector<int> &input_mapping = {},
                            const std::vector<int> &output_mapping = {})
{
    std::vector<int> input_indices = input_mapping;
    std::vector<int> output_indices = output_mapping;
    const auto default_size = input_data.size();

    fill_identity(input_indices, default_size);
    fill_identity(output_indices, default_size);

    if (input_indices.size() != output_indices.size())
        throw std::invalid_argument("input_mapping and output_mapping must have the same size");

    std::vector<T> output_data(default_size);
    for (std::size_t i = 0; i < input_data.size(); ++i)
    {
        const auto input_index = input_indices[i];
        const auto output_index = output_indices[static_cast<std::size_t>(input_index)];
        output_data[static_cast<std::size_t>(output_index)] = input_data[i];
    }
    return output_data;
}

 
// Remap a list of joint indices with the same old/new mapping composition as remap().
// Invalid / out-of-range indices are dropped.
inline std::vector<int> remap_indices(const std::vector<int> &indices,
                                      const std::vector<int> &old_mapping,
                                      const std::vector<int> &new_mapping)
{
    std::vector<int> out;
    out.reserve(indices.size());
    for (const int idx : indices)
    {
        if (idx < 0 || static_cast<std::size_t>(idx) >= old_mapping.size())
            continue;
        const int canonical_index = old_mapping[static_cast<std::size_t>(idx)];
        const auto it = std::find(new_mapping.begin(), new_mapping.end(), canonical_index);
        if (it == new_mapping.end())
            continue;
        out.push_back(static_cast<int>(std::distance(new_mapping.begin(), it)));
    }
    return out;
}
 
// Build gather indices: for each name in to_names, find its index in from_names.
// Returns empty if any to_name is missing from from_names.
inline std::vector<int> mapping_from_names(const std::vector<std::string> &from_names,
                                           const std::vector<std::string> &to_names)
{
    std::vector<int> indices;
    indices.reserve(to_names.size());
    for (const auto &name : to_names)
    {
        const auto it = std::find(from_names.begin(), from_names.end(), name);
        if (it == from_names.end())
            return {};
        indices.push_back(static_cast<int>(std::distance(from_names.begin(), it)));
    }
    return indices;
}
 
// Gather values by indices: output[i] = input[indices[i]].
template <typename T>
inline std::vector<T> gather(const std::vector<T> &input, const std::vector<int> &indices)
{
    std::vector<T> output;
    output.reserve(indices.size());
    for (const int idx : indices)
    {
        if (idx < 0 || static_cast<std::size_t>(idx) >= input.size())
            output.push_back(T{});
        else
            output.push_back(input[static_cast<std::size_t>(idx)]);
    }
    return output;
}

inline void apply_scale(std::vector<float> &values, const std::vector<float> &scales)
{
    if (scales.empty())
        return;
    if (scales.size() == 1U)
    {
        for (auto &value : values)
            value *= scales.front();
        return;
    }
    for (std::size_t i = 0; i < values.size(); ++i)
        values[i] *= i < scales.size() ? scales[i] : 1.0F;
}

inline void clip(std::vector<float> &values, float limit)
{
    if (limit <= 0.0F)
        return;
    for (auto &value : values)
        value = std::clamp(value, -limit, limit);
}

// Quaternion order: wxyz. Body Z elevation: upright 90, horizontal 0, inverted -90 degrees.
// Returns true below limit_deg [0, 90]; yaw does not affect the result.
// Invalid quaternion or limit also returns true to request protection.
inline bool tilt_limit_exceeded(const std::vector<float> &quaternion, double limit_deg)
{
    if (quaternion.size() != 4 || !std::isfinite(limit_deg) ||
        limit_deg < 0.0 || limit_deg > 90.0)
        return true;

    double norm_squared = 0.0;
    for (const float value : quaternion)
    {
        if (!std::isfinite(value))
            return true;
        norm_squared += static_cast<double>(value) * value;
    }
    if (norm_squared < 1e-12)
        return true;

    const double x = quaternion[1];
    const double y = quaternion[2];
    // Dot product of body Z and world Z, normalized for non-unit quaternions.
    const double sin_elevation = std::clamp(1.0 - 2.0 * (x * x + y * y) / norm_squared, -1.0, 1.0);
    constexpr double deg_to_rad = 3.14159265358979323846 / 180.0;
    return sin_elevation < std::sin(limit_deg * deg_to_rad);
}

} // namespace utils

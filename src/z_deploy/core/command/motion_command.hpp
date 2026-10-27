#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <z_deploy/types.hpp>
#include <z_deploy/utils/utils.hpp>
#include <z_deploy/utils/math.hpp>

class MotionCommand
{
public:
    MotionCommand() = default;

    explicit MotionCommand(const std::filesystem::path &csv_path,
                           std::size_t joint_count = 0,
                           double input_fps = 60.0,
                           const std::string &order = "xyzw",
                           const std::vector<std::string> &input_joints_name = {},
                           const std::vector<std::string> &out_joints_name = {},
                           const std::vector<int> &waist_joint_indices = {})
    {
        load(csv_path,
             joint_count,
             input_fps,
             order,
             input_joints_name,
             out_joints_name,
             waist_joint_indices);
    }

    bool load(const std::filesystem::path &csv_path,
              std::size_t joint_count = 0,
              double input_fps = 60.0,
              const std::string &order = "xyzw",
              const std::vector<std::string> &input_joints_name = {},
              const std::vector<std::string> &out_joints_name = {},
              const std::vector<int> &waist_joint_indices = {})
    {
        frames_.clear();
        duration_ = 0.0;
        alignment_ = {1.0F, 0.0F, 0.0F, 0.0F};
        joint_count_ = joint_count;
        order_ = order;
        waist_joint_indices_ = waist_joint_indices;
        input_dt_ = input_fps > 0.0 ? 1.0 / input_fps : 0.0;
        if (csv_path.empty() || input_dt_ <= 0.0)
            return false;
        if (order_ != "xyzw" && order_ != "wxyz")
            return false;

        // Remap only when both name lists are provided; otherwise keep CSV order.
        const bool remap_joints =
            !input_joints_name.empty() && !out_joints_name.empty();
        std::vector<int> gather_indices;
        if (remap_joints)
        {
            gather_indices = utils::mapping_from_names(input_joints_name, out_joints_name);
            if (gather_indices.empty())
                return false;
            if (joint_count_ == 0)
                joint_count_ = input_joints_name.size();
            if (joint_count_ != input_joints_name.size())
                return false;
        }

        std::ifstream file(csv_path);
        if (!file.is_open())
            return false;

        std::string line;
        while (std::getline(file, line))
        {
            std::vector<float> row;
            if (!parse_row(line, row))
                continue;
            if (row.size() < 7)
                continue;

            MotionFrame frame;
            frame.root_position = {row[0], row[1], row[2]};
            // Quaternion columns follow the file order; internals use wxyz.
            const std::vector<float> raw_quat{row[3], row[4], row[5], row[6]};
            const auto layout = math::quat_layout(order_);
            frame.root_quaternion = math::normalize({raw_quat[layout[0]], raw_quat[layout[1]],
                                                     raw_quat[layout[2]], raw_quat[layout[3]]});

            const auto available_joints = row.size() - 7U;
            if (joint_count_ == 0)
                joint_count_ = available_joints;
            if (available_joints < joint_count_)
            {
                frames_.clear();
                duration_ = 0.0;
                return false;
            }

            frame.joint_position.assign(
                row.begin() + 7,
                row.begin() + 7 + static_cast<std::ptrdiff_t>(joint_count_));
            if (remap_joints)
            {
                try
                {
                    frame.joint_position = utils::gather(frame.joint_position, gather_indices);
                }
                catch (...)
                {
                    frames_.clear();
                    duration_ = 0.0;
                    return false;
                }
            }
            frame.joint_velocity.assign(frame.joint_position.size(), 0.0F);
            frame.anchor_quaternion = compute_anchor(
                frame.root_quaternion, frame.joint_position, waist_joint_indices_);
            frame.time = static_cast<double>(frames_.size()) * input_dt_;
            frames_.push_back(std::move(frame));
        }

        if (frames_.empty())
            return false;

        if (remap_joints)
            joint_count_ = out_joints_name.size();

        compute_velocity();
        duration_ = static_cast<double>(frames_.size()) * input_dt_;
        return true;
    }

    bool is_loaded() const
    {
        return !frames_.empty();
    }

    double duration() const
    {
        return duration_;
    }

    double dt() const
    {
        return input_dt_;
    }

    std::size_t joint_count() const
    {
        return joint_count_;
    }

    std::size_t size() const
    {
        return frames_.size();
    }

    void reset_alignment(const std::vector<float> &base_quaternion,
                         const std::vector<float> &waist_angles)
    {
        alignment_ = {1.0F, 0.0F, 0.0F, 0.0F};
        if (frames_.empty())
            return;

        const auto robot_torso =
            math::normalize(apply_waist_angles(base_quaternion, waist_angles));

        const auto robot_yaw = math::euler_to_quat({0.0F, 0.0F, math::yaw_from_quat(robot_torso)});
        const auto reference_yaw =
            math::euler_to_quat({0.0F, 0.0F,
                                 math::yaw_from_quat(frames_.front().anchor_quaternion)});
        alignment_ = math::normalize(math::quat_multiply(robot_yaw, math::conjugate(reference_yaw)));
    }

    // Interpolate motion command at the given time in seconds.
    MotionFrame sample(double time) const
    {
        if (frames_.empty())
            return {};
        if (frames_.size() == 1U)
        {
            auto out = frames_.front();
            out.anchor_quaternion =
                math::normalize(math::quat_multiply(alignment_, out.anchor_quaternion));
            return out;
        }

        time = std::clamp(time, 0.0, duration_);
       
        const double phase = duration_ > 0.0 ? time / duration_ : 0.0;
        const double index = phase * static_cast<double>(frames_.size() - 1U);
        const std::size_t i0 =
            std::min(static_cast<std::size_t>(std::floor(index)), frames_.size() - 1U);
        const std::size_t i1 = std::min(i0 + 1U, frames_.size() - 1U);
        const float alpha = static_cast<float>(index - static_cast<double>(i0));
        auto out = interpolate(frames_[i0], frames_[i1], alpha, time);
    
        out.anchor_quaternion = compute_anchor(
            out.root_quaternion, out.joint_position, waist_joint_indices_);
        out.anchor_quaternion =
            math::normalize(math::quat_multiply(alignment_, out.anchor_quaternion));
        return out;
    }

    const std::vector<MotionFrame> &frames() const
    {
        return frames_;
    }

private:
    static std::vector<float> apply_waist_angles(std::vector<float> torso,
                                                 const std::vector<float> &waist_angles)
    {
        if (!waist_angles.empty())
            torso = math::quat_multiply(torso, math::euler_to_quat({0.0F, 0.0F, waist_angles[0]}));
        if (waist_angles.size() > 1U)
            torso = math::quat_multiply(torso, math::euler_to_quat({waist_angles[1], 0.0F, 0.0F}));
        if (waist_angles.size() > 2U)
            torso = math::quat_multiply(torso, math::euler_to_quat({0.0F, waist_angles[2], 0.0F}));
        return torso;
    }

    static std::vector<float> compute_anchor(const std::vector<float> &root_quaternion,
                                             const std::vector<float> &joint_position,
                                             const std::vector<int> &waist_joint_indices)
    {
        std::vector<float> waist_angles;
        waist_angles.reserve(std::min<std::size_t>(waist_joint_indices.size(), 3U));
        for (std::size_t axis = 0; axis < waist_joint_indices.size() && axis < 3U; ++axis)
        {
            const int index = waist_joint_indices[axis];
            if (index < 0 || static_cast<std::size_t>(index) >= joint_position.size())
                waist_angles.push_back(0.0F);
            else
                waist_angles.push_back(joint_position[static_cast<std::size_t>(index)]);
        }
        return math::normalize(apply_waist_angles(root_quaternion, waist_angles));
    }

    static bool parse_row(const std::string &line, std::vector<float> &row)
    {
        row.clear();
        std::stringstream ss(line);
        std::string value;
        while (std::getline(ss, value, ','))
        {
            if (value.empty())
                continue;
            try
            {
                std::size_t idx = 0;
                const float parsed = std::stof(value, &idx);
                if (idx != value.size())
                    return false;
                row.push_back(parsed);
            }
            catch (...)
            {
                return false;
            }
        }
        return !row.empty();
    }

    static MotionFrame interpolate(const MotionFrame &a,
                                   const MotionFrame &b,
                                   float alpha,
                                   double time)
    {
        MotionFrame out;
        out.time = time;
        out.root_position = math::lerp(a.root_position, b.root_position, alpha);
        out.root_quaternion = math::slerp(a.root_quaternion, b.root_quaternion, alpha);
        out.joint_position = math::lerp(a.joint_position, b.joint_position, alpha);
        out.joint_velocity = math::lerp(a.joint_velocity, b.joint_velocity, alpha);
        out.anchor_quaternion = math::slerp(a.anchor_quaternion, b.anchor_quaternion, alpha);
        return out;
    }

    void compute_velocity()
    {
        if (frames_.size() < 2U || joint_count_ == 0)
            return;

        for (std::size_t i = 0; i + 1U < frames_.size(); ++i)
        {
            frames_[i].joint_velocity.resize(joint_count_);
            for (std::size_t j = 0; j < joint_count_; ++j)
            {
                frames_[i].joint_velocity[j] =
                    (frames_[i + 1U].joint_position[j] - frames_[i].joint_position[j]) /
                    static_cast<float>(input_dt_);
            }
        }

        frames_.back().joint_velocity = frames_[frames_.size() - 2U].joint_velocity;
    }

    std::vector<MotionFrame> frames_;
    double duration_{0.0};
    double input_dt_{0.0};
    std::size_t joint_count_{0};
    std::string order_{"xyzw"};
    std::vector<int> waist_joint_indices_;
    std::vector<float> alignment_{1.0F, 0.0F, 0.0F, 0.0F};
};

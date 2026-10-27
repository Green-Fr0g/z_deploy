#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <yaml-cpp/yaml.h>

#include <z_deploy/config_types.hpp>
#include <z_deploy/utils/utils.hpp>

class ConfigError : public std::runtime_error
{
public:
    using std::runtime_error::runtime_error;
};

class PolicyConfigLoader
{
public:
    // Load base.yaml; if policy_id is non-empty, also load policy config.yaml (RL mode).
    // A failed policy load leaves the last valid base configuration available
    // to the FSM, which can then return to Passive safely.
    bool load(const std::filesystem::path &policy_root,
              std::string robot_name,
              std::string policy_id = {},
              bool overlay = true)
    {
        const auto previous_robot = robot_;
        const auto previous_policy = policy_;
        const auto previous_policy_dir = policy_dir_;
        const auto previous_robot_loaded = robot_loaded_;
        const auto previous_policy_loaded = policy_loaded_;

        reset();
        try
        {
            const auto base_path = policy_root / robot_name / "base.yaml";

            robot_ = parse_robot(first_section(YAML::LoadFile(base_path)));
            robot_.robot_name = robot_name;
            robot_loaded_ = true;
            std::cout << "Loaded robot config from: " << base_path << std::endl;

            if (!policy_id.empty())
            {
                policy_dir_ = policy_root / robot_name / policy_id;
                const auto config_path = policy_dir_ / "config.yaml";
                const YAML::Node policy_node = first_section(YAML::LoadFile(config_path));
                policy_ = parse_policy(policy_node, policy_id);
                policy_loaded_ = true;
                std::cout << "Loaded policy config from: " << config_path << std::endl;
                if (overlay)
                    robot_ = overlay_robot(robot_, policy_node);

                const std::string motion_yaml = scalar<std::string>(policy_node, "motion_config");
                if (!motion_yaml.empty())
                {
                    const auto motion_path = policy_dir_ / motion_yaml;
                    if (!load_motion_meta(motion_path, policy_.motion_meta))
                        throw ConfigError("Failed to load motion config: " + motion_path.string());
                    std::cout << "Loaded motion config from: " << motion_path << std::endl;
                }
            }

            return true;
        }
        catch (const std::exception &e)
        {
            std::cerr << "Failed to load config for " << robot_name;
            if (!policy_id.empty())
                std::cerr << "/" << policy_id;
            std::cerr << ": " << e.what() << '\n';
            robot_ = previous_robot;
            policy_ = previous_policy;
            policy_dir_ = previous_policy_dir;
            robot_loaded_ = previous_robot_loaded;
            policy_loaded_ = previous_policy_loaded;
            return false;
        }
    }

    bool is_robot_loaded() const
    {
        return robot_loaded_;
    }

    bool is_policy_loaded() const
    {
        return policy_loaded_;
    }

    bool is_motion_loaded() const
    {
        return !policy_.motion_meta.csv_file.empty();
    }

    const RobotConfig &robot() const
    {
        return robot_;
    }

    const PolicyConfig &policy() const
    {
        return policy_;
    }

    const MotionMeta &motion() const
    {
        return policy_.motion_meta;
    }

    const std::filesystem::path &policy_dir() const
    {
        return policy_dir_;
    }

    std::filesystem::path motion_csv_path() const
    {
        if (!is_motion_loaded())
            return {};
        return policy_dir_ / policy_.motion_meta.csv_file;
    }

    void reset()
    {
        robot_ = RobotConfig{};
        policy_ = PolicyConfig{};
        policy_dir_.clear();
        robot_loaded_ = false;
        policy_loaded_ = false;
    }

private:
    RobotConfig robot_;
    PolicyConfig policy_;
    std::filesystem::path policy_dir_;
    bool robot_loaded_{false};
    bool policy_loaded_{false};

    static YAML::Node first_section(const YAML::Node &root)
    {
        if (!root || !root.IsMap() || root.size() == 0)
            throw ConfigError("Configuration root must contain one mapping section");
        return root.begin()->second;
    }

    template <typename T>
    static T scalar(const YAML::Node &node, const char *key, const T &fallback = T{})
    {
        return node[key] ? node[key].as<T>() : fallback;
    }

    // Sequence, or a single scalar which becomes a one-element vector {value}.
    template <typename T>
    static std::vector<T> sequence(const YAML::Node &node, const char *key)
    {
        std::vector<T> result;
        if (!node || !node[key])
            return result;
        const auto &value = node[key];
        if (value.IsScalar())
            return {value.as<T>()};
        if (value.IsSequence())
        {
            for (const auto &item : value)
                result.push_back(item.as<T>());
        }
        return result;
    }

    static std::array<float, 6> parse_command_scale(const YAML::Node &node)
    {
        auto result = PolicyConfig{}.command_scale;
        if (!node)
            return result;
        if (!node.IsScalar() && !node.IsSequence())
            throw ConfigError("commands_scale must be a number or a sequence of 1, 3 or 6 numbers");

        const auto values = node.IsScalar() ? std::vector<float>{node.as<float>()}
                                            : node.as<std::vector<float>>();
        if (values.empty())
            return result;
        if (values.size() != 1 && values.size() != 3 && values.size() != 6)
            throw ConfigError("commands_scale requires 1, 3 or 6 values: +x, -x, +y, -y, +yaw, -yaw");
        for (const float value : values)
            if (!std::isfinite(value) || value < 0.0F)
                throw ConfigError("commands_scale values must be finite and nonnegative");

        if (values.size() == 1)
            result.fill(values.front());
        else if (values.size() == 3)
            for (std::size_t i = 0; i < 3; ++i)
                result[2 * i] = result[2 * i + 1] = values[i];
        else
            std::copy(values.begin(), values.end(), result.begin());
        return result;
    }

    static bool load_motion_meta(const std::filesystem::path &yaml_path, MotionMeta &out)
    {
        out = MotionMeta{};
        if (yaml_path.empty())
            return false;

        try
        {
            const YAML::Node root = YAML::LoadFile(yaml_path.string());
            if (!root || !root["motion_data"])
                return false;

            const YAML::Node node = root["motion_data"];
            if (node["robot_name"])
                out.robot_name = node["robot_name"].as<std::string>();
            if (node["csv_file"])
                out.csv_file = node["csv_file"].as<std::string>();
            if (node["csv_fps"])
                out.csv_fps = node["csv_fps"].as<double>();
            if (node["root_quat_order"])
                out.root_quat_order = node["root_quat_order"].as<std::string>();
            for (const auto &item : node["csv_joint_names"])
                out.csv_joint_names.push_back(item.as<std::string>());

            return !out.csv_file.empty();
        }
        catch (...)
        {
            out = MotionMeta{};
            return false;
        }
    }

    static RobotConfig parse_robot(const YAML::Node &node)
    {
        RobotConfig result;
        // sim_dt
        result.control_dt = scalar<double>(node, "dt");
        // joints
        result.dof = scalar<int>(node, "num_of_dofs");
        result.joint_names = sequence<std::string>(node, "joint_names");
        result.joint_mapping = sequence<int>(node, "joint_mapping");
        result.waist_joint_index = sequence<int>(node, "waist_joint_indices");
        result.wheel_joint_index = sequence<int>(node, "wheel_joint_indices");
        // base
        result.default_dof_pos = sequence<float>(node, "default_dof_pos");
        result.fixed_kp = sequence<float>(node, "fixed_kp");
        result.fixed_kd = sequence<float>(node, "fixed_kd");
        // limit
        result.torque_limits = sequence<float>(node, "torque_limits");
        result.joint_position_lower = sequence<float>(node, "joint_position_lower");
        result.joint_position_upper = sequence<float>(node, "joint_position_upper");
        result.joint_velocity_limits = sequence<float>(node, "joint_velocity_limits");

        const auto num = static_cast<std::size_t>(std::max(result.dof, 0));
        utils::fill_identity(result.joint_mapping, num);

        // Expand arrays to match dof, using defaults if necessary.
        utils::expand_to_num(result.fixed_kp, num);
        utils::expand_to_num(result.fixed_kd, num);
        utils::expand_to_num(result.torque_limits, num, 100.0F);
        utils::expand_to_num(result.joint_velocity_limits, num, 100.0F);
        utils::expand_to_num(result.joint_position_lower, num, -100.0F);
        utils::expand_to_num(result.joint_position_upper, num, 100.0F);
        return result;
    }

    static PolicyConfig parse_policy(const YAML::Node &node, const std::string &policy_id)
    {
        PolicyConfig result;
        // policy
        result.policy_name = scalar<std::string>(node, "policy_name", policy_id);
        result.model_name = scalar<std::string>(node, "model_name");
        result.decimation = scalar<int>(node, "decimation", 1);
        // observations
        result.observations = sequence<std::string>(node, "observations");
        result.observation_dims = sequence<int>(node, "observations_dims_list");
        result.observation_history = sequence<int>(node, "observations_history");
        if (result.observation_history.empty())
            result.observation_history = {0};
        result.observation_history_priority =
            scalar<std::string>(node, "observations_history_priority", "time");
        result.input_dim = scalar<int>(
            node, "num_policy_observations", scalar<int>(node, "num_observations"));
        result.clip_obs = scalar<float>(node, "clip_obs");
        // rl_base
        result.default_dof_pos = sequence<float>(node, "default_dof_pos");
        result.kp = sequence<float>(node, "rl_kp");
        result.kd = sequence<float>(node, "rl_kd");
        result.dof = scalar<int>(node, "num_of_dofs", static_cast<int>(result.default_dof_pos.size()));
        result.joint_mapping = sequence<int>(node, "joint_mapping");
        result.waist_joint_index = sequence<int>(node, "waist_joint_indices");
        result.wheel_joint_index = sequence<int>(node, "wheel_joint_indices");
        // scales
        result.ang_vel_scale = sequence<float>(node, "ang_vel_scale");
        result.lin_vel_scale = sequence<float>(node, "lin_vel_scale");
        result.dof_pos_scale = sequence<float>(node, "dof_pos_scale");
        result.dof_vel_scale = sequence<float>(node, "dof_vel_scale");
        result.command_scale = parse_command_scale(node["commands_scale"]);
        result.action_scale = sequence<float>(node, "action_scale");
        result.motion_meta.csv_file = scalar<std::string>(node, "motion_file");

        const auto clip = node["clip_actions"];
        result.action_lower = sequence<float>(clip, "lower");
        result.action_upper = sequence<float>(clip, "upper");
        if (result.action_lower.empty())
            result.action_lower = sequence<float>(node, "clip_actions_lower");
        if (result.action_upper.empty())
            result.action_upper = sequence<float>(node, "clip_actions_upper");

        const auto num = static_cast<std::size_t>(std::max(result.dof, 0));
        utils::fill_identity(result.joint_mapping, num);

        if (result.observation_dims.empty())
        {
            for (const auto &name : result.observations)
            {
                if (name == "whole_body_tracking/motion_command")
                    result.observation_dims.push_back(static_cast<int>(2U * num));
                else if (name == "whole_body_tracking/motion_anchor_ori_b")
                    result.observation_dims.push_back(6);
                else if (name == "ang_vel" || name == "base_gravity_vec" ||
                         name == "gravity_vec" || name == "commands")
                    result.observation_dims.push_back(3);
                else if (name == "dof_pos" || name == "dof_vel" || name == "actions")
                    result.observation_dims.push_back(static_cast<int>(num));
                else if (name == "RoboMimic_Deploy/phase")
                    result.observation_dims.push_back(1);
                else
                    result.observation_dims.push_back(0);
            }
        }

        // Expand arrays to match dof, using defaults if necessary.
        utils::expand_to_num(result.kp, num);
        utils::expand_to_num(result.kd, num);
        utils::expand_to_num(result.ang_vel_scale, 3, 1.0F);
        utils::expand_to_num(result.lin_vel_scale, 3, 1.0F);
        utils::expand_to_num(result.dof_pos_scale, num);
        utils::expand_to_num(result.dof_vel_scale, num);
        utils::expand_to_num(result.action_scale, num);
        utils::expand_to_num(result.action_lower, num, -100.0F);
        utils::expand_to_num(result.action_upper, num, 100.0F);

        return result;
    }

    // Policy vectors paired with joint_mapping are policy-ordered. RobotConfig
    // stores hardware order, so those vectors are scattered onto motor indices.
    static RobotConfig overlay_robot(RobotConfig base, const YAML::Node &node)
    {
        if (node["dt"])
            base.control_dt = node["dt"].as<double>();

        const auto num = static_cast<std::size_t>(std::max(base.dof, 0));

        std::vector<int> policy_mapping;
        const bool has_mapping = static_cast<bool>(node["joint_mapping"]);
        if (has_mapping)
        {
            policy_mapping = sequence<int>(node, "joint_mapping");
            utils::fill_identity(policy_mapping, num);
        }

        auto overlay_float =
            [&](const char *key, std::vector<float> &field, std::optional<float> fill = std::nullopt)
        {
            if (!node[key])
                return;
            auto values = sequence<float>(node, key);
            utils::expand_to_num(values, num, fill);
            field = has_mapping ? utils::scatter(values, policy_mapping, num) : std::move(values);
        };

        auto overlay_names = [&](const char *key, std::vector<std::string> &field)
        {
            if (!node[key])
                return;
            auto values = sequence<std::string>(node, key);
            field = has_mapping ? utils::scatter(values, policy_mapping, num) : std::move(values);
        };

        overlay_float("fixed_kp", base.fixed_kp);
        overlay_float("fixed_kd", base.fixed_kd);
        overlay_float("torque_limits", base.torque_limits, 100.0F);
        overlay_float("joint_velocity_limits", base.joint_velocity_limits, 100.0F);
        overlay_float("joint_position_lower", base.joint_position_lower, -100.0F);
        overlay_float("joint_position_upper", base.joint_position_upper, 100.0F);
        overlay_float("default_dof_pos", base.default_dof_pos);
        overlay_names("joint_names", base.joint_names);

        return base;
    }
};

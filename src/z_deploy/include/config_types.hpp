#pragma once

#include <array>
#include <string>
#include <vector>

struct RobotConfig
{
    std::string robot_name;
    //control_dt
    double control_dt{0.0};
    //dofs
    std::vector<std::string> joint_names;
    //default_dof_pos
    int dof{0};
    std::vector<float> default_dof_pos;
    std::vector<float> fixed_kp;
    std::vector<float> fixed_kd;
    std::vector<int> joint_mapping;
    std::vector<int> waist_joint_index;
    std::vector<int> wheel_joint_index;
    //limit_joint_position_lower
    std::vector<float> torque_limits;
    std::vector<float> joint_position_lower;
    std::vector<float> joint_position_upper;
    std::vector<float> joint_velocity_limits;
};

struct MotionMeta
{
    std::string robot_name;
    std::string csv_file;
    double csv_fps{60.0};
    std::string root_quat_order{"xyzw"};
    std::vector<std::string> csv_joint_names;
};

struct PolicyConfig
{
    std::string policy_name;
    std::string model_name;
    MotionMeta motion_meta;
    // Observation period = robot.control_dt; inference period = control_dt * decimation.
    int decimation{1};
    //observations
    std::vector<std::string> observations;
    std::vector<int> observation_dims;
    std::vector<int> observation_history; // Offsets in observation ticks (0 = latest).
    std::string observation_history_priority{"time"};
    int input_dim{0};
    //default_dof_pos
    int dof{0};
    std::vector<float> default_dof_pos;
    std::vector<float> kp;
    std::vector<float> kd;
    std::vector<int> joint_mapping;
    std::vector<int> waist_joint_index;
    std::vector<int> wheel_joint_index;
    //scales
    std::vector<float> ang_vel_scale{1.0F};
    std::vector<float> lin_vel_scale{1.0F};
    std::vector<float> dof_pos_scale{1.0F};
    std::vector<float> dof_vel_scale{1.0F};
    // Nonnegative magnitudes: +x, -x, +y, -y, +yaw, -yaw.
    std::array<float, 6> command_scale{1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F};
    std::vector<float> action_scale;
    //limit
    std::vector<float> action_lower;
    std::vector<float> action_upper;
    float clip_obs{0.0F};
};

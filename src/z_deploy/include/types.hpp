#pragma once

#include <cstddef>
#include <vector>

//robot state&command
struct RobotState
{
    int sequence{0};
    std::vector<float> joint_pos;
    std::vector<float> joint_vel;
    std::vector<float> joint_torque;
    std::vector<float> base_quat{1.0F, 0.0F, 0.0F, 0.0F}; // wxyz
    std::vector<float> base_euler{0.0F, 0.0F, 0.0F};       // roll, pitch, yaw
    std::vector<float> base_ang_vel{0.0F, 0.0F, 0.0F};      // xyz
    std::vector<float> base_lin_acc{0.0F, 0.0F, 0.0F};      // xyz

    void initialize(std::size_t dof)
    {
        joint_pos.resize(dof, 0.0F);
        joint_vel.resize(dof, 0.0F);
        joint_torque.resize(dof, 0.0F);
    }

};

struct RobotCommand
{
    int sequence{0};
    std::vector<float> position;
    std::vector<float> velocity;
    std::vector<float> torque;
    std::vector<float> kp;
    std::vector<float> kd;

    void initialize(std::size_t dof)
    {
        position.resize(dof, 0.0F);
        velocity.resize(dof, 0.0F);
        torque.resize(dof, 0.0F);
        kp.resize(dof, 0.0F);
        kd.resize(dof, 0.0F);
    }
};

struct VelocityCmd
{
    float x{0.0F};   // forward
    float y{0.0F};   // lateral
    float yaw{0.0F}; // yaw rate
};

// Semantic keyboard keys. Raw axes/buttons are the virtual-joystick view of
// the same events (see hardware/keyboard.hpp); this enum is the key-level view.
enum class Keyboard
{
    None = 0,
    A, B, C, D, E, F, G, H, I, J, K, L, M,
    N, O, P, Q, R, S, T, U, V, W, X, Y, Z,
    Num0, Num1, Num2, Num3, Num4, Num5, Num6, Num7, Num8, Num9,
    Space, Enter, Escape,
    Up, Down, Left, Right
};

//motion frame
struct MotionFrame
{
    double time{0.0};
    std::vector<float> root_position{0.0F, 0.0F, 0.0F};
    std::vector<float> root_quaternion{1.0F, 0.0F, 0.0F, 0.0F};
    std::vector<float> joint_position;
    std::vector<float> joint_velocity;
    std::vector<float> anchor_quaternion{1.0F, 0.0F, 0.0F, 0.0F};
};


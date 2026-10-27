#pragma once

#include <atomic>
#include <cstddef>
#include <utility>
#include <vector>

#include <config_types.hpp>
#include <input_types.hpp>
#include <types.hpp>
#include <utils/triple_buffer.hpp>

// Robot hardware abstraction: flat state readback + a single command entry
// point. Implementations (sim / real robot) only provide the transport hooks;
// the FSM and policy always talk to this interface.
class HardwareBase
{
public:
    explicit HardwareBase(const RobotConfig &config)
        : dof_(static_cast<std::size_t>(config.dof > 0 ? config.dof : 0))
    {
        state_staging_.initialize(dof_);
        state_buffer_.publish(state_staging_);
    }

    virtual ~HardwareBase() = default;

    HardwareBase(const HardwareBase &) = delete;
    HardwareBase &operator=(const HardwareBase &) = delete;

    void start()
    {
        if (running_)
            return;
        start_backend();
        running_ = true;
    }

    void stop()
    {
        if (!running_)
            return;
        stop_backend();
        running_ = false;
    }

    std::vector<float> get_joint_position() const
    {
        return state_buffer_.read().joint_pos;
    }

    std::vector<float> get_joint_velocity() const
    {
        return state_buffer_.read().joint_vel;
    }

    std::vector<float> get_joint_torque() const
    {
        return state_buffer_.read().joint_torque;
    }

    std::vector<float> get_base_quaternion() const
    {
        return state_buffer_.read().base_quat;
    }

    std::vector<float> get_base_angular_velocity() const
    {
        return state_buffer_.read().base_ang_vel;
    }

    std::vector<float> get_base_linear_acceleration() const
    {
        return state_buffer_.read().base_lin_acc;
    }

    // Unified user input is supplied by each backend.
    virtual joybutton get_joy_button() const = 0;
    virtual joyaxis get_joy_axis() const = 0;

    // Dropped until the first state frame arrives, so the robot is never driven
    // from stale zero data.
    void set_joint_command(const RobotCommand &command)
    {
        if (!state_received_.load())
            return;
        send_command_impl(command);
    }

protected:
    virtual void start_backend() = 0;
    virtual void stop_backend() = 0;
    virtual void send_command_impl(const RobotCommand &command) = 0;

    // Backend callbacks publish state through this helper.
    void publish_state(const RobotState &state)
    {
        state_buffer_.publish(state);
        state_received_.store(true);
    }

    std::size_t dof_{0};
    RobotState state_staging_;
    TripleBuffer<RobotState> state_buffer_;
    std::atomic<bool> state_received_{false};

private:
    bool running_{false};
};

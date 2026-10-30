#pragma once

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/b2/motion_switcher/motion_switcher_client.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include <hardware/hardware_base.hpp>
#include <hardware/hardware_register.hpp>
#include <hardware/input.hpp>
#include <tui/display.hpp>

// G1 29-DOF low-level backend. RobotState/RobotCommand use base.yaml order;
// sdk_joint_indices_ maps that order to Unitree's sparse motor IDs.
class HardwareG1 : public HardwareBase
{
public:
    explicit HardwareG1(const RobotConfig &config)
        : HardwareG1(config,
                     {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14,
                      15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28})
    {
    }

    ~HardwareG1() override { stop(); }

    joybutton get_joy_button() const override
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        return state_fresh() ? input_.get_snapshot().current : joybutton::None;
    }

    joyaxis get_joy_axis() const override
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        return state_fresh() ? input_.get_snapshot().axis : joyaxis{};
    }

protected:
    HardwareG1(const RobotConfig &config, std::vector<std::size_t> sdk_joint_indices)
        : HardwareBase(config), network_interface_(network_interface_from_environment()),
          sdk_joint_indices_(std::move(sdk_joint_indices)),
          input_([this](JoySnapshot &snapshot)
                 {
                     std::lock_guard<std::mutex> lock(state_mutex_);
                     if (!state_fresh())
                         return false;
                     snapshot = remote_input_;
                     return true;
                 })
    {
        if (dof_ != sdk_joint_indices_.size())
            throw std::invalid_argument("G1 real hardware joint count does not match base.yaml");
    }

    void start_backend() override
    {
        state_received_.store(false);
        accepting_state_.store(true);
        try
        {
            unitree::robot::ChannelFactory::Instance()->Init(0, network_interface_);
            publisher_ = std::make_unique<unitree::robot::ChannelPublisher<LowCommand>>("rt/lowcmd");
            publisher_->InitChannel();
            subscriber_ = std::make_unique<unitree::robot::ChannelSubscriber<LowState>>("rt/lowstate");
            subscriber_->InitChannel([this](const void *message) { on_state(message); }, 1);

            // Do not take over control until a CRC-checked state has arrived.
            {
                std::unique_lock<std::mutex> lock(state_mutex_);
                if (!state_cv_.wait_for(lock, std::chrono::seconds(3),
                                        [this]() { return state_received_.load(); }))
                    throw std::runtime_error("Timed out waiting for G1 rt/lowstate on " + network_interface_);
            }
            release_motion_control();
            input_.start();
            control_ready_ = true;
            tui::message("INFO", "G1 real hardware ready on " + network_interface_ +
                                     " (" + std::to_string(dof_) + " joints)");
        }
        catch (...)
        {
            stop_backend();
            throw;
        }
    }

    void stop_backend() override
    {
        input_.stop();
        {
            std::lock_guard<std::mutex> lock(command_mutex_);
            // Finish with damping on the mapped motors; unused SDK slots stay disabled.
            if (publisher_ && control_ready_)
            {
                try
                {
                    write_command(nullptr);
                }
                catch (const std::exception &e)
                {
                    tui::message("WARN", std::string("G1 final damping command failed: ") + e.what());
                }
            }
            control_ready_ = false;
            if (publisher_)
                publisher_->CloseChannel();
            publisher_.reset();
        }
        accepting_state_.store(false);
        if (subscriber_)
            subscriber_->CloseChannel();
        subscriber_.reset();
        state_received_.store(false);
        std::lock_guard<std::mutex> lock(state_mutex_);
        remote_input_ = {};
        last_state_time_ = {};
    }

    void send_command_impl(const RobotCommand &command) override
    {
        std::lock_guard<std::mutex> lock(command_mutex_);
        if (!publisher_ || !control_ready_)
            return;

        bool fresh;
        {
            std::lock_guard<std::mutex> state_lock(state_mutex_);
            fresh = state_fresh();
        }
        const bool valid = valid_command(command);
        if ((!fresh || !valid) && !command_warning_)
            tui::message("WARN", "G1 state is stale or command is invalid; sending damping");
        command_warning_ = !fresh || !valid;
        write_command(fresh && valid ? &command : nullptr);
    }

private:
    using LowState = unitree_hg::msg::dds_::LowState_;
    using LowCommand = unitree_hg::msg::dds_::LowCmd_;

    static std::string network_interface_from_environment()
    {
        const char *network_interface = std::getenv("Z_NETIF");
        if (network_interface == nullptr || *network_interface == '\0')
            throw std::invalid_argument("Z_NETIF is not set; export Z_NETIF=<network_interface> before starting real hardware");
        return network_interface;
    }

    // Same polynomial and word order as SDK2's G1 low-level examples.
    template <typename Message>
    static std::uint32_t crc32(const Message &message)
    {
        static_assert(sizeof(Message) % sizeof(std::uint32_t) == 0, "Invalid SDK message layout");
        const auto *bytes = reinterpret_cast<const unsigned char *>(&message);
        std::uint32_t crc = 0xFFFFFFFFU;
        for (std::size_t i = 0; i < sizeof(Message) / sizeof(std::uint32_t) - 1; ++i)
        {
            std::uint32_t word;
            std::memcpy(&word, bytes + i * sizeof(word), sizeof(word));
            for (std::uint32_t bit = 0x80000000U; bit != 0; bit >>= 1)
            {
                const bool high = (crc & 0x80000000U) != 0;
                crc <<= 1;
                if (high)
                    crc ^= 0x04C11DB7U;
                if (word & bit)
                    crc ^= 0x04C11DB7U;
            }
        }
        return crc;
    }

    void release_motion_control()
    {
        unitree::robot::b2::MotionSwitcherClient switcher;
        switcher.SetTimeout(5.0F);
        switcher.Init();
        for (int attempt = 0; attempt < 4; ++attempt)
        {
            std::string form, name;
            if (switcher.CheckMode(form, name) != 0)
                throw std::runtime_error("Cannot query G1 motion control mode");
            if (name.empty())
                return;
            if (attempt == 3 || switcher.ReleaseMode() != 0)
                throw std::runtime_error("Cannot release G1 motion control mode: " + name);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    // Called with state_mutex_ held.
    bool state_fresh() const
    {
        return state_received_.load() &&
               std::chrono::steady_clock::now() - last_state_time_ < std::chrono::milliseconds(100);
    }

    void on_state(const void *message)
    {
        if (!accepting_state_.load() || message == nullptr)
            return;
        const auto &state = *static_cast<const LowState *>(message);
        if (state.crc() != crc32(state))
            return;

        std::lock_guard<std::mutex> lock(state_mutex_);
        for (std::size_t i = 0; i < dof_; ++i)
        {
            const auto &motor = state.motor_state()[sdk_joint_indices_[i]];
            state_staging_.joint_pos[i] = motor.q();
            state_staging_.joint_vel[i] = motor.dq();
            state_staging_.joint_torque[i] = motor.tau_est();
        }
        const auto &imu = state.imu_state();
        std::copy(imu.quaternion().begin(), imu.quaternion().end(), state_staging_.base_quat.begin()); // wxyz
        std::copy(imu.gyroscope().begin(), imu.gyroscope().end(), state_staging_.base_ang_vel.begin());
        std::copy(imu.accelerometer().begin(), imu.accelerometer().end(), state_staging_.base_lin_acc.begin());
        std::copy(imu.rpy().begin(), imu.rpy().end(), state_staging_.base_euler.begin());
        ++state_staging_.sequence;
        mode_machine_.store(state.mode_machine());
        decode_remote(state.wireless_remote());
        last_state_time_ = std::chrono::steady_clock::now();
        publish_state(state_staging_);
        state_cv_.notify_all();
    }

    void decode_remote(const std::array<std::uint8_t, 40> &remote)
    {
        // SDK2 gamepad.hpp: head[2], buttons[2], lx, rx, ry, L2, ly.
        const std::uint16_t buttons = static_cast<std::uint16_t>(remote[2]) |
                                      (static_cast<std::uint16_t>(remote[3]) << 8);
        const auto axis = [&](std::size_t offset)
        {
            float value;
            std::memcpy(&value, remote.data() + offset, sizeof(value));
            if (!std::isfinite(value) || std::fabs(value) < 0.05F)
                return 0.0F;
            return std::clamp(value, -1.0F, 1.0F);
        };
        // Unitree ly/ry are positive forward/up; joyaxis uses positive down.
        remote_input_.axis = {axis(4), -axis(20), axis(8), -axis(12)};

        const bool rb = (buttons & (1U << 0)) != 0;
        const bool lb = (buttons & (1U << 1)) != 0;
        const std::array<joybutton, 8> plain{
            joybutton::A, joybutton::B, joybutton::X, joybutton::Y,
            joybutton::DPadUp, joybutton::DPadRight, joybutton::DPadDown, joybutton::DPadLeft};
        const std::array<joybutton, 8> with_lb{
            joybutton::LB_A, joybutton::LB_B, joybutton::LB_X, joybutton::LB_Y,
            joybutton::LB_DPadUp, joybutton::LB_DPadRight, joybutton::LB_DPadDown, joybutton::LB_DPadLeft};
        const std::array<joybutton, 8> with_rb{
            joybutton::RB_A, joybutton::RB_B, joybutton::RB_X, joybutton::RB_Y,
            joybutton::RB_DPadUp, joybutton::RB_DPadRight, joybutton::RB_DPadDown, joybutton::RB_DPadLeft};
        joybutton button = rb ? joybutton::RB : (lb ? joybutton::LB : joybutton::None);
        for (std::size_t i = plain.size(); i-- > 0;)
            if (buttons & (1U << (8 + i)))
            {
                button = rb ? with_rb[i] : (lb ? with_lb[i] : plain[i]);
                break;
            }
        if (lb && rb)
            button = joybutton::LB_RB;
        remote_input_.previous = remote_input_.current;
        remote_input_.current = button;
    }

    bool valid_command(const RobotCommand &command) const
    {
        if (command.position.size() != dof_ || command.velocity.size() != dof_ ||
            command.torque.size() != dof_ || command.kp.size() != dof_ || command.kd.size() != dof_)
            return false;
        for (std::size_t i = 0; i < dof_; ++i)
            if (!std::isfinite(command.position[i]) || !std::isfinite(command.velocity[i]) ||
                !std::isfinite(command.torque[i]) || !std::isfinite(command.kp[i]) ||
                !std::isfinite(command.kd[i]) || command.kp[i] < 0.0F || command.kd[i] < 0.0F)
                return false;
        return true;
    }

    void write_command(const RobotCommand *command)
    {
        LowCommand message{};
        message.mode_pr() = 0; // PR: joint pitch/roll, not parallel ankle A/B motors.
        message.mode_machine() = mode_machine_.load();
        for (std::size_t i = 0; i < dof_; ++i)
        {
            auto &motor = message.motor_cmd()[sdk_joint_indices_[i]];
            motor.mode() = 1;
            motor.q() = command ? command->position[i] : 0.0F;
            motor.dq() = command ? command->velocity[i] : 0.0F;
            motor.tau() = command ? command->torque[i] : 0.0F;
            motor.kp() = command ? command->kp[i] : 0.0F;
            motor.kd() = command ? command->kd[i] : 5.0F;
        }
        message.crc() = crc32(message);
        if (!publisher_->Write(message))
            throw std::runtime_error("Failed to publish G1 rt/lowcmd");
    }

    std::string network_interface_;
    const std::vector<std::size_t> sdk_joint_indices_;
    std::unique_ptr<unitree::robot::ChannelPublisher<LowCommand>> publisher_;
    std::unique_ptr<unitree::robot::ChannelSubscriber<LowState>> subscriber_;
    std::atomic<std::uint8_t> mode_machine_{0};
    std::atomic<bool> accepting_state_{false};
    mutable std::mutex state_mutex_;
    std::condition_variable state_cv_;
    JoySnapshot remote_input_{};
    std::chrono::steady_clock::time_point last_state_time_{};
    std::mutex command_mutex_;
    bool control_ready_{false};
    bool command_warning_{false};
    InputHandler input_;
};

Z_REGISTER_HARDWARE("real", "g1", HardwareG1)

#pragma once

#include <algorithm>
#include <array>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <numeric>
#include <utility>
#include <vector>

#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include "hardware_base.hpp"

class G1HardwareReceiver : public HardwareReceiver
{
public:
    explicit G1HardwareReceiver(const RobotConfig &config)
        : G1HardwareReceiver(config, joint_indices())
    {
    }

    ~G1HardwareReceiver() override
    {
        if (state_sub_)
            state_sub_->CloseChannel();
    }

    void poll() override {}
    bool has_remote() const override { return true; }

protected:
    G1HardwareReceiver(const RobotConfig &config, std::vector<std::size_t> indices)
        : HardwareReceiver(config), sdk_joint_indices_(std::move(indices))
    {
        if (sdk_joint_indices_.size() != snapshot_.state.joint_pos.size())
            throw std::invalid_argument("SDK joint count does not match base.yaml");
        const char *network_interface = std::getenv("Z_NETIF");
        if (network_interface == nullptr || *network_interface == '\0')
            throw std::invalid_argument("Set Z_NETIF=<network_interface> for real mode");
        unitree::robot::ChannelFactory::Instance()->Init(0, network_interface);
        state_sub_ = std::make_unique<unitree::robot::ChannelSubscriber<LowState>>("rt/lowstate");
        state_sub_->InitChannel([this](const void *message) { on_state(message); }, 1);
    }

private:
    using LowState = unitree_hg::msg::dds_::LowState_;

    static std::vector<std::size_t> joint_indices()
    {
        std::vector<std::size_t> indices(29);
        std::iota(indices.begin(), indices.end(), 0);
        return indices;
    }

    static std::uint32_t crc32(const LowState &message)
    {
        static_assert(sizeof(LowState) % sizeof(std::uint32_t) == 0, "Invalid SDK message layout");
        const auto *bytes = reinterpret_cast<const unsigned char *>(&message);
        std::uint32_t crc = 0xFFFFFFFFU;
        for (std::size_t i = 0; i < sizeof(LowState) / sizeof(std::uint32_t) - 1; ++i)
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

    // Called with mutex_ held. Keep raw axes (SDK positive-up convention).
    void update_remote(const std::array<std::uint8_t, 40> &bytes)
    {
        auto &remote = snapshot_.remote;
        const auto buttons = static_cast<std::uint16_t>(
            static_cast<std::uint16_t>(bytes[2]) | (static_cast<std::uint16_t>(bytes[3]) << 8));
        if (buttons != remote.buttons)
        {
            const auto pressed = static_cast<std::uint16_t>(buttons & ~remote.buttons);
            const auto released = static_cast<std::uint16_t>(remote.buttons & ~buttons);
            if (pressed != 0)
                remote.last_pressed = pressed;
            if (released != 0)
                remote.last_released = released;
            ++remote.changes;
        }
        remote.buttons = buttons;
        const auto axis = [&](std::size_t offset)
        {
            float value;
            std::memcpy(&value, bytes.data() + offset, sizeof(value));
            return value;
        };
        remote.lx = axis(4);
        remote.rx = axis(8);
        remote.ry = axis(12);
        remote.l2 = axis(16);
        remote.ly = axis(20);
        snapshot_.remote_updated = Clock::now();
    }

    void on_state(const void *message)
    {
        if (message == nullptr)
            return;
        const auto &data = *static_cast<const LowState *>(message);
        if (data.crc() != crc32(data))
            return;
        std::lock_guard<std::mutex> lock(mutex_);
        auto &state = snapshot_.state;
        for (std::size_t i = 0; i < sdk_joint_indices_.size(); ++i)
        {
            const auto &motor = data.motor_state()[sdk_joint_indices_[i]];
            state.joint_pos[i] = motor.q();
            state.joint_vel[i] = motor.dq();
            state.joint_torque[i] = motor.tau_est();
        }
        const auto &imu = data.imu_state();
        std::copy(imu.quaternion().begin(), imu.quaternion().end(), state.base_quat.begin());
        std::copy(imu.rpy().begin(), imu.rpy().end(), state.base_euler.begin());
        std::copy(imu.gyroscope().begin(), imu.gyroscope().end(), state.base_ang_vel.begin());
        std::copy(imu.accelerometer().begin(), imu.accelerometer().end(), state.base_lin_acc.begin());
        snapshot_.joints_updated = snapshot_.imu_updated = Clock::now();
        update_remote(data.wireless_remote());
    }

    std::vector<std::size_t> sdk_joint_indices_;
    // Close callbacks before destroying receiver state.
    std::unique_ptr<unitree::robot::ChannelSubscriber<LowState>> state_sub_;
};

#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <stdexcept>
#include <string>

#include <config_types.hpp>
#include <types.hpp>

// Receive-only interface, independent of the deployment hardware lifecycle.
class HardwareReceiver
{
public:
    using Clock = std::chrono::steady_clock;

    struct RemoteState
    {
        std::uint16_t buttons{0};
        std::uint16_t last_pressed{0};
        std::uint16_t last_released{0};
        std::uint64_t changes{0};
        float lx{0.0F};
        float ly{0.0F};
        float rx{0.0F};
        float ry{0.0F};
        float l2{0.0F};
    };

    struct Snapshot
    {
        RobotState state;
        RemoteState remote;
        Clock::time_point joints_updated{};
        Clock::time_point imu_updated{};
        Clock::time_point remote_updated{};

        bool ready() const
        {
            return joints_updated != Clock::time_point{} && imu_updated != Clock::time_point{};
        }
    };

    virtual ~HardwareReceiver() = default;
    virtual void poll() = 0;
    virtual bool has_remote() const { return false; }

    Snapshot get_snapshot() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return snapshot_;
    }

    static std::string button_names(std::uint16_t buttons)
    {
        // SDK2 gamepad.hpp bit order. L1/R1 are LB/RB in deployment controls.
        constexpr std::array<const char *, 16> names{
            "RB", "LB", "Start", "Select", "R2", "L2", "F1", "F2",
            "A", "B", "X", "Y", "DPadUp", "DPadRight", "DPadDown", "DPadLeft"};
        std::string result;
        for (std::size_t i = 0; i < names.size(); ++i)
            if (buttons & (1U << i))
            {
                if (!result.empty())
                    result += '+';
                result += names[i];
            }
        return result.empty() ? "None" : result;
    }

protected:
    explicit HardwareReceiver(const RobotConfig &config)
    {
        if (config.dof <= 0)
            throw std::invalid_argument("Invalid receiver joint count");
        snapshot_.state.initialize(static_cast<std::size_t>(config.dof));
    }

    mutable std::mutex mutex_;
    Snapshot snapshot_;
};

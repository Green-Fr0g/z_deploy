#pragma once

#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <memory>
#include <mutex>
#include <string>
#include <utility>

#include <joystick.hh>

#include <input_types.hpp>
#include <utils/thread_timer.hpp>

// Linux Xbox layout, sampled through library/joystick.
class JoystickHandler
{
public:
    explicit JoystickHandler(std::string device = "/dev/input/js0")
        : device_(std::move(device))
    {
    }

    ~JoystickHandler()
    {
        stop();
    }

    void start()
    {
        if (running_)
            return;
        next_connect_ = {};
        timer_.start(0.01, [this]() { handle(); });
        running_ = true;
    }

    void stop()
    {
        timer_.stop();
        joystick_.reset();
        reset_state();
        running_ = false;
    }

    JoySnapshot get_snapshot() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return snapshot_;
    }

private:
    void handle()
    {
        if (!joystick_)
        {
            const auto now = std::chrono::steady_clock::now();
            if (now < next_connect_)
                return;
            next_connect_ = now + std::chrono::seconds(1);
            auto device = std::make_unique<Joystick>(device_);
            if (!device->isFound())
                return;
            joystick_ = std::move(device);
        }

        JoystickEvent event{};
        for (int count = 0; count < 256 && joystick_->sample(&event); ++count)
            on_event(event);

        // No new events (EAGAIN) leaves the current held state intact.
        if (!joystick_->isFound())
        {
            joystick_.reset();
            reset_state();
        }
    }

    void on_event(JoystickEvent event)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (event.isButton() && event.number < buttons_.size())
            buttons_[event.number] = event.value != 0;
        else if (event.isAxis())
        {
            const float value = std::clamp(event.value / 32767.0F, -1.0F, 1.0F);
            // Axes 2/5 are triggers; axes 6/7 form the D-pad.
            switch (event.number)
            {
            case 0: snapshot_.axis.L_X = value; break;
            case 1: snapshot_.axis.L_Y = value; break;
            case 3: snapshot_.axis.R_X = value; break;
            case 4: snapshot_.axis.R_Y = value; break;
            case 6: dpad_x_ = value; break;
            case 7: dpad_y_ = value; break;
            default: return;
            }
        }
        else
            return;

        const joybutton button = decode_button();
        if (button != snapshot_.current)
        {
            snapshot_.previous = snapshot_.current;
            snapshot_.current = button;
        }
    }

    // Called with mutex_ held. Shoulder buttons modify face/stick/D-pad keys.
    joybutton decode_button() const
    {
        if (buttons_[4] && buttons_[5])
            return joybutton::LB_RB;

        constexpr std::array<joybutton, 10> plain{
            joybutton::A, joybutton::B, joybutton::X, joybutton::Y,
            joybutton::LStick, joybutton::RStick, joybutton::DPadUp,
            joybutton::DPadDown, joybutton::DPadLeft, joybutton::DPadRight};
        constexpr std::array<joybutton, 10> lb{
            joybutton::LB_A, joybutton::LB_B, joybutton::LB_X, joybutton::LB_Y,
            joybutton::LB_LStick, joybutton::LB_RStick, joybutton::LB_DPadUp,
            joybutton::LB_DPadDown, joybutton::LB_DPadLeft, joybutton::LB_DPadRight};
        constexpr std::array<joybutton, 10> rb{
            joybutton::RB_A, joybutton::RB_B, joybutton::RB_X, joybutton::RB_Y,
            joybutton::RB_LStick, joybutton::RB_RStick, joybutton::RB_DPadUp,
            joybutton::RB_DPadDown, joybutton::RB_DPadLeft, joybutton::RB_DPadRight};
        const std::array<bool, 10> held{
            buttons_[0], buttons_[1], buttons_[2], buttons_[3], buttons_[9], buttons_[10],
            dpad_y_ < -0.5F, dpad_y_ > 0.5F, dpad_x_ < -0.5F, dpad_x_ > 0.5F};
        for (std::size_t i = held.size(); i-- > 0;)
            if (held[i])
                return buttons_[5] ? rb[i] : (buttons_[4] ? lb[i] : plain[i]);
        return buttons_[5] ? joybutton::RB : (buttons_[4] ? joybutton::LB : joybutton::None);
    }

    void reset_state()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        buttons_.fill(false);
        dpad_x_ = dpad_y_ = 0.0F;
        snapshot_ = {};
    }

    std::string device_;
    std::unique_ptr<Joystick> joystick_;
    std::chrono::steady_clock::time_point next_connect_{};
    mutable std::mutex mutex_;
    std::array<bool, 11> buttons_{};
    float dpad_x_{0.0F};
    float dpad_y_{0.0F};
    JoySnapshot snapshot_{};
    bool running_{false};
    ThreadTimer timer_;
};

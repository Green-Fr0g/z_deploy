#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <mutex>
#include <string>
#include <utility>

#include <hardware/joystick.hpp>
#include <hardware/keyboard.hpp>
#include <input_types.hpp>
#include <types.hpp>
#include <utils/thread_timer.hpp>

// Both devices share one state. An active joystick owns buttons and axes
// together; centered, released sticks/buttons allow keyboard control.
class InputHandler
{
public:
    explicit InputHandler(std::string joystick_device = "/dev/input/js0")
        : joystick_(std::move(joystick_device))
    {
    }

    ~InputHandler()
    {
        stop();
    }

    void start()
    {
        if (running_)
            return;
        keyboard_.start();
        joystick_.start();
        timer_.start(0.01, [this]() { handle(); });
        running_ = true;
    }

    void stop()
    {
        timer_.stop();
        keyboard_.stop();
        joystick_.stop();
        std::lock_guard<std::mutex> lock(mutex_);
        keyboard_state_ = {};
        button_until_ = {};
        snapshot_ = {};
        running_ = false;
    }

    JoySnapshot get_snapshot() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return snapshot_;
    }

private:
    static constexpr float deadzone_ = 0.05F;
    static constexpr float keyboard_step_ = 0.1F;

    static float filter_axis(float value)
    {
        return std::fabs(value) > deadzone_ ? value : 0.0F;
    }

    static joybutton key_button(Keyboard key)
    {
        // Policy shortcuts: 1-4 = LB + up/down/left/right,
        //                   5-8 = RB + up/down/left/right.
        switch (key)
        {
        case Keyboard::Num0: return joybutton::A;
        case Keyboard::P: return joybutton::LB_X;
        case Keyboard::Num1: return joybutton::LB_DPadUp;
        case Keyboard::Num2: return joybutton::LB_DPadDown;
        case Keyboard::Num3: return joybutton::LB_DPadLeft;
        case Keyboard::Num4: return joybutton::LB_DPadRight;
        case Keyboard::Num5: return joybutton::RB_DPadUp;
        case Keyboard::Num6: return joybutton::RB_DPadDown;
        case Keyboard::Num7: return joybutton::RB_DPadLeft;
        case Keyboard::Num8: return joybutton::RB_DPadRight;
        case Keyboard::R: return joybutton::RB_Y;
        case Keyboard::Enter: return joybutton::RB_X;
        case Keyboard::Up: return joybutton::DPadUp;
        case Keyboard::Down: return joybutton::DPadDown;
        case Keyboard::Left: return joybutton::DPadLeft;
        case Keyboard::Right: return joybutton::DPadRight;
        default: return joybutton::None;
        }
    }

    // Terminal input has no release events: sticks accumulate in 0.1 steps,
    // Space centers them, and mapped buttons are visible for 200 ms.
    void apply_key(Keyboard key, std::chrono::steady_clock::time_point now)
    {
        auto &axis = keyboard_state_.axis;
        switch (key)
        {
        case Keyboard::W: step_axis(axis.L_Y, -keyboard_step_); break;
        case Keyboard::S: step_axis(axis.L_Y, keyboard_step_); break;
        case Keyboard::A: step_axis(axis.L_X, -keyboard_step_); break;
        case Keyboard::D: step_axis(axis.L_X, keyboard_step_); break;
        case Keyboard::Q: step_axis(axis.R_X, -keyboard_step_); break;
        case Keyboard::E: step_axis(axis.R_X, keyboard_step_); break;
        case Keyboard::Space: axis = {}; break;
        default: break;
        }
        keyboard_state_.current = key_button(key);
        button_until_ = now + std::chrono::milliseconds(200);
    }

    static void step_axis(float &axis, float step)
    {
        axis = std::clamp(axis + step, -1.0F, 1.0F);
        if (std::fabs(axis) < 1e-6F)
            axis = 0.0F;
    }

    void handle()
    {
        auto joystick = joystick_.get_snapshot();
        auto &axis = joystick.axis;
        axis.L_X = filter_axis(axis.L_X);
        axis.L_Y = filter_axis(axis.L_Y);
        axis.R_X = filter_axis(axis.R_X);
        axis.R_Y = filter_axis(axis.R_Y);
        const bool joystick_active = joystick.current != joybutton::None ||
                                     axis.L_X != 0.0F || axis.L_Y != 0.0F ||
                                     axis.R_X != 0.0F || axis.R_Y != 0.0F;

        // Drain keyboard input even while the joystick owns control, so keys
        // pressed during an override cannot be replayed on handover.
        const Keyboard key = keyboard_.consume_key();
        keyboard_.consume_raw();
        const auto now = std::chrono::steady_clock::now();
        std::lock_guard<std::mutex> lock(mutex_);
        if (joystick_active || now >= button_until_)
            keyboard_state_.current = joybutton::None;
        if (!joystick_active && key != Keyboard::None)
            apply_key(key, now);

        const auto &selected = joystick_active ? joystick : keyboard_state_;
        if (selected.current != snapshot_.current)
        {
            snapshot_.previous = snapshot_.current;
            snapshot_.current = selected.current;
        }
        snapshot_.axis = selected.axis;
    }

    KeyboardHandler keyboard_;
    JoystickHandler joystick_;
    mutable std::mutex mutex_;
    JoySnapshot keyboard_state_{};
    JoySnapshot snapshot_{};
    std::chrono::steady_clock::time_point button_until_{};
    bool running_{false};
    ThreadTimer timer_;
};

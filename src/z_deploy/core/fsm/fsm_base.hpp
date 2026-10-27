#pragma once

#include <algorithm>
#include <cstddef>
#include <string>
#include <utility>

#include <config/config_loader.hpp>
#include <hardware/hardware_base.hpp>
#include <input_types.hpp>
#include <policy/policy_manager.hpp>
#include <types.hpp>

class FsmBase
{
public:
    explicit FsmBase(std::string name) : name_(std::move(name)) {}
    virtual ~FsmBase() = default;

    virtual void enter() {}
    virtual void run() = 0;
    virtual void exit() {}
    virtual std::string check_change() { return name_; }

    const std::string &name() const { return name_; }

protected:
    std::string name_;
};

// Shared dependencies only. Each robot defines its own states and transitions.
// Policy configuration can be reloaded on entry; read it through config.
struct FsmContext
{
    HardwareBase &robot;
    PolicyConfigLoader &config;
    PolicyManager &policy;
};

// Common state resources and raw button edge detection, without state mappings.
class FsmState : public FsmBase
{
public:
    FsmState(std::string name, FsmContext &ctx)
        : FsmBase(std::move(name)), ctx_(ctx)
    {
        cmd_.initialize(static_cast<std::size_t>(std::max(ctx_.config.robot().dof, 0)));
    }

    void enter() override { skip_pending_buttons(); }

protected:
    void poll_input()
    {
        const auto button = ctx_.robot.get_joy_button();
        current_button_ = button != last_button_ ? button : joybutton::None;
        last_button_ = button;
    }

    // Ignore a button already held when this state is entered.
    void skip_pending_buttons()
    {
        last_button_ = ctx_.robot.get_joy_button();
        current_button_ = joybutton::None;
    }

    FsmContext &ctx_;
    RobotCommand cmd_;
    joybutton current_button_{joybutton::None};

private:
    joybutton last_button_{joybutton::None};
};

#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include <fsm/fsm_register.hpp>
#include <tui/display.hpp>
#include <utils/math.hpp>

namespace adam
{

// Starts in Disabled. Keyboard: P -> Passive, 0 -> GetUp, 1 -> RLLocomotion.
//   Passive:  A / RB+A -> GetUp, Y (or LB+X) -> Disabled
//   Disabled: Y (or LB+X) -> Passive
//   GetUp:    Y / LB+X -> Passive; once standing is complete RB/LB+DPadUp
//             starts RLLocomotion.
//   RLLocomotion: Y / LB+X -> Passive, RB+A -> GetUp.

class Passive : public FsmState
{
public:
    explicit Passive(FsmContext &ctx) : FsmState("Passive", ctx) {}

    void enter() override
    {
        FsmState::enter();
        tui::message("INFO", "Enter Adam Passive");
    }

    void exit() override { tui::message("INFO", "Exit Adam Passive"); }

    void run() override
    {
        for (std::size_t i = 0; i < cmd_.position.size(); ++i)
        {
            cmd_.position[i] = 0.0F;
            cmd_.velocity[i] = cmd_.torque[i] = 0.0F;
            cmd_.kp[i] = 0.0F;
            cmd_.kd[i] = 5.0F;
        }
        ctx_.robot.set_joint_command(cmd_);
    }

    std::string check_change() override
    {
        poll_input();
        if (current_button_ == joybutton::A || current_button_ == joybutton::RB_A)
            return "GetUp";
        if (current_button_ == joybutton::Y || current_button_ == joybutton::LB_X)
            return "Disabled";
        return name_;
    }
};

class Disabled : public FsmState
{
public:
    explicit Disabled(FsmContext &ctx) : FsmState("Disabled", ctx) {}

    void enter() override
    {
        FsmState::enter();
        tui::message("INFO", "Enter Adam Disabled");
    }

    void exit() override { tui::message("INFO", "Exit Adam Disabled"); }

    void run() override
    {
        const auto position = ctx_.robot.get_joint_position();
        for (std::size_t i = 0; i < cmd_.position.size(); ++i)
        {
            cmd_.position[i] = i < position.size() ? position[i] : 0.0F;
            cmd_.velocity[i] = cmd_.torque[i] = 0.0F;
            cmd_.kp[i] = cmd_.kd[i] = 0.0F;
        }
        ctx_.robot.set_joint_command(cmd_);
    }

    std::string check_change() override
    {
        poll_input();
        if (current_button_ == joybutton::Y || current_button_ == joybutton::LB_X)
            return "Passive";
        return name_;
    }
};

class GetUp : public FsmState
{
public:
    explicit GetUp(FsmContext &ctx) : FsmState("GetUp", ctx) {}

    void enter() override
    {
        FsmState::enter();
        tui::message("INFO", "Enter Adam GetUp");
        percent_ = 0.0F;
        progress_completed_ = false;
        start_ = ctx_.robot.get_joint_position();
        target_ = ctx_.config.robot().default_dof_pos;
    }

    void exit() override { tui::message("INFO", "Exit Adam GetUp"); }

    void run() override
    {
        const auto &robot = ctx_.config.robot();
        const double dt = robot.control_dt > 0.0 ? robot.control_dt : 0.005;
        percent_ = std::min(1.0F, percent_ + static_cast<float>(dt / duration_));
        for (std::size_t i = 0; i < cmd_.position.size(); ++i)
        {
            const float from = i < start_.size() ? start_[i] : 0.0F;
            const float to = i < target_.size() ? target_[i] : 0.0F;
            cmd_.position[i] = math::lerp(from, to, percent_);
            cmd_.velocity[i] = cmd_.torque[i] = 0.0F;
            cmd_.kp[i] = i < robot.fixed_kp.size() ? robot.fixed_kp[i] : 0.0F;
            cmd_.kd[i] = i < robot.fixed_kd.size() ? robot.fixed_kd[i] : 0.0F;
        }
        ctx_.robot.set_joint_command(cmd_);
        if (!progress_completed_)
            progress_completed_ = tui::progress(percent_, 1.0, "Getting up");
    }

    std::string check_change() override
    {
        poll_input();
        if (current_button_ == joybutton::Y || current_button_ == joybutton::LB_X)
            return "Passive";
        if (percent_ < 1.0F)
            return name_; // Consume and ignore policy requests until standing is complete.
        if (current_button_ == joybutton::RB_DPadUp ||
            current_button_ == joybutton::LB_DPadUp)
            return "RLLocomotion";
        return name_;
    }

private:
    static constexpr float duration_ = 2.0F;
    float percent_{0.0F};
    bool progress_completed_{false};
    std::vector<float> start_;
    std::vector<float> target_;
};

class RLLocomotion : public FsmState
{
public:
    explicit RLLocomotion(FsmContext &ctx) : FsmState("RLLocomotion", ctx) {}

    void enter() override
    {
        FsmState::enter();
        tui::message("INFO", "Enter Adam RLLocomotion");
        if (ctx_.policy.start_policy("locomotion"))
        {
            const auto command = ctx_.policy.velocity_command();
            tui::velocity(command.x, command.y, command.yaw, true);
        }
    }

    void run() override
    {
        ctx_.policy.run_policy();
        if (ctx_.policy.policy_active())
        {
            const auto command = ctx_.policy.velocity_command();
            tui::velocity(command.x, command.y, command.yaw);
        }
    }

    void exit() override
    {
        tui::message("INFO", "Exit Adam RLLocomotion");
        ctx_.policy.stop_policy();
    }

    std::string check_change() override
    {
        poll_input();
        if (!ctx_.policy.policy_active())
            return "Passive";
        if (current_button_ == joybutton::Y ||
            current_button_ == joybutton::LB_X)
            return "Passive";
        if (current_button_ == joybutton::RB_A)
            return "GetUp";
        return name_;
    }
};

class Factory : public FsmFactory
{
public:
    std::shared_ptr<FsmBase> create_state(FsmContext &ctx, const std::string &name) override
    {
        if (name == "Passive")
            return std::make_shared<Passive>(ctx);
        if (name == "Disabled")
            return std::make_shared<Disabled>(ctx);
        if (name == "GetUp")
            return std::make_shared<GetUp>(ctx);
        if (name == "RLLocomotion")
            return std::make_shared<RLLocomotion>(ctx);
        return nullptr;
    }

    std::string type() const override { return "adam"; }

    std::vector<std::string> supported_states() const override
    {
        return {"Passive", "Disabled", "GetUp", "RLLocomotion"};
    }

    std::string initial_state() const override { return "Disabled"; }
};

} // namespace adam

Z_REGISTER_FSM(adam::Factory)

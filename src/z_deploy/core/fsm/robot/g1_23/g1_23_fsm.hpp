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

namespace g1_23
{

// Key map, aligned with the G1 FSM:
//   Passive:  A / RB+A -> GetUp, Y (or LB+X) -> Disabled
//   Disabled: Y (or LB+X) -> Passive
//   GetUp:    Y / LB+X -> Passive; once standing is complete LB/RB+DPadUp ->
//             RLLocomotion, LB/RB+DPadDown -> Mimic, LB/RB+DPadRight -> Dance
//   RLLocomotion / Dance / Mimic: Y / LB+X -> Passive, RB+A -> GetUp, and the DPad
//             switches between locomotion, Dance (dance1_subject2) and Mimic
//             (mimic). A finished motion returns to RLLocomotion.

class Passive : public FsmState
{
public:
    explicit Passive(FsmContext &ctx) : FsmState("Passive", ctx) {}

    void enter() override
    {
        FsmState::enter();
        tui::message("INFO", "Enter G1-23 Passive");
    }

    void exit() override { tui::message("INFO", "Exit G1-23 Passive"); }

    void run() override
    {
        for (std::size_t i = 0; i < cmd_.position.size(); ++i)
        {
            cmd_.position[i] = 0.0F;
            cmd_.velocity[i] = cmd_.torque[i] = 0.0F;
            cmd_.kp[i] = 0.0F;
            cmd_.kd[i] = 8.0F;
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
        tui::message("INFO", "Enter G1-23 Disabled");
    }

    void exit() override { tui::message("INFO", "Exit G1-23 Disabled"); }

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
        tui::message("INFO", "Enter G1-23 GetUp");
        percent_ = 0.0F;
        progress_completed_ = false;
        start_ = ctx_.robot.get_joint_position();
        target_ = ctx_.config.robot().default_dof_pos;
    }

    void exit() override { tui::message("INFO", "Exit G1-23 GetUp"); }

    void run() override
    {
        const auto &robot = ctx_.config.robot();
        const double dt = robot.control_dt > 0.0 ? robot.control_dt : 0.002;
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
        if (current_button_ == joybutton::RB_DPadRight ||
            current_button_ == joybutton::LB_DPadRight)
            return "Dance";
        if (current_button_ == joybutton::RB_DPadDown ||
            current_button_ == joybutton::LB_DPadDown)
            return "Mimic";
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
        tui::message("INFO", "Enter G1-23 RLLocomotion");
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
        tui::message("INFO", "Exit G1-23 RLLocomotion");
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
        if (current_button_ == joybutton::RB_DPadRight ||
            current_button_ == joybutton::LB_DPadRight)
            return "Dance";
        if (current_button_ == joybutton::RB_DPadDown ||
            current_button_ == joybutton::LB_DPadDown)
            return "Mimic";
        return name_;
    }
};

class Dance : public FsmState
{
public:
    explicit Dance(FsmContext &ctx) : FsmState("Dance", ctx) {}

    void enter() override
    {
        FsmState::enter();
        tui::message("INFO", "Enter G1-23 Dance");
        progress_ = -1.0;
        if (ctx_.policy.start_policy("dance1_subject2"))
            update_progress();
    }

    void run() override
    {
        ctx_.policy.run_policy();
        if (ctx_.policy.policy_active())
            update_progress();
    }

    void exit() override
    {
        tui::message("INFO", "Exit G1-23 Dance");
        ctx_.policy.stop_policy();
    }

    std::string check_change() override
    {
        poll_input();
        if (!ctx_.policy.policy_active())
            return "Passive";
        if (ctx_.policy.motion_finished())
            return "RLLocomotion";
        if (current_button_ == joybutton::Y ||
            current_button_ == joybutton::LB_X)
            return "Passive";
        if (current_button_ == joybutton::RB_A)
            return "GetUp";
        if (current_button_ == joybutton::RB_DPadUp ||
            current_button_ == joybutton::LB_DPadUp)
            return "RLLocomotion";
        if (current_button_ == joybutton::RB_DPadDown ||
            current_button_ == joybutton::LB_DPadDown)
            return "Mimic";
        return name_;
    }

private:
    void update_progress()
    {
        const auto progress = ctx_.policy.motion_progress();
        if (progress >= 1.0 || progress_ < 0.0 || progress - progress_ >= 0.01)
        {
            tui::progress(progress, 1.0, "Dance");
            progress_ = progress;
        }
    }

    double progress_{-1.0};
};

class Mimic : public FsmState
{
public:
    explicit Mimic(FsmContext &ctx) : FsmState("Mimic", ctx) {}

    void enter() override
    {
        FsmState::enter();
        tui::message("INFO", "Enter G1-23 Mimic");
        progress_ = -1.0;
        if (ctx_.policy.start_policy("mimic"))
            update_progress();
    }

    void run() override
    {
        ctx_.policy.run_policy();
        if (ctx_.policy.policy_active())
            update_progress();
    }

    void exit() override
    {
        tui::message("INFO", "Exit G1-23 Mimic");
        ctx_.policy.stop_policy();
    }

    std::string check_change() override
    {
        poll_input();
        if (!ctx_.policy.policy_active())
            return "Passive";
        if (ctx_.policy.motion_finished())
            return "RLLocomotion";
        if (current_button_ == joybutton::Y ||
            current_button_ == joybutton::LB_X)
            return "Passive";
        if (current_button_ == joybutton::RB_A)
            return "GetUp";
        if (current_button_ == joybutton::RB_DPadUp ||
            current_button_ == joybutton::LB_DPadUp)
            return "RLLocomotion";
        if (current_button_ == joybutton::RB_DPadRight ||
            current_button_ == joybutton::LB_DPadRight)
            return "Dance";
        return name_;
    }

private:
    void update_progress()
    {
        const auto progress = ctx_.policy.motion_progress();
        if (progress >= 1.0 || progress_ < 0.0 || progress - progress_ >= 0.01)
        {
            tui::progress(progress, 1.0, "Mimic");
            progress_ = progress;
        }
    }

    double progress_{-1.0};
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
        if (name == "Dance")
            return std::make_shared<Dance>(ctx);
        if (name == "Mimic")
            return std::make_shared<Mimic>(ctx);
        return nullptr;
    }

    std::string type() const override { return "g1_23"; }

    std::vector<std::string> supported_states() const override
    {
        return {"Passive", "Disabled", "GetUp", "RLLocomotion", "Dance", "Mimic"};
    }

    std::string initial_state() const override { return "Disabled"; }
};

} // namespace g1_23

Z_REGISTER_FSM(g1_23::Factory)

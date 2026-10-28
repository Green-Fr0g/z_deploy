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

namespace g1
{

    // Policy shortcuts (keyboard 1-8):
    // LB + Up/Down/Left/Right -> RLLocomotion / AMP / GangnamStyle / Dance102.
    // RB + Up/Down/Left/Right -> AmpZlab / Mimic / ZLocomotion / reserved.
    // Passive: A / RB+A -> GetUp, Y -> Disabled; Disabled: Y -> Passive.
    // GetUp: LB+X -> Passive. RLLocomotion / Dance102: Y -> Passive, RB+A -> GetUp.
    // AMP: LB+X -> Passive, Y -> GetUp.

    class Passive : public FsmState
    {
    public:
        explicit Passive(FsmContext &ctx) : FsmState("Passive", ctx) {}

        void enter() override
        {
            FsmState::enter();
            tui::message("INFO", "Enter G1 Passive");
        }

        void exit() override { tui::message("INFO", "Exit G1 Passive"); }

        void run() override
        {
            const auto position = ctx_.robot.get_joint_position();
            for (std::size_t i = 0; i < cmd_.position.size(); ++i)
            {
                cmd_.position[i] = i < position.size() ? position[i] : 0.0F;
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
            if (current_button_ == joybutton::Y)
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
            tui::message("INFO", "Enter G1 Disabled");
        }

        void exit() override { tui::message("INFO", "Exit G1 Disabled"); }

        void run() override
        {
            for (std::size_t i = 0; i < cmd_.position.size(); ++i)
            {
                cmd_.position[i] = 0.0F;
                cmd_.velocity[i] = cmd_.torque[i] = 0.0F;
                cmd_.kp[i] = cmd_.kd[i] = 0.0F;
            }
            ctx_.robot.set_joint_command(cmd_);
        }

        std::string check_change() override
        {
            poll_input();
            if (current_button_ == joybutton::Y)
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
            tui::message("INFO", "Enter G1 GetUp");
            percent_ = 0.0F;
            progress_completed_ = false;
            start_ = ctx_.robot.get_joint_position();
            target_ = ctx_.config.robot().default_dof_pos;
            const auto &robot = ctx_.config.robot();
            dt = robot.control_dt > 0.0 ? robot.control_dt : 0.005;
        }

        void exit() override { tui::message("INFO", "Exit G1 GetUp"); }

        void run() override
        {
            const auto &robot = ctx_.config.robot();
            
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
            if (current_button_ == joybutton::LB_X)
                return "Passive";
            if (percent_ < 1.0F)
                return name_; // Consume and ignore policy requests until standing is complete.
            if (current_button_ == joybutton::LB_DPadUp)
                return "RLLocomotion";
            if (current_button_ == joybutton::LB_DPadDown)
                return "AMP";
            if (current_button_ == joybutton::LB_DPadLeft)
                return "GangnamStyle";
            if (current_button_ == joybutton::LB_DPadRight)
                return "Dance102";
            if (current_button_ == joybutton::RB_DPadUp)
                return "AmpZlab";
            if (current_button_ == joybutton::RB_DPadDown)
                return "Mimic";
            if (current_button_ == joybutton::RB_DPadLeft)
                return "ZLocomotion";
            return name_;
        }

    private:
        static constexpr float duration_ = 2.0F;
        float percent_{0.0F};
        bool progress_completed_{false};
        std::vector<float> start_;
        std::vector<float> target_;
        double dt{0.005};
    };

    class RLLocomotion : public FsmState
    {
    public:
        explicit RLLocomotion(FsmContext &ctx) : FsmState("RLLocomotion", ctx) {}

        void enter() override
        {
            FsmState::enter();
            tui::message("INFO", "Enter G1 RLLocomotion");
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
            tui::message("INFO", "Exit G1 RLLocomotion");
            ctx_.policy.stop_policy();
        }

        std::string check_change() override
        {
            poll_input();
            if (!ctx_.policy.policy_active())
                return "Passive";
            if (current_button_ == joybutton::Y)
                return "Passive";
            if (current_button_ == joybutton::RB_A)
                return "GetUp";
            if (current_button_ == joybutton::LB_DPadUp)
                return "RLLocomotion";
            if (current_button_ == joybutton::LB_DPadDown)
                return "AMP";
            if (current_button_ == joybutton::LB_DPadLeft)
                return "GangnamStyle";
            if (current_button_ == joybutton::LB_DPadRight)
                return "Dance102";
            if (current_button_ == joybutton::RB_DPadUp)
                return "AmpZlab";
            if (current_button_ == joybutton::RB_DPadDown)
                return "Mimic";
            if (current_button_ == joybutton::RB_DPadLeft)
                return "ZLocomotion";
            return name_;
        }
    };

    class Dance102 : public FsmState
    {
    public:
        explicit Dance102(FsmContext &ctx) : FsmState("Dance102", ctx) {}

        void enter() override
        {
            FsmState::enter();
            tui::message("INFO", "Enter G1 Dance102");
            progress_ = -1.0;
            if (ctx_.policy.start_policy("dance_102"))
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
            tui::message("INFO", "Exit G1 Dance102");
            ctx_.policy.stop_policy();
        }

        std::string check_change() override
        {
            poll_input();
            if (!ctx_.policy.policy_active())
                return "Passive";
            if (ctx_.policy.motion_finished())
                return "RLLocomotion";
            if (current_button_ == joybutton::Y)
                return "Passive";
            if (current_button_ == joybutton::RB_A)
                return "GetUp";
            if (current_button_ == joybutton::LB_DPadUp)
                return "RLLocomotion";
            if (current_button_ == joybutton::LB_DPadDown)
                return "AMP";
            if (current_button_ == joybutton::LB_DPadLeft)
                return "GangnamStyle";
            if (current_button_ == joybutton::LB_DPadRight)
                return "Dance102";
            if (current_button_ == joybutton::RB_DPadUp)
                return "AmpZlab";
            if (current_button_ == joybutton::RB_DPadDown)
                return "Mimic";
            if (current_button_ == joybutton::RB_DPadLeft)
                return "ZLocomotion";
            return name_;
        }

    private:
        void update_progress()
        {
            const auto progress = ctx_.policy.motion_progress();
            if (progress >= 1.0 || progress_ < 0.0 || progress - progress_ >= 0.01)
            {
                tui::progress(progress, 1.0, "Dance 102");
                progress_ = progress;
            }
        }

        double progress_{-1.0};
    };

    class AmpLocomotion : public FsmState
    {
    public:
        explicit AmpLocomotion(FsmContext &ctx) : FsmState("AMP", ctx) {}

        void enter() override
        {
            FsmState::enter();
            tui::message("INFO", "Enter G1 AMP");
            if (ctx_.policy.start_policy("amp"))
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
            tui::message("INFO", "Exit G1 AMP");
            ctx_.policy.stop_policy();
        }

        std::string check_change() override
        {
            poll_input();
            if (!ctx_.policy.policy_active())
                return "Passive";
            if (current_button_ == joybutton::LB_X)
                return "Passive";
            if (current_button_ == joybutton::Y)
                return "GetUp";
            if (current_button_ == joybutton::LB_DPadUp)
                return "RLLocomotion";
            if (current_button_ == joybutton::LB_DPadDown)
                return "AMP";
            if (current_button_ == joybutton::LB_DPadLeft)
                return "GangnamStyle";
            if (current_button_ == joybutton::LB_DPadRight)
                return "Dance102";
            if (current_button_ == joybutton::RB_DPadUp)
                return "AmpZlab";
            if (current_button_ == joybutton::RB_DPadDown)
                return "Mimic";
            if (current_button_ == joybutton::RB_DPadLeft)
                return "ZLocomotion";
            return name_;
        }
    };

    class GangnamStyle : public FsmState
    {
    public:
        explicit GangnamStyle(FsmContext &ctx) : FsmState("GangnamStyle", ctx) {}

        void enter() override
        {
            FsmState::enter();
            tui::message("INFO", "Enter G1 GangnamStyle");
            progress_ = -1.0;
            if (ctx_.policy.start_policy("gangnam_style"))
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
            tui::message("INFO", "Exit G1 GangnamStyle");
            ctx_.policy.stop_policy();
        }

        std::string check_change() override
        {
            poll_input();
            if (!ctx_.policy.policy_active())
                return "Passive";
            if (ctx_.policy.motion_finished())
                return "RLLocomotion";
            if (current_button_ == joybutton::Y)
                return "Passive";
            if (current_button_ == joybutton::RB_A)
                return "GetUp";
            if (current_button_ == joybutton::LB_DPadUp)
                return "RLLocomotion";
            if (current_button_ == joybutton::LB_DPadDown)
                return "AMP";
            if (current_button_ == joybutton::LB_DPadLeft)
                return "GangnamStyle";
            if (current_button_ == joybutton::LB_DPadRight)
                return "Dance102";
            if (current_button_ == joybutton::RB_DPadUp)
                return "AmpZlab";
            if (current_button_ == joybutton::RB_DPadDown)
                return "Mimic";
            if (current_button_ == joybutton::RB_DPadLeft)
                return "ZLocomotion";
            return name_;
        }

    private:
        void update_progress()
        {
            const auto progress = ctx_.policy.motion_progress();
            if (progress >= 1.0 || progress_ < 0.0 || progress - progress_ >= 0.01)
            {
                tui::progress(progress, 1.0, "Gangnam Style");
                progress_ = progress;
            }
        }

        double progress_{-1.0};
    };

    class AmpZlab : public FsmState
    {
    public:
        explicit AmpZlab(FsmContext &ctx) : FsmState("AmpZlab", ctx) {}

        void enter() override
        {
            FsmState::enter();
            tui::message("INFO", "Enter G1 AmpZlab");
            if (ctx_.policy.start_policy("amp_zlab"))
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
            tui::message("INFO", "Exit G1 AmpZlab");
            ctx_.policy.stop_policy();
        }

        std::string check_change() override
        {
            poll_input();
            if (!ctx_.policy.policy_active())
                return "Passive";
            if (current_button_ == joybutton::LB_X)
                return "Passive";
            if (current_button_ == joybutton::Y)
                return "GetUp";
            if (current_button_ == joybutton::LB_DPadUp)
                return "RLLocomotion";
            if (current_button_ == joybutton::LB_DPadDown)
                return "AMP";
            if (current_button_ == joybutton::LB_DPadLeft)
                return "GangnamStyle";
            if (current_button_ == joybutton::LB_DPadRight)
                return "Dance102";
            if (current_button_ == joybutton::RB_DPadUp)
                return "AmpZlab";
            if (current_button_ == joybutton::RB_DPadDown)
                return "Mimic";
            if (current_button_ == joybutton::RB_DPadLeft)
                return "ZLocomotion";
            return name_;
        }
    };

    class Mimic : public FsmState
    {
    public:
        explicit Mimic(FsmContext &ctx) : FsmState("Mimic", ctx) {}

        void enter() override
        {
            FsmState::enter();
            tui::message("INFO", "Enter G1 Mimic");
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
            tui::message("INFO", "Exit G1 Mimic");
            ctx_.policy.stop_policy();
        }

        std::string check_change() override
        {
            poll_input();
            if (!ctx_.policy.policy_active())
                return "Passive";
            if (ctx_.policy.motion_finished())
                return "RLLocomotion";
            if (current_button_ == joybutton::Y)
                return "Passive";
            if (current_button_ == joybutton::RB_A)
                return "GetUp";
            if (current_button_ == joybutton::LB_DPadUp)
                return "RLLocomotion";
            if (current_button_ == joybutton::LB_DPadDown)
                return "AMP";
            if (current_button_ == joybutton::LB_DPadLeft)
                return "GangnamStyle";
            if (current_button_ == joybutton::LB_DPadRight)
                return "Dance102";
            if (current_button_ == joybutton::RB_DPadUp)
                return "AmpZlab";
            if (current_button_ == joybutton::RB_DPadDown)
                return "Mimic";
            if (current_button_ == joybutton::RB_DPadLeft)
                return "ZLocomotion";
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

    class ZLocomotion : public FsmState
    {
    public:
        explicit ZLocomotion(FsmContext &ctx) : FsmState("ZLocomotion", ctx) {}

        void enter() override
        {
            FsmState::enter();
            tui::message("INFO", "Enter G1 ZLocomotion");
            if (ctx_.policy.start_policy("zlocomotion"))
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
            tui::message("INFO", "Exit G1 ZLocomotion");
            ctx_.policy.stop_policy();
        }

        std::string check_change() override
        {
            poll_input();
            if (!ctx_.policy.policy_active())
                return "Passive";
            if (current_button_ == joybutton::Y)
                return "Passive";
            if (current_button_ == joybutton::RB_A)
                return "GetUp";
            if (current_button_ == joybutton::LB_DPadUp)
                return "RLLocomotion";
            if (current_button_ == joybutton::LB_DPadDown)
                return "AMP";
            if (current_button_ == joybutton::LB_DPadLeft)
                return "GangnamStyle";
            if (current_button_ == joybutton::LB_DPadRight)
                return "Dance102";
            if (current_button_ == joybutton::RB_DPadUp)
                return "AmpZlab";
            if (current_button_ == joybutton::RB_DPadDown)
                return "Mimic";
            if (current_button_ == joybutton::RB_DPadLeft)
                return "ZLocomotion";
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
            if (name == "Dance102")
                return std::make_shared<Dance102>(ctx);
            if (name == "AMP")
                return std::make_shared<AmpLocomotion>(ctx);
            if (name == "GangnamStyle")
                return std::make_shared<GangnamStyle>(ctx);
            if (name == "AmpZlab")
                return std::make_shared<AmpZlab>(ctx);
            if (name == "Mimic")
                return std::make_shared<Mimic>(ctx);
            if (name == "ZLocomotion")
                return std::make_shared<ZLocomotion>(ctx);
            return nullptr;
        }

        std::string type() const override { return "g1"; }

        std::vector<std::string> supported_states() const override
        {
            return {"Passive", "Disabled", "GetUp", "RLLocomotion", "AMP", "GangnamStyle",
                    "Dance102", "AmpZlab", "Mimic", "ZLocomotion"};
        }

        std::string initial_state() const override { return "Passive"; }
    };

} // namespace g1

Z_REGISTER_FSM(g1::Factory)

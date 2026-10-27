#pragma once

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <z_deploy/command/motion_command.hpp>
#include <z_deploy/config/config_loader.hpp>
#include <z_deploy/config_types.hpp>
#include <z_deploy/hardware/hardware_base.hpp>
#include <z_deploy/policy/observation/obs_collection.hpp>
#include <z_deploy/policy/runtime/policy_register.hpp>
#include <z_deploy/tui/display.hpp>
#include <z_deploy/utils/math.hpp>
#include <z_deploy/utils/thread_timer.hpp>
#include <z_deploy/utils/utils.hpp>

class PolicyManager
{
public:
    using ObsFn = std::function<std::vector<float>()>;

    PolicyManager(HardwareBase &hardware,
                  PolicyConfigLoader &config,
                  std::filesystem::path policy_root,
                  MotionCommand &motion)
        : hardware_(hardware)
        , config_(config)
        , policy_root_(std::move(policy_root))
        , motion_(motion)
    {
    }

    ~PolicyManager() { stop_policy(); }

    bool start_policy(const std::string &policy_id)
    {
        stop_policy();

        const auto robot_name = config_.robot().robot_name;
        if (!config_.load(policy_root_, robot_name, policy_id))
        {
            tui::message("ERROR", "Failed to load policy " + policy_id);
            return false;
        }

        motion_ = MotionCommand{};
        const auto &motion_meta = config_.motion();
        const auto &policy = config_.policy();
        const auto &robot = config_.robot();
        const double motion_fps = robot.control_dt > 0.0 && policy.decimation > 0
                                      ? 1.0 / (robot.control_dt * policy.decimation)
                                      : motion_meta.csv_fps;
        if (!motion_meta.csv_file.empty() &&
            !motion_.load(config_.motion_csv_path(),
                          static_cast<std::size_t>(policy.dof),
                          motion_fps,
                          motion_meta.root_quat_order,
                          motion_meta.csv_joint_names,
                          robot.joint_names,
                          robot.waist_joint_index))
        {
            tui::message("ERROR", "Failed to load motion for policy " + policy_id);
            return false;
        }

        const auto joint_pos = hardware_.get_joint_position();
        std::vector<float> waist;
        for (const auto index : robot.waist_joint_index)
        {
            const bool valid = index >= 0 && static_cast<std::size_t>(index) < joint_pos.size();
            waist.push_back(valid ? joint_pos[static_cast<std::size_t>(index)] : 0.0F);
        }
        motion_.reset_alignment(hardware_.get_base_quaternion(), waist);

        const auto model = config_.policy_dir() / policy.model_name;
        if (!configure(robot, policy, model))
        {
            tui::message("ERROR", "Failed to start policy " + policy_id);
            return false;
        }

        hold_cmd_ = RobotCommand{};
        hold_cmd_.initialize(static_cast<std::size_t>(robot.dof));
        const auto joints = hardware_.get_joint_position();
        for (std::size_t i = 0; i < hold_cmd_.position.size(); ++i)
        {
            hold_cmd_.position[i] = value_at(joints, i, 0.0F);
            hold_cmd_.kp[i] = value_at(robot.fixed_kp, i, 0.0F);
            hold_cmd_.kd[i] = value_at(robot.fixed_kd, i, 0.0F);
        }

        if (!start_inference())
        {
            tui::message("ERROR", "Failed to start policy " + policy_id);
            return false;
        }
        policy_active_ = true;
        return true;
    }

    // Called every control tick while a policy state is active.
    void run_policy()
    {
        if (policy_active_)
            hardware_.set_joint_command(actions_.has_output() ? target_command() : hold_cmd_);
    }

    void stop_policy()
    {
        policy_active_ = false;
        reset_inference();
    }

    bool policy_active() const
    {
        return policy_active_;
    }

    bool motion_finished() const
    {
        return policy_active_ && motion_.is_loaded() && motion_time_.load() >= motion_.duration();
    }

    double motion_progress() const
    {
        if (!policy_active_ || !motion_.is_loaded() || motion_.duration() <= 0.0)
            return 0.0;
        return std::clamp(motion_time_.load() / motion_.duration(), 0.0, 1.0);
    }

    // Latest sampled command, after directional scaling and observation clipping.
    VelocityCmd velocity_command() const { return observations_.velocity_command(); }

private:
    // One observation frame, owned exclusively by the sampling worker.
    struct ObservationSample
    {
        RobotState state;
        VelocityCmd command;
        std::vector<float> action;
    };

    // Latest complete observation and its consumption cursor share one lock.
    struct ObservationBuffer
    {
        void reset()
        {
            std::lock_guard<std::mutex> lock(mutex);
            values.clear();
            command = {};
            sequence  = 0;
            consumed_sequence = 0;
        }

        void publish(const std::vector<float> &observation, const VelocityCmd &velocity)
        {
            std::lock_guard<std::mutex> lock(mutex);
            values = observation;
            command = velocity;
            ++sequence;
        }

        bool consume(std::vector<float> &observation)
        {
            std::lock_guard<std::mutex> lock(mutex);
            if (sequence == consumed_sequence)
                return false;
            observation = values;
            consumed_sequence = sequence;
            return true;
        }

        VelocityCmd velocity_command() const
        {
            std::lock_guard<std::mutex> lock(mutex);
            return command;
        }

    private:
        mutable std::mutex mutex;
        std::vector<float> values;
        VelocityCmd command;
        std::uint64_t sequence{0};
        std::uint64_t consumed_sequence{0};
    };

    // A single action snapshot serves the controller and observation worker.
    struct ActionBuffer
    {
        void reset(std::size_t count = 0)
        {
            std::lock_guard<std::mutex> lock(mutex);
            values.assign(count, 0.0F);
            ready = false;
        }

        std::vector<float> read() const
        {
            std::lock_guard<std::mutex> lock(mutex);
            return values;
        }

        bool has_output() const
        {
            std::lock_guard<std::mutex> lock(mutex);
            return ready;
        }

        void publish(std::vector<float> action)
        {
            std::lock_guard<std::mutex> lock(mutex);
            values = std::move(action);
            ready = true;
        }

    private:
        mutable std::mutex mutex;
        std::vector<float> values;
        bool ready{false};
    };

    bool configure(const RobotConfig &robot,
                   const PolicyConfig &policy,
                   const std::filesystem::path &model_path)
    {
        ready_ = false;
        robot_cfg_ = robot;
        policy_ = policy;
        motion_time_.store(0.0);
        if (robot.dof <= 0 || policy.dof <= 0 || policy.dof > robot.dof ||
            !std::isfinite(robot.control_dt) || robot.control_dt <= 0.0 || policy.decimation <= 0)
        {
            tui::message("ERROR", "Invalid policy joint count or inference period");
            return false;
        }
        for (const int index : policy.joint_mapping)
        {
            if (index < 0 || index >= robot.dof)
            {
                tui::message("ERROR", "Policy joint mapping exceeds hardware joint count");
                return false;
            }
        }

        obs_.configure(policy_);
        reset_observation();
        register_builtin_obs();
        if (obs_.value().size() != static_cast<std::size_t>(std::max(policy_.input_dim, 0)))
        {
            std::cerr << "[PolicyManager] Configured observation size " << obs_.value().size()
                      << " != policy input_dim " << policy_.input_dim << '\n';
            return false;
        }

        backend_ = PolicyRegister::instance().create(policy_.model_name);
        const auto path = model_path.empty() ? std::filesystem::path(policy_.model_name) : model_path;
        if (!backend_)
        {
            std::cerr << "[PolicyManager] No inference backend registered for model extension: "
                      << std::filesystem::path(policy_.model_name).extension() << '\n';
            return false;
        }
        if (!backend_->load(policy_, path))
        {
            std::cerr << "[PolicyManager] Failed to load policy model: " << path << '\n';
            backend_.reset();
            return false;
        }

        const auto dim = backend_->output_dim() == 0
                             ? static_cast<std::size_t>(std::max(policy_.dof, 0))
                             : backend_->output_dim();
        actions_.reset(dim);
        observation_failed_.store(false);
        infer_failed_.store(false);

        ready_ = true;
        return true;
    }

    // The producer samples at dt; inference consumes its latest complete
    // snapshot independently at dt * decimation.
    bool start_inference()
    {
        if (!ready_)
            return false;
        if (!observation_timer_.start(robot_cfg_.control_dt, [this]() { on_observation_tick(); }))
            return false;
        if (!infer_timer_.start(robot_cfg_.control_dt * static_cast<double>(policy_.decimation),
                                [this]() { on_infer_tick(); }))
        {
            observation_timer_.stop();
            return false;
        }
        return true;
    }

    void stop_inference()
    {
        observation_timer_.stop();
        infer_timer_.stop();
    }

    void reset_inference()
    {
        stop_inference();
        reset_observation();
        motion_time_.store(0.0);
        actions_.reset();
    }

    // Called only with both workers stopped, before configuring or restarting.
    void reset_observation()
    {
        obs_.reset();
        sample_ = {};
        observations_.reset();
    }

    RobotCommand target_command() const
    {
        const auto action = actions_.read();
        const auto count = static_cast<std::size_t>(robot_cfg_.dof);
        RobotCommand command = hold_cmd_;
        for (std::size_t policy_index = 0; policy_index < static_cast<std::size_t>(policy_.dof); ++policy_index)
        {
            const int mapped = policy_index < policy_.joint_mapping.size()
                                   ? policy_.joint_mapping[policy_index]
                                   : static_cast<int>(policy_index);
            if (mapped < 0 || static_cast<std::size_t>(mapped) >= count)
                continue;

            const auto hardware_index = static_cast<std::size_t>(mapped);
            const float action_value = policy_index < action.size() ? action[policy_index] : 0.0F;
            const float scale = value_at(policy_.action_scale, policy_index, 1.0F);
            command.position[hardware_index] =
                value_at(policy_.default_dof_pos, policy_index, 0.0F) + action_value * scale;
            command.kp[hardware_index] = value_at(policy_.kp, policy_index, 0.0F);
            command.kd[hardware_index] = value_at(policy_.kd, policy_index, 0.0F);
        }
        return command;
    }

    static float value_at(const std::vector<float> &values, std::size_t index, float fallback)
    {
        return index < values.size() ? values[index] : fallback;
    }

    static float directional_command(float input, float positive, float negative)
    {
        if (!std::isfinite(input))
            return 0.0F;
        input = std::clamp(input, -1.0F, 1.0F);
        return input * (input >= 0.0F ? positive : negative);
    }

    // Observation worker owns sample_ and obs_.
    void sample_observation()
    {
        motion_time_.store(motion_time_.load() + robot_cfg_.control_dt);
        sample_.state.joint_pos = hardware_.get_joint_position();
        sample_.state.joint_vel = hardware_.get_joint_velocity();
        sample_.state.joint_torque = hardware_.get_joint_torque();
        sample_.state.base_quat = hardware_.get_base_quaternion();
        sample_.state.base_ang_vel = hardware_.get_base_angular_velocity();
        sample_.state.base_lin_acc = hardware_.get_base_linear_acceleration();
        // Unified sticks use right/down as positive; commands use forward/left/yaw.
        const auto axis = hardware_.get_joy_axis();
        const auto &scale = policy_.command_scale;
        sample_.command = {
            directional_command(-axis.L_Y, scale[0], scale[1]),
            directional_command(-axis.L_X, scale[2], scale[3]),
            directional_command(-axis.R_X, scale[4], scale[5])};
        if (policy_.clip_obs > 0.0F)
        {
            const float limit = policy_.clip_obs;
            sample_.command.x = std::clamp(sample_.command.x, -limit, limit);
            sample_.command.y = std::clamp(sample_.command.y, -limit, limit);
            sample_.command.yaw = std::clamp(sample_.command.yaw, -limit, limit);
        }
        sample_.action = actions_.read();
        collect_obs();
    }

    // Advance history at dt and publish one complete flattened observation.
    void on_observation_tick()
    {
        sample_observation();
        obs_.push();
        const auto &observation = obs_.value();
        if (observation.size() != static_cast<std::size_t>(policy_.input_dim))
        {
            if (!observation_failed_.exchange(true))
                std::cerr << "[PolicyManager] Observation size " << observation.size()
                          << " != expected " << policy_.input_dim << '\n';
            return;
        }
        observation_failed_.store(false);
        observations_.publish(observation, sample_.command);
    }

    // Copy the latest snapshot, then release the lock before model execution.
    // Slow inference never blocks sampling or queues up older observations.
    void on_infer_tick()
    {
        std::vector<float> observation;
        if (!observations_.consume(observation))
            return; // Wait for the first frame, or skip an unchanged frame.

        auto next = actions_.read();
        const auto expected_size = next.size();
        if (!backend_ || !backend_->infer(observation, next) || next.size() != expected_size)
        {
            if (!infer_failed_.exchange(true))
                std::cerr << "[PolicyManager] Inference failed; retaining previous action\n";
            return;
        }

        for (std::size_t i = 0; i < next.size(); ++i)
        {
            if (!std::isfinite(next[i]))
                next[i] = 0.0F;
            next[i] = std::clamp(next[i],
                                 value_at(policy_.action_lower, i, -100.0F),
                                 value_at(policy_.action_upper, i, 100.0F));
        }

        infer_failed_.store(false);
        actions_.publish(std::move(next));
    }

    void collect_obs()
    {
        for (const auto &name : policy_.observations)
        {
            const auto it = obs_fns_.find(name);
            if (it == obs_fns_.end())
                continue;
            auto values = it->second();
            utils::clip(values, policy_.clip_obs);
            obs_.set(name, values);
        }
    }

    void register_builtin_obs()
    {
        auto add = [this](const char *name, ObsFn fn) {
            if (obs_fns_.count(name) == 0)
                obs_fns_[name] = std::move(fn);
        };

        add("ang_vel", [this]() {
            auto out = sample_.state.base_ang_vel;
            utils::apply_scale(out, policy_.ang_vel_scale);
            return out;
        });

        add("base_gravity_vec", [this]() {
            return math::gravity_from_quat(sample_.state.base_quat);
        });

        add("gravity_vec", [this]() {
            return math::gravity_from_quat(sample_.state.base_quat);
        });

        add("commands", [this]() {
            return std::vector<float>{sample_.command.x, sample_.command.y, sample_.command.yaw};
        });

        add("dof_pos", [this]() {
            auto out = utils::gather(sample_.state.joint_pos, policy_.joint_mapping);
            auto defaults = policy_.default_dof_pos;
            utils::expand_to_num(defaults, out.size(), 0.0F);
            for (std::size_t i = 0; i < out.size(); ++i)
                out[i] -= defaults[i];
            for (const int index : policy_.wheel_joint_index)
            {
                if (index >= 0 && static_cast<std::size_t>(index) < out.size())
                    out[static_cast<std::size_t>(index)] = 0.0F;
            }
            utils::apply_scale(out, policy_.dof_pos_scale);
            return out;
        });

        add("dof_vel", [this]() {
            auto out = utils::gather(sample_.state.joint_vel, policy_.joint_mapping);
            utils::apply_scale(out, policy_.dof_vel_scale);
            return out;
        });

        add("actions", [this]() {
            return sample_.action;
        });

        add("whole_body_tracking/motion_command", [this]() {
            const auto frame = motion_.sample(motion_time_.load());
            auto out = utils::gather(frame.joint_position, policy_.joint_mapping);
            auto velocity = utils::gather(frame.joint_velocity, policy_.joint_mapping);
            out.insert(out.end(), velocity.begin(), velocity.end());
            return out;
        });

        add("whole_body_tracking/motion_anchor_ori_b", [this]() {
            const auto frame = motion_.sample(motion_time_.load());
            const auto &waist = policy_.waist_joint_index;
            if (waist.empty() || sample_.state.joint_pos.empty())
                return std::vector<float>(6, 0.0F);

            const auto joint = [&](int index) {
                return index >= 0 && static_cast<std::size_t>(index) < sample_.state.joint_pos.size()
                           ? sample_.state.joint_pos[static_cast<std::size_t>(index)]
                           : 0.0F;
            };
            const auto waist_angle = [&](std::size_t axis) {
                return axis < waist.size() ? joint(waist[axis]) : 0.0F;
            };
            const auto q_yaw = math::euler_to_quat({0.0F, 0.0F, waist_angle(0)});
            const auto q_roll = math::euler_to_quat({waist_angle(1), 0.0F, 0.0F});
            const auto q_pitch = math::euler_to_quat({0.0F, waist_angle(2), 0.0F});
            auto robot_torso = sample_.state.base_quat;
            robot_torso = math::quat_multiply(robot_torso, q_yaw);
            if (waist.size() > 1U)
                robot_torso = math::quat_multiply(robot_torso, q_roll);
            if (waist.size() > 2U)
                robot_torso = math::quat_multiply(robot_torso, q_pitch);

            const auto robot_inv = math::conjugate(math::normalize(robot_torso));
            const auto relative =
                math::normalize(math::quat_multiply(robot_inv, frame.anchor_quaternion));
            return math::quat_to_ori_6d(relative);
        });

        add("RoboMimic_Deploy/phase", [this]() {
            const double duration = motion_.duration();
            const double time = motion_time_.load();
            const float phase =
                duration > 0.0 ? static_cast<float>(std::clamp(time / duration, 0.0, 1.0)) : 0.0F;
            return std::vector<float>{phase};
        });
    }

    HardwareBase &hardware_;
    PolicyConfigLoader &config_;
    std::filesystem::path policy_root_;
    MotionCommand &motion_;
    bool policy_active_{false};
    RobotCommand hold_cmd_;
    RobotConfig robot_cfg_;
    PolicyConfig policy_;
    ObsCollection obs_;
    ObservationBuffer observations_;
    std::unique_ptr<PolicyBackend> backend_;
    std::unordered_map<std::string, ObsFn> obs_fns_;
    ActionBuffer actions_;
    std::atomic<double> motion_time_{0.0};
    bool ready_{false};
    std::atomic<bool> infer_failed_{false};
    std::atomic<bool> observation_failed_{false};
    ObservationSample sample_;
    ThreadTimer observation_timer_;
    ThreadTimer infer_timer_;
};

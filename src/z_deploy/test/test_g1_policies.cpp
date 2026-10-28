#include <atomic>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <policy/policy_manager.hpp>

namespace
{
void require(bool condition, const std::string &message)
{
    if (!condition)
        throw std::runtime_error(message);
}

bool finite(const std::vector<float> &values)
{
    for (const float value : values)
        if (!std::isfinite(value))
            return false;
    return true;
}

class Hardware : public HardwareBase
{
public:
    explicit Hardware(const RobotConfig &robot) : HardwareBase(robot)
    {
        state_staging_.joint_pos = robot.default_dof_pos;
        publish_state(state_staging_);
    }
    joybutton get_joy_button() const override { return joybutton::None; }
    joyaxis get_joy_axis() const override { return {}; }
    RobotCommand last;

private:
    void start_backend() override {}
    void stop_backend() override {}
    void send_command_impl(const RobotCommand &command) override { last = command; }
};

struct Snapshot
{
    std::atomic<std::size_t> frames{0};
    std::atomic<bool> failed{false};
    // Written only on the first inference and read after workers stop.
    std::vector<float> first_observation;
};

// Exercise real TorchScript inference with the observations built by the
// deployment workers, including tracking and history, without ROS or motors.
class RecordingBackend : public PolicyBackend
{
public:
    explicit RecordingBackend(std::shared_ptr<Snapshot> snapshot) : snapshot_(std::move(snapshot)) {}

    bool load(const PolicyConfig &config, const std::filesystem::path &path) override
    {
        return backend_.load(config, path);
    }

    bool infer(const std::vector<float> &observation, std::vector<float> &action) override
    {
        if (observation.size() != input_dim() || !finite(observation) ||
            !backend_.infer(observation, action) || action.size() != output_dim() || !finite(action))
        {
            snapshot_->failed.store(true);
            return false;
        }
        if (snapshot_->frames.load() == 0)
            snapshot_->first_observation = observation;
        ++snapshot_->frames;
        return true;
    }

    std::size_t input_dim() const override { return backend_.input_dim(); }
    std::size_t output_dim() const override { return backend_.output_dim(); }

private:
    std::shared_ptr<Snapshot> snapshot_;
    TorchPolicyBackend backend_;
};

void check_policy(const std::string &id, int input_dim, bool tracking)
{
    PolicyConfigLoader config;
    require(config.load(Z_G1_POLICY_ROOT, "g1", id), "Cannot load G1 policy " + id);
    const auto expected = config.policy();
    const auto &robot = config.robot();
    require(robot.dof == 29 && expected.dof == 29 && expected.input_dim == input_dim &&
                robot.joint_names.size() == 29 && expected.joint_mapping.size() == 29,
            id + ": incorrect joint or observation dimensions");
    std::vector<bool> seen(29, false);
    for (const int index : expected.joint_mapping)
    {
        require(index >= 0 && index < 29 && !seen[static_cast<std::size_t>(index)],
                id + ": joint mapping must be a permutation of the 29 robot joints");
        seen[static_cast<std::size_t>(index)] = true;
    }
    require(expected.model_name == "policy.pt" &&
                std::filesystem::is_regular_file(config.policy_dir() / expected.model_name),
            id + ": configured TorchScript model does not exist");
    if (tracking)
        require(expected.waist_joint_index == robot.waist_joint_index &&
                    expected.waist_joint_index.size() == 3,
                id + ": tracking must use the robot's yaw/roll/pitch waist indices");

    Hardware hardware(robot);
    MotionCommand motion;
    const auto snapshot = std::make_shared<Snapshot>();
    PolicyRegister::instance().register_backend(".pt", [snapshot]() {
        return std::make_unique<RecordingBackend>(snapshot);
    });
    PolicyManager manager(hardware, config, Z_G1_POLICY_ROOT, motion);
    require(manager.start_policy(id), id + ": cannot start policy manager");
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (snapshot->frames.load() < 3 && !snapshot->failed.load() &&
           std::chrono::steady_clock::now() < deadline)
    {
        manager.run_policy();
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    manager.run_policy();
    manager.stop_policy();
    require(!snapshot->failed.load() && snapshot->frames.load() >= 3,
            id + ": real model inference failed or timed out");
    require(snapshot->first_observation.size() == static_cast<std::size_t>(input_dim),
            id + ": deployment observation shape does not match the model");
    require(hardware.last.position.size() == 29 && hardware.last.kp.size() == 29 &&
                hardware.last.kd.size() == 29 && finite(hardware.last.position) &&
                finite(hardware.last.kp) && finite(hardware.last.kd),
            id + ": command must contain 29 finite joint targets and gains");
    for (std::size_t i = 0; i < expected.joint_mapping.size(); ++i)
    {
        const auto index = static_cast<std::size_t>(expected.joint_mapping[i]);
        require(hardware.last.kp[index] == expected.kp[i] && hardware.last.kd[index] == expected.kd[i],
                id + ": command gains do not follow the policy joint mapping");
    }
    require(motion.is_loaded() == tracking, id + ": reference motion loading is incorrect");
    if (tracking)
    {
        require(motion.size() > 1 && motion.joint_count() == 29 && motion.duration() > 0.0 &&
                    finite(motion.sample(0.0).joint_position),
                id + ": invalid reference motion");
        float norm_squared = 0.0F;
        for (std::size_t i = 58; i < 64; ++i)
            norm_squared += snapshot->first_observation[i] * snapshot->first_observation[i];
        require(std::fabs(norm_squared - 2.0F) < 1e-3F,
                id + ": motion anchor observation is not a valid 6D rotation");
    }
    std::cout << "G1 " << id << " deployment inference passed: " << input_dim << " -> 29\n";
}
}

int main()
{
    try
    {
        check_policy("gangnam_style", 154, true);
        check_policy("amp_zlab", 384, false);
        check_policy("mimic", 154, true);
        check_policy("zlocomotion", 480, false);
    }
    catch (const std::exception &error)
    {
        std::cerr << error.what() << '\n';
        return 1;
    }
}

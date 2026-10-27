#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <iterator>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <thread>

#include <fsm/robot/g1/g1_fsm.hpp>
#include <fsm/robot/g1_23/g1_23_fsm.hpp>

namespace
{
void require(bool ok, const char *message)
{
    if (!ok)
        throw std::runtime_error(message);
}

bool near(float left, float right)
{
    return std::fabs(left - right) < 1e-5F;
}

struct Capture
{
    std::mutex mutex;
    std::condition_variable cv;
    std::vector<std::vector<float>> observations;
    std::vector<std::chrono::steady_clock::time_point> inference_times;
    bool input_changed{false};
    bool released{false};
};

class Backend : public PolicyBackend
{
public:
    explicit Backend(Capture &capture) : capture_(capture) {}
    bool load(const PolicyConfig &config, const std::filesystem::path &) override
    {
        config_ = config;
        return true;
    }
    bool infer(const std::vector<float> &observation, std::vector<float> &action) override
    {
        std::unique_lock<std::mutex> lock(capture_.mutex);
        capture_.observations.push_back(observation);
        capture_.inference_times.push_back(std::chrono::steady_clock::now());
        capture_.cv.notify_all();
        if (!capture_.cv.wait_for(lock, std::chrono::seconds(2), [&]() { return capture_.released; }))
            return false;
        capture_.input_changed = capture_.input_changed || observation != capture_.observations.back();
        action = {1.0F, -2.0F};
        return true;
    }
    std::size_t input_dim() const override { return config_.input_dim; }
    std::size_t output_dim() const override { return config_.dof; }
private:
    Capture &capture_;
    PolicyConfig config_;
};

class Hardware : public HardwareBase
{
public:
    explicit Hardware(const RobotConfig &config) : HardwareBase(config)
    {
        state_staging_.joint_pos = {1.0F, 2.0F, 3.0F};
        publish_state(state_staging_);
    }
    joybutton get_joy_button() const override { return button; }
    joyaxis get_joy_axis() const override { return axis; }
    joyaxis axis{0.3F, -0.4F, -0.2F, 0.0F};
    joybutton button{joybutton::None};
    RobotCommand last;
    int commands{0};
private:
    void start_backend() override {}
    void stop_backend() override {}
    void send_command_impl(const RobotCommand &command) override
    {
        last = command;
        ++commands;
    }
};

// Encode the observation sequence in every command axis. This lets the test
// distinguish a recent complete snapshot from a queued or partially read frame.
class SamplingHardware : public Hardware
{
public:
    using Hardware::Hardware;
    joyaxis get_joy_axis() const override
    {
        const auto sequence = ++samples;
        const float value = static_cast<float>(sequence) / 1000.0F;
        return {-value, -value, -value, 0.0F};
    }
    mutable std::atomic<int> samples{0};
};

struct TemporaryPolicies
{
    TemporaryPolicies()
    {
        char path[] = "/tmp/frog-policy-test-XXXXXX";
        require(mkdtemp(path) != nullptr, "Cannot create policy fixtures");
        root = path;
        std::filesystem::create_directories(root / "test_robot" / "locomotion");
        std::ofstream(root / "test_robot" / "base.yaml") << R"(robot:
  dt: 0.01
  num_of_dofs: 3
  joint_names: [j0, j1, j2]
  default_dof_pos: [0, 0, 0]
  fixed_kp: [10]
  fixed_kd: [2]
)";
        std::ofstream(root / "test_robot" / "locomotion" / "config.yaml") << R"(policy:
  model_name: test.fake
  num_of_dofs: 2
  default_dof_pos: [0.1, 0.2]
  joint_mapping: [2, 0]
  rl_kp: [30, 40]
  rl_kd: [3, 4]
  action_scale: [0.5, 0.25]
  clip_actions_lower: [-0.5, -1]
  clip_actions_upper: [0.5, 1]
  decimation: 2
  observations: [commands, dof_pos, actions]
  observations_history: [0]
  num_observations: 7
  dof_pos_scale: [1]
  commands_scale: [2]
)";
        std::ofstream(root / "old_motion.csv") << "0,0,0,0,0,0,1,0,0,0\n";
    }
    void write_scale_policy(const std::string &id, const std::string &scale)
    {
        std::ifstream source(root / "test_robot" / "locomotion" / "config.yaml");
        std::string content{std::istreambuf_iterator<char>(source), std::istreambuf_iterator<char>()};
        const std::string original = "  commands_scale: [2]\n";
        content.replace(content.find(original), original.size(), scale.empty() ? "" : "  " + scale + "\n");
        const auto directory = root / "test_robot" / id;
        std::filesystem::create_directories(directory);
        std::ofstream(directory / "config.yaml") << content;
    }

    ~TemporaryPolicies() { std::filesystem::remove_all(root); }
    std::filesystem::path root;
};

void check_command_scale_config()
{
    TemporaryPolicies files;
    PolicyConfigLoader config;
    const std::array<float, 6> defaults{1, 1, 1, 1, 1, 1};
    require(PolicyConfig{}.command_scale == defaults, "Default scales must cover all six directions");
    const std::vector<std::pair<std::string, std::array<float, 6>>> valid{
        {"", defaults},
        {"commands_scale: []", defaults},
        {"commands_scale: 0.5", {0.5F, 0.5F, 0.5F, 0.5F, 0.5F, 0.5F}},
        {"commands_scale: [2]", {2, 2, 2, 2, 2, 2}},
        {"commands_scale: [2, 3, 4]", {2, 2, 3, 3, 4, 4}},
        {"commands_scale: [2, 0.5, 3, 1.5, 4, 2]", {2, 0.5F, 3, 1.5F, 4, 2}},
        {"commands_scale: [0, 0.5, 3, 0, 4, 0]", {0, 0.5F, 3, 0, 4, 0}},
    };
    for (const auto &test : valid)
    {
        files.write_scale_policy("scale", test.first);
        require(config.load(files.root, "test_robot", "scale"), "Valid command scales were rejected");
        require(config.policy().command_scale == test.second, "Command scales expanded to incorrect directions");
    }
    const auto previous = config.policy().command_scale;
    const std::vector<std::string> invalid{
        "[1, 2]", "[1, 2, 3, 4]", "[1, 2, 3, 4, 5]", "[1, 2, 3, 4, 5, 6, 7]",
        "[-1]", "[1, -1, 1]", "[1, 1, 1, 1, 1, -1]", "[.nan]", "[.inf]", "[-.inf]",
        "[1, 1, 1, 1, 1, .nan]", "[1, .inf, 1]", "{x: 1}", "null", "invalid",
    };
    for (const auto &value : invalid)
    {
        files.write_scale_policy("invalid", "commands_scale: " + value);
        require(!config.load(files.root, "test_robot", "invalid"), "Invalid command scales were accepted");
        require(config.is_policy_loaded() && config.policy().command_scale == previous,
                "Rejected scales overwrote the previous valid configuration");
    }
}

void check_directional_commands()
{
    TemporaryPolicies files;
    files.write_scale_policy("directional", "commands_scale: [2, 0.5, 3, 1.5, 4, 2]");
    files.write_scale_policy("disabled", "commands_scale: [0, 0.5, 3, 0, 4, 0]");
    files.write_scale_policy("clipped", "commands_scale: [2, 0.5, 3, 1.5, 4, 2]");
    std::ofstream(files.root / "test_robot" / "clipped" / "config.yaml", std::ios::app) << "  clip_obs: 0.25\n";
    files.write_scale_policy("invalid", "commands_scale: [1, -1, 1]");
    PolicyConfigLoader config;
    require(config.load(files.root, "test_robot"), "Cannot load hardware configuration");
    Hardware hardware(config.robot());
    MotionCommand motion;
    Capture capture;
    capture.released = true;
    PolicyRegister::instance().register_backend(".fake", [&]() { return std::make_unique<Backend>(capture); });
    PolicyManager manager(hardware, config, files.root, motion);
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const float inf = std::numeric_limits<float>::infinity();
    struct Case
    {
        std::string policy;
        joyaxis axis;
        std::array<float, 3> expected;
    };
    const std::vector<Case> cases{
        {"directional", {-0.5F, -0.5F, -0.5F, 0}, {1, 1.5F, 2}},
        {"directional", {0.5F, 0.5F, 0.5F, 0}, {-0.25F, -0.75F, -1}},
        {"directional", {-2, -3, -4, 0}, {2, 3, 4}},
        {"directional", {2, 3, 4, 0}, {-0.5F, -1.5F, -2}},
        {"directional", {0, 0, 0, 0}, {0, 0, 0}},
        {"directional", {-0.5F, nan, 0.5F, 0}, {0, 1.5F, -1}},
        {"directional", {inf, -0.5F, -inf, 0}, {1, 0, 0}},
        {"disabled", {-1, -1, -1, 0}, {0, 3, 4}},
        {"disabled", {1, 1, 1, 0}, {-0.5F, 0, 0}},
        {"clipped", {-1, -1, -1, 0}, {0.25F, 0.25F, 0.25F}},
    };
    for (const auto &test : cases)
    {
        hardware.axis = test.axis; // Inference is stopped between cases.
        {
            std::lock_guard<std::mutex> lock(capture.mutex);
            capture.observations.clear();
        }
        require(manager.start_policy(test.policy), "Directional policy did not start");
        std::vector<float> observation;
        {
            std::unique_lock<std::mutex> lock(capture.mutex);
            capture.cv.wait_for(lock, std::chrono::seconds(1), [&]() { return !capture.observations.empty(); });
            if (!capture.observations.empty())
                observation = capture.observations.front();
        }
        const auto command = manager.velocity_command();
        require(near(command.x, test.expected[0]) && near(command.y, test.expected[1]) &&
                near(command.yaw, test.expected[2]), "Displayed command differs from model input");
        manager.stop_policy();
        const auto stopped = manager.velocity_command();
        require(stopped.x == 0 && stopped.y == 0 && stopped.yaw == 0, "Stopped policy retained displayed speed");
        require(observation.size() == 7, "Directional scaling changed model observation dimensions");
        for (std::size_t axis = 0; axis < test.expected.size(); ++axis)
            require(near(observation[axis], test.expected[axis]), "Incorrect directional command or runtime fallback");
    }
    // Configuration errors must leave the policy inactive so the FSM can fall back.
    FsmContext context{hardware, config, manager};
    g1::RLLocomotion state(context);
    require(!manager.start_policy("invalid") && !manager.policy_active(), "Invalid scales started a policy");
    require(state.check_change() == "Passive", "Invalid policy did not request Passive fallback");
    PolicyRegister::instance().register_backend(".fake", []() { return std::unique_ptr<PolicyBackend>{}; });
}

void check_independent_workers(double dt, int decimation, bool term_priority)
{
    TemporaryPolicies files;
    const auto directory = files.root / "test_robot" / "timing";
    std::filesystem::create_directories(directory);
    {
        std::ofstream policy_file(directory / "config.yaml");
        policy_file << "policy:\n"
                    << "  dt: " << dt << "\n"
                    << "  decimation: " << decimation << "\n"
                    << "  model_name: test.fake\n"
                    << "  num_of_dofs: 2\n"
                    << "  default_dof_pos: [0, 0]\n"
                    << "  clip_actions_lower: [-0.5, -1]\n"
                    << "  clip_actions_upper: [0.5, 1]\n"
                    << "  observations: [commands, actions]\n"
                    << "  observations_history: [0, 1, 2, 3, 4]\n"
                    << "  observations_history_priority: " << (term_priority ? "term" : "time") << "\n"
                    << "  num_observations: 25\n";
    }
    PolicyConfigLoader config;
    require(config.load(files.root, "test_robot"), "Cannot load sampling hardware configuration");
    SamplingHardware hardware(config.robot());
    MotionCommand motion;
    Capture capture;
    PolicyRegister::instance().register_backend(".fake", [&]() { return std::make_unique<Backend>(capture); });
    PolicyManager manager(hardware, config, files.root, motion);
    require(manager.start_policy("timing"), "Independent workers did not start");
    {
        std::unique_lock<std::mutex> lock(capture.mutex);
        require(capture.cv.wait_for(lock, std::chrono::seconds(1), [&]()
                                   { return !capture.observations.empty(); }), "First inference did not run");
    }
    const int first_sample = hardware.samples.load();
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
    while (hardware.samples.load() < first_sample + 12 && std::chrono::steady_clock::now() < deadline)
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    const int before_release = hardware.samples.load();
    {
        std::lock_guard<std::mutex> lock(capture.mutex);
        capture.released = true;
        capture.cv.notify_all();
    }
    require(before_release >= first_sample + 12, "Blocked inference prevented observation updates");
    {
        std::unique_lock<std::mutex> lock(capture.mutex);
        require(capture.cv.wait_for(lock, std::chrono::seconds(2), [&]()
                                   { return capture.observations.size() >= 8; }), "Periodic inference stalled");
    }
    manager.stop_policy();
    require(!capture.input_changed, "Inference input changed while the observation worker was running");
    const auto sequence = [](const std::vector<float> &observation)
    { return static_cast<int>(std::lround(observation.front() * 1000.0F)); };
    require(sequence(capture.observations[1]) >= before_release - 1,
            "Inference consumed queued observations instead of the latest snapshot");
    for (std::size_t sample = 1; sample < capture.observations.size(); ++sample)
    {
        const auto &observation = capture.observations[sample];
        require(observation.size() == 25, "History dimensions changed across threads");
        require(sequence(observation) > sequence(capture.observations[sample - 1]),
                "Inference reused an old observation snapshot");
        const float latest = observation.front();
        for (std::size_t history = 0; history < 5; ++history)
        {
            for (std::size_t axis = 0; axis < 3; ++axis)
            {
                const auto index = term_priority ? history * 3 + axis : history * 5 + axis;
                require(near(observation[index], latest - static_cast<float>(history) / 1000.0F),
                        "History did not advance once per observation tick, or the snapshot was torn");
            }
        }
        const auto action_offset = term_priority ? 15U : 3U;
        require(near(observation[action_offset], 0.5F) && near(observation[action_offset + 1], -1.0F),
                "Observation did not sample the last completed action");
    }
    // Ignore the deliberately blocked first inference when measuring cadence.
    const auto intervals = capture.observations.size() - 2;
    const int sample_gap = sequence(capture.observations.back()) - sequence(capture.observations[1]);
    const int expected_gap = static_cast<int>(intervals) * decimation;
    require(std::abs(sample_gap - expected_gap) <= 2 * decimation + 2,
            "Observation/inference sampling ratio does not follow decimation");
    const double elapsed = std::chrono::duration<double>(
        capture.inference_times.back() - capture.inference_times[1]).count();
    const double expected_elapsed = static_cast<double>(intervals) * dt * decimation;
    require(elapsed >= expected_elapsed * 0.8 && elapsed <= expected_elapsed * 1.8 + 0.03,
            "Inference period does not follow configured dt * decimation");
    const int stopped_samples = hardware.samples.load();
    std::this_thread::sleep_for(std::chrono::duration<double>(dt * 3));
    require(hardware.samples.load() == stopped_samples, "Observation worker continued after stop_policy");

    // Restart with a new source sequence: no old history/action snapshot may survive.
    hardware.samples.store(0);
    {
        std::lock_guard<std::mutex> lock(capture.mutex);
        capture.observations.clear();
        capture.inference_times.clear();
        capture.released = false;
    }
    require(manager.start_policy("timing"), "Independent workers could not restart");
    std::vector<float> restarted;
    {
        std::unique_lock<std::mutex> lock(capture.mutex);
        capture.cv.wait_for(lock, std::chrono::seconds(1), [&]() { return !capture.observations.empty(); });
        if (!capture.observations.empty())
            restarted = capture.observations.front();
        capture.released = true;
        capture.cv.notify_all();
    }
    manager.stop_policy();
    require(restarted.size() == 25 && sequence(restarted) <= decimation + 2,
            "Restart consumed an old observation snapshot");
    const auto action_offset = term_priority ? 15U : 3U;
    require(restarted[action_offset] == 0 && restarted[action_offset + 1] == 0,
            "Restart retained an action from the previous policy");
    PolicyRegister::instance().register_backend(".fake", []() { return std::unique_ptr<PolicyBackend>{}; });
}

void check_policy_lifecycle()
{
    TemporaryPolicies files;
    PolicyConfigLoader config;
    require(config.load(files.root, "test_robot"), "Cannot load base configuration");
    Hardware hardware(config.robot());
    MotionCommand motion;
    require(motion.load(files.root / "old_motion.csv", 3), "Cannot load old motion");
    Capture capture;
    PolicyRegister::instance().register_backend(".fake", [&]() { return std::make_unique<Backend>(capture); });
    PolicyManager manager(hardware, config, files.root, motion);
    require(manager.start_policy("locomotion") && manager.policy_active(), "Policy did not start");
    require(!motion.is_loaded() && !manager.motion_finished(), "Previous policy motion was retained");
    manager.run_policy();
    require(hardware.last.position == std::vector<float>({1, 2, 3}), "First output must hold measured position");
    {
        std::unique_lock<std::mutex> lock(capture.mutex);
        const bool sampled = capture.cv.wait_for(lock, std::chrono::seconds(1), [&]()
                                                 { return !capture.observations.empty(); });
        capture.released = true;
        capture.cv.notify_all();
        require(sampled, "Inference did not sample hardware");
        const auto &obs = capture.observations.front();
        require(obs.size() == 7 && near(obs[0], 0.8F) && near(obs[1], -0.6F) && near(obs[2], 0.4F),
                "Unified joystick axes or command scaling are incorrect");
        require(near(obs[3], 2.9F) && near(obs[4], 0.8F) && obs[5] == 0 && obs[6] == 0,
                "Joint mapping or previous action observation is incorrect");
    }
    bool produced = false;
    for (int i = 0; i < 100; ++i)
    {
        manager.run_policy();
        if (near(hardware.last.position[2], 0.35F))
        {
            produced = true;
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    require(produced && hardware.last.position.size() == 3 && near(hardware.last.position[0], -0.05F),
            "Clipped actions were not mapped to hardware joints");
    require(hardware.last.position[1] == 2 && hardware.last.kp[2] == 30 && hardware.last.kp[0] == 40,
            "Unmapped joint hold or policy gains are incorrect");
    manager.stop_policy();
    const int commands = hardware.commands;
    manager.run_policy();
    require(!manager.policy_active() && hardware.commands == commands, "Stopped policy still sent commands");
    require(!manager.start_policy("missing") && !manager.policy_active(), "Failed policy remained active");
    manager.run_policy();
    require(hardware.commands == commands, "Failed policy reused old commands");
    require(manager.start_policy("locomotion"), "Policy could not restart after a failed load");
    manager.stop_policy();
    // Factory captures a local reference; remove it before that reference expires.
    PolicyRegister::instance().register_backend(".fake", []() { return std::unique_ptr<PolicyBackend>{}; });
}
}

int main()
{
    try
    {
        check_command_scale_config();
        check_directional_commands();
        check_independent_workers(0.005, 4, false);
        check_independent_workers(0.01, 3, true);
        check_policy_lifecycle();
        std::cout << "policy manager tests passed\n";
    }
    catch (const std::exception &error)
    {
        std::cerr << error.what() << '\n';
        return 1;
    }
}

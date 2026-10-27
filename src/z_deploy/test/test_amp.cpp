#include <chrono>
#include <cmath>
#include <condition_variable>
#include <iostream>
#include <mutex>
#include <stdexcept>

#include <z_deploy/fsm/robot/g1/g1_fsm.hpp>
#include <z_deploy/hardware/base_interface.hpp>

namespace
{
void require(bool ok, const char *message)
{
    if (!ok)
        throw std::runtime_error(message);
}

void check_history_collection()
{
    PolicyConfig cfg;
    cfg.observations = {"first", "second"};
    cfg.observation_dims = {1, 1};
    cfg.observation_history = {3, 1, 0};
    for (const auto *order : {"time", "term"})
    {
        cfg.observation_history_priority = order;
        ObsCollection history;
        history.configure(cfg);
        require(history.value() == std::vector<float>(6, 0.0F), "Wrong initial history size");
        for (int step = 1; step <= 4; ++step)
        {
            history.set("first", {static_cast<float>(step)});
            history.set("second", {static_cast<float>(step + 10)});
            history.push();
        }
        const auto expected = cfg.observation_history_priority == "time"
                                  ? std::vector<float>{1, 11, 3, 13, 4, 14}
                                  : std::vector<float>{1, 3, 4, 11, 13, 14};
        require(history.value() == expected, "History indices or flatten order changed");
        history.reset();
        require(history.value() == std::vector<float>(6, 0.0F), "Reset retained old history");
        history.set("first", {7.0F});
        history.push();
        const auto seeded = cfg.observation_history_priority == "time"
                                ? std::vector<float>{7, 0, 7, 0, 7, 0}
                                : std::vector<float>{7, 7, 7, 0, 0, 0};
        require(history.value() == seeded, "First frame after reset was not seeded correctly");
    }
    cfg.observation_history = {0};
    ObsCollection single;
    single.configure(cfg);
    single.set("first", {1.0F});
    single.push();
    single.set("first", {2.0F});
    single.push();
    require(single.value() == std::vector<float>{2, 0}, "Single-frame observation is not current");
}

class TestRobot : public RobotInterface
{
public:
    explicit TestRobot(const RobotConfig &config) : RobotInterface(config)
    {
        state_staging_.joint_pos = config.default_dof_pos;
        publish_state(state_staging_);
    }
    RobotCommand last;
protected:
    void start_backend() override {}
    void stop_backend() override {}
    void send_command_impl(const RobotCommand &cmd) override { last = cmd; }
};

class TestInput : public InputBase
{
public:
    using InputBase::InputBase;
    using InputBase::feed_key;
    using InputBase::feed_joy;
    UserCommand get_user_command() const override
    {
        auto command = InputBase::get_user_command();
        command.x = static_cast<float>(++samples);
        return command;
    }
    mutable std::size_t samples{0}; // read by the test only after inference stops
};

struct Capture
{
    std::mutex mutex;
    std::condition_variable cv;
    std::vector<std::vector<float>> observations;
};

class CaptureBackend : public PolicyBackend
{
public:
    explicit CaptureBackend(Capture &capture) : capture_(capture) {}
    bool load(const PolicyConfig &config, const std::filesystem::path &) override
    {
        config_ = config;
        return true;
    }
    bool infer(const std::vector<float> &obs, std::vector<float> &action) override
    {
        std::lock_guard<std::mutex> lock(capture_.mutex);
        capture_.observations.push_back(obs);
        action.assign(config_.dof, static_cast<float>(capture_.observations.size()));
        capture_.cv.notify_one();
        return true;
    }
    std::size_t input_dim() const override { return config_.input_dim; }
    std::size_t output_dim() const override { return config_.dof; }
private:
    Capture &capture_;
    PolicyConfig config_;
};
} // namespace

int main()
{
    try
    {
        check_history_collection();
        const std::filesystem::path root(Z_POLICY_ROOT);
        PolicyConfigLoader config;
        require(config.load(root, "g1", "amp"), "AMP config failed to load");
        const auto cfg = config.policy();
        require(cfg.input_dim == 384 && cfg.dof == 29, "Wrong model dimensions");
        require(cfg.observation_history_priority == "term", "Exported AMP model requires term-major history");
        require(config.robot().control_dt * cfg.decimation == 0.02, "Wrong policy period");
        require(config.robot().joint_names.size() == 29, "Wrong joint names");
        require(std::isinf(cfg.action_lower[0]) && std::isinf(cfg.action_upper[0]),
                "AMP actions should not be clipped");

        OnnxPolicyBackend real;
        const auto model = root / "g1/amp" / cfg.model_name;
        require(real.load(cfg, model), "Real ONNX model failed to load");
        ObsCollection standing;
        standing.configure(cfg);
        standing.set("gravity_vec", {0.0F, 0.0F, -1.0F});
        standing.push();
        const auto &obs = standing.value();
        std::vector<float> action;
        require(real.infer(obs, action) && action.size() == 29, "Real ONNX inference failed");
        bool nonzero = false;
        for (const auto value : action)
        {
            require(std::isfinite(value), "ONNX returned non-finite action");
            nonzero = nonzero || std::abs(value) > 1e-6F;
        }
        require(nonzero, "ONNX returned placeholder zero actions");
        auto wrong = cfg;
        wrong.input_dim = 96;
        OnnxPolicyBackend bad;
        require(!bad.load(wrong, model), "Model input mismatch was accepted");
        require(!bad.load(cfg, root / "missing.onnx"), "Missing model was accepted");

        Capture capture;
        PolicyRegister::instance().register_backend(".onnx", [&capture]() {
            return std::make_unique<CaptureBackend>(capture);
        });
        TestRobot robot(config.robot());
        TestInput input(config.robot(), nullptr);
        MotionCommand motion;
        PolicyManager policy(robot, input, config, root, motion);
        FsmContext ctx{robot, input, config, policy};
        g1::Factory factory;
        auto getup = factory.create_state(ctx, "GetUp");
        auto amp = factory.create_state(ctx, "AMP");
        require(amp != nullptr, "AMP state is not registered");
        getup->enter();
        input.feed_key(Keyboard::Num3);
        require(getup->check_change() == "GetUp", "AMP started before GetUp completed");
        for (int i = 0; i < 410; ++i)
            getup->run();
        input.feed_key(Keyboard::Num3);
        require(getup->check_change() == "AMP", "GetUp cannot enter AMP");

        amp->enter();
        bool collected;
        {
            std::unique_lock<std::mutex> lock(capture.mutex);
            collected = capture.cv.wait_for(lock, std::chrono::seconds(2), [&]() {
                return capture.observations.size() >= 5;
            });
        }
        input.feed_key(Keyboard::Num1);
        const auto next = amp->check_change();
        amp->run();
        amp->exit();
        require(collected, "Policy-rate inference did not run");
        require(next == "RLLocomotion", "AMP cannot switch to locomotion");
        require(!policy.policy_active(), "AMP exit did not stop the policy");
        require(input.samples == capture.observations.size(), "History advanced between inferences");
        for (std::size_t n = 0; n < 5; ++n)
            for (int h = 0; h < 4; ++h)
            {
                const auto &frame = capture.observations[n];
                require(frame.size() == 384, "Wrong flattened history size");
                const int t = static_cast<int>(n) + h - 2;
                require(frame[24 + h * 3] == static_cast<float>(std::max(1, t)),
                        "History order or first-frame fill differs from mjlab");
                require(frame[268 + h * 29] == static_cast<float>(std::max(0, t - 1)),
                        "Action observation is not the previous raw policy action");
            }
        require(robot.last.kp == cfg.kp && robot.last.kd == cfg.kd, "AMP PD gains were not applied");

        std::vector<int> buttons(6, 0);
        std::vector<float> axes(8, 0.0F);
        buttons[5] = 1;
        axes[7] = -1.0F;
        input.feed_joy(buttons, axes);
        require(input.consume_target_state() == "AMP", "RB + DPadDown does not request AMP");
        std::cout << "AMP config, ONNX inference, history and FSM checks passed\n";
        return 0;
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
        return 1;
    }
}

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <vector>

#include <fsm/robot/adam/adam_fsm.hpp>
#include <fsm/robot/g1/g1_fsm.hpp>
#include <fsm/robot/g1_23/g1_23_fsm.hpp>

namespace
{
void require(bool ok, const char *message)
{
    if (!ok)
        throw std::runtime_error(message);
}

class Hardware : public HardwareBase
{
public:
    explicit Hardware(const RobotConfig &config) : HardwareBase(config)
    {
        state_staging_.joint_pos = {0.5F};
        publish_state(state_staging_);
    }
    joybutton get_joy_button() const override { return button; }
    joyaxis get_joy_axis() const override { return axis; }
    joyaxis axis{};
    joybutton button{joybutton::None};
    RobotCommand last;
private:
    void start_backend() override {}
    void stop_backend() override {}
    void send_command_impl(const RobotCommand &command) override { last = command; }
};

class Backend : public PolicyBackend
{
public:
    bool load(const PolicyConfig &config, const std::filesystem::path &) override
    {
        config_ = config;
        return true;
    }
    bool infer(const std::vector<float> &, std::vector<float> &action) override
    {
        action.assign(static_cast<std::size_t>(config_.dof), 0.0F);
        return true;
    }
    std::size_t input_dim() const override { return config_.input_dim; }
    std::size_t output_dim() const override { return config_.dof; }
private:
    PolicyConfig config_;
};

struct Files
{
    Files()
    {
        char path[] = "/tmp/frog-fsm-test-XXXXXX";
        require(mkdtemp(path) != nullptr, "Cannot create FSM fixtures");
        root = path;
        std::filesystem::create_directories(root / "robot");
        std::ofstream(root / "robot" / "base.yaml") << R"(robot:
  dt: 1.0
  num_of_dofs: 1
  default_dof_pos: [2.0]
  fixed_kp: [10]
  fixed_kd: [2]
)";
        for (const auto *policy : {"locomotion", "dance_102", "amp", "dance1_subject2", "mimic",
                                   "gangnam_style", "amp_zlab", "zlocomotion"})
        {
            const auto directory = root / "robot" / policy;
            std::filesystem::create_directories(directory);
            std::ofstream(directory / "config.yaml") << R"(policy:
  model_name: test.fake
  num_of_dofs: 1
  default_dof_pos: [0]
  rl_kp: [10]
  rl_kd: [2]
  decimation: 4
  observations: [commands]
  observations_history: [0]
  num_observations: 3
  commands_scale: [2, 0.5, 3, 1.5, 4, 2]
)";
        }
    }
    ~Files() { std::filesystem::remove_all(root); }
    std::filesystem::path root;
};

// Generic states receive raw buttons; each state owns its input cursor.
class Probe : public FsmState
{
public:
    explicit Probe(FsmContext &ctx) : FsmState("CustomState", ctx) {}
    void run() override {}
    joybutton poll()
    {
        poll_input();
        return current_button_;
    }
};

void check_generic_input(FsmContext &ctx, Hardware &hardware)
{
    Probe first(ctx), second(ctx);
    first.enter();
    second.enter();
    hardware.button = joybutton::B;
    require(first.poll() == joybutton::B && second.poll() == joybutton::B,
            "One state consumed another state's input");
    require(first.poll() == joybutton::None, "Held button repeated");
    first.enter();
    require(first.poll() == joybutton::None, "State entry replayed a held button");
    hardware.button = joybutton::None;
    first.poll();
    hardware.button = joybutton::B;
    require(first.poll() == joybutton::B, "Button release did not rearm input");
    hardware.button = joybutton::None;
}

void check_robot(const std::string &robot_type)
{
    Files files;
    PolicyConfigLoader config;
    require(config.load(files.root, "robot"), "Cannot load FSM configuration");
    Hardware hardware(config.robot());
    MotionCommand motion;
    PolicyManager policy(hardware, config, files.root, motion);
    FsmContext context{hardware, config, policy};
    check_generic_input(context, hardware);
    auto manager = FsmRegister::instance().create(robot_type, context);
    const bool adam = robot_type == "adam";
    require(manager && manager->current()->name() == (adam ? "Disabled" : "Passive"),
            "Factory selected the wrong initial state");
    // Exercise state lifecycle methods one step at a time without starting
    // the wall-clock scheduler; its continuous loop has a separate test.
    const auto tick = [&]()
    {
        auto *state = manager->current();
        state->run();
        const auto next = state->check_change();
        if (next != state->name())
            manager->start(next);
    };
    const auto current = [&]() { return manager->current()->name(); };
    const auto press = [&](joybutton button)
    {
        hardware.button = joybutton::None;
        tick();
        hardware.button = button;
        tick();
    };
    const bool g1 = robot_type == "g1";
    const auto get_up = g1 ? joybutton::RB_A : joybutton::A;
    const auto locomotion = g1 ? joybutton::LB_DPadUp : joybutton::RB_DPadUp;
    const auto dance = g1 ? joybutton::LB_DPadRight : joybutton::RB_DPadRight;
    const auto amp = g1 ? joybutton::LB_DPadDown : joybutton::RB_DPadDown;
    if (adam)
    {
        tick();
        require(hardware.last.kp[0] == 0 && hardware.last.kd[0] == 0,
                "Adam must start with motor gains disabled");
        press(joybutton::LB_X);
        require(current() == "Passive", "Adam could not leave Disabled");
    }
    hardware.button = get_up;
    tick();
    require(current() == "GetUp", "GetUp button did not request GetUp");
    hardware.button = amp;
    tick(); // Halfway through standing: request must be ignored.
    require(current() == "GetUp" && std::fabs(hardware.last.position[0] - 1.25F) < 1e-5F,
            "GetUp interpolation or early policy gating failed");
    require(hardware.last.kp[0] == 10 && hardware.last.kd[0] == 2, "GetUp did not use configured gains");
    tick(); // Finish while the request remains held.
    require(current() == "GetUp" && hardware.last.position[0] == 2.0F,
            "GetUp replayed a request made before standing completed");
    press(dance);
    if (robot_type == "g1")
    {
        require(current() == "Dance102" && config.policy().policy_name == "dance_102",
                "G1 dance selection did not stay in the robot FSM");
        press(amp);
        require(current() == "AMP" && config.policy().policy_name == "amp", "G1 AMP selection failed");
    }
    else if (robot_type == "g1_23")
        require(current() == "Dance" && config.policy().policy_name == "dance1_subject2" &&
                    policy.policy_active(), "G1-23 did not start its dance policy");
    else
        require(current() == "GetUp", "Adam accepted an unsupported dance request");
    press(locomotion);
    require(current() == "RLLocomotion" && config.policy().policy_name == "locomotion" && policy.policy_active(),
            "Locomotion did not start its policy");
    if (robot_type == "g1_23" || adam)
    {
        press(amp);
        if (robot_type == "g1_23")
        {
            require(current() == "Mimic" && config.policy().policy_name == "mimic" &&
                        policy.policy_active(), "G1-23 did not start its mimic policy");
            press(locomotion);
        }
        else
            require(current() == "RLLocomotion", "Adam accepted an unsupported mimic request");
    }
    press(joybutton::RB_Y);
    require(current() == "RLLocomotion", "Simulation-only button changed FSM state");
    press(g1 ? joybutton::Y : joybutton::LB_X);
    require(current() == "Passive" && !policy.policy_active(), "Leaving a policy did not stop it");
    press(joybutton::Y);
    require(current() == "Disabled", "Passive could not enter Disabled");
    tick();
    require(hardware.last.kp[0] == 0 && hardware.last.kd[0] == 0 && hardware.last.torque[0] == 0,
            "Disabled did not unload gains and torque");
    press(joybutton::Y);
    require(current() == "Passive", "Disabled did not return to Passive");
    tick();
    require(current() == "Passive", "Recovery button was replayed by the next state");

    // A failed policy entry must return to the robot's own fallback state.
    std::filesystem::remove(files.root / "robot" / "locomotion" / "config.yaml");
    manager->start("RLLocomotion");
    require(!policy.policy_active(), "Failed policy became active");
    tick();
    require(current() == "Passive", "Failed policy did not fall back to Passive");
}

void check_g1_policy_shortcuts()
{
    struct Shortcut
    {
        joybutton button;
        const char *state;
        const char *policy;
    };
    const std::vector<Shortcut> shortcuts{
        {joybutton::LB_DPadUp, "RLLocomotion", "locomotion"},
        {joybutton::LB_DPadDown, "AMP", "amp"},
        {joybutton::LB_DPadLeft, "GangnamStyle", "gangnam_style"},
        {joybutton::LB_DPadRight, "Dance102", "dance_102"},
        {joybutton::RB_DPadUp, "AmpZlab", "amp_zlab"},
        {joybutton::RB_DPadDown, "Mimic", "mimic"},
        {joybutton::RB_DPadLeft, "ZLocomotion", "zlocomotion"}};
    Files files;
    PolicyConfigLoader config;
    require(config.load(files.root, "robot"), "Cannot load G1 shortcut configuration");
    Hardware hardware(config.robot());
    MotionCommand motion;
    PolicyManager policy(hardware, config, files.root, motion);
    FsmContext context{hardware, config, policy};
    auto manager = FsmRegister::instance().create("g1", context);
    const auto tick = [&]()
    {
        auto *state = manager->current();
        state->run();
        const auto next = state->check_change();
        if (next != state->name())
            manager->start(next);
    };
    const auto press = [&](joybutton button)
    {
        hardware.button = joybutton::None;
        tick();
        hardware.button = button;
        tick();
    };
    const auto selected = [&](const Shortcut &shortcut)
    {
        require(manager->current()->name() == shortcut.state &&
                    config.policy().policy_name == shortcut.policy && policy.policy_active(),
                "G1 shortcut selected the wrong state or policy");
    };

    for (const auto &shortcut : shortcuts)
    {
        hardware.button = joybutton::None;
        manager->start("GetUp");
        hardware.button = shortcut.button;
        tick();
        require(manager->current()->name() == "GetUp", "G1 shortcut bypassed standing completion");
        tick();
        require(manager->current()->name() == "GetUp", "G1 replayed an early policy request");
        press(shortcut.button);
        selected(shortcut);
    }

    // Every active policy must support all seven destinations, including
    // re-pressing its own shortcut without restarting the current state.
    for (const auto &from : shortcuts)
        for (const auto &to : shortcuts)
        {
            manager->start(from.state);
            press(joybutton::RB_DPadRight);
            selected(from); // Slot 8 is reserved.
            press(to.button);
            selected(to);
            tick(); // A held key must not trigger another transition.
            selected(to);
        }

    for (const auto &shortcut : shortcuts)
    {
        manager->start(shortcut.state);
        const bool amp = std::string(shortcut.policy) == "amp" ||
                         std::string(shortcut.policy) == "amp_zlab";
        press(amp ? joybutton::LB_X : joybutton::Y);
        require(manager->current()->name() == "Passive" && !policy.policy_active(),
                "G1 policy exit did not stop inference");
        manager->start(shortcut.state);
        press(amp ? joybutton::Y : joybutton::RB_A);
        require(manager->current()->name() == "GetUp" && !policy.policy_active(),
                "G1 policy recovery did not return to GetUp");

        std::filesystem::remove(files.root / "robot" / shortcut.policy / "config.yaml");
        manager->start(shortcut.state);
        require(!policy.policy_active(), "Missing G1 policy became active");
        tick();
        require(manager->current()->name() == "Passive", "Missing G1 policy did not fall back to Passive");
    }
}

void check_g1_motion_completion()
{
    Files files;
    std::ofstream(files.root / "robot" / "base.yaml") << R"(robot:
  dt: 0.001
  num_of_dofs: 1
  default_dof_pos: [0]
  fixed_kp: [10]
  fixed_kd: [2]
)";
    for (const auto *id : {"dance_102", "gangnam_style", "mimic"})
    {
        const auto directory = files.root / "robot" / id;
        std::ofstream(directory / "config.yaml") << R"(policy:
  model_name: test.fake
  num_of_dofs: 1
  observations: [commands]
  num_observations: 3
  motion_file: reference.csv
)";
        std::ofstream(directory / "reference.csv") << "0,0,1,0,0,0,1,0\n0,0,1,0,0,0,1,0\n";
    }
    PolicyConfigLoader config;
    require(config.load(files.root, "robot"), "Cannot load G1 motion test configuration");
    Hardware hardware(config.robot());
    MotionCommand motion;
    PolicyManager policy(hardware, config, files.root, motion);
    FsmContext context{hardware, config, policy};
    auto manager = FsmRegister::instance().create("g1", context);
    for (const auto *name : {"Dance102", "GangnamStyle", "Mimic"})
    {
        manager->start(name);
        require(policy.policy_active() && motion.is_loaded(), "G1 motion policy did not start");
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
        while (!policy.motion_finished() && std::chrono::steady_clock::now() < deadline)
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        require(policy.motion_finished(), "G1 reference motion did not finish");
        require(manager->current()->check_change() == "RLLocomotion",
                "Completed G1 motion did not request locomotion");
        manager->start("RLLocomotion");
        require(policy.policy_active() && config.policy().policy_name == "locomotion" && !motion.is_loaded(),
                "G1 motion completion left reference state in locomotion");
    }
}

void check_manager_loop()
{
    struct CountingState : FsmBase
    {
        explicit CountingState(std::string name) : FsmBase(std::move(name)) {}
        void run() override { ++runs; }
        void exit() override { ++exits; }
        std::atomic<int> runs{0};
        std::atomic<int> exits{0};
    };
    FsmManager manager(0.001);
    auto first = std::make_shared<CountingState>("First");
    auto second = std::make_shared<CountingState>("Second");
    manager.add(first);
    manager.add(second);
    manager.start("First");
    std::thread worker([&]() { manager.run(); });
    const auto wait_for = [](const std::atomic<int> &count, int minimum)
    {
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
        while (count.load() < minimum && std::chrono::steady_clock::now() < deadline)
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        return count.load() >= minimum;
    };
    const bool continued = wait_for(first->runs, 3);
    const int before_invalid = first->runs.load();
    manager.request("Unknown");
    const bool ignored_invalid = wait_for(first->runs, before_invalid + 3);
    manager.request("Second");
    const bool switched = wait_for(second->runs, 3);
    manager.stop();
    worker.join();
    require(continued, "Unchanged state terminated the FSM control loop");
    require(ignored_invalid, "Unknown state request terminated the FSM control loop");
    require(switched && first->exits == 1 && manager.current() == second.get(),
            "Scheduled state transition failed");
    require(!manager.running(), "FSM loop remained marked running after stop");
}

void check_velocity_display(const std::string &robot_type)
{
    Files files;
    PolicyConfigLoader config;
    require(config.load(files.root, "robot"), "Cannot load velocity display configuration");
    Hardware hardware(config.robot());
    hardware.axis = {0.2F, -0.4F, -0.3F, 0.0F};
    MotionCommand motion;
    PolicyManager policy(hardware, config, files.root, motion);
    FsmContext context{hardware, config, policy};
    std::shared_ptr<FsmBase> state;
    if (robot_type == "g1")
        state = std::make_shared<g1::RLLocomotion>(context);
    else if (robot_type == "g1_amp")
        state = std::make_shared<g1::AmpLocomotion>(context);
    else if (robot_type == "g1_amp_zlab")
        state = std::make_shared<g1::AmpZlab>(context);
    else if (robot_type == "g1_zlocomotion")
        state = std::make_shared<g1::ZLocomotion>(context);
    else if (robot_type == "adam")
        state = std::make_shared<adam::RLLocomotion>(context);
    else
        state = std::make_shared<g1_23::RLLocomotion>(context);
    const std::string expected = "vx=+0.80 m/s  vy=-0.30 m/s  yaw=+1.20 rad/s";
    std::ostringstream output;
    {
        struct Redirect
        {
            explicit Redirect(std::ostringstream &stream) : previous(std::cout.rdbuf(stream.rdbuf())) {}
            ~Redirect() { std::cout.rdbuf(previous); }
            std::streambuf *previous;
        } redirect(output);
        state->enter();
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
        while (output.str().find(expected) == std::string::npos && std::chrono::steady_clock::now() < deadline)
        {
            state->run();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        state->exit();
    }
    const auto text = output.str();
    const auto velocity = text.find(expected);
    require(velocity != std::string::npos, "Locomotion did not display the scaled velocity command");
    require(text.find("\r[Velocity cmd]") != std::string::npos, "Velocity display did not refresh in place");
    const auto exit_message = text.find("Exit", velocity);
    require(exit_message != std::string::npos && text.find('\n', velocity) < exit_message,
            "State exit did not finish the velocity line before logging");
}

}

int main(int argc, char **argv)
{
    try
    {
        PolicyRegister::instance().register_backend(".fake", []() { return std::make_unique<Backend>(); });
        if (argc == 2 && std::string(argv[1]) == "adam")
        {
            check_robot("adam");
            check_velocity_display("adam");
            std::cout << "Adam FSM tests passed\n";
            return 0;
        }
        if (argc != 1)
            throw std::runtime_error("Usage: test_fsm [adam]");
        check_manager_loop();
        check_robot("g1");
        check_g1_policy_shortcuts();
        check_g1_motion_completion();
        check_robot("g1_23");
        check_robot("adam");
        check_velocity_display("g1");
        check_velocity_display("g1_23");
        check_velocity_display("g1_amp");
        check_velocity_display("g1_amp_zlab");
        check_velocity_display("g1_zlocomotion");
        check_velocity_display("adam");
        std::cout << "FSM tests passed\n";
    }
    catch (const std::exception &error)
    {
        std::cerr << error.what() << '\n';
        return 1;
    }
}

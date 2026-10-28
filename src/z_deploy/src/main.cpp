#include <csignal>
#include <filesystem>
#include <iostream>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <command/motion_command.hpp>
#include <config/config_loader.hpp>
#include <fsm/robot/adam/adam_fsm.hpp>
#include <fsm/robot/dr02/dr02_fsm.hpp>
#include <fsm/robot/g1/g1_fsm.hpp>
#include <fsm/robot/g1_23/g1_23_fsm.hpp>
#include <fsm/fsm_register.hpp>
#include <hardware/hardware_sim.hpp>
#include <policy/policy_manager.hpp>

namespace
{
FsmManager *active_fsm = nullptr;

void signal_handler(int)
{
    if (active_fsm != nullptr)
        active_fsm->stop();
}

void usage(const char *program)
{
    std::cout << "Usage: " << program
              << " --robot <g1|g1_23|dr02|adam> --mode <sim|real>\n";
}

std::filesystem::path policy_root()
{
    return std::filesystem::path(Z_POLICY_ROOT);
}
} // namespace

int main(int argc, char **argv)
{
    std::string robot;
    std::string mode="sim";
    const auto policies = policy_root();

    for (int i = 1; i < argc; ++i)
    {
        const std::string arg = argv[i];
        if (arg == "--help" || arg == "-h")
        {
            usage(argv[0]);
            return 0;
        }
        if (i + 1 >= argc)
        {
            std::cerr << "Missing value for " << arg << '\n';
            usage(argv[0]);
            return 2;
        }
        if (arg == "--robot" || arg == "-r")
            robot = argv[++i];
        else if (arg == "--mode" || arg == "-m")
            mode = argv[++i];
        else
        {
            std::cerr << "Unknown argument: " << arg << '\n';
            usage(argv[0]);
            return 2;
        }
    }

    if (robot.empty() || mode.empty() || (mode != "sim" && mode != "real"))
    {
        usage(argv[0]);
        return 2;
    }

    // Simulation backends create rclcpp nodes in their constructors. ROS2 must
    // be initialized before those objects are created.
    rclcpp::init(argc, argv);

    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    PolicyConfigLoader config;
    // Only load the robot base configuration here. Policies are selected by FSM states.
    if (!config.load(policies, robot))
    {
        std::cerr << "Failed to load configuration for robot: " << robot << '\n';
        rclcpp::shutdown();
        return 1;
    }

    if (mode != "sim")
    {
        std::cerr << "The current hardware implementation supports simulation mode only\n";
        rclcpp::shutdown();
        return 1;
    }
    auto hardware = std::make_unique<HardwareSim>(config.robot());

    MotionCommand motion;
    PolicyManager policy(*hardware, config, policies, motion);
    FsmContext context{*hardware, config, policy};
    auto fsm = FsmRegister::instance().create(robot, context);
    if (!fsm)
    {
        std::cerr << "No FSM registered for robot: " << robot << '\n';
        rclcpp::shutdown();
        return 1;
    }

    hardware->start();
    active_fsm = fsm.get();
    fsm->run();
    active_fsm = nullptr;
    policy.stop_policy();
    hardware->stop();
    rclcpp::shutdown();
    return 0;
}

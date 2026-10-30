#include <csignal>
#include <filesystem>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <command/motion_command.hpp>
#include <config/config_loader.hpp>
#include <fsm/robot/adam/adam_fsm.hpp>
#include <fsm/robot/dr02/dr02_fsm.hpp>
#include <fsm/robot/g1/g1_fsm.hpp>
#include <fsm/robot/g1_23/g1_23_fsm.hpp>
#include <fsm/fsm_register.hpp>
#include <hardware/hardware_register.hpp>
#include <hardware/hardware_sim.hpp>
#if Z_DEPLOY_HAS_UNITREE
#include <hardware/real/hardware_g1.hpp>
#include <hardware/real/hardware_g1_23.hpp>
#endif
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
              << " --robot <g1|g1_23|dr02|adam> --mode <sim|real>\n"
              << "Real mode supports g1 and g1_23 and reads the network interface from Z_NETIF.\n";
}

std::filesystem::path policy_root()
{
    return std::filesystem::path(Z_POLICY_ROOT);
}
} // namespace

int main(int argc, char **argv)
{
    tui::print_banner();
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

    if (robot.empty() || mode.empty())
    {
        usage(argv[0]);
        return 2;
    }
    if (!HardwareRegister::instance().supported(mode, robot))
    {
        std::cerr << "No hardware backend registered for mode=" << mode << ", robot=" << robot << '\n';
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

    try
    {
        auto hardware = HardwareRegister::instance().create(mode, config.robot());
        if (!hardware)
            throw std::runtime_error("No hardware backend registered for mode=" + mode + ", robot=" + robot);

        MotionCommand motion;
        PolicyManager policy(*hardware, config, policies, motion);
        FsmContext context{*hardware, config, policy};
        // Read the real remote before state entry so held buttons are ignored.
        hardware->start();
        auto fsm = FsmRegister::instance().create(robot, context);
        if (!fsm)
            throw std::runtime_error("No FSM registered for robot: " + robot);

        tui::policy_bindings(robot, FsmRegister::instance().key_bindings(robot));
        active_fsm = fsm.get();
        fsm->run();
        active_fsm = nullptr;
        policy.stop_policy();
        hardware->stop();
    }
    catch (const std::exception &e)
    {
        active_fsm = nullptr;
        std::cerr << "Deployment failed: " << e.what() << '\n';
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}

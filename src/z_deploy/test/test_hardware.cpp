#include <chrono>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <unistd.h>
#include <rclcpp/rclcpp.hpp>

#include <config/config_loader.hpp>
#include "hardware.hpp"

namespace
{
void usage(const char *program)
{
    std::cout << "Usage: " << program << " --robot <g1|g1_23|dr02|adam> --mode <sim|real>\n"
              << "Receive and display IMU/joint positions only. Real mode reads Z_NETIF.\n";
}

void print_vector(std::ostream &out, const char *label, const std::vector<float> &values)
{
    out << label;
    for (const float value : values)
        out << ' ' << std::setw(9) << value;
    out << '\n';
}

std::string display(const RobotConfig &config, const std::string &mode,
                    const RobotState &state, bool stale)
{
    std::ostringstream out;
    out << std::fixed << std::setprecision(4);
    out << "Hardware monitor | robot=" << config.robot_name << " | mode=" << mode
        << " | receive only | " << (stale ? "STALE: no new state for 1 s" : "receiving") << '\n';
    print_vector(out, "IMU quaternion (wxyz):", state.base_quat);
    print_vector(out, "IMU RPY (rad):        ", state.base_euler);
    print_vector(out, "IMU gyro (rad/s):     ", state.base_ang_vel);
    print_vector(out, "IMU accel (m/s^2):    ", state.base_lin_acc);
    out << "Joint positions (rad, base.yaml order):\n";
    for (std::size_t i = 0; i < state.joint_pos.size(); ++i)
    {
        const auto name = i < config.joint_names.size() ? config.joint_names[i] : "joint_" + std::to_string(i);
        out << "  [" << std::setw(2) << i << "] " << std::left << std::setw(32) << name
            << std::right << std::setw(10) << state.joint_pos[i] << '\n';
    }
    out << "Ctrl+C to exit\n";
    return out.str();
}
} // namespace

int main(int argc, char **argv)
{
    std::string robot;
    std::string mode = "sim";
    for (int i = 1; i < argc; ++i)
    {
        const std::string arg = argv[i];
        if (arg == "--help" || arg == "-h")
        {
            usage(argv[0]);
            return 0;
        }
        if ((arg != "--robot" && arg != "-r" && arg != "--mode" && arg != "-m") || i + 1 >= argc)
        {
            std::cerr << "Invalid or missing argument: " << arg << '\n';
            usage(argv[0]);
            return 2;
        }
        if (arg == "--robot" || arg == "-r")
            robot = argv[++i];
        else
            mode = argv[++i];
    }
    if (robot.empty())
    {
        usage(argv[0]);
        return 2;
    }
    if (!HardwareReceiver::supported(mode, robot))
    {
        std::cerr << "No receiver available for mode=" << mode << ", robot=" << robot << '\n';
        return 2;
    }

    rclcpp::init(argc, argv);
    try
    {
        PolicyConfigLoader config;
        if (!config.load(std::filesystem::path(Z_POLICY_ROOT), robot))
            throw std::runtime_error("Cannot load base.yaml for " + robot);
        HardwareReceiver hardware(mode, config.robot());
        std::cout << "Waiting for joint state and IMU (receive only)...\n";
        const bool terminal = isatty(STDOUT_FILENO) != 0;
        while (rclcpp::ok())
        {
            hardware.poll();
            const auto snapshot = hardware.get_snapshot();
            if (snapshot.ready())
            {
                const auto now = std::chrono::steady_clock::now();
                const bool stale = now - snapshot.joints_updated >= std::chrono::seconds(1) ||
                                   now - snapshot.imu_updated >= std::chrono::seconds(1);
                if (terminal)
                    std::cout << "\033[H\033[2J";
                std::cout << display(config.robot(), mode, snapshot.state, stale) << std::flush;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << "Hardware monitor failed: " << e.what() << '\n';
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}

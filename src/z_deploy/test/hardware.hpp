#pragma once

#include <memory>
#include <stdexcept>
#include <string>

#include "hardware_base.hpp"
#include "hardware_sim.hpp"
#if Z_DEPLOY_HAS_UNITREE
#include "hardware_g1.hpp"
#include "hardware_g1_23.hpp"
#endif

inline bool receiver_supported(const std::string &mode, const std::string &robot)
{
    if (mode == "sim")
        return robot == "g1" || robot == "g1_23" || robot == "dr02" || robot == "adam";
#if Z_DEPLOY_HAS_UNITREE
    if (mode == "real")
        return robot == "g1" || robot == "g1_23";
#endif
    return false;
}

inline std::unique_ptr<HardwareReceiver> create_receiver(const std::string &mode,
                                                         const RobotConfig &config)
{
    if (!receiver_supported(mode, config.robot_name))
        throw std::invalid_argument("Unsupported receiver mode/robot");
    if (mode == "sim")
        return std::make_unique<SimHardwareReceiver>(config);
#if Z_DEPLOY_HAS_UNITREE
    if (config.robot_name == "g1")
        return std::make_unique<G1HardwareReceiver>(config);
    if (config.robot_name == "g1_23")
        return std::make_unique<G1_23HardwareReceiver>(config);
#endif
    throw std::invalid_argument("No receiver available");
}

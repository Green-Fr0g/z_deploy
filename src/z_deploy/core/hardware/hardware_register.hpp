#pragma once

#include <functional>
#include <map>
#include <memory>
#include <string>
#include <utility>

#include <hardware/hardware_base.hpp>

class HardwareRegister
{
public:
    using Creator = std::function<std::unique_ptr<HardwareBase>(const RobotConfig &)>;

    static HardwareRegister &instance()
    {
        static HardwareRegister registry;
        return registry;
    }

    void add(std::string mode, std::string robot, Creator creator)
    {
        if (creator)
            creators_[{std::move(mode), std::move(robot)}] = std::move(creator);
    }

    std::unique_ptr<HardwareBase> create(const std::string &mode, const RobotConfig &config) const
    {
        const auto it = creators_.find({mode, config.robot_name});
        if (it == creators_.end())
            return nullptr;
        return it->second(config);
    }

    bool supported(const std::string &mode, const std::string &robot) const
    {
        return creators_.count({mode, robot}) != 0;
    }

private:
    HardwareRegister() = default;
    std::map<std::pair<std::string, std::string>, Creator> creators_;
};

#ifndef Z_CONCAT_DEFINED
#define Z_CONCAT_DEFINED
#define Z_CONCAT2(x, y) x##y
#define Z_CONCAT(x, y) Z_CONCAT2(x, y)
#endif

// Include a backend's header to register its mode/robot combination.
#define Z_REGISTER_HARDWARE(MODE, ROBOT, TYPE)                                      \
    namespace                                                                      \
    {                                                                              \
    const bool Z_CONCAT(z_hardware_reg_, __COUNTER__) = []() {                       \
        HardwareRegister::instance().add(MODE, ROBOT, [](const RobotConfig &config)  \
        {                                                                          \
            return std::make_unique<TYPE>(config);                                  \
        });                                                                        \
        return true;                                                               \
    }();                                                                           \
    }

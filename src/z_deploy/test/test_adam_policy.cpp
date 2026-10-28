#include <cmath>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <vector>

#include <config/config_loader.hpp>
#include <policy/observation/obs_collection.hpp>
#include <policy/runtime/policy_backend.hpp>

int main()
{
    try
    {
        PolicyConfigLoader config;
        if (!config.load(Z_ADAM_POLICY_ROOT, "adam", "locomotion"))
            throw std::runtime_error("Cannot load Adam locomotion config");
        const auto &robot = config.robot();
        const auto &policy = config.policy();
        if (robot.dof != 27 || policy.dof != 27 || policy.input_dim != 450 ||
            robot.joint_names.size() != 27 || policy.joint_mapping.size() != 27)
            throw std::runtime_error("Invalid Adam joint or observation dimensions");
        for (int i = 0; i < policy.dof; ++i)
            if (policy.joint_mapping[static_cast<std::size_t>(i)] != i)
                throw std::runtime_error("Adam policy and actuator order disagree");

        ObsCollection observations;
        observations.configure(policy);
        if (observations.value().size() != 450)
            throw std::runtime_error("Adam history does not form a 450-dimensional observation");
        TorchPolicyBackend backend;
        if (!backend.load(policy, config.policy_dir() / policy.model_name))
            throw std::runtime_error("Cannot load Adam TorchScript policy");
        std::vector<float> input(450, 0.0F), action;
        if (!backend.infer(input, action) || action.size() != 27)
            throw std::runtime_error("Adam policy must accept 450 observations and return 27 actions");
        for (const auto value : action)
            if (!std::isfinite(value))
                throw std::runtime_error("Adam policy returned a non-finite action");
        std::cout << "Adam policy inference passed: 450 -> 27\n";
    }
    catch (const std::exception &error)
    {
        std::cerr << error.what() << '\n';
        return 1;
    }
}

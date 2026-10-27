#pragma once

#include <filesystem>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

#include <policy/runtime/policy_backend.hpp>

// Extension-based factory for PolicyBackend.
// Built-ins: ".pt", ".onnx".
class PolicyRegister
{
public:
    using BackendPtr = std::unique_ptr<PolicyBackend>;
    using BackendFactory = std::function<BackendPtr()>;

    static PolicyRegister &instance()
    {
        static PolicyRegister registry;
        return registry;
    }

    bool register_backend(const std::string &extension, BackendFactory factory)
    {
        if (!factory)
            return false;
        std::lock_guard<std::mutex> lock(mutex_);
        factories_[extension] = std::move(factory);
        return true;
    }

    // Create backend from model file name by its extension.
    BackendPtr create(const std::string &model_name) const
    {
        const auto extension = std::filesystem::path(model_name).extension().string();
        BackendFactory factory;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            const auto it = factories_.find(extension);
            if (it == factories_.end())
                return nullptr;
            factory = it->second;
        }
        return factory();
    }

    bool is_registered(const std::string &extension) const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return factories_.count(extension) != 0;
    }

private:
    PolicyRegister()
    {
        register_builtins();
    }

    void register_builtins()
    {
        factories_[".pt"] = []() -> BackendPtr {
            return std::make_unique<TorchPolicyBackend>();
        };
        factories_[".onnx"] = []() -> BackendPtr {
            return std::make_unique<OnnxPolicyBackend>();
        };
    }

    mutable std::mutex mutex_;
    std::unordered_map<std::string, BackendFactory> factories_;
};

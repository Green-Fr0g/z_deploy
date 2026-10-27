#pragma once

#include <array>
#include <cstddef>
#include <filesystem>
#include <iostream>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <z_deploy/config_types.hpp>

#ifndef Z_DEPLOY_HAS_TORCH
#if __has_include(<torch/script.h>)
#define Z_DEPLOY_HAS_TORCH 1
#include <ATen/Parallel.h>
#include <torch/script.h>
#else
#define Z_DEPLOY_HAS_TORCH 0
#endif
#elif Z_DEPLOY_HAS_TORCH
#include <ATen/Parallel.h>
#include <torch/script.h>
#endif

#ifndef Z_DEPLOY_HAS_ONNX
#if __has_include(<onnxruntime_cxx_api.h>)
#define Z_DEPLOY_HAS_ONNX 1
#else
#define Z_DEPLOY_HAS_ONNX 0
#endif
#endif

#if Z_DEPLOY_HAS_ONNX
#include <onnxruntime_cxx_api.h>
#endif

class PolicyBackend
{
public:
    virtual ~PolicyBackend() = default;

    virtual bool load(const PolicyConfig &config,
                      const std::filesystem::path &model_path) = 0;
    virtual bool infer(const std::vector<float> &observation,
                       std::vector<float> &action) = 0;
    virtual std::size_t input_dim() const = 0;
    virtual std::size_t output_dim() const = 0;
};


class TorchPolicyBackend final : public PolicyBackend
{
public:
    bool load(const PolicyConfig &config,
              const std::filesystem::path &model_path) override
    {
        config_ = config;
#if Z_DEPLOY_HAS_TORCH
        try
        {
            at::set_num_threads(1);
            module_ = torch::jit::load(model_path.string());
            module_.eval();
            loaded_ = true;
            return true;
        }
        catch (const std::exception &e)
        {
            std::cerr << "[TorchPolicy] Failed to load " << model_path << ": " << e.what() << '\n';
            loaded_ = false;
            return false;
        }
#else
        (void)model_path;
        loaded_ = false;
        return false;
#endif
    }

    bool infer(const std::vector<float> &observation,
               std::vector<float> &action) override
    {
        if (!loaded_ || observation.size() != static_cast<std::size_t>(config_.input_dim))
            return false;
#if Z_DEPLOY_HAS_TORCH
        try
        {
            torch::NoGradGuard guard;
            at::set_num_threads(1);
            auto input = torch::tensor(observation, torch::TensorOptions().dtype(torch::kFloat32))
                             .reshape({1, static_cast<int64_t>(observation.size())});
            auto output = module_.forward({input}).toTensor().to(torch::kCPU).contiguous();
            const auto *data = output.data_ptr<float>();
            const auto numel = static_cast<std::size_t>(output.numel());
            action.assign(data, data + numel);
            if (action.size() != static_cast<std::size_t>(config_.dof))
            {
                std::cerr << "[TorchPolicy] Action size " << action.size()
                          << " != dof " << config_.dof << '\n';
                return false;
            }
            return true;
        }
        catch (const std::exception &e)
        {
            std::cerr << "[TorchPolicy] Inference error: " << e.what() << '\n';
            return false;
        }
#else
        (void)action;
        return false;
#endif
    }

    std::size_t input_dim() const override
    {
        return static_cast<std::size_t>(config_.input_dim);
    }

    std::size_t output_dim() const override
    {
        return static_cast<std::size_t>(config_.dof);
    }

private:
    PolicyConfig config_;
    bool loaded_{false};
#if Z_DEPLOY_HAS_TORCH
    torch::jit::script::Module module_;
#endif
};

class OnnxPolicyBackend final : public PolicyBackend
{
public:
    bool load(const PolicyConfig &config,
              const std::filesystem::path &model_path) override
    {
        config_ = config;
#if Z_DEPLOY_HAS_ONNX
        try
        {
            env_.emplace(ORT_LOGGING_LEVEL_WARNING, "z_deploy");
            session_options_.SetIntraOpNumThreads(1);
            session_options_.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
            session_.emplace(*env_, model_path.c_str(), session_options_);
            if (session_->GetInputCount() != 1 || session_->GetOutputCount() != 1)
                throw std::runtime_error("Expected one observation input and one action output");
            const auto input_info = session_->GetInputTypeInfo(0);
            const auto output_info = session_->GetOutputTypeInfo(0);
            const auto input_tensor = input_info.GetTensorTypeAndShapeInfo();
            const auto output_tensor = output_info.GetTensorTypeAndShapeInfo();
            const auto input_shape = input_tensor.GetShape();
            const auto output_shape = output_tensor.GetShape();
            if (input_tensor.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT ||
                output_tensor.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT ||
                input_shape.size() != 2 || output_shape.size() != 2 ||
                (input_shape[0] > 0 && input_shape[0] != 1) ||
                (output_shape[0] > 0 && output_shape[0] != 1) ||
                input_shape[1] != config.input_dim || output_shape[1] != config.dof)
                throw std::runtime_error("Model tensor shapes/types do not match the policy config");
            {
                auto input_name = session_->GetInputNameAllocated(0, allocator_);
                input_name_ = input_name.get();
            }
            {
                auto output_name = session_->GetOutputNameAllocated(0, allocator_);
                output_name_ = output_name.get();
            }
            loaded_ = true;
            return true;
        }
        catch (const std::exception &e)
        {
            std::cerr << "[OnnxPolicy] Failed to load " << model_path << ": " << e.what() << '\n';
            loaded_ = false;
            return false;
        }
#else
        (void)model_path;
        std::cerr << "[OnnxPolicy] ONNX Runtime is unavailable\n";
        loaded_ = false;
        return false;
#endif
    }

    bool infer(const std::vector<float> &observation,
               std::vector<float> &action) override
    {
        if (!loaded_ || observation.size() != static_cast<std::size_t>(config_.input_dim))
            return false;
#if Z_DEPLOY_HAS_ONNX
        try
        {
            std::array<int64_t, 2> shape{1, static_cast<int64_t>(observation.size())};
            auto input_tensor = Ort::Value::CreateTensor<float>(memory_info_,
                                                                const_cast<float *>(observation.data()),
                                                                observation.size(),
                                                                shape.data(),
                                                                shape.size());
            const char *input_names[] = {input_name_.c_str()};
            const char *output_names[] = {output_name_.c_str()};
            auto output_tensors = session_->Run(Ort::RunOptions{nullptr},
                                                input_names,
                                                &input_tensor,
                                                1,
                                                output_names,
                                                1);
            const auto *data = output_tensors.front().GetTensorMutableData<float>();
            const auto count = output_tensors.front().GetTensorTypeAndShapeInfo().GetElementCount();
            action.assign(data, data + count);
            return action.size() == static_cast<std::size_t>(config_.dof);
        }
        catch (const std::exception &e)
        {
            std::cerr << "[OnnxPolicy] Inference error: " << e.what() << '\n';
            return false;
        }
#else
        (void)action;
        return false;
#endif
    }

    std::size_t input_dim() const override
    {
        return static_cast<std::size_t>(config_.input_dim);
    }

    std::size_t output_dim() const override
    {
        return static_cast<std::size_t>(config_.dof);
    }

private:
    PolicyConfig config_;
    bool loaded_{false};
#if Z_DEPLOY_HAS_ONNX
    std::optional<Ort::Env> env_;
    Ort::SessionOptions session_options_;
    Ort::MemoryInfo memory_info_{Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault)};
    std::optional<Ort::Session> session_;
    Ort::AllocatorWithDefaultOptions allocator_;
    std::string input_name_;
    std::string output_name_;
#endif
};

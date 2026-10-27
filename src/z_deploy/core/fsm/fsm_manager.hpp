#pragma once

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>

#include <fsm/fsm_base.hpp>

class FsmManager
{
public:
    explicit FsmManager(double control_dt = 0.002)
        : period_(std::chrono::duration_cast<std::chrono::steady_clock::duration>(
              std::chrono::duration<double>(control_dt > 0.0 ? control_dt : 0.002)))
    {
    }

    void add(std::shared_ptr<FsmBase> state)
    {
        if (!state)
            return;
        states_[state->name()] = std::move(state);
    }

    void start(const std::string &name)
    {
        auto it = states_.find(name);
        if (it == states_.end())
            return;
        if (current_ != nullptr)
            current_->exit();
        current_ = it->second.get();
        current_->enter();
    }

    void request(const std::string &name)
    {
        std::lock_guard<std::mutex> lock(pending_mutex_);
        pending_ = name;
    }

    void run()
    {
        bool expected = false;
        if (!running_.compare_exchange_strong(expected, true))
            return;

        auto next_tick = std::chrono::steady_clock::now();
        while (running_.load(std::memory_order_acquire))
        {
            if (current_ == nullptr)
                break;

            current_->run();

            std::string next;
            {
                std::lock_guard<std::mutex> lock(pending_mutex_);
                next = std::move(pending_);
                pending_.clear();
            }
            if (next.empty())
                next = current_->check_change();
            if (next != current_->name())
            {
                const auto it = states_.find(next);
                if (it != states_.end())
                {
                    current_->exit();
                    current_ = it->second.get();
                    current_->enter();
                }
            }

            next_tick += period_;
            std::this_thread::sleep_until(next_tick);

            const auto now = std::chrono::steady_clock::now();
            if (now > next_tick + period_)
                next_tick = now;
        }
        running_.store(false, std::memory_order_release);
    }

    void stop()
    {
        running_.store(false, std::memory_order_release);
    }

    bool running() const
    {
        return running_.load(std::memory_order_acquire);
    }

    FsmBase *current() const
    {
        return current_;
    }

private:
    std::unordered_map<std::string, std::shared_ptr<FsmBase>> states_;
    FsmBase *current_{nullptr};
    std::string pending_;
    std::mutex pending_mutex_;
    std::chrono::steady_clock::duration period_;
    std::atomic<bool> running_{false};
};

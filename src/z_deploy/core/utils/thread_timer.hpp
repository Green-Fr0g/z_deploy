#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <exception>
#include <functional>
#include <iostream>
#include <mutex>
#include <thread>
#include <utility>

class ThreadTimer
{
private:
    using Clock = std::chrono::steady_clock;

    void run(Clock::duration period, const std::function<void()> &callback)
    {
        auto next = Clock::now();
        while (running_.load())
        {
            try
            {
                callback();
            }
            catch (const std::exception &e)
            {
                std::cerr << "[ThreadTimer] " << e.what() << '\n';
            }

            next += period;
            const auto now = Clock::now();
            if (next < now)
                next = now + period; // deadline missed: skip instead of bursting

            std::unique_lock<std::mutex> lk(mutex_);
            cv_.wait_until(lk, next, [this]() { return !running_.load(); });
        }
    }

    std::atomic<bool> running_{false};
    std::thread thread_;
    std::mutex mutex_;
    std::condition_variable cv_;

public:
    ThreadTimer() = default;

    ThreadTimer(const ThreadTimer &) = delete;
    ThreadTimer &operator=(const ThreadTimer &) = delete;

    ~ThreadTimer()
    {
        stop();
    }

    // Runs `callback` on its own thread every `interval_sec`. Returns false if
    // the timer is already running or the interval rounds to zero.
    bool start(double interval_sec, std::function<void()> callback)
    {
        const auto period = std::chrono::duration_cast<Clock::duration>(
            std::chrono::duration<double>(interval_sec));
        if (running_.load() || period <= Clock::duration::zero())
            return false;

        running_.store(true);
        thread_ = std::thread([this, period, callback = std::move(callback)]()
                              { run(period, callback); });
        return true;
    }

    void stop()
    {
        if (!running_.exchange(false))
            return;

        cv_.notify_all();
        if (thread_.joinable())
            thread_.join();
    }
};

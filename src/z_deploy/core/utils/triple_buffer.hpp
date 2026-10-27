#pragma once

#include <atomic>
#include <cstdint>
#include <utility>


// Lock-free triple buffer for exactly one producer and one consumer thread.
//
// Producer thread: publish() only.
// Consumer thread: read() only.
//
// Do not call read() from the producer thread (including read-modify-publish).
// Keep a producer-side staging object and publish() the updated snapshot instead.
template <typename T>
class TripleBuffer
{
public:
    TripleBuffer() = default;

    explicit TripleBuffer(const T &initial)
    {
        slots_[0] = initial;
        slots_[1] = initial;
        slots_[2] = initial;
        latest_.store(0U, std::memory_order_relaxed);
        write_index_ = 1U;
    }

    TripleBuffer(const TripleBuffer &) = delete;
    TripleBuffer &operator=(const TripleBuffer &) = delete;

    void publish(const T &value)
    {
        slots_[write_index_] = value;
        const auto previous = latest_.exchange(write_index_, std::memory_order_acq_rel);
        write_index_ = previous;
    }

    void publish(T &&value)
    {
        slots_[write_index_] = std::move(value);
        const auto previous = latest_.exchange(write_index_, std::memory_order_acq_rel);
        write_index_ = previous;
    }

    T read() const
    {
        return slots_[latest_.load(std::memory_order_acquire)];
    }

private:
    T slots_[3]{};
    std::atomic<std::uint32_t> latest_{0U};
    std::uint32_t write_index_{1U}; // producer thread only
};

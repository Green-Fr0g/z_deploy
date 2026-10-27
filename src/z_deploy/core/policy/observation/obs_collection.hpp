#pragma once

#include <algorithm>
#include <cstddef>
#include <deque>
#include <string>
#include <utility>
#include <vector>

#include <z_deploy/config_types.hpp>

// Stores observation terms, maintains history frames, and flattens to policy input.
// push() advances history once per observation tick; indices count observation steps.
// The flatten order follows `policy.observation_history_priority`:
//   "time" (default) -> frame-major, "term" -> term-major; see build_output().
class ObsCollection
{
public:
    ObsCollection() = default;

    void configure(const PolicyConfig &policy)
    {
        terms_.clear();
        history_indices_.clear();
        history_length_ = 0;
        frame_dim_ = 0;
        term_priority_ = policy.observation_history_priority == "term";

        const auto count = std::min(policy.observations.size(), policy.observation_dims.size());
        terms_.reserve(count);
        for (std::size_t i = 0; i < count; ++i)
        {
            const auto dim = static_cast<std::size_t>(std::max(policy.observation_dims[i], 0));
            if (policy.observations[i].empty() || dim == 0)
                continue;

            terms_.push_back({policy.observations[i], dim, std::vector<float>(dim, 0.0F)});
            frame_dim_ += dim;
        }

        history_indices_.reserve(policy.observation_history.size());
        for (const int history : policy.observation_history)
        {
            const auto index = static_cast<std::size_t>(std::max(history, 0));
            history_length_ = std::max(history_length_, index);
            history_indices_.push_back(index);
        }

        reset();
    }

    void set(const std::string &name, const std::vector<float> &values)
    {
        for (auto &term : terms_)
        {
            if (term.name != name)
                continue;
            if (values.size() == term.dimension)
                term.values = values;
            return;
        }
    }

    void reset()
    {
        history_.clear();
        for (auto &term : terms_)
            std::fill(term.values.begin(), term.values.end(), 0.0F);
        output_.assign(frame_dim_ * history_indices_.size(), 0.0F);
    }

    void push()
    {
        auto frame = make_frame();
        // Seed all slots on the first policy step, including after a reset.
        if (history_.empty())
            history_.assign(history_length_ + 1U, frame);
        else
        {
            history_.push_front(std::move(frame));
            history_.pop_back();
        }
        build_output();
    }

    const std::vector<float> &value() const
    {
        return output_;
    }

private:
    struct Term
    {
        std::string name;
        std::size_t dimension{0};
        std::vector<float> values;
    };

    // A frame is the term values concatenated in term order.
    std::vector<float> make_frame() const
    {
        std::vector<float> frame;
        frame.reserve(frame_dim_);
        for (const auto &term : terms_)
            frame.insert(frame.end(), term.values.begin(), term.values.end());
        return frame;
    }

    void build_output()
    {
        output_.clear();
        output_.reserve(frame_dim_ * history_.size());

        // "term": term-major — each term's values across every history frame.
        if (term_priority_)
        {
            std::size_t offset = 0;
            for (const auto &term : terms_)
            {
                for (const auto index : history_indices_)
                {
                    const auto &frame = history_[index];
                    output_.insert(output_.end(),
                                   frame.begin() + static_cast<std::ptrdiff_t>(offset),
                                   frame.begin() + static_cast<std::ptrdiff_t>(offset + term.dimension));
                }
                offset += term.dimension;
            }
            return;
        }

        // "time" (default): frame-major — every term for each history frame.
        for (const auto index : history_indices_)
            output_.insert(output_.end(), history_[index].begin(), history_[index].end());
    }

    std::vector<Term> terms_;
    std::deque<std::vector<float>> history_;
    std::vector<float> output_;
    std::vector<std::size_t> history_indices_;
    std::size_t frame_dim_{0};
    std::size_t history_length_{0};
    bool term_priority_{false};
};

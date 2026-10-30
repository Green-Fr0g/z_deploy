#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <cstdlib>
#include <mutex>
#include <sstream>
#include <string>
#include <sys/ioctl.h>
#include <unistd.h>
#include <vector>

namespace tui
{
namespace detail
{

constexpr const char *kRed = "\033[31m";
constexpr const char *kGreen = "\033[32m";
constexpr const char *kYellow = "\033[33m";
constexpr const char *kCyan = "\033[36m";
constexpr const char *kDim = "\033[2m";
constexpr const char *kReset = "\033[0m";
constexpr int kMaxBarWidth = 50;
constexpr int kCellPad = 1;

inline std::mutex mtx;
inline bool progress_dirty = false;
inline std::string progress_label;

inline const char *color_for_status(const std::string &status)
{
    if (status == "WARN" || status == "WAIT")
        return kYellow;
    if (status == "ERROR")
        return kRed;
    if (status == "INFO")
        return kCyan;
    return kGreen;
}

inline std::string timestamp()
{
    const auto now = std::chrono::system_clock::now();
    const std::time_t now_c = std::chrono::system_clock::to_time_t(now);
    std::tm tm_now{};
    localtime_r(&now_c, &tm_now);
    std::ostringstream oss;
    oss << std::put_time(&tm_now, "%H:%M:%S");
    return oss.str();
}

inline int terminal_columns()
{
    struct winsize ws{};
    if (ioctl(STDOUT_FILENO, TIOCGWINSZ, &ws) == 0 && ws.ws_col > 0)
        return static_cast<int>(ws.ws_col);
    return 80;
}

inline int codepoint_width(std::uint32_t cp)
{
    if (cp == 0 || cp < 0x20 || (cp >= 0x7F && cp < 0xA0))
        return 0;
    if (cp >= 0x0300 && cp <= 0x036F)
        return 0;
    if ((cp >= 0x1100 && cp <= 0x115F) ||
        (cp >= 0x2E80 && cp <= 0xA4CF) ||
        (cp >= 0xAC00 && cp <= 0xD7A3) ||
        (cp >= 0xF900 && cp <= 0xFAFF) ||
        (cp >= 0xFE10 && cp <= 0xFE6F) ||
        (cp >= 0xFF00 && cp <= 0xFF60) ||
        (cp >= 0xFFE0 && cp <= 0xFFE6) ||
        (cp >= 0x1F300 && cp <= 0x1FAFF) ||
        (cp >= 0x20000 && cp <= 0x3FFFD))
        return 2;
    return 1;
}

inline bool next_codepoint(const std::string &text, std::size_t i, std::size_t &n, int &w)
{
    if (i >= text.size())
        return false;
    const auto c = static_cast<unsigned char>(text[i]);
    std::uint32_t cp = c;
    n = 1;
    if ((c & 0xE0) == 0xC0 && i + 1 < text.size())
    {
        n = 2;
        cp = (static_cast<std::uint32_t>(c & 0x1F) << 6)
             | (static_cast<unsigned char>(text[i + 1]) & 0x3F);
    }
    else if ((c & 0xF0) == 0xE0 && i + 2 < text.size())
    {
        n = 3;
        cp = (static_cast<std::uint32_t>(c & 0x0F) << 12)
             | ((static_cast<unsigned char>(text[i + 1]) & 0x3F) << 6)
             | (static_cast<unsigned char>(text[i + 2]) & 0x3F);
    }
    else if ((c & 0xF8) == 0xF0 && i + 3 < text.size())
    {
        n = 4;
        cp = (static_cast<std::uint32_t>(c & 0x07) << 18)
             | ((static_cast<unsigned char>(text[i + 1]) & 0x3F) << 12)
             | ((static_cast<unsigned char>(text[i + 2]) & 0x3F) << 6)
             | (static_cast<unsigned char>(text[i + 3]) & 0x3F);
    }
    w = codepoint_width(cp);
    return true;
}

inline int display_width(const std::string &text)
{
    int width = 0;
    for (std::size_t i = 0; i < text.size();)
    {
        std::size_t n = 1;
        int w = 1;
        if (!next_codepoint(text, i, n, w))
            break;
        width += w;
        i += n;
    }
    return width;
}

inline std::string pad_cell(const std::string &text, int width, bool right_align = false)
{
    if (width <= 0)
        return {};

    const bool overflow = display_width(text) > width;
    const int limit = overflow ? std::max(width - 1, 0) : width;

    std::string out;
    int used = 0;
    for (std::size_t i = 0; i < text.size();)
    {
        std::size_t n = 1;
        int w = 1;
        if (!next_codepoint(text, i, n, w))
            break;
        if (used + w > limit)
            break;
        out.append(text, i, n);
        used += w;
        i += n;
    }
    if (overflow)
    {
        out += "…";
        ++used;
    }
    if (used >= width)
        return out;
    const std::string pad(static_cast<std::size_t>(width - used), ' ');
    return right_align ? pad + out : out + pad;
}

inline bool looks_numeric(const std::string &text)
{
    if (text.empty())
        return false;
    char *end = nullptr;
    std::strtod(text.c_str(), &end);
    return end != text.c_str() && *end == '\0';
}

inline void fit_widths(std::vector<int> &widths)
{
    const int cols = static_cast<int>(widths.size());
    const int budget = terminal_columns() - (cols + 1) - cols * (2 * kCellPad);
    if (budget <= cols)
        return;

    int total = 0;
    for (int w : widths)
        total += w;
    if (total <= budget)
        return;

    for (int &w : widths)
        w = std::max(1, w * budget / std::max(total, 1));
}

inline void finalize_progress()
{
    if (!progress_dirty)
        return;
    std::cout << '\n';
    progress_dirty = false;
    progress_label.clear();
}

inline void write_rule(const std::vector<int> &widths,
                       const char *left,
                       const char *mid,
                       const char *right)
{
    std::cout << left;
    for (std::size_t c = 0; c < widths.size(); ++c)
    {
        if (c != 0)
            std::cout << mid;
        const int inner = widths[c] + 2 * kCellPad;
        for (int i = 0; i < inner; ++i)
            std::cout << "─";
    }
    std::cout << right << '\n';
}

inline void write_row(const std::vector<std::string> &cells,
                      const std::vector<int> &widths,
                      const std::vector<char> &right_align)
{
    std::cout << "│";
    for (std::size_t c = 0; c < widths.size(); ++c)
    {
        const std::string &cell = c < cells.size() ? cells[c] : "";
        const bool align_right = c < right_align.size() && right_align[c] != 0;
        std::cout << std::string(static_cast<std::size_t>(kCellPad), ' ')
                  << pad_cell(cell, widths[c], align_right)
                  << std::string(static_cast<std::size_t>(kCellPad), ' ')
                  << "│";
    }
    std::cout << '\n';
}

} // namespace detail
inline void print_banner()
{
    std::lock_guard<std::mutex> lock(detail::mtx);
    detail::finalize_progress();
    std::cout << detail::kCyan
              << "\n"
              << "  ███████╗\n"
              << "  ╚══███╔╝\n"
              << "    ███╔╝\n"
              << "   ███╔╝\n"
              << "  ███████╗\n"
              << "  ╚══════╝\n"
              << "\033[37m" << detail::kReset << "\n" << std::flush;
}


inline void message(const std::string &status, const std::string &text)
{
    std::lock_guard<std::mutex> lock(detail::mtx);
    detail::finalize_progress();

    constexpr int kStatusWidth = 5;
    const char *color = detail::color_for_status(status);
    std::cout << '[' << detail::kDim << detail::timestamp() << detail::kReset << "] "
              << '[' << color << std::left << std::setw(kStatusWidth) << status << std::right
              << detail::kReset << "]  " << text << '\n'
              << std::flush;
}

// Share the progress line so log messages and state transitions finish it cleanly.
inline void velocity(float x, float y, float yaw, bool force = false)
{
    std::lock_guard<std::mutex> lock(detail::mtx);
    static auto next_refresh = std::chrono::steady_clock::time_point{};
    static std::string previous;
    constexpr const char *label = "Velocity cmd";
    const bool active = detail::progress_dirty && detail::progress_label == label;
    const auto now = std::chrono::steady_clock::now();
    if (!force && active && now < next_refresh)
        return;
    next_refresh = now + std::chrono::milliseconds(100);

    // Avoid displaying negative zero after rounding to two decimal places.
    const auto rounded_zero = [](float value) { return std::fabs(value) < 0.005F ? 0.0F : value; };
    std::ostringstream line;
    line << '[' << label << "] " << std::fixed << std::setprecision(2) << std::showpos
         << "vx=" << rounded_zero(x) << " m/s  "
         << "vy=" << rounded_zero(y) << " m/s  "
         << "yaw=" << rounded_zero(yaw) << " rad/s";
    const auto text = line.str();
    if (!force && active && text == previous)
        return;
    if (detail::progress_dirty && !active)
        detail::finalize_progress();

    std::cout << '\r' << text << "\033[K" << std::flush;
    detail::progress_dirty = true;
    detail::progress_label = label;
    previous = text;
}

inline bool progress(double current, double total = 100.0, const std::string &label = {})
{
    std::lock_guard<std::mutex> lock(detail::mtx);

    if (total <= 0.0)
        total = 1.0;
    if (current < 0.0)
        current = 0.0;

    double ratio = current / total;
    if (ratio < 0.0)
        ratio = 0.0;
    if (ratio > 1.0 || std::fabs(total - current) < 1e-4)
        ratio = 1.0;
    const bool completed = ratio >= 1.0;

    constexpr int kLabelWidth = 12;
    std::string prefix;
    if (!label.empty())
        prefix = detail::pad_cell("[" + label + "]", kLabelWidth) + "  ";

    int percent = static_cast<int>(std::round(ratio * 100.0));
    percent = std::clamp(percent, 0, 100);
    std::ostringstream suffix_stream;
    suffix_stream << ' ' << std::right << std::setw(3) << percent << '%';
    const std::string suffix = suffix_stream.str();

    int bar_width = detail::terminal_columns() - detail::display_width(prefix)
                    - detail::display_width(suffix) - 2;
    if (bar_width > detail::kMaxBarWidth)
        bar_width = detail::kMaxBarWidth;
    if (bar_width < 1)
        bar_width = 1;

    const double cells = completed ? bar_width : ratio * bar_width;
    int full = static_cast<int>(cells);
    int eighths = static_cast<int>(std::round((cells - full) * 8.0));
    if (eighths >= 8)
    {
        ++full;
        eighths = 0;
    }
    full = std::clamp(full, 0, bar_width);
    if (full >= bar_width)
        eighths = 0;

    if (detail::progress_dirty && !label.empty() && label != detail::progress_label)
        detail::finalize_progress();

    detail::progress_label = label;
    detail::progress_dirty = !completed;

    static constexpr const char *kPartial[] = {"", "▏", "▎", "▍", "▌", "▋", "▊", "▉"};
    std::cout << '\r' << prefix << '[';
    std::cout << detail::kGreen;
    for (int i = 0; i < full; ++i)
        std::cout << "█";
    int rest = bar_width - full;
    if (eighths > 0 && rest > 0)
    {
        std::cout << kPartial[eighths];
        --rest;
    }
    std::cout << detail::kReset << detail::kDim;
    for (int i = 0; i < rest; ++i)
        std::cout << "░";
    std::cout << detail::kReset << ']' << suffix << "\033[K" << std::flush;

    if (completed)
    {
        std::cout << '\n';
        detail::progress_label.clear();
    }
    return completed;
}

inline void table(const std::vector<std::string> &headers,
                  const std::vector<std::vector<std::string>> &rows)
{
    std::lock_guard<std::mutex> lock(detail::mtx);
    detail::finalize_progress();

    std::size_t cols = headers.size();
    for (const auto &row : rows)
        cols = std::max(cols, row.size());
    if (cols == 0)
        return;

    std::vector<int> widths(cols, 0);
    for (std::size_t c = 0; c < cols; ++c)
    {
        if (c < headers.size())
            widths[c] = std::max(widths[c], detail::display_width(headers[c]));
        for (const auto &row : rows)
        {
            if (c < row.size())
                widths[c] = std::max(widths[c], detail::display_width(row[c]));
        }
        widths[c] = std::max(widths[c], 1);
    }

    detail::fit_widths(widths);

    std::vector<char> right_align(cols, 0);
    for (std::size_t c = 0; c < cols; ++c)
    {
        bool any = false;
        bool all_numeric = true;
        for (const auto &row : rows)
        {
            if (c >= row.size() || row[c].empty())
                continue;
            any = true;
            if (!detail::looks_numeric(row[c]))
            {
                all_numeric = false;
                break;
            }
        }
        right_align[c] = static_cast<char>(any && all_numeric);
    }

    const bool has_header = !headers.empty();
    detail::write_rule(widths, "┌", "┬", "┐");
    if (has_header)
    {
        detail::write_row(headers, widths, std::vector<char>(cols, 0));
        detail::write_rule(widths, "├", "┼", "┤");
    }
    for (const auto &row : rows)
        detail::write_row(row, widths, right_align);
    detail::write_rule(widths, "└", "┴", "┘");
}

inline void policy_bindings(const std::string &robot,
                            const std::vector<std::vector<std::string>> &bindings)
{
    message("INFO", robot );
    std::vector<std::vector<std::string>> rows;
    for (const auto &binding : bindings)
        if (binding.size() >= 3)
            rows.push_back({binding[0], binding[1] == "Unassigned" ? "未绑定" : binding[2]});
    table({"手柄按键", "策略目录"}, rows);
}

} // namespace tui

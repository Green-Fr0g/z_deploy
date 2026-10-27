#pragma once

#include <fcntl.h>
#include <mutex>
#include <string>
#include <termios.h>
#include <unistd.h>
#include <utility>

#include <types.hpp>
#include <utils/thread_timer.hpp>


class KeyboardHandler
{
public:
    ~KeyboardHandler()
    {
        stop();
    }

    void start()
    {
        if (raw_mode_)
            return;
        if (!isatty(STDIN_FILENO) || tcgetattr(STDIN_FILENO, &original_terminal_) == -1)
            return;

        termios raw = original_terminal_;
        raw.c_lflag &= ~(ICANON | ECHO);
        raw.c_cc[VMIN] = 0;
        raw.c_cc[VTIME] = 0;
        if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) != 0)
            return;
            
        original_flags_ = fcntl(STDIN_FILENO, F_GETFL, 0);
        if (original_flags_ != -1)
            fcntl(STDIN_FILENO, F_SETFL, original_flags_ | O_NONBLOCK);

        raw_mode_ = true;
        timer_.start(0.01, [this]() { handle(); });
    }

    void stop()
    {
        timer_.stop();
        if (raw_mode_)
        {
            tcsetattr(STDIN_FILENO, TCSANOW, &original_terminal_);
            if (original_flags_ != -1)
                fcntl(STDIN_FILENO, F_SETFL, original_flags_);
            raw_mode_ = false;
        }

        std::lock_guard<std::mutex> lock(mutex_);
        raw_.clear();
        pending_.clear();
        key_ = Keyboard::None;
    }

    // Raw bytes read since the last call.
    std::string consume_raw()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return std::exchange(raw_, {});
    }

    // Latest parsed key since the last call; consuming resets it to None.
    Keyboard consume_key()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return std::exchange(key_, Keyboard::None);
    }

private:
    // char -> Keyboard key. Returns Keyboard::None for unmapped characters.
    static Keyboard to_key(char character)
    {
        if (character >= 'a' && character <= 'z')
            return static_cast<Keyboard>(static_cast<int>(Keyboard::A) + (character - 'a'));
        if (character >= 'A' && character <= 'Z')
            return static_cast<Keyboard>(static_cast<int>(Keyboard::A) + (character - 'A'));
        if (character >= '0' && character <= '9')
            return static_cast<Keyboard>(static_cast<int>(Keyboard::Num0) + (character - '0'));

        switch (character)
        {
        case ' ':
            return Keyboard::Space;
        case '\n':
        case '\r':
            return Keyboard::Enter;
        case '\t':
            return Keyboard::None;
        default:
            return Keyboard::None;
        }
    }

    // Polled by timer_.
    void handle()
    {
        char buffer[64];
        const ssize_t count = read(STDIN_FILENO, buffer, sizeof(buffer));
        if (count <= 0)
            return;

        std::lock_guard<std::mutex> lock(mutex_);
        raw_.append(buffer, static_cast<std::size_t>(count));
        pending_.append(buffer, static_cast<std::size_t>(count));
        parse_pending();
    }

    // Parse pending_ and retain the latest key; incomplete escape sequences are kept for
    // the next poll. Called with mutex_ held.
    void parse_pending()
    {
        std::size_t i = 0;
        while (i < pending_.size())
        {
            const auto character = static_cast<unsigned char>(pending_[i]);
            if (character != 27)
            {
                const Keyboard key = to_key(pending_[i]);
                if (key != Keyboard::None)
                    key_ = key;
                ++i;
                continue;
            }

            // ESC: wait until at least the next byte is available.
            if (pending_.size() - i < 2)
                break;

            if (pending_[i + 1] == '[')
            {
                // "ESC [ X": wait for the final byte.
                if (pending_.size() - i < 3)
                    break;
                switch (pending_[i + 2])
                {
                case 'A':
                    key_ = Keyboard::Up;
                    break;
                case 'B':
                    key_ = Keyboard::Down;
                    break;
                case 'C':
                    key_ = Keyboard::Right;
                    break;
                case 'D':
                    key_ = Keyboard::Left;
                    break;
                default:
                    break;
                }
                i += 3;
                continue;
            }

            key_ = Keyboard::Escape;
            ++i;
        }
        pending_.erase(0, i);
    }

    termios original_terminal_{};
    int original_flags_{-1};
    bool raw_mode_{false};
    std::mutex mutex_;
    ThreadTimer timer_;
    std::string raw_;
    std::string pending_;
    Keyboard key_{Keyboard::None};
};

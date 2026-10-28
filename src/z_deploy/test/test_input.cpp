#include <array>
#include <chrono>
#include <cstdlib>
#include <fcntl.h>
#include <iostream>
#include <stdexcept>
#include <thread>
#include <unistd.h>
#include <utility>

#include <hardware/input.hpp>

namespace
{
void require(bool condition, const char *message)
{
    if (!condition)
        throw std::runtime_error(message);
}

// Exercise the actual terminal parser and input workers, without a joystick.
class Terminal
{
public:
    Terminal()
    {
        master_ = posix_openpt(O_RDWR | O_NOCTTY);
        require(master_ >= 0 && grantpt(master_) == 0 && unlockpt(master_) == 0,
                "Cannot create test terminal");
        const int slave = open(ptsname(master_), O_RDWR | O_NOCTTY);
        require(slave >= 0, "Cannot open test terminal");
        original_stdin_ = dup(STDIN_FILENO);
        require(original_stdin_ >= 0 && dup2(slave, STDIN_FILENO) >= 0,
                "Cannot attach test terminal to stdin");
        close(slave);
    }

    ~Terminal()
    {
        if (original_stdin_ >= 0)
        {
            dup2(original_stdin_, STDIN_FILENO);
            close(original_stdin_);
        }
        if (master_ >= 0)
            close(master_);
    }

    void press(char key)
    {
        require(write(master_, &key, 1) == 1, "Cannot write test key");
    }

private:
    int master_{-1};
    int original_stdin_{-1};
};

void wait_button(InputHandler &input, joybutton expected)
{
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
    while (input.get_snapshot().current != expected && std::chrono::steady_clock::now() < deadline)
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    require(input.get_snapshot().current == expected, "Keyboard combination mapping failed");
}
} // namespace

int main()
{
    try
    {
        Terminal terminal;
        InputHandler input("/dev/z_deploy_test_no_joystick");
        input.start();
        const std::array<std::pair<char, joybutton>, 12> keys{{
            {'1', joybutton::LB_DPadUp}, {'2', joybutton::LB_DPadDown},
            {'3', joybutton::LB_DPadLeft}, {'4', joybutton::LB_DPadRight},
            {'5', joybutton::RB_DPadUp}, {'6', joybutton::RB_DPadDown},
            {'7', joybutton::RB_DPadLeft}, {'8', joybutton::RB_DPadRight},
            {'0', joybutton::A}, {'p', joybutton::LB_X},
            {'r', joybutton::RB_Y}, {'\n', joybutton::RB_X},
        }};
        for (const auto &[key, button] : keys)
        {
            terminal.press(key);
            wait_button(input, button);
            wait_button(input, joybutton::None);
        }
        input.stop();
        require(input.get_snapshot().current == joybutton::None, "Stop retained a keyboard button");
        std::cout << "Input mapping passed: eight policy shortcuts and four base shortcuts\n";
    }
    catch (const std::exception &error)
    {
        std::cerr << error.what() << '\n';
        return 1;
    }
}

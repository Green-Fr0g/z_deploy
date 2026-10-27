#include <chrono>
#include <cstdlib>
#include <fcntl.h>
#include <iostream>
#include <stdexcept>
#include <string>
#include <sys/stat.h>
#include <thread>
#include <unistd.h>

#include <hardware/hardware_sim.hpp>

namespace
{
void require(bool condition, const char *message)
{
    if (!condition)
        throw std::runtime_error(message);
}

template <typename Predicate>
void wait_for(Predicate predicate, const char *message)
{
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (!predicate())
    {
        require(std::chrono::steady_clock::now() < deadline, message);
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}

// Feed the real library through a FIFO without requiring a physical joystick.
class JoystickDevice
{
public:
    JoystickDevice()
    {
        char directory[] = "/tmp/frog-joystick-XXXXXX";
        require(mkdtemp(directory) != nullptr, "Cannot create temporary directory");
        directory_ = directory;
        path_ = directory_ + "/js0";
    }

    ~JoystickDevice()
    {
        disconnect();
        unlink(path_.c_str());
        rmdir(directory_.c_str());
    }

    const std::string &path() const { return path_; }

    void connect()
    {
        if (access(path_.c_str(), F_OK) != 0)
            require(mkfifo(path_.c_str(), 0600) == 0, "Cannot create joystick FIFO");
        writer_ = open(path_.c_str(), O_RDWR | O_NONBLOCK);
        require(writer_ >= 0, "Cannot open joystick FIFO");
    }

    void disconnect()
    {
        if (writer_ >= 0)
            close(writer_);
        writer_ = -1;
    }

    void send(unsigned char type, unsigned char number, short value)
    {
        JoystickEvent event{};
        event.type = type;
        event.number = number;
        event.value = value;
        require(write(writer_, &event, sizeof(event)) == sizeof(event), "Cannot send joystick event");
    }

private:
    std::string directory_;
    std::string path_;
    int writer_{-1};
};

bool centered(const joyaxis &axis)
{
    return axis.L_X == 0.0F && axis.L_Y == 0.0F && axis.R_X == 0.0F && axis.R_Y == 0.0F;
}

void check_joystick()
{
    JoystickDevice device;
    RobotConfig config;
    config.robot_name = "joystick_test";
    config.dof = 1;
    HardwareSim hardware(config, device.path());
    HardwareBase &base = hardware;
    require(base.get_joy_button() == joybutton::None && centered(base.get_joy_axis()),
            "Initial joystick state must be neutral");
    base.start();
    base.start();
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    require(centered(base.get_joy_axis()), "Missing joystick must stay neutral");

    // Connecting after startup must work, including JS_EVENT_INIT state.
    device.connect();
    device.send(JS_EVENT_AXIS | JS_EVENT_INIT, 0, -32768);
    device.send(JS_EVENT_AXIS, 1, 32767);
    device.send(JS_EVENT_AXIS, 3, 32767);
    device.send(JS_EVENT_AXIS, 4, -32768);
    wait_for([&]() {
        const auto axis = base.get_joy_axis();
        return axis.L_X == -1.0F && axis.L_Y == 1.0F && axis.R_X == 1.0F && axis.R_Y == -1.0F;
    }, "Stick axes were not mapped or normalized correctly");

    device.send(JS_EVENT_BUTTON | JS_EVENT_INIT, 0, 1);
    wait_for([&]() { return base.get_joy_button() == joybutton::A; }, "Initial A button missing");
    device.send(JS_EVENT_AXIS, 2, 32767);
    device.send(JS_EVENT_AXIS, 5, -32768);
    device.send(JS_EVENT_BUTTON, 255, 1);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    require(base.get_joy_axis().R_X == 1.0F && base.get_joy_axis().R_Y == -1.0F,
            "Trigger axes must not overwrite right stick axes");
    require(base.get_joy_button() == joybutton::A, "Unknown buttons must be ignored");

    device.send(JS_EVENT_BUTTON, 0, 0);
    device.send(JS_EVENT_BUTTON, 4, 1);
    device.send(JS_EVENT_BUTTON, 2, 1);
    wait_for([&]() { return base.get_joy_button() == joybutton::LB_X; }, "LB+X mapping failed");
    device.send(JS_EVENT_BUTTON, 4, 0);
    wait_for([&]() { return base.get_joy_button() == joybutton::X; }, "Modifier release lost held X");
    device.send(JS_EVENT_BUTTON, 2, 0);
    device.send(JS_EVENT_BUTTON, 5, 1);
    device.send(JS_EVENT_AXIS, 7, -32768);
    wait_for([&]() { return base.get_joy_button() == joybutton::RB_DPadUp; }, "RB+D-pad mapping failed");
    device.send(JS_EVENT_BUTTON, 4, 1);
    wait_for([&]() { return base.get_joy_button() == joybutton::LB_RB; }, "LB+RB mapping failed");
    device.send(JS_EVENT_BUTTON, 4, 0);
    device.send(JS_EVENT_BUTTON, 5, 0);
    device.send(JS_EVENT_AXIS, 7, 0);
    device.send(JS_EVENT_BUTTON, 9, 1);
    wait_for([&]() { return base.get_joy_button() == joybutton::LStick; }, "Stick button mapping failed");
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    require(base.get_joy_button() == joybutton::LStick && base.get_joy_axis().L_X == -1.0F,
            "Idle device must retain held buttons and axes");

    device.disconnect();
    wait_for([&]() { return base.get_joy_button() == joybutton::None && centered(base.get_joy_axis()); },
             "Disconnect must clear buttons and axes");
    device.connect();
    device.send(JS_EVENT_BUTTON, 1, 1);
    wait_for([&]() { return base.get_joy_button() == joybutton::B; }, "Joystick did not reconnect");

    base.stop();
    base.stop();
    require(base.get_joy_button() == joybutton::None && centered(base.get_joy_axis()),
            "Stop must clear joystick state");
    base.start();
    device.send(JS_EVENT_BUTTON, 3, 1);
    wait_for([&]() { return base.get_joy_button() == joybutton::Y; }, "Restart did not resume input");
    // Destruction while started must join the polling thread.
}
} // namespace

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    try
    {
        check_joystick();
        std::cout << "joystick integration tests passed\n";
    }
    catch (const std::exception &error)
    {
        std::cerr << error.what() << '\n';
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
}

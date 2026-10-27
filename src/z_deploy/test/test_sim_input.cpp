#include <cmath>
#include <cstdlib>
#include <fcntl.h>
#include <iostream>
#include <stdexcept>
#include <thread>
#include <sys/stat.h>
#include <unistd.h>

#include <hardware/sim_interface.hpp>

namespace
{
void require(bool condition, const char *message)
{
    if (!condition)
        throw std::runtime_error(message);
}

struct Simulation : SimulationControl
{
    int resets{0}, pauses{0};
    void reset() override { ++resets; }
    void toggle_pause() override { ++pauses; }
};

struct Input : SimInput
{
    explicit Input(SimulationControl *simulation, std::string device = "/dev/input/js0")
        : SimInput(RobotConfig{}, simulation, std::move(device)) {}
    using SimInput::on_keyboard;
    using SimInput::on_joystick;
};

JoystickEvent event(unsigned char type, unsigned char number, short value)
{
    JoystickEvent result{};
    result.type = type;
    result.number = number;
    result.value = value;
    return result;
}

void check_mapping()
{
    Simulation sim;
    Input input(&sim);
    input.on_keyboard(Keyboard::Num1);
    require(input.get_joystick().current == joybutton::RB_DPadUp,
            "Keyboard must map to a joystick button");
    const auto keyboard_state = input.consume_target_state();
    input.on_joystick(event(JS_EVENT_BUTTON, 5, 1));
    input.on_joystick(event(JS_EVENT_AXIS, 7, -32768));
    require(input.consume_target_state() == keyboard_state && keyboard_state == "RLLocomotion",
            "Keyboard and joystick must request the same state");
    input.on_joystick(event(JS_EVENT_AXIS, 7, -32768));
    require(input.consume_target_state().empty(), "Held buttons must not repeat");
    input.on_joystick(event(JS_EVENT_AXIS, 7, 0));
    input.on_keyboard(Keyboard::R);
    require(sim.resets == 1, "Keyboard reset mapping failed");
    input.on_joystick(event(JS_EVENT_BUTTON | JS_EVENT_INIT, 3, 1));
    input.on_joystick(event(JS_EVENT_AXIS, 0, 0));
    require(sim.resets == 1, "Initial held button must remain suppressed");
    input.on_joystick(event(JS_EVENT_BUTTON, 3, 0));
    input.on_joystick(event(JS_EVENT_BUTTON, 3, 1));
    require(sim.resets == 2, "Joystick reset mapping failed");
    input.on_keyboard(Keyboard::W);
    require(std::fabs(input.get_user_command().x - 0.1F) < 1e-5F, "Keyboard virtual axis failed");
    input.on_joystick(event(JS_EVENT_AXIS, 1, -32768));
    require(input.get_user_command().x == 3.0F, "Joystick velocity failed");
    input.on_joystick(event(JS_EVENT_AXIS, 1, 0));
    require(std::fabs(input.get_user_command().x - 0.1F) < 1e-5F, "Centered stick fallback failed");
    input.on_keyboard(Keyboard::Space);
    require(input.get_user_command().x == 0, "Space must center virtual sticks");
    input.stop();
    require(input.get_joystick().current == joybutton::None, "Stop must clear input state");
}

// A FIFO supplies real library events without physical hardware or ROS.
void check_linux_transport()
{
    char directory[] = "/tmp/frog-input-XXXXXX";
    require(mkdtemp(directory) != nullptr, "Cannot create test directory");
    const auto path = std::string(directory) + "/js0";
    require(mkfifo(path.c_str(), 0600) == 0, "Cannot create joystick FIFO");
    const int writer = open(path.c_str(), O_RDWR | O_NONBLOCK);
    require(writer >= 0, "Cannot open FIFO");
    Input input(nullptr, path);
    input.start();
    JoystickEvent event{};
    event.type = JS_EVENT_AXIS;
    event.number = 1;
    event.value = -32768;
    require(write(writer, &event, sizeof(event)) == sizeof(event), "Cannot write event");
    for (int count = 0; count < 200 && input.get_user_command().x != 3.0F; ++count)
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    const bool received = input.get_user_command().x == 3.0F;
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    const bool retained = input.get_user_command().x == 3.0F;
    close(writer);
    for (int count = 0; count < 200 && input.get_user_command().x != 0.0F; ++count)
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    const bool disconnected = input.get_user_command().x == 0.0F;
    input.stop();
    unlink(path.c_str());
    rmdir(directory);
    require(received, "Library event did not reach input pipeline");
    require(retained, "An idle joystick must retain its axis state");
    require(disconnected, "Disconnected joystick must clear velocity");
}

void check_robot_transport()
{
    RobotConfig config;
    config.robot_name = "sim_input_test";
    config.dof = 1;
    SimRobot robot(config);
    auto peer = std::make_shared<rclcpp::Node>("sim_input_test_peer");
    auto state_pub = peer->create_publisher<z_msgs::msg::JointsData>("/sim_input_test/state", 1);
    auto imu_pub = peer->create_publisher<z_msgs::msg::ImuData>("/sim_input_test/imu", 1);
    z_msgs::msg::JointsCmd received;
    bool command_received = false;
    auto subscription = peer->create_subscription<z_msgs::msg::JointsCmd>(
        "/sim_input_test/command", 1, [&](z_msgs::msg::JointsCmd::SharedPtr msg)
        { received = *msg; command_received = true; });
    int resets = 0, pauses = 0, resumes = 0;
    auto reset = peer->create_service<std_srvs::srv::Empty>("/sim_input_test/reset",
        [&](const std_srvs::srv::Empty::Request::SharedPtr, std_srvs::srv::Empty::Response::SharedPtr)
        { ++resets; });
    auto pause = peer->create_service<std_srvs::srv::Empty>("/sim_input_test/pause",
        [&](const std_srvs::srv::Empty::Request::SharedPtr, std_srvs::srv::Empty::Response::SharedPtr)
        { ++pauses; });
    auto resume = peer->create_service<std_srvs::srv::Empty>("/sim_input_test/resume",
        [&](const std_srvs::srv::Empty::Request::SharedPtr, std_srvs::srv::Empty::Response::SharedPtr)
        { ++resumes; });
    auto wait_for = [&](auto condition)
    {
        for (int count = 0; count < 300; ++count)
        {
            rclcpp::spin_some(peer);
            if (condition())
                return true;
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        return false;
    };
    robot.start();
    require(wait_for([&]() { return state_pub->get_subscription_count() > 0 &&
        imu_pub->get_subscription_count() > 0 && subscription->get_publisher_count() > 0; }),
        "SimRobot topics did not connect");
    z_msgs::msg::ImuData imu;
    imu.orientation_w = 1;
    imu.angular_velocity_z = 0.25;
    imu_pub->publish(imu);
    RobotCommand command;
    command.initialize(1);
    command.position[0] = 0.5F;
    // IMU-only input must not enable commands.
    for (int count = 0; count < 20; ++count)
    {
        robot.set_joint_command(command);
        rclcpp::spin_some(peer);
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    require(!command_received, "IMU-only input enabled joint commands");
    z_msgs::msg::JointsData state;
    state.joints_data.resize(1);
    state.joints_data[0].position = 0.75F;
    state_pub->publish(state);
    require(wait_for([&]() { robot.set_joint_command(command); return command_received; }),
            "SimRobot did not publish a command after valid joint state");
    require(received.joints_cmd.size() == 1 && received.joints_cmd[0].position == 0.5F,
            "SimRobot command serialization failed");
    robot.reset();
    robot.toggle_pause();
    require(wait_for([&]() { return resets == 1 && pauses == 1; }), "Reset/pause services failed");
    robot.toggle_pause();
    require(wait_for([&]() { return resumes == 1; }), "Resume service failed");
    robot.stop();
    require(robot.get_joint_position()[0] == 0.75F, "Joint state decoding failed");
    require(robot.get_base_angular_velocity()[2] == 0.25F, "IMU decoding failed");
}

}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    try
    {
        check_mapping();
        check_linux_transport();
        check_robot_transport();
        std::cout << "sim input tests passed\n";
        rclcpp::shutdown();
    }
    catch (const std::exception &error)
    {
        std::cerr << error.what() << '\n';
        rclcpp::shutdown();
        return 1;
    }
}

#pragma once

#include <cstddef>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>
#include <z_msgs/msg/imu_data.hpp>
#include <z_msgs/msg/joints_cmd.hpp>
#include <z_msgs/msg/joints_data.hpp>

#include <z_deploy/hardware/hardware_base.hpp>
#include <z_deploy/hardware/input.hpp>

// MuJoCo simulation robot backend.
//
//   state in :  /{robot}/state (joints), /{robot}/imu
//   command out: /{robot}/command
//   services :  /{robot}/pause, /resume, /reset
class HardwareSim : public HardwareBase
{
public:
    explicit HardwareSim(const RobotConfig &config, std::string joystick_device = "/dev/input/js0")
        : HardwareBase(config), input_(std::move(joystick_device))
    {
        node_ = std::make_shared<rclcpp::Node>("sim_hardware_" + config.robot_name);

        const auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
        command_pub_ = node_->create_publisher<z_msgs::msg::JointsCmd>(
            "/" + config.robot_name + "/command", qos);
        state_sub_ = node_->create_subscription<z_msgs::msg::JointsData>(
            "/" + config.robot_name + "/state", qos,
            [this](const z_msgs::msg::JointsData::SharedPtr msg)
            { on_state(msg); });
        imu_sub_ = node_->create_subscription<z_msgs::msg::ImuData>(
            "/" + config.robot_name + "/imu", qos,
            [this](const z_msgs::msg::ImuData::SharedPtr msg)
            { on_imu(msg); });

        pause_client_ = node_->create_client<std_srvs::srv::Empty>("/" + config.robot_name + "/pause");
        resume_client_ = node_->create_client<std_srvs::srv::Empty>("/" + config.robot_name + "/resume");
        reset_client_ = node_->create_client<std_srvs::srv::Empty>("/" + config.robot_name + "/reset");
    }

    ~HardwareSim() override
    {
        stop();
    }

    joybutton get_joy_button() const override
    {
        std::lock_guard<std::mutex> lock(button_mutex_);
        const auto button = input_.get_snapshot().current;
        if (button != last_service_button_)
        {
            last_service_button_ = button;
            switch (button)
            {
            case joybutton::RB_Y: // Keyboard R.
                reset();
                break;
            case joybutton::RB_X: // Keyboard Enter.
                toggle_pause();
                break;
            default:
                break;
            }
        }
        return button;
    }

    joyaxis get_joy_axis() const override
    {
        return input_.get_snapshot().axis;
    }

    void reset() const
    {
        call(reset_client_);
    }

    void toggle_pause() const
    {
        std::lock_guard<std::mutex> lock(service_mutex_);
        auto &client = paused_ ? resume_client_ : pause_client_;
        if (call(client))
            paused_ = !paused_;
    }

protected:
    void start_backend() override
    {
        state_received_.store(false);
        executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
        executor_->add_node(node_);
        executor_thread_ = std::thread([this]()
                                       { executor_->spin(); });
        input_.start();
        call(reset_client_);
    }

    void stop_backend() override
    {
        input_.stop();
        {
            std::lock_guard<std::mutex> lock(button_mutex_);
            last_service_button_ = joybutton::None;
        }
        if (!executor_)
            return;

        executor_->cancel();
        if (executor_thread_.joinable())
            executor_thread_.join();
        executor_.reset();
    }

    void send_command_impl(const RobotCommand &command) override
    {
        if (command.position.size() != dof_ || command.velocity.size() != dof_ ||
            command.torque.size() != dof_ || command.kp.size() != dof_ || command.kd.size() != dof_)
        {
            RCLCPP_WARN(node_->get_logger(), "Ignoring command with incorrect joint count");
            return;
        }

        z_msgs::msg::JointsCmd msg;
        msg.stamp = node_->now();
        msg.joints_cmd.resize(dof_);
        for (std::size_t i = 0; i < dof_; ++i)
        {
            msg.joints_cmd[i].position = command.position[i];
            msg.joints_cmd[i].velocity = command.velocity[i];
            msg.joints_cmd[i].torque = command.torque[i];
            msg.joints_cmd[i].kp = command.kp[i];
            msg.joints_cmd[i].kd = command.kd[i];
        }
        command_pub_->publish(msg);
    }

private:
    void on_state(const z_msgs::msg::JointsData::SharedPtr msg)
    {
        if (msg->joints_data.size() != dof_)
        {
            RCLCPP_WARN(node_->get_logger(), "Ignoring state with incorrect joint count");
            return;
        }
        for (std::size_t i = 0; i < dof_; ++i)
        {
            state_staging_.joint_pos[i] = msg->joints_data[i].position;
            state_staging_.joint_vel[i] = msg->joints_data[i].velocity;
            state_staging_.joint_torque[i] = msg->joints_data[i].torque;
        }
        publish_state(state_staging_);
    }

    void on_imu(const z_msgs::msg::ImuData::SharedPtr msg)
    {
        state_staging_.base_quat = {static_cast<float>(msg->orientation_w),
                                    static_cast<float>(msg->orientation_x),
                                    static_cast<float>(msg->orientation_y),
                                    static_cast<float>(msg->orientation_z)};

        state_staging_.base_ang_vel[0] = static_cast<float>(msg->angular_velocity_x);
        state_staging_.base_ang_vel[1] = static_cast<float>(msg->angular_velocity_y);
        state_staging_.base_ang_vel[2] = static_cast<float>(msg->angular_velocity_z);

        state_staging_.base_lin_acc[0] = static_cast<float>(msg->linear_acceleration_x);
        state_staging_.base_lin_acc[1] = static_cast<float>(msg->linear_acceleration_y);
        state_staging_.base_lin_acc[2] = static_cast<float>(msg->linear_acceleration_z);

        state_staging_.base_euler[0] = static_cast<float>(msg->roll);
        state_staging_.base_euler[1] = static_cast<float>(msg->pitch);
        state_staging_.base_euler[2] = static_cast<float>(msg->yaw);

        // An IMU frame alone must not enable joint commands.
        if (state_received_.load())
            publish_state(state_staging_);
    }

    bool call(const rclcpp::Client<std_srvs::srv::Empty>::SharedPtr &client) const
    {
        if (!client->service_is_ready())
        {
            RCLCPP_WARN(node_->get_logger(), "Simulation service %s is not available",
                        client->get_service_name());
            return false;
        }
        client->async_send_request(std::make_shared<std_srvs::srv::Empty::Request>());
        return true;
    }

    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<z_msgs::msg::JointsCmd>::SharedPtr command_pub_;
    rclcpp::Subscription<z_msgs::msg::JointsData>::SharedPtr state_sub_;
    rclcpp::Subscription<z_msgs::msg::ImuData>::SharedPtr imu_sub_;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr pause_client_;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr resume_client_;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr reset_client_;
    std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
    std::thread executor_thread_;
    InputHandler input_;
    mutable std::mutex button_mutex_;
    mutable joybutton last_service_button_{joybutton::None};
    mutable std::mutex service_mutex_;
    mutable bool paused_{false};
};

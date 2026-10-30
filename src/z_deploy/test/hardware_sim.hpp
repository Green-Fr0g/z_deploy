#pragma once

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <z_msgs/msg/imu_data.hpp>
#include <z_msgs/msg/joints_data.hpp>

#include "hardware_base.hpp"

class SimHardwareReceiver : public HardwareReceiver
{
public:
    explicit SimHardwareReceiver(const RobotConfig &config) : HardwareReceiver(config)
    {
        node_ = std::make_shared<rclcpp::Node>("hardware_monitor_" + config.robot_name);
        const auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
        joints_sub_ = node_->create_subscription<z_msgs::msg::JointsData>(
            "/" + config.robot_name + "/state", qos,
            [this](const z_msgs::msg::JointsData::SharedPtr msg) { on_joints(*msg); });
        imu_sub_ = node_->create_subscription<z_msgs::msg::ImuData>(
            "/" + config.robot_name + "/imu", qos,
            [this](const z_msgs::msg::ImuData::SharedPtr msg) { on_imu(*msg); });
        executor_.add_node(node_);
    }

    void poll() override { executor_.spin_some(); }

private:
    void on_joints(const z_msgs::msg::JointsData &message)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (message.joints_data.size() != snapshot_.state.joint_pos.size())
        {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                                 "Ignoring state with incorrect joint count");
            return;
        }
        for (std::size_t i = 0; i < message.joints_data.size(); ++i)
        {
            snapshot_.state.joint_pos[i] = message.joints_data[i].position;
            snapshot_.state.joint_vel[i] = message.joints_data[i].velocity;
            snapshot_.state.joint_torque[i] = message.joints_data[i].torque;
        }
        snapshot_.joints_updated = Clock::now();
    }

    void on_imu(const z_msgs::msg::ImuData &message)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        auto &state = snapshot_.state;
        state.base_quat = {static_cast<float>(message.orientation_w),
                           static_cast<float>(message.orientation_x),
                           static_cast<float>(message.orientation_y),
                           static_cast<float>(message.orientation_z)};
        state.base_euler = {static_cast<float>(message.roll), static_cast<float>(message.pitch),
                            static_cast<float>(message.yaw)};
        state.base_ang_vel = {static_cast<float>(message.angular_velocity_x),
                              static_cast<float>(message.angular_velocity_y),
                              static_cast<float>(message.angular_velocity_z)};
        state.base_lin_acc = {static_cast<float>(message.linear_acceleration_x),
                              static_cast<float>(message.linear_acceleration_y),
                              static_cast<float>(message.linear_acceleration_z)};
        snapshot_.imu_updated = Clock::now();
    }

    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<z_msgs::msg::JointsData>::SharedPtr joints_sub_;
    rclcpp::Subscription<z_msgs::msg::ImuData>::SharedPtr imu_sub_;
    rclcpp::executors::SingleThreadedExecutor executor_;
};

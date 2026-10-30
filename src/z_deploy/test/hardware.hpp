#pragma once

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <mutex>
#include <numeric>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <z_msgs/msg/imu_data.hpp>
#include <z_msgs/msg/joints_data.hpp>

#include <config_types.hpp>
#include <types.hpp>

#if Z_DEPLOY_HAS_UNITREE
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#endif

// Standalone receiver for this terminal tool; it never starts deployment hardware.
class HardwareReceiver
{
public:
    using Clock = std::chrono::steady_clock;
    struct Snapshot
    {
        RobotState state;
        Clock::time_point joints_updated{};
        Clock::time_point imu_updated{};

        bool ready() const
        {
            return joints_updated != Clock::time_point{} && imu_updated != Clock::time_point{};
        }
    };

    static bool supported(const std::string &mode, const std::string &robot)
    {
        if (mode == "sim")
            return robot == "g1" || robot == "g1_23" || robot == "dr02" || robot == "adam";
#if Z_DEPLOY_HAS_UNITREE
        if (mode == "real")
            return robot == "g1" || robot == "g1_23";
#endif
        return false;
    }

    HardwareReceiver(const std::string &mode, const RobotConfig &config)
    {
        if (!supported(mode, config.robot_name) || config.dof <= 0)
            throw std::invalid_argument("Unsupported receiver mode/robot or invalid joint count");
        snapshot_.state.initialize(static_cast<std::size_t>(config.dof));
        if (mode == "sim")
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
            return;
        }
#if Z_DEPLOY_HAS_UNITREE
        sdk_joint_indices_.resize(config.robot_name == "g1" ? 29 : 23);
        std::iota(sdk_joint_indices_.begin(), sdk_joint_indices_.end(), 0);
        if (config.robot_name == "g1_23")
            sdk_joint_indices_ = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12,
                                  15, 16, 17, 18, 19, 22, 23, 24, 25, 26};
        if (sdk_joint_indices_.size() != snapshot_.state.joint_pos.size())
            throw std::invalid_argument("SDK joint count does not match base.yaml");
        const char *network_interface = std::getenv("Z_NETIF");
        if (network_interface == nullptr || *network_interface == '\0')
            throw std::invalid_argument("Set Z_NETIF=<network_interface> for real mode");
        unitree::robot::ChannelFactory::Instance()->Init(0, network_interface);
        state_sub_ = std::make_unique<unitree::robot::ChannelSubscriber<LowState>>("rt/lowstate");
        state_sub_->InitChannel([this](const void *message) { on_real_state(message); }, 1);
#endif
    }

    ~HardwareReceiver()
    {
#if Z_DEPLOY_HAS_UNITREE
        if (state_sub_)
            state_sub_->CloseChannel();
#endif
    }

    void poll()
    {
        if (node_)
            executor_.spin_some();
    }

    Snapshot get_snapshot() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return snapshot_;
    }

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
            snapshot_.state.joint_pos[i] = message.joints_data[i].position;
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

#if Z_DEPLOY_HAS_UNITREE
    using LowState = unitree_hg::msg::dds_::LowState_;

    static std::uint32_t crc32(const LowState &message)
    {
        static_assert(sizeof(LowState) % sizeof(std::uint32_t) == 0, "Invalid SDK message layout");
        const auto *bytes = reinterpret_cast<const unsigned char *>(&message);
        std::uint32_t crc = 0xFFFFFFFFU;
        for (std::size_t i = 0; i < sizeof(LowState) / sizeof(std::uint32_t) - 1; ++i)
        {
            std::uint32_t word;
            std::memcpy(&word, bytes + i * sizeof(word), sizeof(word));
            for (std::uint32_t bit = 0x80000000U; bit != 0; bit >>= 1)
            {
                const bool high = (crc & 0x80000000U) != 0;
                crc <<= 1;
                if (high)
                    crc ^= 0x04C11DB7U;
                if (word & bit)
                    crc ^= 0x04C11DB7U;
            }
        }
        return crc;
    }

    void on_real_state(const void *message)
    {
        if (message == nullptr)
            return;
        const auto &data = *static_cast<const LowState *>(message);
        if (data.crc() != crc32(data))
            return;
        std::lock_guard<std::mutex> lock(mutex_);
        auto &state = snapshot_.state;
        for (std::size_t i = 0; i < sdk_joint_indices_.size(); ++i)
            state.joint_pos[i] = data.motor_state()[sdk_joint_indices_[i]].q();
        const auto &imu = data.imu_state();
        std::copy(imu.quaternion().begin(), imu.quaternion().end(), state.base_quat.begin());
        std::copy(imu.rpy().begin(), imu.rpy().end(), state.base_euler.begin());
        std::copy(imu.gyroscope().begin(), imu.gyroscope().end(), state.base_ang_vel.begin());
        std::copy(imu.accelerometer().begin(), imu.accelerometer().end(), state.base_lin_acc.begin());
        snapshot_.joints_updated = snapshot_.imu_updated = Clock::now();
    }
#endif

    mutable std::mutex mutex_;
    Snapshot snapshot_;
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<z_msgs::msg::JointsData>::SharedPtr joints_sub_;
    rclcpp::Subscription<z_msgs::msg::ImuData>::SharedPtr imu_sub_;
    rclcpp::executors::SingleThreadedExecutor executor_;
#if Z_DEPLOY_HAS_UNITREE
    std::vector<std::size_t> sdk_joint_indices_;
    // Keep the subscription last so callbacks stop before snapshot members are destroyed.
    std::unique_ptr<unitree::robot::ChannelSubscriber<LowState>> state_sub_;
#endif
};

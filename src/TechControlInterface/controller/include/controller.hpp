// Copyright 2023 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "control_msgs/msg/joint_trajectory_controller_state.hpp"
#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace TechControlInterface
{
    class TechController : public controller_interface::ControllerInterface
    {
    public:
        TechController();

        controller_interface::InterfaceConfiguration command_interface_configuration() const override;

        controller_interface::InterfaceConfiguration state_interface_configuration() const override;

        controller_interface::return_type update(
            const rclcpp::Time &time, const rclcpp::Duration &period) override;

        controller_interface::CallbackReturn on_init() override;

        controller_interface::CallbackReturn on_configure(
            const rclcpp_lifecycle::State &previous_state) override;

        controller_interface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State &previous_state) override;

        controller_interface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State &previous_state) override;
        rclcpp::Logger get_logger() const { return *m_Logger; }
    protected:
        // Objects for logging
        std::shared_ptr<rclcpp::Logger> m_Logger;
        std::vector<std::string> m_JointNames;
        std::vector<std::string> m_CommandInterfaceTypes;
        std::vector<std::string> m_StateInterfaceTypes;

        rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr m_JointCommandSubscriber;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr m_JointStatePublisher;
        realtime_tools::RealtimeBuffer<std::shared_ptr<trajectory_msgs::msg::JointTrajectory>> m_TrajMsgExternalPointPtr;
        bool m_NewMsg = false;
        rclcpp::Time m_StartTime;
        std::shared_ptr<trajectory_msgs::msg::JointTrajectory> m_TrajectoryMsg;
        sensor_msgs::msg::JointState m_JointStateMsg;
        trajectory_msgs::msg::JointTrajectoryPoint m_PointInterp;

        std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
            m_JointPositionCommandInterface;
        std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
            m_JointVelocityCommandInterface;
        std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
            m_JointPositionStateInterface;
        std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
            m_JointVelocityStateInterface;

        std::unordered_map<
            std::string, std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> *>
            m_CommandInterfaceMap = {
                {"position", &m_JointPositionCommandInterface},
                {"velocity", &m_JointVelocityCommandInterface}};

        std::unordered_map<
            std::string, std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> *>
            m_StateInterfaceMap = {
                {"position", &m_JointPositionStateInterface},
                {"velocity", &m_JointVelocityStateInterface}};
    };

} // namespace TechControlInterface

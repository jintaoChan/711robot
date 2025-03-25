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

#include "controller.hpp"

#include <stddef.h>
#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/qos.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

using config_type = controller_interface::interface_configuration_type;

namespace TechControlInterface
{
    TechController::TechController() : controller_interface::ControllerInterface() {}

    controller_interface::CallbackReturn TechController::on_init()
    {
        m_Logger = std::make_shared<rclcpp::Logger>(rclcpp::get_logger("TechController"));
        // should have error handling
        m_JointNames = auto_declare<std::vector<std::string>>("joints", m_JointNames);
        m_CommandInterfaceTypes =
            auto_declare<std::vector<std::string>>("command_interfaces", m_CommandInterfaceTypes);
        m_StateInterfaceTypes =
            auto_declare<std::vector<std::string>>("state_interfaces", m_StateInterfaceTypes);

        m_PointInterp.positions.assign(m_JointNames.size(), 0);
        m_PointInterp.velocities.assign(m_JointNames.size(), 0);

        return CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration TechController::command_interface_configuration()
        const
    {
        controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};

        conf.names.reserve(m_JointNames.size() * m_CommandInterfaceTypes.size());
        for (const auto &joint_name : m_JointNames)
        {
            for (const auto &interface_type : m_CommandInterfaceTypes)
            {
                conf.names.push_back(joint_name + "/" + interface_type);
            }
        }

        return conf;
    }

    controller_interface::InterfaceConfiguration TechController::state_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};

        conf.names.reserve(m_JointNames.size() * m_StateInterfaceTypes.size());
        for (const auto &joint_name : m_JointNames)
        {
            for (const auto &interface_type : m_StateInterfaceTypes)
            {
                conf.names.push_back(joint_name + "/" + interface_type);
            }
        }

        return conf;
    }

    controller_interface::CallbackReturn TechController::on_configure(const rclcpp_lifecycle::State &)
    {
        auto callback =
            [this](const std::shared_ptr<trajectory_msgs::msg::JointTrajectory> traj_msg) -> void
        {
            m_TrajMsgExternalPointPtr.writeFromNonRT(traj_msg);
            m_NewMsg = true;
        };
        
        m_JointStatePublisher = get_node()->create_publisher<sensor_msgs::msg::JointState>("tech_joint_state", 10);
        m_JointCommandSubscriber =
            get_node()->create_subscription<trajectory_msgs::msg::JointTrajectory>(
                "~/joint_trajectory", rclcpp::SystemDefaultsQoS(), callback);

        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn TechController::on_activate(const rclcpp_lifecycle::State &)
    {
        // clear out vectors in case of restart
        m_JointPositionCommandInterface.clear();
        m_JointVelocityCommandInterface.clear();
        m_JointPositionStateInterface.clear();
        m_JointVelocityStateInterface.clear();

        // assign command interfaces
        for (auto &interface : command_interfaces_)
        {
            m_CommandInterfaceMap[interface.get_interface_name()]->push_back(interface);
        }

        // assign state interfaces
        for (auto &interface : state_interfaces_)
        {
            m_StateInterfaceMap[interface.get_interface_name()]->push_back(interface);
        }

        return CallbackReturn::SUCCESS;
    }

    void interpolate_point(
        const trajectory_msgs::msg::JointTrajectoryPoint &point_1,
        const trajectory_msgs::msg::JointTrajectoryPoint &point_2,
        trajectory_msgs::msg::JointTrajectoryPoint &point_interp, double delta)
    {
        for (size_t i = 0; i < point_1.positions.size(); i++)
        {
            point_interp.positions[i] = delta * point_2.positions[i] + (1.0 - delta) * point_2.positions[i];
        }
        for (size_t i = 0; i < point_1.positions.size(); i++)
        {
            point_interp.velocities[i] =
                delta * point_2.velocities[i] + (1.0 - delta) * point_2.velocities[i];
        }
    }

    void interpolate_trajectory_point(
        const trajectory_msgs::msg::JointTrajectory &traj_msg, const rclcpp::Duration &cur_time,
        trajectory_msgs::msg::JointTrajectoryPoint &point_interp)
    {
        double traj_len = traj_msg.points.size();
        auto last_time = traj_msg.points[traj_len - 1].time_from_start;
        double total_time = last_time.sec + last_time.nanosec * 1E-9;

        size_t ind = cur_time.seconds() * (traj_len / total_time);
        ind = std::min(static_cast<double>(ind), traj_len - 2);
        double delta = cur_time.seconds() - ind * (total_time / traj_len);
        interpolate_point(traj_msg.points[ind], traj_msg.points[ind + 1], point_interp, delta);
    }

    controller_interface::return_type TechController::update(
        const rclcpp::Time &time, const rclcpp::Duration & /*period*/)
    {

        // Publish joint state
        {
            sensor_msgs::msg::JointState jointState;
            for(const auto& state : m_JointPositionStateInterface){
                double val{100};
                jointState.position.push_back(state.get().get_value(val));
                RCLCPP_INFO(get_logger(), "Getting %s: %d", std::string(state.get().get_name()).c_str(), (int)val);
            }
            m_JointStatePublisher->publish(jointState);
        }


        // Received a msg. Write to robot
        if (m_NewMsg)
        {
            m_TrajectoryMsg = *m_TrajMsgExternalPointPtr.readFromRT();
            m_StartTime = time;
            m_NewMsg = false;
        }

        if (m_TrajectoryMsg != nullptr)
        {
            interpolate_trajectory_point(*m_TrajectoryMsg, time - m_StartTime, m_PointInterp);
            for (size_t i = 0; i < m_JointPositionCommandInterface.size(); i++)
            {
                m_JointPositionCommandInterface[i].get().set_value(m_PointInterp.positions[i]);
            }
            for (size_t i = 0; i < m_JointVelocityCommandInterface.size(); i++)
            {
                m_JointVelocityCommandInterface[i].get().set_value(m_PointInterp.velocities[i]);
            }
        }

        return controller_interface::return_type::OK;
    }

    controller_interface::CallbackReturn TechController::on_deactivate(const rclcpp_lifecycle::State &)
    {
        release_interfaces();

        return CallbackReturn::SUCCESS;
    }

}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    TechControlInterface::TechController, controller_interface::ControllerInterface)

/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, SRI International
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of SRI International nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Sachin Chitta, Dave Coleman, Mike Lautman */

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

#include <moveit_visual_tools/moveit_visual_tools.h>

// All source files that use ROS logging should define a file-specific
// static const rclcpp::Logger named LOGGER, located at the top of the file
// and inside the namespace with the narrowest scope (if there is one)
static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto move_group_node = rclcpp::Node::make_shared("move_group_interface_tutorial", node_options);

    // We spin up a SingleThreadedExecutor for the current state monitor to get information
    // about the robot's state.
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(move_group_node);
    std::thread([&executor]()
                { executor.spin(); })
        .detach();
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    static const std::string PLANNING_GROUP = "store_manipulator";

    moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // Raw pointers are frequently used to refer to the planning group for improved performance.
    const moveit::core::JointModelGroup *joint_model_group =
        move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    // Visualization
    // ^^^^^^^^^^^^^
    geometry_msgs::msg::Pose end_effector_pose = move_group.getCurrentPose().pose;

    // Print the position and orientation of the end-effector
    RCLCPP_INFO(LOGGER, "End Effector Position:");
    RCLCPP_INFO(LOGGER, "Position: x=%.3f, y=%.3f, z=%.3f", 
                end_effector_pose.position.x, 
                end_effector_pose.position.y, 
                end_effector_pose.position.z);

    RCLCPP_INFO(LOGGER, "Orientation: x=%.3f, y=%.3f, z=%.3f, w=%.3f", 
                end_effector_pose.orientation.x, 
                end_effector_pose.orientation.y,
                end_effector_pose.orientation.z,
                end_effector_pose.orientation.w);
    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(end_effector_pose);

    geometry_msgs::msg::Pose target_pose3 = end_effector_pose;

    target_pose3.position.y -= 0.2;
    waypoints.push_back(target_pose3);  // right
    target_pose3.position.x += 0.1;
    waypoints.push_back(target_pose3);  // right


    // We want the Cartesian path to be interpolated at a resolution of 1 cm,
    // which is why we will specify 0.01 as the max step in Cartesian translation.
    const double eef_step = 0.01;
    moveit_msgs::msg::RobotTrajectory trajectory;
    double fraction = move_group.computeCartesianPath(waypoints, eef_step, trajectory);
    RCLCPP_INFO(LOGGER, "Visualizing plan 4 (Cartesian path) (%.2f%% achieved)", fraction * 100.0);
    move_group.execute(trajectory);
    // move_group.move();
    rclcpp::shutdown();
    return 0;
}
#include <switcher.h>

Switcher::Switcher(){
    m_Node = rclcpp::Node::make_shared("QtControlModeSwitcher");
    m_Client = m_Node->create_client<controller_manager_msgs::srv::SwitchController>("/controller_manager/switch_controller", 10);
}


void Switcher::run(){
    //20HZ
    rclcpp::WallRate loop_rate(1);
    while (rclcpp::ok())
    {
        rclcpp::spin_some(m_Node);
        loop_rate.sleep();
    }
    rclcpp::shutdown();
}


void Switcher::switchControlMode(ControlMode mode){

    auto req = std::make_shared<controller_manager_msgs::srv::SwitchController_Request>();
    controller_manager_msgs::srv::SwitchController_Response res;
    req->strictness = 1;
    req->timeout = builtin_interfaces::msg::Duration{rclcpp::Duration::from_seconds(1)};
    switch(mode) {
        case ControlMode::Disable : {
            req->activate_controllers = {"TechDisableController"};
            req->deactivate_controllers = {"TechPositionController", "TechVelocityController"};
            break;
        }
        case ControlMode::Position : {
            req->activate_controllers = {"TechPositionController"};
            req->deactivate_controllers = {"TechDisableController", "TechVelocityController"};
            break;
        }
        case ControlMode::Velocity : {
            req->activate_controllers = {"TechVelocityController"};
            req->deactivate_controllers = {"TechDisableController", "TechPositionController"};
            break;
        }
    }

    auto result = m_Client->async_send_request(req);
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(m_Node, result) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Currently using %d mode", (int)mode);
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call controller switch service!");
    }

    return;
}

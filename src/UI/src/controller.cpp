#include "controller.h"

Controller::Controller()  {
    int argc=0;
    char **argv=NULL;
    rclcpp::init(argc,argv);
    m_Node = rclcpp::Node::make_shared("QtController");
    m_Publisher = m_Node->create_publisher<sensor_msgs::msg::JointState>("/tech_joint_cmd", 10);
    m_Subscription = m_Node->create_subscription<sensor_msgs::msg::JointState>("/tech_joint_state", 10, std::bind(&Controller::recvCallback,this, std::placeholders::_1));
    this->start();
}
void Controller::run(){
    //20HZ
    sensor_msgs::msg::JointState tmp;
    rclcpp::WallRate loop_rate(100);
    while (rclcpp::ok())
    {
        rclcpp::spin_some(m_Node);
        loop_rate.sleep();
    }
    rclcpp::shutdown();
}

void Controller::recvCallback(const sensor_msgs::msg::JointState& msg) {
    emitTopicData(msg);
}

void Controller::pubMsg(const sensor_msgs::msg::JointState& msg){
    m_Publisher->publish(msg);
}

/*
 * @Author: chengyangkj
 * @Date: 2021-10-30 02:15:28
 * @LastEditTime: 2021-12-01 06:03:51
 * @LastEditors: chengyangkj
 * @Description: ros2的通信类 在这个类进行订阅与发布话题
 * @FilePath: /ros2_qt_demo/include/ros2_qt_demo/Controller.h
 * https://github.com/chengyangkj
 */
#ifndef CONTROLLER_H
#define CONTROLLER_H
#include <QObject>
#include <QThread>
#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/joint_state.hpp"

class Controller:public QThread
{
    Q_OBJECT
public:
    Controller();
    void recvCallback(const sensor_msgs::msg::JointState& msg);
    void pubMsg(const sensor_msgs::msg::JointState& msg);
protected:
    void run();
private:
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr m_Subscription;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr m_Publisher;
    std::shared_ptr<rclcpp::Node> m_Node;
signals:
    void emitTopicData(sensor_msgs::msg::JointState jointState);
};
#endif // CONTROLLER_H

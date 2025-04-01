#ifndef SWITCHER_H
#define SWITCHER_H


#include <QObject>
#include <QThread>
#include <rclcpp/rclcpp.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>

enum class ControlMode : uint8_t {
    Disable = 0,
    Position,
    Velocity
};

class Switcher:public QThread
{
    Q_OBJECT
public:
    Switcher();

public:
    void switchControlMode(ControlMode mode);

protected:
    void run();
private:
    rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr m_Client;
    std::shared_ptr<rclcpp::Node> m_Node;
signals:
};
#endif // CONTROLLER_H



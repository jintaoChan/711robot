#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "controller.h"
#include "switcher.h"

QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

public slots:
    void updateTopicInfo(sensor_msgs::msg::JointState jointState);

private slots:

    void on_button_enable_clicked();

    void on_button_disable_clicked();

    void on_text_step_editingFinished();

    void on_text_velocity_editingFinished();

    void on_button_joint2_forw_clicked();

    void on_button_joint3_forw_clicked();

    void on_button_joint4_forw_clicked();

    void on_button_joint5_forw_clicked();

    void on_button_joint2_back_clicked();

    void on_button_joint3_back_clicked();

    void on_button_joint4_back_clicked();

    void on_button_joint5_back_clicked();

    void on_button_joint2_forw_velo_pressed();

    void on_button_joint2_forw_velo_released();

    void on_button_joint2_back_velo_pressed();

    void on_button_joint2_back_velo_released();

    void on_button_joint3_forw_velo_pressed();

    void on_button_joint3_forw_velo_released();

    void on_button_joint3_back_velo_pressed();

    void on_button_joint3_back_velo_released();

    void on_button_joint4_forw_velo_pressed();

    void on_button_joint4_forw_velo_released();

    void on_button_joint4_back_velo_pressed();

    void on_button_joint4_back_velo_released();

    void on_button_joint5_forw_velo_pressed();

    void on_button_joint5_forw_velo_released();

    void on_button_joint5_back_velo_pressed();

    void on_button_joint5_back_velo_released();

private:
    void setButtonState(bool state);
    rclcpp::Logger get_logger() const { return *m_Logger; }

private:
    std::shared_ptr<rclcpp::Logger> m_Logger;
    Ui::MainWindow *ui;
    Controller* m_ControllerNode;
    Switcher* m_SwitcherNode;
    sensor_msgs::msg::JointState m_Msg;
    int m_Step;
    double m_Velocity;
};
#endif // MAINWINDOW_H

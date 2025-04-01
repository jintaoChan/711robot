#include <qstandarditemmodel.h>
#include <QObject>
#include <QIntValidator>

#include "mainwindow.h"
#include "./ui_mainwindow.h"


MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    m_Logger = std::make_shared<rclcpp::Logger>(rclcpp::get_logger("UI"));


    m_ControllerNode = new Controller();
    m_SwitcherNode = new Switcher();
    qRegisterMetaType<sensor_msgs::msg::JointState>("sensor_msgs::msg::JointState");
    connect(m_ControllerNode,SIGNAL(emitTopicData(sensor_msgs::msg::JointState)),this,SLOT(updateTopicInfo(sensor_msgs::msg::JointState)));
    QIntValidator * intVld = new QIntValidator(this);
    intVld->setRange(1,100);
    ui->text_step->setValidator(intVld);
    m_Step = ui->text_step->text().toInt();
    m_Velocity = ui->text_velocity->text().toInt();

    setButtonState(false);

}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::updateTopicInfo(sensor_msgs::msg::JointState jointState){
    if(jointState.position.size() != 4) {
        RCLCPP_WARN(get_logger(), "The number:%d of joint state may wrong!", jointState.position.size());
    }
    else {
        m_Msg = jointState;
        ui->label_joint2_position->setText(QString::number(m_Msg.position[0]));
        ui->label_joint3_position->setText(QString::number(m_Msg.position[1]));
        ui->label_joint4_position->setText(QString::number(m_Msg.position[2]));
        ui->label_joint5_position->setText(QString::number(m_Msg.position[3]));
    }
}



void MainWindow::on_button_enable_clicked()
{
    setButtonState(true);
    m_SwitcherNode->switchControlMode(ControlMode::Position);

}

void MainWindow::on_button_disable_clicked()
{
    setButtonState(false);
    m_SwitcherNode->switchControlMode(ControlMode::Disable);
}

void MainWindow::on_text_step_editingFinished()
{
    m_Step = ui->text_step->text().toInt();
}

void MainWindow::on_text_velocity_editingFinished()
{
    m_Velocity = ui->text_velocity->text().toInt();
}

void MainWindow::on_button_joint2_forw_clicked()
{
    m_SwitcherNode->switchControlMode(ControlMode::Position);
    sensor_msgs::msg::JointState msg(m_Msg);
    msg.position[0] += m_Step;
    m_ControllerNode->pubMsg(msg);
}


void MainWindow::on_button_joint3_forw_clicked()
{
    m_SwitcherNode->switchControlMode(ControlMode::Position);
    sensor_msgs::msg::JointState msg(m_Msg);
    msg.position[1] += m_Step;
    m_ControllerNode->pubMsg(msg);
}


void MainWindow::on_button_joint4_forw_clicked()
{
    m_SwitcherNode->switchControlMode(ControlMode::Position);
    sensor_msgs::msg::JointState msg(m_Msg);
    msg.position[2] += m_Step;
    m_ControllerNode->pubMsg(msg);
}


void MainWindow::on_button_joint5_forw_clicked()
{
    m_SwitcherNode->switchControlMode(ControlMode::Position);
    sensor_msgs::msg::JointState msg(m_Msg);
    msg.position[3] += m_Step;
    m_ControllerNode->pubMsg(msg);
}

void MainWindow::on_button_joint2_back_clicked()
{
    m_SwitcherNode->switchControlMode(ControlMode::Position);
    sensor_msgs::msg::JointState msg(m_Msg);
    msg.position[0] -= m_Step;
    m_ControllerNode->pubMsg(msg);
}

void MainWindow::on_button_joint3_back_clicked()
{
    m_SwitcherNode->switchControlMode(ControlMode::Position);
    sensor_msgs::msg::JointState msg(m_Msg);
    msg.position[1] -= m_Step;
    m_ControllerNode->pubMsg(msg);
}

void MainWindow::on_button_joint4_back_clicked()
{
    m_SwitcherNode->switchControlMode(ControlMode::Position);
    sensor_msgs::msg::JointState msg(m_Msg);
    msg.position[2] -= m_Step;
    m_ControllerNode->pubMsg(msg);
}

void MainWindow::on_button_joint5_back_clicked()
{
    m_SwitcherNode->switchControlMode(ControlMode::Position);
    sensor_msgs::msg::JointState msg(m_Msg);
    msg.position[3] -= m_Step;
    m_ControllerNode->pubMsg(msg);
}

void MainWindow::on_button_joint2_forw_velo_pressed()
{
    m_SwitcherNode->switchControlMode(ControlMode::Velocity);
    sensor_msgs::msg::JointState msg(m_Msg);
    msg.velocity[0] = m_Velocity;
    m_ControllerNode->pubMsg(msg);
}


void MainWindow::on_button_joint2_forw_velo_released()
{
    m_SwitcherNode->switchControlMode(ControlMode::Velocity);
    sensor_msgs::msg::JointState msg(m_Msg);
    msg.velocity[0] = 0;
    m_ControllerNode->pubMsg(msg);
}

void MainWindow::on_button_joint2_back_velo_pressed()
{
    m_SwitcherNode->switchControlMode(ControlMode::Velocity);
    sensor_msgs::msg::JointState msg(m_Msg);
    msg.velocity[0] = -m_Velocity;
    m_ControllerNode->pubMsg(msg);
}


void MainWindow::on_button_joint2_back_velo_released()
{
    m_SwitcherNode->switchControlMode(ControlMode::Velocity);
    sensor_msgs::msg::JointState msg(m_Msg);
    msg.velocity[0] = 0;
    m_ControllerNode->pubMsg(msg);
}

void MainWindow::on_button_joint3_forw_velo_pressed()
{
    m_SwitcherNode->switchControlMode(ControlMode::Velocity);
    sensor_msgs::msg::JointState msg(m_Msg);
    msg.velocity[1] = m_Velocity;
    m_ControllerNode->pubMsg(msg);
}


void MainWindow::on_button_joint3_forw_velo_released()
{
    m_SwitcherNode->switchControlMode(ControlMode::Velocity);
    sensor_msgs::msg::JointState msg(m_Msg);
    msg.velocity[1] = 0;
    m_ControllerNode->pubMsg(msg);
}

void MainWindow::on_button_joint3_back_velo_pressed()
{
    m_SwitcherNode->switchControlMode(ControlMode::Velocity);
    sensor_msgs::msg::JointState msg(m_Msg);
    msg.velocity[1] = -m_Velocity;
    m_ControllerNode->pubMsg(msg);
}


void MainWindow::on_button_joint3_back_velo_released()
{
    m_SwitcherNode->switchControlMode(ControlMode::Velocity);
    sensor_msgs::msg::JointState msg(m_Msg);
    msg.velocity[1] = 0;
    m_ControllerNode->pubMsg(msg);
}

void MainWindow::on_button_joint4_forw_velo_pressed()
{
    m_SwitcherNode->switchControlMode(ControlMode::Velocity);
    sensor_msgs::msg::JointState msg(m_Msg);
    msg.velocity[2] = m_Velocity;
    m_ControllerNode->pubMsg(msg);
}


void MainWindow::on_button_joint4_forw_velo_released()
{
    m_SwitcherNode->switchControlMode(ControlMode::Velocity);
    sensor_msgs::msg::JointState msg(m_Msg);
    msg.velocity[2] = 0;
    m_ControllerNode->pubMsg(msg);
}

void MainWindow::on_button_joint4_back_velo_pressed()
{
    m_SwitcherNode->switchControlMode(ControlMode::Velocity);
    sensor_msgs::msg::JointState msg(m_Msg);
    msg.velocity[2] = -m_Velocity;
    m_ControllerNode->pubMsg(msg);
}


void MainWindow::on_button_joint4_back_velo_released()
{
    m_SwitcherNode->switchControlMode(ControlMode::Velocity);
    sensor_msgs::msg::JointState msg(m_Msg);
    msg.velocity[2] = 0;
    m_ControllerNode->pubMsg(msg);
}

void MainWindow::on_button_joint5_forw_velo_pressed()
{
    m_SwitcherNode->switchControlMode(ControlMode::Velocity);
    sensor_msgs::msg::JointState msg(m_Msg);
    msg.velocity[3] = m_Velocity;
    m_ControllerNode->pubMsg(msg);
}

void MainWindow::on_button_joint5_forw_velo_released()
{
    m_SwitcherNode->switchControlMode(ControlMode::Velocity);
    sensor_msgs::msg::JointState msg(m_Msg);
    msg.velocity[3] = 0;
    m_ControllerNode->pubMsg(msg);
}

void MainWindow::on_button_joint5_back_velo_pressed()
{
    m_SwitcherNode->switchControlMode(ControlMode::Velocity);
    sensor_msgs::msg::JointState msg(m_Msg);
    msg.velocity[3] = -m_Velocity;
    m_ControllerNode->pubMsg(msg);
}


void MainWindow::on_button_joint5_back_velo_released()
{
    m_SwitcherNode->switchControlMode(ControlMode::Velocity);
    sensor_msgs::msg::JointState msg(m_Msg);
    msg.velocity[3] = 0;
    m_ControllerNode->pubMsg(msg);
}

void MainWindow::setButtonState(bool state){
    ui->button_joint2_forw->setEnabled(state);
    ui->button_joint2_back->setEnabled(state);
    ui->button_joint3_forw->setEnabled(state);
    ui->button_joint3_back->setEnabled(state);
    ui->button_joint4_forw->setEnabled(state);
    ui->button_joint4_back->setEnabled(state);
    ui->button_joint5_forw->setEnabled(state);
    ui->button_joint5_back->setEnabled(state);

    ui->button_joint2_forw_velo->setEnabled(state);
    ui->button_joint2_back_velo->setEnabled(state);
    ui->button_joint3_forw_velo->setEnabled(state);
    ui->button_joint3_back_velo->setEnabled(state);
    ui->button_joint4_forw_velo->setEnabled(state);
    ui->button_joint4_back_velo->setEnabled(state);
    ui->button_joint5_forw_velo->setEnabled(state);
    ui->button_joint5_back_velo->setEnabled(state);
}


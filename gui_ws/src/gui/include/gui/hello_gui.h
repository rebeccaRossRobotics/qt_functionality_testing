#ifndef HELLOGUI_H
#define HELLOGUI_H

#include <QMainWindow>
#include <qtimer.h>
#include <QtMultimedia>
#include <QtMultimediaWidgets>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"
#include "rviz_common/ros_integration/ros_node_abstraction.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"
#include "action_tutorials_interfaces/action/fibonacci.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rviz_common/ros_integration/ros_node_abstraction.hpp"
#include "rviz_common/render_panel.hpp"

#include "gui/rvizpanel.h"

QT_BEGIN_NAMESPACE
namespace Ui {
    class HelloGui;
}
QT_END_NAMESPACE

class HelloGui : public QMainWindow
{
    Q_OBJECT

public:
    HelloGui(QApplication *app,
    rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr rviz_ros_node,
    QWidget *parent = nullptr);
    ~HelloGui();

public slots:
    void spin_once();
    void update_count();

private slots:
    void on_publish_to_topic_clicked();

    void on_add_btn_clicked();

    void on_send_goal_btn_clicked();

private:
    QApplication * app_;
    Ui::HelloGui *ui;

    QTimer *ros_timer;
    QTimer *count_timer;

    QMediaPlayer *player;

    std::shared_ptr<RvizPanel> rviz_panel;
    QWidget * rviz_panel_widet;

    rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr node_weak_ptr_;
    rclcpp::Node::SharedPtr node;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription;
    rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr service_client;
    rclcpp::CallbackGroup::SharedPtr client_cb_group;
    rclcpp_action::Client<action_tutorials_interfaces::action::Fibonacci>::SharedPtr action_client;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher;

    void subscription_callback(const std_msgs::msg::String & msg);
    void handle_client_response(example_interfaces::srv::AddTwoInts::Response::SharedPtr response);

    void action_feedback(
        const rclcpp_action::ClientGoalHandle<action_tutorials_interfaces::action::Fibonacci>::SharedPtr,
        const std::shared_ptr<const typename action_tutorials_interfaces::action::Fibonacci::Feedback> feedback);

    void action_result_callback(
        const rclcpp_action::ClientGoalHandle<action_tutorials_interfaces::action::Fibonacci>::WrappedResult & result);

    void publish_cmd_vel(float x, float y);
};
#endif // HELLOGUI_H

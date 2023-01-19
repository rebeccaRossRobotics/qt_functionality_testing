//sudo apt-get install libqt5multimedia5-plugins

#include "gui/hello_gui.h"
#include "./ui_hello_gui.h"
#include "rclcpp/rclcpp.hpp"
#include "stdio.h"
#include <chrono> // NOLINT
#include "gui/rrjoystick.h"
#include <rviz_common/display_context.hpp>
#include "gui/rvizpanel.h"

HelloGui::HelloGui(
    QApplication *app,
    rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr rviz_ros_node,
    QWidget *parent)
    : app_(app), node_weak_ptr_(rviz_ros_node), QMainWindow(parent)
    , ui(new Ui::HelloGui)
{
    ui->setupUi(this);

    connect(ui->joystick_widget, &RRJoystick::xChanged, this, [this](float x){
        publish_cmd_vel(x, ui->joystick_widget->y());
    });


    connect(ui->joystick_widget, &RRJoystick::yChanged, this, [this](float y){
        publish_cmd_vel(ui->joystick_widget->x(), y);
    });

    rviz_panel = std::make_shared<RvizPanel>(app_);
//    rviz_panel->initializeWidget(node_weak_ptr_);
    ui->rviz_dock_parent_4->setWidget(rviz_panel->initializeWidget(node_weak_ptr_));
    // ui->rviz_widget->initialize(node, ui->rviz_widget);

    node = node_weak_ptr_.lock()->get_raw_node();
    publisher = node->create_publisher<std_msgs::msg::String>(
        "talker",
        1);
    subscription = node->create_subscription<std_msgs::msg::String>(
        "listener",
        1,
        std::bind(
            &HelloGui::subscription_callback,
            this,
            std::placeholders::_1));

    twist_publisher = node->create_publisher<geometry_msgs::msg::Twist>(
        "cmd_vel",
        1);

    client_cb_group = node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    service_client = node->create_client<example_interfaces::srv::AddTwoInts>(
        "/add_two_ints",
        rmw_qos_profile_services_default,
        client_cb_group);

    action_client = rclcpp_action::create_client<action_tutorials_interfaces::action::Fibonacci>(
        node,
        "/fibonacci");

    ros_timer = new QTimer(this);
    connect(ros_timer, SIGNAL(timeout()), this, SLOT(spin_once()));
    ros_timer->start(100);

    count_timer = new QTimer(this);
    connect(count_timer, SIGNAL(timeout()), this, SLOT(update_count()));
    count_timer->start(1000);

    player = new QMediaPlayer;
    player->setVideoOutput(ui->rtsp_video);
    player->setMedia(QUrl("rtsp://192.168.1.201:8554/unicast"));
    player->play();


}

HelloGui::~HelloGui()
{
    delete ui;
}

void HelloGui::spin_once()
{
    if (rclcpp::ok()) {
        app_->processEvents();
    } else {
        app_->quit();
    }
}

void HelloGui::update_count()
{
    static uint64_t count = 0;
    std::string s = "Count: " + std::to_string(count);
    auto q_string_msg = QString::fromStdString(s.c_str());
    ui->count_lbl->setText(q_string_msg);
    count++;
}

void HelloGui::subscription_callback(const std_msgs::msg::String & msg)
{
    auto q_string_msg = QString::fromStdString(msg.data.c_str());
    ui->topic_subscription_lbl->setText(q_string_msg);
}

void HelloGui::on_publish_to_topic_clicked()
{
    auto msg_data = std_msgs::msg::String();
    msg_data.data = "qwerty";
    publisher->publish(msg_data);
}


void HelloGui::on_add_btn_clicked()
{
    uint64_t int_1 = ui->int_1_edit->text().toInt();
    uint64_t int_2 = ui->int_2_edit->text().toInt();

    auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
    request->a = int_1;
    request->b = int_2;

    auto response_received_callback = [this](rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedFuture future) {
        auto result = future.get();
        handle_client_response(result);
    };
    auto result = service_client->async_send_request(request, response_received_callback);

    RCLCPP_INFO(node->get_logger(), "Sent request");
}

void HelloGui::handle_client_response(example_interfaces::srv::AddTwoInts::Response::SharedPtr response)
{
    RCLCPP_INFO(node->get_logger(), "Response Received");
    uint64_t sum = response->sum;
    std::string response_msg = "Result: " + std::to_string(sum);
    auto q_response_msg = QString::fromStdString(response_msg.c_str());
    ui->service_result_lbl->setText(q_response_msg);
}

void HelloGui::on_send_goal_btn_clicked()
{
    uint32_t order = ui->order_edit->text().toInt();
    action_tutorials_interfaces::action::Fibonacci::Goal goal;
    goal.order = order;

    auto send_goal_options = rclcpp_action::Client<action_tutorials_interfaces::action::Fibonacci>::SendGoalOptions();
    send_goal_options.feedback_callback = std::bind(
        &HelloGui::action_feedback,
        this,
        std::placeholders::_1,
        std::placeholders::_2);

    send_goal_options.result_callback = std::bind(
        &HelloGui::action_result_callback,
        this,
        std::placeholders::_1);

    auto goal_handle_future = action_client->async_send_goal(goal, send_goal_options);

    RCLCPP_INFO(node->get_logger(), "Sent goal");
}

void HelloGui::action_feedback(
    const rclcpp_action::ClientGoalHandle<action_tutorials_interfaces::action::Fibonacci>::SharedPtr,
    const std::shared_ptr<const typename action_tutorials_interfaces::action::Fibonacci::Feedback> feedback)
{
    std::string num = "0";
    for(auto number : feedback->partial_sequence) {
        num = std::to_string(number);
    }

    std::string fbk = "Feedback: next number in sequence is " + num;
    ui->action_feedback_lbl->setText(QString::fromStdString(fbk.c_str()));
    RCLCPP_INFO(node->get_logger(), "Feedback Received");
}

void HelloGui::action_result_callback(
    const rclcpp_action::ClientGoalHandle<action_tutorials_interfaces::action::Fibonacci>::WrappedResult &result)
{
    QStringList list;
    uint8_t row = 0;
    for (auto number : result.result->sequence) {
        std::string num = std::to_string(number);
        QListWidgetItem *item = new QListWidgetItem;
        item->setText(QString::fromStdString(num.c_str()));
        ui->result_list->insertItem(row,item);
        row++;
    }


    RCLCPP_INFO(node->get_logger(), "Result Received");
}

void HelloGui::publish_cmd_vel(float x, float y)
{
    auto cmd_vel_twist = geometry_msgs::msg::Twist();
    cmd_vel_twist.linear.x = -y;
    cmd_vel_twist.angular.z = x;

    twist_publisher->publish(cmd_vel_twist);
}

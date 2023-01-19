#include "gui/hello_gui.h"

#include <QApplication>
#include "rclcpp/rclcpp.hpp"
#include "rviz_common/ros_integration/ros_node_abstraction.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    QApplication a(argc, argv);

    auto node_weak_ptr = std::make_shared<rviz_common::ros_integration::RosNodeAbstraction>(
        "gui_test");
    auto hello_gui = std::make_shared<HelloGui>(&a, node_weak_ptr);
    hello_gui->setWindowTitle("Testing");
    hello_gui->show();
    return a.exec();
}

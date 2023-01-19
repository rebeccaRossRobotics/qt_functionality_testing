#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

#include <memory>

class AddTwoInts : public rclcpp::Node
{
public:
  AddTwoInts()
  : Node("add_two_ints_server")
  {
    service = create_service<example_interfaces::srv::AddTwoInts>("add_two_ints",
      std::bind(
        &AddTwoInts::add,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

    RCLCPP_INFO(get_logger(), "Ready to add two ints.");
  }

private:
  rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr service;

  void add(const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
          std::shared_ptr<example_interfaces::srv::AddTwoInts::Response>      response)
  {
    response->sum = request->a + request->b;

    std::this_thread::sleep_for(std::chrono::hours(1));
    RCLCPP_INFO(get_logger(), "Incoming request\na: %ld" " b: %ld",
                  request->a, request->b);
    RCLCPP_INFO(get_logger(), "sending back response: [%ld]", (long int)response->sum);
  }
};


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<AddTwoInts>();

  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}

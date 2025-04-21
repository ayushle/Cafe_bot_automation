#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int8.hpp"
#include <regex>

using std::placeholders::_1;

class OrderHandler : public rclcpp::Node
{
public:
  OrderHandler()
  : Node("order_handler")
  {
    table_pub_ = this->create_publisher<std_msgs::msg::Int8>("table_number", 10);
    order_pub_ = this->create_publisher<std_msgs::msg::String>("order_name", 10);

    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "give_order", 10, std::bind(&OrderHandler::order_callback, this, _1));
  }

private:
  void order_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    std::string input = msg->data;
    std::regex pattern(R"(Table\s*(\d+)\s*:\s*(.+))");
    std::smatch match;

    if (std::regex_match(input, match, pattern)) {
      int table_number = std::stoi(match[1]);
      std::string order_name = match[2];

      std_msgs::msg::Int8 table_msg;
      table_msg.data = static_cast<int8_t>(table_number);
      table_pub_->publish(table_msg);

      std_msgs::msg::String order_msg;
      order_msg.data = order_name;
      order_pub_->publish(order_msg);

      RCLCPP_INFO(this->get_logger(), "Parsed table: %d, order: %s",
                  table_number, order_name.c_str());
    } else {
      RCLCPP_WARN(this->get_logger(), "Invalid order format: '%s'", input.c_str());
    }
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr table_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr order_pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OrderHandler>());
  rclcpp::shutdown();
  return 0;
}

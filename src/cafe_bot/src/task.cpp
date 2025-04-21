#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int8.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <yaml-cpp/yaml.h>
#include <unordered_set>
#include <queue>
#include <cmath>
#include <string>

class Navigator : public rclcpp::Node
{
public:
  Navigator() : Node("goal_pose_publisher"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
  {
    table_sub_ = this->create_subscription<std_msgs::msg::Int8>(
      "/table_number", 10, std::bind(&Navigator::table_callback, this, std::placeholders::_1));

    cancel_sub_ = this->create_subscription<std_msgs::msg::Int8>(
      "/cancel_order", 10, std::bind(&Navigator::cancel_callback, this, std::placeholders::_1));

    goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);

    timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&Navigator::check_goal_reached, this));
  }

private:
  std::queue<int> order_queue;
  std::unordered_set<int> canceled_orders;
  geometry_msgs::msg::Pose goal_pose_;
  bool goal_active_ = false;
  bool any_cancellation_ = false;
  std::string state_ = "waiting";
  int current_table_ = -1;

  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr table_sub_;
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr cancel_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  void table_callback(const std_msgs::msg::Int8::SharedPtr msg)
  {
    int table = msg->data;
    order_queue.push(table);
    print_status("Order received: Table " + std::to_string(table));

    if (state_ == "waiting") {
      state_ = "going_to_kitchen";
      navigate_to("kitchen");
    }
  }

  void cancel_callback(const std_msgs::msg::Int8::SharedPtr msg)
  {
    int cancel_table = msg->data;
    canceled_orders.insert(cancel_table);
    any_cancellation_ = true;
    print_status("Order canceled for Table " + std::to_string(cancel_table));
  }

  void check_goal_reached()
  {
    if (!goal_active_) return;

    try {
      auto transform = tf_buffer_.lookupTransform("map", "base_link", tf2::TimePointZero);
      double dx = transform.transform.translation.x - goal_pose_.position.x;
      double dy = transform.transform.translation.y - goal_pose_.position.y;
      double distance = std::sqrt(dx * dx + dy * dy);

      if (distance < 0.4) {
        goal_active_ = false;

        if (state_ == "going_to_kitchen") {
          state_ = "delivering";
          deliver_next();
        }
        else if (state_ == "delivering") {
          deliver_next();
        }
        else if (state_ == "returning_to_kitchen") {
          navigate_to("home");
          state_ = "returning_home";
        }
        else if (state_ == "returning_home") {
          print_status("Robot returned home. System idle.");
          reset();
        }
      }
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(), "TF error: %s", ex.what());
    }
  }

  void deliver_next()
  {
    while (!order_queue.empty()) {
      int table = order_queue.front();
      order_queue.pop();
      if (canceled_orders.count(table)) {
        print_status("Skipping canceled Table " + std::to_string(table));
        continue;
      }
      current_table_ = table;
      print_status("Delivering to Table " + std::to_string(table));
      navigate_to("table" + std::to_string(table));
      return;
    }

    if (any_cancellation_) {
      state_ = "returning_to_kitchen";
      navigate_to("kitchen");
    } else {
      state_ = "returning_home";
      navigate_to("home");
    }
  }

  void reset()
  {
    state_ = "waiting";
    current_table_ = -1;
    canceled_orders.clear();
    any_cancellation_ = false;
  }

  void navigate_to(const std::string &key)
  {
    std::string path = "/home/ayush/Documents/cafeBot_ws/src/cafe_bot/config/location.yaml";
    YAML::Node config = YAML::LoadFile(path);

    if (!config[key]) {
      RCLCPP_WARN(this->get_logger(), "Location key '%s' not found in YAML", key.c_str());
      return;
    }

    std::vector<double> coords = config[key].as<std::vector<double>>();
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = this->now();
    pose.header.frame_id = "map";
    pose.pose.position.x = coords[0];
    pose.pose.position.y = coords[1];
    pose.pose.position.z = 0.0;

    float theta = coords[2];
    pose.pose.orientation.z = std::sin(theta / 2.0);
    pose.pose.orientation.w = std::cos(theta / 2.0);

    goal_pose_ = pose.pose;
    goal_pub_->publish(pose);
    goal_active_ = true;

    print_status("Navigating to " + key);
  }

  void print_status(const std::string &msg)
  {
    RCLCPP_INFO(this->get_logger(), "[STATUS] %s", msg.c_str());
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Navigator>());
  rclcpp::shutdown();
  return 0;
}
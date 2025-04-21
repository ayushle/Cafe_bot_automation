#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int8.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cmath>
#include <iostream>
#include <queue>
#include <unordered_set>
#include <unordered_map>

class Navigator : public rclcpp::Node
{
public:
  Navigator() : Node("goal_pose_publisher"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
  {
    table_pos_sub_ = this->create_subscription<std_msgs::msg::Int8>(
      "/table_number", 10, std::bind(&Navigator::table_callback, this, std::placeholders::_1));

    cancel_sub_ = this->create_subscription<std_msgs::msg::Int8>(
      "/cancel_order", 10, std::bind(&Navigator::cancel_callback, this, std::placeholders::_1));

    confirm_sub_ = this->create_subscription<std_msgs::msg::Int8>(
      "/confirm_order", 10, std::bind(&Navigator::confirm_callback, this, std::placeholders::_1));

    goal_pos_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);

    timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&Navigator::check_goal_reached, this));
  }

private:
  std::queue<int> table_queue;
  std::unordered_set<int> confirmed_tables_;
  std::unordered_map<int, bool> delivery_status_;
  geometry_msgs::msg::Pose goal_pose_;
  bool goal_active_ = false;
  std::string state_ = "waiting_for_order";
  int current_table_ = -1;
  int total_tables_to_serve_ = 0;

  void table_callback(const std_msgs::msg::Int8::SharedPtr msg)
  {
    int table_no = msg->data;
    table_queue.push(table_no);
    delivery_status_[table_no] = false;
    total_tables_to_serve_++;
    print_status("Received table order: Table " + std::to_string(table_no));

    if (state_ == "waiting_for_order") {
      nav_kitchen();
      state_ = "going_to_kitchen";
    }
  }

  void cancel_callback(const std_msgs::msg::Int8::SharedPtr msg)
  {
    int cancel_table = msg->data;
    std::queue<int> temp;
    while (!table_queue.empty()) {
      if (table_queue.front() != cancel_table)
        temp.push(table_queue.front());
      table_queue.pop();
    }
    table_queue = temp;
    delivery_status_.erase(cancel_table);
    print_status("Canceled delivery to Table " + std::to_string(cancel_table));
  }

  void confirm_callback(const std_msgs::msg::Int8::SharedPtr msg)
  {
    int confirm_table = msg->data;
    confirmed_tables_.insert(confirm_table);
    delivery_status_[confirm_table] = true;
    print_status("Order confirmed at Table " + std::to_string(confirm_table));
  }

  void nav_to_location(const std::string& key)
  {
    std::string yaml_path = "/home/ayush/Documents/cafeBot_ws/src/cafe_bot/config/location.yaml";
    YAML::Node config = YAML::LoadFile(yaml_path);

    if (!config[key]) {
      RCLCPP_WARN(this->get_logger(), "%s not found in location.yaml", key.c_str());
      return;
    }

    std::vector<double> coords = config[key].as<std::vector<double>>();
    geometry_msgs::msg::PoseStamped goal_msg;
    goal_msg.header.stamp = this->now();
    goal_msg.header.frame_id = "map";
    goal_msg.pose.position.x = coords[0];
    goal_msg.pose.position.y = coords[1];
    goal_msg.pose.position.z = 0.0;

    float theta = coords[2];
    goal_msg.pose.orientation.z = std::sin(theta / 2.0);
    goal_msg.pose.orientation.w = std::cos(theta / 2.0);

    goal_pose_ = goal_msg.pose;
    goal_active_ = true;
    goal_pos_pub_->publish(goal_msg);
  }

  void nav_kitchen() {
    print_status("Heading to kitchen");
    nav_to_location("kitchen");
  }

  void nav_table(int table_no) {
    print_status("Heading to Table " + std::to_string(table_no));
    nav_to_location("table" + std::to_string(table_no));
    current_table_ = table_no;
  }

  void nav_home() {
    print_status("Returning to home");
    nav_to_location("home");
  }

  void check_goal_reached()
  {
    if (!goal_active_) return;

    try {
      auto transform = tf_buffer_.lookupTransform("map", "base_link", tf2::TimePointZero);
      double dx = transform.transform.translation.x - goal_pose_.position.x;
      double dy = transform.transform.translation.y - goal_pose_.position.y;
      double distance = std::sqrt(dx * dx + dy * dy);

      if (distance < 0.35) {
        goal_active_ = false;

        if (state_ == "going_to_kitchen") {
          if (!table_queue.empty()) {
            int next_table = table_queue.front();
            table_queue.pop();
            nav_table(next_table);
            current_table_ = next_table;
            state_ = "delivering_to_table";
          } else {
            nav_home();
            state_ = "returning_home";
          }
        } else if (state_ == "delivering_to_table") {
          if (!table_queue.empty()) {
            int next_table = table_queue.front();
            table_queue.pop();
            nav_table(next_table);
            current_table_ = next_table;
          } else {
            int served = 0;
            for (const auto& [table, served_flag] : delivery_status_) {
              if (served_flag) served++;
            }
            if (served == total_tables_to_serve_) {
              nav_home();
              state_ = "returning_home";
            } else {
              nav_kitchen();
              state_ = "going_to_kitchen_to_return";
            }
          }
        } else if (state_ == "going_to_kitchen_to_return") {
          nav_home();
          state_ = "returning_home";
        } else if (state_ == "returning_home") {
          print_status("Robot is ready for next order.");
          state_ = "waiting_for_order";
          delivery_status_.clear();
          confirmed_tables_.clear();
          total_tables_to_serve_ = 0;
        }
      }
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(), "Could not transform: %s", ex.what());
    }
  }

  void print_status(const std::string& msg)
  {
    RCLCPP_INFO(this->get_logger(), "[Status] %s", msg.c_str());
  }

  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr table_pos_sub_;
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr cancel_sub_;
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr confirm_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pos_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Navigator>());
  rclcpp::shutdown();
  return 0;
}
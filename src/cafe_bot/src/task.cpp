#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cmath>
#include <iostream>
#include <queue>

class Navigator : public rclcpp::Node
{
public:
  Navigator() : Node("goal_pose_publisher"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
  {
    table_pos_sub_ = this->create_subscription<std_msgs::msg::Int8>(
        "/table_number", 10, std::bind(&Navigator::table_callback, this, std::placeholders::_1));

    cancel_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/cancel_order", 10, std::bind(&Navigator::cancel_callback, this, std::placeholders::_1));

    goal_pos_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);

    timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&Navigator::check_goal_reached, this));
  }

private:
  geometry_msgs::msg::Pose goal_pose_;
  bool goal_active_ = false;
  bool kitchen_visited_ = false;
  bool returning_home_ = false;
  std::queue<int> order_queue_;

  void table_callback(const std_msgs::msg::Int8::SharedPtr msg)
  {
    order_queue_.push(msg->data);
    if (!goal_active_ && !kitchen_visited_)
    {
      nav_kitchen();
    }
  }

  void cancel_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (msg->data)
    {
      RCLCPP_WARN(this->get_logger(), "Cancel order received. Returning to home.");
      clear_orders();
      nav_home();
    }
  }

  void clear_orders()
  {
    std::queue<int> empty;
    std::swap(order_queue_, empty);
    kitchen_visited_ = false;
    returning_home_ = true;
  }

  void nav_to_pose(const std::string &key)
  {
    std::string yaml_path = "/home/ayush/Documents/cafeBot_ws/src/cafe_bot/config/location.yaml";
    YAML::Node config = YAML::LoadFile(yaml_path);

    std::vector<double> coords = config[key].as<std::vector<double>>();

    geometry_msgs::msg::PoseStamped goal_msg;
    goal_msg.header.stamp = this->now();
    goal_msg.header.frame_id = "map";
    goal_msg.pose.position.x = coords[0];
    goal_msg.pose.position.y = coords[1];
    goal_msg.pose.position.z = 0.0;

    float theta = coords[2];
    goal_msg.pose.orientation.x = 0.0;
    goal_msg.pose.orientation.y = 0.0;
    goal_msg.pose.orientation.z = std::sin(theta / 2.0);
    goal_msg.pose.orientation.w = std::cos(theta / 2.0);

    goal_pose_ = goal_msg.pose;
    goal_active_ = true;
    goal_pos_pub_->publish(goal_msg);
  }

  void nav_kitchen()
  {
    RCLCPP_INFO(this->get_logger(), "Navigating to kitchen...");
    nav_to_pose("kitchen");
  }

  void nav_table(int table_no)
  {
    RCLCPP_INFO(this->get_logger(), "Navigating to table %d...", table_no);
    nav_to_pose("table" + std::to_string(table_no));
  }

  void nav_home()
  {
    RCLCPP_INFO(this->get_logger(), "Returning to home...");
    nav_to_pose("home");
  }

  void check_goal_reached()
  {
    if (!goal_active_)
      return;

    try
    {
      geometry_msgs::msg::TransformStamped transform = tf_buffer_.lookupTransform("map", "base_link", tf2::TimePointZero);
      double dx = transform.transform.translation.x - goal_pose_.position.x;
      double dy = transform.transform.translation.y - goal_pose_.position.y;
      double distance = std::sqrt(dx * dx + dy * dy);

      if (distance < 0.35)
      {
        RCLCPP_INFO(this->get_logger(), "Goal Reached!");
        goal_active_ = false;

        if (!kitchen_visited_)
        {
          kitchen_visited_ = true;
          if (!order_queue_.empty())
          {
            int next_table = order_queue_.front();
            order_queue_.pop();
            nav_table(next_table);
          }
          else
          {
            nav_home();
          }
        }
        else if (!order_queue_.empty())
        {
          int next_table = order_queue_.front();
          order_queue_.pop();
          nav_table(next_table);
        }
        else if (!returning_home_)
        {
          returning_home_ = true;
          nav_home();
        }
        else
        {
          returning_home_ = false;
          kitchen_visited_ = false;
          RCLCPP_INFO(this->get_logger(), "All tasks completed. Waiting for new orders.");
        }
      }
    }
    catch (const tf2::TransformException &ex)
    {
      RCLCPP_WARN(this->get_logger(), "Could not transform: %s", ex.what());
    }
  }

  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr table_pos_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr cancel_sub_;
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
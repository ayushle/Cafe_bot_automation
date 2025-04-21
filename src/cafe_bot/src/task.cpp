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
int time_passed=0;
int timeout=5;
class Navigator : public rclcpp::Node
{
public:
  Navigator() : Node("goal_pose_publisher"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
  {
    table_pos_sub_ = this->create_subscription<std_msgs::msg::Int8>(
      "/table_number", 10, std::bind(&Navigator::corr_callback, this, std::placeholders::_1));

    goal_pos_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);

    timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&Navigator::check_goal_reached, this));
    done_pub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/done", 10, std::bind(&Navigator::nav_next, this, std::placeholders::_1));
      cancel_pub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/cancel_order", 10,
        std::bind(&Navigator::cancel_order, this, std::placeholders::_1));
    
  }

private:
  geometry_msgs::msg::Pose goal_pose_;
  geometry_msgs::msg::Pose goal_pose_kit;
  geometry_msgs::msg::Pose goal_pose_tab;
  geometry_msgs::msg::Pose goal_pose_hom;

  bool goal_active_ = false;
  int home_flag=0;
  int kitchen_flag=0;
  int table_flag=0;
  int done_flag=0;
  int kit=0;
  int tab=0;
  int hom=0;
  bool cancel_flag=false;



  void nav_kitchen()
  {
    time_passed=0;
    
    std::string yaml_path = "/home/ayush/Documents/cafeBot_ws/src/cafe_bot/config/location.yaml";
    YAML::Node config = YAML::LoadFile(yaml_path);

    std::string table_key = "kitchen" ;
    std::vector<double> coords = config[table_key].as<std::vector<double>>();
    
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
    goal_pose_kit=goal_msg.pose;
  }
  void nav_table(const std_msgs::msg::Int8::SharedPtr msg)
  {
    done_flag=0;
    time_passed=0;
    int table_no = msg->data;
    std::string yaml_path = "/home/ayush/Documents/cafeBot_ws/src/cafe_bot/config/location.yaml";
    YAML::Node config = YAML::LoadFile(yaml_path);

    std::string table_key = "table" + std::to_string(table_no);
    if (!config[table_key])
    {
      RCLCPP_WARN(this->get_logger(), "Table%d not found in location.yaml", table_no);
      return;
    }

    std::vector<double> coords = config[table_key].as<std::vector<double>>();
    RCLCPP_INFO(this->get_logger(), "Table %d coordinates: x=%.2f, y=%.2f, theta=%.2f", table_no, coords[0], coords[1], coords[2]);

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
    goal_pose_tab=goal_msg.pose;
  }
  void nav_home()
  {
    std::string yaml_path = "/home/ayush/Documents/cafeBot_ws/src/cafe_bot/config/location.yaml";
    YAML::Node config = YAML::LoadFile(yaml_path);

    std::string table_key = "home" ;
    std::vector<double> coords = config[table_key].as<std::vector<double>>();
    
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
    goal_pose_hom=goal_msg.pose;
  }
  void corr_callback(const std_msgs::msg::Int8::SharedPtr msg)
  { 
    if (kitchen_flag==0){
      nav_kitchen();
      }else if (table_flag==0 && done_flag==1){
        nav_table(msg);
      }else if(home_flag==0 && done_flag==1){
        nav_home();
      }
    
      try
      {
        geometry_msgs::msg::TransformStamped transform = tf_buffer_.lookupTransform("map", "base_link", tf2::TimePointZero);
        double dx = transform.transform.translation.x - goal_pose_kit.position.x;
        double dy = transform.transform.translation.y - goal_pose_kit.position.y;
        double distance_kit = std::sqrt(dx * dx + dy * dy);
  
        if (distance_kit < 0.35)
        {
          kit=1;
          tab=0;
          hom=0;
        }
      }catch (const tf2::TransformException &ex)
      {}
      try
      {
        geometry_msgs::msg::TransformStamped transform = tf_buffer_.lookupTransform("map", "base_link", tf2::TimePointZero);
        double dx = transform.transform.translation.x - goal_pose_tab.position.x;
        double dy = transform.transform.translation.y - goal_pose_tab.position.y;
        double distance_kit = std::sqrt(dx * dx + dy * dy);
  
        if (distance_kit < 0.35)
        {
          kit=0;
          tab=1;
          hom=0;
        }
      }catch (const tf2::TransformException &ex)
      {}
      try
      {
        geometry_msgs::msg::TransformStamped transform = tf_buffer_.lookupTransform("map", "base_link", tf2::TimePointZero);
        double dx = transform.transform.translation.x - goal_pose_hom.position.x;
        double dy = transform.transform.translation.y - goal_pose_hom.position.y;
        double distance_kit = std::sqrt(dx * dx + dy * dy);
  
        if (distance_kit < 0.35)
        {
          kit=0;
          tab=0;
          hom=1;
        }
      }catch (const tf2::TransformException &ex)
      {}
  }

  void check_goal_reached()
  {
    RCLCPP_INFO(this->get_logger(), "time=%d", time_passed);
    
    if (time_passed<timeout){

    try
    {
      geometry_msgs::msg::TransformStamped transform = tf_buffer_.lookupTransform("map", "base_link", tf2::TimePointZero);
      double dx = transform.transform.translation.x - goal_pose_.position.x;
      double dy = transform.transform.translation.y - goal_pose_.position.y;
      double distance = std::sqrt(dx * dx + dy * dy);

      if (distance < 0.35)
      {
        time_passed+=1;
        RCLCPP_INFO(this->get_logger(), "Goal Reached! Robot is within %.2f meters of the goal.", distance);
        if (kitchen_flag==0){
          kitchen_flag=1;
          }else if (table_flag==0){
            table_flag=1;
          }else if(home_flag==0){
            home_flag=1;
          }
        goal_active_ = false;
      }
      else
      {
        RCLCPP_INFO(this->get_logger(), "Distance to goal: %.2f", distance);
      }
    }
    catch (const tf2::TransformException &ex)
    {
      RCLCPP_WARN(this->get_logger(), "Could not transform: %s", ex.what());
    }}else {
      if (tab==1){
        nav_kitchen();}
      else{
      nav_home();
    }}
    }
  

  void nav_next(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if(msg->data==true)  {
      done_flag=1;
    }
  }
  void cancel_order(const std_msgs::msg::Bool::SharedPtr msg)
  {
    cancel_flag=msg->data;
    if(msg->data){
    if(home_flag==0 && kitchen_flag==0 && table_flag==0){
      nav_home();
      
    }else if (kit==1){
        
        nav_kitchen();

    }}
  }
  
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr table_pos_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pos_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr done_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr cancel_pub_;
  

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
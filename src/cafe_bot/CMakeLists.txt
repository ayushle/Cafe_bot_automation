cmake_minimum_required(VERSION 3.8)
project(cafe_bot)

# Standard options for warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)

# Declare the executable
add_executable(order src/order.cpp)
add_executable(task src/task.cpp)




# Link against necessary libraries
ament_target_dependencies(order rclcpp std_msgs)
ament_target_dependencies(task rclcpp std_msgs geometry_msgs ament_index_cpp nav2_msgs tf2 tf2_ros tf2_geometry_msgs) 



target_link_libraries(task yaml-cpp)




# Install the executable
install(TARGETS order
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS
  task
  DESTINATION lib/${PROJECT_NAME}
)

# Install resource files (including msgs, launch files, etc.)
install(
  DIRECTORY config description launch world 
  DESTINATION share/${PROJECT_NAME}
)
ament_package()

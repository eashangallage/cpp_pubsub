cmake_minimum_required(VERSION 3.8)
project(cpp_pubsub)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(listener src/my_sbr.cpp)
ament_target_dependencies(listener rclcpp std_msgs geometry_msgs sensor_msgs)

install(TARGETS
  listener
  DESTINATION lib/${PROJECT_NAME})
  
add_executable(talker src/my_pub.cpp)
ament_target_dependencies(talker rclcpp std_msgs geometry_msgs sensor_msgs)
  
install(TARGETS
  talker    
  DESTINATION lib/${PROJECT_NAME})
  
add_executable(commander src/pubsub_node.cpp)
ament_target_dependencies(commander rclcpp std_msgs geometry_msgs sensor_msgs)

install(TARGETS
  commander
  DESTINATION lib/${PROJECT_NAME})

ament_package()

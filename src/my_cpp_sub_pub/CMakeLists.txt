# CMakeLists.txt

cmake_minimum_required(VERSION 3.5)
project(my_cpp_sub_pub)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(mocap4r2_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(px4_ros_com REQUIRED)

add_executable(sub_pub src/sub_pub.cpp)
ament_target_dependencies(sub_pub rclcpp std_msgs mocap4r2_msgs px4_msgs px4_ros_com)

install(TARGETS
  sub_pub
  DESTINATION lib/${PROJECT_NAME})

ament_package()


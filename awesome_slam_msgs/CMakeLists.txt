cmake_minimum_required(VERSION 3.0.2)
project(awesome_slam_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation)

add_message_files(
  FILES
  Landmarks.msg
)

generate_messages(DEPENDENCIES)

catkin_package(CATKIN_DEPENDS message_runtime)

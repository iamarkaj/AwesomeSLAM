cmake_minimum_required(VERSION 3.0.2)
project(awesome_slam)

find_package(catkin REQUIRED COMPONENTS roscpp message_generation)

find_package(gazebo REQUIRED turtlebot3_gazebo turtlebot3_teleop)

add_message_files(
  FILES
  Landmarks.msg
)

generate_messages(
  DEPENDENCIES
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS turtlebot3_gazebo turtlebot3_teleop message_runtime
  DEPENDS GAZEBO 
)

link_directories(${GAZEBO_LIBRARY_DIRS})

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

add_executable(ekf src/ekf.cpp)

install(DIRECTORY 
  models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

add_dependencies(ekf 
  ${ekf_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(ekf 
  ${catkin_LIBRARIES} 
  ${GAZEBO_LIBRARIES}
)
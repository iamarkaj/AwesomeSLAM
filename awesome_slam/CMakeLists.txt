cmake_minimum_required(VERSION 3.0.2)
project(awesome_slam)

find_package(catkin REQUIRED COMPONENTS roscpp awesome_slam_msgs)

find_package(gazebo REQUIRED turtlebot3_gazebo turtlebot3_teleop)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS turtlebot3_gazebo turtlebot3_teleop awesome_slam_msgs
  DEPENDS GAZEBO 
)

link_directories(${GAZEBO_LIBRARY_DIRS})

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

install(DIRECTORY 
  models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

add_executable(ekf src/ekf.cpp)
add_dependencies(ekf ${ekf_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ekf ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_executable(visualize src/visualize.cpp)
add_dependencies(visualize ${visualize_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(visualize ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_executable(ukf src/ukf.cpp)
add_dependencies(ukf ${ekf_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ukf ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
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

add_executable(gazebo_spawn src/gazebo_spawn/gazebo_spawn.cpp)
add_dependencies(gazebo_spawn ${gazebo_spawn_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(gazebo_spawn ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_executable(sensor_landmark src/sensor_landmark/sensor_landmark.cpp)
add_dependencies(sensor_landmark ${sensor_landmark_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(sensor_landmark ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_executable(visualize_landmark src/visualize_landmark/visualize_landmark.cpp)
add_dependencies(visualize_landmark ${rviz_visualize_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(visualize_landmark ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_executable(ekf src/ekf/ekf.cpp)
add_dependencies(ekf ${ekf_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ekf ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_executable(ukf src/ukf/ukf.cpp)
add_dependencies(ukf ${ukf_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ukf ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
# cmake_minimum_required(VERSION 3.0.2)
# project(awesome_slam)
# cmake_policy(SET CMP0054 NEW)

# add_compile_options(-std=c++11)

# find_package(catkin REQUIRED COMPONENTS
# )

# find_package(gazebo REQUIRED turtlebot3_drive turtlebot3_gazebo turtlebot3_teleop)

# catkin_package(
#   INCLUDE_DIRS include
#   CATKIN_DEPENDS turtlebot3_drive turtlebot3_gazebo turtlebot3_teleop
#   DEPENDS GAZEBO 
# )

# link_directories(${GAZEBO_LIBRARY_DIRS})

# include_directories(
#   include
#   ${catkin_INCLUDE_DIRS}
#   ${GAZEBO_INCLUDE_DIRS}
# )

# add_executable(turtlebot3_drive src/turtlebot3_drive.cpp)
# add_dependencies(turtlebot3_drive ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(turtlebot3_drive ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

# install(TARGETS turtlebot3_drive
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
# )

# install(DIRECTORY launch models rviz worlds 
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

cmake_minimum_required(VERSION 3.0.2)
project(awesome_slam)

find_package(catkin REQUIRED COMPONENTS roscpp)

find_package(gazebo REQUIRED turtlebot3_gazebo turtlebot3_teleop)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS turtlebot3_gazebo turtlebot3_teleop
  DEPENDS GAZEBO 
)

link_directories(${GAZEBO_LIBRARY_DIRS})

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

add_executable(laser_test src/laser_test.cpp)

install(DIRECTORY 
  models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

add_dependencies(laser_test 
  ${laser_test_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(laser_test 
  ${catkin_LIBRARIES} 
  ${GAZEBO_LIBRARIES}
)
cmake_minimum_required(VERSION 3.0.2)
project(GuidAR)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  std_srvs
  geometry_msgs
  trajectory_msgs
  message_generation
)

## Messages
add_message_files(FILES
  Goal.msg
  ObstacleListList.msg
  ObstacleList.msg
  Obstacle.msg
  Point.msg
  PotentialFieldMsg.msg
  ARObstacle.msg)

## Services
add_service_files(FILES
  generateTraj.srv
  modifyTrajectory.srv
)

generate_messages(DEPENDENCIES
  std_msgs
  geometry_msgs
  trajectory_msgs
)



catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs std_srvs geometry_msgs trajectory_msgs message_runtime
)


install(PROGRAMS
  scripts/guidar_services.py
  scripts/obstacle_publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


include_directories(${catkin_INCLUDE_DIRS})
add_executable(trajectory_modifier src/trajectory_modifier.cpp)
add_dependencies(trajectory_modifier ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(trajectory_modifier ${catkin_LIBRARIES})

cmake_minimum_required(VERSION 3.0.2)
project(GuidAR)

find_package(catkin REQUIRED COMPONENTS
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
  CATKIN_DEPENDS rospy std_msgs std_srvs geometry_msgs trajectory_msgs message_runtime
)

install(PROGRAMS
  scripts/guidar_services.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

cmake_minimum_required(VERSION 2.8.3)
project(%ROBOT_NAME%)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  urdf
  xacro
)

catkin_package()

install(DIRECTORY meshes urdf launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
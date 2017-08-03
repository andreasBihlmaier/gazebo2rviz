cmake_minimum_required(VERSION 2.8.3)
project(gazebo2rviz)

find_package(catkin REQUIRED COMPONENTS
  rospy
  pysdf
)

catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES gazebo2rviz
  CATKIN_DEPENDS rospy pysdf python-rospkg
#  DEPENDS system_lib
)


###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
)


#############
## Install ##
#############
install(PROGRAMS
  src/gazebo2rviz/gazebo2tf_node.py
  src/gazebo2rviz/gazebo2marker_node.py
  src/gazebo2rviz/sdf2marker_node.py
  src/gazebo2rviz/sdf2tfstatic_node.py
  src/gazebo2rviz/sdf2extract_tfstatic_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

cmake_minimum_required(VERSION 2.8.3)
project(dji_bridge)

find_package(catkin REQUIRED COMPONENTS
   roscpp
   rospy
   std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS script/dji_bridge.py
					DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
## Mark cpp header files for installation

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)





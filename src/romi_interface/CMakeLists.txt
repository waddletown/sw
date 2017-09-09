cmake_minimum_required(VERSION 2.8.3)
project(romi_interface)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
)

catkin_package()
catkin_python_setup()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
install(PROGRAMS
  nodes/romi_driver.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES
  launch/*.launch
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

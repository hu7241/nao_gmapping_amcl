cmake_minimum_required(VERSION 2.8.3)
project(nao_slam_amcl)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  gmapping
  hector_mapping
  laser_scan_matcher
  map_server
  move_base
  nao_bringup
  rplidar_ros
  rviz
  tf
)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()


###################################
## catkin specific configuration ##
###################################

catkin_package(
#    CATKIN_DEPENDS
#	roscpp
#	dynamic_reconfigure
#	tf
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(nao_slam_amcl
#   src/${PROJECT_NAME}/nao_slam_amcl.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(nao_slam_amcl ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
# add_executable(nao_slam_amcl_node src/nao_slam_amcl_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(nao_slam_amcl_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(nao_slam_amcl_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS nao_slam_amcl nao_slam_amcl_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
catkin_install_python(PROGRAMS src/nao_walker_v2.py
    DESTINATION /opt/ros/indigo/lib/nao_apps/
)

catkin_install_python(PROGRAMS launch/nao_partial_py.launch
    DESTINATION /opt/ros/indigo/share/nao_bringup/launch/
)

catkin_install_python(PROGRAMS launch/walker_v2.launch
    DESTINATION /opt/ros/indigo/share/nao_apps/launch/
)

catkin_install_python(PROGRAMS src/naoqi_joint_states.py
    DESTINATION /opt/ros/indigo/lib/naoqi_driver_py/
)

#install(DIRECTORY launch
#    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)

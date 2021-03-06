cmake_minimum_required(VERSION 2.8.3)
project(scitos_2d_navigation)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
                    roscpp
                    rospy
                    sensor_msgs
                    std_msgs pcl_ros
                    tf
                    costmap_2d
                    dynamic_reconfigure
                    navfn
                    strands_perception_msgs
                    ros_datacentre_cpp_client
                    nav_core
                    pluginlib
                    actionlib_msgs
                    sound_play
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS} -std=c++0x)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

## Add dynamic reconfigure API
generate_dynamic_reconfigure_options(
  cfg/dynamic_layer_params.cfg
  cfg/overwrite_layer_params.cfg
  cfg/smart_obstacle_layer_params.cfg
)

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'action' folder
add_action_files(
  DIRECTORY action
  FILES UnleashStaticPlanner.action
)

## Generate messages in the 'msg' folder
#~ add_message_files(
  #~ DEPENDENCIES actionlib_msgs std_msgs
#~ )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
    std_msgs  # Or other packages containing msgs
    actionlib_msgs
)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES dynamic_layer overwrite_layer smart_obstacle_layer unleash_static_planner static_planner 
  CATKIN_DEPENDS costmap_2d dynamic_reconfigure roscpp nav_core actionlib_msgs
  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

## Declare a cpp library
#add_library(noise_voxel_grid src/noise_voxel_grid.cpp)
add_library(noise_approximate_voxel_grid src/noise_approximate_voxel_grid.cpp)
add_library(dynamic_layer plugins/dynamic_layer.cpp)
add_library(overwrite_layer plugins/overwrite_layer.cpp)
add_library(smart_obstacle_layer plugins/smart_obstacle_layer.cpp)
add_library(unleash_static_planner plugins/unleash_static_planner.cpp)

## Declare a cpp executable
add_executable(subsample_cloud src/subsample_cloud.cpp)
add_executable(remove_edges_cloud src/remove_edges_cloud.cpp)
add_executable(mirror_floor_points src/mirror_floor_points.cpp)
add_executable(static_planner src/static_planner.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
#add_dependencies(scitos_2d_navigation_node scitos_2d_navigation_generate_messages_cpp)
add_dependencies(dynamic_layer ${PROJECT_NAME}_gencfg) # please do not ask me why... black magic maybe?
add_dependencies(dynamic_layer plugins/dynamic_layer.cpp)
add_dependencies(overwrite_layer ${PROJECT_NAME}_gencfg) # please do not ask me why... black magic maybe?
add_dependencies(overwrite_layer plugins/overwrite_layer.cpp)
add_dependencies(smart_obstacle_layer ${PROJECT_NAME}_gencfg) # please do not ask me why... black magic maybe?
add_dependencies(smart_obstacle_layer plugins/smart_obstacle_layer.cpp)
add_dependencies(smart_obstacle_layer smart_obstacle_layer_gencfg) # please do not ask me why... black magic maybe?
add_dependencies(unleash_static_planner plugins/unleash_static_planner.cpp)
add_dependencies(unleash_static_planner strands_perception_msgs_generate_messages_cpp)
add_dependencies(static_planner src/static_planner.cpp)
add_dependencies(static_planner strands_perception_msgs_generate_messages_cpp)
add_dependencies(static_planner ${scitos_2d_navigation_EXPORTED_TARGETS})

target_link_libraries(noise_approximate_voxel_grid
  ${PCL_LIBRARIES}
)

target_link_libraries(subsample_cloud
  ${catkin_LIBRARIES} ${PCL_LIBRARIES} noise_approximate_voxel_grid
)

target_link_libraries(remove_edges_cloud
  ${catkin_LIBRARIES} ${PCL_LIBRARIES}
)

target_link_libraries(mirror_floor_points
  ${catkin_LIBRARIES} ${PCL_LIBRARIES}
)

target_link_libraries(unleash_static_planner
  ${catkin_LIBRARIES} ${PCL_LIBRARIES}
)

target_link_libraries(static_planner
  ${catkin_LIBRARIES} ${PCL_LIBRARIES}
)

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
install(TARGETS unleash_static_planner
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES cvr_plugin.xml
  # myfile1
  # myfile2
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##- 
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_scitos_2d_navigation.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

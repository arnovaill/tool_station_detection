cmake_minimum_required(VERSION 2.8.3)
project(tool_station_detection)

add_compile_options(-std=c++11)

if("$ENV{ROS_DISTRO}" STRGREATER "jade")
find_package( catkin REQUIRED 
  roscpp
  roslib
  eigen_conversions
  eigen_helper_functions
  # tool_exchanger_info_manager
  flexbotics_manipulator_manager_msgs
  flexbotics_manipulation_planner_msgs
  tool_station_detection_msgs
  ar_track_alvar
  # flexbotics_tool_exchanger_manager_msgs
)
find_package( Eigen3 REQUIRED )
else()
find_package( catkin REQUIRED COMPONENTS
  roscpp
  roslib
  cmake_modules
  eigen_conversions
  eigen_helper_functions
  # tool_exchanger_info_manager
  flexbotics_manipulator_manager_msgs
  flexbotics_manipulation_planner_msgs
  tool_station_detection_msgs
  ar_track_alvar
  # flexbotics_tool_exchanger_manager_msgs
)
find_package( Eigen REQUIRED )
endif()

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_executable(tool_station_detection_node src/tool_station_detection_node.cpp)
target_link_libraries(tool_station_detection_node ${catkin_LIBRARIES})

## Mark executables and/or libraries for installation
install(TARGETS tool_station_detection_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
#install(DIRECTORY include/${PROJECT_NAME}/
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#  FILES_MATCHING PATTERN "*.h"
#  PATTERN ".svn" EXCLUDE
#)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE
)

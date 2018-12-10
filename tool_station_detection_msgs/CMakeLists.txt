cmake_minimum_required(VERSION 2.8.3)
project(tool_station_detection_msgs)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  trajectory_msgs
  message_generation
)

add_service_files(DIRECTORY srv
  FILES
  ToolStationDetection.srv
  # MoveJoint.srv
  # MoveTcp.srv
  # MoveLin.srv
  # LoadGripperIoInfo.srv
  # GetDigitalInput.srv
  # GetDigitalOutput.srv
  # SetDigitalOutput.srv
  # ActivateGripper.srv
  # DeactivateGripper.srv
)

generate_messages(DEPENDENCIES
  geometry_msgs
  trajectory_msgs
)

catkin_package(CATKIN_DEPENDS message_runtime)

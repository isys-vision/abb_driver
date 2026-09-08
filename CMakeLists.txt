cmake_minimum_required(VERSION 3.0.2)
project(abb_driver)

set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS industrial_robot_client simple_message)

catkin_package(
  CATKIN_DEPENDS industrial_robot_client
)

include_directories(
  include
  include/abb_driver
  include/abb_driver/simple_message
  include/abb_driver/simple_message/messages
  include/abb_driver/simple_message/classes
  ${catkin_INCLUDE_DIRS}
)

set(MSG_SRC_FILES
  src/simple_message/classes/mikado_connection_info.cpp
  src/simple_message/classes/mikado_dynamic_joints_traj_pt.cpp
  src/simple_message/classes/mikado_dynamic_joints.cpp
  src/simple_message/messages/mikado_dynamic_joints_traj_pt_message.cpp
  src/simple_message/messages/mikado_dynamic_joints_message.cpp
  src/simple_message/messages/mikado_connection_info_message.cpp
)

add_library(abb_interfaces
    src/abb_joint_trajectory_interface.cpp
    src/abb_joint_trajectory_downloader.cpp
    src/abb_simple_comms_fault_handler.cpp
    src/abb_message_manager.cpp
    src/abb_robot_state_interface.cpp
)

add_library(abb_mikado_simple_message ${MSG_SRC_FILES})
target_link_libraries(abb_mikado_simple_message
  simple_message)

add_definitions(-DLINUXSOCKETS)

add_executable(${PROJECT_NAME}_robot_state
  src/abb_robot_state_node.cpp
  src/abb_utils.cpp
  src/dynamic_joints_relay_handler.cpp
  src/abb_robot_state_interface.cpp
  src/abb_message_manager.cpp
  src/abb_simple_comms_fault_handler.cpp
  )
target_link_libraries(${PROJECT_NAME}_robot_state
  abb_mikado_simple_message
  industrial_robot_client
  simple_message
  ${catkin_LIBRARIES})
set_target_properties(
  ${PROJECT_NAME}_robot_state
  PROPERTIES OUTPUT_NAME robot_state PREFIX "")
target_compile_definitions(
  ${PROJECT_NAME}_robot_state
  PRIVATE
  ${industrial_robot_client_DEFINITIONS})

add_executable(${PROJECT_NAME}_motion_download_interface
  src/abb_joint_downloader_node.cpp
  src/abb_utils.cpp)
target_link_libraries(${PROJECT_NAME}_motion_download_interface
  abb_interfaces
  abb_mikado_simple_message
  industrial_robot_client
  simple_message
  ${catkin_LIBRARIES})
set_target_properties(${PROJECT_NAME}_motion_download_interface
  PROPERTIES OUTPUT_NAME motion_download_interface PREFIX "")
target_compile_definitions(
  ${PROJECT_NAME}_motion_download_interface
  PRIVATE
  ${industrial_robot_client_DEFINITIONS})

install(
  TARGETS
    ${PROJECT_NAME}_robot_state
    ${PROJECT_NAME}_motion_download_interface
  DESTINATION
    ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  TARGETS
    abb_mikado_simple_message
    abb_interfaces
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

foreach(dir launch rapid)
  install(
    DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach()

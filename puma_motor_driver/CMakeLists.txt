cmake_minimum_required(VERSION 2.8.3)
project(puma_motor_driver)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  serial
  puma_motor_msgs
)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   sensor_msgs
# )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES puma_motor_driver
  CATKIN_DEPENDS roscpp sensor_msgs serial puma_motor_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(puma_motor_driver
  src/driver.cpp
  src/serial_gateway.cpp
  src/socketcan_gateway.cpp
  src/multi_driver_node.cpp
  src/diagnostic_updater.cpp
)

target_link_libraries(puma_motor_driver
  ${catkin_LIBRARIES}
)
add_dependencies(puma_motor_driver puma_motor_msgs_gencpp)

add_executable(puma_multi_controller_node src/multi_controller_node.cpp)

target_link_libraries(puma_multi_controller_node
  puma_motor_driver
  ${catkin_LIBRARIES}
)



# Mark executables and/or libraries for installation
install(TARGETS puma_motor_driver puma_multi_controller_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(fixed_point_test test/fixed_point_test.cpp)
endif()

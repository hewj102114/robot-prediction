cmake_minimum_required(VERSION 2.8.3)
project(robo_control)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
geometry_msgs
roscpp
rospy
std_msgs
message_generation
nav_msgs
sensor_msgs
tf
move_base_msgs
robo_vision
)

add_message_files(
  FILES
  GameInfo.msg
  TeamInfo.msg
) 

generate_messages(
  DEPENDENCIES
  geometry_msgs
std_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES serial
CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs  nav_msgs tf sensor_msgs  move_base_msgs message_runtime robo_vision
#  DEPENDS system_lib
)

include_directories(
include
${catkin_INCLUDE_DIRS} 
)

 install(PROGRAMS
   scripts/navigation.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )


add_executable(nav_test src/navigation_test.cpp src/robo_control.cpp src/serial.cpp)
target_link_libraries(nav_test
  ${catkin_LIBRARIES}
)

add_executable(odom_broadcaster src/odom_broadcaster.cpp)

target_link_libraries(odom_broadcaster
  ${catkin_LIBRARIES} 
)

add_executable(enemy_listen src/enemy_listen.cpp)

target_link_libraries(enemy_listen
  ${catkin_LIBRARIES} 
)

add_executable(enemy_pnp_listen src/enemy_listen_pnp.cpp)

target_link_libraries(enemy_pnp_listen
  ${catkin_LIBRARIES} 
)


add_executable(control_node src/MainControl.cpp src/robo_control.cpp src/serial.cpp)
add_dependencies(control_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(control_node
  ${catkin_LIBRARIES}
)

add_executable(NCEC src/MainControl.cpp src/robo_control.cpp src/serial.cpp)
add_dependencies(NCEC ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(NCEC
  ${catkin_LIBRARIES}
)
cmake_minimum_required(VERSION 2.8.3)
project(robo_navigation)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
  tf2
  nav_msgs
  geometry_msgs
  sensor_msgs
  std_msgs
  visualization_msgs
  message_generation
  robo_perception
)

add_definitions(-std=c++11) 

################################################
## Declare ROS messages, services and actions ##
################################################

find_package(OpenCV 3 REQUIRED)

# # msg 文件
# # Generate messages in the 'msg' folder
# add_message_files(
#    FILES
#    Message1.msg
# )
# # msg 依赖
# generate_messages(
#     DEPENDENCIES
#     std_msgs
# )
# # Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
# )
# # Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
# )


###########
## Build ##
###########

catkin_package(
    # INCLUDE_DIRS include   #声明给其它package的include路径
    # LIBRARIES ${PRIJECT_NAME}    #声明给其它package的库
    CATKIN_DEPENDS
    roscpp
    rospy
    tf
    tf2
    nav_msgs
    geometry_msgs
    sensor_msgs
    std_msgs
    visualization_msgs
    message_runtime
    robo_perception
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

list(APPEND SOURCEFILES
  src/global_planner.cpp
)

add_executable(${PROJECT_NAME}
    src/robo_navigation_node.cpp
    ${SOURCEFILES}
)


add_executable(lidar_nav 
     src/robo_navigation_1.cpp
    ${SOURCEFILES}
)

target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
   ${catkin_LIBRARIES}
 )

 target_link_libraries(lidar_nav
${OpenCV_LIBS}
   ${catkin_LIBRARIES}
 )




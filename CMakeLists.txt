cmake_minimum_required(VERSION 2.8.3)
project(velocity_qp)


set(CMAKE_BUILD_TYPE Release)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  controller_interface
  geometry_msgs
  realtime_tools
  dynamic_reconfigure
  franka_hw
  franka_control
  franka_gripper
  hardware_interface
  pluginlib
  roscpp
  rospy  trac_ik_lib
  qpOASES
  kdl_conversions
  eigen_conversions
  std_msgs
)

find_package(Franka 0.5.0 REQUIRED)
find_package(gazebo REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS date_time)
find_package(orocos_kdl REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

generate_dynamic_reconfigure_options(
  cfg/param.cfg
)

add_message_files(
  FILES
  PandaRunMsg.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

add_library(${PROJECT_NAME} 
  src/controller/controller.cpp
  src/robot/panda_control.cpp
  src/robot/panda_simulation.cpp
)

add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
)

target_link_libraries(${PROJECT_NAME}
  ${Franka_LIBRARIES}
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  ${Boost_LIBRARIES}  
  ${orocos_kdl_LIBRARIES}
)


target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
  include
  ${GAZEBO_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${Franka_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

target_include_directories(${PROJECT_NAME} PUBLIC
  include/
  src
  ${GAZEBO_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  LIBRARIES ${PROJECT_NAME} 
  INCLUDE_DIRS include 
  CATKIN_DEPENDS
    controller_interface
    franka_hw
    franka_control
    franka_gripper
    hardware_interface
    message_runtime
    pluginlib
    roscpp
  DEPENDS 
    Franka
    roscpp
    gazebo_ros
)



## Installation

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES velocity_qp_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES scripts/killgazebo
    PERMISSIONS OWNER_READ GROUP_READ WORLD_READ WORLD_EXECUTE GROUP_EXECUTE OWNER_EXECUTE
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


## Tools
include(${CMAKE_CURRENT_LIST_DIR}/../cmake/ClangTools.cmake OPTIONAL
  RESULT_VARIABLE CLANG_TOOLS
)

if(CLANG_TOOLS)
  file(GLOB_RECURSE SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
  file(GLOB_RECURSE HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/include/*.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.h
  )
  add_format_target(velocity_qp FILES ${SOURCES} ${HEADERS})
  add_tidy_target(velocity_qp
    FILES ${SOURCES}
    DEPENDS velocity_qp
  ) 
  
endif()


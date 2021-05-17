cmake_minimum_required(VERSION 3.5)
project(velocity_qp VERSION 0.0.1 LANGUAGES CXX )


set(CMAKE_BUILD_TYPE Release)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Franka 0.5.0 REQUIRED)
find_package(gazebo REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS date_time)
find_package(pinocchio REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  panda_traj
  qp_solver
  controller_interface
  geometry_msgs
  realtime_tools
  franka_hw
  franka_control
  hardware_interface
  roscpp
  rospy  
  message_generation
  eigen_conversions
  std_msgs
)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})


add_message_files(
  FILES
  PandaRunMsg.msg
)

add_service_files(
  FILES
  UI.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)

add_library(${PROJECT_NAME} 
  src/controller/controller.cpp
  src/controller/load_trajectory.py
  src/robot/panda_control.cpp
  src/robot/panda_simulation.cpp
)

add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}
  pinocchio::pinocchio
  ${catkin_LIBRARIES}
  ${Franka_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  ${Boost_LIBRARIES}  
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


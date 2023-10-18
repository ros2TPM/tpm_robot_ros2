cmake_minimum_required(VERSION 3.8)
project(tpm_core_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ignore -Wunused-parameter
add_compile_options(-Wno-unused-parameter)

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
    ament_cmake
    rclcpp
    rclcpp_action
    tpm_msgs
    sensor_msgs
    control_msgs
)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

set(INCLUDE_DIR
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/def>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/ros_interface>
  $<INSTALL_INTERFACE:include>
  ${CMAKE_CURRENT_SOURCE_DIR}/../ExtraLib/RobC/include
  ${CMAKE_CURRENT_SOURCE_DIR}/../ExtraLib/RPX-L132/include
)

###########
## Build ##
###########
add_executable(${PROJECT_NAME}
  src/tpm_core_node.cpp
  src/global_instance.cpp
  src/global_config.cpp
  src/myAxis.cpp
  src/myRobot.cpp
  src/hwLib.cpp
  src/hwLib_real.cpp
  src/hwLib_sim.cpp
  src/ros_interface/manager_param.cpp
  src/ros_interface/manager_topic.cpp
  src/ros_interface/manager_service.cpp
  src/ros_interface/manager_action.cpp
  src/robot_kinematics/robotKinematics.cpp
  src/FJStrategy/FJStrategy_mclcPVT.cpp
  src/FJStrategy/FJStrategy_robcAxisPTP.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${THIS_PACKAGE_INCLUDE_DEPENDS})

target_include_directories(
  ${PROJECT_NAME}
  PUBLIC
  ${INCLUDE_DIR}
)

target_link_libraries(${PROJECT_NAME} RobC RPiMNet_64)


#############
## Install ##
#############

install(
  TARGETS tpm_core_node
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch 
  DESTINATION share/${PROJECT_NAME}
  )
install(
  DIRECTORY config 
  DESTINATION share/${PROJECT_NAME}
  )  

ament_package()

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
  $<INSTALL_INTERFACE:include>
  /home/tpm/Desktop/RPX-L132D/include
  /home/tpm/Desktop/RobCLib/RobC
  /home/tpm/Desktop/RobCLib/MCLC
  /home/tpm/Desktop/RobCLib/MCLC/Core
  /home/tpm/Desktop/ws_tpm_light/src/tpm_controller/include
)

###########
## Build ##
###########

#add '--cmake-args -DUSE_REAL_MNET=ON' to build with RPiMnet
option(USE_REAL_MNET "Build with RPiMnet library" OFF)
if(USE_REAL_MNET)
  message("******* Build real lib ********")
  # define REAL_MNET, in the tpm_core_node.cpp will use MNetMaster_Real
  add_definitions(-DREAL_MNET)
  add_definitions(-DROB_REAL)
  
  set(LIB_SRC
    src/hwLib_real.cpp
  )
  
  set(LIB_DEPENDENCIES
    RPiMNet_64
  )
else()
  message("******* Build sim lib ********")
  
  set(LIB_SRC
    src/hwLib_sim.cpp
  )
  
  set(LIB_DEPENDENCIES
    RobC
    MCLC
  )
endif()
# because cmake cache the value, unset USE_REAL_MNET
# so it will always be default if user doesn't set USE_REAL_MNET
unset(USE_REAL_MNET CACHE)


add_executable(${PROJECT_NAME}
  src/tpm_core_node.cpp
  src/global_instance.cpp
  src/global_config.cpp
  src/myAxis.cpp
  src/myRobot.cpp
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
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/def>
  ${INCLUDE_DIR}
)


add_library(core_lib ${LIB_SRC})
target_include_directories(core_lib PUBLIC 
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/def>
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/ros_interface>
${INCLUDE_DIR}
)
target_link_libraries(core_lib ${LIB_DEPENDENCIES})
target_link_libraries(${PROJECT_NAME} core_lib)
ament_target_dependencies(core_lib rclcpp)


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

cmake_minimum_required(VERSION 3.8)
project(robot_command)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(simple_thread 
  src/simple_thread.cpp
  src/MyNode.cpp
  src/PointsManager.cpp
  src/SolutionChooser.cpp
)
ament_target_dependencies(
  simple_thread 
  "moveit_ros_planning_interface"
  "rclcpp" 
  )
  
target_include_directories(simple_thread PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(simple_thread PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS simple_thread
  DESTINATION lib/${PROJECT_NAME})


ament_package()

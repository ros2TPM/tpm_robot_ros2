cmake_minimum_required(VERSION 3.22)
project(moveit_ar3)

find_package(ament_cmake REQUIRED)

ament_package()

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})



#================= moveit_loop ================
find_package(moveit_ros_planning_interface REQUIRED)
  add_executable(moveit_loop sample_client/moveit_loop.cpp)
  target_include_directories(moveit_loop PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  target_compile_features(moveit_loop PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
  
  ament_target_dependencies(moveit_loop
    rclcpp
    moveit_ros_planning_interface
  )  
  install(TARGETS moveit_loop
    DESTINATION lib/${PROJECT_NAME})

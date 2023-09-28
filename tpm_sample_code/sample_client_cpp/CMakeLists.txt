cmake_minimum_required(VERSION 3.8)
project(sample_client_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tpm_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(move_ptp src/move_ptp.cpp)
target_include_directories(move_ptp PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(move_ptp PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(move_ptp
  rclcpp
  tpm_msgs
)

install(TARGETS move_ptp
  DESTINATION lib/${PROJECT_NAME})

add_executable(move_lin src/move_lin.cpp)
target_include_directories(move_lin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(move_lin PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(move_lin
  rclcpp
  tpm_msgs
)

install(TARGETS move_lin
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

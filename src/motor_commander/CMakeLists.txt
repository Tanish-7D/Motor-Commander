cmake_minimum_required(VERSION 3.8)
project(motor_commander)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(motor_control REQUIRED)
find_package(Threads REQUIRED)

add_executable(stepper_pwm src/stepper_pwm.cpp)
add_executable(dualmotortest src/dualmotortest.cpp)
add_executable(dualmotortest src/dualmo.cpp)
add_executable(test_rev src/test_rev.cpp)

ament_target_dependencies(stepper_pwm
  rclcpp
  motor_control
)

ament_target_dependencies(dualmotortest
  rclcpp
  motor_control
)

ament_target_dependencies(bldc_pwm
  rclcpp
  motor_control
)

target_link_libraries(stepper_pwm
  pigpiod_if2
  pigpio
  Threads::Threads
)

target_link_libraries(dualmotortest
  pigpiod_if2
  pigpio
  Threads::Threads
)

target_include_directories(stepper_pwm PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_include_directories(dualmotortest PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

install(TARGETS stepper_pwm dualmotortest
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

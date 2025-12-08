cmake_minimum_required(VERSION 3.8)
project(motor_commander)

# Compiler warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(motorcontrol_messages REQUIRED)
find_package(Threads REQUIRED)

# Create executable
add_executable(stepper_pwm
  src/stepper_pwm.cpp
)

# Add dependencies
ament_target_dependencies(stepper_pwm
  rclcpp
  motorcontrol_messages
)

# Link external libraries
target_link_libraries(stepper_pwm
  pigpiod_if2
  pigpio
  Threads::Threads
)

# Include directories
target_include_directories(stepper_pwm PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Install the executable
install(TARGETS
  stepper_pwm
  DESTINATION lib/${PROJECT_NAME}
)

# Testing (optional)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Must be last
ament_package()

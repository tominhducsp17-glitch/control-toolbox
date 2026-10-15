cmake_minimum_required(VERSION 3.16)
project(openarm_controller LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ct_nlc REQUIRED)     # controllers (ct::core::Controller) and the reference filter
find_package(ct_models REQUIRED)  # OpenArm model (generated code)
# openarm_can 1.4.0 (CAN-FD, Damiao MIT); without it only the simulated bus is built.
find_package(OpenArmCAN QUIET)

# Control loop of one arm (bus -> reference filter -> full CTC -> bus), independent of ROS.
add_library(control_loop STATIC src/arm_bus.cpp src/control_loop.cpp src/sim_bus.cpp)
target_include_directories(control_loop PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(control_loop PUBLIC ct_nlc OpenArmModel)
if(OpenArmCAN_FOUND)
  target_sources(control_loop PRIVATE src/openarm_can_bus.cpp)
  target_link_libraries(control_loop PUBLIC OpenArmCAN::openarm_can)
  target_compile_definitions(control_loop PUBLIC OPENARM_CONTROLLER_HAVE_CAN=1)
else()
  message(WARNING "openarm_can not found: controller_node supports only bus 'sim'")
endif()

# Node "controller": the 1 kHz loop with the motor bus inside, one thread.
add_executable(controller_node src/controller_node.cpp)
target_link_libraries(controller_node control_loop)
ament_target_dependencies(controller_node rclcpp sensor_msgs)
install(TARGETS controller_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_control_loop test/test_control_loop.cpp)
  target_link_libraries(test_control_loop control_loop)
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_trajectory_controller.py TIMEOUT 180)
endif()

ament_package()

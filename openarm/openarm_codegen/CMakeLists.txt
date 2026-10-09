cmake_minimum_required(VERSION 3.16)
project(openarm_codegen LANGUAGES CXX)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(ct_core REQUIRED)
find_package(pinocchio REQUIRED)

# Offline tool only: Pinocchio (CppADCodeGen scalar) + control-toolbox code generation.
# The runtime library (openarm_control) uses the generated sources and does not link Pinocchio.
add_executable(generate_model src/generate_model.cpp src/model_ad.cpp)
target_link_libraries(generate_model ct_core pinocchio::pinocchio)
install(TARGETS generate_model DESTINATION lib/${PROJECT_NAME})

ament_package()

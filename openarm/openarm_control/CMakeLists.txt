cmake_minimum_required(VERSION 3.16)
project(openarm_control LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(ct_core REQUIRED)   # control-toolbox core (interfaces, generated-code base class)
find_package(Eigen3 REQUIRED)

# Library only (no ROS node). The model is generated code (generated/, written by openarm_codegen
# from the official URDF); Pinocchio is not a dependency at run time.
add_library(${PROJECT_NAME} SHARED
  src/openarm_model.cpp
  src/controller.cpp
  generated/OpenArmRightModel.cpp
  generated/OpenArmLeftModel.cpp)
target_include_directories(${PROJECT_NAME}
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/generated)
target_link_libraries(${PROJECT_NAME} PUBLIC ct_core Eigen3::Eigen)
# Generated code is long straight-line arithmetic; silence its unused-variable style warnings only.
set_source_files_properties(generated/OpenArmRightModel.cpp generated/OpenArmLeftModel.cpp
  PROPERTIES COMPILE_OPTIONS "-Wno-unused-parameter;-Wno-unused-variable")

install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY urdf DESTINATION share/${PROJECT_NAME})  # official OpenArm v1 bimanual URDF (codegen input)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(ct_core Eigen3)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_openarm_control test/test_openarm_control.cpp)
  target_link_libraries(test_openarm_control ${PROJECT_NAME})
  target_compile_definitions(test_openarm_control PRIVATE
    GOLDEN_CSV="${CMAKE_CURRENT_SOURCE_DIR}/test/data/model_golden.csv")
endif()

ament_package()

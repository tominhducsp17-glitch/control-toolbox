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
find_package(ct_core REQUIRED)
find_package(ct_models REQUIRED)  # OpenArm model (generated code)
find_package(Eigen3 REQUIRED)

# Library only (no ROS node). Transitional: controllers move to ct_nlc; the model is ct_models OpenArm.
add_library(${PROJECT_NAME} SHARED src/controller.cpp)
target_include_directories(${PROJECT_NAME}
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME} PUBLIC ct_core OpenArmModel Eigen3::Eigen)

install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(ct_core ct_models Eigen3)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_openarm_control test/test_openarm_control.cpp)
  target_link_libraries(test_openarm_control ${PROJECT_NAME})
  target_compile_definitions(test_openarm_control PRIVATE
    GOLDEN_CSV="${CMAKE_CURRENT_SOURCE_DIR}/../../ct_models/test/OpenArm/data/openarm_reference_values.csv")
endif()

ament_package()

cmake_minimum_required (VERSION 3.5)

if(NOT TARGET clang-format)
	include(${CMAKE_CURRENT_SOURCE_DIR}/../ct/cmake/compilerSettings.cmake)
	include(${CMAKE_CURRENT_SOURCE_DIR}/../ct/cmake/explicitTemplateHelpers.cmake)
	include(${CMAKE_CURRENT_SOURCE_DIR}/../ct/cmake/clang-cxx-dev-tools.cmake)
endif()
include(${CMAKE_CURRENT_SOURCE_DIR}/../ct/cmake/ct-cmake-helpers.cmake)

project(ct_nlc VERSION 3.0.2 LANGUAGES CXX)


set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wfatal-errors -std=c++14 -Wall -Wno-unknown-pragmas")
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

if(NOT TARGET ${ct_core})
	find_package(ct_core REQUIRED)
endif()

# extract interface compile definitions from ct_core as options
importInterfaceCompileDefinitionsAsOptions(ct_core)

## models for the examples and tests (OpenArm); the controllers themselves only need ct_core
find_package(ct_models QUIET)


###################
# BUILD LIBRARIES #
###################

## define the directories to be included in all ct_nlc targets
set(ct_nlc_target_include_dirs
    ${ct_core_INCLUDE_DIRS}
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

## create ct_nlc library (header only, the controllers are templates on the model)
add_library(ct_nlc INTERFACE)
target_include_directories(ct_nlc INTERFACE ${ct_nlc_target_include_dirs})
target_link_libraries(ct_nlc INTERFACE ct_core)

set(ct_nlc ct_nlc CACHE INTERNAL "ct_nlc library target.")


############
# EXAMPLES #
############

if(BUILD_EXAMPLES)
    if(ct_models_FOUND)
        add_subdirectory(examples)
    else()
        message(STATUS "ct_nlc examples not built (need ct_models)")
    endif()
endif()


###########
# TESTING #
###########

if(BUILD_TESTS)
    if(ct_models_FOUND)
        enable_testing()
        add_subdirectory(test)
    else()
        message(STATUS "ct_nlc tests not built (need ct_models)")
    endif()
endif()


#################
# INSTALLATION  #
#################

# for correct libraries locations across platforms
include(GNUInstallDirs)

## copy the header files
install(DIRECTORY include/ct/nlc DESTINATION include/ct)

## copy the cmake files required for find_package()
install(FILES "cmake/ct_nlcConfig.cmake" DESTINATION "share/ct_nlc/cmake")

## install library and targets
install(
    TARGETS ct_nlc
    EXPORT ct_nlc_export
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_LIBDIR}
    )

## create the ct_nlc.cmake file which holds target includes and dependencies
install (EXPORT ct_nlc_export DESTINATION share/ct_nlc/cmake)

## add uninstall target
if(NOT TARGET uninstall)
    configure_file(
        "${CMAKE_CURRENT_SOURCE_DIR}/../ct/cmake/cmake_uninstall.cmake.in"
        "${CMAKE_CURRENT_BINARY_DIR}/../ct/cmake/cmake_uninstall.cmake"
        IMMEDIATE @ONLY)

    add_custom_target(uninstall
        COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/../ct/cmake/cmake_uninstall.cmake)
endif()

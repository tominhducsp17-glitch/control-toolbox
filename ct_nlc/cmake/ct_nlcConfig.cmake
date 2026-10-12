# cmake-config file for the ct_nlc package

find_package(ct_core REQUIRED)

include(${CMAKE_CURRENT_LIST_DIR}/ct_nlc_export.cmake)

#define includes in legacy mode
get_target_property(ct_nlc_INCLUDE_DIRS ct_nlc INTERFACE_INCLUDE_DIRECTORIES)

# define libraries in legacy mode
get_property(ct_nlc_LIB TARGET ct_nlc PROPERTY INTERFACE_LINK_LIBRARIES)
list(APPEND ct_nlc_LIBRARIES ${ct_nlc_LIB})

# Required for catkin_simple to link against these libraries
set(ct_nlc_FOUND_CATKIN_PROJECT true)

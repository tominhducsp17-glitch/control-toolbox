# cmake-config file for the ct_optcon package

find_package(ct_core REQUIRED)

# ct_optcon links hpipm/blasfeo by target name; load those imported targets (and their include
# directories) for downstream packages too. Installed in /opt by ct/install_hpipm.sh.
list(APPEND CMAKE_PREFIX_PATH "/opt")
find_package(blasfeo QUIET)
find_package(hpipm QUIET)

include(${CMAKE_CURRENT_LIST_DIR}/ct_optcon_export.cmake)

#define includes in legacy mode
get_target_property(ct_optcon_INCLUDE_DIRS ct_optcon INTERFACE_INCLUDE_DIRECTORIES)

# define libraries in legacy mode
get_property(ct_optcon_LIB TARGET ct_optcon PROPERTY INTERFACE_LINK_LIBRARIES)
list(APPEND ct_optcon_LIBRARIES ${ct_optcon_LIB})

# Required for catkin_simple to link against these libraries
set(ct_optcon_FOUND_CATKIN_PROJECT true)


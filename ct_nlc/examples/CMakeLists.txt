include(GNUInstallDirs)

## OpenArm right arm in simulation with computed-torque control
add_executable(ex_OpenArmComputedTorque OpenArmComputedTorque.cpp)
target_link_libraries(ex_OpenArmComputedTorque ct_nlc OpenArmModel)
list(APPEND nlc_ex_TARGETS ex_OpenArmComputedTorque)

install(TARGETS ${nlc_ex_TARGETS} RUNTIME DESTINATION ${CMAKE_INSTALL_LIBDIR}/ct_nlc)

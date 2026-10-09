set(CT_OPTCON_EXAMPLE_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/exampleDir.h.in ${CMAKE_CURRENT_SOURCE_DIR}/exampleDir.h)

# openarm-ctc branch: examples re-enabled (they were commented out in v3.0.2), each behind the
# dependency it needs, so `colcon build` with BUILD_EXAMPLES=ON gives runnable optimal-control demos.
foreach(ex ConstraintOutput:ConstraintExampleOutput NLOC:NLOC NLOC_MPC:NLOC_MPC
           KalmanFiltering:KalmanFiltering KalmanDisturbanceFiltering:KalmanDisturbanceFiltering)
    string(REPLACE ":" ";" pair ${ex})
    list(GET pair 0 name)
    list(GET pair 1 src)
    add_executable(ex_${name} ${src}.cpp)
    target_link_libraries(ex_${name} ct_optcon)
    list(APPEND optcon_ex_TARGETS ex_${name})
endforeach()

if(BUILD_WITH_IPOPT_SUPPORT)  # direct multiple shooting and NLP examples use the IPOPT backend
    add_executable(ex_DMS DMS.cpp)
    target_link_libraries(ex_DMS ct_optcon)
    list(APPEND optcon_ex_TARGETS ex_DMS)
    add_executable(ex_Nlp_2D nlp/Nlp2D.cpp)
    target_link_libraries(ex_Nlp_2D ct_optcon)
    list(APPEND optcon_ex_TARGETS ex_Nlp_2D)
    add_executable(ex_Nlp_3D nlp/Nlp3D.cpp)
    target_link_libraries(ex_Nlp_3D ct_optcon)
    list(APPEND optcon_ex_TARGETS ex_Nlp_3D)
endif()

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../../ct_core/cmake)
find_package(CppADCG QUIET)
if(CPPADCG_FOUND)  # LQR linearizes the system with auto-diff code generation
    add_executable(ex_LQR LQR.cpp)
    target_link_libraries(ex_LQR ct_optcon)
    list(APPEND optcon_ex_TARGETS ex_LQR)
endif()

if(HPIPM)
    add_executable(ex_NLOC_boxConstrained NLOC_boxConstrained.cpp)
    target_link_libraries(ex_NLOC_boxConstrained ct_optcon)
    list(APPEND optcon_ex_TARGETS ex_NLOC_boxConstrained)
    
    if(CPPADCG)
      add_executable(ex_NLOC_generalConstrained NLOC_generalConstrained.cpp)
      target_link_libraries(ex_NLOC_generalConstrained ct_optcon)
      list(APPEND optcon_ex_TARGETS ex_NLOC_generalConstrained)
    
      # add_executable(switched_continuous_optcon_example switched_systems_optcon/switched_continuous_optcon.cpp)
      # target_link_libraries(switched_continuous_optcon_example ct_optcon)
      # list(APPEND optcon_ex_TARGETS switched_continuous_optcon_example)
    endif()
endif(HPIPM)


## install examples
include(GNUInstallDirs)
install(
    TARGETS ${optcon_ex_TARGETS}
    RUNTIME DESTINATION ${CMAKE_INSTALL_LIBDIR}/ct_optcon
    )
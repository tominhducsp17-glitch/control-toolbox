macro(package_add_test TESTNAME)
    add_executable(${TESTNAME} ${ARGN})
    add_test(NAME ${TESTNAME} COMMAND ${TESTNAME})
    set_target_properties(${TESTNAME} PROPERTIES FOLDER test)
    list(APPEND UNIT_TEST_TARGETS ${TESTNAME})
endmacro()

if(BUILD_TESTS)

    package_add_test(ComputedTorqueControllerTest ComputedTorqueControllerTest.cpp)
    target_compile_definitions(ComputedTorqueControllerTest PRIVATE
      OPENARM_REFERENCE_CSV="${CMAKE_CURRENT_SOURCE_DIR}/../../ct_models/test/OpenArm/data/openarm_reference_values.csv")
    target_link_libraries(ComputedTorqueControllerTest gtest gtest_main ct_nlc OpenArmModel)

    package_add_test(FullComputedTorqueControllerTest FullComputedTorqueControllerTest.cpp)
    target_link_libraries(FullComputedTorqueControllerTest gtest gtest_main ct_nlc OpenArmModel)


    # Run all unit tests post-build.
    add_custom_target(run_tests DEPENDS ${UNIT_TEST_TARGETS})
    add_custom_command(TARGET run_tests
        COMMENT "Running tests"
        POST_BUILD COMMAND ${PROJECT_SOURCE_DIR}/../ct/run_tests.py ${CMAKE_CURRENT_BINARY_DIR} ${UNIT_TEST_TARGETS}
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
        )

    ## install tests
    include(GNUInstallDirs)
    install(
        TARGETS ${UNIT_TEST_TARGETS}
        RUNTIME DESTINATION ${CMAKE_INSTALL_LIBDIR}/ct_nlc
        )
endif()

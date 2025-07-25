# Add test function
function(alicevision_add_test test_file)
    set(options "")
    set(singleValues NAME)
    set(multipleValues LINKS INCLUDE_DIRS)

    cmake_parse_arguments(TEST "${options}" "${singleValues}" "${multipleValues}" ${ARGN})

    if (NOT test_file)
        message(FATAL_ERROR "You must provide the test file in 'alicevision_add_test'")
    endif()

    if (NOT TEST_NAME)
        message(FATAL_ERROR "You must provide the NAME in 'alicevision_add_test'")
    endif()

    if (NOT ALICEVISION_BUILD_TESTS)
        return()
    endif()

    set(TEST_EXECUTABLE_NAME "AVTest_${TEST_NAME}")

    add_executable(${TEST_EXECUTABLE_NAME} ${test_file})

    target_link_libraries(${TEST_EXECUTABLE_NAME}
            PUBLIC ${TEST_LINKS}
            ${ALICEVISION_LIBRARY_DEPENDENCIES}
            Boost::unit_test_framework
            Boost::log
    )

    target_include_directories(${TEST_EXECUTABLE_NAME}
            PUBLIC ${TEST_INCLUDE_DIRS}
    )

    set_property(TARGET ${TEST_EXECUTABLE_NAME}
            PROPERTY FOLDER Test
    )

    if ((MSVC) AND (MSVC_VERSION GREATER_EQUAL 1914))
        target_compile_options(${TEST_EXECUTABLE_NAME} PUBLIC "/Zc:__cplusplus")
    endif()

    add_test(NAME test_${TEST_EXECUTABLE_NAME}
            WORKING_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}
            COMMAND $<TARGET_FILE:${TEST_EXECUTABLE_NAME}> --catch_system_error=yes --log_level=all
    )

    if (ALICEVISION_BUILD_COVERAGE AND CMAKE_COMPILER_IS_GNUCXX)
        append_coverage_compiler_flags_to_target(${TEST_EXECUTABLE_NAME})
    endif()

    if (UNIX)
        # setup LD_LIBRARY_PATH for running tests
        get_property(TEST_LINK_DIRS TARGET ${TEST_EXECUTABLE_NAME} PROPERTY LINK_DIRECTORIES)
    endif()

    if (WIN32)
        set_property(TEST test_${TEST_EXECUTABLE_NAME} PROPERTY ENVIRONMENT "ALICEVISION_ROOT=${CMAKE_INSTALL_PREFIX}")
    endif()
endfunction()
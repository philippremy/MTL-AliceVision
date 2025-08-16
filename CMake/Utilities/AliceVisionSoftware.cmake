# Add software function
function(alicevision_add_software software_name)
    set(options "")
    set(singleValues FOLDER)
    set(multipleValues SOURCE LINKS INCLUDE_DIRS)

    cmake_parse_arguments(SOFTWARE "${options}" "${singleValues}" "${multipleValues}" ${ARGN})

    if (NOT software_name)
        message(FATAL_ERROR "You must provide the software name in 'alicevision_add_software'")
    endif()

    if (NOT SOFTWARE_SOURCE)
        message(FATAL_ERROR "You must provide the software SOURCE in 'alicevision_add_software'")
    endif()

    if (NOT SOFTWARE_FOLDER)
        message(FATAL_ERROR "You must provide the software FOLDER in 'alicevision_add_software'")
    endif()

    list(GET SOFTWARE_SOURCE 0 SOFTWARE_MAIN_CPP)
    file(STRINGS ${SOFTWARE_MAIN_CPP} _ALICEVISION_SOFTWARE_CONTENTS REGEX "#define ALICEVISION_SOFTWARE_VERSION_")

    foreach (v MAJOR MINOR)
        if ("${_ALICEVISION_SOFTWARE_CONTENTS}" MATCHES "#define ALICEVISION_SOFTWARE_VERSION_${v} ([0-9]+)")
            set(ALICEVISION_SOFTWARE_VERSION_${v} "${CMAKE_MATCH_1}")
        else()
            message(FATAL_ERROR "Failed to retrieve the AliceVision software version the source code. Missing ALICEVISION_SOFTWARE_VERSION_${v}.")
        endif()
    endforeach()

    # Generate Windows versioning information
    if (MSVC)
        set(ALICEVISION_INSTALL_VERSION_MAJOR ${ALICEVISION_SOFTWARE_VERSION_MAJOR})
        set(ALICEVISION_INSTALL_VERSION_MINOR ${ALICEVISION_SOFTWARE_VERSION_MINOR})
        set(ALICEVISION_INSTALL_VERSION_REVISION 0)
        set(ALICEVISION_INSTALL_NAME ${software_name})
        set(ALICEVISION_INSTALL_LIBRARY 0) # software
        configure_file(
            "${CMAKE_SOURCE_DIR}/src/cmake/version.rc.in"
            "${CMAKE_CURRENT_BINARY_DIR}/${software_name}_version.rc"
            @ONLY
        )
        list(APPEND SOFTWARE_SOURCE "${CMAKE_CURRENT_BINARY_DIR}/${software_name}_version.rc")
    endif()

    add_executable(${software_name}_exe ${SOFTWARE_SOURCE})
    set_target_properties(${software_name}_exe PROPERTIES
        OUTPUT_NAME ${software_name}
    )

    if (ALICEVISION_BUILD_COVERAGE AND CMAKE_COMPILER_IS_GNUCXX)
        append_coverage_compiler_flags_to_target(${software_name}_exe)
    endif()

    target_link_libraries(${software_name}_exe
        PUBLIC ${SOFTWARE_LINKS}
    )

    target_include_directories(${software_name}_exe
        PUBLIC ${SOFTWARE_INCLUDE_DIRS}
    )

    if ((MSVC) AND (MSVC_VERSION GREATER_EQUAL 1914))
        target_compile_options(${software_name}_exe PUBLIC "/Zc:__cplusplus")
    endif()

    set_property(TARGET ${software_name}_exe
        PROPERTY FOLDER ${SOFTWARE_FOLDER}
    )

    set_target_properties(${software_name}_exe
        PROPERTIES SOVERSION ${ALICEVISION_SOFTWARE_VERSION_MAJOR}
        VERSION "${ALICEVISION_SOFTWARE_VERSION_MAJOR}_${ALICEVISION_SOFTWARE_VERSION_MINOR}"
    )

    # Append to library list
    set(ALICEVISION_BINARIES "${ALICEVISION_BINARIES};${software_name}_exe" CACHE INTERNAL "AliceVision Binaries")

    install(TARGETS ${software_name}_exe
        RUNTIME
            DESTINATION ${CMAKE_INSTALL_BINDIR}
    )
endfunction()

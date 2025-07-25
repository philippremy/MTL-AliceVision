# Add interface function
function(alicevision_add_interface interface_name)
    set(options "")
    set(singleValues NAME)
    set(multipleValues SOURCES HEADERS LINKS)

    cmake_parse_arguments(LIBRARY "${options}" "${singleValues}" "${multipleValues}" ${ARGN})

    if (NOT interface_name)
        message(FATAL_ERROR "You must provide the interface name in 'alicevision_add_interface'")
    endif()

    if (NOT LIBRARY_SOURCES AND NOT LIBRARY_HEADERS)
        message(FATAL_ERROR "You must provide the interface SOURCES or HEADERS in 'alicevision_add_interface'")
    endif()

    add_library(${interface_name} INTERFACE)

    target_link_libraries(${interface_name}
            INTERFACE ${LIBRARY_LINKS}
    )

    install(TARGETS ${interface_name}
            EXPORT aliceVision-targets
    )

    # Transform Headers
    set(TRANSFORMED_HEADERS)
    foreach(HEADER IN LISTS LIBRARY_HEADERS)
        list(APPEND TRANSFORMED_HEADERS "${ALICEVISION_INCLUDE_DIR}/${interface_name}/${HEADER}")
    endforeach()

    # Add Include Directories
    target_include_directories(${interface_name}
            INTERFACE $<BUILD_INTERFACE:${ALICEVISION_INCLUDE_DIR}>
            $<INSTALL_INTERFACE:include>
            ${LIBRARY_PUBLIC_INCLUDE_DIRS}
    )

    set(LIBRARY_NAME_INTERFACE "${interface_name}_interface")
    add_custom_target(${LIBRARY_NAME_INTERFACE} SOURCES ${LIBRARY_SOURCES} ${TRANSFORMED_HEADERS})

    set_property(TARGET ${LIBRARY_NAME_INTERFACE}
            PROPERTY FOLDER "AliceVision"
    )
endfunction()
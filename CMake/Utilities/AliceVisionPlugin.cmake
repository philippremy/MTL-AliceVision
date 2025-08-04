# Add library function
function(alicevision_add_plugin library_name)
    set(options)
    set(singleValues)
    set(multipleValues SOURCES HEADERS PUBLIC_LINKS PRIVATE_LINKS PUBLIC_INCLUDE_DIRS PRIVATE_INCLUDE_DIRS PUBLIC_DEFINITIONS PRIVATE_DEFINITIONS RESOURCES FRAMEWORKS)

    cmake_parse_arguments(LIBRARY "${options}" "${singleValues}" "${multipleValues}" ${ARGN})

    if (NOT library_name)
        message(FATAL_ERROR "[AliceVision] You must provide the plugin name in 'alicevision_add_plugin'")
    endif()

    if(NOT LIBRARY_SOURCES)
        message(FATAL_ERROR "[AliceVision] You must provide the plugin SOURCES in 'alicevision_add_plugin'")
    endif()

    # Generate Windows versioning information
    if (MSVC)
        set(ALICEVISION_INSTALL_VERSION_MAJOR ${ALICEVISION_VERSION_MAJOR})
        set(ALICEVISION_INSTALL_VERSION_MINOR ${ALICEVISION_VERSION_MINOR})
        set(ALICEVISION_INSTALL_VERSION_REVISION ${ALICEVISION_VERSION_REVISION})
        set(ALICEVISION_INSTALL_NAME ${library_name})
        set(ALICEVISION_INSTALL_LIBRARY 1)
        configure_file(
                "${CMAKE_SOURCE_DIR}/CMake/Resources/version.rc.in"
                "${CMAKE_CURRENT_BINARY_DIR}/${library_name}_version.rc"
                @ONLY
        )
        list(APPEND LIBRARY_SOURCES "${CMAKE_CURRENT_BINARY_DIR}/${library_name}_version.rc")
    endif()

    # Generate the actual header paths
    # Always include the generic AV headers
    set(_RESOLVED_HEADERS)
    foreach(HEADER IN LISTS LIBRARY_HEADERS)
        list(APPEND _RESOLVED_HEADERS "${ALICEVISION_PLUGIN_INCLUDE_DIR}/${library_name}/${HEADER}")
    endforeach()

    add_library(${library_name} ${LIBRARY_SOURCES} ${_RESOLVED_HEADERS} ${LIBRARY_RESOURCES})

    if (ALICEVISION_BUILD_COVERAGE AND CMAKE_COMPILER_IS_GNUCXX)
        append_coverage_compiler_flags_to_target(${library_name})
    endif()

    if (ALICEVISION_REMOVE_ABSOLUTE)
        foreach (item ${LIBRARY_PUBLIC_LINKS})
            get_filename_component(nameItem ${item} NAME)
            list(APPEND TRANSFORMED_LIBRARY_PUBLIC_LINKS ${nameItem})
        endforeach()
    else()
        set(TRANSFORMED_LIBRARY_PUBLIC_LINKS ${LIBRARY_PUBLIC_LINKS})
    endif()

    target_link_libraries(${library_name}
            PUBLIC ${TRANSFORMED_LIBRARY_PUBLIC_LINKS}
            PRIVATE ${LIBRARY_PRIVATE_LINKS}
    )

    target_include_directories(${library_name}
            PUBLIC $<BUILD_INTERFACE:${ALICEVISION_INCLUDE_DIR}> $<BUILD_INTERFACE:${ALICEVISION_PLUGIN_INCLUDE_DIR}>
            $<INSTALL_INTERFACE:include>
            ${LIBRARY_PUBLIC_INCLUDE_DIRS}
            PRIVATE ${LIBRARY_PRIVATE_INCLUDE_DIRS}
    )

    target_compile_definitions(${library_name}
            PUBLIC ${LIBRARY_PUBLIC_DEFINITIONS}
            PRIVATE ${LIBRARY_PRIVATE_DEFINITIONS}
    )

    set_property(TARGET ${library_name}
            PROPERTY FOLDER "AliceVisionPlugins"
    )

    set_target_properties(${library_name}
            PROPERTIES SOVERSION "${ALICEVISION_VERSION_MAJOR}.${ALICEVISION_VERSION_MINOR}"
            VERSION "${ALICEVISION_VERSION_MAJOR}.${ALICEVISION_VERSION_MINOR}.${ALICEVISION_VERSION_REVISION}"
    )

    if ((MSVC) AND (MSVC_VERSION GREATER_EQUAL 1914))
        target_compile_options(${library_name} PUBLIC "/Zc:__cplusplus")
    endif()

    # Resolve resources
    set(PLUGIN_RESOLVED_RESOURCES)
    foreach(RESOURCE IN LISTS LIBRARY_RESOURCES)
        set(SOURCE_PATH_ABSOLUTE)
        # Normalize Path
        if(NOT IS_ABSOLUTE "${RESOURCE}")
            cmake_path(SET RESOURCE_PATH "${RESOURCE}")
            cmake_path(NORMAL_PATH RESOURCE_PATH)
            cmake_path(ABSOLUTE_PATH RESOURCE_PATH BASE_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}" OUTPUT_VARIABLE SOURCE_PATH_ABSOLUTE)
        else()
            set(SOURCE_PATH_ABSOLUTE "${RESOURCE}")
        endif()
        # Append
        list(APPEND PLUGIN_RESOLVED_RESOURCES "${SOURCE_PATH_ABSOLUTE}")
    endforeach()

    # Append to library list
    set(ALICEVISION_PLUGINS "${ALICEVISION_PLUGINS};${library_name}" CACHE INTERNAL "AliceVision Plugins")
    define_property(TARGET PROPERTY PLUGIN_RESOURCES BRIEF_DOCS "The resources which should be placed next to the plugin")
    set_property(TARGET ${library_name} PROPERTY PLUGIN_RESOURCES "${PLUGIN_RESOLVED_RESOURCES}")

    install(TARGETS ${library_name}
            EXPORT aliceVision-targets
            ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}/Plugins/${library_name}
            LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}/Plugins/${library_name}
            RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}/Plugins/${library_name}
            FRAMEWORK DESTINATION ${CMAKE_INSTALL_LIBDIR}/Plugins/${library_name}
            RESOURCE DESTINATION "${CMAKE_INSTALL_LIBDIR}/Plugins/${library_name}"
    )

endfunction()

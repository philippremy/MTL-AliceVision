# Add library function
function(alicevision_add_library library_name)
    set(options USE_CUDA)
    set(singleValues "")
    set(multipleValues SOURCES HEADERS PUBLIC_LINKS PRIVATE_LINKS PUBLIC_INCLUDE_DIRS PRIVATE_INCLUDE_DIRS PUBLIC_DEFINITIONS PRIVATE_DEFINITIONS RESOURCES FRAMEWORKS)

    cmake_parse_arguments(LIBRARY "${options}" "${singleValues}" "${multipleValues}" ${ARGN})

    if (NOT library_name)
        message(FATAL_ERROR "[AliceVision] You must provide the library name in 'alicevision_add_library'")
    endif()

    if(NOT LIBRARY_SOURCES)
        message(FATAL_ERROR "[AliceVision] You must provide the library SOURCES in 'alicevision_add_library'")
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
        list(APPEND _RESOLVED_HEADERS "${ALICEVISION_INCLUDE_DIR}/${library_name}/${HEADER}")
    endforeach()

    if (NOT LIBRARY_USE_CUDA)

        add_library(${library_name} ${LIBRARY_SOURCES} ${_RESOLVED_HEADERS} ${LIBRARY_RESOURCES})

        # Add Apple Framework Information
        if(BUILD_APPLE_FRAMEWORKS)
            file(GLOB_RECURSE INCLUDE_FILES_FRAMEWORK "${ALICEVISION_INCLUDE_DIR}/${library_name}/*.hpp")
            set_target_properties(${library_name} PROPERTIES
                    INSTALL_NAME_DIR "@rpath"
                    FRAMEWORK TRUE
                    FRAMEWORK_VERSION A
                    MACOSX_FRAMEWORK_NAME "${library_name}"
                    MACOSX_FRAMEWORK_BUNDLE_NAME "${library_name}"
                    MACOSX_FRAMEWORK_IDENTIFIER "org.aliceVision.${library_name}"
                    XCODE_ATTRIBUTE_PRODUCT_BUNDLE_IDENTIFIER "org.aliceVision.${library_name}"
                    MACOSX_FRAMEWORK_BUNDLE_VERSION "${ALICEVISION_VERSION_MAJOR}.${ALICEVISION_VERSION_MINOR}.${ALICEVISION_VERSION_REVISION}"
                    MACOSX_FRAMEWORK_SHORT_VERSION_STRING "${ALICEVISION_VERSION_MAJOR}.${ALICEVISION_VERSION_MINOR}"
                    VERSION "${ALICEVISION_VERSION_MAJOR}.${ALICEVISION_VERSION_MINOR}.${ALICEVISION_VERSION_REVISION}"
                    PUBLIC_HEADER "${_RESOLVED_HEADERS}"
                    RESOURCE "${LIBRARY_RESOURCES}"
                    XCODE_EMBED_FRAMEWORKS "${LIBRARY_FRAMEWORKS}"
                    MACOSX_FRAMEWORK_INFO_PLIST "${CMAKE_SOURCE_DIR}/CMake/Resources/FrameworkInfo.plist.in"
            )
        endif()

        if (ALICEVISION_BUILD_COVERAGE AND CMAKE_COMPILER_IS_GNUCXX)
            append_coverage_compiler_flags_to_target(${library_name})
        endif()

    elseif (BUILD_SHARED_LIBS)
        cuda_add_library(${library_name} SHARED ${LIBRARY_SOURCES} ${_RESOLVED_HEADERS} OPTIONS --compiler-options "-fPIC")
    else()
        cuda_add_library(${library_name} ${LIBRARY_SOURCES} ${_RESOLVED_HEADERS})
    endif()

    if (ALICEVISION_REMOVE_ABSOLUTE)
        foreach (item ${LIBRARY_PUBLIC_LINKS})
            get_filename_component(nameItem ${item} NAME)
            list(APPEND TRANSFORMED_LIBRARY_PUBLIC_LINKS ${nameItem})
        endforeach()
    else()
        set(TRANSFORMED_LIBRARY_PUBLIC_LINKS ${LIBRARY_PUBLIC_LINKS})
    endif()

    # FindCUDA.cmake implicit	target_link_libraries() can not be mixed with new signature (CMake < 3.9.0)
    if (NOT LIBRARY_USE_CUDA)
        target_link_libraries(${library_name}
                PUBLIC ${TRANSFORMED_LIBRARY_PUBLIC_LINKS}
                PRIVATE ${LIBRARY_PRIVATE_LINKS}
        )
    else()
        target_link_libraries(${library_name}
                ${TRANSFORMED_LIBRARY_PUBLIC_LINKS}
                ${LIBRARY_PRIVATE_LINKS}
        )
    endif()

    target_include_directories(${library_name}
            PUBLIC $<BUILD_INTERFACE:${ALICEVISION_INCLUDE_DIR}>
            $<INSTALL_INTERFACE:include>
            ${LIBRARY_PUBLIC_INCLUDE_DIRS}
            PRIVATE ${LIBRARY_PRIVATE_INCLUDE_DIRS}
    )

    target_compile_definitions(${library_name}
            PUBLIC ${LIBRARY_PUBLIC_DEFINITIONS}
            PRIVATE ${LIBRARY_PRIVATE_DEFINITIONS}
    )

    set_property(TARGET ${library_name}
            PROPERTY FOLDER "AliceVision"
    )

    set_target_properties(${library_name}
            PROPERTIES SOVERSION "${ALICEVISION_VERSION_MAJOR}.${ALICEVISION_VERSION_MINOR}"
            VERSION "${ALICEVISION_VERSION_MAJOR}.${ALICEVISION_VERSION_MINOR}.${ALICEVISION_VERSION_REVISION}"
    )

    if ((MSVC) AND (MSVC_VERSION GREATER_EQUAL 1914))
        target_compile_options(${library_name} PUBLIC "/Zc:__cplusplus")
    endif()

    # Only Xcode handles copying resources/frameworks correctly, on other generators we need to imitate
    if(BUILD_APPLE_FRAMEWORKS AND NOT ${CMAKE_GENERATOR} STREQUAL "Xcode")

        # We will add a post-build command for each resource specified
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

            if(NOT IS_DIRECTORY "${SOURCE_PATH_ABSOLUTE}")
                # Get the targets library output directory
                set(LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}")

                # Get the file name
                get_filename_component(RESOURCE_NAME "${SOURCE_PATH_ABSOLUTE}" NAME)

                # Create the custom copy command
                add_custom_command(TARGET ${library_name} POST_BUILD
                        COMMAND ${CMAKE_COMMAND} -E copy
                        "${SOURCE_PATH_ABSOLUTE}" "${LIBRARY_OUTPUT_DIRECTORY}/${library_name}.framework/Resources/${RESOURCE_NAME}"
                        COMMENT "Copying Framework Resource file: ${RESOURCE}"
                )
            else()
                # Get the targets library output directory
                set(LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}")

                # Get the file name
                get_filename_component(RESOURCE_NAME "${SOURCE_PATH_ABSOLUTE}" NAME)

                # Create the custom copy command
                add_custom_command(TARGET ${library_name} POST_BUILD
                        COMMAND ${CMAKE_COMMAND} -E copy_directory
                        "${SOURCE_PATH_ABSOLUTE}" "${LIBRARY_OUTPUT_DIRECTORY}/${library_name}.framework/Resources/${RESOURCE_NAME}"
                        COMMENT "Copying Framework Resource directory: ${RESOURCE}"
                )
            endif()
        endforeach()

        # Copy Frameworks
        foreach(FRAMEWORK IN LISTS LIBRARY_FRAMEWORKS)
            set(SOURCE_PATH_ABSOLUTE)
            # Normalize Path
            if(NOT IS_ABSOLUTE "${FRAMEWORK}")
                cmake_path(SET RESOURCE_PATH "${FRAMEWORK}")
                cmake_path(NORMAL_PATH RESOURCE_PATH)
                cmake_path(ABSOLUTE_PATH RESOURCE_PATH BASE_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}" OUTPUT_VARIABLE SOURCE_PATH_ABSOLUTE)
            else()
                set(SOURCE_PATH_ABSOLUTE "${FRAMEWORK}")
            endif()

            if(NOT IS_DIRECTORY "${FRAMEWORK}")
                # Get the targets library output directory
                set(LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}")

                # Get the file name
                get_filename_component(FRAMEWORK_NAME "${SOURCE_PATH_ABSOLUTE}" NAME)

                # Create the custom copy command
                add_custom_command(TARGET ${library_name} POST_BUILD
                        COMMAND ${CMAKE_COMMAND} -E copy
                        "${SOURCE_PATH_ABSOLUTE}" "${LIBRARY_OUTPUT_DIRECTORY}/${library_name}.framework/Versions/Current/Frameworks/${FRAMEWORK_NAME}"
                        COMMENT "Copying Framework/Library into Framework: ${FRAMEWORK}"
                )
            else()
                # Get the targets library output directory
                set(LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}")

                # Get the file name
                get_filename_component(FRAMEWORK_NAME "${SOURCE_PATH_ABSOLUTE}" NAME)

                # Create the custom copy command
                add_custom_command(TARGET ${library_name} POST_BUILD
                        COMMAND ${CMAKE_COMMAND} -E copy_directory
                        "${SOURCE_PATH_ABSOLUTE}" "${LIBRARY_OUTPUT_DIRECTORY}/${library_name}.framework/Versions/Current/Frameworks/${FRAMEWORK_NAME}"
                        COMMENT "Copying Framework/Library into Framework: ${FRAMEWORK}"
                )
            endif()
        endforeach()

    endif()

    # Append to library list
    set(ALICEVISION_LIBRARIES "${ALICEVISION_LIBRARIES};${library_name}" CACHE INTERNAL "AliceVision Libraries")

    install(TARGETS ${library_name}
            EXPORT aliceVision-targets
            ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
            LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
            RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
            FRAMEWORK DESTINATION ${CMAKE_INSTALL_LIBDIR}
            RESOURCE DESTINATION "${CMAKE_INSTALL_DATADIR}/AliceVision"
    )

endfunction()

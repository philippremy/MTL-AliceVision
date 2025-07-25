# Add kernel library function
function(alicevision_add_kernel_library library_name)
    set(options)
    set(singleValues LANGUAGE OUTPUT_DIR OUTPUT_HEADER)
    set(multipleValues SOURCES HEADERS INCLUDE_DIRS RESOURCES FRAMEWORKS)

    cmake_parse_arguments(LIBRARY "${options}" "${singleValues}" "${multipleValues}" ${ARGN})

    if (NOT library_name)
        message(FATAL_ERROR "[AliceVision] You must provide the library name in 'alicevision_add_kernel_library'")
    endif()

    if(NOT LIBRARY_SOURCES)
        message(FATAL_ERROR "[AliceVision] You must provide the library SOURCES in 'alicevision_add_kernel_library'")
    endif()

    if(NOT LIBRARY_LANGUAGE)
        message(FATAL_ERROR "[AliceVision] You must provide the library language in 'alicevision_add_kernel_library'")
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
    set(_RESOLVED_HEADERS)
    foreach(HEADER IN LISTS INCLUDE_DIRS)
        list(APPEND _RESOLVED_HEADERS "${ALICEVISION_INCLUDE_DIR}/${library_name}/${HEADER}")
    endforeach()

    # Handle GLSL
    if("${LIBRARY_LANGUAGE}" STREQUAL "GLSL")

        # Generate the Include Options
        set(INCLUDE_OPTIONS)
        foreach(INCLUDE_PATH IN LISTS LIBRARY_INCLUDE_DIRS)
            list(APPEND INCLUDE_OPTIONS "-I${INCLUDE_PATH}")
        endforeach()

        set(GLSL_TARGET_PRODUCTS)
        set(GLSL_TARGET_MODULES)
        set(GLSL_METAL_SHADERS)

        # Create a custom target for each GLSL source
        foreach(GLSL_SOURCE IN LISTS LIBRARY_SOURCES)

            # Get filename without extension
            get_filename_component(GLSL_SOURCE_NAME "${GLSL_SOURCE}" NAME_WE)
            list(APPEND GLSL_TARGET_MODULES "${GLSL_SOURCE_NAME}")

            # Full path to the source file
            set(GLSL_SOURCE_FULL "${CMAKE_SOURCE_DIR}/Sources/Libraries/${library_name}/${GLSL_SOURCE}")

            # Path to generated header
            set(GENERATED_HEADER_PATH "${CMAKE_SOURCE_DIR}/Sources/Headers/${library_name}/${LIBRARY_OUTPUT_DIR}/Generated/${GLSL_SOURCE_NAME}.hpp")

            add_custom_command(OUTPUT "${GENERATED_HEADER_PATH}"
                    COMMAND ${Vulkan_GLSLANG_VALIDATOR_EXECUTABLE}
                    -V
                    --auto-map-bindings
                    --enhanced-msgs
                    --error-column
                    --vn "${GLSL_SOURCE_NAME}"
                    --quiet
                    -t
                    ${INCLUDE_OPTIONS}
                    -o "${GENERATED_HEADER_PATH}"
                    "${GLSL_SOURCE_FULL}"
                    COMMAND ${CMAKE_COMMAND}
                    "-DHEADER=${GENERATED_HEADER_PATH}"
                    "-DMODULE_NAME=${GLSL_SOURCE_NAME}"
                    -P "${CMAKE_SOURCE_DIR}/CMake/Helpers/AddGLSLSize.cmake"
                    MAIN_DEPENDENCY "${GLSL_SOURCE_FULL}"
                    DEPENDS "${GLSL_SOURCE_FULL}" ${_RESOLVED_HEADERS}
                    DEPENDS_EXPLICIT_ONLY
                    COMMENT "Building SPIR-V Kernel: ${GLSL_SOURCE_FULL}"
            )

            list(APPEND GLSL_TARGET_PRODUCTS "${GENERATED_HEADER_PATH}")

            # Path to generated header
            set(GENERATED_SPV_PATH_METAL "${CMAKE_SOURCE_DIR}/Sources/Headers/${library_name}/${LIBRARY_OUTPUT_DIR}/Generated/${GLSL_SOURCE_NAME}.metal.spv")
            set(GENERATED_HEADER_PATH_METAL "${CMAKE_SOURCE_DIR}/Sources/Headers/${library_name}/${LIBRARY_OUTPUT_DIR}/Generated/${GLSL_SOURCE_NAME}.metal")

            add_custom_command(OUTPUT "${GENERATED_HEADER_PATH_METAL}" "${GENERATED_SPV_PATH_METAL}"
                    COMMAND ${Vulkan_GLSLANG_VALIDATOR_EXECUTABLE}
                    -V
                    --auto-map-bindings
                    --enhanced-msgs
                    --error-column
                    --quiet
                    -t
                    ${INCLUDE_OPTIONS}
                    --ku
                    -o
                    "${GENERATED_SPV_PATH_METAL}"
                    "${GLSL_SOURCE_FULL}"
                    COMMAND "/Users/philippremy/VulkanSDK/1.3.275.0/macOS/bin/MoltenVKShaderConverter"
                    -mo
                    "${GENERATED_HEADER_PATH_METAL}"
                    -XS
                    -mab
                    -si
                    "${GENERATED_SPV_PATH_METAL}"
                    MAIN_DEPENDENCY "${GLSL_SOURCE_FULL}"
                    DEPENDS "${GLSL_SOURCE_FULL}" ${_RESOLVED_HEADERS}
                    DEPENDS_EXPLICIT_ONLY
                    COMMENT "Building Metal Kernel: ${GLSL_SOURCE_FULL}"
            )

            list(APPEND GLSL_METAL_SHADERS "${GENERATED_HEADER_PATH_METAL}")
            list(APPEND GLSL_TARGET_PRODUCTS "${GENERATED_SPV_PATH_METAL}")
            list(APPEND GLSL_TARGET_PRODUCTS "${GENERATED_HEADER_PATH_METAL}")

        endforeach()

        # Umbrella header generation
        set(UMBRELLA_HEADER "${CMAKE_SOURCE_DIR}/Sources/Headers/${library_name}/${LIBRARY_OUTPUT_DIR}/${LIBRARY_OUTPUT_HEADER}")
        string(REPLACE ";" "\\;" ESCAPED_HEADER_LIST "${GLSL_TARGET_PRODUCTS}")

        add_custom_command(OUTPUT ${UMBRELLA_HEADER}
                COMMAND ${CMAKE_COMMAND} -E remove "${UMBRELLA_HEADER}"
                COMMAND ${CMAKE_COMMAND}
                -DHEADERS="${ESCAPED_HEADER_LIST}"
                -DOUT="${UMBRELLA_HEADER}"
                -P "${CMAKE_SOURCE_DIR}/CMake/Helpers/GenerateUmbrellaHeader.cmake"
                DEPENDS ${GLSL_TARGET_PRODUCTS}
                DEPENDS_EXPLICIT_ONLY
                COMMENT "Generating Umbrella Header for: ${library_name}"
        )

        # Public header generation
        set(PUBLIC_HEADER "${CMAKE_SOURCE_DIR}/Sources/Headers/${library_name}/${library_name}.hpp")
        string(REPLACE ";" "\\;" ESCAPED_MODULE_LIST "${GLSL_TARGET_MODULES}")

        add_custom_command(OUTPUT ${PUBLIC_HEADER}
                COMMAND ${CMAKE_COMMAND} -E remove "${PUBLIC_HEADER}"
                COMMAND ${CMAKE_COMMAND}
                -DMODULES="${ESCAPED_MODULE_LIST}"
                -DOUT="${PUBLIC_HEADER}"
                -P "${CMAKE_SOURCE_DIR}/CMake/Helpers/GeneratePublicHeader.cmake"
                DEPENDS ${GLSL_TARGET_PRODUCTS}
                DEPENDS_EXPLICIT_ONLY
                COMMENT "Generating Public Header for: ${library_name}"
        )

        # Configure the cpp source file
        set(GENERATED_CPP_SOURCE "${CMAKE_SOURCE_DIR}/Sources/Libraries/${library_name}/${library_name}.cpp")
        configure_file(
                "${CMAKE_SOURCE_DIR}/CMake/Resources/LinkerStub.cpp.in"
                "${GENERATED_CPP_SOURCE}"
                @ONLY
        )

        # Add the library with sources and generated headers
        add_library(${library_name}
                ${LIBRARY_SOURCES}
                ${_RESOLVED_HEADERS}
                ${LIBRARY_RESOURCES}
                "${PUBLIC_HEADER}"
                "${GENERATED_CPP_SOURCE}"
                "${GLSL_METAL_SHADERS}"
        )

        set_property(TARGET ${library_name} APPEND PROPERTY OBJECT_DEPENDS "${PUBLIC_HEADER}" "${UMBRELLA_HEADER}")

        # Add Apple Framework Information
        if(BUILD_APPLE_FRAMEWORKS)
            file(GLOB_RECURSE INCLUDE_FILES_FRAMEWORK "${ALICEVISION_INCLUDE_DIR}/${library_name}/*.hpp")
            set_target_properties(${library_name} PROPERTIES
                    INSTALL_NAME_DIR "@rpath"
                    BUILD_WITH_INSTALL_RPATH ON
                    FRAMEWORK TRUE
                    FRAMEWORK_VERSION A
                    MACOSX_FRAMEWORK_NAME "${library_name}"
                    MACOSX_FRAMEWORK_BUNDLE_NAME "${library_name}"
                    MACOSX_FRAMEWORK_IDENTIFIER "org.aliceVision.${library_name}"
                    XCODE_ATTRIBUTE_PRODUCT_BUNDLE_IDENTIFIER "org.aliceVision.${library_name}"
                    MACOSX_FRAMEWORK_BUNDLE_VERSION "${ALICEVISION_VERSION_MAJOR}.${ALICEVISION_VERSION_MINOR}.${ALICEVISION_VERSION_REVISION}"
                    MACOSX_FRAMEWORK_SHORT_VERSION_STRING "${ALICEVISION_VERSION_MAJOR}.${ALICEVISION_VERSION_MINOR}"
                    VERSION "${ALICEVISION_VERSION_MAJOR}.${ALICEVISION_VERSION_MINOR}.${ALICEVISION_VERSION_REVISION}"
                    PUBLIC_HEADER "${PUBLIC_HEADER}"
                    RESOURCE "${LIBRARY_RESOURCES} ${GLSL_METAL_SHADERS}"
                    XCODE_EMBED_FRAMEWORKS "${LIBRARY_FRAMEWORKS}"
                    MACOSX_FRAMEWORK_INFO_PLIST "${CMAKE_SOURCE_DIR}/CMake/Resources/FrameworkInfo.plist.in"
            )
        endif()

    elseif("${LIBRARY_LANGUAGE}" STREQUAL "Metal")

        add_metal_shader_library(${library_name}
                ${LIBRARY_SOURCES}
                INCLUDE_PATHS
                    "${LIBRARY_INCLUDE_DIRS}"
                STANDARD 3.0
        )

    else()
        message(FATAL_ERROR "[AliceVision] Unsupported library language in 'alicevision_add_kernel_library'")
    endif()

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

    target_include_directories(${library_name}
            PUBLIC ${LIBRARY_OUTPUT_DIR}
            $<INSTALL_INTERFACE:include>
            ${LIBRARY_INCLUDE_DIRS}
    )

    set_property(TARGET ${library_name}
            PROPERTY FOLDER "AliceVision"
    )

    set_target_properties(${library_name}
            PROPERTIES SOVERSION "${ALICEVISION_VERSION_MAJOR}.${ALICEVISION_VERSION_MINOR}"
            VERSION "${ALICEVISION_VERSION_MAJOR}.${ALICEVISION_VERSION_MINOR}.${ALICEVISION_INSTALL_VERSION_REVISION}"
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

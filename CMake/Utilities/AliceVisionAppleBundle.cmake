function(alicevision_add_apple_bundle bundle_name)
    set(options SYMLINK_FRAMEWORK_RESOURCES)
    set(singleValues ICON)
    set(multipleValues BINARIES LIBRARIES RESOURCES)

    cmake_parse_arguments(BUNDLE "${options}" "${singleValues}" "${multipleValues}" ${ARGN})

    # include(BundleUtilities)

    # Create a plain old bundle structure
    file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/Bundle/AliceVision.bundle")
    file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/Bundle/AliceVision.bundle/Contents")
    file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/Bundle/AliceVision.bundle/Contents/MacOS")
    file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/Bundle/AliceVision.bundle/Contents/Resources")
    file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/Bundle/AliceVision.bundle/Contents/Libraries")
    file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/Bundle/AliceVision.bundle/Contents/Frameworks")

    # Configure the Info.plist file
    set(BUNDLE_NAME "${bundle_name}")
    set(VERSION "${ALICEVISION_VERSION}")
    set(SHORT_VERSION "${ALICEVISION_VERSION_MAJOR}.${ALICEVISION_VERSION_MINOR}")
    set(OSX_DEPLOYMENT_MINIMUM "${CMAKE_OSX_DEPLOYMENT_TARGET}")
    get_filename_component(BUNDLE_ICON_NAME "${BUNDLE_ICON}" NAME)
    set(ICON_FILE "${BUNDLE_ICON_NAME}")
    configure_file("${CMAKE_SOURCE_DIR}/CMake/Resources/BundleInfo.plist.in"
            "${CMAKE_BINARY_DIR}/Bundle/AliceVision.bundle/Contents/Info.plist"
            @ONLY
    )

    # Create a custom target for the bundle (not built by default)
    add_custom_target(AliceVisionBundle ALL
            COMMENT "Copying files to AliceVision Bundle..."
    )

    # For each library:
    foreach(LIBRARY IN LISTS BUNDLE_LIBRARIES)
        # Make sure the bundle target depends on the library target,
        # so the libraries get built before the bundle.
        add_dependencies(AliceVisionBundle ${LIBRARY})

        # Add a post-build command to copy the framework directory into the bundle folder
        add_custom_command(TARGET AliceVisionBundle POST_BUILD
                COMMAND rsync -a
                $<TARGET_LINKER_FILE_DIR:${LIBRARY}>
                ${CMAKE_BINARY_DIR}/Bundle/AliceVision.bundle/Contents/Frameworks
        )
    endforeach()

    # Symlink Framework Resources
    if(BUNDLE_SYMLINK_FRAMEWORK_RESOURCES)
        # Iterate through the Framework Paths and check their resource dir for content
        foreach(LIBRARY IN LISTS BUNDLE_LIBRARIES)
            add_custom_command(TARGET AliceVisionBundle POST_BUILD
                COMMAND ${CMAKE_COMMAND}
                    -DSOURCE_FRAMEWORK="${CMAKE_BINARY_DIR}/Bundle/AliceVision.bundle/Contents/Frameworks/${LIBRARY}.framework/Resources"
                    -DDESTINATION=${CMAKE_BINARY_DIR}/Bundle/AliceVision.bundle/Contents/Resources
                    -P "${CMAKE_SOURCE_DIR}/CMake/Helpers/CreateResourceSymlinks.cmake"
            )
        endforeach()
    endif()

    # For each binary:
    foreach(BINARY IN LISTS BUNDLE_BINARIES)
        # Make sure the bundle target depends on the library target,
        # so the libraries get built before the bundle.

        # Strip the _exe suffix for creating a symlink
        add_dependencies(AliceVisionBundle ${BINARY})
        string(LENGTH "${BINARY}" VAR_LEN)
        math(EXPR NEW_LEN "${VAR_LEN} - 4")
        string(SUBSTRING "${BINARY}" 0 ${NEW_LEN} BINARAY_NAME)

        # Add a post-build command to copy the framework directory into the bundle folder
        add_custom_command(TARGET AliceVisionBundle POST_BUILD
                COMMAND rsync -a
                    $<TARGET_FILE:${BINARY}>
                    ${CMAKE_BINARY_DIR}/Bundle/AliceVision.bundle/Contents/MacOS
                COMMAND ${CMAKE_COMMAND} -E create_symlink
                    ${CMAKE_BINARY_DIR}/Bundle/AliceVision.bundle/Contents/MacOS/$<TARGET_FILE_NAME:${BINARY}>
                    ${CMAKE_BINARY_DIR}/Bundle/AliceVision.bundle/Contents/MacOS/${BINARAY_NAME}
        )
    endforeach()

    # Resources (always add the Bundle icon)
    list(APPEND BUNDLE_RESOURCES "${BUNDLE_ICON}")
    foreach(RESOURCE IN LISTS BUNDLE_RESOURCES)
        # Add a post-build command to copy the resource into the bundle folder
        add_custom_command(TARGET AliceVisionBundle POST_BUILD
                COMMAND rsync -a
                ${RESOURCE}
                ${CMAKE_BINARY_DIR}/Bundle/AliceVision.bundle/Contents/Resources
        )
    endforeach()

    # Per custom command and target, attempt to find all dependencies
    # Create a custom target for the bundle (not built by default)
    add_custom_target(AliceVisionBundleFixUpDependencies ALL
            COMMENT "Fixing up dependencies in AliceVision Bundle..."
    )

    # The bundle must be completely populated
    add_dependencies(AliceVisionBundleFixUpDependencies
            AliceVisionBundle
    )

    # Execute the fixup script
    add_custom_command(TARGET AliceVisionBundleFixUpDependencies POST_BUILD
            COMMAND ${CMAKE_COMMAND}
                "-DBUNDLE=${CMAKE_BINARY_DIR}/Bundle/AliceVision.bundle"
                -P "${CMAKE_SOURCE_DIR}/CMake/Helpers/FixupAppleBundle.cmake"
    )

    # Set the icon file
    add_custom_command(TARGET AliceVisionBundleFixUpDependencies POST_BUILD
            COMMAND "${CMAKE_SOURCE_DIR}/Utils/ThirdParty/fileicon.sh"
            set
                "${CMAKE_BINARY_DIR}/Bundle/AliceVision.bundle"
                "${BUNDLE_ICON}"
            --quiet
    )

endfunction()
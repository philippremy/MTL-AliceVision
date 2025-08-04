function(alicevision_add_apple_bundle bundle_name)
    set(options SYMLINK_FRAMEWORK_RESOURCES)
    set(singleValues ICON)
    set(multipleValues BINARIES LIBRARIES PLUGINS RESOURCES)

    cmake_parse_arguments(BUNDLE "${options}" "${singleValues}" "${multipleValues}" ${ARGN})

    # include(BundleUtilities)

    # Create a plain old bundle structure
    file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/Bundle/AliceVision.bundle")
    file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/Bundle/AliceVision.bundle/Contents")
    file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/Bundle/AliceVision.bundle/Contents/MacOS")
    file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/Bundle/AliceVision.bundle/Contents/Resources")
    file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/Bundle/AliceVision.bundle/Contents/Libraries")
    file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/Bundle/AliceVision.bundle/Contents/PlugIns")
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
            COMMENT "Copying library ${LIBRARY} to AliceVision Bundle..."
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
                COMMENT "Copying binary ${BINARAY_NAME} to AliceVision Bundle..."
        )
    endforeach()

    foreach(PLUGIN IN LISTS BUNDLE_PLUGINS)
        # Get Version from the target
        get_target_property(PLUGIN_VERSION ${PLUGIN} VERSION)
        # Extract major version (before first dot)
        string(REGEX MATCH "^[0-9]+" PLUGIN_VERSION_MAJOR "${PLUGIN_VERSION}")

        # Copy the output file of the target
        file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/Bundle/AliceVision.bundle/Contents/PlugIns/${PLUGIN}")
        add_custom_command(TARGET AliceVisionBundle POST_BUILD
                COMMAND rsync -a
                $<TARGET_FILE:${PLUGIN}>
                ${CMAKE_BINARY_DIR}/Bundle/AliceVision.bundle/Contents/PlugIns/${PLUGIN}
                COMMENT "Copying plugin ${PLUGIN} to AliceVision Bundle..."
        )
        # Regular without soversion
        add_custom_command(TARGET AliceVisionBundle POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E create_symlink
                ${CMAKE_BINARY_DIR}/Bundle/AliceVision.bundle/Contents/PlugIns/${PLUGIN}/$<TARGET_FILE_NAME:${PLUGIN}>
                ${CMAKE_BINARY_DIR}/Bundle/AliceVision.bundle/Contents/PlugIns/${PLUGIN}/$<TARGET_FILE_PREFIX:${PLUGIN}>$<TARGET_FILE_BASE_NAME:${PLUGIN}>$<TARGET_FILE_SUFFIX:${PLUGIN}>
        )
        # With soversion
        add_custom_command(TARGET AliceVisionBundle POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E create_symlink
                ${CMAKE_BINARY_DIR}/Bundle/AliceVision.bundle/Contents/PlugIns/${PLUGIN}/$<TARGET_FILE_NAME:${PLUGIN}>
                ${CMAKE_BINARY_DIR}/Bundle/AliceVision.bundle/Contents/PlugIns/${PLUGIN}/$<TARGET_FILE_PREFIX:${PLUGIN}>$<TARGET_FILE_BASE_NAME:${PLUGIN}>.$<TARGET_PROPERTY:${PLUGIN},SOVERSION>$<TARGET_FILE_SUFFIX:${PLUGIN}>
        )
        # With major version
        add_custom_command(TARGET AliceVisionBundle POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E create_symlink
                ${CMAKE_BINARY_DIR}/Bundle/AliceVision.bundle/Contents/PlugIns/${PLUGIN}/$<TARGET_FILE_NAME:${PLUGIN}>
                ${CMAKE_BINARY_DIR}/Bundle/AliceVision.bundle/Contents/PlugIns/${PLUGIN}/$<TARGET_FILE_PREFIX:${PLUGIN}>$<TARGET_FILE_BASE_NAME:${PLUGIN}>.${PLUGIN_VERSION_MAJOR}$<TARGET_FILE_SUFFIX:${PLUGIN}>
        )
        # Now get the resources which should be copied with it
        get_target_property(PLUGIN_RESOURCES_LIST ${PLUGIN} PLUGIN_RESOURCES)
        foreach(RES IN LISTS PLUGIN_RESOURCES_LIST)
        add_custom_command(TARGET AliceVisionBundle POST_BUILD
                COMMAND rsync -a
                "${RES}"
                ${CMAKE_BINARY_DIR}/Bundle/AliceVision.bundle/Contents/PlugIns/${PLUGIN}
        )
        endforeach()
    endforeach()

    # Resources (always add the Bundle icon)
    list(APPEND BUNDLE_RESOURCES "${BUNDLE_ICON}")
    foreach(RESOURCE IN LISTS BUNDLE_RESOURCES)
        # Add a post-build command to copy the resource into the bundle folder
        add_custom_command(TARGET AliceVisionBundle POST_BUILD
                COMMAND rsync -a
                ${RESOURCE}
                ${CMAKE_BINARY_DIR}/Bundle/AliceVision.bundle/Contents/Resources
            COMMENT "Copying resource ${RESOURCE} to AliceVision Bundle..."
        )
    endforeach()

    # Per custom command and target, attempt to find all dependencies
    # Create a custom target for the bundle (not built by default)
    add_custom_target(AliceVisionBundleFixUpDependencies ALL)

    # The bundle must be completely populated
    add_dependencies(AliceVisionBundleFixUpDependencies
            AliceVisionBundle
    )

    # Execute the fixup script
    add_custom_command(TARGET AliceVisionBundleFixUpDependencies POST_BUILD
            COMMAND ${CMAKE_COMMAND}
                "-DBUNDLE=${CMAKE_BINARY_DIR}/Bundle/AliceVision.bundle"
                -P "${CMAKE_SOURCE_DIR}/CMake/Helpers/FixupAppleBundle.cmake"
            COMMENT "Fixing up dependencies in AliceVision Bundle..."
    )

    # Set the icon file
    add_custom_command(TARGET AliceVisionBundleFixUpDependencies POST_BUILD
            COMMAND "${CMAKE_SOURCE_DIR}/Utils/ThirdParty/fileicon.sh"
            set
                "${CMAKE_BINARY_DIR}/Bundle/AliceVision.bundle"
                "${BUNDLE_ICON}"
            --quiet
            COMMENT "Setting icon for AliceVision Bundle..."
    )

endfunction()

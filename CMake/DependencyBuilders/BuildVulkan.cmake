# Utility to build a simple OpenMP library from source

macro(build_vulkan)

    set(CURRENT_DEPENDENCY Vulkan)

    # Create Logging Folder
    file(MAKE_DIRECTORY "${CMAKE_SOURCE_DIR}/External/Logs/${CURRENT_DEPENDENCY}")

    # Fetch Vulkan SDK
    message(STATUS "[AliceVision] Fetching ${CURRENT_DEPENDENCY}...")
    if(APPLE)
        file(DOWNLOAD
                "https://sdk.lunarg.com/sdk/download/1.4.313.1/mac/vulkansdk-macos-1.4.313.1.zip"
                "${CMAKE_SOURCE_DIR}/External/Logs/${CURRENT_DEPENDENCY}/VulkanSDK.zip"
        )
        file(ARCHIVE_EXTRACT
                INPUT "${CMAKE_SOURCE_DIR}/External/Logs/${CURRENT_DEPENDENCY}/VulkanSDK.zip"
                DESTINATION "${CMAKE_SOURCE_DIR}/External/Logs/${CURRENT_DEPENDENCY}"
        )
    elseif(LINUX)
    elseif(WIN32)
    else()
        message(FATAL_ERROR "[AliceVision] The target OS has no distributed SDKs for Vulkan!")
    endif()

    # Create Build Directory
    file(MAKE_DIRECTORY "${CMAKE_SOURCE_DIR}/External/${CURRENT_DEPENDENCY}/Build")

    # Configure
    message(STATUS "[AliceVision] Configuring ${CURRENT_DEPENDENCY}...")
    execute_process(COMMAND ${CMAKE_COMMAND}
            -DCMAKE_INSTALL_PREFIX=${CMAKE_SOURCE_DIR}/External/Products
            -DCMAKE_BUILD_TYPE=Release
            "-DCMAKE_OSX_ARCHITECTURES=${CMAKE_OSX_ARCHITECTURES}"
            -DCMAKE_PREFIX_PATH=${CMAKE_SOURCE_DIR}/External/Products
            -DBUILD_SHARED_LIBS=ON
            -G "${CMAKE_GENERATOR}"
            ..
            WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}/External/${CURRENT_DEPENDENCY}/Build"
            RESULT_VARIABLE CONFIGURE_RESULT_${CURRENT_DEPENDENCY}
            OUTPUT_FILE "${CMAKE_SOURCE_DIR}/External/Logs/${CURRENT_DEPENDENCY}/CMAKE_STDOUT.log"
            ERROR_FILE "${CMAKE_SOURCE_DIR}/External/Logs/${CURRENT_DEPENDENCY}/CMAKE_STDERR.log"
    )
    if(NOT ${CONFIGURE_RESULT_${CURRENT_DEPENDENCY}} EQUAL 0)
        message(FATAL_ERROR "[AliceVision] Failed to configure ${CURRENT_DEPENDENCY}. Check ${CMAKE_SOURCE_DIR}/External/Logs/${CURRENT_DEPENDENCY}/CMAKE_STDERR.log for more information.")
    endif()

    # Build
    message(STATUS "[AliceVision] Building ${CURRENT_DEPENDENCY}...")
    execute_process(COMMAND ${CMAKE_COMMAND}
            --build .
            -j${NCPUs}
            WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}/External/${CURRENT_DEPENDENCY}/Build"
            RESULT_VARIABLE BUILD_RESULT_${CURRENT_DEPENDENCY}
            OUTPUT_FILE "${CMAKE_SOURCE_DIR}/External/Logs/${CURRENT_DEPENDENCY}/BUILD_STDOUT.log"
            ERROR_FILE "${CMAKE_SOURCE_DIR}/External/Logs/${CURRENT_DEPENDENCY}/BUILD_STDERR.log"
    )
    if(NOT ${BUILD_RESULT_${CURRENT_DEPENDENCY}} EQUAL 0)
        message(FATAL_ERROR "[AliceVision] Failed to build ${CURRENT_DEPENDENCY}. Check ${CMAKE_SOURCE_DIR}/External/Logs/${CURRENT_DEPENDENCY}/BUILD_STDERR.log for more information.")
    endif()

    # Install (aka copy)
    message(STATUS "[AliceVision] Installing ${CURRENT_DEPENDENCY}...")
    execute_process(COMMAND ${CMAKE_COMMAND}
            --install .
            --strip
            WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}/External/${CURRENT_DEPENDENCY}/Build"
            RESULT_VARIABLE INSTALL_RESULT_${CURRENT_DEPENDENCY}
            OUTPUT_FILE "${CMAKE_SOURCE_DIR}/External/Logs/${CURRENT_DEPENDENCY}/INSTALL_STDOUT.log"
            ERROR_FILE "${CMAKE_SOURCE_DIR}/External/Logs/${CURRENT_DEPENDENCY}/INSTALL_STDERR.log"
    )
    if(NOT ${INSTALL_RESULT_${CURRENT_DEPENDENCY}} EQUAL 0)
        message(FATAL_ERROR "[AliceVision] Failed to install ${CURRENT_DEPENDENCY}. Check ${CMAKE_SOURCE_DIR}/External/Logs/${CURRENT_DEPENDENCY}/INSTALL_STDERR.log for more information.")
    endif()

    # Find openjph-config and set OpenJPH_DIR
    file(GLOB_RECURSE ${CURRENT_DEPENDENCY}_CONFIG "${CMAKE_SOURCE_DIR}/External/Products/lib/cmake/ClpConfig.cmake")
    if(${CURRENT_DEPENDENCY}_CONFIG)
        get_filename_component(SUBFOLDER_PATH "${${CURRENT_DEPENDENCY}_CONFIG}" DIRECTORY)
        # Store in cache
        set("${CURRENT_DEPENDENCY}_DIR" "${SUBFOLDER_PATH}" CACHE PATH "" FORCE)
    else()
        message(FATAL_ERROR "Could not find CMake Configuration file for ${CURRENT_DEPENDENCY}. Please submit a bug report.")
    endif()

    message(STATUS "[AliceVision] Made dependency ${CURRENT_DEPENDENCY} available.")

endmacro()
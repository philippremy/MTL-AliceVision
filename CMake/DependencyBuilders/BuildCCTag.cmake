# Utility to build a simple OpenMP library from source

macro(build_cctag)

    set(CURRENT_DEPENDENCY CCTag)

    # Create Logging Folder
    file(MAKE_DIRECTORY "${CMAKE_SOURCE_DIR}/External/Logs/${CURRENT_DEPENDENCY}")

    set(DEPENDENCY_DIR "${CMAKE_SOURCE_DIR}/External/${CURRENT_DEPENDENCY}")
    set(DEPENDENCY_LOG_DIR "${CMAKE_SOURCE_DIR}/External/Logs/${CURRENT_DEPENDENCY}")
    set(DEPENDENCY_GIT_HEAD "${DEPENDENCY_DIR}/.git/HEAD")

    # Only fetch if needed
    if(NOT EXISTS "${DEPENDENCY_GIT_HEAD}")
        # Fetch Boost
        message(STATUS "[AliceVision] Fetching ${CURRENT_DEPENDENCY}...")
        execute_process(COMMAND ${GIT_EXECUTABLE}
                clone --depth 1
                https://github.com/alicevision/CCTag.git
                "${CMAKE_SOURCE_DIR}/External/${CURRENT_DEPENDENCY}"
                RESULT_VARIABLE GIT_RESULT_${CURRENT_DEPENDENCY}
                OUTPUT_FILE "${CMAKE_SOURCE_DIR}/External/Logs/${CURRENT_DEPENDENCY}/GIT_STDOUT.log"
                ERROR_FILE "${CMAKE_SOURCE_DIR}/External/Logs/${CURRENT_DEPENDENCY}/GIT_STDERR.log"
        )
        if(NOT ${GIT_RESULT_${CURRENT_DEPENDENCY}} EQUAL 0)
            message(FATAL_ERROR "[AliceVision] Failed to fetch ${CURRENT_DEPENDENCY}. Check ${CMAKE_SOURCE_DIR}/External/Logs/${CURRENT_DEPENDENCY}/GIT_STDERR.log for more information.")
        endif()
    endif()

    # Apply patch for Boost math_c99
    # We need to patch the CMakeLists.txt file
    execute_process(COMMAND ${GIT_EXECUTABLE}
            apply "${CMAKE_SOURCE_DIR}/CMake/Patches/CCTAG-fix-math_c99-dependency.patch"
            WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}/External/${CURRENT_DEPENDENCY}"
            RESULT_VARIABLE PATCH_RESULT_${CURRENT_DEPENDENCY}
            OUTPUT_FILE "${CMAKE_SOURCE_DIR}/External/Logs/${CURRENT_DEPENDENCY}/PATCH_STDOUT.log"
            ERROR_FILE "${CMAKE_SOURCE_DIR}/External/Logs/${CURRENT_DEPENDENCY}/PATCH_STDERR.log"
    )
    if(NOT ${PATCH_RESULT_${CURRENT_DEPENDENCY}} EQUAL 0)
        # DO NOT FAIL. Probably already applied.
        # message(FATAL_ERROR "[AliceVision] Failed to patch ${CURRENT_DEPENDENCY}. Check ${CMAKE_SOURCE_DIR}/External/Logs/${CURRENT_DEPENDENCY}/PATCH_STDERR.log for more information.")
    endif()

    # Create Build Directory
    file(MAKE_DIRECTORY "${CMAKE_SOURCE_DIR}/External/${CURRENT_DEPENDENCY}/Build")

    # Configure
    message(STATUS "[AliceVision] Configuring ${CURRENT_DEPENDENCY}...")
    execute_process(COMMAND ${CMAKE_COMMAND}
            -DCMAKE_INSTALL_PREFIX=${CMAKE_SOURCE_DIR}/External/Products
            -DCMAKE_BUILD_TYPE=Release
            "-DCMAKE_OSX_ARCHITECTURES=x86_64;arm64"
            -DCMAKE_PREFIX_PATH=${CMAKE_SOURCE_DIR}/External/Products
            -DBUILD_SHARED_LIBS=ON
            -DCCTAG_WITH_CUDA=${ALICEVISION_USE_CUDA}
            -DCCTAG_BUILD_APPS=OFF
            -DCCTAG_EIGEN_MEMORY_ALIGNMENT=${ALICEVISION_EIGEN_MEMORY_ALIGNMENT}
            -DCCTAG_BUILD_TESTS=OFF
            -DCCTAG_BUILD_DOC=OFF
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

    # Patch Config to use new Boost policy
    # Read the file contents
    set(CONFIG_FILE "${CMAKE_SOURCE_DIR}/External/Products/lib/cmake/CCTag/CCTagConfig.cmake")
    file(READ "${CONFIG_FILE}" FILE_CONTENTS)

    # Append at the end
    string(REGEX REPLACE
            "stacktrace_basic\\)"
            "stacktrace_basic CONFIG)"
            FILE_PATCHED
            "${FILE_CONTENTS}"
    )

    # Write back (or to a new file)
    file(WRITE "${CONFIG_FILE}" "${FILE_PATCHED}")

    # Find openjph-config and set OpenJPH_DIR
    file(GLOB_RECURSE ${CURRENT_DEPENDENCY}_CONFIG "${CMAKE_SOURCE_DIR}/External/Products/lib/cmake/CCTagConfig.cmake")
    if(${CURRENT_DEPENDENCY}_CONFIG)
        get_filename_component(SUBFOLDER_PATH "${${CURRENT_DEPENDENCY}_CONFIG}" DIRECTORY)
        # Store in cache
        set("${CURRENT_DEPENDENCY}_DIR" "${SUBFOLDER_PATH}" CACHE PATH "" FORCE)
    else()
        message(FATAL_ERROR "Could not find CMake Configuration file for ${CURRENT_DEPENDENCY}. Please submit a bug report.")
    endif()

    message(STATUS "[AliceVision] Made dependency ${CURRENT_DEPENDENCY} available.")

endmacro()
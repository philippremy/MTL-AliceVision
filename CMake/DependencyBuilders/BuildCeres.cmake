# Utility to build a simple OpenMP library from source

macro(build_ceres)

    set(CURRENT_DEPENDENCY Ceres)

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
                https://github.com/ceres-solver/ceres-solver.git
                "${CMAKE_SOURCE_DIR}/External/${CURRENT_DEPENDENCY}"
                RESULT_VARIABLE GIT_RESULT_${CURRENT_DEPENDENCY}
                OUTPUT_FILE "${CMAKE_SOURCE_DIR}/External/Logs/${CURRENT_DEPENDENCY}/GIT_STDOUT.log"
                ERROR_FILE "${CMAKE_SOURCE_DIR}/External/Logs/${CURRENT_DEPENDENCY}/GIT_STDERR.log"
        )
        if(NOT ${GIT_RESULT_${CURRENT_DEPENDENCY}} EQUAL 0)
            message(FATAL_ERROR "[AliceVision] Failed to fetch ${CURRENT_DEPENDENCY}. Check ${CMAKE_SOURCE_DIR}/External/Logs/${CURRENT_DEPENDENCY}/GIT_STDERR.log for more information.")
        endif()
    endif()

    # Create Build Directory
    file(MAKE_DIRECTORY "${CMAKE_SOURCE_DIR}/External/${CURRENT_DEPENDENCY}/BuildDir")

    # Configure
    message(STATUS "[AliceVision] Configuring ${CURRENT_DEPENDENCY}...")
    execute_process(COMMAND ${CMAKE_COMMAND}
            -DCMAKE_INSTALL_PREFIX=${CMAKE_SOURCE_DIR}/External/Products
            -DCMAKE_BUILD_TYPE=Release
            "-DCMAKE_OSX_ARCHITECTURES=${CMAKE_OSX_ARCHITECTURES}"
            -DBUILD_SHARED_LIBS=ON
            -DACCELERATESPARSE=ON
            -DENABLE_BITCODE=OFF
            -DLAPACK=ON
            -DSCHUR_SPECIALIZATIONS=ON
            -DCUSTOM_BLAS=ON
            -DEIGENSPARSE=ON
            -DBUILD_TESTING=OFF
            -DBUILD_DOCUMENTATION=OFF
            -DBUILD_EXAMPLES=OFF
            -DBUILD_BENCHMARKS=OFF
            -Dabsl_VERSION=20250701
            -G "${CMAKE_GENERATOR}"
            ..
            WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}/External/${CURRENT_DEPENDENCY}/BuildDir"
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
            WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}/External/${CURRENT_DEPENDENCY}/BuildDir"
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
            WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}/External/${CURRENT_DEPENDENCY}/BuildDir"
            RESULT_VARIABLE INSTALL_RESULT_${CURRENT_DEPENDENCY}
            OUTPUT_FILE "${CMAKE_SOURCE_DIR}/External/Logs/${CURRENT_DEPENDENCY}/INSTALL_STDOUT.log"
            ERROR_FILE "${CMAKE_SOURCE_DIR}/External/Logs/${CURRENT_DEPENDENCY}/INSTALL_STDERR.log"
    )
    if(NOT ${INSTALL_RESULT_${CURRENT_DEPENDENCY}} EQUAL 0)
        message(FATAL_ERROR "[AliceVision] Failed to install ${CURRENT_DEPENDENCY}. Check ${CMAKE_SOURCE_DIR}/External/Logs/${CURRENT_DEPENDENCY}/INSTALL_STDERR.log for more information.")
    endif()

    # Patch the config file to accept any Abseil Version
    # Read the file contents
    file(READ "${CMAKE_SOURCE_DIR}/External/Products/lib/cmake/Ceres/CeresConfig.cmake" FILE_CONTENTS)

    # Replace all message(STATUS ...) and message( STATUS ...)
    # For example, downgrade to DEBUG or just remove
    string(REGEX REPLACE "ceres_message*\\(STATUS" "ceres_message(TRACE" FILE_PATCHED "${FILE_CONTENTS}")
    string(REGEX REPLACE "set*\\(_CERES_ABSL_VERSION 20250701\\)" "set(_CERES_ABSL_VERSION)" FILE_PATCHED "${FILE_PATCHED}")

    # Write back (or to a new file)
    file(WRITE "${CMAKE_SOURCE_DIR}/External/Products/lib/cmake/Ceres/CeresConfig.cmake" "${FILE_PATCHED}")

    # Patch FindSuiteSparse helper file to shut up
    # Read the file contents
    file(READ "${CMAKE_SOURCE_DIR}/External/Products/lib/cmake/Ceres/FindSuiteSparse.cmake" FILE_CONTENTS)

    # Replace all message(STATUS ...) and message( STATUS ...)
    # For example, downgrade to DEBUG or just remove
    string(REGEX REPLACE "message *\\( *STATUS" "message(TRACE" FILE_PATCHED "${FILE_CONTENTS}")
    string(REGEX REPLACE "find_package *\\(METIS\\)" "find_package (METIS QUIET)" FILE_PATCHED "${FILE_PATCHED}")

    # Write back (or to a new file)
    file(WRITE "${CMAKE_SOURCE_DIR}/External/Products/lib/cmake/Ceres/FindSuiteSparse.cmake" "${FILE_PATCHED}")

    # Find openjph-config and set OpenJPH_DIR
    file(GLOB_RECURSE ${CURRENT_DEPENDENCY}_CONFIG "${CMAKE_SOURCE_DIR}/External/Products/lib/cmake/CeresConfig.cmake")
    if(${CURRENT_DEPENDENCY}_CONFIG)
        get_filename_component(SUBFOLDER_PATH "${${CURRENT_DEPENDENCY}_CONFIG}" DIRECTORY)
        # Store in cache
        set("${CURRENT_DEPENDENCY}_DIR" "${SUBFOLDER_PATH}" CACHE PATH "" FORCE)
    else()
        message(FATAL_ERROR "Could not find CMake Configuration file for ${CURRENT_DEPENDENCY}. Please submit a bug report.")
    endif()

    message(STATUS "[AliceVision] Made dependency ${CURRENT_DEPENDENCY} available.")

endmacro()
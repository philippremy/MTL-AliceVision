# Utility to build a simple OpenMP library from source

macro(build_eigen3)

    set(CURRENT_DEPENDENCY Eigen)

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
            https://gitlab.com/libeigen/eigen.git
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
    file(MAKE_DIRECTORY "${CMAKE_SOURCE_DIR}/External/${CURRENT_DEPENDENCY}/Build")

    # Configure
    message(STATUS "[AliceVision] Configuring ${CURRENT_DEPENDENCY}...")
    execute_process(COMMAND ${CMAKE_COMMAND}
        -DCMAKE_INSTALL_PREFIX=${CMAKE_SOURCE_DIR}/External/Products
        -DCMAKE_BUILD_TYPE=Release
        "-DCMAKE_OSX_ARCHITECTURES=x86_64;arm64"
        -DBUILD_TESTING=OFF
        -DEIGEN_BUILD_TESTING=OFF
        -DEIGEN_LEAVE_TEST_IN_ALL_TARGET=OFF
        -DEIGEN_BUILD_BLAS=ON
        -DEIGEN_BUILD_LAPACK=ON
        -DEIGEN_BUILD_BTL=OFF
        -DEIGEN_BUILD_SPBENCH=OFF
        -DEIGEN_BUILD_DOC_DEFAULT=OFF
        -DEIGEN_BUILD_DOC=OFF
        -DEIGEN_BUILD_DEMOS=OFF
        -DEIGEN_BUILD_CMAKE_PACKAGE=ON
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

    # Find EigenConfig and set Eigen3_DIR
    file(GLOB_RECURSE ${CURRENT_DEPENDENCY}_CONFIG "${CMAKE_SOURCE_DIR}/External/Products/share/eigen3/cmake/Eigen3Config.cmake")
    if(${CURRENT_DEPENDENCY}_CONFIG)
        get_filename_component(SUBFOLDER_PATH "${${CURRENT_DEPENDENCY}_CONFIG}" DIRECTORY)
        # Store in cache
        set("${CURRENT_DEPENDENCY}_DIR" "${SUBFOLDER_PATH}" CACHE PATH "" FORCE)
    else()
        message(FATAL_ERROR "Could not find CMake Configuration file for ${CURRENT_DEPENDENCY}. Please submit a bug report.")
    endif()

    message(STATUS "[AliceVision] Made dependency ${CURRENT_DEPENDENCY} available.")

endmacro()
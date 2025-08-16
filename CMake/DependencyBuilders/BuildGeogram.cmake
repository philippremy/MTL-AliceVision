# Utility to build a simple OpenMP library from source

macro(build_geogram)

    set(CURRENT_DEPENDENCY Geogram)

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
                --recursive
                https://github.com/BrunoLevy/geogram.git
                "${CMAKE_SOURCE_DIR}/External/${CURRENT_DEPENDENCY}"
                RESULT_VARIABLE GIT_RESULT_${CURRENT_DEPENDENCY}
                OUTPUT_FILE "${CMAKE_SOURCE_DIR}/External/Logs/${CURRENT_DEPENDENCY}/GIT_STDOUT.log"
                ERROR_FILE "${CMAKE_SOURCE_DIR}/External/Logs/${CURRENT_DEPENDENCY}/GIT_STDERR.log"
        )
        if(NOT ${GIT_RESULT_${CURRENT_DEPENDENCY}} EQUAL 0)
            message(FATAL_ERROR "[AliceVision] Failed to fetch ${CURRENT_DEPENDENCY}. Check ${CMAKE_SOURCE_DIR}/External/Logs/${CURRENT_DEPENDENCY}/GIT_STDERR.log for more information.")
        endif()
    endif()

    # Apply patch
    # We need to patch the CMakeLists.txt file
    execute_process(COMMAND ${GIT_EXECUTABLE}
            apply "${CMAKE_SOURCE_DIR}/CMake/Patches/GEOGRAM-oneTBB-FindGeogram.patch"
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
            "-DCMAKE_OSX_ARCHITECTURES=${CMAKE_OSX_ARCHITECTURES}"
            -DCMAKE_PREFIX_PATH=${CMAKE_SOURCE_DIR}/External/Products
            -DBUILD_SHARED_LIBS=ON
            -DGEOGRAM_WITH_GRAPHICS=OFF
            -DGEOGRAM_WITH_LUA=OFF
            -DGEOGRAM_LIB_ONLY=ON
            -DGEOGRAM_WITH_TBB=ON
            -DGEOGRAM_INSTALL_CMAKE_DIR=${CMAKE_SOURCE_DIR}/External/Products/lib/cmake
            -DVORPALINE_BUILD_DYNAMIC=OFF
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

    # F**** we need to fix the install name for geogram and its third party library
    # Get all final dynamic libraries
    file(GLOB DYN_LIBS_SYMLINKS "${CMAKE_SOURCE_DIR}/External/Products/lib/libgeogram*.dylib*")
    set(DYN_LIBS)
    foreach(DYN_LIB IN LISTS DYN_LIBS_SYMLINKS)
        if(NOT IS_SYMLINK "${DYN_LIB}")
            list(APPEND DYN_LIBS "${DYN_LIB}")
        endif()
    endforeach()
    foreach(DYN_LIB IN LISTS DYN_LIBS)
        # Get file name
        get_filename_component(FILE_NAME "${DYN_LIB}" NAME)
        execute_process(
                COMMAND install_name_tool -id
                "@rpath/${FILE_NAME}"
                "${DYN_LIB}"
        )
    endforeach()

    message(STATUS "[AliceVision] Made dependency ${CURRENT_DEPENDENCY} available.")

endmacro()
# Utility to build a simple OpenMP library from source

macro(build_png)

    # NOTE: PNG does not support building a fat binary in a single CMake invocation. Therefore, we build two seperate versions
    # and lipo them together, if necessary.

    set(CURRENT_DEPENDENCY PNG)

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
                https://github.com/pnggroup/libpng.git
                "${CMAKE_SOURCE_DIR}/External/${CURRENT_DEPENDENCY}"
                RESULT_VARIABLE GIT_RESULT_${CURRENT_DEPENDENCY}
                OUTPUT_FILE "${CMAKE_SOURCE_DIR}/External/Logs/${CURRENT_DEPENDENCY}/GIT_STDOUT.log"
                ERROR_FILE "${CMAKE_SOURCE_DIR}/External/Logs/${CURRENT_DEPENDENCY}/GIT_STDERR.log"
        )
        if(NOT ${GIT_RESULT_${CURRENT_DEPENDENCY}} EQUAL 0)
            message(FATAL_ERROR "[AliceVision] Failed to fetch ${CURRENT_DEPENDENCY}. Check ${CMAKE_SOURCE_DIR}/External/Logs/${CURRENT_DEPENDENCY}/GIT_STDERR.log for more information.")
        endif()
    endif()

    # If not APPLE
    if(NOT APPLE OR (NOT "${CMAKE_OSX_ARCHITECTURES}" STREQUAL "arm64;x86_64" AND NOT "${CMAKE_OSX_ARCHITECTURES}" STREQUAL "x86_64;arm64"))

        # Create Build Directory
        file(MAKE_DIRECTORY "${CMAKE_SOURCE_DIR}/External/${CURRENT_DEPENDENCY}/Build")

        # Configure
        message(STATUS "[AliceVision] Configuring ${CURRENT_DEPENDENCY}...")
        execute_process(COMMAND ${CMAKE_COMMAND}
                -DCMAKE_INSTALL_PREFIX=${CMAKE_SOURCE_DIR}/External/Products
                -DCMAKE_BUILD_TYPE=Release
                -DBUILD_SHARED_LIBS=ON
                -DCMAKE_PREFIX_PATH=${CMAKE_SOURCE_DIR}/External/Products
                -DPNG_SHARED=ON
                -DPNG_STATIC=OFF
                -DPNG_FRAMEWORK=OFF
                -DPNG_TESTS=OFF
                -DPNG_TOOLS=OFF
                -DPNG_EXECUTABLES=OFF
                -DPNG_BUILD_ZLIB=OFF
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

    else() # NOT UNIVERSAL

        # Build for each Architecture
        # Note: This will overwrite the library and headers, but that is fine
        foreach(ARCH IN LISTS CMAKE_OSX_ARCHITECTURES)

            # Create Build Directory
            file(MAKE_DIRECTORY "${CMAKE_SOURCE_DIR}/External/${CURRENT_DEPENDENCY}/Build_${ARCH}")

            # Configure
            message(STATUS "[AliceVision] Configuring ${CURRENT_DEPENDENCY}...")
            execute_process(COMMAND ${CMAKE_COMMAND}
                    -DCMAKE_INSTALL_PREFIX=${CMAKE_SOURCE_DIR}/External/Products
                    -DCMAKE_BUILD_TYPE=Release
                    "-DCMAKE_OSX_ARCHITECTURES=${ARCH}"
                    -DBUILD_SHARED_LIBS=ON
                    -DCMAKE_PREFIX_PATH=${CMAKE_SOURCE_DIR}/External/Products
                    -DPNG_SHARED=ON
                    -DPNG_STATIC=OFF
                    -DPNG_FRAMEWORK=OFF
                    -DPNG_TESTS=OFF
                    -DPNG_TOOLS=OFF
                    -DPNG_EXECUTABLES=OFF
                    -DPNG_BUILD_ZLIB=OFF
                    -G "${CMAKE_GENERATOR}"
                    ..
                    WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}/External/${CURRENT_DEPENDENCY}/Build_${ARCH}"
                    RESULT_VARIABLE CONFIGURE_RESULT_${CURRENT_DEPENDENCY}
                    OUTPUT_FILE "${CMAKE_SOURCE_DIR}/External/Logs/${CURRENT_DEPENDENCY}/CMAKE_${ARCH}_STDOUT.log"
                    ERROR_FILE "${CMAKE_SOURCE_DIR}/External/Logs/${CURRENT_DEPENDENCY}/CMAKE_${ARCH}_STDERR.log"
            )
            if(NOT ${CONFIGURE_RESULT_${CURRENT_DEPENDENCY}} EQUAL 0)
                message(FATAL_ERROR "[AliceVision] Failed to configure ${CURRENT_DEPENDENCY}. Check ${CMAKE_SOURCE_DIR}/External/Logs/${CURRENT_DEPENDENCY}/CMAKE_${ARCH}_STDERR.log for more information.")
            endif()

            # Build
            message(STATUS "[AliceVision] Building ${CURRENT_DEPENDENCY}...")
            execute_process(COMMAND ${CMAKE_COMMAND}
                    --build .
                    -j${NCPUs}
                    WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}/External/${CURRENT_DEPENDENCY}/Build_${ARCH}"
                    RESULT_VARIABLE BUILD_RESULT_${CURRENT_DEPENDENCY}
                    OUTPUT_FILE "${CMAKE_SOURCE_DIR}/External/Logs/${CURRENT_DEPENDENCY}/BUILD_${ARCH}_STDOUT.log"
                    ERROR_FILE "${CMAKE_SOURCE_DIR}/External/Logs/${CURRENT_DEPENDENCY}/BUILD_${ARCH}_STDERR.log"
            )
            if(NOT ${BUILD_RESULT_${CURRENT_DEPENDENCY}} EQUAL 0)
                message(FATAL_ERROR "[AliceVision] Failed to build ${CURRENT_DEPENDENCY}. Check ${CMAKE_SOURCE_DIR}/External/Logs/${CURRENT_DEPENDENCY}/BUILD_${ARCH}_STDERR.log for more information.")
            endif()

            # Install (aka copy)
            message(STATUS "[AliceVision] Installing ${CURRENT_DEPENDENCY}...")
            execute_process(COMMAND ${CMAKE_COMMAND}
                    --install .
                    --strip
                    WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}/External/${CURRENT_DEPENDENCY}/Build_${ARCH}"
                    RESULT_VARIABLE INSTALL_RESULT_${CURRENT_DEPENDENCY}
                    OUTPUT_FILE "${CMAKE_SOURCE_DIR}/External/Logs/${CURRENT_DEPENDENCY}/INSTALL_${ARCH}_STDOUT.log"
                    ERROR_FILE "${CMAKE_SOURCE_DIR}/External/Logs/${CURRENT_DEPENDENCY}/INSTALL_${ARCH}_STDERR.log"
            )
            if(NOT ${INSTALL_RESULT_${CURRENT_DEPENDENCY}} EQUAL 0)
                message(FATAL_ERROR "[AliceVision] Failed to install ${CURRENT_DEPENDENCY}. Check ${CMAKE_SOURCE_DIR}/External/Logs/${CURRENT_DEPENDENCY}/INSTALL_${ARCH}_STDERR.log for more information.")
            endif()

        endforeach()

        # Now create a universal binary with lipo
        # Read real path
        execute_process(COMMAND realpath
                "${CMAKE_SOURCE_DIR}/External/${CURRENT_DEPENDENCY}/Build_x86_64/libpng.dylib"
                OUTPUT_VARIABLE PNG_x86_64
        )
        string(STRIP "${PNG_x86_64}" PNG_x86_64)

        execute_process(COMMAND realpath
                "${CMAKE_SOURCE_DIR}/External/${CURRENT_DEPENDENCY}/Build_arm64/libpng.dylib"
                OUTPUT_VARIABLE PNG_AARCH64
        )
        string(STRIP "${PNG_AARCH64}" PNG_AARCH64)

        get_filename_component(PNG_NAME "${PNG_x86_64}" NAME)

        execute_process(COMMAND lipo
                "${PNG_x86_64}"
                "${PNG_AARCH64}"
                -create
                -output "${CMAKE_SOURCE_DIR}/External/Products/lib/${PNG_NAME}"
                WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}/External/${CURRENT_DEPENDENCY}"
                RESULT_VARIABLE INSTALL_RESULT_${CURRENT_DEPENDENCY}
                OUTPUT_FILE "${CMAKE_SOURCE_DIR}/External/Logs/${CURRENT_DEPENDENCY}/LIPO_STDOUT.log"
                ERROR_FILE "${CMAKE_SOURCE_DIR}/External/Logs/${CURRENT_DEPENDENCY}/LIPO_STDERR.log"
        )

    endif() # NOT UNIVERSAL

    # Find openjph-config and set OpenJPH_DIR
    file(GLOB_RECURSE ${CURRENT_DEPENDENCY}_CONFIG "${CMAKE_SOURCE_DIR}/External/Products/lib/cmake/PNGConfig.cmake")
    if(${CURRENT_DEPENDENCY}_CONFIG)
        get_filename_component(SUBFOLDER_PATH "${${CURRENT_DEPENDENCY}_CONFIG}" DIRECTORY)
        # Store in cache
        set("${CURRENT_DEPENDENCY}_DIR" "${SUBFOLDER_PATH}" CACHE PATH "" FORCE)
    else()
        message(FATAL_ERROR "Could not find CMake Configuration file for ${CURRENT_DEPENDENCY}. Please submit a bug report.")
    endif()

    message(STATUS "[AliceVision] Made dependency ${CURRENT_DEPENDENCY} available.")

endmacro()
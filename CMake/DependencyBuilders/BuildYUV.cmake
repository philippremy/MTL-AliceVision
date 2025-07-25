# Utility to build a simple OpenMP library from source

macro(build_yuv)

    set(CURRENT_DEPENDENCY YUV)

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
                https://chromium.googlesource.com/libyuv/libyuv
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

        # We need to patch the CMakeLists.txt file
        execute_process(COMMAND ${GIT_EXECUTABLE}
                apply "${CMAKE_SOURCE_DIR}/CMake/Patches/YUV-universal-binary-detection.patch"
                WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}/External/${CURRENT_DEPENDENCY}"
                RESULT_VARIABLE PATCH_RESULT_${CURRENT_DEPENDENCY}
                OUTPUT_FILE "${CMAKE_SOURCE_DIR}/External/Logs/${CURRENT_DEPENDENCY}/PATCH_STDOUT.log"
                ERROR_FILE "${CMAKE_SOURCE_DIR}/External/Logs/${CURRENT_DEPENDENCY}/PATCH_STDERR.log"
        )
        if(NOT ${PATCH_RESULT_${CURRENT_DEPENDENCY}} EQUAL 0)
            # DO NOT FAIL. Probably already applied.
            # message(FATAL_ERROR "[AliceVision] Failed to patch ${CURRENT_DEPENDENCY}. Check ${CMAKE_SOURCE_DIR}/External/Logs/${CURRENT_DEPENDENCY}/PATCH_STDERR.log for more information.")
        endif()

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

        # Now create a universal binary with lipo (here for dynamic and static library)

        # Dynamic
        execute_process(COMMAND realpath
                "${CMAKE_SOURCE_DIR}/External/${CURRENT_DEPENDENCY}/Build_x86_64/libyuv.dylib"
                OUTPUT_VARIABLE YUV_x86_64
        )
        string(STRIP "${YUV_x86_64}" YUV_x86_64)

        execute_process(COMMAND realpath
                "${CMAKE_SOURCE_DIR}/External/${CURRENT_DEPENDENCY}/Build_arm64/libyuv.dylib"
                OUTPUT_VARIABLE YUV_AARCH64
        )
        string(STRIP "${YUV_AARCH64}" YUV_AARCH64)

        get_filename_component(YUV_NAME "${YUV_x86_64}" NAME)

        execute_process(COMMAND lipo
                "${YUV_x86_64}"
                "${YUV_AARCH64}"
                -create
                -output "${CMAKE_SOURCE_DIR}/External/Products/lib/${YUV_NAME}"
                WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}/External/${CURRENT_DEPENDENCY}"
                RESULT_VARIABLE INSTALL_RESULT_${CURRENT_DEPENDENCY}
                OUTPUT_FILE "${CMAKE_SOURCE_DIR}/External/Logs/${CURRENT_DEPENDENCY}/LIPO_DYNAMIC_STDOUT.log"
                ERROR_FILE "${CMAKE_SOURCE_DIR}/External/Logs/${CURRENT_DEPENDENCY}/LIPO_DYNAMIC_STDERR.log"
        )

        # Static
        execute_process(COMMAND realpath
                "${CMAKE_SOURCE_DIR}/External/${CURRENT_DEPENDENCY}/Build_x86_64/libyuv.a"
                OUTPUT_VARIABLE YUV_x86_64
        )
        string(STRIP "${YUV_x86_64}" YUV_x86_64)

        execute_process(COMMAND realpath
                "${CMAKE_SOURCE_DIR}/External/${CURRENT_DEPENDENCY}/Build_arm64/libyuv.a"
                OUTPUT_VARIABLE YUV_AARCH64
        )
        string(STRIP "${YUV_AARCH64}" YUV_AARCH64)

        get_filename_component(YUV_NAME "${YUV_x86_64}" NAME)

        execute_process(COMMAND lipo
                "${YUV_x86_64}"
                "${YUV_AARCH64}"
                -create
                -output "${CMAKE_SOURCE_DIR}/External/Products/lib/${YUV_NAME}"
                WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}/External/${CURRENT_DEPENDENCY}"
                RESULT_VARIABLE INSTALL_RESULT_${CURRENT_DEPENDENCY}
                OUTPUT_FILE "${CMAKE_SOURCE_DIR}/External/Logs/${CURRENT_DEPENDENCY}/LIPO_STATIC_STDOUT.log"
                ERROR_FILE "${CMAKE_SOURCE_DIR}/External/Logs/${CURRENT_DEPENDENCY}/LIPO_STATIC_STDERR.log"
        )

    endif() # NOT UNIVERSAL

    # Find openjph-config and set OpenJPH_DIR
    # LibYUV has no Config File.
    # Set _LIBYUV_INCLUDEDIR and _LIBYUV_LIBDIR
    set(_LIBYUV_INCLUDEDIR "${CMAKE_SOURCE_DIR}/External/Products/include/libyuv" CACHE PATH "" FORCE)
    set(_LIBYUV_LIBDIR "${CMAKE_SOURCE_DIR}/External/Products/lib" CACHE PATH "" FORCE)

    message(STATUS "[AliceVision] Made dependency ${CURRENT_DEPENDENCY} available.")

endmacro()
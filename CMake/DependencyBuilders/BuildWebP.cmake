# Utility to build a simple OpenMP library from source

macro(build_webp)

    set(CURRENT_DEPENDENCY WebP)

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
                https://chromium.googlesource.com/webm/libwebp
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
                "-DCMAKE_OSX_ARCHITECTURES=${CMAKE_OSX_ARCHITECTURES}"
                -DBUILD_SHARED_LIBS=ON
                -DWEBP_BUILD_ANIM_UTILS=OFF
                -DWEBP_BUILD_CWEBP=OFF
                -DWEBP_BUILD_DWEBP=OFF
                -DWEBP_BUILD_GIF2WEBP=OFF
                -DWEBP_BUILD_IMG2WEBP=OFF
                -DWEBP_BUILD_VWEBP=OFF
                -DWEBP_BUILD_WEBPINFO=OFF
                -DWEBP_BUILD_LIBWEBPMUX=ON
                -DWEBP_BUILD_WEBPMUX=OFF
                -DWEBP_BUILD_EXTRAS=OFF
                -DWEBP_BUILD_WEBP_JS=OFF
                -DWEBP_BUILD_FUZZTEST=OFF
                -DWEBP_USE_THREAD=ON
                -DWEBP_NEAR_LOSSLESS=ON
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
                    -DWEBP_BUILD_ANIM_UTILS=OFF
                    -DWEBP_BUILD_CWEBP=OFF
                    -DWEBP_BUILD_DWEBP=OFF
                    -DWEBP_BUILD_GIF2WEBP=OFF
                    -DWEBP_BUILD_IMG2WEBP=OFF
                    -DWEBP_BUILD_VWEBP=OFF
                    -DWEBP_BUILD_WEBPINFO=OFF
                    -DWEBP_BUILD_LIBWEBPMUX=ON
                    -DWEBP_BUILD_WEBPMUX=OFF
                    -DWEBP_BUILD_EXTRAS=OFF
                    -DWEBP_BUILD_WEBP_JS=OFF
                    -DWEBP_BUILD_FUZZTEST=OFF
                    -DWEBP_USE_THREAD=ON
                    -DWEBP_NEAR_LOSSLESS=ON
                    -G "${CMAKE_GENERATOR}"
                    ..
                    WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}/External/${CURRENT_DEPENDENCY}/Build_${ARCH}"
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
                    WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}/External/${CURRENT_DEPENDENCY}/Build_${ARCH}"
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
                    WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}/External/${CURRENT_DEPENDENCY}/Build_${ARCH}"
                    RESULT_VARIABLE INSTALL_RESULT_${CURRENT_DEPENDENCY}
                    OUTPUT_FILE "${CMAKE_SOURCE_DIR}/External/Logs/${CURRENT_DEPENDENCY}/INSTALL_STDOUT.log"
                    ERROR_FILE "${CMAKE_SOURCE_DIR}/External/Logs/${CURRENT_DEPENDENCY}/INSTALL_STDERR.log"
            )
            if(NOT ${INSTALL_RESULT_${CURRENT_DEPENDENCY}} EQUAL 0)
                message(FATAL_ERROR "[AliceVision] Failed to install ${CURRENT_DEPENDENCY}. Check ${CMAKE_SOURCE_DIR}/External/Logs/${CURRENT_DEPENDENCY}/INSTALL_STDERR.log for more information.")
            endif()

        endforeach()

        # Now create a universal binary with lipo

        # Three libraries
        # 1. libwebp
        # 2. libwebpdecoder
        # 3. libwebpdemux
        # 4. libwebpmux
        # 5. libsharpyuv

        # Read real path
        execute_process(COMMAND realpath
                "${CMAKE_SOURCE_DIR}/External/${CURRENT_DEPENDENCY}/Build_x86_64/libwebp.dylib"
                OUTPUT_VARIABLE WEBP_x86_64
        )
        string(STRIP "${WEBP_x86_64}" WEBP_x86_64)

        execute_process(COMMAND realpath
                "${CMAKE_SOURCE_DIR}/External/${CURRENT_DEPENDENCY}/Build_arm64/libwebp.dylib"
                OUTPUT_VARIABLE WEBP_AARCH64
        )
        string(STRIP "${WEBP_AARCH64}" WEBP_AARCH64)

        get_filename_component(WEBP_NAME "${WEBP_x86_64}" NAME)

        execute_process(COMMAND lipo
                "${WEBP_x86_64}"
                "${WEBP_AARCH64}"
                -create
                -output "${CMAKE_SOURCE_DIR}/External/Products/lib/${WEBP_NAME}"
                WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}/External/${CURRENT_DEPENDENCY}"
                RESULT_VARIABLE INSTALL_RESULT_${CURRENT_DEPENDENCY}
                OUTPUT_FILE "${CMAKE_SOURCE_DIR}/External/Logs/${CURRENT_DEPENDENCY}/LIPO_STDOUT.log"
                ERROR_FILE "${CMAKE_SOURCE_DIR}/External/Logs/${CURRENT_DEPENDENCY}/LIPO_STDERR.log"
        )

        # Read real path
        execute_process(COMMAND realpath
                "${CMAKE_SOURCE_DIR}/External/${CURRENT_DEPENDENCY}/Build_x86_64/libwebpdecoder.dylib"
                OUTPUT_VARIABLE WEBP_x86_64
        )
        string(STRIP "${WEBP_x86_64}" WEBP_x86_64)

        execute_process(COMMAND realpath
                "${CMAKE_SOURCE_DIR}/External/${CURRENT_DEPENDENCY}/Build_arm64/libwebpdecoder.dylib"
                OUTPUT_VARIABLE WEBP_AARCH64
        )
        string(STRIP "${WEBP_AARCH64}" WEBP_AARCH64)

        get_filename_component(WEBP_NAME "${WEBP_x86_64}" NAME)

        execute_process(COMMAND lipo
                "${WEBP_x86_64}"
                "${WEBP_AARCH64}"
                -create
                -output "${CMAKE_SOURCE_DIR}/External/Products/lib/${WEBP_NAME}"
                WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}/External/${CURRENT_DEPENDENCY}"
                RESULT_VARIABLE INSTALL_RESULT_${CURRENT_DEPENDENCY}
                OUTPUT_FILE "${CMAKE_SOURCE_DIR}/External/Logs/${CURRENT_DEPENDENCY}/LIPO_STDOUT.log"
                ERROR_FILE "${CMAKE_SOURCE_DIR}/External/Logs/${CURRENT_DEPENDENCY}/LIPO_STDERR.log"
        )

        # Read real path
        execute_process(COMMAND realpath
                "${CMAKE_SOURCE_DIR}/External/${CURRENT_DEPENDENCY}/Build_x86_64/libwebpdemux.dylib"
                OUTPUT_VARIABLE WEBP_x86_64
        )
        string(STRIP "${WEBP_x86_64}" WEBP_x86_64)

        execute_process(COMMAND realpath
                "${CMAKE_SOURCE_DIR}/External/${CURRENT_DEPENDENCY}/Build_arm64/libwebpdemux.dylib"
                OUTPUT_VARIABLE WEBP_AARCH64
        )
        string(STRIP "${WEBP_AARCH64}" WEBP_AARCH64)

        get_filename_component(WEBP_NAME "${WEBP_x86_64}" NAME)

        execute_process(COMMAND lipo
                "${WEBP_x86_64}"
                "${WEBP_AARCH64}"
                -create
                -output "${CMAKE_SOURCE_DIR}/External/Products/lib/${WEBP_NAME}"
                WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}/External/${CURRENT_DEPENDENCY}"
                RESULT_VARIABLE INSTALL_RESULT_${CURRENT_DEPENDENCY}
                OUTPUT_FILE "${CMAKE_SOURCE_DIR}/External/Logs/${CURRENT_DEPENDENCY}/LIPO_STDOUT.log"
                ERROR_FILE "${CMAKE_SOURCE_DIR}/External/Logs/${CURRENT_DEPENDENCY}/LIPO_STDERR.log"
        )

        # Read real path
        execute_process(COMMAND realpath
                "${CMAKE_SOURCE_DIR}/External/${CURRENT_DEPENDENCY}/Build_x86_64/libwebpmux.dylib"
                OUTPUT_VARIABLE WEBP_x86_64
        )
        string(STRIP "${WEBP_x86_64}" WEBP_x86_64)

        execute_process(COMMAND realpath
                "${CMAKE_SOURCE_DIR}/External/${CURRENT_DEPENDENCY}/Build_arm64/libwebpmux.dylib"
                OUTPUT_VARIABLE WEBP_AARCH64
        )
        string(STRIP "${WEBP_AARCH64}" WEBP_AARCH64)

        get_filename_component(WEBP_NAME "${WEBP_x86_64}" NAME)

        execute_process(COMMAND lipo
                "${WEBP_x86_64}"
                "${WEBP_AARCH64}"
                -create
                -output "${CMAKE_SOURCE_DIR}/External/Products/lib/${WEBP_NAME}"
                WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}/External/${CURRENT_DEPENDENCY}"
                RESULT_VARIABLE INSTALL_RESULT_${CURRENT_DEPENDENCY}
                OUTPUT_FILE "${CMAKE_SOURCE_DIR}/External/Logs/${CURRENT_DEPENDENCY}/LIPO_STDOUT.log"
                ERROR_FILE "${CMAKE_SOURCE_DIR}/External/Logs/${CURRENT_DEPENDENCY}/LIPO_STDERR.log"
        )

        # Read real path
        execute_process(COMMAND realpath
                "${CMAKE_SOURCE_DIR}/External/${CURRENT_DEPENDENCY}/Build_x86_64/libsharpyuv.dylib"
                OUTPUT_VARIABLE WEBP_x86_64
        )
        string(STRIP "${WEBP_x86_64}" WEBP_x86_64)

        execute_process(COMMAND realpath
                "${CMAKE_SOURCE_DIR}/External/${CURRENT_DEPENDENCY}/Build_arm64/libsharpyuv.dylib"
                OUTPUT_VARIABLE WEBP_AARCH64
        )
        string(STRIP "${WEBP_AARCH64}" WEBP_AARCH64)

        get_filename_component(WEBP_NAME "${WEBP_x86_64}" NAME)

        execute_process(COMMAND lipo
                "${WEBP_x86_64}"
                "${WEBP_AARCH64}"
                -create
                -output "${CMAKE_SOURCE_DIR}/External/Products/lib/${WEBP_NAME}"
                WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}/External/${CURRENT_DEPENDENCY}"
                RESULT_VARIABLE INSTALL_RESULT_${CURRENT_DEPENDENCY}
                OUTPUT_FILE "${CMAKE_SOURCE_DIR}/External/Logs/${CURRENT_DEPENDENCY}/LIPO_STDOUT.log"
                ERROR_FILE "${CMAKE_SOURCE_DIR}/External/Logs/${CURRENT_DEPENDENCY}/LIPO_STDERR.log"
        )

    endif() # NOT UNIVERSAL

    # Find openjph-config and set OpenJPH_DIR
    file(GLOB_RECURSE ${CURRENT_DEPENDENCY}_CONFIG "${CMAKE_SOURCE_DIR}/External/Products/share/WebP/cmake/WebPConfig.cmake")
    if(${CURRENT_DEPENDENCY}_CONFIG)
        get_filename_component(SUBFOLDER_PATH "${${CURRENT_DEPENDENCY}_CONFIG}" DIRECTORY)
        # Store in cache
        set("${CURRENT_DEPENDENCY}_DIR" "${SUBFOLDER_PATH}" CACHE PATH "" FORCE)
    else()
        message(FATAL_ERROR "Could not find CMake Configuration file for ${CURRENT_DEPENDENCY}. Please submit a bug report.")
    endif()

    message(STATUS "[AliceVision] Made dependency ${CURRENT_DEPENDENCY} available.")

endmacro()
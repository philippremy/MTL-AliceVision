
# Utility to build a simple OpenMP library from source

macro(build_aom)

    set(CURRENT_DEPENDENCY AOM)

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
                https://aomedia.googlesource.com/aom
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

        # Build for each Architecture
        # Note: This will overwrite the library and headers, but that is fine
        foreach(ARCH IN LISTS CMAKE_OSX_ARCHITECTURES)

            # Create Build Directory
            file(MAKE_DIRECTORY "${CMAKE_SOURCE_DIR}/External/${CURRENT_DEPENDENCY}/Build_${ARCH}")

            if("${ARCH}" STREQUAL arm64)
                # Configure
                message(STATUS "[AliceVision] Configuring ${CURRENT_DEPENDENCY}...")
                execute_process(COMMAND ${CMAKE_COMMAND}
                        -DCMAKE_INSTALL_PREFIX=${CMAKE_SOURCE_DIR}/External/Products
                        -DCMAKE_BUILD_TYPE=Release
                        "-DCMAKE_OSX_ARCHITECTURES=${ARCH}"
                        -DBUILD_SHARED_LIBS=ON
                        -DCMAKE_PREFIX_PATH=${CMAKE_SOURCE_DIR}/External/Products
                        "-DCMAKE_TOOLCHAIN_FILE=${CMAKE_SOURCE_DIR}/External/${CURRENT_DEPENDENCY}/build/cmake/toolchains/arm64-macos.cmake"
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
            else() # ARCH == arm64
                # Configure
                message(STATUS "[AliceVision] Configuring ${CURRENT_DEPENDENCY}...")
                execute_process(COMMAND ${CMAKE_COMMAND}
                        -DCMAKE_INSTALL_PREFIX=${CMAKE_SOURCE_DIR}/External/Products
                        -DCMAKE_BUILD_TYPE=Release
                        "-DCMAKE_OSX_ARCHITECTURES=${ARCH}"
                        -DBUILD_SHARED_LIBS=ON
                        -DCMAKE_PREFIX_PATH=${CMAKE_SOURCE_DIR}/External/Products
                        "-DCMAKE_TOOLCHAIN_FILE=${CMAKE_SOURCE_DIR}/External/${CURRENT_DEPENDENCY}/build/cmake/toolchains/x86_64-macos.cmake"
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
            endif() # ARCH == arm64

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
                "${CMAKE_SOURCE_DIR}/External/${CURRENT_DEPENDENCY}/Build_x86_64/libaom.dylib"
                OUTPUT_VARIABLE AOM_x86_64
        )
        string(STRIP "${AOM_x86_64}" AOM_x86_64)

        execute_process(COMMAND realpath
                "${CMAKE_SOURCE_DIR}/External/${CURRENT_DEPENDENCY}/Build_arm64/libaom.dylib"
                OUTPUT_VARIABLE AOM_AARCH64
        )
        string(STRIP "${AOM_AARCH64}" AOM_AARCH64)

        get_filename_component(AOM_NAME "${AOM_x86_64}" NAME)

        execute_process(COMMAND lipo
                "${AOM_x86_64}"
                "${AOM_AARCH64}"
                -create
                -output "${CMAKE_SOURCE_DIR}/External/Products/lib/${AOM_NAME}"
                WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}/External/${CURRENT_DEPENDENCY}"
                RESULT_VARIABLE INSTALL_RESULT_${CURRENT_DEPENDENCY}
                OUTPUT_FILE "${CMAKE_SOURCE_DIR}/External/Logs/${CURRENT_DEPENDENCY}/LIPO_DYNAMIC_STDOUT.log"
                ERROR_FILE "${CMAKE_SOURCE_DIR}/External/Logs/${CURRENT_DEPENDENCY}/LIPO_DYNAMIC_STDERR.log"
        )

        # Static
        execute_process(COMMAND realpath
                "${CMAKE_SOURCE_DIR}/External/${CURRENT_DEPENDENCY}/Build_x86_64/libaom.a"
                OUTPUT_VARIABLE AOM_x86_64
        )
        string(STRIP "${AOM_x86_64}" AOM_x86_64)

        execute_process(COMMAND realpath
                "${CMAKE_SOURCE_DIR}/External/${CURRENT_DEPENDENCY}/Build_arm64/libaom.a"
                OUTPUT_VARIABLE AOM_AARCH64
        )
        string(STRIP "${AOM_AARCH64}" AOM_AARCH64)

        get_filename_component(AOM_NAME "${AOM_x86_64}" NAME)

        execute_process(COMMAND lipo
                "${AOM_x86_64}"
                "${AOM_AARCH64}"
                -create
                -output "${CMAKE_SOURCE_DIR}/External/Products/lib/${AOM_NAME}"
                WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}/External/${CURRENT_DEPENDENCY}"
                RESULT_VARIABLE INSTALL_RESULT_${CURRENT_DEPENDENCY}
                OUTPUT_FILE "${CMAKE_SOURCE_DIR}/External/Logs/${CURRENT_DEPENDENCY}/LIPO_STATIC_STDOUT.log"
                ERROR_FILE "${CMAKE_SOURCE_DIR}/External/Logs/${CURRENT_DEPENDENCY}/LIPO_STATIC_STDERR.log"
        )

    endif() # NOT UNIVERSAL


    # AOM has a Config File, but AVIF finds it differently. So we use it and set the exotic stuff seperatly
    # Set _LIBYUV_INCLUDEDIR and _LIBYUV_LIBDIR
    file(GLOB_RECURSE ${CURRENT_DEPENDENCY}_CONFIG "${CMAKE_SOURCE_DIR}/External/Products/lib/AOMConfig.cmake")
    if(${CURRENT_DEPENDENCY}_CONFIG)
        get_filename_component(SUBFOLDER_PATH "${${CURRENT_DEPENDENCY}_CONFIG}" DIRECTORY)
        # Store in cache
        set("${CURRENT_DEPENDENCY}_DIR" "${SUBFOLDER_PATH}" CACHE PATH "" FORCE)
    else()
        message(FATAL_ERROR "Could not find CMake Configuration file for ${CURRENT_DEPENDENCY}. Please submit a bug report.")
    endif()
    set(_LIBAOM_INCLUDEDIR "${CMAKE_SOURCE_DIR}/External/Products/include/aom" CACHE PATH "" FORCE)
    set(_LIBAOM_LIBDIR "${CMAKE_SOURCE_DIR}/External/Products/lib" CACHE PATH "" FORCE)

    message(STATUS "[AliceVision] Made dependency ${CURRENT_DEPENDENCY} available.")

endmacro()
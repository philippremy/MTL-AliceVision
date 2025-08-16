# Utility for building a libJPEG from source

macro(build_jpeg)

    set(CURRENT_DEPENDENCY JPEG)

    # Create Logging Folder
    file(MAKE_DIRECTORY "${CMAKE_SOURCE_DIR}/External/Logs/${CURRENT_DEPENDENCY}")

    # Create Source Folder
    file(MAKE_DIRECTORY "${CMAKE_SOURCE_DIR}/External/${CURRENT_DEPENDENCY}")

    # Fetch libJPEG Source
    message(STATUS "[AliceVision] Fetching ${CURRENT_DEPENDENCY}...")
    file(DOWNLOAD
            https://jpegclub.org/reference/wp-content/uploads/2024/01/jpegsrc.v9f.tar.gz
            "${CMAKE_SOURCE_DIR}/External/${CURRENT_DEPENDENCY}/jpegsrc.v9f.tar.gz"
            EXPECTED_HASH MD5=9ca58d68febb0fa9c1c087045b9a5483
    )
    file(ARCHIVE_EXTRACT
            INPUT "${CMAKE_SOURCE_DIR}/External/${CURRENT_DEPENDENCY}/jpegsrc.v9f.tar.gz"
            DESTINATION "${CMAKE_SOURCE_DIR}/External/${CURRENT_DEPENDENCY}"
    )

    # On Apple and Windows, use a small CMakeLists.txt wrapper
    if(APPLE OR WIN32)
        # Use custom CMakeLists.txt file
        # Copy the CMakeLists.txt
        file(COPY "${CMAKE_SOURCE_DIR}/CMake/ExternalBridges/JPEG/CMakeLists.txt" DESTINATION "${CMAKE_SOURCE_DIR}/External/${CURRENT_DEPENDENCY}/jpeg-9f")
        # Copy the header
        file(COPY "${CMAKE_SOURCE_DIR}/CMake/ExternalBridges/JPEG/jconfig.h" DESTINATION "${CMAKE_SOURCE_DIR}/External/${CURRENT_DEPENDENCY}/jpeg-9f")

        # Configure
        message(STATUS "[AliceVision] Configuring ${CURRENT_DEPENDENCY}...")
        execute_process(COMMAND ${CMAKE_COMMAND}
                -DCMAKE_INSTALL_PREFIX=${CMAKE_SOURCE_DIR}/External/Products
                -DCMAKE_BUILD_TYPE=Release
                "-DCMAKE_OSX_ARCHITECTURES=${CMAKE_OSX_ARCHITECTURES}"
                -DBUILD_SHARED_LIBS=ON
                -G "${CMAKE_GENERATOR}"
                WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}/External/${CURRENT_DEPENDENCY}/jpeg-9f"
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
                WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}/External/${CURRENT_DEPENDENCY}/jpeg-9f"
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
                WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}/External/${CURRENT_DEPENDENCY}/jpeg-9f"
                RESULT_VARIABLE INSTALL_RESULT_${CURRENT_DEPENDENCY}
                OUTPUT_FILE "${CMAKE_SOURCE_DIR}/External/Logs/${CURRENT_DEPENDENCY}/INSTALL_STDOUT.log"
                ERROR_FILE "${CMAKE_SOURCE_DIR}/External/Logs/${CURRENT_DEPENDENCY}/INSTALL_STDERR.log"
        )
        if(NOT ${INSTALL_RESULT_${CURRENT_DEPENDENCY}} EQUAL 0)
            message(FATAL_ERROR "[AliceVision] Failed to install ${CURRENT_DEPENDENCY}. Check ${CMAKE_SOURCE_DIR}/External/Logs/${CURRENT_DEPENDENCY}/INSTALL_STDERR.log for more information.")
        endif()
    elseif(UNIX AND NOT APPLE)
        # Use configure script
        # Configure
        message(STATUS "[AliceVision] Configuring ${CURRENT_DEPENDENCY}...")
        execute_process(COMMAND "${CMAKE_SOURCE_DIR}/External/${CURRENT_DEPENDENCY}/jpeg-9f/configure"
                --prefix=${CMAKE_SOURCE_DIR}/External/Products
                WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}/External/${CURRENT_DEPENDENCY}/jpeg-9f"
                RESULT_VARIABLE CONFIGURE_RESULT_${CURRENT_DEPENDENCY}
                OUTPUT_FILE "${CMAKE_SOURCE_DIR}/External/Logs/${CURRENT_DEPENDENCY}/CONFIGURE_STDOUT.log"
                ERROR_FILE "${CMAKE_SOURCE_DIR}/External/Logs/${CURRENT_DEPENDENCY}/CONFIGURE_STDERR.log"
        )
        if(NOT ${CONFIGURE_RESULT_${CURRENT_DEPENDENCY}} EQUAL 0)
            message(FATAL_ERROR "[AliceVision] Failed to configure ${CURRENT_DEPENDENCY}. Check ${CMAKE_SOURCE_DIR}/External/Logs/${CURRENT_DEPENDENCY}/CONFIGURE_STDERR.log for more information.")
        endif()

        # Build
        message(STATUS "[AliceVision] Building ${CURRENT_DEPENDENCY}...")
        execute_process(COMMAND make
                -j${NCPUs}
                WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}/External/${CURRENT_DEPENDENCY}/jpeg-9f"
                RESULT_VARIABLE BUILD_RESULT_${CURRENT_DEPENDENCY}
                OUTPUT_FILE "${CMAKE_SOURCE_DIR}/External/Logs/${CURRENT_DEPENDENCY}/BUILD_STDOUT.log"
                ERROR_FILE "${CMAKE_SOURCE_DIR}/External/Logs/${CURRENT_DEPENDENCY}/BUILD_STDERR.log"
        )
        if(NOT ${BUILD_RESULT_${CURRENT_DEPENDENCY}} EQUAL 0)
            message(FATAL_ERROR "[AliceVision] Failed to build ${CURRENT_DEPENDENCY}. Check ${CMAKE_SOURCE_DIR}/External/Logs/${CURRENT_DEPENDENCY}/BUILD_STDERR.log for more information.")
        endif()

        # Install (aka copy)
        message(STATUS "[AliceVision] Installing ${CURRENT_DEPENDENCY}...")
        execute_process(COMMAND make
                install
                WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}/External/${CURRENT_DEPENDENCY}/jpeg-9f"
                RESULT_VARIABLE INSTALL_RESULT_${CURRENT_DEPENDENCY}
                OUTPUT_FILE "${CMAKE_SOURCE_DIR}/External/Logs/${CURRENT_DEPENDENCY}/INSTALL_STDOUT.log"
                ERROR_FILE "${CMAKE_SOURCE_DIR}/External/Logs/${CURRENT_DEPENDENCY}/INSTALL_STDERR.log"
        )
        if(NOT ${INSTALL_RESULT_${CURRENT_DEPENDENCY}} EQUAL 0)
            message(FATAL_ERROR "[AliceVision] Failed to install ${CURRENT_DEPENDENCY}. Check ${CMAKE_SOURCE_DIR}/External/Logs/${CURRENT_DEPENDENCY}/INSTALL_STDERR.log for more information.")
        endif()
    else()
        message(FATAL_ERROR "[AliceVision] Cannot build JPEG. No known build procedure for this OS.")
    endif()

endmacro()
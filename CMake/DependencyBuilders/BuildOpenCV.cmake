# Utility to build a simple OpenMP library from source

macro(build_opencv)

    set(CURRENT_DEPENDENCY OpenCV)

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
                https://github.com/opencv/opencv.git
                "${CMAKE_SOURCE_DIR}/External/${CURRENT_DEPENDENCY}"
                RESULT_VARIABLE GIT_RESULT_${CURRENT_DEPENDENCY}
                OUTPUT_FILE "${CMAKE_SOURCE_DIR}/External/Logs/${CURRENT_DEPENDENCY}/GIT_STDOUT.log"
                ERROR_FILE "${CMAKE_SOURCE_DIR}/External/Logs/${CURRENT_DEPENDENCY}/GIT_STDERR.log"
        )
        if(NOT ${GIT_RESULT_${CURRENT_DEPENDENCY}} EQUAL 0)
            message(FATAL_ERROR "[AliceVision] Failed to fetch ${CURRENT_DEPENDENCY}. Check ${CMAKE_SOURCE_DIR}/External/Logs/${CURRENT_DEPENDENCY}/GIT_STDERR.log for more information.")
        endif()
        # Fetch Contrib Module
        execute_process(COMMAND ${GIT_EXECUTABLE}
                clone --depth 1
                https://github.com/opencv/opencv_contrib.git
                "${CMAKE_SOURCE_DIR}/External/${CURRENT_DEPENDENCY}/opencv_contrib"
                RESULT_VARIABLE GIT_RESULT_${CURRENT_DEPENDENCY}
                OUTPUT_FILE "${CMAKE_SOURCE_DIR}/External/Logs/${CURRENT_DEPENDENCY}/GIT_CONTRIB_STDOUT.log"
                ERROR_FILE "${CMAKE_SOURCE_DIR}/External/Logs/${CURRENT_DEPENDENCY}/GIT_CONTRIB_STDERR.log"
        )
        if(NOT ${GIT_RESULT_${CURRENT_DEPENDENCY}} EQUAL 0)
            message(FATAL_ERROR "[AliceVision] Failed to fetch ${CURRENT_DEPENDENCY}. Check ${CMAKE_SOURCE_DIR}/External/Logs/${CURRENT_DEPENDENCY}/GIT__CONTRIB_STDERR.log for more information.")
        endif()
    endif()

    # We need to patch the Logging.h file, because glog is missing an import here
    execute_process(COMMAND ${GIT_EXECUTABLE}
            apply "${CMAKE_SOURCE_DIR}/CMake/Patches/OPENCV_CONTRIB-fix-glog-export.patch"
            WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}/External/${CURRENT_DEPENDENCY}/opencv_contrib"
            RESULT_VARIABLE PATCH_RESULT_${CURRENT_DEPENDENCY}
            OUTPUT_FILE "${CMAKE_SOURCE_DIR}/External/Logs/${CURRENT_DEPENDENCY}/PATCH_STDOUT.log"
            ERROR_FILE "${CMAKE_SOURCE_DIR}/External/Logs/${CURRENT_DEPENDENCY}/PATCH_STDERR.log"
    )
    if(NOT ${PATCH_RESULT_${CURRENT_DEPENDENCY}} EQUAL 0)
        # DO NOT FAIL. Probably already applied.
        # message(FATAL_ERROR "[AliceVision] Failed to patch ${CURRENT_DEPENDENCY}. Check ${CMAKE_SOURCE_DIR}/External/Logs/${CURRENT_DEPENDENCY}/PATCH_STDERR.log for more information.")
    endif()

    # Generate the list of modules to build
    string(REPLACE ";" "," OPENCV_MODULE_LIST "${ALICEVISION_OPENCV_COMPONENTS}")

    if(NOT APPLE OR (NOT "${CMAKE_OSX_ARCHITECTURES}" STREQUAL "arm64;x86_64" AND NOT "${CMAKE_OSX_ARCHITECTURES}" STREQUAL "x86_64;arm64"))

        # Create Build Directory
        file(MAKE_DIRECTORY "${CMAKE_SOURCE_DIR}/External/${CURRENT_DEPENDENCY}/Build")

        # Configure
        message(STATUS "[AliceVision] Configuring ${CURRENT_DEPENDENCY}...")
        execute_process(COMMAND ${CMAKE_COMMAND}
                -DCMAKE_INSTALL_PREFIX=${CMAKE_SOURCE_DIR}/External/Products
                -DCMAKE_BUILD_TYPE=Release
                -DCMAKE_CXX_STANDARD=20
                -DCMAKE_CXX_STANDARD_REQUIRED=ON
                -DCMAKE_PREFIX_PATH=${CMAKE_SOURCE_DIR}/External/Products
                -DBUILD_SHARED_LIBS=ON
                -DBUILD_opencv_apps=OFF
                -DBUILD_opencv_js=OFF
                -DBUILD_ANDROID_PROJECTS=OFF
                -DBUILD_ANDROID_EXAMPLE=OFF
                -DBUILD_DOCS=OFF
                -DBUILD_EXAMPLES=OFF
                -DBUILD_PACKAGE=OFF
                -DBUILD_PERF_TESTS=OFF
                -DBUILD_TESTS=OFF
                -DBUILD_ZLIB=OFF
                -DBUILD_TIFF=OFF
                -DBUILD_OPENJPEG=OFF
                -DBUILD_JASPER=OFF
                -DBUILD_JPEG=OFF
                -DBUILD_PNG=OFF
                -DBUILD_OPENEXR=OFF
                -DBUILD_WEBP=OFF
                -DBUILD_TBB=OFF
                -DBUILD_IPP_IW=OFF
                -DBUILD_ITT=OFF
                -DWITH_VTK=OFF
                -DWITH_EIGEN=ON
                -DWITH_IPP=OFF
                -DWITH_JASPER=OFF
                -DWITH_OPENMP=OFF # Causes Linker Issues on macOS!
                -DWITH_LAPACK=ON
                -DWITH_ITT=OFF
                -DWITH_PROTOBUF=OFF
                -DENABLE_LTO=OFF
                -DENABLE_THIN_LTO=ON
                -DCV_TRACE=OFF
                "-DOPENCV_EXTRA_MODULES_PATH='../opencv_contrib/modules'"
                "-DBUILD_LIST=${OPENCV_MODULE_LIST}"
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

            if("${ARCH}" STREQUAL "arm64")

                # Configure
                message(STATUS "[AliceVision] Configuring ${CURRENT_DEPENDENCY}...")
                execute_process(COMMAND ${CMAKE_COMMAND}
                        -DCMAKE_INSTALL_PREFIX=${CMAKE_SOURCE_DIR}/External/Products
                        -DCMAKE_BUILD_TYPE=Release
                        -DCMAKE_CXX_STANDARD=20
                        -DCMAKE_CXX_STANDARD_REQUIRED=ON
                        -DCMAKE_PREFIX_PATH=${CMAKE_SOURCE_DIR}/External/Products
                        -DBUILD_SHARED_LIBS=ON
                        "-DCMAKE_OSX_ARCHITECTURES=${ARCH}"
                        -DOPENCV_SKIP_SYSTEM_PROCESSOR_DETECTION=ON
                        -DAARCH64=ON
                        -DOpenCV_ARCH=ARM64
                        -DBUILD_opencv_apps=OFF
                        -DBUILD_opencv_js=OFF
                        -DBUILD_ANDROID_PROJECTS=OFF
                        -DBUILD_ANDROID_EXAMPLE=OFF
                        -DBUILD_DOCS=OFF
                        -DBUILD_EXAMPLES=OFF
                        -DBUILD_PACKAGE=OFF
                        -DBUILD_PERF_TESTS=OFF
                        -DBUILD_TESTS=OFF
                        -DBUILD_ZLIB=OFF
                        -DBUILD_TIFF=OFF
                        -DBUILD_OPENJPEG=OFF
                        -DBUILD_JASPER=OFF
                        -DBUILD_JPEG=OFF
                        -DBUILD_PNG=OFF
                        -DBUILD_OPENEXR=OFF
                        -DBUILD_WEBP=OFF
                        -DBUILD_TBB=OFF
                        -DBUILD_IPP_IW=OFF
                        -DBUILD_ITT=OFF
                        -DWITH_VTK=OFF
                        -DWITH_EIGEN=ON
                        -DWITH_IPP=OFF
                        -DWITH_JASPER=OFF
                        -DWITH_OPENMP=OFF
                        -DWITH_LAPACK=ON
                        -DWITH_ITT=OFF
                        -DWITH_PROTOBUF=OFF
                        -DENABLE_LTO=OFF
                        -DENABLE_THIN_LTO=ON
                        -DCV_TRACE=OFF
                        "-DOPENCV_EXTRA_MODULES_PATH='../opencv_contrib/modules'"
                        "-DBUILD_LIST=${OPENCV_MODULE_LIST}"
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

            else()

                # Configure
                message(STATUS "[AliceVision] Configuring ${CURRENT_DEPENDENCY}...")
                execute_process(COMMAND ${CMAKE_COMMAND}
                        -DCMAKE_INSTALL_PREFIX=${CMAKE_SOURCE_DIR}/External/Products
                        -DCMAKE_BUILD_TYPE=Release
                        -DCMAKE_CXX_STANDARD=20
                        -DCMAKE_CXX_STANDARD_REQUIRED=ON
                        -DCMAKE_PREFIX_PATH=${CMAKE_SOURCE_DIR}/External/Products
                        -DBUILD_SHARED_LIBS=ON
                        "-DCMAKE_OSX_ARCHITECTURES=${ARCH}"
                        -DOPENCV_SKIP_SYSTEM_PROCESSOR_DETECTION=ON
                        -DX86_64=ON
                        -DOpenCV_ARCH=x64
                        -DBUILD_opencv_apps=OFF
                        -DBUILD_opencv_js=OFF
                        -DBUILD_ANDROID_PROJECTS=OFF
                        -DBUILD_ANDROID_EXAMPLE=OFF
                        -DBUILD_DOCS=OFF
                        -DBUILD_EXAMPLES=OFF
                        -DBUILD_PACKAGE=OFF
                        -DBUILD_PERF_TESTS=OFF
                        -DBUILD_TESTS=OFF
                        -DBUILD_ZLIB=OFF
                        -DBUILD_TIFF=OFF
                        -DBUILD_OPENJPEG=OFF
                        -DBUILD_JASPER=OFF
                        -DBUILD_JPEG=OFF
                        -DBUILD_PNG=OFF
                        -DBUILD_OPENEXR=OFF
                        -DBUILD_WEBP=OFF
                        -DBUILD_TBB=OFF
                        -DBUILD_IPP_IW=OFF
                        -DBUILD_ITT=OFF
                        -DWITH_VTK=OFF
                        -DWITH_EIGEN=ON
                        -DWITH_IPP=OFF
                        -DWITH_JASPER=OFF
                        -DWITH_OPENMP=OFF
                        -DWITH_LAPACK=ON
                        -DWITH_ITT=OFF
                        -DWITH_PROTOBUF=OFF
                        -DENABLE_LTO=OFF
                        -DENABLE_THIN_LTO=ON
                        -DCV_TRACE=OFF
                        "-DOPENCV_EXTRA_MODULES_PATH='../opencv_contrib/modules'"
                        "-DBUILD_LIST=${OPENCV_MODULE_LIST}"
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

    endif() # NOT UNIVERSAL

    # Use lipo to fuse
    foreach(MODULE IN LISTS ALICEVISION_OPENCV_COMPONENTS)

        execute_process(COMMAND realpath
                "${CMAKE_SOURCE_DIR}/External/${CURRENT_DEPENDENCY}/Build_x86_64/lib/libopencv_${MODULE}.dylib"
                OUTPUT_VARIABLE OPENCV_x86_64
        )
        string(STRIP "${OPENCV_x86_64}" OPENCV_x86_64)

        execute_process(COMMAND realpath
                "${CMAKE_SOURCE_DIR}/External/${CURRENT_DEPENDENCY}/Build_arm64/lib/libopencv_${MODULE}.dylib"
                OUTPUT_VARIABLE OPENCV_AARCH64
        )
        string(STRIP "${OPENCV_AARCH64}" OPENCV_AARCH64)

        get_filename_component(OPENCV_NAME "${OPENCV_x86_64}" NAME)

        execute_process(COMMAND lipo
                "${OPENCV_x86_64}"
                "${OPENCV_AARCH64}"
                -create
                -output "${CMAKE_SOURCE_DIR}/External/Products/lib/${OPENCV_NAME}"
                WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}/External/${CURRENT_DEPENDENCY}"
                RESULT_VARIABLE INSTALL_RESULT_${CURRENT_DEPENDENCY}
                OUTPUT_FILE "${CMAKE_SOURCE_DIR}/External/Logs/${CURRENT_DEPENDENCY}/LIPO_${MODULE}_DYNAMIC_STDOUT.log"
                ERROR_FILE "${CMAKE_SOURCE_DIR}/External/Logs/${CURRENT_DEPENDENCY}/LIPO_${MODULE}_DYNAMIC_STDERR.log"
        )

    endforeach()

    # Fixup the third party modules manually
    file(GLOB STATIC_TP_MODULES "${CMAKE_SOURCE_DIR}/External/${CURRENT_DEPENDENCY}/Build_x86_64/lib/libopencv.sfm.*.a")
    message(STATUS "STATIC_MODS: ${STATIC_TP_MODULES}")
    foreach(STATIC_MODULE IN LISTS STATIC_TP_MODULES)
        get_filename_component(OPENCV_NAME "${STATIC_MODULE}" NAME)

        execute_process(COMMAND lipo
                "${CMAKE_SOURCE_DIR}/External/${CURRENT_DEPENDENCY}/Build_x86_64/lib/${OPENCV_NAME}"
                "${CMAKE_SOURCE_DIR}/External/${CURRENT_DEPENDENCY}/Build_arm64/lib/${OPENCV_NAME}"
                -create
                -output "${CMAKE_SOURCE_DIR}/External/Products/lib/opencv4/3rdparty/${OPENCV_NAME}"
                WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}/External/${CURRENT_DEPENDENCY}"
                RESULT_VARIABLE INSTALL_RESULT_${CURRENT_DEPENDENCY}
                OUTPUT_FILE "${CMAKE_SOURCE_DIR}/External/Logs/${CURRENT_DEPENDENCY}/LIPO_${STATIC_MODULE}_DYNAMIC_STDOUT.log"
                ERROR_FILE "${CMAKE_SOURCE_DIR}/External/Logs/${CURRENT_DEPENDENCY}/LIPO_${STATIC_MODULE}_DYNAMIC_STDERR.log"
        )
    endforeach()

    # Find openjph-config and set OpenJPH_DIR
    file(GLOB_RECURSE ${CURRENT_DEPENDENCY}_CONFIG "${CMAKE_SOURCE_DIR}/External/Products/lib/cmake/OpenCVConfig.cmake")
    if(${CURRENT_DEPENDENCY}_CONFIG)
        get_filename_component(SUBFOLDER_PATH "${${CURRENT_DEPENDENCY}_CONFIG}" DIRECTORY)
        # Store in cache
        set("${CURRENT_DEPENDENCY}_DIR" "${SUBFOLDER_PATH}" CACHE PATH "" FORCE)
    else()
        message(FATAL_ERROR "Could not find CMake Configuration file for ${CURRENT_DEPENDENCY}. Please submit a bug report.")
    endif()

    message(STATUS "[AliceVision] Made dependency ${CURRENT_DEPENDENCY} available.")

endmacro()
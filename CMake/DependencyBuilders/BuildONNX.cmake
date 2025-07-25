# Utility to build a simple OpenMP library from source

macro(build_onnxruntime)

    set(CURRENT_DEPENDENCY ONNX)

    # Create Logging Folder
    file(MAKE_DIRECTORY "${CMAKE_SOURCE_DIR}/External/Logs/${CURRENT_DEPENDENCY}")

    set(DEPENDENCY_DIR "${CMAKE_SOURCE_DIR}/External/${CURRENT_DEPENDENCY}")
    set(DEPENDENCY_LOG_DIR "${CMAKE_SOURCE_DIR}/External/Logs/${CURRENT_DEPENDENCY}")
    set(DEPENDENCY_GIT_HEAD "${DEPENDENCY_DIR}/.git/HEAD")

    set(ONNX_EXTRACT_PATH)
    # Match OS and architecture as well as whether GPU acceleration should be used
    if(APPLE)
        if(ALICEVISION_USE_ONNX_GPU)
            message(WARNING "[AliceVision] Using GPU acceleration for ONNX Runtime on macOS is unsupported! The setting will forcibly turned OFF!")
            set(ALICEVISION_USE_ONNX_GPU OFF FORCE CACHE BOOL "Use ONNX GPU acceleration")
        endif()
        # Fetch
        if("${CMAKE_OSX_ARCHITECTURES}" STREQUAL "arm64;x86_64" OR "${CMAKE_OSX_ARCHITECTURES}" STREQUAL "x86_64;arm64")
            file(DOWNLOAD
                    https://github.com/microsoft/onnxruntime/releases/download/v1.22.0/onnxruntime-osx-universal2-1.22.0.tgz
                    ${CMAKE_SOURCE_DIR}/External/${CURRENT_DEPENDENCY}/onnxruntime.tgz
            )
            file(ARCHIVE_EXTRACT
                    INPUT ${CMAKE_SOURCE_DIR}/External/${CURRENT_DEPENDENCY}/onnxruntime.tgz
                    DESTINATION ${CMAKE_SOURCE_DIR}/External/${CURRENT_DEPENDENCY}
            )
            set(ONNX_EXTRACT_PATH "${CMAKE_SOURCE_DIR}/External/${CURRENT_DEPENDENCY}/onnxruntime-osx-universal2-1.22.0")
        elseif("${CMAKE_SYSTEM_PROCESSOR}" STREQUAL "arm64")
            file(DOWNLOAD
                    https://github.com/microsoft/onnxruntime/releases/download/v1.22.0/onnxruntime-osx-arm64-1.22.0.tgz
                    ${CMAKE_SOURCE_DIR}/External/${CURRENT_DEPENDENCY}/onnxruntime.tgz
            )
            file(ARCHIVE_EXTRACT
                    INPUT ${CMAKE_SOURCE_DIR}/External/${CURRENT_DEPENDENCY}/onnxruntime.tgz
                    DESTINATION ${CMAKE_SOURCE_DIR}/External/${CURRENT_DEPENDENCY}
            )
            set(ONNX_EXTRACT_PATH "${CMAKE_SOURCE_DIR}/External/${CURRENT_DEPENDENCY}/onnxruntime-osx-arm64-1.22.0")
        elseif("${CMAKE_SYSTEM_PROCESSOR}" STREQUAL "x86_64")
            file(DOWNLOAD
                    https://github.com/microsoft/onnxruntime/releases/download/v1.22.0/onnxruntime-osx-x86_64-1.22.0.tgz
                    ${CMAKE_SOURCE_DIR}/External/${CURRENT_DEPENDENCY}/onnxruntime.tgz
            )
            file(ARCHIVE_EXTRACT
                    INPUT ${CMAKE_SOURCE_DIR}/External/${CURRENT_DEPENDENCY}/onnxruntime.tgz
                    DESTINATION ${CMAKE_SOURCE_DIR}/External/${CURRENT_DEPENDENCY}
            )
            set(ONNX_EXTRACT_PATH "${CMAKE_SOURCE_DIR}/External/${CURRENT_DEPENDENCY}/onnxruntime-osx-x86_64-1.22.0")
        else()
            message(FATAL_ERROR "[AliceVision] Unknown target architecture for APPLE. Please make sure that CMAKE_SYSTEM_PROCESSOR is set to a valid value.")
        endif()
        # Copy/Install
        file(COPY "${ONNX_EXTRACT_PATH}/lib" DESTINATION "${CMAKE_SOURCE_DIR}/External/Products")
        file(COPY "${ONNX_EXTRACT_PATH}/include" DESTINATION "${CMAKE_SOURCE_DIR}/External/Products/include/onnxruntime")
    elseif(LINUX)
        # Fetch
        if("${CMAKE_SYSTEM_PROCESSOR}" STREQUAL "arm64"
                OR "${CMAKE_SYSTEM_PROCESSOR}" STREQUAL "aarch64"
                OR "${CMAKE_SYSTEM_PROCESSOR}" STREQUAL "aarch64_be"
                OR "${CMAKE_SYSTEM_PROCESSOR}" STREQUAL "armv8b"
                OR "${CMAKE_SYSTEM_PROCESSOR}" STREQUAL "armv8l"
        )
            if(ALICEVISION_USE_ONNX_GPU)
                message(WARNING "[AliceVision] Using GPU acceleration for ONNX Runtime on Linux with target architecture aarch64 is unsupported! The setting will forcibly turned OFF!")
                set(ALICEVISION_USE_ONNX_GPU OFF FORCE CACHE BOOL "Use ONNX GPU acceleration")
            endif()
            file(DOWNLOAD
                    https://github.com/microsoft/onnxruntime/releases/download/v1.22.0/onnxruntime-linux-aarch64-1.22.0.tgz
                    ${CMAKE_SOURCE_DIR}/External/${CURRENT_DEPENDENCY}/onnxruntime.tgz
            )
            file(ARCHIVE_EXTRACT
                    INPUT ${CMAKE_SOURCE_DIR}/External/${CURRENT_DEPENDENCY}/onnxruntime.tgz
                    DESTINATION ${CMAKE_SOURCE_DIR}/External/${CURRENT_DEPENDENCY}
            )
            set(ONNX_EXTRACT_PATH "${CMAKE_SOURCE_DIR}/External/${CURRENT_DEPENDENCY}/onnxruntime-linux-aarch64-1.22.0")
        elseif("${CMAKE_SYSTEM_PROCESSOR}" STREQUAL "x86_64" OR "${CMAKE_SYSTEM_PROCESSOR}" STREQUAL "x64")
            if(ALICEVISION_USE_ONNX_GPU)
                file(DOWNLOAD
                        https://github.com/microsoft/onnxruntime/releases/download/v1.22.0/onnxruntime-linux-x64-gpu-1.22.0.tgz
                        ${CMAKE_SOURCE_DIR}/External/${CURRENT_DEPENDENCY}/onnxruntime.tgz
                )
                file(ARCHIVE_EXTRACT
                        INPUT ${CMAKE_SOURCE_DIR}/External/${CURRENT_DEPENDENCY}/onnxruntime.tgz
                        DESTINATION ${CMAKE_SOURCE_DIR}/External/${CURRENT_DEPENDENCY}
                )
                set(ONNX_EXTRACT_PATH "${CMAKE_SOURCE_DIR}/External/${CURRENT_DEPENDENCY}/onnxruntime-linux-x64-gpu-1.22.0")
            else()
                file(DOWNLOAD
                        https://github.com/microsoft/onnxruntime/releases/download/v1.22.0/onnxruntime-linux-x64-1.22.0.tgz
                        ${CMAKE_SOURCE_DIR}/External/${CURRENT_DEPENDENCY}/onnxruntime.tgz
                )
                file(ARCHIVE_EXTRACT
                        INPUT ${CMAKE_SOURCE_DIR}/External/${CURRENT_DEPENDENCY}/onnxruntime.tgz
                        DESTINATION ${CMAKE_SOURCE_DIR}/External/${CURRENT_DEPENDENCY}
                )
                set(ONNX_EXTRACT_PATH "${CMAKE_SOURCE_DIR}/External/${CURRENT_DEPENDENCY}/onnxruntime-linux-x64-1.22.0")
            endif()
        else()
            message(FATAL_ERROR "[AliceVision] Unsupported target architecture for LINUX.")
        endif()
        # Copy/Install
        file(COPY "${ONNX_EXTRACT_PATH}/lib" DESTINATION "${CMAKE_SOURCE_DIR}/External/Products")
        file(COPY "${ONNX_EXTRACT_PATH}/include" DESTINATION "${CMAKE_SOURCE_DIR}/External/Products/include/onnxruntime")
    elseif(WIN32)
        # Fetch
        if("${CMAKE_SYSTEM_PROCESSOR}" STREQUAL "AMD64"
                OR "${CMAKE_SYSTEM_PROCESSOR}" STREQUAL "IA64"
        )
            if(ALICEVISION_USE_ONNX_GPU)
                file(DOWNLOAD
                        https://github.com/microsoft/onnxruntime/releases/download/v1.22.0/onnxruntime-win-x64-gpu-1.22.0.zip
                        ${CMAKE_SOURCE_DIR}/External/${CURRENT_DEPENDENCY}/onnxruntime.tgz
                )
                file(ARCHIVE_EXTRACT
                        INPUT ${CMAKE_SOURCE_DIR}/External/${CURRENT_DEPENDENCY}/onnxruntime.tgz
                        DESTINATION ${CMAKE_SOURCE_DIR}/External/${CURRENT_DEPENDENCY}
                )
                set(ONNX_EXTRACT_PATH "${CMAKE_SOURCE_DIR}/External/${CURRENT_DEPENDENCY}/onnxruntime-win-x64-gpu-1.22.0")
            else()
                file(DOWNLOAD
                        https://github.com/microsoft/onnxruntime/releases/download/v1.22.0/onnxruntime-win-x64-1.22.0.zip
                        ${CMAKE_SOURCE_DIR}/External/${CURRENT_DEPENDENCY}/onnxruntime.tgz
                )
                file(ARCHIVE_EXTRACT
                        INPUT ${CMAKE_SOURCE_DIR}/External/${CURRENT_DEPENDENCY}/onnxruntime.tgz
                        DESTINATION ${CMAKE_SOURCE_DIR}/External/${CURRENT_DEPENDENCY}
                )
                set(ONNX_EXTRACT_PATH "${CMAKE_SOURCE_DIR}/External/${CURRENT_DEPENDENCY}/onnxruntime-win-x64-1.22.0")
            endif()
        elseif("${CMAKE_SYSTEM_PROCESSOR}" STREQUAL "X86")
            if(ALICEVISION_USE_ONNX_GPU)
                message(WARNING "[AliceVision] Using GPU acceleration for ONNX Runtime on Windows with target architecture x86 (32-Bit) is unsupported! The setting will forcibly turned OFF!")
                set(ALICEVISION_USE_ONNX_GPU OFF FORCE CACHE BOOL "Use ONNX GPU acceleration")
            endif()
            file(DOWNLOAD
                    https://github.com/microsoft/onnxruntime/releases/download/v1.22.0/onnxruntime-win-x86-1.22.0.zip
                    ${CMAKE_SOURCE_DIR}/External/${CURRENT_DEPENDENCY}/onnxruntime.tgz
            )
            file(ARCHIVE_EXTRACT
                    INPUT ${CMAKE_SOURCE_DIR}/External/${CURRENT_DEPENDENCY}/onnxruntime.tgz
                    DESTINATION ${CMAKE_SOURCE_DIR}/External/${CURRENT_DEPENDENCY}
            )
            set(ONNX_EXTRACT_PATH "${CMAKE_SOURCE_DIR}/External/${CURRENT_DEPENDENCY}/onnxruntime-win-x86-1.22.0")
        elseif("${CMAKE_SYSTEM_PROCESSOR}" STREQUAL "ARM64")
            if(ALICEVISION_USE_ONNX_GPU)
                message(WARNING "[AliceVision] Using GPU acceleration for ONNX Runtime on Windows with target architecture arm64 is unsupported! The setting will forcibly turned OFF!")
                set(ALICEVISION_USE_ONNX_GPU OFF FORCE CACHE BOOL "Use ONNX GPU acceleration")
            endif()
            file(DOWNLOAD
                    https://github.com/microsoft/onnxruntime/releases/download/v1.22.0/onnxruntime-win-arm64-1.22.0.zip
                    ${CMAKE_SOURCE_DIR}/External/${CURRENT_DEPENDENCY}/onnxruntime.tgz
            )
            file(ARCHIVE_EXTRACT
                    INPUT ${CMAKE_SOURCE_DIR}/External/${CURRENT_DEPENDENCY}/onnxruntime.tgz
                    DESTINATION ${CMAKE_SOURCE_DIR}/External/${CURRENT_DEPENDENCY}
            )
            set(ONNX_EXTRACT_PATH "${CMAKE_SOURCE_DIR}/External/${CURRENT_DEPENDENCY}/onnxruntime-win-arm64-1.22.0")
        else()
            message(FATAL_ERROR "[AliceVision] Unsupported target architecture for WIN32.")
        endif()
        # Copy/Install
        file(COPY "${ONNX_EXTRACT_PATH}/lib" DESTINATION "${CMAKE_SOURCE_DIR}/External/Products")
        file(COPY "${ONNX_EXTRACT_PATH}/include" DESTINATION "${CMAKE_SOURCE_DIR}/External/Products/include/onnxruntime")
    else()
        message(FATAL_ERROR "[AliceVision] Fetching ONNX Runtime for the current target OS is unsupported! Supported OSes are Linux, macOS and Windows. Please set ALICEVISION_USE_ONNX to OFF and configure again!")
    endif()

    message(STATUS "[AliceVision] Made dependency ${CURRENT_DEPENDENCY} available.")

endmacro()
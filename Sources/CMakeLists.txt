###===================================================
###
### AliceVision Versioning
###
###===================================================
file(STRINGS "${CMAKE_SOURCE_DIR}/Sources/Headers/AV/version.hpp" _ALICEVISION_VERSION_HPP_CONTENTS REGEX "#define ALICEVISION_VERSION_")
foreach (v MAJOR MINOR REVISION)
    if ("${_ALICEVISION_VERSION_HPP_CONTENTS}" MATCHES "#define ALICEVISION_VERSION_${v} ([0-9]+)")
        set(ALICEVISION_VERSION_${v} "${CMAKE_MATCH_1}")
    else()
        message(FATAL_ERROR "[AliceVision] Failed to retrieve the AliceVision version the source code. Missing ALICEVISION_VERSION_${v}.")
    endif()
endforeach()
set(ALICEVISION_VERSION ${ALICEVISION_VERSION_MAJOR}.${ALICEVISION_VERSION_MINOR}.${ALICEVISION_VERSION_REVISION})
message(STATUS "[AliceVision] The global AliceVision Project version is ${ALICEVISION_VERSION}.")

###===================================================
###
### Dependency Handling
###
###===================================================
include(HandleDependency)
include(AppleClangOpenMP)

# Set the CMAKE_PREFIX_PATH to the External Build Products
list(APPEND CMAKE_PREFIX_PATH "${CMAKE_SOURCE_DIR}/External/Products")

# Set the CMake Module Path to the FindHelpers
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/CMake/FindHelpers")
# And add the build directory for any dependencies which install FindModules instead of Configs (Geogram ^^)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/External/Products/lib/cmake")

# CUDA
set(ALICEVISION_HAVE_CUDA 0)
if(ALICEVISION_USE_CUDA)
    # TODO
    set(ALICEVISION_HAVE_CUDA 1)
endif()

# OpenMP
set(ALICEVISION_HAVE_OPENMP 0)
if(ALICEVISION_USE_OPENMP)
    # Handle Apple Clang (the find_package() Module fails here)
    if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "AppleClang")
        find_package(OpenMP QUIET COMPONENTS C CXX)
        if(NOT OpenMP_FOUND)
            include(DependencyBuilders/BuildDependency)
            build_dependency(OpenMP)
            message(STATUS "[AliceVision] Fixing AppleClang OpenMP support.")
            alicevision_fixup_appleclang_openmp()
        else()
            message(STATUS "[AliceVision] Found dependency OpenMP.")
        endif()
    else()
        alicevision_handle_dependency(OpenMP REQUIRED COMPONENTS C CXX)
    endif()
    set(ALICEVISION_HAVE_OPENMP 1)
endif()

# Boost
set(ALICEVISION_HAVE_BOOST 0)
set(ALICEVISION_BOOST_COMPONENTS atomic container date_time graph json log log_setup program_options regex serialization system thread timer)
if(ALICEVISION_BUILD_TESTS)
    list(APPEND ALICEVISION_BOOST_COMPONENTS unit_test_framework)
endif()
if(ALICEVISION_USE_CCTAG)
    list(APPEND ALICEVISION_BOOST_COMPONENTS accumulators chrono filesystem math multi_array ptr_container stacktrace)
endif()
alicevision_handle_dependency(Boost 1.76 CONFIG REQUIRED COMPONENTS ${ALICEVISION_BOOST_COMPONENTS})
set(ALICEVISION_HAVE_BOOST 1)

# Eigen3
set(ALICEVISION_HAVE_EIGEN3 0)
alicevision_handle_dependency(Eigen3 3 REQUIRED)
set(ALICEVISION_HAVE_EIGEN3 1)

# LAPACK
set(ALICEVISION_HAVE_LAPACK 0)
set(ALICEVISION_HAVE_BLAS 0)
alicevision_handle_dependency(LAPACK 3.12 REQUIRED COMPONENTS BLAS LAPACK)
set(ALICEVISION_HAVE_LAPACK 1)
set(ALICEVISION_HAVE_BLAS 1)

# Imath
set(ALICEVISION_HAVE_IMATH 0)
alicevision_handle_dependency(Imath REQUIRED)
set(ALICEVISION_HAVE_IMATH 1)

# OpenJPH
set(ALICEVISION_HAVE_OPENJPH 0)
alicevision_handle_dependency(openjph REQUIRED)
set(ALICEVISION_HAVE_OPENJPH 1)

# OpenEXR
set(ALICEVISION_HAVE_OPENEXR 0)
alicevision_handle_dependency(OpenEXR REQUIRED)
set(ALICEVISION_HAVE_OPENEXR 1)

# TIFF
set(ALICEVISION_HAVE_TIFF 0)
alicevision_handle_dependency(TIFF REQUIRED)
set(ALICEVISION_HAVE_TIFF 1)

# JPEG
set(ALICEVISION_HAVE_JPEG 0)
alicevision_handle_dependency(JPEG REQUIRED)
set(ALICEVISION_HAVE_JPEG 1)

# ZLIB and minizip
set(ALICEVISION_HAVE_ZLIB 0)
alicevision_handle_dependency(ZLIB REQUIRED COMPONENTS)
set(ALICEVISION_HAVE_ZLIB 1)

# PNG
set(ALICEVISION_HAVE_PNG 0)
alicevision_handle_dependency(PNG REQUIRED)
set(ALICEVISION_HAVE_PNG 1)

# Expat
set(ALICEVISION_HAVE_EXPAT 0)
alicevision_handle_dependency(EXPAT REQUIRED)
set(ALICEVISION_HAVE_EXPAT 1)

# YAML-CPP
set(ALICEVISION_HAVE_YAML-CPP 0)
alicevision_handle_dependency(yaml-cpp REQUIRED)
set(ALICEVISION_HAVE_YAML-CPP 1)

# PyString
set(ALICEVISION_HAVE_PYSTRING 0)
alicevision_handle_dependency(pystring REQUIRED)
set(ALICEVISION_HAVE_PYSTRING 1)

# OpenColorIO
set(ALICEVISION_HAVE_OPENCOLORIO 0)
alicevision_handle_dependency(OpenColorIO REQUIRED)
set(ALICEVISION_HAVE_OPENCOLORIO 1)

# OpenImageIO
set(ALICEVISION_HAVE_OPENIMAGEIO 0)
alicevision_handle_dependency(OpenImageIO REQUIRED)
set(ALICEVISION_HAVE_OPENIMAGEIO 1)

# YUV
set(ALICEVISION_HAVE_YUV 0)
alicevision_handle_dependency(libyuv REQUIRED)
set(ALICEVISION_HAVE_YUV 1)

# AOM
set(ALICEVISION_HAVE_AOM 0)
alicevision_handle_dependency(AOM REQUIRED)
set(ALICEVISION_HAVE_AOM 1)

# AVIF
set(ALICEVISION_HAVE_AVIF 0)
alicevision_handle_dependency(libavif REQUIRED)
set(ALICEVISION_HAVE_AVIF 1)

# GFLAGS
set(ALICEVISION_HAVE_GFLAGS 0)
alicevision_handle_dependency(gflags REQUIRED)
set(ALICEVISION_HAVE_GFLAGS 1)

# GLOG
set(ALICEVISION_HAVE_GLOG 0)
alicevision_handle_dependency(glog REQUIRED)
set(ALICEVISION_HAVE_GLOG 1)

# OpenJPEG
set(ALICEVISION_HAVE_OPENJPEG 0)
alicevision_handle_dependency(OpenJPEG REQUIRED)
set(ALICEVISION_HAVE_OPENJPEG 1)

# WebP
set(ALICEVISION_HAVE_WEBP 0)
alicevision_handle_dependency(WebP REQUIRED)
set(ALICEVISION_HAVE_WEBP 1)

# Abseil
set(ALICEVISION_HAVE_ABSEIL 0)
alicevision_handle_dependency(absl REQUIRED)
set(ALICEVISION_HAVE_ABSEIL 1)

# Suitesparse
set(ALICEVISION_HAVE_SUITESPARSE 0)
alicevision_handle_dependency(SuiteSparse 4.5.6 REQUIRED COMPONENTS CHOLMOD SPQR)
set(ALICEVISION_HAVE_SUITESPARSE 1)

# Ceres
set(ALICEVISION_HAVE_CERES 0)
alicevision_handle_dependency(Ceres REQUIRED)
set(ALICEVISION_HAVE_CERES 1)

# OpenCV
set(ALICEVISION_HAVE_OPENCV 0)
if(ALICEVISION_USE_OPENCV)
    set(ALICEVISION_OPENCV_COMPONENTS core calib3d dnn features2d flann highgui imgcodecs imgproc mcc optflow photo sfm video videoio xfeatures2d ximgproc)
    alicevision_handle_dependency(OpenCV REQUIRED COMPONENTS ${ALICEVISION_OPENCV_COMPONENTS})
    # There appears to be a bug somewhere, so call this again manually!
    find_package(OpenCV REQUIRED COMPONENTS ${ALICEVISION_OPENCV_COMPONENTS})
    set(ALICEVISION_HAVE_OPENCV 1)
endif()

# ONNX Runtime (and GPU)
set(ALICEVISION_HAVE_ONNX 0)
set(ALICEVISION_HAVE_ONNX_GPU 0)
if(ALICEVISION_USE_ONNX)
    alicevision_handle_dependency(onnxruntime REQUIRED)
    set(ALICEVISION_HAVE_ONNX 1)
    if(ALICEVISION_USE_ONNX_GPU)
        set(ALICEVISION_HAVE_ONNX_GPU 1)
    endif()
endif()

# CoinUtils
set(ALICEVISION_HAVE_COINUTILS 0)
alicevision_handle_dependency(CoinUtils REQUIRED)
set(ALICEVISION_HAVE_COINUTILS 1)

# Osi
set(ALICEVISION_HAVE_OSI 0)
alicevision_handle_dependency(Osi REQUIRED)
set(ALICEVISION_HAVE_OSI 1)

# Clp
set(ALICEVISION_HAVE_CLP 0)
alicevision_handle_dependency(Clp REQUIRED)
set(ALICEVISION_HAVE_CLP 1)

# SoPLEX
set(ALICEVISION_HAVE_SOPLEX 0)
alicevision_handle_dependency(SOPLEX REQUIRED)
set(ALICEVISION_HAVE_SOPLEX 1)

# LEMON
set(ALICEVISION_HAVE_LEMON 0)
alicevision_handle_dependency(LEMON REQUIRED)
set(ALICEVISION_HAVE_LEMON 1)

# oneTBB
set(ALICEVISION_HAVE_ONETBB 0)
alicevision_handle_dependency(TBB REQUIRED)
set(ALICEVISION_HAVE_ONETBB 1)

# CCTag
set(ALICEVISION_HAVE_CCTAG 0)
if(ALICEVISION_USE_CCTAG)
    alicevision_handle_dependency(CCTag REQUIRED)
    set(ALICEVISION_HAVE_CCTAG 1)
endif()

# AprilTag
set(ALICEVISION_HAVE_APRILTAG 0)
if(ALICEVISION_USE_APRILTAG)
    alicevision_handle_dependency(apriltag REQUIRED)
    set(ALICEVISION_HAVE_APRILTAG 1)
endif()

# PopSIFT
set(ALICEVISION_HAVE_POPSIFT 0)
if(ALICEVISION_USE_POPSIFT)
    alicevision_handle_dependency(PopSift REQUIRED)
    set(ALICEVISION_HAVE_POPSIFT 1)
endif()

# LZ4
set(ALICEVISION_HAVE_LZ4 0)
alicevision_handle_dependency(lz4 REQUIRED)
set(ALICEVISION_HAVE_LZ4 1)

# Flann
set(ALICEVISION_HAVE_FLANN 0)
alicevision_handle_dependency(flann REQUIRED)
set(ALICEVISION_HAVE_FLANN 1)

# nanoflann
set(ALICEVISION_HAVE_NANOFLANN 0)
alicevision_handle_dependency(nanoflann REQUIRED)
set(ALICEVISION_HAVE_NANOFLANN 1)

# OpenMesh
set(ALICEVISION_HAVE_OPENMESH 0)
alicevision_handle_dependency(OpenMesh REQUIRED)
set(ALICEVISION_HAVE_OPENMESH 1)

# Geogram
set(ALICEVISION_HAVE_GEOGRAM 0)
alicevision_handle_dependency(Geogram REQUIRED)
set(ALICEVISION_HAVE_GEOGRAM 1)

# Assimp
set(ALICEVISION_HAVE_ASSIMP 0)
alicevision_handle_dependency(assimp REQUIRED)
set(ALICEVISION_HAVE_ASSIMP 1)

# Alembic
set(ALICEVISION_HAVE_ALEMBIC 0)
if(ALICEVISION_USE_ALEMBIC)
    alicevision_handle_dependency(Alembic REQUIRED)
    set(ALICEVISION_HAVE_ALEMBIC 1)
endif()

# OpenGV
set(ALICEVISION_HAVE_OPENGV 0)
if(ALICEVISION_USE_OPENGV)
    alicevision_handle_dependency(opengv REQUIRED)
    set(ALICEVISION_HAVE_OPENGV 1)
endif()

# Xerces-C++
set(ALICEVISION_HAVE_XERCESCPP 0)
alicevision_handle_dependency(XercesC 3.2 REQUIRED)
set(ALICEVISION_HAVE_XERCESCPP 1)

# e57Format
set(ALICEVISION_HAVE_E57FORMAT 0)
if(ALICEVISION_USE_E57FORMAT)
    alicevision_handle_dependency(E57Format REQUIRED)
    set(ALICEVISION_HAVE_E57FORMAT 1)
endif()

# Metal
set(ALICEVISION_HAVE_METAL 0)
if(APPLE AND ALICEVISION_USE_METAL)
    list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/CMake/ThirdParty/CMake-Metal")
    include(MetalShaderSupport)
    include(CheckLanguage)
    check_language(Metal)
    if(CMAKE_Metal_COMPILER)
        enable_language(Metal)
        include_directories("${CMAKE_SOURCE_DIR}/ThirdParty/metal-cpp")
    else()
        message(WARNING "[AliceVision] No Metal compiler was found on this system! Metal GPU acceleration will forcibly be turned OFF. You may try setting CMAKE_Metal_COMPILER to the path of the 'metal' binary.")
    endif()
    set(ALICEVISION_HAVE_METAL 1)
endif()

# Vulkan
set(ALICEVISION_HAVE_VULKAN 0)
set(ALICEVISION_HAVE_VMA 0)
set(ALICEVISION_HAVE_FP16 0)
if(ALICEVISION_USE_VULKAN)
    if(APPLE)
        find_package(Vulkan QUIET COMPONENTS glslangValidator MoltenVK)
    else()
        find_package(Vulkan QUIET COMPONENTS glslangValidator)
    endif()
    if(NOT Vulkan_FOUND)
        message(FATAL_ERROR "[AliceVision] A path to a valid Vulkan SDK MUST be provided manually. This cannot be build from source. Please set the VULKAN_SDK environment variable to help finding it.")
    else()
        message(STATUS "[AliceVision] Found dependency Vulkan (${Vulkan_VERSION}).")
    endif()
    alicevision_handle_dependency(VulkanMemoryAllocator REQUIRED)
    alicevision_handle_dependency(fp16 REQUIRED)
    set(ALICEVISION_HAVE_VULKAN 1)
    set(ALICEVISION_HAVE_VMA 1)
    set(ALICEVISION_HAVE_FP16 1)
endif()

# Qt
set(ALICEVISION_HAVE_QT6 0)
if(ALICEVISION_BUILD_MESHROOM_QT_PLUGINS OR ALICEVISION_BUILD_GUI)
    find_package(Qt6 COMPONENTS 3DCore 3DExtras 3DRender Core Svg QuickControls2 Quick Qml Charts Gui ShaderTools REQUIRED)
    set(ALICEVISION_HAVE_QT6 1)
endif()

# TODO: CUDA; uncertaintyTE; MAGMA; MOSEK

###===================================================
###
### Dependency Summary
###
###===================================================
include(Utilities/PrintDependency)
string(ASCII 27 Esc)
message("\n┌───────────── Dependency Summary ───────────────┐")
message("├ Required Dependencies:${Esc}[50G│")
alicevision_print_dep(Boost ALICEVISION_HAVE_BOOST)
alicevision_print_dep(Eigen3 ALICEVISION_HAVE_EIGEN3)
alicevision_print_dep(LAPACK ALICEVISION_HAVE_LAPACK)
alicevision_print_dep(BLAS ALICEVISION_HAVE_BLAS)
alicevision_print_dep(IMath ALICEVISION_HAVE_IMATH)
alicevision_print_dep(OpenJPH ALICEVISION_HAVE_OPENJPH)
alicevision_print_dep(OpenEXR ALICEVISION_HAVE_OPENEXR)
alicevision_print_dep(TIFF ALICEVISION_HAVE_TIFF)
alicevision_print_dep(JPEG ALICEVISION_HAVE_JPEG)
alicevision_print_dep(ZLIB ALICEVISION_HAVE_ZLIB)
alicevision_print_dep(PNG ALICEVISION_HAVE_PNG)
alicevision_print_dep(EXPAT ALICEVISION_HAVE_EXPAT)
alicevision_print_dep(Yaml-Cpp ALICEVISION_HAVE_YAML-CPP)
alicevision_print_dep(Pystring ALICEVISION_HAVE_PYSTRING)
alicevision_print_dep(OpenColorIO ALICEVISION_HAVE_OPENCOLORIO)
alicevision_print_dep(OpenImageIO ALICEVISION_HAVE_OPENIMAGEIO)
alicevision_print_dep(YUV ALICEVISION_HAVE_YUV)
alicevision_print_dep(AOM ALICEVISION_HAVE_AOM)
alicevision_print_dep(AVIF ALICEVISION_HAVE_AVIF)
alicevision_print_dep(GFLAGS ALICEVISION_HAVE_GFLAGS)
alicevision_print_dep(GLOG ALICEVISION_HAVE_GLOG)
alicevision_print_dep(OpenJPEG ALICEVISION_HAVE_OPENJPEG)
alicevision_print_dep(WEBP ALICEVISION_HAVE_WEBP)
alicevision_print_dep(AbseilCpp ALICEVISION_HAVE_ABSEIL)
alicevision_print_dep(SuiteSparse ALICEVISION_HAVE_SUITESPARSE)
alicevision_print_dep(Ceres ALICEVISION_HAVE_CERES)
alicevision_print_dep(CoinOr-CoinUtils ALICEVISION_HAVE_COINUTILS)
alicevision_print_dep(CoinOr-Osi ALICEVISION_HAVE_OSI)
alicevision_print_dep(CoinOr-Clp ALICEVISION_HAVE_CLP)
alicevision_print_dep(SoPLEX ALICEVISION_HAVE_SOPLEX)
alicevision_print_dep(LEMON ALICEVISION_HAVE_LEMON)
alicevision_print_dep(lz4 ALICEVISION_HAVE_LZ4)
alicevision_print_dep(Flann ALICEVISION_HAVE_FLANN)
alicevision_print_dep(nanoflann ALICEVISION_HAVE_NANOFLANN)
alicevision_print_dep(OpenMesh ALICEVISION_HAVE_OPENMESH)
alicevision_print_dep(Geogram ALICEVISION_HAVE_GEOGRAM)
alicevision_print_dep(Assimp ALICEVISION_HAVE_ASSIMP)
message("├ Optional Dependencies:${Esc}[50G│")
alicevision_print_dep(OpenMP ALICEVISION_HAVE_OPENMP)
alicevision_print_dep(OpenCV ALICEVISION_HAVE_OPENCV)
alicevision_print_dep(ONNX ALICEVISION_HAVE_ONNX)
alicevision_print_dep(oneTBB ALICEVISION_HAVE_ONETBB)
alicevision_print_dep(CCTag ALICEVISION_HAVE_CCTAG)
alicevision_print_dep(AprilTag ALICEVISION_HAVE_APRILTAG)
alicevision_print_dep(Alembic ALICEVISION_HAVE_ALEMBIC)
alicevision_print_dep(OpenGV ALICEVISION_HAVE_OPENGV)
alicevision_print_dep(Xerces-C ALICEVISION_HAVE_XERCESCPP)
alicevision_print_dep(E57Format ALICEVISION_HAVE_E57FORMAT)
alicevision_print_dep(Metal ALICEVISION_HAVE_METAL)
alicevision_print_dep(Vulkan ALICEVISION_HAVE_VULKAN)
message("├ CUDA-Only Dependencies:${Esc}[50G│")
alicevision_print_dep(ONNX-GPU ALICEVISION_HAVE_ONNX_GPU)
alicevision_print_dep(PopSift ALICEVISION_HAVE_POPSIFT)
message("├ Vulkan-Only Dependencies:${Esc}[50G│")
alicevision_print_dep(VulkanMemoryAllocator ALICEVISION_HAVE_VMA)
alicevision_print_dep(FP16 ALICEVISION_HAVE_FP16)
message("├ Plugin-Only Dependencies:${Esc}[50G│")
alicevision_print_dep(Qt6 ALICEVISION_HAVE_QT6)
message("└──────────── Dependency Summary End ────────────┘\n")

###===================================================
###
### Create the Configuration Header
###
###===================================================
set(configfile "${CMAKE_SOURCE_DIR}/Sources/Headers/AV/config.hpp")
configure_file("${CMAKE_SOURCE_DIR}/CMake/Resources/config.hpp.in" "${configfile}" @ONLY)

###===================================================
###
### AliceVision CMake Helpers
###
###===================================================
set(ALICEVISION_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/Sources/Headers")
set(ALICEVISION_PLUGIN_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/Sources/Plugins/AliceVisionQt/Sources/Headers")
include(Utilities/AliceVisionAppleBundle)
include(Utilities/AliceVisionInterface)
include(Utilities/AliceVisionKernelLibrary)
include(Utilities/AliceVisionLibrary)
include(Utilities/AliceVisionPlugin)
include(Utilities/AliceVisionSoftware)
include(Utilities/AliceVisionTest)

###===================================================
###
### Tracked Targets
###
###===================================================
set(ALICEVISION_LIBRARIES CACHE INTERNAL "AliceVision Libraries")
set(ALICEVISION_BINARIES  CACHE INTERNAL "AliceVision Binaries")
set(ALICEVISION_PLUGINS  CACHE INTERNAL "AliceVision Plugins")

###===================================================
###
### AliceVision Libraries
###
###===================================================
add_subdirectory(Libraries)

###===================================================
###
### AliceVision Binaries
###
###===================================================
add_subdirectory(Binaries)

###===================================================
###
### AliceVision Plugins
###
###===================================================
if(ALICEVISION_BUILD_MESHROOM_QT_PLUGINS)
    add_subdirectory(Plugins/AliceVisionQt)
endif()

###===================================================
###
### Bundling (macOS only)
###
###===================================================
if(APPLE AND ALICEVISION_CREATE_AV_BUNDLE)

    # Set Resources to copy
    set(BUNDLE_RESOURCE_LIST
            "${CMAKE_SOURCE_DIR}/LICENSE-MPL2.md"
            "${CMAKE_SOURCE_DIR}/LICENSE-MIT-libmv.md"
            "${CMAKE_SOURCE_DIR}/COPYING.md"
            "${CMAKE_SOURCE_DIR}/CONTRIBUTORS.md"
    )
    if(ALICEVISION_INSTALL_MESHROOM_PLUGIN)
        list(APPEND BUNDLE_RESOURCE_LIST "${CMAKE_SOURCE_DIR}/Meshroom")
    endif()

    alicevision_add_apple_bundle(AliceVision
            SYMLINK_FRAMEWORK_RESOURCES
            ICON "${CMAKE_SOURCE_DIR}/Resources/AliceVision.icns"
            LIBRARIES
                ${ALICEVISION_LIBRARIES}
            BINARIES
                ${ALICEVISION_BINARIES}
            PLUGINS
                ${ALICEVISION_PLUGINS}
            RESOURCES
                ${BUNDLE_RESOURCE_LIST}
    )
endif()

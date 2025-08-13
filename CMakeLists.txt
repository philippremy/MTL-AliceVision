###===================================================
###
###       ╱░▒╲
###     ╱░░░▒▒▒╲
###   ╱░░╱    ╲▒▒╲
###  ╱░░╱      ╲▒▒╲     The AliceVision Project
###  ╲▒▒╲      ╱▓▓╱     3D Computer Vision Framework
###   ╲▒▒╲    ╱▓▓╱
###     ╲▒▒▒▓▓▓╱
###       ╲▒▓╱
###
###===================================================

cmake_minimum_required(VERSION 3.28)

# Check if on macOS, then build fat binaries (arm64;x86_64)
if(CMAKE_HOST_SYSTEM_NAME STREQUAL "Darwin")
    set(CMAKE_OSX_ARCHITECTURES "arm64;x86_64" CACHE STRING "The Architectures to build on macOS")
endif()

project(AliceVision LANGUAGES C CXX)
message("\n       ╱░▒╲\n     ╱░░░▒▒▒╲\n   ╱░░╱    ╲▒▒╲\n  ╱░░╱      ╲▒▒╲     The AliceVision Project\n  ╲▒▒╲      ╱▓▓╱     3D Computer Vision Framework\n   ╲▒▒╲    ╱▓▓╱\n     ╲▒▒▒▓▓▓╱\n       ╲▒▓╱\n")

# Enable CUDA language on non-macOS targets
if(NOT APPLE)
    check_language(CUDA)
    if(CMAKE_CUDA_COMPILER)
        enable_language(CUDA)
    else()
        message(STATUS "[AliceVision] No CUDA compiler was found. AliceVision will be build without CUDA support.")
    endif()
endif()

# At least C++20 is required to build all modules
set(CMAKE_CXX_STANDARD 20)

###===================================================
###
### Project Options
###
###===================================================

# Feature Settings
option(ALICEVISION_EIGEN_MEMORY_ALIGNMENT "Enable Eigen memory alignment (mostly required for C++ < 17)" ON)
option(ALICEVISION_USE_OPENMP "Enable OpenMP as threading library" ON)
option(ALICEVISION_USE_APRILTAG "Whether to enable support for AprilTag Markers" ON)
option(ALICEVISION_USE_CCTAG "Whether to enable support for CCTag Marker" ON)
option(ALICEVISION_USE_OPENGV "Whether to use OpenGV" ON)
option(ALICEVISION_USE_ALEMBIC "Whether to use Alembic" ON)
option(ALICEVISION_USE_ONNX "Whether to use ONNX Runtime" ON)
option(ALICEVISION_USE_OPENCV "Whether to use OpenCV" ON)
option(ALICEVISION_USE_E57FORMAT "Whether to use e57Format" ON)
option(ALICEVISION_USE_MESHSDFILTER "Use MeshSDFilter library (enable MeshDenoising and MeshDecimate)" ON)
if(CMAKE_CUDA_COMPILER)
    option(ALICEVISION_USE_CUDA "Whether to use NVIDIA CUDA acceleration" ON)
endif()
if(APPLE)
    option(ALICEVISION_USE_METAL "Whether to use Metal GPU acceleration" ON)
endif()
if(ALICEVISION_USE_CUDA)
    option(ALICEVISION_USE_UNCERTAINTYTE "Whether to use UncertaintyTE" ON)
endif()
if(ALICEVISION_USE_CUDA AND ALICEVISION_USE_ONNX)
    option(ALICEVISION_USE_ONNX_GPU "Use ONNX GPU acceleration" ON)
endif()
if(ALICEVISION_USE_CUDA)
    option(ALICEVISION_USE_POPSIFT "Enable CUDA accelerated SIFT implementation" ON)
endif()
if(NOT ALICEVISION_USE_CUDA AND NOT ALICEVISION_USE_METAL)
    option(ALICEVISION_USE_VULKAN "Whether to use Vulkan GPU acceleration" ON)
endif()

# Build Settings
option(BUILD_SHARED_LIBS "Whether the project should build shared libraries instead of static libraries" ON)
if(APPLE AND BUILD_SHARED_LIBS)
    option(BUILD_APPLE_FRAMEWORKS "Whether the project should build Apple Frameworks" ON)
endif()
if(APPLE)
    option(ALICEVISION_CREATE_AV_BUNDLE "Whether to create a self contained bundle" ON)
endif()
option(ALICEVISION_BUILD_SOFTWARE "Build the command line tools of the AliceVision Project" ON)
option(ALICEVISION_BUILD_TESTS "Whether to build the module tests" ON)
option(ALICEVISION_BUILD_MESHROOM_QT_PLUGINS "Whether to build the Meshroom Qt Plugins" ON)
option(ALICEVISION_BUILD_GUI "Whether to build the Meshroom GUI Application" ON)
option(ALICEVISION_USE_RPATH "Add RPATH on software with relative paths to libraries" ON)
option(ALICEVISION_REMOVE_ABSOLUTE "Remove absolute paths in dependencies" OFF)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type for AliceVision" FORCE)
endif()
if(ALICEVISION_USE_VULKAN)
    option(ALICEVISION_USE_VULKAN_VALIDATION_LAYERS "Whether to use Vulkan Validation Layers (WARNING: Will decrease performance!)" OFF)
endif()

# Install Settings
option(ALICEVISION_INSTALL_MESHROOM_PLUGIN "Whether to install AliceVision's plugin for Meshroom" ON)
option(ALICEVISION_CREATE_BUNDLE "Whether to create a final bundle containing all modules and dependencies" ON)

###===================================================
###
### Project Configuration
###
###===================================================

# Add Module Search Path
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/CMake")

# Binary Stripping
if (CMAKE_BUILD_TYPE MATCHES Release)
    message(STATUS "[AliceVision] Final binaries will be stripped.")
    set(CMAKE_INSTALL_DO_STRIP TRUE)
endif()

# RPaths
if (ALICEVISION_USE_RPATH)
    if (APPLE)
        set(CMAKE_MACOSX_RPATH 1)
        set(CMAKE_INSTALL_RPATH
                "@loader_path/../${CMAKE_INSTALL_LIBDIR}"
                "@loader_path"
                "@loader_path/.."
                "@loader_path/../Frameworks"
                "@loader_path/../Libraries"
                "@executable_path/../${CMAKE_INSTALL_LIBDIR}"
                "@executable_path"
                "@executable_path/.."
                "@executable_path/../Frameworks"
                "@executable_path/../Libraries"
                "${CMAKE_INSTALL_LIBDIR}"
        )
    elseif (UNIX)
        set(CMAKE_INSTALL_RPATH "\\$ORIGIN/../${CMAKE_INSTALL_LIBDIR};\\$ORIGIN;${CMAKE_INSTALL_LIBDIR};")
    endif()
endif()

# Eigen Memory Alignment Definitions
if (NOT ALICEVISION_EIGEN_MEMORY_ALIGNMENT)
    set(EIGEN_CMAKE_ALIGNMENT_FLAGS
            "-DCMAKE_CXX_FLAGS:STRING=-DEIGEN_MAX_ALIGN_BYTES=0 -DEIGEN_MAX_STATIC_ALIGN_BYTES=0"
            "-DCMAKE_C_FLAGS:STRING=-DEIGEN_MAX_ALIGN_BYTES=0 -DEIGEN_MAX_STATIC_ALIGN_BYTES=0")
    set(ALICEVISION_EIGEN_REQUIRE_ALIGNMENT 0)
else()
    set(ALICEVISION_EIGEN_REQUIRE_ALIGNMENT 1)
endif()

# Testing
if (ALICEVISION_BUILD_TESTS)
    include(CTest)
    enable_testing()
endif()

# Windows specific defines
if (WIN32)
    add_definitions(-DNOMINMAX)
    add_definitions(-D_USE_MATH_DEFINES)
    if (MSVC)
        add_compile_options(/bigobj)
        add_compile_options(/MP)
    endif()
    if (BUILD_SHARED_LIBS)
        # Export all symbols from the dynamic libraries by default (avoid dllexport markup)
        set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
    endif()
    # Disable BOOST autolink
    add_definitions(-DBOOST_ALL_NO_LIB)

    # To be removed later, a bug to make things work with current vcpkg
    # https://github.com/microsoft/vcpkg/issues/22495
    add_definitions(-DBOOST_USE_WINAPI_VERSION=BOOST_WINAPI_VERSION_WIN10)
endif()

# Boost Definitions
if(BUILD_SHARED_LIBS)
    # Force BOOST to use dynamic libraries (avoid link error with boost program_options)
    # https://lists.boost.org/boost-users/2009/11/54015.php
    add_definitions(-DBOOST_ALL_DYN_LINK=1)
    add_definitions(-DBOOST_TEST_DYN_LINK=1)
    add_definitions(-DBOOST_PROGRAM_OPTIONS_DYN_LINK=1)
else()
    set(Boost_USE_STATIC_LIBS ON)
endif()

# Xcode Definitions
set(CMAKE_XCODE_GENERATE_TOP_LEVEL_PROJECT_ONLY ON)

# Vulkan Validation Layers
if(ALICEVISION_USE_VULKAN_VALIDATION_LAYERS)
    set(ALICEVISION_USE_VULKAN_VALIDATION 1)
else()
    set(ALICEVISION_USE_VULKAN_VALIDATION 0)
endif()

###===================================================
###
### CMake Checks
###
###===================================================

# Guard against in-source builds
if (${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
    message(FATAL_ERROR "[AliceVision] In-source builds not allowed. Please create a dedicated build folder and run CMake again.
    This command produced the file `CMakeCache.txt' and the directory `CMakeFiles'. Please delete them.")
endif()

###===================================================
###
### Project Utils
###
###===================================================

# Optimize build for known architectures
include(OFA/OptimizeForArchitecture)
# Only optimize on Apple if not both architectures are build
set(ALICEVISION_HAVE_SSE 0)
OptimizeForArchitecture()
if (SSE2_FOUND OR TARGET_ARCHITECTURE STREQUAL "native")
    if (MSVC AND NOT ${CMAKE_CL_64})
        add_definitions(/arch:SSE2)
    endif()
    set(ALICEVISION_HAVE_SSE 1)
endif()

# GNU Install Dirs
include(GNUInstallDirs)

###===================================================
###
### Sources
###
###===================================================
add_subdirectory(Sources)

project(SDFilter)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_VERBOSE_MAKEFILE ON)


# Additional compiler flags
if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
	set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE}")
elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "AppleClang")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
	set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE}")
elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
	set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE}")
elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
    add_definitions(/DUSE_MSVC)
    add_definitions(/D_USE_MATH_DEFINES)
endif()

# Detect OpenMP environment
set(OPENMP ON CACHE BOOL "OpenMP")
if (OPENMP)
    find_package(OpenMP QUIET COMPONENTS CXX)
    if (OPENMP_FOUND)
        add_definitions(-DUSE_OPENMP)
    endif()
endif()


# Detect Eigen3
if (NOT EIGEN3_FOUND)
    find_package(Eigen3 REQUIRED)
    if (EIGEN3_FOUND)
    endif()
endif()

find_package(OpenMesh REQUIRED)

add_library(MeshSDLibrary INTERFACE)
target_include_directories(MeshSDLibrary INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(MeshSDLibrary INTERFACE Eigen3::Eigen OpenMeshCore)


# Executable for filtering
add_executable(MeshSDFilter
    EigenTypes.h
    MeshTypes.h
    SDFilter.h
    MeshNormalFilter.h
    MeshSDFilter.cpp
)
target_link_libraries(MeshSDFilter MeshSDLibrary)


# Executable for denoising
add_executable(MeshDenoiser
    EigenTypes.h
    MeshTypes.h
    SDFilter.h
    MeshNormalFilter.h
    MeshNormalDenoising.h
    MeshDenoiser.cpp
)
target_link_libraries(MeshDenoiser MeshSDLibrary)


if (OPENMP_FOUND)
    # target_compile_options(MeshSDLibrary PUBLIC "$<$<CONFIG:RELEASE>:${OpenMP_CXX_FLAGS}>")
    # target_compile_definitions(MeshSDLibrary PUBLIC "$<$<CONFIG:RELEASE>:USE_OPENMP>")
    # target_link_libraries(MeshSDLibrary "$<$<CONFIG:RELEASE>:${OpenMP_CXX_FLAGS}>")

    target_compile_options(MeshSDFilter PUBLIC "$<$<CONFIG:RELEASE>:${OpenMP_CXX_FLAGS}>")
    target_compile_definitions(MeshSDFilter PUBLIC "$<$<CONFIG:RELEASE>:USE_OPENMP>")
    target_link_libraries(MeshSDFilter "$<$<CONFIG:RELEASE>:${OpenMP_CXX_FLAGS}>" OpenMP::OpenMP_CXX)

    target_compile_options(MeshDenoiser PUBLIC "$<$<CONFIG:RELEASE>:${OpenMP_CXX_FLAGS}>")
    target_compile_definitions(MeshDenoiser PUBLIC "$<$<CONFIG:RELEASE>:USE_OPENMP>")
    target_link_libraries(MeshDenoiser "$<$<CONFIG:RELEASE>:${OpenMP_CXX_FLAGS}>" OpenMP::OpenMP_CXX)
endif()

macro(alicevision_fixup_appleclang_openmp)

    # We assume that the dependency helper actually build OpenMP successfully
    find_file(OpenMP_C_Library libomp.a
        PATHS "${CMAKE_SOURCE_DIR}/External/Products/lib"
        PATH_SUFFIXES "llvm"
    )
    find_file(OpenMP_C_IncludeDir omp.h
            PATHS "${CMAKE_SOURCE_DIR}/External/Products/include"
            PATH_SUFFIXES "llvm"
    )

    # Unlikely case in which the built product cannot be found
    if(NOT OpenMP_C_Library OR NOT OpenMP_C_IncludeDir)
        message(FATAL_ERROR "[AliceVision] Failed to find freshly build OpenMP for Apple Clang! Please submit a bug report!")
    endif()

    get_filename_component(OpenMP_BASE_DIR "${OpenMP_C_Library}" DIRECTORY)
    set(OpenMP_ROOT "${OpenMP_BASE_DIR}" FORCE CACHE PATH "")
    set(OpenMP_C "${CMAKE_C_COMPILER}" CACHE STRING "" FORCE)
    set(OpenMP_C_FLAGS "-Xclang -fopenmp" CACHE STRING "" FORCE)
    set(OpenMP_C_LIB_NAMES "libomp" "libgomp" "libiomp5" CACHE STRING "" FORCE)
    set(OpenMP_CXX "${CMAKE_CXX_COMPILER}" CACHE STRING "" FORCE)
    set(OpenMP_CXX_FLAGS "-Xclang -fopenmp" CACHE STRING "" FORCE)
    set(OpenMP_CXX_LIB_NAMES "libomp" "libgomp" "libiomp5" CACHE STRING "" FORCE)
    set(OpenMP_libomp_LIBRARY "-lomp" CACHE STRING "" FORCE)
    set(OpenMP_libgomp_LIBRARY "-lgomp" CACHE STRING "" FORCE)
    set(OpenMP_libiomp5_LIBRARY "-liomp5" CACHE STRING "" FORCE)
    find_package(OpenMP QUIET REQUIRED COMPONENTS C CXX)

    # Apple Clang needs an include dir and a library search path
    get_filename_component(OpenMP_C_INCLUDE_DIR "${OpenMP_C_IncludeDir}" DIRECTORY)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -I${OpenMP_C_INCLUDE_DIR}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -I${OpenMP_C_INCLUDE_DIR}")
    target_link_options(OpenMP::OpenMP_C INTERFACE "-L${OpenMP_BASE_DIR}")
    target_link_options(OpenMP::OpenMP_CXX INTERFACE "-L${OpenMP_BASE_DIR}")
endmacro()
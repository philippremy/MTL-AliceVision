find_package(SuiteSparse_config QUIET)
find_package(CHOLMOD QUIET)
find_package(SPQR QUIET)

if(NOT (SuiteSparse_config_FOUND OR CHOLMOD_FOUND OR SPQR_FOUND))
    return()
endif()

set(SuiteSparse_FOUND TRUE)
set(SuiteSparse_VERSION ${SuiteSparse_config_VERSION})

# look for suitesparse metis link (ceres shouldn't link metis itself, but it still needs to know)
include (CheckSymbolExists)
include (CMakePushCheckState)

cmake_push_check_state (RESET)
set (CMAKE_REQUIRED_LIBRARIES SuiteSparse::CHOLMOD)
check_symbol_exists (cholmod_metis cholmod.h SuiteSparse_CHOLMOD_USES_METIS)
cmake_pop_check_state ()

if (SuiteSparse_CHOLMOD_USES_METIS)
   add_library(SuiteSparse::Partition IMPORTED INTERFACE)
   set(SuiteSparse_Partition_FOUND TRUE)
else()
   set(SuiteSparse_Partition_FOUND FALSE)
endif()
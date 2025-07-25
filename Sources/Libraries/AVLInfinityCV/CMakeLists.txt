# Headers
set(lInftycomputervision_files_headers
    LInfinityCV.hpp
    resection.hpp
    resection_kernel.hpp
    triangulation.hpp
    global_translations_fromTij.hpp
    global_translations_fromTriplets.hpp
    tijsAndXis_From_xi_Ri.hpp
    tijsAndXis_From_xi_Ri_noise.hpp
    triplet_tijsAndXis_kernel.hpp
)

# Sources
set(lInftycomputervision_files_sources
    resection_kernel.cpp
)

# Disable some warnings on MSVC.
if (MSVC)
    # Disable Warning	C4267: conversion from 'size_t' to 'int', possible loss of data
    add_definitions("/wd4267")
endif()

alicevision_add_library(AVLInfinityCV
    SOURCES ${lInftycomputervision_files_sources}
    HEADERS ${lInftycomputervision_files_headers}
    PUBLIC_LINKS
        AVLinearProgramming
)

if(ALICEVISION_HAVE_MOSEK)
    target_link_libraries(AVLInfinityCV PUBLIC ${MOSEK_LIB})
endif()

# Unit tests
# Linfinity Triangulation
alicevision_add_test(Tests/LInftyCV_triangulation_test.cpp
    NAME "LInftyCV_Triangulation"
    LINKS
        AVMultiviewTestData
        AVMultiview
        AVLInfinityCV
)

# Linfinity Pose/Resection estimation
alicevision_add_test(Tests/LInftyCV_resection_test.cpp
    NAME "LInftyCV_Resection"
    LINKS
        AVMultiviewTestData
        AVMultiview
        AVLInfinityCV
)
alicevision_add_test(Tests/LInftyCV_resection_robust_test.cpp
    NAME "LInftyCV_ResectionRobust"
    LINKS
        AVMultiviewTestData
        AVMultiview
        AVLInfinityCV
)
# Linfinity Global translations from triplets of heading directions
alicevision_add_test(Tests/LInftyCV_global_translations_fromTriplets_test.cpp
    NAME "LInftyCV_GlobalTranslationsFromTriplets"
    LINKS
        AVMultiviewTestData
        AVMultiview
        AVLInfinityCV
)

# Linfinity Global translations from heading directions
alicevision_add_test(Tests/LInftyCV_global_translations_fromTij_test.cpp
    NAME "LInftyCV_GlobalTranslationsFromTij"
    LINKS
        AVMultiviewTestData
        AVMultiview
        AVLInfinityCV
)
# Linfinity Translation and Structure
alicevision_add_test(Tests/LInftyCV_tijsAndXis_test.cpp
    NAME "LInftyCV_TijsAndXis"
    LINKS
        AVMultiviewTestData
        AVMultiview
        AVLInfinityCV
)

# Linfinity Translation and Structure with noise handling
alicevision_add_test(Tests/LInftyCV_tijsAndXis_outlier_test.cpp
    NAME "lInftyCV_TijsAndXisOutlier"
    LINKS
        AVMultiviewTestData
        AVMultiview
        AVLInfinityCV
)

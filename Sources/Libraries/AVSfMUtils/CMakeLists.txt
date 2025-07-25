# Headers
set(sfm_utils_files_headers
    alignment.hpp
    statistics.hpp
    preprocess.hpp
    syntheticScene.hpp
)

# Sources
set(sfm_utils_files_sources
    alignment.cpp
    preprocess.cpp
    statistics.cpp
    syntheticScene.cpp
)

alicevision_add_library(AVSfMUtils
    SOURCES ${sfm_utils_files_sources}
    HEADERS ${sfm_utils_files_headers}
    PUBLIC_LINKS
        AVFeature
        AVGeometry
        AVMatching
        AVMultiviewTestData
        AVNumeric
        AVSfMData
        AVSTL
        AVTrack
        Eigen3::Eigen
    PRIVATE_LINKS
        Boost::boost
)

# Unit tests

alicevision_add_test(Tests/Alignment_test.cpp
    NAME "SfMUtils_Alignment"
    LINKS
        AVMultiview
        AVMultiviewTestData
        AVSfM
        ${LEMON_LIBRARY}
)

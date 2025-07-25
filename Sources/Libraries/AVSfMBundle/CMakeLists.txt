# Headers
set(sfm_bundle_files_headers
    BundleAdjustment.hpp
    BundleAdjustmentCeres.hpp
)

# Sources
set(sfm_bundle_files_sources
    BundleAdjustmentCeres.cpp
    LocalBundleAdjustmentGraph.cpp
)

alicevision_add_library(AVSfMBundle
    SOURCES ${sfm_bundle_files_sources}
    HEADERS ${sfm_bundle_files_headers}
    PRIVATE_LINKS
        glog::glog
    PUBLIC_LINKS
        AVCamera
        AVGeometry
        AVNumeric
        AVSfMData
        AVSystem
        Ceres::ceres
        ${SUITESPARSE_LIBRARIES}
)

# Unit tests
alicevision_add_test(Tests/BundleAdjustment_test.cpp
    NAME "SfM_BundleAdjustment"
    LINKS
        AVFeature
        AVMultiview
        AVMultiviewTestData
        AVSfM
        AVSystem
        ${LEMON_LIBRARY}
)

alicevision_add_test(Tests/BundleAdjustment_Enhanced_test.cpp
    NAME "SfM_BundleAdjustment_Enhanced"
    LINKS
        AVFeature
        AVMultiview
        AVMultiviewTestData
        AVSfM
        AVSystem
        ${LEMON_LIBRARY}
)

alicevision_add_test(RotationAveraging_test.cpp
    NAME "Multiview_RotationAveraging"
    LINKS
        AVMultiview
        AVMultiviewTestData
        Ceres::ceres
        ${SUITESPARSE_LIBRARIES}
)

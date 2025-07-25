alicevision_add_test(TranslationAveraging_test.cpp
    NAME "Multiview_TranslationAveraging"
    LINKS
        AVMultiview
        AVMultiviewTestData
        Ceres::ceres
        ${SUITESPARSE_LIBRARIES}
)

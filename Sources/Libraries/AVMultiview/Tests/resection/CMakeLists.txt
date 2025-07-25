alicevision_add_test(P4pfSolver_test.cpp
    NAME "Multiview_P4pfSolver"
    LINKS AVMultiview AVMultiviewTestData
)
alicevision_add_test(P5pfrSolver_test.cpp
    NAME "Multiview_P5pfrSolver"
    LINKS AVMultiview AVMultiviewTestData
)
alicevision_add_test(Resection6PSolver_test.cpp
    NAME "Multiview_Resection6PSolver"
    LINKS AVMultiview AVMultiviewTestData
)
alicevision_add_test(ResectionKernel_test.cpp
    NAME "Multiview_ResectionKernel"
    LINKS AVMultiview AVMultiviewTestData
)
alicevision_add_test(ResectionLORansac_test.cpp
    NAME "Multiview_ResectionLORansac"
    LINKS
        AVMultiview
        AVMultiviewTestData
        AVFeature
        AVSfM
        Ceres::ceres
        ${SUITESPARSE_LIBRARIES}
)

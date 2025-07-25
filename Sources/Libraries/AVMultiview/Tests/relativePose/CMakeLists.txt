# Kernels
alicevision_add_test(FundamentalKernel_test.cpp
    NAME "Multiview_RelativePose_FundamentalKernel"
    LINKS AVMultiview
)
alicevision_add_test(HomographyKernel_test.cpp
   NAME "Multiview_RelativePose_HomographyKernel"
   LINKS AVMultiview AVMultiviewTestData
)
alicevision_add_test(EssentialKernel_test.cpp
   NAME "Multiview_RelativePose_EssentialKernel"
   LINKS AVMultiview AVMultiviewTestData
)

# Solvers
alicevision_add_test(Fundamental10PSolver_test.cpp
    NAME "Multiview_RelativePose_Fundamental10Solver"
    LINKS AVMultiview AVMultiviewTestData
)
alicevision_add_test(Essential5PSolver_test.cpp
    NAME "Multiview_RelativePose_Essential5PSolver"
    LINKS AVMultiview AVMultiviewTestData
)

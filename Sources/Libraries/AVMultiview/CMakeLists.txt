# Headers
set(multiview_files_headers
    affineSolver.hpp
    essential.hpp
    epipolarEquation.hpp
    knownRotationTranslationKernel.hpp
    Unnormalizer.hpp
    RelativePoseKernel.hpp
    ResectionKernel.hpp
    relativePose/Essential5PSolver.hpp
    relativePose/Essential8PSolver.hpp
    relativePose/EssentialKernel.hpp
    relativePose/Fundamental7PSolver.hpp
    relativePose/Fundamental8PSolver.hpp
    relativePose/Fundamental10PSolver.hpp
    relativePose/FundamentalError.hpp
    relativePose/FundamentalKernel.hpp
    relativePose/Homography4PSolver.hpp
    relativePose/HomographyError.hpp
    relativePose/HomographyKernel.hpp
    relativePose/Rotation3PSolver.hpp
    relativePose/ISolverErrorRelativePose.hpp
    relativePose/RelativeSphericalKernel.hpp
    relativePose/RotationSphericalKernel.hpp
    resection/EPnPSolver.hpp
    resection/EPnPKernel.hpp
    resection/P3PSolver.hpp
    resection/P4PfSolver.hpp
    resection/P5PfrSolver.hpp
    resection/ProjectionDistanceError.hpp
    resection/Resection6PSolver.hpp
    resection/ResectionResectionKernel.hpp
    resection/ISolverErrorResection.hpp
    rotationAveraging/rotationAveragingCommon.hpp
    rotationAveraging/rotationAveraging.hpp
    rotationAveraging/l1.hpp
    rotationAveraging/l2.hpp
    translationAveraging/translationAveragingCommon.hpp
    translationAveraging/solver.hpp
    triangulation/Triangulation.hpp
    triangulation/triangulationDLT.hpp
    triangulation/NViewsTriangulationLORansac.hpp
)

# Sources
set(multiview_files_sources
    affineSolver.cpp
    essential.cpp
    Unnormalizer.cpp
    relativePose/Essential5PSolver.cpp
    relativePose/Essential8PSolver.cpp
    relativePose/Fundamental7PSolver.cpp
    relativePose/Fundamental8PSolver.cpp
    relativePose/Fundamental10PSolver.cpp
    relativePose/Homography4PSolver.cpp
    relativePose/Rotation3PSolver.cpp
    resection/EPnPSolver.cpp
    resection/P3PSolver.cpp
    resection/P4PfSolver.cpp
    resection/P5PfrSolver.cpp
    resection/Resection6PSolver.cpp
    rotationAveraging/l1.cpp
    rotationAveraging/l2.cpp
    translationAveraging/solverL2Chordal.cpp
    translationAveraging/solverL1Soft.cpp
    triangulation/triangulationDLT.cpp
    triangulation/Triangulation.cpp
)

alicevision_add_library(AVMultiview
    SOURCES ${multiview_files_sources}
    HEADERS ${multiview_files_headers}
    PUBLIC_LINKS
        AVNumeric
        AVRobustEstimation
        AVCamera
        Ceres::ceres
        ${SUITESPARSE_LIBRARIES}
    PRIVATE_LINKS
        AVSystem
        ${LEMON_LIBRARY}
)

# Unit tests
add_subdirectory(Tests/relativePose)
add_subdirectory(Tests/resection)
add_subdirectory(Tests/rotationAveraging)
add_subdirectory(Tests/translationAveraging)
add_subdirectory(Tests/triangulation)

alicevision_add_test(Tests/AffineSolver_test.cpp
    NAME "Multiview_affineSolver"
    LINKS AVMultiview
)
alicevision_add_test(Tests/KnownRotationTranslationKernel_test.cpp
    NAME "Multiview_knownRotationTranslationKernel"
    LINKS AVMultiview AVMultiviewTestData
)

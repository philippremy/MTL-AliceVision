# Headers
set(robustEstimation_files_headers
    conditioning.hpp
    estimators.hpp
    LineKernel.hpp
    lineTestGenerator.hpp
    PointFittingKernel.hpp
    PointFittingRansacKernel.hpp
    ISolver.hpp
    IRansacKernel.hpp
    Ransac.hpp
    ACRansac.hpp
    LORansac.hpp
    ransacTools.hpp
    randSampling.hpp
    leastMedianOfSquares.hpp
    ScoreEvaluator.hpp
    maxConsensus.hpp
)

# Sources
set(robustEstimation_files_sources
    conditioning.cpp
)

alicevision_add_library(AVRobustEstimation
    SOURCES ${robustEstimation_files_sources}
    HEADERS ${robustEstimation_files_headers}
    PUBLIC_LINKS
        AVNumeric
        AVSystem
)

# Unit tests
alicevision_add_test(Tests/RandSampling_test.cpp
    NAME "RobustEstimation_RandSampling"
    LINKS AVRobustEstimation
)
alicevision_add_test(Tests/LineKernel_test.cpp
    NAME "RobustEstimation_LineKernel"
    LINKS AVRobustEstimation
)
alicevision_add_test(Tests/Ransac_test.cpp
    NAME "RobustEstimation_Ransac"
    LINKS AVRobustEstimation
)
alicevision_add_test(Tests/ACRansac_test.cpp
    NAME "RobustEstimation_ACRansac"
    LINKS AVRobustEstimation
)
alicevision_add_test(Tests/LORansac_test.cpp
    NAME "RobustEstimation_LORansac"
    LINKS AVRobustEstimation
)
alicevision_add_test(Tests/MaxConsensus_test.cpp
    NAME "RobustEstimation_MaxConsensus"
    LINKS AVRobustEstimation
)

# TODO: WHY?
# alicevision_add_test(leastMedianOfSquares_test.cpp
#    NAME "robustEstimation_leastMedianOfSquares"
#    LINKS aliceVision_robustEstimation
# )

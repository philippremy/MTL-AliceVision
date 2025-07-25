# Headers
set(colorHarmonization_files_headers
        CommonDataByPair.hpp
        CommonDataByPair_fullFrame.hpp
        CommonDataByPair_matchedPoints.hpp
        CommonDataByPair_vldSegment.hpp
        GainOffsetConstraintBuilder.hpp
)

# Sources
set(colorHarmonization_files_sources
        GainOffsetConstraintBuilder.cpp
)

alicevision_add_library(AVColorHarmonization
        SOURCES ${colorHarmonization_files_sources}
        HEADERS ${colorHarmonization_files_headers}
        PUBLIC_LINKS
        AVFeature
        AVImage
        AVLinearProgramming
        AVMatching
)

# Unit tests
alicevision_add_test(Tests/GainOffsetConstraintBuilder_test.cpp
        NAME "ColorHarmonization_GainOffsetConstraintBuilder"
        LINKS AVColorHarmonization
)
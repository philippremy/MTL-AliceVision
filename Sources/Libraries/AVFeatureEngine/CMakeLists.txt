# Headers
set(featureEngine_files_headers
    FeatureExtractor.hpp
)

# Sources
set(featureEngine_files_sources
    FeatureExtractor.cpp
)

# Library
alicevision_add_library(AVFeatureEngine
    SOURCES ${featureEngine_files_sources}
    HEADERS ${featureEngine_files_headers}
    PUBLIC_LINKS
        AVFeature
        AVImage
        AVSfMData
        AVSystem
)
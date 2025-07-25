# Headers
set(rig_files_headers
    colorHarmonizeEngineGlobal.hpp
)

# Sources
set(rig_files_sources
    colorHarmonizeEngineGlobal.cpp
)

alicevision_add_library(AVSfMColorHarmonize
    SOURCES ${rig_files_sources}
    HEADERS ${rig_files_headers}
    PUBLIC_LINKS
        AVColorHarmonization
        AVFeature
        AVGraph
        AVImage
        AVKVLD
        AVMatching
        AVNumeric
        AVUtils
        AVSfM
        AVSfMData
        AVSfMDataIO
        AVSTL
        AVSystem
        AVTrack
)

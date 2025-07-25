# Headers
set(sfmMvsUtils_files_headers
    visibility.hpp
)

# Sources
set(sfmMvsUtils_files_sources
    visibility.cpp
)

alicevision_add_library(AVSfMMVSUtils
    SOURCES ${sfmMvsUtils_files_sources}
    HEADERS ${sfmMvsUtils_files_headers}
    PUBLIC_LINKS
        AVMesh
        AVMVSData
        AVSfMData
    PRIVATE_LINKS
        AVSystem
)

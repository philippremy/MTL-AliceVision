# Headers
set(kvld_files_headers
    kvld.h
    kvld_draw.h
    algorithm.h
)

# Sources
set(kvld_files_sources
    kvld.cpp
    algorithm.cpp
)

alicevision_add_library(AVKVLD
    SOURCES ${kvld_files_sources}
    HEADERS ${kvld_files_headers}
    PUBLIC_LINKS
        AVImage
        AVFeature
        AVNumeric
        AVSystem
)
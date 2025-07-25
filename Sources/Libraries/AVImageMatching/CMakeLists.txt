# Headers
set(imageMatching_files_headers
    ImageMatching.hpp
)

# Sources
set(imageMatching_files_sources
    ImageMatching.cpp
)

alicevision_add_library(AVImageMatching
    SOURCES ${imageMatching_files_sources}
    HEADERS ${imageMatching_files_headers}
    PRIVATE_LINKS
        AVImage
        AVVoctree
)
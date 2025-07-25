# Headers
set(lensCorrectionProfile_files_headers
    lcp.hpp
)

# Sources
set(lensCorrectionProfile_files_sources
    lcp.cpp
)

alicevision_add_library(AVLensCorrectionProfile
    SOURCES ${lensCorrectionProfile_files_sources}
    HEADERS ${lensCorrectionProfile_files_headers}
    PRIVATE_LINKS
        Boost::log
        expat::expat
)
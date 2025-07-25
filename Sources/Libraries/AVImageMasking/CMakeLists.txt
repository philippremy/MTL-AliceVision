# Headers
set(imageMasking_files_headers
    imageMasking.hpp
    eigen2cvHelpers.hpp
)

# Sources
set(imageMasking_files_sources
    imageMasking.cpp
)

alicevision_add_library(AVImageMasking
    SOURCES ${imageMasking_files_sources}
    HEADERS ${imageMasking_files_headers}
    PUBLIC_LINKS
        ${OpenCV_LIBS}
    PRIVATE_LINKS
        AVImage
        Boost::program_options
)
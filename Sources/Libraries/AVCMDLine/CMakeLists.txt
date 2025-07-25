# Sources
set(cmdline_files_sources
        cmdline.cpp
)

# Headers
set(cmdline_files_headers
        cmdline.hpp
)

alicevision_add_library(AVCMDLine
        SOURCES ${cmdline_files_sources}
        HEADERS ${cmdline_files_headers}
        PUBLIC_LINKS
        Boost::program_options
        PRIVATE_LINKS
        AVSystem
)
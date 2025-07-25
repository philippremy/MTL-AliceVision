# Headers
set(photometric_stereo_headers
    photometricDataIO.hpp
    normalIntegration.hpp
    photometricStereo.hpp
)

# Sources
set(photometric_stereo_sources
    photometricDataIO.cpp
    normalIntegration.cpp
    photometricStereo.cpp
)

alicevision_add_library(AVPhotometricStereo
    SOURCES ${photometric_stereo_sources}
    HEADERS ${photometric_stereo_headers}
    PUBLIC_LINKS
        ${OpenCV_LIBS}
        AVImage
        AVMVSData
        AVSfMData
        AVSfMDataIO
        AVSystem
)
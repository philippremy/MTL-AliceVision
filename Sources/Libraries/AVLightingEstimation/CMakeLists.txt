# Headers
set(lightingEstimation_files_headers
    augmentedNormals.hpp
    lightingEstimation.hpp
    lightingCalibration.hpp
    ellipseGeometry.hpp
)

# Sources
set(lightingEstimation_files_sources
    augmentedNormals.cpp
    lightingEstimation.cpp
    lightingCalibration.cpp
    ellipseGeometry.cpp
)

alicevision_add_library(AVLightingEstimation
    SOURCES ${lightingEstimation_files_sources}
    HEADERS ${lightingEstimation_files_headers}
    PUBLIC_LINKS
        AVImage
        AVPhotometricStereo
        AVSystem
        ${OpenCV_LIBS}
)

# Unit tests
alicevision_add_test(Tests/LightingEstimation_test.cpp
    NAME "LightingEstimation_LightingEstimation"
    LINKS AVLightingEstimation
)

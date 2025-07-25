# Headers
set(panorama_files_headers
    alphaCompositer.hpp
    boundingBox.hpp
    panoramaMap.hpp
    compositer.hpp
    coordinatesMap.hpp
    distance.hpp
    feathering.hpp
    gaussian.hpp
    graphcut.hpp
    imageOps.hpp
    laplacianCompositer.hpp
    laplacianPyramid.hpp
    remapBbox.hpp
    seams.hpp
    sphericalMapping.hpp
    warper.hpp
)

# Sources
set(panorama_files_sources
    warper.cpp
    gaussian.cpp
    boundingBox.cpp
    coordinatesMap.cpp
    distance.cpp
    remapBbox.cpp
    sphericalMapping.cpp
    feathering.cpp
    laplacianPyramid.cpp
    seams.cpp
    imageOps.cpp
    cachedImage.cpp
    panoramaMap.cpp
)

alicevision_add_library(AVPanorama
    SOURCES ${panorama_files_sources}
    HEADERS ${panorama_files_headers}
    PUBLIC_LINKS
        AVNumeric
    PRIVATE_LINKS
        AVCamera
        AVImage
        AVSystem
        ${OpenCV_LIBS}
)
# Headers
set(mvsUtils_files_headers
    common.hpp
    fileIO.hpp
    ImagesCache.hpp
    mapIO.hpp
    MultiViewParams.hpp
    TileParams.hpp
)

# Sources
set(mvsUtils_files_sources
    common.cpp
    fileIO.cpp
    ImagesCache.cpp
    mapIO.cpp
    MultiViewParams.cpp
    TileParams.cpp
)

alicevision_add_library(AVMVSUtils
    SOURCES ${mvsUtils_files_sources}
    HEADERS ${mvsUtils_files_headers}
    PUBLIC_LINKS
        AVCamera
        AVImage
        AVNumeric
        AVMultiview
        AVMVSData
        OpenImageIO::OpenImageIO_Util
        ${OpenCV_LIBS}
    PRIVATE_LINKS
        AVSystem
        Boost::boost
)
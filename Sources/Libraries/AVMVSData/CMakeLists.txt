# Headers
set(mvsData_files_headers
    geometry.hpp
    geometryTriTri.hpp
    Matrix3x3.hpp
    Matrix3x4.hpp
    OrientedPoint.hpp
    Point2d.hpp
    Point3d.hpp
    Point4d.hpp
    Pixel.hpp
    ROI.hpp
    Stat3d.hpp
    StaticVector.hpp
    structures.hpp
    Universe.hpp
    Voxel.hpp
)

# Sources
set(mvsData_files_sources
    geometry.cpp
    geometryTriTri.cpp
    Stat3d.cpp
    StaticVector.cpp
    structures.cpp
    Universe.cpp
)

alicevision_add_library(AVMVSData
    SOURCES ${mvsData_files_sources}
    HEADERS ${mvsData_files_headers}
    PUBLIC_LINKS
        AVImage
        AVNumeric
        AVSystem
        ZLIB::ZLIB
        Boost::boost
        OpenImageIO::OpenImageIO
        OpenImageIO::OpenImageIO_Util
)
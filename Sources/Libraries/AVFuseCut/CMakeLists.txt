# Headers
set(fuseCut_files_headers
    delaunayGraphCutTypes.hpp
    Fuser.hpp
    MaxFlow_AdjList.hpp
    Octree.hpp
    InputSet.hpp
    BoundingBox.hpp
    Intersections.hpp
    PointCloud.hpp
    GraphFiller.hpp
    Mesher.hpp
)

# Sources
set(fuseCut_files_sources
    Fuser.cpp
    MaxFlow_AdjList.cpp
    InputSet.cpp
    Tetrahedralization.cpp
    Intersections.cpp
    PointCloud.cpp
    GraphFiller.cpp
    Mesher.cpp
)

alicevision_add_library(AVFuseCut
    SOURCES ${fuseCut_files_sources}
    HEADERS ${fuseCut_files_headers}
    PUBLIC_LINKS
        AVMVSData
        AVMVSUtils
        AVMesh
        AVSfMData
        AVSystem
        Geogram::geogram
        Boost::graph
        Boost::container
    PRIVATE_LINKS
        nanoflann::nanoflann
        Boost::boost
        Boost::json
)


# Unit tests
alicevision_add_test(Tests/DelaunayGraphCut_test.cpp
    NAME "FuseCut_DelaunayGraphCut"
    LINKS
        AVFuseCut
        AVSfM
        AVMultiview
        AVMultiviewTestData
)

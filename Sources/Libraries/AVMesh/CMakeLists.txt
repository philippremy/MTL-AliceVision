# Headers
set(mesh_files_headers
    geoMesh.hpp
    Material.hpp
    Mesh.hpp
    MeshAnalyze.hpp
    MeshClean.hpp
    MeshEnergyOpt.hpp
    meshPostProcessing.hpp
    meshVisibility.hpp
    Texturing.hpp
    UVAtlas.hpp
    ModQuadricMetricT.hpp
    QuadricMetricT.hpp
    MeshIntersection.hpp
)

# Sources
set(mesh_files_sources
    Material.cpp
    Mesh.cpp
    MeshAnalyze.cpp
    MeshClean.cpp
    MeshEnergyOpt.cpp
    meshPostProcessing.cpp
    meshVisibility.cpp
    Texturing.cpp
    UVAtlas.cpp
    ModQuadricMetricT.cpp
    MeshIntersection.cpp
)

alicevision_add_library(AVMesh
    SOURCES ${mesh_files_sources}
    HEADERS ${mesh_files_headers}
    PUBLIC_LINKS
        AVMVSData
        AVMVSUtils
        Geogram::geogram
        assimp::assimp
    PRIVATE_LINKS
        AVSystem
        Boost::boost
        OpenMeshCore
)
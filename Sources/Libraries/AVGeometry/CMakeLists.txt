# Headers
set(geometry_files_headers
    Frustum.hpp
    HalfPlane.hpp
    lie.hpp
    Pose3.hpp
    rigidTransformation3D.hpp
    Similarity3.hpp
    Intersection.hpp
)

# Sources
set(geometry_files_sources
    rigidTransformation3D.cpp
    Intersection.cpp
)

alicevision_add_library(AVGeometry
    SOURCES ${geometry_files_sources}
    HEADERS ${geometry_files_headers}
    PUBLIC_LINKS
        AVNumeric
        AVLinearProgramming
    PRIVATE_LINKS
        AVRobustEstimation
)

# Unit tests
alicevision_add_test(Tests/RigidTransformation3D_test.cpp NAME "Geometry_RigidTransformation3D" LINKS AVGeometry)
alicevision_add_test(Tests/HalfSpaceIntersection_test.cpp NAME "Geometry_HalfSpaceIntersection" LINKS AVGeometry)
alicevision_add_test(Tests/Pose3D_test.cpp NAME "Geometry_Pose3D" LINKS AVGeometry)
alicevision_add_test(Tests/FrustumIntersection_test.cpp
    NAME "Geometry_FrustumIntersection"
    LINKS
        AVGeometry
        AVMultiview
        AVMultiviewTestData
)

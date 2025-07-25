# Headers
set(rig_files_headers
    Rig.hpp
    ResidualError.hpp
)

# Sources
set(rig_files_sources
    Rig.cpp
)

alicevision_add_library(AVRig
    SOURCES ${rig_files_sources}
    HEADERS ${rig_files_headers}
    PUBLIC_LINKS
        AVCamera
        AVGeometry
        AVLocalization
        AVNumeric
        Ceres::ceres
        ${SUITESPARSE_LIBRARIES}
    PRIVATE_LINKS
        AVSfM
        AVSystem
)

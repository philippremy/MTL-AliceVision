# Headers
set(sfmData_files_headers
    SfMData.hpp
    CameraPose.hpp
    Landmark.hpp
    View.hpp
    Rig.hpp
    uid.hpp
    colorize.hpp
    exif.hpp
    ImageInfo.hpp
    ExposureSetting.hpp
    Observation.hpp
    ConstraintPoint.hpp
    Constraint2D.hpp
    SurveyPoint.hpp
)

# Sources
set(sfmData_files_sources
    SfMData.cpp
    uid.cpp
    View.cpp
    colorize.cpp
    exif.cpp
    ImageInfo.cpp
    Observation.cpp
)

alicevision_add_library(AVSfMData
    SOURCES ${sfmData_files_sources}
    HEADERS ${sfmData_files_headers}
    PUBLIC_LINKS
        AVCamera
        AVGeometry
        AVFeature
        AVImage
        AVNumeric
        AVSensorDB
        AVSTL
        AVSystem
    PRIVATE_LINKS
        Boost::boost
)

# Unit tests

alicevision_add_test(Tests/SfMData_test.cpp
    NAME "SfMData_SfMData"
    LINKS
        AVSfMData
        AVSystem
)
alicevision_add_test(Tests/View_test.cpp
    NAME "SfMData_View"
    LINKS AVSfMData
)

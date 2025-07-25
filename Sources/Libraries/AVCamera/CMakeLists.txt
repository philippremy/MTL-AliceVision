# Headers
set(camera_files_headers
        camera.hpp
        cameraCommon.hpp
        cameraUndistortImage.hpp
        Distortion.hpp
        DistortionBrown.hpp
        DistortionFisheye.hpp
        DistortionFisheye1.hpp
        DistortionRadial.hpp
        Undistortion.hpp
        Undistortion3DEA4.hpp
        Undistortion3DERadial4.hpp
        Undistortion3DEClassicLD.hpp
        UndistortionRadial.hpp
        Equidistant.hpp
        Equirectangular.hpp
        IntrinsicBase.hpp
        IntrinsicInitMode.hpp
        IntrinsicScaleOffset.hpp
        IntrinsicScaleOffsetDisto.hpp
        Pinhole.hpp
)

set(camera_files_sources
        DistortionBrown.cpp
        DistortionFisheye.cpp
        DistortionFisheye1.cpp
        DistortionRadial.cpp
        UndistortionRadial.cpp
        Equidistant.cpp
        Equirectangular.cpp
        IntrinsicBase.cpp
        IntrinsicScaleOffset.cpp
        IntrinsicScaleOffsetDisto.cpp
        Pinhole.cpp
        Undistortion.cpp
        Undistortion3DEA4.cpp
        Undistortion3DERadial4.cpp
        Undistortion3DEClassicLD.cpp
)

alicevision_add_library(AVCamera
        SOURCES ${camera_files_sources}
        HEADERS ${camera_files_headers}
        PUBLIC_LINKS
        AVNumeric
        AVGeometry
        AVImage
        AVSTL
        AVSystem
)

# Unit tests
alicevision_add_test(Tests/Distortion_test.cpp        NAME "Camera_DistortionRadial"    LINKS AVCamera)
alicevision_add_test(Tests/PinholeBrown_test.cpp      NAME "Camera_PinholeBrown"        LINKS AVCamera)
alicevision_add_test(Tests/PinholeFisheye_test.cpp    NAME "Camera_PinholeFisheye"      LINKS AVCamera)
alicevision_add_test(Tests/PinholeFisheye1_test.cpp   NAME "Camera_PinholeFisheye1"     LINKS AVCamera)
alicevision_add_test(Tests/PinholeRadial_test.cpp     NAME "Camera_PinholeRadial"       LINKS AVCamera)
alicevision_add_test(Tests/Equidistant_test.cpp       NAME "Camera_Equidistant"         LINKS AVCamera)
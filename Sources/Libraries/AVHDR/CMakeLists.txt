
# Headers
set(hdr_files_headers
    LaguerreBACalibration.hpp
    sampling.hpp
    rgbCurve.hpp
    hdrMerge.hpp
    DebevecCalibrate.hpp
    GrossbergCalibrate.hpp
    emorCurve.hpp
    QuadProg++.hpp
    brackets.hpp
)

# Sources
set(hdr_files_sources
    LaguerreBACalibration.cpp
    sampling.cpp
    rgbCurve.cpp
    hdrMerge.cpp
    DebevecCalibrate.cpp
    GrossbergCalibrate.cpp
    emorCurve.cpp
    emorCurveInv.cpp
    QuadProg++.cpp
    brackets.cpp
)

alicevision_add_library(AVHDR
    SOURCES ${hdr_files_sources}
    HEADERS ${hdr_files_headers}
    PRIVATE_LINKS
        AVSystem
        AVImage
        AVSfMData
        Ceres::ceres
        ${SUITESPARSE_LIBRARIES}
        ${OpenCV_LIBS}
)

# Unit tests
alicevision_add_test(Tests/HDRDebevec_test.cpp
    NAME "HDR_debevec"
    LINKS AVImage AVHDR ${OpenCV_LIBS}
)

alicevision_add_test(Tests/HDRGrossberg_test.cpp
    NAME "HDR_grossberg"
    LINKS AVImage AVHDR ${OpenCV_LIBS}
)

alicevision_add_test(Tests/HDRLaguerre_test.cpp
    NAME "HDR_laguerre"
    LINKS AVImage AVHDR ${OpenCV_LIBS}
)

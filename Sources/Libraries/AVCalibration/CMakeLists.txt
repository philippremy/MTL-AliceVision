# Headers
set(calibration_files_headers
        calibration.hpp
        bestImages.hpp
        patternDetect.hpp
        exportData.hpp
        distortionEstimation.hpp
        distortionEstimationLine.hpp
        distortionEstimationPoint.hpp
        distortionEstimationGeometry.hpp
        checkerDetector.hpp
        checkerDetector_io.hpp
)

# Sources
set(calibration_files_sources
        distortionEstimationLine.cpp
        distortionEstimationPoint.cpp
        distortionEstimationGeometry.cpp
        checkerDetector.cpp
        checkerDetector_io.cpp
)

if (ALICEVISION_HAVE_OPENCV)
    list(APPEND calibration_files_sources
            bestImages.cpp
            calibration.cpp
            patternDetect.cpp
            exportData.cpp
    )
endif()

alicevision_add_library(AVCalibration
        SOURCES ${calibration_files_sources}
        HEADERS ${calibration_files_headers}
        PRIVATE_LINKS
        AVImage
        AVSystem
        AVDataIO
        Boost::program_options
        Boost::boost
        Boost::json
        Ceres::ceres
        ${SuiteSparse_LIBRARIES}
)

if (ALICEVISION_HAVE_OPENCV)
    target_link_libraries(AVCalibration PUBLIC ${OpenCV_LIBS})
endif()

if (ALICEVISION_HAVE_CCTAG)
    target_link_libraries(AVCalibration PRIVATE CCTag::CCTag)
endif()
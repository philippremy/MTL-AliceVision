# Headers
set(features_files_headers
    akaze/AKAZE.hpp
    akaze/descriptorLIOP.hpp
    akaze/descriptorMLDB.hpp
    akaze/descriptorMSURF.hpp
    akaze/ImageDescriber_AKAZE.hpp
    sift/ImageDescriber_SIFT.hpp
    sift/ImageDescriber_SIFT_vlfeat.hpp
    sift/ImageDescriber_SIFT_vlfeatFloat.hpp
    sift/ImageDescriber_DSPSIFT_vlfeat.hpp
    sift/SIFT.hpp
    Descriptor.hpp
    feature.hpp
    FeaturesPerView.hpp
    Hamming.hpp
    ImageDescriber.hpp
    imageDescriberCommon.hpp
    imageStats.hpp
    KeypointSet.hpp
    metric.hpp
    PointFeature.hpp
    Regions.hpp
    regionsFactory.hpp
    RegionsPerView.hpp
)

# Sources
set(features_files_sources
    akaze/AKAZE.cpp
    akaze/descriptorLIOP.cpp
    akaze/ImageDescriber_AKAZE.cpp
    sift/SIFT.cpp
    sift/ImageDescriber_DSPSIFT_vlfeat.cpp
    FeaturesPerView.cpp
    ImageDescriber.cpp
    imageDescriberCommon.cpp
    imageStats.cpp
)

# CCTAG ImageDescriber
if (ALICEVISION_HAVE_CCTAG)
    list(APPEND features_files_headers cctag/ImageDescriber_CCTAG.hpp)
    list(APPEND features_files_sources cctag/ImageDescriber_CCTAG.cpp)
endif()

# APRILTAG ImageDescriber
if (ALICEVISION_HAVE_APRILTAG)
    list(APPEND features_files_headers apriltag/ImageDescriber_APRILTAG.hpp)
    list(APPEND features_files_sources apriltag/ImageDescriber_APRILTAG.cpp)
endif()

# PopSIFT ImageDescriber
if (ALICEVISION_HAVE_POPSIFT)
    list(APPEND features_files_headers sift/ImageDescriber_SIFT_popSIFT.hpp)
    list(APPEND features_files_sources sift/ImageDescriber_SIFT_popSIFT.cpp)
endif()

# OpenCV ImageDescriber and SIFT
if (ALICEVISION_HAVE_OPENCV)
    list(APPEND features_files_headers openCV/ImageDescriber_AKAZE_OCV.hpp openCV/ImageDescriber_SIFT_OCV.hpp)
    list(APPEND features_files_sources openCV/ImageDescriber_AKAZE_OCV.cpp openCV/ImageDescriber_SIFT_OCV.cpp)
endif()


alicevision_add_library(AVFeature
    SOURCES ${features_files_sources}
    HEADERS ${features_files_headers}
    PUBLIC_LINKS
        AVImage
        AVNumeric
        AVSystem
        AVGPU
        VLFeat
    PRIVATE_LINKS
        Boost::boost
)

# Link CCTAG library
if (ALICEVISION_HAVE_CCTAG)
    target_link_libraries(AVFeature PUBLIC CCTag::CCTag)
endif()

# Link APRILTAG library
if (ALICEVISION_HAVE_APRILTAG)
    target_link_libraries(AVFeature PUBLIC apriltag::apriltag)
endif()

# Link PopSIFT library
if (ALICEVISION_HAVE_POPSIFT)
    target_link_libraries(AVFeature PRIVATE PopSift::popsift)
endif()

# Link OpenCV library
if (ALICEVISION_HAVE_OPENCV)
    target_link_libraries(AVFeature PUBLIC ${OpenCV_LIBS})
endif()

# Unit tests
alicevision_add_test(Tests/Features_test.cpp NAME "Feature_Features"            LINKS AVFeature)
alicevision_add_test(Tests/Metric_test.cpp   NAME "Feature_Descriptor_metric"   LINKS AVFeature)

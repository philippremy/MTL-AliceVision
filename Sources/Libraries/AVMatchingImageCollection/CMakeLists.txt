# Headers
set(matching_collection_images_files_headers
    matchingCommon.hpp
    IImageCollectionMatcher.hpp
    ImageCollectionMatcher_generic.hpp
    ImageCollectionMatcher_cascadeHashing.hpp
    GeometricFilter.hpp
    GeometricFilterMatrix.hpp
    GeometricFilterMatrix_E_AC.hpp
    GeometricFilterMatrix_F_AC.hpp
    GeometricFilterMatrix_H_AC.hpp
    GeometricFilterMatrix_HGrowing.hpp
    GeometricFilterType.hpp
    ImagePairListIO.hpp
    geometricFilterUtils.hpp
    pairBuilder.hpp
)

# Sources
set(matching_collection_images_files_sources
    matchingCommon.cpp
    ImageCollectionMatcher_generic.cpp
    ImageCollectionMatcher_cascadeHashing.cpp
    GeometricFilter.cpp
    GeometricFilterMatrix_HGrowing.cpp
    geometricFilterUtils.cpp
    ImagePairListIO.cpp
    pairBuilder.cpp
)

alicevision_add_library(AVMatchingImageCollection
    SOURCES ${matching_collection_images_files_sources}
    HEADERS ${matching_collection_images_files_headers}
    PUBLIC_LINKS
        AVFeature
        AVMatching
        AVMultiview
        AVRobustEstimation
        AVSfMData
        Boost::boost
        Boost::timer
    PRIVATE_LINKS
        AVSystem
        Ceres::ceres
        ${SUITESPARSE_LIBRARIES}
        flann::flann_cpp
)

# Unit tests
alicevision_add_test(Tests/ImagePairListIO_test.cpp
    NAME "MatchingImageCollection_ImagePairListIO"
    LINKS AVMatchingImageCollection)

alicevision_add_test(Tests/PairBuilder_test.cpp
    NAME "MatchingImageCollection_PairBuilder"
    LINKS AVMatchingImageCollection
)
alicevision_add_test(Tests/GeometricFilterUtils_test.cpp
    NAME "MatchingImageCollection_GeometricFilterUtils"
    LINKS AVMatchingImageCollection
)

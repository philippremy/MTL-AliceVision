# Headers
set(localization_files_headers
    LocalizationResult.hpp
    VoctreeLocalizer.hpp
    optimization.hpp
    reconstructed_regions.hpp
    ILocalizer.hpp
    rigResection.hpp
)

# Sources
set(localization_files_sources
    LocalizationResult.cpp
    VoctreeLocalizer.cpp
    optimization.cpp
    rigResection.cpp
)

if (ALICEVISION_HAVE_CCTAG)
    list(APPEND localization_files_headers CCTagLocalizer.hpp)
    list(APPEND localization_files_sources CCTagLocalizer.cpp)
endif()

alicevision_add_library(AVLocalization
    SOURCES ${localization_files_sources}
    HEADERS ${localization_files_headers}
    PUBLIC_LINKS
        AVCamera
        AVFeature
        AVGeometry
        AVImage
        AVMatching
        AVNumeric
        AVRobustEstimation
        AVSfM
        AVVoctree
    PRIVATE_LINKS
        AVSystem
        AVMatchingImageCollection
        Boost::boost
        flann::flann_cpp
)

if (ALICEVISION_HAVE_CCTAG)
    target_link_libraries(AVLocalization PRIVATE CCTag::CCTag)
endif()

if (ALICEVISION_HAVE_OPENGV)
    target_link_libraries(AVLocalization PRIVATE opengv)
endif()

# Unit tests
alicevision_add_test(Tests/LocalizationResult_test.cpp NAME "Localization_LocalizationResult" LINKS AVLocalization)

if (ALICEVISION_HAVE_OPENGV)
    alicevision_add_test(Tests/RigResection_test.cpp NAME "Localization_RigResection" LINKS AVLocalization)
endif()

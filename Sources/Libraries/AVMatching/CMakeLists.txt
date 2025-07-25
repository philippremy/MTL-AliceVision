# Headers
set(matching_files_headers
    ArrayMatcher.hpp
    ArrayMatcher_bruteForce.hpp
    ArrayMatcher_cascadeHashing.hpp
    ArrayMatcher_kdtreeFlann.hpp
    IndMatch.hpp
    MatchesCollections.hpp
    IndMatchDecorator.hpp
    filters.hpp
    guidedMatching.hpp
    io.hpp
    matcherType.hpp
    CascadeHasher.hpp
    RegionsMatcher.hpp
    pairwiseAdjacencyDisplay.hpp
    supportEstimation.hpp
    matchesFiltering.hpp
    svgVisualization.hpp
)

# Sources
set(matching_files_sources
    io.cpp
    guidedMatching.cpp
    matcherType.cpp
    RegionsMatcher.cpp
    supportEstimation.cpp
    matchesFiltering.cpp
    svgVisualization.cpp
)

alicevision_add_library(AVMatching
    SOURCES ${matching_files_sources}
    HEADERS ${matching_files_headers}
    PUBLIC_LINKS
        AVCamera
        AVFeature
        AVNumeric
        AVSTL
        AVSystem
    PRIVATE_LINKS
        Boost::boost
        flann::flann_cpp
)

# Unit tests
alicevision_add_test(Tests/Matching_test.cpp NAME "Matching_Matching"   LINKS AVMatching flann::flann_cpp)
alicevision_add_test(Tests/Filters_test.cpp  NAME "Matching_Filters"    LINKS AVMatching)
alicevision_add_test(Tests/IndMatch_test.cpp NAME "Matching_IndMatch"   LINKS AVMatching)

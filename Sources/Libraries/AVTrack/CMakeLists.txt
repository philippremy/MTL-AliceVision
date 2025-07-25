# Headers
set(tracks_files_headers
    Track.hpp
    TracksBuilder.hpp
    TracksHandler.hpp
    tracksUtils.hpp
    trackIO.hpp
    TracksMerger.hpp
)

# Sources
set(tracks_files_sources
    TracksBuilder.cpp
    TracksHandler.cpp
    tracksUtils.cpp
    trackIO.cpp
    TracksMerger.cpp
)

alicevision_add_library(AVTrack
    SOURCES ${tracks_files_sources}
    HEADERS ${tracks_files_headers}
    PUBLIC_LINKS
        AVFeature
        AVMatching
        AVSTL
        Boost::json
    PRIVATE_LINKS
        ${LEMON_LIBRARY}
)

# Unit tests
alicevision_add_test(Tests/Track_test.cpp NAME "Track_Track" LINKS AVTrack)

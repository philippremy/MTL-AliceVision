# Headers
set(dataio_files_headers
    FeedProvider.hpp
    IFeed.hpp
    ImageFeed.hpp
    SfMDataFeed.hpp
    json.hpp
    E57Reader.hpp
)

# Sources
set(dataio_files_sources
    FeedProvider.cpp
    IFeed.cpp
    ImageFeed.cpp
    SfMDataFeed.cpp
    json.cpp
    E57Reader.cpp
)

if (ALICEVISION_HAVE_OPENCV)
    list(APPEND dataio_files_headers VideoFeed.hpp)
    list(APPEND dataio_files_sources VideoFeed.cpp)
endif()

alicevision_add_library(AVDataIO
    SOURCES ${dataio_files_sources}
    HEADERS ${dataio_files_headers}
    PUBLIC_LINKS
        AVCamera
        AVImage
    PRIVATE_LINKS
        AVSfMData
        AVSfMDataIO
        AVSystem
        Boost::boost
        Boost::json
        E57Format
)

if (ALICEVISION_HAVE_OPENCV)
    target_link_libraries(AVDataIO PRIVATE ${OpenCV_LIBS})
endif()

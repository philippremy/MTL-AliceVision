# Headers
set(keyframe_files_headers
    KeyframeSelector.hpp
)

# Sources
set(keyframe_files_sources
    KeyframeSelector.cpp
)

alicevision_add_library(AVKeyframe
    SOURCES ${keyframe_files_sources}
    HEADERS ${keyframe_files_headers}
    PUBLIC_LINKS
        AVDataIO
        AVSfMData
        AVSfMDataIO
        OpenImageIO::OpenImageIO
    PRIVATE_LINKS
        AVSensorDB
        AVSystem
)

if (ALICEVISION_HAVE_OPENCV)
    target_link_libraries(AVKeyframe PUBLIC ${OpenCV_LIBS})
endif()
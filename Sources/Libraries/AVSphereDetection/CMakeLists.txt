# Headers
set(sphereDetection_headers
    sphereDetection.hpp
)

# Sources
set(sphereDetection_sources
    sphereDetection.cpp
)

alicevision_add_library(AVSphereDetection
    SOURCES ${sphereDetection_sources}
    HEADERS ${sphereDetection_headers}
    PUBLIC_LINKS
        AVImage
        onnxruntime::onnxruntime
        ${OpenCV_LIBS}
)

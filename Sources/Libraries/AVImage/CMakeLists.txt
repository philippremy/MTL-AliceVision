# Headers
set(image_files_headers
    all.hpp
    Image.hpp
    imageAlgo.hpp
    colorspace.hpp
    concat.hpp
    conversion.hpp
    convolutionBase.hpp
    convolution.hpp
    dcp.hpp
    diffusion.hpp
    drawing.hpp
    filtering.hpp
    io.hpp
    jetColorMap.hpp
    resampling.hpp
    warping.hpp
    pixelTypes.hpp
    Rgb.hpp
    Sampler.hpp
    cache.hpp
)

# Sources
set(image_files_sources
    colorspace.cpp
    convolution.cpp
    dcp.cpp
    filtering.cpp
    io.cpp
    imageAlgo.cpp
    jetColorMap.cpp
    cache.cpp
)

alicevision_add_library(AVImage
    SOURCES ${image_files_sources}
    HEADERS ${image_files_headers}
    PUBLIC_LINKS
        AVNumeric
        OpenImageIO::OpenImageIO
        OpenImageIO::OpenImageIO_Util
    PRIVATE_LINKS
        AVSystem
        OpenEXR::OpenEXR
        ${OpenCV_LIBS}
    RESOURCES
        "./Resources/config.ocio"
        "./Resources/LUTs"
)

# On Apple, link CoreFoundation for bundle helpers
if(APPLE)
    target_link_libraries(AVImage PUBLIC "-framework CoreFoundation")
endif()

# Unit tests
alicevision_add_test(Tests/Image_test.cpp        NAME "Image_Image"            LINKS AVImage ${OpenCV_LIBS})
alicevision_add_test(Tests/IO_test.cpp           NAME "Image_Image_io"         LINKS AVImage ${OpenCV_LIBS})
alicevision_add_test(Tests/Drawing_test.cpp      NAME "Image_Image_drawing"    LINKS AVImage ${OpenCV_LIBS})
alicevision_add_test(Tests/Filtering_test.cpp    NAME "Image_Image_filtering"  LINKS AVImage ${OpenCV_LIBS})
alicevision_add_test(Tests/Resampling_test.cpp   NAME "Image_Image_resampling" LINKS AVImage ${OpenCV_LIBS})

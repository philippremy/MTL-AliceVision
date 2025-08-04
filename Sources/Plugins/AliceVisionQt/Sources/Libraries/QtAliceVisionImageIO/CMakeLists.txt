# Target srcs
set(PLUGIN_SOURCES
    plugin.cpp
    QtAliceVisionImageIOHandler.cpp
)

set(PLUGIN_HEADERS
    plugin.hpp
    QtAliceVisionImageIOHandler.hpp
)

alicevision_add_plugin(QtAliceVisionImageIO
    SOURCES
        ${PLUGIN_SOURCES}
	HEADERS
	    ${PLUGIN_HEADERS}
    PUBLIC_LINKS
        ${OPENIMAGEIO_LIBRARIES}
        AVImage
        Qt6::Core
        Qt6::Gui
    PUBLIC_INCLUDE_DIRS
        ${OPENIMAGEIO_INCLUDE_DIRS}
)

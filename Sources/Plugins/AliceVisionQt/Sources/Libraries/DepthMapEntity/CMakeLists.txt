# Target srcs
set(PLUGIN_SOURCES
    DepthMapEntity.cpp
)

set(PLUGIN_HEADERS
    plugin.hpp
    DepthMapEntity.hpp
)

alicevision_add_plugin(DepthMapEntity
    SOURCES
        ${PLUGIN_SOURCES}
	HEADERS
	    ${PLUGIN_HEADERS}
    PUBLIC_LINKS
        ${OPENIMAGEIO_LIBRARIES}
        AVImage
        AVMVSData
        AVNumeric
        Qt6::Core
        Qt6::Gui
        Qt6::Qml
        Qt6::Quick
        Qt6::3DCore
        Qt6::3DRender
        Qt6::3DExtras
    PUBLIC_INCLUDE_DIRS
        ${OPENIMAGEIO_INCLUDE_DIRS}
    RESOURCES
        "qmldir"
)

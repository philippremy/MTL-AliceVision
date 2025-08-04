# Target srcs
set(PLUGIN_SOURCES
    IOThread.cpp
    SfmDataEntity.cpp
    CameraLocatorEntity.cpp
    PointCloudEntity.cpp
)

set(PLUGIN_HEADERS
    IOThread.hpp
    plugin.hpp
    SfmDataEntity.hpp
    CameraLocatorEntity.hpp
    PointCloudEntity.hpp
)

alicevision_add_plugin(SfMDataEntityQML
    SOURCES
        ${PLUGIN_SOURCES}
	HEADERS
	    ${PLUGIN_HEADERS}
    PUBLIC_LINKS
        AVSfM
        AVSfMDataIO
        AVSystem
        Qt6::Core
        Qt6::Qml
        Qt6::3DCore
        Qt6::3DRender
    PRIVATE_LINKS
        Qt6::3DExtras
    RESOURCES
        "qmldir"
)

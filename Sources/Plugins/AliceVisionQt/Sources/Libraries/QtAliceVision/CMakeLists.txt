# Target srcs
set(PLUGIN_SOURCES
    FeaturesViewer.cpp
    MFeatures.cpp
    MSfMData.cpp
    MViewStats.cpp
    MTracks.cpp
    FloatImageViewer.cpp
    FloatTexture.cpp
    Surface.cpp
    MSfMDataStats.cpp
    PanoramaViewer.cpp
    PhongImageViewer.cpp
    Painter.cpp
    SequenceCache.cpp
    SingleImageLoader.cpp
    ImageCache.cpp
    AsyncFetcher.cpp
)

set(PLUGIN_HEADERS
    plugin.hpp
    FeaturesViewer.hpp
    MFeatures.hpp
    MSfMData.hpp
    MTracks.hpp
    MViewStats.hpp
    FloatImageViewer.hpp
    FloatTexture.hpp
    MSfMDataStats.hpp
    PanoramaViewer.hpp
    PhongImageViewer.hpp
    Surface.hpp
    Painter.hpp
    ImageServer.hpp
    SequenceCache.hpp
    SingleImageLoader.hpp
    ImageCache.hpp
    AsyncFetcher.hpp
)

set(PLUGIN_MOCS
    FeaturesViewer.hpp
    MFeatures.hpp
    MSfMData.hpp
    MViewStats.hpp
    MTracks.hpp
    MSfMDataStats.hpp
    SequenceCache.hpp
    SingleImageLoader.hpp
    AsyncFetcher.hpp
)

alicevision_add_plugin(QtAliceVision
    SOURCES
        ${PLUGIN_SOURCES}
	HEADERS
	    ${PLUGIN_HEADERS}
		${PLUGIN_MOCS}
    PUBLIC_LINKS
        AVFeature
        AVSfMDataIO
        AVSfM
        AVSystem
        AVImage
        Qt6::Core
        Qt6::Qml
        Qt6::Quick
        Qt6::Charts
        Qt6::GuiPrivate
    RESOURCES
        "qmldir"
)

qt6_add_shaders(QtAliceVision "QtAliceVisionShaders"
    BATCHABLE
    PRECOMPILE
    OPTIMIZED
    PREFIX "/Shaders"
    FILES FloatImageViewer.vert FloatImageViewer.frag
          FeaturesViewer.vert FeaturesViewer.frag
          PhongImageViewer.vert PhongImageViewer.frag
          ImageOverlay.frag
          AttributeItemDelegate.frag
)

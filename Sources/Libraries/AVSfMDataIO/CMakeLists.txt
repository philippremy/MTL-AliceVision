# Headers
set(sfmDataIO_files_headers
    sfmDataIO.hpp
    bafIO.hpp
    colmap.hpp
    gtIO.hpp
    jsonIO.hpp
    middlebury.hpp
    plyIO.hpp
    viewIO.hpp
    sceneSample.hpp
)

# Sources
set(sfmDataIO_files_sources
    sfmDataIO.cpp
    bafIO.cpp
    colmap.cpp
    gtIO.cpp
    jsonIO.cpp
    middlebury.cpp
    plyIO.cpp
    viewIO.cpp
    sceneSample.cpp
)

if (ALICEVISION_HAVE_ALEMBIC)
    list(APPEND sfmDataIO_files_headers
        AlembicExporter.hpp
        AlembicImporter.hpp
        ExternalAlembicImporter.hpp
    )
    list(APPEND sfmDataIO_files_sources
        AlembicExporter.cpp
        AlembicImporter.cpp
        ExternalAlembicImporter.cpp
    )
endif()

alicevision_add_library(AVSfMDataIO
    SOURCES ${sfmDataIO_files_sources}
    HEADERS ${sfmDataIO_files_headers}
    PUBLIC_LINKS
        AVSfMData
        AVCamera
    PRIVATE_LINKS
        assimp::assimp
        AVImage
        Boost::boost
        Boost::regex
    RESOURCES
        "./Resources/CompatibilityData"
)

if (ALICEVISION_HAVE_ALEMBIC)
    target_link_libraries(AVSfMDataIO
        PRIVATE Alembic::Alembic
    )
endif()

# Unit tests

alicevision_add_test(Tests/SfMDataIO_test.cpp
    NAME "SfMDataIO_SfMDataIO"
    LINKS
        AVFeature
        AVSfMData
        AVSfMDataIO
        AVSystem
)

alicevision_add_test(Tests/SfMDataIOCompatibilityJson_test.cpp
    NAME "SfMDataIO_SfMDataIOCompatibilityJson"
    LINKS
        AVFeature
        AVSfMData
        AVSfMDataIO
        AVSystem
)

if (ALICEVISION_HAVE_ALEMBIC)
    alicevision_add_test(Tests/SfMDataIOCompatibilityAbc_test.cpp
        NAME "SfMDataIO_SfMDataIOCompatibilityAbc"
        LINKS
            AVFeature
            AVSfMData
            AVSfMDataIO
            AVSystem
    )
    alicevision_add_test(Tests/AlembicIO_test.cpp
        NAME "SfMDataIO_SfMDataIO_alembic"
        LINKS
            AVSfM
            AVSfMDataIO
            Alembic::Alembic
    )
endif()


alicevision_add_test(Tests/Middlebury_test.cpp
    NAME "SfMDataIO_SfMDataIO_middlebury"
    LINKS
        AVNumeric
        AVSfMData
        AVSfMDataIO
)

alicevision_add_test(Tests/Colmap_test.cpp
    NAME "SfMDataIO_SfMDataIO_colmap"
    LINKS
        AVNumeric
        AVSfMData
        AVSfMDataIO
)

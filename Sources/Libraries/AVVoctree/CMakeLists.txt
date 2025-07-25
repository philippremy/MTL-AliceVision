# Headers
set(voctree_headers
    Database.hpp
    databaseIO.hpp
    descriptorLoader.hpp
    descriptorLoader.tcc
    distance.hpp
    DefaultAllocator.hpp
    MutableVocabularyTree.hpp
    SimpleKmeans.hpp
    TreeBuilder.hpp
    VocabularyTree.hpp
)

# Sources
set(voctree_sources
    Database.cpp
    descriptorLoader.cpp
    VocabularyTree.cpp
)

alicevision_add_library(AVVoctree
    SOURCES ${voctree_sources}
    HEADERS ${voctree_headers}
    PUBLIC_LINKS
        AVFeature
        AVSfMData
        AVSystem
        Boost::boost
)

# Unit tests
alicevision_add_test(Tests/Kmeans_test.cpp
    NAME "Voctree_Kmeans"
    LINKS AVVoctree
)
alicevision_add_test(Tests/VocabularyTree_test.cpp
    NAME "Voctree_VocabularyTree"
    LINKS AVVoctree
)
alicevision_add_test(Tests/VocabularyTreeBuild_test.cpp
    NAME "Voctree_VocabularyTreeBuild"
    LINKS AVVoctree
)

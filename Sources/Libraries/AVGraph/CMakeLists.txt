# Headers
set(graph_files_headers
        graph.hpp
        connectedComponent.hpp
        IndexedGraph.hpp
        indexedGraphGraphvizExport.hpp
        Triplet.hpp
)

alicevision_add_interface(AVGraph
        HEADERS ${graph_files_headers}
        LINKS
        AVSystem
)

# Unit tests
alicevision_add_test(Tests/ConnectedComponent_test.cpp NAME "Graph_ConnectedComponent" LINKS AVGraph ${LEMON_LIBRARY})
alicevision_add_test(Tests/Triplet_test.cpp            NAME "Graph_Triplet"            LINKS AVGraph ${LEMON_LIBRARY})
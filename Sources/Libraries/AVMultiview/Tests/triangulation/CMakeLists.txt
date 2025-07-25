alicevision_add_test(TriangulationDLT_test.cpp
    NAME "Multiview_TriangulationDLT"
    LINKS AVMultiview AVMultiviewTestData
)
alicevision_add_test(Triangulation_test.cpp
    NAME "Multiview_Triangulation"
    LINKS AVMultiview AVMultiviewTestData
)

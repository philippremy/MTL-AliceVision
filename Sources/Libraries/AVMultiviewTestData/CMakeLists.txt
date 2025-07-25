# Test Data Sources
set(multiview_sources_test_data
    NViewDataSet.cpp
)

# Test Data Headers
set(multiview_headers_test_data
    NViewDataSet.hpp
)

alicevision_add_library(AVMultiviewTestData
    SOURCES ${multiview_sources_test_data}
    HEADERS ${multiview_headers_test_data}
    PUBLIC_LINKS
        AVNumeric
)
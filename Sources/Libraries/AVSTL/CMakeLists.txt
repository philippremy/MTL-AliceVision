# Headers
set(stl_files_headers
        bitmask.hpp
        DynamicBitset.hpp
        FlatMap.hpp
        hash.hpp
        indexedSort.hpp
        stl.hpp
        mapUtils.hpp
)

alicevision_add_interface(AVSTL
        HEADERS ${stl_files_headers}
        LINKS Boost::container
)

# Unit tests
alicevision_add_test(Tests/DynamicBitset_test.cpp NAME "STL_DynamicBitset" LINKS AVSTL)
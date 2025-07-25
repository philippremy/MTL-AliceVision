# Headers
set(utils_files_headers
        Histogram.hpp
        regexFilter.hpp
        filesIO.hpp
)

alicevision_add_interface(AVUtils
        HEADERS ${utils_files_headers}
)

# Unit tests
alicevision_add_test(Tests/RegexFilter_test.cpp NAME "Utils_RegexFilter" LINKS AVUtils)
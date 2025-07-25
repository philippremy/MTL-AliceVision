# Headers
set(numeric_files_headers
    numeric.hpp
    Container.hpp
    projection.hpp
    gps.hpp
    algebra.hpp
    BoxStats.hpp
)

# Sources
set(numeric_files_sources
    numeric.cpp
    Container.cpp
    projection.cpp
    gps.cpp
)

alicevision_add_library(AVNumeric
    SOURCES ${numeric_files_sources}
    HEADERS ${numeric_files_headers}
    PUBLIC_LINKS
        Eigen3::Eigen
        Boost::boost
        Boost::log
        Boost::log_setup
    PUBLIC_DEFINITIONS
        ${ALICEVISION_EIGEN_DEFINITIONS}
)

# Unit tests
alicevision_add_test(Tests/Numeric_test.cpp
    NAME "Numeric_Numeric"
    LINKS AVNumeric
)
alicevision_add_test(Tests/Polynomial_test.cpp
    NAME "Numeric_Polynomial"
    LINKS AVNumeric
)
alicevision_add_test(Tests/LMFunctor_test.cpp
    NAME "Numeric_LMFunctor"
    LINKS AVNumeric
)
alicevision_add_test(Tests/Projection_test.cpp
    NAME "Numeric_Projection"
    LINKS AVNumeric
)
alicevision_add_test(Tests/GPS_test.cpp
    NAME "Numeric_GPS"
    LINKS AVNumeric
)

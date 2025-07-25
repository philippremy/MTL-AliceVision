
# The tests are split into multiple executables so that it's possible to easily run the tests
# in parallel using ctest -jX

alicevision_add_test(PanoramaSfM_radial3_test.cpp
    NAME "SfM_Panorama_radial3"
    LINKS
        AVFeature
        AVMatching
        AVSfM
        AVSystem
)

alicevision_add_test(PanoramaSfM_radial3_outliers_test.cpp
    NAME "SfM_Panorama_radial3_outliers"
    LINKS
        AVFeature
        AVMatching
        AVSfM
        AVSystem
)

alicevision_add_test(PanoramaSfM_equidistant_test.cpp
    NAME "SfM_Panorama_equidistant"
    LINKS
        AVFeature
        AVMatching
        AVSfM
        AVSystem
)

alicevision_add_test(PanoramaSfM_equidistant_outliers_test.cpp
    NAME "SfM_Panorama_equidistant_outliers"
    LINKS
        AVFeature
        AVMatching
        AVSfM
        AVSystem
)

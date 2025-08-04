if(BUILD_IMAGEIO)
    add_subdirectory(Libraries/QtAliceVisionImageIO)
endif()

if(BUILD_DEPTHMAPENTITY)
    add_subdirectory(Libraries/DepthMapEntity)
endif()

if(BUILD_SFM)
    add_subdirectory(Libraries/QtAliceVision)
    add_subdirectory(Libraries/SfMDataEntityQML)
endif()

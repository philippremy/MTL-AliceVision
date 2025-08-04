project(QtAliceVision LANGUAGES CXX VERSION ${ALICEVISION_VERSION})

# Build options
option(BUILD_IMAGEIO "Build QtAliceVisionImageIO plugin" ON)
option(BUILD_DEPTHMAPENTITY "Build DepthMapEntity plugin" ON)
option(BUILD_SFM "Build QtAliceVision and QMLSfMData plugin" ON)

set(CMAKE_AUTOMOC ON)
add_definitions(-DQT_PLUGIN)
add_definitions(-DQT_SHARED)
add_definitions(-DQT_NO_DEBUG)
add_definitions(-DQT_NO_KEYWORDS)

# for performance reasons let Qt handle jpeg, png and ico formats
add_definitions(-DQT_ALICEVISIONIMAGEIO_USE_FORMATS_BLACKLIST)

# Enable AUTOMOC
set(CMAKE_AUTOMOC ON)

# Builds
add_subdirectory(Sources)

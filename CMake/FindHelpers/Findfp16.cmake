include(CheckIncludeFileCXX)
include(FindPackageHandleStandardArgs)

# List of required headers
set(_FP16_REQUIRED_HEADERS
        "fp16.h"
        "fp16/fp16.h"
        "fp16/bitcasts.h"
        "fp16/macros.h"
)

# Search for each header in all CMAKE_PREFIX_PATH/include locations
set(fp16_INCLUDE_DIRS "")
set(_fp16_MISSING_HEADERS "")
set(fp16_FOUND TRUE)

foreach(_hdr IN LISTS _FP16_REQUIRED_HEADERS)
    find_path(_fp16_${_hdr}_DIR
            NAMES ${_hdr}
            HINTS ${CMAKE_PREFIX_PATH}
            PATH_SUFFIXES include
    )
    if(_fp16_${_hdr}_DIR)
        list(APPEND fp16_INCLUDE_DIRS ${_fp16_${_hdr}_DIR})
    else()
        set(fp16_FOUND FALSE)
        list(APPEND _fp16_MISSING_HEADERS ${_hdr})
    endif()
endforeach()

# Remove duplicate paths
list(REMOVE_DUPLICATES fp16_INCLUDE_DIRS)

# Use the standard handler to set fp16_FOUND
find_package_handle_standard_args(fp16
        REQUIRED_VARS fp16_FOUND
        FAIL_MESSAGE "Missing required fp16 headers: ${_fp16_MISSING_HEADERS}"
)

# Create an interface target if found
if(fp16_FOUND AND NOT TARGET fp16)
    add_library(fp16 INTERFACE)
    target_include_directories(fp16 INTERFACE ${fp16_INCLUDE_DIRS})
endif()
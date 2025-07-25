# Check for Resources in the specified folder and create symlinks in the destination folder

file(GLOB RESOURCES_FOUND "${SOURCE_FRAMEWORK}/*")

# Filter out Info.plist
set(FILTERED_RESOURCES "")
foreach(FILE ${RESOURCES_FOUND})
    get_filename_component(NAME "${FILE}" NAME)
    if(NOT NAME STREQUAL "Info.plist")
        list(APPEND FILTERED_RESOURCES "${FILE}")
    endif()
endforeach()

foreach(RESOURCE IN LISTS FILTERED_RESOURCES)
    # Get the base file name of the resource
    get_filename_component(RESOURCE_NAME "${RESOURCE}" NAME)
    file(CREATE_LINK "${RESOURCE}" "${DESTINATION}/${RESOURCE_NAME}" SYMBOLIC)
endforeach()
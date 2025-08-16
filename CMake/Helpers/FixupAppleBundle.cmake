# This fixes up the dependencies in a pre-populated macOS bundle

# Extract all executables, libraries and frameworks in the bundle

# Binaries are easy, so do them first
file(GLOB BUNDLE_BINARIES "${BUNDLE}/Contents/MacOS/*")
# Filter out the symlinks
set(BUNDLE_BINARIES_NOSYMLINKS)
foreach(FILE IN LISTS BUNDLE_BINARIES)
    if(NOT IS_SYMLINK "${FILE}")
        list(APPEND BUNDLE_BINARIES_NOSYMLINKS "${FILE}")
    endif()
endforeach()

# Libraries can be either .dylib files (then collect them directly) or .frameworks, then we need to get the actual binary inside
# Frameworks are assumed to have the same binary name like their folder name
file(GLOB BUNDLE_LIBRARIES "${BUNDLE}/Contents/Frameworks/*.dylib")
file(GLOB BUNDLE_FRAMEWORKS "${BUNDLE}/Contents/Frameworks/*.framework")
file(GLOB_RECURSE BUNDLE_PLUGINS "${BUNDLE}/Contents/PlugIns/**/*.dylib")
# Filter out symlinks
set(BUNDLE_LIBRARIES_NOSYMLINKS)
foreach(FILE IN LISTS BUNDLE_LIBRARIES)
    if(NOT IS_SYMLINK "${FILE}")
        list(APPEND BUNDLE_LIBRARIES_NOSYMLINKS "${FILE}")
    endif()
endforeach()
# For Frameworks, extract their name and get the current executable inside
set(BUNDLE_FRAMEWORKS_EXECUTABLES)
foreach(FILE IN LISTS BUNDLE_FRAMEWORKS)
    get_filename_component(FRAMEWORK_NAME "${FILE}" NAME_WE)
    list(APPEND BUNDLE_FRAMEWORKS_EXECUTABLES "${FILE}/Versions/Current/${FRAMEWORK_NAME}")
endforeach()
# Filter out symlinks
set(BUNDLE_PLUGINS_NOSYMLINKS)
foreach(FILE IN LISTS BUNDLE_PLUGINS)
    if(NOT IS_SYMLINK "${FILE}")
        list(APPEND BUNDLE_PLUGINS_NOSYMLINKS "${FILE}")
    endif()
endforeach()
# Join the lists
set(BUNDLE_LIBRARIES_FRAMEWORKS)
list(APPEND BUNDLE_LIBRARIES_FRAMEWORKS ${BUNDLE_LIBRARIES_NOSYMLINKS})
list(APPEND BUNDLE_LIBRARIES_FRAMEWORKS ${BUNDLE_FRAMEWORKS_EXECUTABLES})
list(APPEND BUNDLE_LIBRARIES_FRAMEWORKS ${BUNDLE_PLUGINS_NOSYMLINKS})

# BROKEN! Does not include all required dependencies!
#file(GET_RUNTIME_DEPENDENCIES
#        RESOLVED_DEPENDENCIES_VAR DEPS_RESOLVED
#        UNRESOLVED_DEPENDENCIES_VAR DEPS_UNRESOLVED
#        CONFLICTING_DEPENDENCIES_PREFIX DEPS_CONFLICTING
#        EXECUTABLES ${BUNDLE_BINARIES_NOSYMLINKS}
#        LIBRARIES ${BUNDLE_LIBRARIES_FRAMEWORKS}
#        DIRECTORIES "${CMAKE_SOURCE_DIR}/External/Products/lib"
#)

# Extract rpaths for all libraries
set(GLOBAL_RPATHS)
foreach(FILE IN LISTS BUNDLE_LIBRARIES_FRAMEWORKS BUNDLE_BINARIES_NOSYMLINKS)
    # Extract rpaths
    execute_process(
            COMMAND otool -l "${FILE}"
            COMMAND grep -A2 LC_RPATH
            COMMAND grep path
            OUTPUT_VARIABLE RPATHS_RAW
            OUTPUT_STRIP_TRAILING_WHITESPACE
    )

    # Split into list
    string(REPLACE "\n" ";" RPATHS_LIST "${RPATHS_RAW}")

    # Parse and clean up rpath lines
    set(EXTRACTED_RPATH_PATHS)
    foreach(RPATH_LINE IN LISTS RPATHS_LIST)
        # Match only valid path lines and extract the path
        string(REGEX MATCH "path ([^ ]+) \\(offset [0-9]+\\)" _match "${RPATH_LINE}")
        if(_match)
            string(REGEX REPLACE "path ([^ ]+) \\(offset [0-9]+\\)" "\\1" CLEANED_RPATH "${_match}")
            list(APPEND EXTRACTED_RPATH_PATHS "${CLEANED_RPATH}")
        endif()
    endforeach()

    list(APPEND GLOBAL_RPATHS "${EXTRACTED_RPATH_PATHS}")
endforeach()
# Remove duplicates
list(REMOVE_DUPLICATES GLOBAL_RPATHS)

# Remove rpaths which do not exist
set(VALID_GLOBAL_RPATHS)
foreach(MAYBE_RPATH IN LISTS GLOBAL_RPATHS)
    if(EXISTS "${MAYBE_RPATH}")
        list(APPEND VALID_GLOBAL_RPATHS "${MAYBE_RPATH}")
    endif()
endforeach()

# Now extract each required library for each binary/framework
set(GLOBAL_REQUIRED_LIBS)
foreach(FILE IN LISTS BUNDLE_LIBRARIES_FRAMEWORKS BUNDLE_BINARIES_NOSYMLINKS)

    execute_process(
            COMMAND /bin/sh -c "otool -L \"${FILE}\" | tail -n +3"
            OUTPUT_VARIABLE REQUIERED_LIBS_UNFILTERED
            OUTPUT_STRIP_TRAILING_WHITESPACE
    )

    # Split into a list by newline
    string(REPLACE "\n" ";" LIB_LINES "${REQUIERED_LIBS_UNFILTERED}")

    set(LIBS_CLEAN)
    foreach(line IN LISTS LIB_LINES)
        # Strip trailing version info: remove everything starting from " (" to the end
        string(REGEX REPLACE " *\\(compatibility version [^)]*\\)" "" clean_line "${line}")
        string(STRIP "${clean_line}" clean_line)  # remove leading/trailing whitespace
        if(NOT clean_line STREQUAL "")
            list(APPEND LIBS_CLEAN "${clean_line}")
        endif()
    endforeach()
    list(APPEND GLOBAL_REQUIRED_LIBS "${LIBS_CLEAN}")

endforeach()
# Remove duplicates
list(REMOVE_DUPLICATES GLOBAL_REQUIRED_LIBS)

# Filter System Libraries
set(GLOBAL_REQUIRED_LIBS_NO_SYS_LIBS)
foreach(LIB IN LISTS GLOBAL_REQUIRED_LIBS)
    if("${LIB}" MATCHES "/System" OR "${LIB}" MATCHES "/usr/lib" )
        continue()
    endif()
    list(APPEND GLOBAL_REQUIRED_LIBS_NO_SYS_LIBS "${LIB}")
endforeach()

# Iterate through each required dependency and check if it can be found using one of the rpaths
set(DEPS_RESOLVED)
set(DEPS_UNRESOLVED)
set(DEPS_CONFLICTING)
foreach(LIB IN LISTS GLOBAL_REQUIRED_LIBS_NO_SYS_LIBS)
    # Strip @rpath from LIB
    string(REGEX REPLACE "@rpath/" "" PLAIN_LIB_NAME "${LIB}")
    set(FOUND_PATHS)
    foreach(RPATH IN LISTS VALID_GLOBAL_RPATHS)
        if(EXISTS "${RPATH}/${PLAIN_LIB_NAME}")
            list(APPEND FOUND_PATHS "${RPATH}/${PLAIN_LIB_NAME}")
        endif()
    endforeach()
    # message("FOUND: ${FOUND_PATHS}")
    # Get length of list
    list(LENGTH FOUND_PATHS FOUND_PATHS_COUNT)
    # If FOUND_PATHS is empty, set as DEPS_UNRESOLVED
    if(FOUND_PATHS_COUNT EQUAL 0)
        list(APPEND DEPS_UNRESOLVED "${LIB}")
    elseif(FOUND_PATHS_COUNT GREATER 1)
        list(APPEND DEPS_CONFLICTING "${FOUND_PATHS}")
    else()
        list(APPEND DEPS_RESOLVED "${FOUND_PATHS}")
    endif()
endforeach()

if(DEPS_UNRESOLVED)
    set(UNRESOLVED_NON_ABSOLUTE)
    # Try to resolve them by using an absolute path!
    # If this exists, then we can append it to the resolved path list
    foreach(UNRESOLVED IN LISTS DEPS_UNRESOLVED)
        if(EXISTS "${UNRESOLVED}")
            list(APPEND DEPS_RESOLVED "${UNRESOLVED}")
        else()
            list(APPEND UNRESOLVED_NON_ABSOLUTE "${UNRESOLVED}")
        endif()
    endforeach()
    if(UNRESOLVED_NON_ABSOLUTE)
        message(WARNING "[AliceVision] There are unresolved dependencies! The bundle will not be created. The following dependencies were not resolved:")
        foreach(UNRESOLVED IN LISTS UNRESOLVED_NON_ABSOLUTE)
            message("\tUnresolved: ${UNRESOLVED}")
        endforeach()
        return()
    endif()
endif()

if(DEPS_CONFLICTING)
    message(WARNING "[AliceVision] There are conflicting dependencies. One will be included in the final bundle, which one, is undefined. The following dependencies conflicted:")
    foreach(CONFLICTING IN LISTS DEPS_CONFLICTING)
        message("\tConflicting: ${CONFLICTING}")
    endforeach()
endif()

# All dependencies resolved, we will copy them
# Generate normalized paths for every dependency found
set(REAL_DEPENDENCY_PATHS)
foreach(PATH IN LISTS DEPS_RESOLVED DEPS_CONFLICTING)
    # Dirty hack to restore the symlinks after REAL_PATH
    # Save the file name, and later re-append it
    get_filename_component(TEMP_FILENAME "${PATH}" NAME)

    # Get the real path
    file(REAL_PATH "${PATH}" NORMALIZED_PATH EXPAND_TILDE)

    # Get the directory of this real path
    get_filename_component(TEMP_FILEDIR "${NORMALIZED_PATH}" DIRECTORY)

    # Now stitch together
    set(CANONICAL_PATH "${TEMP_FILEDIR}/${TEMP_FILENAME}")

    # Append new path
    list(APPEND REAL_DEPENDENCY_PATHS "${CANONICAL_PATH}")
endforeach()
# Filter out any doubles
list(REMOVE_DUPLICATES REAL_DEPENDENCY_PATHS)

# We will filter out any AV* frameworks and other libraries which might already be available in Frameworks
# We reuse BUNDLE_LIBRARIES_FRAMEWORKS
# Get the filename of all Framework dependencies
set(BUNDLE_FRAMEWORKS_LIBRARYS_NAMES)
foreach(FRLI IN LISTS BUNDLE_LIBRARIES_FRAMEWORKS)
    get_filename_component(FRLI_NAME "${FRLI}" NAME)
    list(APPEND BUNDLE_FRAMEWORKS_LIBRARYS_NAMES "${FRLI_NAME}")
endforeach()

set(DEPENDENCY_PATHS_FILTERED)
foreach(PATH IN LISTS REAL_DEPENDENCY_PATHS)
    # Get the file name
    get_filename_component(DEP_NAME "${PATH}" NAME)
    list(FIND BUNDLE_FRAMEWORKS_LIBRARYS_NAMES "${DEP_NAME}" IDX)
    if(IDX EQUAL -1)
        list(APPEND DEPENDENCY_PATHS_FILTERED "${PATH}")
    endif()
endforeach()

# Copy all the dependencies into the bundle
foreach(DEPENDENCY IN LISTS DEPENDENCY_PATHS_FILTERED)

    # Detect if the dependency is inside a .framework
    string(FIND "${DEPENDENCY}" ".framework/" FRAMEWORK_INDEX)

    if(NOT FRAMEWORK_INDEX EQUAL -1)
        # It's a framework — extract root path (e.g., /path/to/Foo.framework)
        string(REGEX MATCH ".+\\.framework" FRAMEWORK_ROOT "${DEPENDENCY}")

        # Copy entire framework directory
        execute_process(COMMAND rsync
            -a
            "${FRAMEWORK_ROOT}"
            "${BUNDLE}/Contents/Libraries"
        )
    else()
        # It's a regular shared library (.dylib) — copy as-is
        file(COPY "${DEPENDENCY}" DESTINATION "${BUNDLE}/Contents/Libraries" FOLLOW_SYMLINK_CHAIN)
    endif()

endforeach()

# Start with the initially copied dependencies
set(ALL_COPIED_DEPENDENCIES ${DEPENDENCY_PATHS_FILTERED})

# We'll recursively find dependencies of copied dependencies
set(NEW_DEPENDENCIES_FOUND TRUE)

while(NEW_DEPENDENCIES_FOUND)
    set(NEW_DEPENDENCIES_FOUND FALSE)
    set(NEW_DEPENDENCIES)

    foreach(DEP IN LISTS ALL_COPIED_DEPENDENCIES)
        execute_process(
                COMMAND /bin/sh -c "otool -L \"${DEP}\" | tail -n +2"
                OUTPUT_VARIABLE DEP_LIBS_RAW
                OUTPUT_STRIP_TRAILING_WHITESPACE
        )

        string(REPLACE "\n" ";" DEP_LIBS_LIST "${DEP_LIBS_RAW}")

        foreach(LIB_LINE IN LISTS DEP_LIBS_LIST)
            # Strip trailing version info and whitespace
            string(REGEX REPLACE " *\\(compatibility version [^)]*\\)" "" CLEAN_LIB_LINE "${LIB_LINE}")
            string(STRIP "${CLEAN_LIB_LINE}" CLEAN_LIB_LINE)

            # Skip empty lines and system libs
            if(CLEAN_LIB_LINE STREQUAL "")
                continue()
            endif()
            if(CLEAN_LIB_LINE MATCHES "^/System" OR CLEAN_LIB_LINE MATCHES "^/usr/lib")
                continue()
            endif()

            # Resolve @rpath prefix if present
            string(REGEX REPLACE "^@rpath/" "" LIB_BASENAME "${CLEAN_LIB_LINE}")

            # Try to find full path in VALID_GLOBAL_RPATHS (reuse from earlier)
            set(FOUND_FULL_PATH)
            foreach(RPATH IN LISTS VALID_GLOBAL_RPATHS)
                set(CANDIDATE_PATH "${RPATH}/${LIB_BASENAME}")
                if(EXISTS "${CANDIDATE_PATH}")
                    set(FOUND_FULL_PATH "${CANDIDATE_PATH}")
                    break()
                endif()
            endforeach()

            # If we couldn't find full path, skip (might be unresolved or system lib)
            if(NOT FOUND_FULL_PATH)
                continue()
            endif()

            # If not already copied, mark for copying
            list(FIND ALL_COPIED_DEPENDENCIES "${FOUND_FULL_PATH}" FOUND_INDEX)
            if(FOUND_INDEX EQUAL -1)
                list(FIND NEW_DEPENDENCIES "${FOUND_FULL_PATH}" NEW_INDEX)
                if(NEW_INDEX EQUAL -1)
                    list(APPEND NEW_DEPENDENCIES "${FOUND_FULL_PATH}")
                    set(NEW_DEPENDENCIES_FOUND TRUE)
                endif()
            endif()
        endforeach()
    endforeach()

    # Copy new dependencies found in this iteration
    foreach(NEW_DEP IN LISTS NEW_DEPENDENCIES)
        # Detect if the dependency is inside a .framework
        string(FIND "${NEW_DEP}" ".framework/" FRAMEWORK_INDEX)

        if(NOT FRAMEWORK_INDEX EQUAL -1)
            # It's a framework — extract root path (e.g., /path/to/Foo.framework)
            string(REGEX MATCH ".+\\.framework" FRAMEWORK_ROOT "${NEW_DEP}")

            # Copy entire framework directory
            execute_process(COMMAND rsync
                -a --delete --copy-unsafe-links
                "${FRAMEWORK_ROOT}"
                "${BUNDLE}/Contents/Libraries"
            )
        else()
            # It's a regular shared library (.dylib) — copy as-is
            file(COPY "${NEW_DEP}" DESTINATION "${BUNDLE}/Contents/Libraries" FOLLOW_SYMLINK_CHAIN)
        endif()
        # Add to ALL_COPIED_DEPENDENCIES to check their dependencies next iteration
        list(APPEND ALL_COPIED_DEPENDENCIES "${NEW_DEP}")
    endforeach()
endwhile()

# Now remove all rpaths of the Frameworks and Binaries and replace them with the simples approach so it only looks within the bundle
foreach(BINARY IN LISTS BUNDLE_BINARIES_NOSYMLINKS)
    # Extract rpaths
    execute_process(
            COMMAND otool -l "${BINARY}"
            COMMAND grep -A2 LC_RPATH
            COMMAND grep path
            OUTPUT_VARIABLE RPATHS_RAW
            OUTPUT_STRIP_TRAILING_WHITESPACE
    )

    # Split into list
    string(REPLACE "\n" ";" RPATHS_LIST "${RPATHS_RAW}")

    # Parse and clean up rpath lines
    set(EXTRACTED_RPATH_PATHS)
    foreach(RPATH_LINE IN LISTS RPATHS_LIST)
        # Match only valid path lines and extract the path
        string(REGEX MATCH "path ([^ ]+) \\(offset [0-9]+\\)" _match "${RPATH_LINE}")
        if(_match)
            string(REGEX REPLACE "path ([^ ]+) \\(offset [0-9]+\\)" "\\1" CLEANED_RPATH "${_match}")
            list(APPEND EXTRACTED_RPATH_PATHS "${CLEANED_RPATH}")
        endif()
    endforeach()

    # Generate argument list
    set(REMOVE_RPATHS_CLI)
    foreach(RPATH IN LISTS EXTRACTED_RPATH_PATHS)
        list(APPEND REMOVE_RPATHS_CLI -delete_rpath "${RPATH}")
    endforeach()

    # Run install_name_tool
    if(REMOVE_RPATHS_CLI)
        execute_process(
                COMMAND install_name_tool ${REMOVE_RPATHS_CLI} "${BINARY}"
        )
    endif()

    # Now append the new rpaths
    execute_process(
            COMMAND install_name_tool
                -add_rpath "@executable_path/../Frameworks"
                -add_rpath "@executable_path/../Libraries"
                "${BINARY}"
    )
endforeach()

# Now remove all rpaths of the Frameworks and Binaries and replace them with the simples approach so it only looks within the bundle
foreach(LIBRARY IN LISTS BUNDLE_LIBRARIES_FRAMEWORKS)
    # Extract rpaths
    execute_process(
            COMMAND otool -l "${LIBRARY}"
            COMMAND grep -A2 LC_RPATH
            COMMAND grep path
            OUTPUT_VARIABLE RPATHS_RAW
            OUTPUT_STRIP_TRAILING_WHITESPACE
    )

    # Split into list
    string(REPLACE "\n" ";" RPATHS_LIST "${RPATHS_RAW}")

    # Parse and clean up rpath lines
    set(EXTRACTED_RPATH_PATHS)
    foreach(RPATH_LINE IN LISTS RPATHS_LIST)
        # Match only valid path lines and extract the path
        string(REGEX MATCH "path ([^ ]+) \\(offset [0-9]+\\)" _match "${RPATH_LINE}")
        if(_match)
            string(REGEX REPLACE "path ([^ ]+) \\(offset [0-9]+\\)" "\\1" CLEANED_RPATH "${_match}")
            list(APPEND EXTRACTED_RPATH_PATHS "${CLEANED_RPATH}")
        endif()
    endforeach()

    # Generate argument list
    set(REMOVE_RPATHS_CLI)
    foreach(RPATH IN LISTS EXTRACTED_RPATH_PATHS)
        list(APPEND REMOVE_RPATHS_CLI -delete_rpath "${RPATH}")
    endforeach()

    # Run install_name_tool
    if(REMOVE_RPATHS_CLI)
        execute_process(
                COMMAND install_name_tool ${REMOVE_RPATHS_CLI} "${LIBRARY}"
        )
    endif()

    # Now append the new rpaths
    execute_process(
            COMMAND install_name_tool
            -add_rpath "@executable_path/../Frameworks"
            -add_rpath "@executable_path/../Libraries"
            -add_rpath "@loader_path/../../Frameworks" # For PlugIns
            -add_rpath "@loader_path/../../Libraries"  # For PlugIns
            "${LIBRARY}"
    )
endforeach()

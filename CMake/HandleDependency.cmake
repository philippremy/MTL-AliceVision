###===================================================
###
### A utility function for acquiring dependencies
###
###===================================================

# Abstract macro for switching the different dependency names
include(DependencyBuilders/BuildDependency)

#
# The call signature is:
#
# alicevision_handle_dependency(
#   <NAME>
#   VERSION <VERSION>
#   REQUIRED
#   COMPONENTS <COMPONENTS>
#   BUILD_OPTIONS <BUILD_OPTIONS>
# )
#
function(alicevision_handle_dependency)
    set(MVA COMPONENTS BUILD_OPTIONS)
    set(O REQUIRED FORCE_BUILD CONFIG)
    set(OVA VERSION)
    cmake_parse_arguments(PARSE_ARGV 1 DEP "${O}" "${OVA}" "${MVA}")
    set(NAME "${ARGV0}")

    set(FOUND OFF)
    set(COMPONENTS "")
    set(VERSION)
    set(CONFIG)
    if(NOT NAME STREQUAL "")
         if(DEP_COMPONENTS)
             set(COMPONENTS COMPONENTS ${DEP_COMPONENTS})
         endif()
         if(DEP_VERSION)
             set(VERSION "${DEP_VERSION}")
         endif()
         if(DEP_CONFIG)
            set(CONFIG CONFIG)
         endif()
         if(NOT DEP_FORCE_BUILD)
            find_package(${NAME} ${DEP_VERSION} ${CONFIG} QUIET ${COMPONENTS})
             if(NOT ${NAME}_FOUND)
                if(DEP_REQUIRED)
                    message(STATUS "[AliceVision] Did NOT find required dependency ${NAME}. Will attempt to build from source.")
                    build_dependency(${NAME})
                    find_package(${NAME} ${DEP_VERSION} QUIET ${CONFIG} ${COMPONENTS})
                    if(NOT ${NAME}_FOUND)
                        message(FATAL_ERROR "[AliceVision] Dependency ${NAME} was built, but still cannot be found. Please submit a bug report.")
                    else()
                        if(${NAME}_VERSION)
                            message(STATUS "[AliceVision] Found dependency ${NAME} (${${NAME}_VERSION}).")
                        else()
                            message(STATUS "[AliceVision] Found dependency ${NAME}.")
                        endif()
                    endif()
                else()
                    message(STATUS "[AliceVision] Did NOT find optional dependency ${NAME}. It will NOT be build from source.")
                endif()
             else()
                if(${NAME}_VERSION)
                    message(STATUS "[AliceVision] Found dependency ${NAME} (${${NAME}_VERSION}).")
                else()
                    message(STATUS "[AliceVision] Found dependency ${NAME}.")
                endif()
             endif()
         else()
             message(STATUS "[AliceVision] Requested dependency ${NAME} to be force build.")
             build_dependency(${NAME})
             find_package(${NAME} ${DEP_VERSION} ${CONFIG} QUIET ${COMPONENTS})
             if(NOT ${NAME}_FOUND)
                 message(FATAL_ERROR "[AliceVision] Dependency ${NAME} was built, but still cannot be found. Please submit a bug report.")
             else()
                 if(${NAME}_VERSION)
                     message(STATUS "[AliceVision] Found dependency ${NAME} (${${NAME}_VERSION}).")
                 else()
                     message(STATUS "[AliceVision] Found dependency ${NAME}.")
                 endif()
             endif()
         endif()
    else()
         message(WARNING "[AliceVision] No dependency name was provided! Cannot resolve dependency.")
    endif()

endfunction()
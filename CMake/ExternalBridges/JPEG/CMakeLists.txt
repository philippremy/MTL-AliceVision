cmake_minimum_required(VERSION 3.28)

project(libjpeg LANGUAGES C)

option(BUILD_SHARED_LIBS "Whether to build a shared library instead of a static one")

set(LIBRARY_SOURCES
        jmemnobs.c
        jaricom.c
        jcomapi.c
        jutils.c
        jerror.c
        jmemmgr.c
        jcapimin.c
        jcapistd.c
        jcarith.c
        jctrans.c
        jcparam.c
        jdatadst.c
        jcinit.c
        jcmaster.c
        jcmarker.c
        jcmainct.c
        jcprepct.c
        jccoefct.c
        jccolor.c
        jcsample.c
        jchuff.c
        jcdctmgr.c
        jfdctfst.c
        jfdctflt.c
        jfdctint.c
        jdapimin.c
        jdapistd.c
        jdarith.c
        jdtrans.c
        jdatasrc.c
        jdmaster.c
        jdinput.c
        jdmarker.c
        jdhuff.c
        jdmainct.c
        jdcoefct.c
        jdpostct.c
        jddctmgr.c
        jidctfst.c
        jidctflt.c
        jidctint.c
        jdsample.c
        jdcolor.c
        jquant1.c
        jquant2.c
        jdmerge.c
)

if(BUILD_SHARED_LIBS)
    add_library(jpeg SHARED ${LIBRARY_SOURCES})
else()
    add_library(jpeg STATIC ${LIBRARY_SOURCES})
endif()

# Include GNU Install Directories
include(GNUInstallDirs)

# Install library
install(TARGETS jpeg DESTINATION "${CMAKE_INSTALL_LIBDIR}")

# Install headers (directly goes into the include folder)
install(FILES
        jpeglib.h
        jerror.h
        jconfig.h
        jmorecfg.h
        DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}"
)
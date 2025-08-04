set(FEATS
    covdet.c
    generic.c
    host.c
    imopv.c
    imopv_sse2.c
    mathop.c
    mathop_avx.c
    mathop_sse2.c
    mser.c
    random.c
    scalespace.c
    sift.c
    stringop.c
)

set(FEATS_H
    covdet.h
    float.th
    generic.h
    host.h
    imopv.h
    imopv_sse2.h
    mathop.h
    mathop_avx.h
    mathop_sse2.h
    mser.h
    random.h
    scalespace.h
    sift.h
    stringop.h
)

set_source_files_properties(${FEATS} ${FEATS_H} PROPERTIES LANGUAGE C)
set_source_files_properties(${FEATS_H} PROPERTIES HEADER_FILE_ONLY TRUE)

if(ALICEVISION_HAVE_OPENMP)
    set(VLFEAT_OPENMP_TARGETS OpenMP::OpenMP_C)
endif()

alicevision_add_library(VLFeat
    SOURCES ${FEATS}
    HEADERS ${FEATS_H}
    PRIVATE_INCLUDE_DIRS
        # VLFeat uses "#include <VLFeat/A_SOURCE_FILE.c>": WTF?
        "${CMAKE_SOURCE_DIR}/Sources/Libraries"
    PUBLIC_LINKS
        ${VLFEAT_OPENMP_TARGETS}
    PRIVATE_DEFINITIONS
        "-DVL_BUILD_DLL"
)

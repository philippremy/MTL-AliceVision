# Headers
set(linearProgramming_files_headers
    bisectionLP.hpp
    linearProgramming.hpp
    ISolver.hpp
    OSIXSolver.hpp
)

# Dependencies
set(AVLinearProgramming_DEPS
    AVNumeric
    AVSystem
    Coin::Clp        # clp + solver wrapper
    Coin::CoinUtils  # container tools
    Coin::Osi        # generic LP
)

if(NOT ALICEVISION_HAVE_MOSEK)
    alicevision_add_interface(AVLinearProgramming
        HEADERS ${linearProgramming_files_headers}
        LINKS ${AVLinearProgramming_DEPS}
    )
else()
    # Add MOSEK related sources
    set(linearProgramming_files_sources MOSEKSolver.cpp)
    list(APPEND linearProgramming_files_headers MOSEKSolver.hpp)
    alicevision_add_library(AVLinearProgramming
        SOURCES ${linearProgramming_files_sources}
        HEADERS ${linearProgramming_files_headers}
        PUBLIC_LINKS
            ${AVLinearProgramming_DEPS}
            Coin::OsiMsk  # OSI solver wrapper for the Mosek backend
            ${MOSEK_LIB}  # Mosek lib
    )
endif()

# Unit tests
alicevision_add_test(Tests/LinearProgramming_test.cpp NAME "LinearProgramming_LinearProgramming" LINKS AVLinearProgramming)

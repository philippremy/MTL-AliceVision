set(system_files_headers
        cpu.hpp
        main.hpp
        MemoryInfo.hpp
        system.hpp
        Timer.hpp
        Logger.hpp
        ProgressDisplay.hpp
        nvtx.hpp
        hardwareContext.hpp
)

# Sources
set(system_files_sources
        cpu.cpp
        MemoryInfo.cpp
        Timer.cpp
        Logger.cpp
        ProgressDisplay.cpp
        nvtx.cpp
        hardwareContext.cpp
)

alicevision_add_library(AVSystem
        SOURCES ${system_files_sources}
        HEADERS ${system_files_headers}
        PUBLIC_LINKS
        Boost::log
        Boost::log_setup
        OpenMP::OpenMP_CXX
        PRIVATE_LINKS
        Boost::boost
        Boost::date_time
        Boost::thread
        Boost::system
        ${ALICEVISION_NVTX_LIBRARY}
)

alicevision_add_test(Tests/Logger_test.cpp NAME "System_Logger" LINKS AVSystem)
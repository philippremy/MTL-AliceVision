file(WRITE "${OUT}" "#pragma once\n\n")
file(APPEND "${OUT}" "#include <cstdint>\n#include <cstddef>\n\n")

foreach(header IN LISTS HEADERS)
    file(APPEND "${OUT}" "#include <${header}>\n")
endforeach()

file(APPEND "${OUT}" "\n#define ALICEVISION_LOOKUP_SPIRV_MODULE(NAME) std::make_pair(NAME, NAME##_SIZE)\n\n")

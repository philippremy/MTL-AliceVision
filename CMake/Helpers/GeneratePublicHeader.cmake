file(WRITE "${OUT}" "#pragma once\n\n")
file(APPEND "${OUT}" "#include <cstdint>\n#include <cstddef>\n\n")

foreach(module IN LISTS MODULES)
    file(APPEND "${OUT}" "extern const uint32_t ${module}[];\nextern const size_t ${module}_SIZE;\n\n")
endforeach()

file(APPEND "${OUT}" "\n#define ALICEVISION_LOOKUP_SPIRV_MODULE(NAME) std::make_pair(NAME, NAME##_SIZE)\n\n")

# Headers
set(gpu_files_headers
    gpu.hpp
)

# Sources
set(gpu_files_sources
    gpu.cpp
)

set(GPU_PRIVATE_LINKS AVSystem)
set(GPU_PRIVATE_INCLUDE_DIRS "")

if (ALICEVISION_HAVE_CUDA)
    set(GPU_PRIVATE_LINKS ${GPU_PRIVATE_LINKS} ${CUDA_LIBRARIES})
    set(GPU_PRIVATE_INCLUDE_DIRS ${GPU_PRIVATE_INCLUDE_DIRS} ${CUDA_INCLUDE_DIRS})
endif()

if(ALICEVISION_HAVE_VULKAN)
    list(APPEND gpu_files_sources
        Vulkan/device.cpp
        Vulkan/init.cpp
        Vulkan/vulkan.cpp
    )
    list(APPEND gpu_files_headers
        Vulkan/debug.hpp
        Vulkan/device.hpp
        Vulkan/exception.hpp
        Vulkan/memory.hpp
        Vulkan/utils.hpp
        Vulkan/impl/commandmanager.inl
        Vulkan/impl/memory.inl
        Vulkan/impl/pushconstant.inl
    )
    set(GPU_PRIVATE_LINKS ${GPU_PRIVATE_LINKS} Vulkan::Vulkan GPUOpen::VulkanMemoryAllocator)
    set(GPU_PUBLIC_LINKS ${GPU_PUBLIC_LINKS} Vulkan::Headers)
    if(APPLE)
        set(GPU_PRIVATE_LINKS ${GPU_PRIVATE_LINKS} "-framework CoreFoundation")
    endif()
    set(MOLTENVK_RESOURCES "./Resources/Vulkan")
    set(MOLTENVK_FRAMEWORK "$ENV{VULKAN_SDK}/lib/libMoltenVK.dylib")
endif()

if(ALICEVISION_HAVE_METAL)
    list(APPEND gpu_files_sources
            Metal/Metal.cpp
            Metal/device.cpp
            Metal/command.cpp
    )
    list(APPEND gpu_files_headers
            Metal/command.hpp
            Metal/device.hpp
            Metal/Impl/device.inl
    )
    set(GPU_PUBLIC_LINKS ${GPU_PRIVATE_LINKS} "-framework Metal" "-framework Foundation" "-framework CoreFoundation")
endif()

alicevision_add_library(AVGPU
    SOURCES ${gpu_files_sources}
    HEADERS ${gpu_files_headers}
    PRIVATE_LINKS
        ${GPU_PRIVATE_LINKS}
    PUBLIC_LINKS
        ${GPU_PUBLIC_LINKS}
    PRIVATE_INCLUDE_DIRS
        ${GPU_PRIVATE_INCLUDE_DIRS}
    RESOURCES
        ${MOLTENVK_RESOURCES}
    FRAMEWORKS
        ${MOLTENVK_FRAMEWORK}
)

if(ALICEVISION_HAVE_VULKAN)
    target_compile_definitions(AVGPU
        PUBLIC
            "VULKAN_HPP_DISPATCH_LOADER_DYNAMIC=1"
            "VULKAN_HPP_NO_EXCEPTIONS"
            "VULKAN_HPP_RAII_NO_EXCEPTIONS"
            "VULKAN_HPP_ASSERT_ON_RESULT=bool"
    )
endif()

if(ALICEVISION_HAVE_METAL)
    # See https://developer.apple.com/documentation/corefoundation/cfstr?language=objc
    target_compile_options(AVGPU PUBLIC "-fconstant-cfstrings")
endif()

# Unit tests
alicevision_add_test(Tests/VulkanCapableDevice_test.cpp NAME "GPU_VulkanCapableDevice" LINKS AVGPU)
alicevision_add_test(Tests/MetalCapableDevice_test.cpp NAME "GPU_MetalCapableDevice" LINKS AVGPU)

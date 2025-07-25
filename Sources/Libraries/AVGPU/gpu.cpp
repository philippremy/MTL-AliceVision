// This file is part of the AliceVision project.
// Copyright (c) 2025 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <AVGPU/gpu.hpp>

#include <AV/config.hpp>
#include <AVSystem/Logger.hpp>

#include <sstream>
#include <memory.h>

#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_CUDA)
    #include <cuda_runtime.h>
#elif ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_VULKAN)
    #include <vulkan/vulkan.hpp>
    #include <vulkan/vulkan_shared.hpp>
    #include <AVGPU/Vulkan/device.hpp>

    #include <ranges>
    #include <numeric>
#elif ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_METAL)
    #include <AVGPU/Metal/device.hpp>
    #include <sys/sysctl.h>
    #include <ranges>
#endif

namespace aliceVision {
namespace gpu {

#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_CUDA)

bool gpuSupportCUDA(int minComputeCapabilityMajor, int minComputeCapabilityMinor, int minTotalDeviceMemory)
{
    int nbDevices = 0;
    cudaError_t success;
    success = cudaGetDeviceCount(&nbDevices);
    if (success != cudaSuccess)
    {
        ALICEVISION_LOG_ERROR("cudaGetDeviceCount failed: " << cudaGetErrorString(success));
        nbDevices = 0;
    }

    if (nbDevices > 0)
    {
        for (int i = 0; i < nbDevices; ++i)
        {
            cudaDeviceProp deviceProperties;

            if (cudaGetDeviceProperties(&deviceProperties, i) != cudaSuccess)
            {
                ALICEVISION_LOG_ERROR("Cannot get properties for CUDA gpu device " << i);
                continue;
            }

            if ((deviceProperties.major > minComputeCapabilityMajor ||
                 (deviceProperties.major == minComputeCapabilityMajor && deviceProperties.minor >= minComputeCapabilityMinor)) &&
                deviceProperties.totalGlobalMem >= (minTotalDeviceMemory * 1024 * 1024))
            {
                ALICEVISION_LOG_INFO("Supported CUDA-Enabled GPU detected.");
                return true;
            }
            else
            {
                ALICEVISION_LOG_ERROR("CUDA-Enabled GPU detected, but the compute capabilities is not enough.\n"
                                      << " - Device " << i << ": " << deviceProperties.major << "." << deviceProperties.minor
                                      << ", global memory: " << int(deviceProperties.totalGlobalMem / (1024 * 1024)) << "MB\n"
                                      << " - Requirements: " << minComputeCapabilityMajor << "." << minComputeCapabilityMinor
                                      << ", global memory: " << minTotalDeviceMemory << "MB\n");
            }
        }
        ALICEVISION_LOG_INFO("CUDA-Enabled GPU not supported.");
    }
    else
    {
        ALICEVISION_LOG_INFO("Can't find CUDA-Enabled GPU.");
    }
    return false;
}

std::string gpuInformationCUDA()
{
    std::string information;
    int nbDevices = 0;
    if (cudaGetDeviceCount(&nbDevices) != cudaSuccess)
    {
        ALICEVISION_LOG_WARNING("Could not determine number of CUDA cards in this system");
        nbDevices = 0;
    }

    if (nbDevices > 0)
    {
        information = "CUDA-Enabled GPU.\n";
        for (int i = 0; i < nbDevices; ++i)
        {
            cudaDeviceProp deviceProperties;
            if (cudaGetDeviceProperties(&deviceProperties, i) != cudaSuccess)
            {
                ALICEVISION_LOG_ERROR("Cannot get properties for CUDA gpu device " << i);
                continue;
            }

            if (cudaSetDevice(i) != cudaSuccess)
            {
                ALICEVISION_LOG_WARNING("Device with number " << i << " does not exist");
                continue;
            }

            std::size_t avail;
            std::size_t total;
            cudaError_t memInfoErr = cudaMemGetInfo(&avail, &total);
            if (memInfoErr != cudaSuccess)
            {
                // if the card does not provide this information.
                avail = 0;
                total = 0;
                ALICEVISION_LOG_WARNING("Cannot get available memory information for CUDA gpu device " << i << ":" << std::endl
                                                                                                       << "\t (error code: " << memInfoErr << ") "
                                                                                                       << cudaGetErrorName(memInfoErr));

                cudaError_t err = cudaGetLastError();  // clear error
            }

            std::stringstream deviceSS;

            deviceSS << "Device information:" << std::endl
                     << "\t- id:                      " << i << std::endl
                     << "\t- name:                    " << deviceProperties.name << std::endl
                     << "\t- compute capability:      " << deviceProperties.major << "." << deviceProperties.minor << std::endl
                     << "\t- clock frequency (kHz):   " << deviceProperties.clockRate << std::endl
                     << "\t- total device memory:     " << deviceProperties.totalGlobalMem / (1024 * 1024) << " MB " << std::endl
                     << "\t- device memory available: " << avail / (1024 * 1024) << " MB " << std::endl
                     << "\t- per-block shared memory: " << deviceProperties.sharedMemPerBlock << std::endl
                     << "\t- warp size:               " << deviceProperties.warpSize << std::endl
                     << "\t- max threads per block:   " << deviceProperties.maxThreadsPerBlock << std::endl
                     << "\t- max threads per SM(X):   " << deviceProperties.maxThreadsPerMultiProcessor << std::endl
                     << "\t- max block sizes:         "
                     << "{" << deviceProperties.maxThreadsDim[0] << "," << deviceProperties.maxThreadsDim[1] << ","
                     << deviceProperties.maxThreadsDim[2] << "}" << std::endl
                     << "\t- max grid sizes:          "
                     << "{" << deviceProperties.maxGridSize[0] << "," << deviceProperties.maxGridSize[1] << "," << deviceProperties.maxGridSize[2]
                     << "}" << std::endl
                     << "\t- max 2D array texture:    "
                     << "{" << deviceProperties.maxTexture2D[0] << "," << deviceProperties.maxTexture2D[1] << "}" << std::endl
                     << "\t- max 3D array texture:    "
                     << "{" << deviceProperties.maxTexture3D[0] << "," << deviceProperties.maxTexture3D[1] << "," << deviceProperties.maxTexture3D[2]
                     << "}" << std::endl
                     << "\t- max 2D linear texture:   "
                     << "{" << deviceProperties.maxTexture2DLinear[0] << "," << deviceProperties.maxTexture2DLinear[1] << ","
                     << deviceProperties.maxTexture2DLinear[2] << "}" << std::endl
                     << "\t- max 2D layered texture:  "
                     << "{" << deviceProperties.maxTexture2DLayered[0] << "," << deviceProperties.maxTexture2DLayered[1] << ","
                     << deviceProperties.maxTexture2DLayered[2] << "}" << std::endl
                     << "\t- number of SM(x)s:        " << deviceProperties.multiProcessorCount << std::endl
                     << "\t- registers per SM(x):     " << deviceProperties.regsPerMultiprocessor << std::endl
                     << "\t- registers per block:     " << deviceProperties.regsPerBlock << std::endl
                     << "\t- concurrent kernels:      " << (deviceProperties.concurrentKernels ? "yes" : "no") << std::endl
                     << "\t- mapping host memory:     " << (deviceProperties.canMapHostMemory ? "yes" : "no") << std::endl
                     << "\t- unified addressing:      " << (deviceProperties.unifiedAddressing ? "yes" : "no") << std::endl
                     << "\t- texture alignment:       " << deviceProperties.textureAlignment << " byte" << std::endl
                     << "\t- pitch alignment:         " << deviceProperties.texturePitchAlignment << " byte" << std::endl;

            information += deviceSS.str();
        }
    }
    else
    {
        information = "No CUDA-Enabled GPU.\n";
    }
    std::stringstream ss;
    ss << "CUDA build version: " << CUDART_VERSION / 1000 << "." << CUDART_VERSION / 10 % 100;
    information += ss.str();
    return information;
}

#elif ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_VULKAN)

bool gpuSupportVulkan(const int minTotalDeviceMemory)
{
    const auto vkManager = VulkanManager::getInstance();
    if(vkManager->m_devs.empty())
        return false;
    for(const auto& physDevice : std::views::values(vkManager->m_physDevs)) {
        auto const& memProps = physDevice->getMemoryProperties();
        vk::DeviceSize availableMem = 0;
        for(auto const& heaps : memProps.memoryHeaps) {
            availableMem += heaps.size;
        }
        if(availableMem > minTotalDeviceMemory * 1024 * 1024) {
            ALICEVISION_LOG_INFO("Supported Vulkan device detected.");
            return true;
        }
        auto const& devProps = physDevice->getProperties();
        ALICEVISION_LOG_ERROR("Vulkan-capable GPU detected, but the memory size is not big enough.\n"
                      << " - Device " << std::string(devProps.deviceName)
                      << " - API Version: " << VK_VERSION_MAJOR(devProps.apiVersion) << "." << VK_VERSION_MINOR(devProps.apiVersion) << "." << VK_VERSION_PATCH(devProps.apiVersion)
                      << " - Available Memory: " << static_cast<int>(availableMem / (1024 * 1024)) << "MB\n"
                      << " - Required Memory: " << minTotalDeviceMemory << "MB\n");
    }
    ALICEVISION_LOG_INFO("Can't find Vulkan-capable GPU.");
    return false;
}

std::string gpuInformationVulkan()
{
    std::string information;
    std::stringstream deviceSS;
    const auto vkManager = VulkanManager::getInstance();
    if(vkManager->m_physDevs.empty())
        return "No Vulkan-capable GPU.";
    for(const auto& physDevice : std::views::values(vkManager->m_physDevs)) {
        auto const& memProps = physDevice->getMemoryProperties();
        auto const& devProps = physDevice->getProperties();
        auto const& queueFamilies = physDevice->getQueueFamilyProperties();
        vk::DeviceSize availableMem = 0;
        for(auto const& heaps : memProps.memoryHeaps) {
            availableMem += heaps.size;
        }
        VmaTotalStatistics currentMemStatistics = {};
        vmaCalculateStatistics(vkManager->m_allocators.at(devProps.deviceID).get(), &currentMemStatistics);
        vk::StructureChain<
            vk::PhysicalDeviceProperties2,
            vk::PhysicalDeviceDriverProperties
        > props2Chain;
        physDevice->getProperties2(&props2Chain.get<vk::PhysicalDeviceProperties2>());
        const vk::PhysicalDeviceDriverProperties driverProps = props2Chain.get<vk::PhysicalDeviceDriverProperties>();
        deviceSS << "Device information:" << std::endl
             << "\t- id:                      " << devProps.deviceID << std::endl
             << "\t- name:                    " << std::string(devProps.deviceName) << std::endl
             << "\t- Vulkan Version:          " << VK_VERSION_MAJOR(devProps.apiVersion) << "." << VK_VERSION_MINOR(devProps.apiVersion) << "." << VK_VERSION_PATCH(devProps.apiVersion) << std::endl
             << "\t- Driver:                  " << "[ID: " << std::string(vk::to_string(driverProps.driverID)) << "] " << std::string(driverProps.driverName) << " " << std::string(driverProps.driverInfo) << " (Coformance: " << std::to_string(driverProps.conformanceVersion.major) << "." << std::to_string(driverProps.conformanceVersion.minor) << "." << std::to_string(driverProps.conformanceVersion.subminor) << "." << std::to_string(driverProps.conformanceVersion.patch) << ")" << std::endl
             << "\t- clock frequency (kHz):   " << "N/A" << std::endl
             << "\t- total device memory:     " << static_cast<int>(availableMem / (1024 * 1024)) << " MB " << std::endl
             << "\t- device memory available: " << static_cast<int>((availableMem - currentMemStatistics.total.statistics.allocationBytes) / (1024 * 1024)) << " MB " << std::endl
             << "\t- per-block shared memory: " << devProps.limits.maxComputeSharedMemorySize << " B" << std::endl
             << "\t- max threads per block:   " << devProps.limits.maxComputeWorkGroupInvocations << std::endl
             << "\t- max threads per SM(X):   " << "N/A" << std::endl
             << "\t- max block sizes:         " << "{" << devProps.limits.maxComputeWorkGroupSize[0] << "," << devProps.limits.maxComputeWorkGroupSize[1] << "," << devProps.limits.maxComputeWorkGroupSize[2] << "}" << std::endl
             << "\t- max grid sizes:          " << "{" << devProps.limits.maxComputeWorkGroupCount[0] << "," << devProps.limits.maxComputeWorkGroupCount[1] << "," << devProps.limits.maxComputeWorkGroupCount[2] << "}" << std::endl
             << "\t- max 2D array texture:    " << "{" << devProps.limits.maxImageDimension2D << "," << devProps.limits.maxImageDimension2D << "}" << std::endl
             << "\t- max 3D array texture:    " << "{" << devProps.limits.maxImageDimension3D << "," << devProps.limits.maxImageDimension3D << "," << devProps.limits.maxImageDimension3D << "}" << std::endl
             << "\t- max 2D linear texture:   " << "N/A" << std::endl
             << "\t- max 2D layered texture:  " << "N/A" << std::endl
             << "\t- number of SM(x)s:        " << "N/A" << std::endl
             << "\t- registers per SM(x):     " << "N/A" << std::endl
             << "\t- registers per block:     " << "N/A" << std::endl
             << "\t- concurrent kernels:      " << (std::count_if(queueFamilies.begin(), queueFamilies.end(), [](const vk::QueueFamilyProperties& queueProps){ return queueProps.queueFlags & vk::QueueFlags(vk::QueueFlagBits::eCompute); }) > 1 ? "yes" : "no") << std::endl
             << "\t- mapping host memory:     " << "N/A" << std::endl
             << "\t- unified addressing:      " << "N/A" << std::endl
             << "\t- texture alignment:       " << devProps.limits.minTexelBufferOffsetAlignment << " B" << std::endl
             << "\t- pitch alignment:         " << devProps.limits.optimalBufferCopyRowPitchAlignment << " B" << std::endl;
    }
    information += deviceSS.str();
    return information;
}

#elif ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_METAL)

bool gpuSupportMTL(const int minTotalDeviceMemory)
{
    const auto mtlDevManager = MTLDeviceManager::getInstance();
    if(mtlDevManager->getDevices().empty())
        return false;
    uint64_t host_memsize = 0;
    size_t size = sizeof(host_memsize);
    if (sysctlbyname("hw.memsize", &host_memsize, &size, nullptr, 0) != KERN_SUCCESS) {
        host_memsize = 0;
    }
    for(const auto& device : std::views::values(mtlDevManager->getDevices())) {
        const uint64_t deviceMem = device->hasUnifiedMemory() ? host_memsize : device->recommendedMaxWorkingSetSize() + host_memsize;
        if(deviceMem > minTotalDeviceMemory * 1024 * 1024) {
            ALICEVISION_LOG_INFO("Supported Metal device detected.");
            return true;
        }
        ALICEVISION_LOG_ERROR("Vulkan-capable GPU detected, but the memory size is not big enough.\n"
                      << " - Device " << std::string(device->name()->utf8String())
                      << " - Available Memory: " << deviceMem / (1024 * 1024) << "MB\n"
                      << " - Required Memory: " << minTotalDeviceMemory << "MB\n");
    }
    ALICEVISION_LOG_INFO("Can't find Vulkan-capable GPU.");
    return false;
}

std::string gpuInformationMTL()
{
    std::string information;
    std::stringstream deviceSS;
    const auto mtlManager = MTLDeviceManager::getInstance();
    if(mtlManager->getDevices().empty())
        return "No Metal-capable GPU.";
    uint64_t host_memsize = 0;
    size_t size = sizeof(host_memsize);
    if (sysctlbyname("hw.memsize", &host_memsize, &size, nullptr, 0) != KERN_SUCCESS) {
        host_memsize = 0;
    }
    for(const auto& device : std::views::values(mtlManager->getDevices())) {
        const uint64_t deviceMem = device->hasUnifiedMemory() ? host_memsize : device->recommendedMaxWorkingSetSize() + host_memsize;
        const NS::SharedPtr<NS::String> deviceName = NS::TransferPtr(device->name());
        const MTL::Size threadgroupSize = device->maxThreadsPerThreadgroup();
        deviceSS << "Device information:" << std::endl
             << "\t- id:                      " << device->registryID() << std::endl
             << "\t- name:                    " << std::string(deviceName->utf8String()) << std::endl
             << "\t- clock frequency (kHz):   " << "N/A" << std::endl
             << "\t- total device memory:     " << deviceMem / (1024 * 1024) << " MB " << std::endl
             << "\t- device memory available: " << (deviceMem - device->currentAllocatedSize()) / (1024 * 1024) << " MB " << std::endl
             << "\t- per-block shared memory: " << device->maxThreadgroupMemoryLength() << " B" << std::endl
             << "\t- max threads per block:   " << (threadgroupSize.width * threadgroupSize.height * threadgroupSize.depth) << std::endl
             << "\t- max threads per SM(X):   " << "N/A" << std::endl
             << "\t- max block sizes:         " << "{" << threadgroupSize.width << "," << threadgroupSize.height << "," << threadgroupSize.depth << "}" << std::endl
             << "\t- max grid sizes:          " << "N/A (assumingly 2^32–1)" << std::endl
             << "\t- max 2D array texture:    " << "N/A (usually {16384, 16384})" << std::endl
             << "\t- max 3D array texture:    " << "N/A (usually {16384, 16384, 16384})" << std::endl
             << "\t- max 2D linear texture:   " << "N/A" << std::endl
             << "\t- max 2D layered texture:  " << "N/A" << std::endl
             << "\t- number of SM(x)s:        " << "N/A" << std::endl
             << "\t- registers per SM(x):     " << "N/A" << std::endl
             << "\t- registers per block:     " << "N/A" << std::endl
             << "\t- concurrent kernels:      " << "Yes" << std::endl
             << "\t- mapping host memory:     " << "Yes" << std::endl
             << "\t- unified addressing:      " << (device->hasUnifiedMemory() ? "Yes" : "No") << std::endl
             << "\t- texture alignment:       " << device->minimumLinearTextureAlignmentForPixelFormat(MTL::PixelFormatRGBA32Float) << " B" << std::endl
             << "\t- pitch alignment:         " << device->minimumTextureBufferAlignmentForPixelFormat(MTL::PixelFormatRGBA32Float) << " B" << std::endl;
    }
    information += deviceSS.str();
    return information;
}

#endif

}  // namespace gpu
}  // namespace aliceVision

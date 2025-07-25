// This file is part of the AliceVision project.
// Copyright (c) 2025 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <vulkan/vulkan.hpp>
#include <vulkan/vulkan_shared.hpp>

#include <AVSystem/Logger.hpp>

#include <AV/config.hpp>
#include <AVGPU/Vulkan/device.hpp>
#include <AVGPU/Vulkan/exception.hpp>
#include <AVGPU/Vulkan/debug.hpp>
#include <AVGPU/Vulkan/memory.hpp>

#if VULKAN_HPP_DISPATCH_LOADER_DYNAMIC == 1
    #include <vulkan/vulkan_hpp_macros.hpp>
#endif

#include <vulkan/vulkan_core.h>

#include <vector>
#include <mutex>
#include <iostream>
#include <filesystem>
#include <string>
#include <fstream>
#include <ranges>

#if defined(__APPLE__) || defined(__MACH__)
    #include <MoltenVK/vk_mvk_moltenvk.h>
#endif

#ifdef _WIN32
    #include <windows.h>
    #include <shlobj.h>
#elif defined(__APPLE__)
    #include <pwd.h>
    #include <unistd.h>
#elif defined(__linux__)
    #include <pwd.h>
    #include <unistd.h>
#endif

VulkanManager::VulkanManager()
{
#if VULKAN_HPP_DISPATCH_LOADER_DYNAMIC == 1
    VULKAN_HPP_DEFAULT_DISPATCHER.init();
#endif
    // Create the Application Info
    vk::ApplicationInfo appInfo("AVGPU", VK_MAKE_VERSION(1, 0, 0), "AliceVision", VK_MAKE_VERSION(1, 0, 0), VK_API_VERSION_1_2);
    bool useValidationLayers = false;
    bool useDebugExtension = false;
#if ALICEVISION_IS_DEFINED(ALICEVISION_USE_VULKAN_VALIDATION)
    // Enumerate the available layers
    const std::vector<vk::LayerProperties>& availableLayers = ALICEVISION_THROW_ON_VULKAN_HPP_ERROR(vk::enumerateInstanceLayerProperties(), "Failed to enumerate available instance layers!");
    for(auto const& layer : availableLayers) {
        if(strcmp(layer.layerName, "VK_LAYER_KHRONOS_validation") == 0) {
            useValidationLayers = true;
            break;
        }
    }
    if(!useValidationLayers)
        throw VulkanException("VK_LAYER_KHRONOS_validation was requested, but is not available in this driver!", vk::Result::eErrorLayerNotPresent);
    // Enumerate the available extensions
    const std::vector<vk::ExtensionProperties>& availableExtensions = ALICEVISION_THROW_ON_VULKAN_HPP_ERROR(vk::enumerateInstanceExtensionProperties(), "Failed to enumerate available instance extensions!");
    for(auto const& extension : availableExtensions) {
        if(strcmp(extension.extensionName, VK_EXT_DEBUG_UTILS_EXTENSION_NAME) == 0) {
            useDebugExtension = true;
            break;
        }
    }
    if(!useDebugExtension)
        throw VulkanException("VK_EXT_DEBUG_UTILS_EXTENSION_NAME was requested, but is not available in this driver!", vk::Result::eErrorLayerNotPresent);
    const char* layerNames[1] = { "VK_LAYER_KHRONOS_validation" };
    const char* extensionNames[2] = { VK_EXT_DEBUG_UTILS_EXTENSION_NAME, VK_KHR_PORTABILITY_ENUMERATION_EXTENSION_NAME };
    // Configure the debug handler
    const vk::DebugUtilsMessengerCreateInfoEXT debugHandlerInfo({}, vk::DebugUtilsMessageSeverityFlagsEXT(vk::DebugUtilsMessageSeverityFlagBitsEXT::eVerbose | vk::DebugUtilsMessageSeverityFlagBitsEXT::eInfo | vk::DebugUtilsMessageSeverityFlagBitsEXT::eWarning | vk::DebugUtilsMessageSeverityFlagBitsEXT::eError), vk::DebugUtilsMessageTypeFlagsEXT(vk::DebugUtilsMessageTypeFlagBitsEXT::eGeneral | vk::DebugUtilsMessageTypeFlagBitsEXT::eValidation | vk::DebugUtilsMessageTypeFlagBitsEXT::ePerformance), reinterpret_cast<vk::PFN_DebugUtilsMessengerCallbackEXT>(validationMessageCallback), const_cast<void*>(static_cast<const void*>(USER_INFO)));
    const vk::InstanceCreateInfo instanceInfo(vk::InstanceCreateFlags(vk::InstanceCreateFlagBits::eEnumeratePortabilityKHR), &appInfo, layerNames, extensionNames, debugHandlerInfo);
#else
    const char* extensionNames[1] = { VK_KHR_PORTABILITY_ENUMERATION_EXTENSION_NAME };
    const vk::InstanceCreateInfo instanceInfo(vk::InstanceCreateFlags(vk::InstanceCreateFlagBits::eEnumeratePortabilityKHR), &appInfo, 0, nullptr, std::size(extensionNames), extensionNames);
#endif
    // Create the Vulkan Instance
    this->m_vkInstance = vk::SharedInstance(ALICEVISION_THROW_ON_VULKAN_HPP_ERROR(vk::createInstance(instanceInfo), "Unable to create the global vk::SharedInstance!"));
#if VULKAN_HPP_DISPATCH_LOADER_DYNAMIC == 1
    VULKAN_HPP_DEFAULT_DISPATCHER.init(*this->m_vkInstance);
#endif
#if ALICEVISION_IS_DEFINED(ALICEVISION_USE_VULKAN_VALIDATION)
    this->m_debugMessenger = vk::SharedDebugUtilsMessengerEXT(ALICEVISION_THROW_ON_VULKAN_HPP_ERROR(this->m_vkInstance->createDebugUtilsMessengerEXT(debugHandlerInfo), "Failed to create DebugUtilsMessengerEXT!"), this->m_vkInstance);
#endif
    // Enumerate phyiscal devices
    for(auto const& physDev : ALICEVISION_THROW_ON_VULKAN_HPP_ERROR(this->m_vkInstance->enumeratePhysicalDevices(), "Failed to enumerate physical devices!")) {
        // Get physical devices properties
        const vk::PhysicalDeviceProperties physicalDeviceProperties = physDev.getProperties();
        this->m_physDevs.insert(std::make_pair(physicalDeviceProperties.deviceID, vk::SharedPhysicalDevice(physDev, this->m_vkInstance)));
        // Get the queues
        const std::vector<vk::QueueFamilyProperties>& availableQueues = physDev.getQueueFamilyProperties();
        uint32_t queueIdx = UINT32_MAX;
        uint32_t currentIdx = 0;
        for(auto const& queue : availableQueues) {
            if(queue.queueFlags & vk::QueueFlags(vk::QueueFlagBits::eCompute | vk::QueueFlagBits::eTransfer)) {
                queueIdx = currentIdx;
            }
            currentIdx++;
        }
        if(queueIdx == UINT32_MAX) {
            ALICEVISION_LOG_WARNING("Vulkan device " << physicalDeviceProperties.deviceName << " does not have a queue with vk::QueueFlagBits::eCompute and vk::QueueFlagBits::eTransfer. Continuing...");
            continue;
        }
        float priority = 1.f;
        const vk::DeviceQueueCreateInfo queueInfos[1] = {vk::DeviceQueueCreateInfo({}, queueIdx, 1, &priority)};
        vk::DeviceCreateInfo deviceInfo({}, queueInfos);
        // Check if VK_KHR_portability_subset must be enabled
        const std::vector<vk::ExtensionProperties>& availableDeviceExtensions = ALICEVISION_THROW_ON_VULKAN_HPP_ERROR(physDev.enumerateDeviceExtensionProperties(), "Failed to enumerate available phyiscal device extensions!");
        // Required and optional extensions
        std::vector<const char*> requiredExtensions = {
            VK_KHR_SHADER_FLOAT16_INT8_EXTENSION_NAME,
            VK_KHR_8BIT_STORAGE_EXTENSION_NAME,
            VK_KHR_16BIT_STORAGE_EXTENSION_NAME,
            VK_EXT_SCALAR_BLOCK_LAYOUT_EXTENSION_NAME
        };

        std::vector<const char*> optionalExtensions = {
            "VK_KHR_portability_subset",
            VK_EXT_MEMORY_BUDGET_EXTENSION_NAME
        };

        // Query available extension names into a set for fast lookup
        std::unordered_set<std::string> availableExtNames;
        for (const auto& ext : availableDeviceExtensions) {
            availableExtNames.insert(ext.extensionName);
        }

        // Check for required extensions
        for (const char* ext : requiredExtensions) {
            if (availableExtNames.find(ext) == availableExtNames.end()) {
                throw VulkanException(
                    std::string("Required device extension not available: ") + ext,
                    vk::Result::eErrorExtensionNotPresent
                );
            }
        }

        // Add required and found optional extensions to final list
        std::vector<const char*> enabledExtensions = requiredExtensions;
        for (const char* ext : optionalExtensions) {
            if (availableExtNames.find(ext) != availableExtNames.end()) {
                enabledExtensions.push_back(ext);
            }
        }

        // Build a feature chain using StructureChain
        vk::StructureChain<
            vk::PhysicalDeviceFeatures2,
            vk::PhysicalDeviceScalarBlockLayoutFeaturesEXT,
            vk::PhysicalDevice8BitStorageFeaturesKHR,
            vk::PhysicalDevice16BitStorageFeaturesKHR,
            vk::PhysicalDeviceShaderFloat16Int8FeaturesKHR
        > featureChain;

        auto& scalarBlock = featureChain.get<vk::PhysicalDeviceScalarBlockLayoutFeaturesEXT>();
        scalarBlock.scalarBlockLayout = VK_TRUE;

        auto& storage8 = featureChain.get<vk::PhysicalDevice8BitStorageFeaturesKHR>();
        storage8.storageBuffer8BitAccess = VK_TRUE;
        storage8.uniformAndStorageBuffer8BitAccess = VK_TRUE;

        auto& storage16 = featureChain.get<vk::PhysicalDevice16BitStorageFeaturesKHR>();
        storage16.storageBuffer16BitAccess = VK_TRUE;
        storage16.uniformAndStorageBuffer16BitAccess = VK_TRUE;

        auto& float16int8 = featureChain.get<vk::PhysicalDeviceShaderFloat16Int8FeaturesKHR>();
        float16int8.shaderFloat16 = VK_TRUE;
        float16int8.shaderInt8 = VK_TRUE;


        deviceInfo.setEnabledExtensionCount(static_cast<uint32_t>(enabledExtensions.size()));
        deviceInfo.setPEnabledExtensionNames(enabledExtensions);
        deviceInfo.setPNext(&featureChain);
        /*
        bool portabilityExtensionFound = false;
        bool scalarLayoutExtensionFound = false;
        bool deviceMemoryExtensionFound = false;
        for(auto const& extension : availableDeviceExtensions) {
            if(strcmp(extension.extensionName, "VK_KHR_portability_subset") == 0) {
                portabilityExtensionFound = true;
                continue;
            }
            if(strcmp(extension.extensionName, VK_EXT_SCALAR_BLOCK_LAYOUT_EXTENSION_NAME) == 0)
            {
                scalarLayoutExtensionFound = true;
                continue;
            }
            if(strcmp(extension.extensionName, VK_EXT_MEMORY_BUDGET_EXTENSION_NAME) == 0)
            {
                deviceMemoryExtensionFound = true;
                continue;
            }
            if(portabilityExtensionFound && scalarLayoutExtensionFound && deviceMemoryExtensionFound) {
                break;
            }
        }
        if(!scalarLayoutExtensionFound)
            throw VulkanException("VK_EXT_SCALAR_BLOCK_LAYOUT_EXTENSION_NAME was requested, but is not available in this driver!", vk::Result::eErrorExtensionNotPresent);
        std::vector deviceExtensionNames = { VK_EXT_SCALAR_BLOCK_LAYOUT_EXTENSION_NAME };
        deviceInfo.setEnabledExtensionCount(1);
        if(portabilityExtensionFound) {
            deviceExtensionNames.push_back("VK_KHR_portability_subset");
            deviceInfo.setEnabledExtensionCount(2);
        }
        if(deviceMemoryExtensionFound) {
            deviceExtensionNames.push_back(VK_EXT_MEMORY_BUDGET_EXTENSION_NAME);
            deviceInfo.setEnabledExtensionCount(3);
        }
        // We require VK_KHR_8bit_storage, VK_KHR_16bit_storage and VK_KHR_shader_float16_int8 to replicate the CUDA implementation
        std::vector<const char*> requiredExtensions = { VK_KHR_SHADER_FLOAT16_INT8_EXTENSION_NAME, VK_KHR_8BIT_STORAGE_EXTENSION_NAME, VK_KHR_16BIT_STORAGE_EXTENSION_NAME };
        deviceExtensionNames.append_range(requiredExtensions);
        deviceInfo.setEnabledExtensionCount(deviceExtensionNames.size());
        deviceInfo.setPEnabledExtensionNames(deviceExtensionNames);
        // Set the required next structure
        vk::PhysicalDeviceScalarBlockLayoutFeaturesEXT scalarBlockLayoutInfo(vk::True);
        deviceInfo.setPNext(scalarBlockLayoutInfo);
        // TODO: Append the other required structures
        */
        const vk::Device logicalDevice = ALICEVISION_THROW_ON_VULKAN_HPP_ERROR(physDev.createDevice(deviceInfo), "Failed to create logical device!");
        this->m_devs.insert(std::make_pair(physicalDeviceProperties.deviceID, vk::SharedDevice(logicalDevice)));
        // Create the VmaAllocator
        VmaVulkanFunctions vulkanFunctions = {};
        vulkanFunctions.vkGetInstanceProcAddr = &vkGetInstanceProcAddr;
        vulkanFunctions.vkGetDeviceProcAddr = &vkGetDeviceProcAddr;
        VmaAllocatorCreateInfo allocatorCreateInfo = {};
        allocatorCreateInfo.flags = {};
        allocatorCreateInfo.vulkanApiVersion = VK_API_VERSION_1_2;
        allocatorCreateInfo.physicalDevice = *this->m_physDevs.at(physicalDeviceProperties.deviceID);
        allocatorCreateInfo.device = *this->m_devs.at(physicalDeviceProperties.deviceID);
        allocatorCreateInfo.instance = *this->m_vkInstance;
        allocatorCreateInfo.pVulkanFunctions = &vulkanFunctions;
        // Create the allocator
        this->m_allocators.insert(std::make_pair(physicalDeviceProperties.deviceID, std::shared_ptr<VmaAllocator_T>(ALICEVISION_THROW_ON_VULKAN_ERROR<VmaAllocator>(vmaCreateAllocator, "Failed to create Vulkan Memory Allocator!", &allocatorCreateInfo), [](const VmaAllocator allocator){ vmaDestroyAllocator(allocator); })));
        // Get the associated queue with the index
        this->m_queues.insert(std::make_pair(physicalDeviceProperties.deviceID, vk::SharedQueue(this->m_devs.at(physicalDeviceProperties.deviceID)->getQueue(queueIdx, 0), this->m_devs.at(physicalDeviceProperties.deviceID))));
        // Create a command pool
        vk::CommandPoolCreateInfo commandPoolInfo(vk::CommandPoolCreateFlags(vk::CommandPoolCreateFlagBits::eResetCommandBuffer | vk::CommandPoolCreateFlagBits::eTransient), queueIdx);
        this->m_cmdPools.insert(std::make_pair(physicalDeviceProperties.deviceID, vk::SharedCommandPool(ALICEVISION_THROW_ON_VULKAN_HPP_ERROR(this->m_devs.at(physicalDeviceProperties.deviceID)->createCommandPool(commandPoolInfo), "Failed to create command pool!"), this->m_devs.at(physicalDeviceProperties.deviceID))));
        // Create a command buffer
        vk::CommandBufferAllocateInfo cmdBufferInfo(*this->m_cmdPools.at(physicalDeviceProperties.deviceID), vk::CommandBufferLevel::ePrimary, 1);
        this->m_cmdBuffers.insert(std::make_pair(physicalDeviceProperties.deviceID, vk::SharedCommandBuffer(ALICEVISION_THROW_ON_VULKAN_HPP_ERROR(this->m_devs.at(physicalDeviceProperties.deviceID)->allocateCommandBuffers(cmdBufferInfo), "Failed to allocate command buffer!").front(), this->m_devs.at(physicalDeviceProperties.deviceID), this->m_cmdPools.at(physicalDeviceProperties.deviceID))));
        // Try to load pipeline cache from disk, else create a new one
        auto [data, size] = GetPipelineCache(physicalDeviceProperties.deviceID);
        vk::PipelineCacheCreateInfo pipelineCacheInfo({}, 0);
        if(data != nullptr && size != 0) {
            pipelineCacheInfo.setPInitialData(data);
            pipelineCacheInfo.setInitialDataSize(size);
        }
        this->m_pipelineCaches.insert(std::make_pair(physicalDeviceProperties.deviceID, vk::SharedPipelineCache(ALICEVISION_THROW_ON_VULKAN_HPP_ERROR(this->m_devs.at(physicalDeviceProperties.deviceID)->createPipelineCache(pipelineCacheInfo), "Failed to create pipeline cache!"), this->m_devs.at(physicalDeviceProperties.deviceID))));
    }
}

VulkanManager::~VulkanManager()
{
    // Save any Pipeline Cache to disk
    for(auto const& [deviceID, cache] : this->m_pipelineCaches) {
        auto const& [result, data] = this->m_devs.at(deviceID)->getPipelineCacheData(*cache);
        if(result != vk::Result::eSuccess) {
            ALICEVISION_LOG_WARNING("Could not retrieve cache data for device: " << deviceID << "!");
        }
        try {
            SavePipelineCache(deviceID, data);
        } catch(std::exception const& e) {
            ALICEVISION_LOG_ERROR("Failed to save pipeline cache for device: " << deviceID << "! Error: " << e.what());
        }
    }
    // Free the imported caches
    for(const auto& [cache, size] : std::views::values(m_cacheData)) {
        if(cache != nullptr) {
            free(cache);
        }
    }
}

VulkanManager* VulkanManager::getInstance()
{
    // Acquire lock before checking instance
    std::lock_guard lock(m_initMutex);
    static VulkanManager instance;
    if (m_this == nullptr) {
        m_this = &instance;
    }
    return m_this;
}

std::pair<char*, size_t> VulkanManager::GetPipelineCache(uint32_t deviceID)
{
    std::filesystem::path base;

#ifdef _WIN32
    wchar_t* path = nullptr;
    if (SUCCEEDED(SHGetKnownFolderPath(FOLDERID_LocalAppData, 0, NULL, &path))) {
        base = std::filesystem::path(path);
        CoTaskMemFree(path);
    }
#elif defined(__APPLE__)
    const char* home = getenv("HOME");
    if (!home) home = getpwuid(getuid())->pw_dir;
    base = std::filesystem::path(home) / "Library" / "Caches";
#elif defined(__linux__)
    const char* xdg = getenv("XDG_CACHE_HOME");
    if (xdg) {
        base = std::filesystem::path(xdg);
    } else {
        const char* home = getenv("HOME");
        if (!home) home = getpwuid(getuid())->pw_dir;
        base = std::filesystem::path(home) / ".cache";
    }
#endif

    const std::filesystem::path full_path = base / "org.aliceVision.AVGPU" / "VulkanPipelineCaches";
    std::filesystem::create_directories(full_path);

    // Check if a cache file for this deviceID exists, else return early with nullptr
    const std::filesystem::path cache_path = full_path / (std::to_string(deviceID) + ".bin");
    if (!std::filesystem::exists(cache_path))
        return std::make_pair(nullptr, 0);

    // Load the data
    std::ifstream cacheFile(cache_path, std::ios::binary | std::ios::ate | std::ios::in);
    if (!cacheFile)
        return std::make_pair(nullptr, 0);
    const std::streamsize size = cacheFile.tellg();
    cacheFile.seekg(0, std::ios::beg);
    m_cacheData.insert(std::make_pair(deviceID, std::make_pair(static_cast<char*>(malloc(size)), size)));
    if(m_cacheData.at(deviceID).first == nullptr) {
        m_cacheData.erase(deviceID);
        return std::make_pair(nullptr, 0);
    }
    if(!cacheFile.read(m_cacheData.at(deviceID).first, size)) {
        delete m_cacheData.at(deviceID).first;
        m_cacheData.erase(deviceID);
        return std::make_pair(nullptr, 0);
    }

    return m_cacheData.at(deviceID);
}

void VulkanManager::SavePipelineCache(uint32_t deviceID, const std::vector<unsigned char>& data)
{
    std::filesystem::path base;

#ifdef _WIN32
    wchar_t* path = nullptr;
    if (SUCCEEDED(SHGetKnownFolderPath(FOLDERID_LocalAppData, 0, NULL, &path))) {
        base = std::filesystem::path(path);
        CoTaskMemFree(path);
    }
#elif defined(__APPLE__)
    const char* home = getenv("HOME");
    if (!home) home = getpwuid(getuid())->pw_dir;
    base = std::filesystem::path(home) / "Library" / "Caches";
#elif defined(__linux__)
    const char* xdg = getenv("XDG_CACHE_HOME");
    if (xdg) {
        base = std::filesystem::path(xdg);
    } else {
        const char* home = getenv("HOME");
        if (!home) home = getpwuid(getuid())->pw_dir;
        base = std::filesystem::path(home) / ".cache";
    }
#endif

    const std::filesystem::path full_path = base / "org.aliceVision.AVGPU" / "VulkanPipelineCaches";
    std::filesystem::create_directories(full_path);  // Ensure it exists

    // Check if a cache file for this deviceID exists, else return early with nullptr
    const std::filesystem::path cache_path = full_path / (std::to_string(deviceID) + ".bin");

    // Save the data
    std::ofstream cacheFile(cache_path, std::ios::binary | std::ios::out | std::ios::trunc);
    if(!cacheFile)
        ALICEVISION_THROW_ERROR("Failed to write cache file!");
    cacheFile << data.data();
    cacheFile.flush();
    cacheFile.close();
}

std::mutex VulkanManager::m_initMutex;
VulkanManager* VulkanManager::m_this = nullptr;
std::unordered_map<uint32_t, std::pair<char*, size_t>> VulkanManager::m_cacheData = {};

VulkanCommandManager* VulkanCommandManager::getInstance(uint32_t deviceID)
{
    std::lock_guard lock(m_initMutex);
    auto it = m_these.find(deviceID);
    if (it != m_these.end()) {
        return it->second;
    }

    // Create a new instance and store it per deviceID
    VulkanCommandManager* newInstance = new VulkanCommandManager(deviceID);
    m_these[deviceID] = newInstance;

    VulkanManager::getInstance()->m_fences[deviceID] = {};
    VulkanManager::getInstance()->m_shaderModules[deviceID] = {};
    VulkanManager::getInstance()->m_pipelines[deviceID] = {};
    VulkanManager::getInstance()->m_pipelineLayouts[deviceID] = {};
    VulkanManager::getInstance()->m_descriptorSetLayouts[deviceID] = {};
    VulkanManager::getInstance()->m_descriptorPools[deviceID] = {};

    return newInstance;
}

const uint32_t VulkanManager::getPriorityDeviceID() const
{
    uint32_t deviceIDCurrent = UINT32_MAX;
    vk::DeviceSize maxMem = 0;
    for(const auto& [deviceID, device] : this->m_physDevs) {
        const vk::PhysicalDeviceProperties& deviceProperties = device->getProperties();
        const vk::PhysicalDeviceMemoryProperties& deviceMemoryProperties = device->getMemoryProperties();
        vk::DeviceSize availableMem = 0;
        for(auto const& heaps : deviceMemoryProperties.memoryHeaps) {
            availableMem += heaps.size;
        }
        if(deviceProperties.deviceType == vk::PhysicalDeviceType::eDiscreteGpu) {
            if(availableMem >= 3584000000) {
                return deviceID;
            }
        }
        if(maxMem < availableMem) {
            maxMem = availableMem;
            deviceIDCurrent = deviceID;
        }
    }
    if(deviceIDCurrent != UINT32_MAX) {
        return deviceIDCurrent;
    }
    ALICEVISION_THROW_ERROR("No Vulkan device found!");
}

VulkanCommandManager* VulkanCommandManager::begin()
{
    constexpr vk::CommandBufferBeginInfo beginInfo({ vk::CommandBufferUsageFlagBits::eOneTimeSubmit }, {});
    ALICEVISION_THROW_ON_VULKAN_HPP_RESULT_ERROR(VulkanManager::getInstance()->m_cmdBuffers.at(this->m_deviceID)->begin(beginInfo), "Failed to begin recording into the command buffer!");
    VulkanManager::getInstance()->m_fences.at(this->m_deviceID).emplace_back(ALICEVISION_THROW_ON_VULKAN_HPP_ERROR(VulkanManager::getInstance()->m_devs.at(this->m_deviceID)->createFence({ vk::FenceCreateFlagBits::eSignaled }), "Failed to create fence!"), VulkanManager::getInstance()->m_devs.at(this->m_deviceID));
    return this;
}

VulkanCommandManager* VulkanCommandManager::end()
{
    ALICEVISION_THROW_ON_VULKAN_HPP_RESULT_ERROR(VulkanManager::getInstance()->m_cmdBuffers.at(this->m_deviceID)->end(), "Failed to end recording into the command buffer!");
    return this;
}

VulkanCommandManager* VulkanCommandManager::reset()
{
    if(!VulkanManager::getInstance()->m_shaderModules.at(this->m_deviceID).empty())
        VulkanManager::getInstance()->m_shaderModules.at(this->m_deviceID).pop_front();
    if(!VulkanManager::getInstance()->m_pipelines.at(this->m_deviceID).empty())
        VulkanManager::getInstance()->m_pipelines.at(this->m_deviceID).pop_front();
    if(!VulkanManager::getInstance()->m_pipelineLayouts.at(this->m_deviceID).empty())
        VulkanManager::getInstance()->m_pipelineLayouts.at(this->m_deviceID).pop_front();
    if(!VulkanManager::getInstance()->m_descriptorSetLayouts.at(this->m_deviceID).empty())
        VulkanManager::getInstance()->m_descriptorSetLayouts.at(this->m_deviceID).pop_front();
    if(!VulkanManager::getInstance()->m_descriptorPools.at(this->m_deviceID).empty())
        VulkanManager::getInstance()->m_descriptorPools.at(this->m_deviceID).pop_front();
    this->m_workgroupSize = vk::Extent3D(0u, 0u, 0u);
    this->m_ShaderModule = std::make_pair(nullptr, 0);
    this->m_pushConstantTemporaryStorage = std::make_pair(nullptr, 0);
    this->m_boundMemoryInstances.clear();
    ALICEVISION_THROW_ON_VULKAN_HPP_RESULT_ERROR(VulkanManager::getInstance()->m_cmdBuffers.at(this->m_deviceID)->reset({}), "Failed to reset the command buffer!");
    return this;
}

VulkanCommandManager* VulkanCommandManager::standardOp(const std::function<void(vk::SharedCommandBuffer cmdBuffer)>& baseFunc)
{
    baseFunc(VulkanManager::getInstance()->m_cmdBuffers.at(this->m_deviceID));
    return this;
}

VulkanCommandManager* VulkanCommandManager::shader(const std::pair<const uint32_t*, size_t>& shaderModule)
{
    this->m_ShaderModule = shaderModule;
    return this;
}

VulkanCommandManager* VulkanCommandManager::workgroups(const vk::Extent3D& workgroups)
{
    this->m_workgroupSize = workgroups;
    return this;
}

VulkanCommandManager* VulkanCommandManager::dispatch()
{
    std::vector<vk::DescriptorSetLayoutBinding> layoutBindings;
    int layoutBindingIdx = 0;
    for(auto const& mem : this->m_boundMemoryInstances) {
        layoutBindings.emplace_back(layoutBindingIdx, mem->getDescriptorType(), 1, vk::ShaderStageFlagBits::eCompute);
        layoutBindingIdx++;
    }
    const vk::DescriptorSetLayoutCreateInfo descriptorSetLayoutInfo({}, layoutBindings);
    VulkanManager::getInstance()->m_descriptorSetLayouts.at(this->m_deviceID).emplace_back(ALICEVISION_THROW_ON_VULKAN_HPP_ERROR(VulkanManager::getInstance()->m_devs.at(this->m_deviceID)->createDescriptorSetLayout(descriptorSetLayoutInfo), "Failed to create descriptor set layout!"), VulkanManager::getInstance()->m_devs.at(this->m_deviceID));
    vk::PushConstantRange pushConstantRange(vk::ShaderStageFlagBits::eCompute, 0, 0);
    if(this->m_pushConstantTemporaryStorage.first != nullptr && this->m_pushConstantTemporaryStorage.second != 0) {
        pushConstantRange.setSize(this->m_pushConstantTemporaryStorage.second);
    }
    const vk::DescriptorSetLayout descriptorSetLayoutTemp = *VulkanManager::getInstance()->m_descriptorSetLayouts.at(this->m_deviceID).back();
    const vk::PipelineLayoutCreateInfo pipelineLayoutInfo({}, 1, &descriptorSetLayoutTemp, pushConstantRange.size > 0 ? 1u : 0u, pushConstantRange.size > 0 ? &pushConstantRange : nullptr);
    VulkanManager::getInstance()->m_pipelineLayouts.at(this->m_deviceID).emplace_back(ALICEVISION_THROW_ON_VULKAN_HPP_ERROR(VulkanManager::getInstance()->m_devs.at(this->m_deviceID)->createPipelineLayout(pipelineLayoutInfo), "Failed to create pipeline layout!"), VulkanManager::getInstance()->m_devs.at(this->m_deviceID));
    const vk::ShaderModuleCreateInfo shaderModuleInfo({}, this->m_ShaderModule.second, this->m_ShaderModule.first);
    VulkanManager::getInstance()->m_shaderModules.at(this->m_deviceID).emplace_back(ALICEVISION_THROW_ON_VULKAN_HPP_ERROR(VulkanManager::getInstance()->m_devs.at(this->m_deviceID)->createShaderModule(shaderModuleInfo), "Failed to create shader module!"), VulkanManager::getInstance()->m_devs.at(this->m_deviceID));
    const vk::PipelineShaderStageCreateInfo pipelineStageInfo({}, vk::ShaderStageFlagBits::eCompute, *VulkanManager::getInstance()->m_shaderModules.at(this->m_deviceID).back(), "main", {});
    const vk::ComputePipelineCreateInfo computePipelineInfo({}, pipelineStageInfo, *VulkanManager::getInstance()->m_pipelineLayouts.at(this->m_deviceID).back());
    VulkanManager::getInstance()->m_pipelines.at(this->m_deviceID).emplace_back(ALICEVISION_THROW_ON_VULKAN_HPP_ERROR(VulkanManager::getInstance()->m_devs.at(this->m_deviceID)->createComputePipeline(*VulkanManager::getInstance()->m_pipelineCaches.at(this->m_deviceID), computePipelineInfo), "Failed to create compute pipeline!"), VulkanManager::getInstance()->m_devs.at(this->m_deviceID));
    std::vector<vk::DescriptorPoolSize> poolSizes;
    std::unordered_map<vk::DescriptorType, uint32_t> distinctDescriptors;
    for(auto const& buf : this->m_boundMemoryInstances) {
        if(distinctDescriptors.contains(buf->getDescriptorType())) {
            uint32_t count = distinctDescriptors.at(buf->getDescriptorType());
            distinctDescriptors.insert_or_assign(buf->getDescriptorType(), count + 1);
        } else {
            distinctDescriptors.insert_or_assign(buf->getDescriptorType(), 1);
        }
    }
    for(auto const& [type, count] : distinctDescriptors) {
        poolSizes.emplace_back(type, count);
    }
    const vk::DescriptorPoolCreateInfo descriptorPoolInfo({}, 1, poolSizes);
    VulkanManager::getInstance()->m_descriptorPools.at(this->m_deviceID).emplace_back(ALICEVISION_THROW_ON_VULKAN_HPP_ERROR(VulkanManager::getInstance()->m_devs.at(this->m_deviceID)->createDescriptorPool(descriptorPoolInfo), "Failed to create descriptor pool!"), VulkanManager::getInstance()->m_devs.at(this->m_deviceID));
    vk::DescriptorSetLayout descriptorSetLayout = *VulkanManager::getInstance()->m_descriptorSetLayouts.at(this->m_deviceID).back();
    vk::DescriptorSetAllocateInfo descriptorSetAllocateInfo(*VulkanManager::getInstance()->m_descriptorPools.at(this->m_deviceID).back(), 1, &descriptorSetLayout);
    vk::DescriptorSet descriptorSet = ALICEVISION_THROW_ON_VULKAN_HPP_ERROR(VulkanManager::getInstance()->m_devs.at(this->m_deviceID)->allocateDescriptorSets(descriptorSetAllocateInfo), "Failed to allocate descriptor set!").front();
    std::vector<vk::DescriptorBufferInfo> bufferInfos = {};
    std::vector<vk::DescriptorImageInfo> imageInfos = {};
    for(auto const& buf : this->m_boundMemoryInstances) {
        if(buf->isVkImage()) {
            if(buf->getDescriptorType() == vk::DescriptorType::eStorageImage) {
                if(buf->getMipLevel() != -1) {
                    imageInfos.emplace_back(nullptr, *buf->getImageViewForMipLevel(static_cast<uint32_t>(buf->getMipLevel())), buf->getCurrentImageLayout());
                } else {
                    imageInfos.emplace_back(nullptr, *buf->getImageViewForAllMipMapLevels(), buf->getCurrentImageLayout());
                }
            } else {
                imageInfos.emplace_back(*buf->getSampler(), *buf->getImageViewForAllMipMapLevels(), buf->getCurrentImageLayout());
            }
        } else {
            bufferInfos.emplace_back(*buf->getVkBuffer(), 0, buf->getMemorySize());
        }
    }
    std::vector<vk::WriteDescriptorSet> writes = {};
    int imageDescriptorIdx = 0;
    int bufferDescriptorIdx = 0;
    for(auto const& buf : this->m_boundMemoryInstances) {
        if(buf->isVkImage()) {
            writes.emplace_back(descriptorSet, buf->getBindingIndex(), 0, 1, buf->getDescriptorType(), &imageInfos.at(imageDescriptorIdx));
            imageDescriptorIdx++;
        } else {
            writes.emplace_back(descriptorSet, buf->getBindingIndex(), 0, 1, buf->getDescriptorType(), nullptr, &bufferInfos.at(bufferDescriptorIdx));
            bufferDescriptorIdx++;
        }
    }
    VulkanManager::getInstance()->m_devs.at(this->m_deviceID)->updateDescriptorSets(writes, {});
    VulkanManager::getInstance()->m_cmdBuffers.at(this->m_deviceID)->bindPipeline(vk::PipelineBindPoint::eCompute, *VulkanManager::getInstance()->m_pipelines.at(this->m_deviceID).back());
    VulkanManager::getInstance()->m_cmdBuffers.at(this->m_deviceID)->bindDescriptorSets(vk::PipelineBindPoint::eCompute, *VulkanManager::getInstance()->m_pipelineLayouts.at(this->m_deviceID).back(), 0, { descriptorSet }, {});
    if(this->m_pushConstantTemporaryStorage.first != nullptr && this->m_pushConstantTemporaryStorage.second != 0) {
        VulkanManager::getInstance()->m_cmdBuffers.at(this->m_deviceID)->pushConstants(*VulkanManager::getInstance()->m_pipelineLayouts.at(this->m_deviceID).back(), vk::ShaderStageFlagBits::eCompute, 0, this->m_pushConstantTemporaryStorage.second, this->m_pushConstantTemporaryStorage.first);
    }
    VulkanManager::getInstance()->m_cmdBuffers.at(this->m_deviceID)->dispatch(this->m_workgroupSize.width, this->m_workgroupSize.height, this->m_workgroupSize.depth);
    return this;
}

VulkanCommandManager *VulkanCommandManager::submit()
{
    ALICEVISION_THROW_ON_VULKAN_HPP_RESULT_ERROR(VulkanManager::getInstance()->m_devs.at(this->m_deviceID)->resetFences({ *VulkanManager::getInstance()->m_fences.at(this->m_deviceID).back() }), "Failed to reset fence!");
    vk::CommandBuffer cmdBuf = *VulkanManager::getInstance()->m_cmdBuffers.at(this->m_deviceID);
    vk::SubmitInfo submitInfo(0, nullptr, nullptr, 1, &cmdBuf, 0, nullptr);
    ALICEVISION_THROW_ON_VULKAN_HPP_RESULT_ERROR(VulkanManager::getInstance()->m_queues.at(this->m_deviceID)->submit({ submitInfo }, *VulkanManager::getInstance()->m_fences.at(this->m_deviceID).back()), "Failed to submit command buffer!");
    return this;
}

VulkanCommandManager* VulkanCommandManager::wait()
{
    if(VulkanManager::getInstance()->m_fences.at(this->m_deviceID).empty())
        return this;
    const vk::SharedFence fence = VulkanManager::getInstance()->m_fences.at(this->m_deviceID).front();
    VulkanManager::getInstance()->m_fences.at(this->m_deviceID).pop_front();
    if(VulkanManager::getInstance()->m_devs.at(this->m_deviceID)->getFenceStatus(*fence) != vk::Result::eSuccess) {
        ALICEVISION_THROW_ON_VULKAN_HPP_RESULT_ERROR(VulkanManager::getInstance()->m_devs.at(this->m_deviceID)->waitForFences({ *fence }, true, UINT64_MAX), "Failed to wait for fence!");
    }
    return this;
}

std::mutex VulkanCommandManager::m_initMutex;
std::unordered_map<uint32_t, VulkanCommandManager*> VulkanCommandManager::m_these = {};

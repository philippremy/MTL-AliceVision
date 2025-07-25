// This file is part of the AliceVision project.
// Copyright (c) 2025 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <vulkan/vulkan.hpp>
#include <vulkan/vulkan_shared.hpp>

#include <AV/config.hpp>

#include "vk_mem_alloc.h"

#include <unordered_map>
#include <mutex>
#include <vector>
#include <deque>
#include <functional>
#include <type_traits>
#include <utility>

/**
 * Forward declare VulkanMemoryBase and derived classes
 */
template<typename Type>
class VulkanMemoryBase;
template<typename Type>
class VulkanImage;

/**
 * Concept for Buffer Types available
 *
 * Can either be a vk::SharedBuffer or a vk::SharedImage
 */
template<class Buffer>
concept BufferType = std::is_same_v<Buffer, vk::SharedBuffer> || std::is_same_v<Buffer, vk::SharedImage>;

/**
 * A manager for handling Vulkan Devices and Instances
 *
 * This class abstracts away the tedious setup of creating a Vulkan
 * context. This includes managing creation, lifetime and destruction
 * of Vulkan instances, devices, queues and command pools.
 *
 * @note This class uses the vk::SharedHandle<T> types internally.
 * @note This class uses the Singleton design pattern.
 * @warning This class is only internally synchronized for initialization, any
 * further synchronization must be ensured by the caller.
 */
class VulkanManager final
{
    /*
     * VulkanCommandManager will be friends, so it can easily update all needed properties.
     */
    friend class VulkanCommandManager;

    /*
     * VulkanMemoryBase and derived classes will be friends, so it can easily update all needed properties.
     */
    template <typename Type>
    friend class VulkanMemoryBase;
    template<BufferType BufferType, typename Type>
    friend class VulkanMemory;
    template<typename Type>
    friend class VulkanBuffer;
    template<typename Type>
    friend class VulkanImage;

public:
    VulkanManager(const VulkanManager&) = delete;               // Delete implicit copy constructor
    VulkanManager& operator=(const VulkanManager&) = delete;    // Delete implicit assignment constructor

    /**
     * Gets a pointer to the static global instance
     *
     * @throws VulkanException Throws if creation of the Vulkan instance, the
     * enumeration of Vulkan physical devices or the creation of Vulkan logical
     * devices fails.
     * @throws std::runtime_exception Throws, if no devices with the required
     * capabilities can be found.
     *
     * @return A pointer to an initialized instance of VulkanManager
     */
    static VulkanManager* getInstance();

    /**
     * This function iterates over the devices and checks which device provides
     * the best performance for using in DepthMap Vulkan computations.
     *
     * @return Returns the deviceID with the best fitted device
     *
     * @note It checks if a discrete GPU is available and if this GPU has
     * > 3.5GB memory, it selects it. Otherwise it will fall back to the GPU
     * with the biggest amount of GPU memory (can be an integrated GPU then).
     */
    const uint32_t getPriorityDeviceID() const;

public:
    VulkanManager();    // Internal constructor
    ~VulkanManager();   // Internal destructor

    static std::pair<char*, size_t> GetPipelineCache(uint32_t deviceID);
    static void SavePipelineCache(uint32_t deviceID, const std::vector<unsigned char>& data);

    static std::mutex m_initMutex;                                             // Mutex used for initialization
    static VulkanManager* m_this;                                              // Static Member holding a pointer to self
    static std::unordered_map<uint32_t, std::pair<char*, size_t>> m_cacheData; // Static Member holding the disk-loaded cache data per de

    vk::SharedInstance m_vkInstance;                                                         // The global vk::SharedInstance (globally unique, context)
#if ALICEVISION_IS_DEFINED(ALICEVISION_USE_VULKAN_VALIDATION)
    vk::SharedDebugUtilsMessengerEXT m_debugMessenger;                                       // On Debug builds, this holds the debug messenger extension object
#endif
    std::unordered_map<uint32_t, vk::SharedPhysicalDevice> m_physDevs = {};                                 // A map holding vk::SharedPhysicalDevice per deviceID
    std::unordered_map<uint32_t, vk::SharedDevice> m_devs = {};                                             // A map holding vk::SharedDevices per deviceID
    std::unordered_map<uint32_t, std::shared_ptr<VmaAllocator_T>> m_allocators = {};                        // A map holding shared_ptrs to a VmaAllocator per deviceID (with custom deleter)
    std::unordered_map<uint32_t, vk::SharedQueue> m_queues = {};                                            // A map holding vk::SharedQueue per deviceID
    std::unordered_map<uint32_t, vk::SharedCommandPool> m_cmdPools = {};                                    // A map holding vk::SharedCommandPool per deviceID
    std::unordered_map<uint32_t, vk::SharedCommandBuffer> m_cmdBuffers = {};                                // A map holding vk::SharedCommandBuffer per deviceID
    std::unordered_map<uint32_t, vk::SharedPipelineCache> m_pipelineCaches = {};                            // A map holding vk::SharedPipelineCache per deviceID
    std::unordered_map<uint32_t, std::deque<vk::SharedFence>> m_fences = {};                                // A map holding a map to vk::SharedFences per deviceID
    std::unordered_map<uint32_t, std::deque<vk::SharedShaderModule>> m_shaderModules = {};                  // A map holding vk::SharedShaderModules per deviceID
    std::unordered_map<uint32_t, std::deque<vk::SharedPipeline>> m_pipelines = {};                          // A map holding the current vk::SharedPipeline per deviceID
    std::unordered_map<uint32_t, std::deque<vk::SharedPipelineLayout>> m_pipelineLayouts = {};              // A map holding the current vk::SharedPipelineLayout per deviceID
    std::unordered_map<uint32_t, std::deque<vk::SharedDescriptorSetLayout>> m_descriptorSetLayouts = {};    // A map holding the current vk::SharedDescriptorSetLayout per deviceID
    std::unordered_map<uint32_t, std::deque<vk::SharedDescriptorPool>> m_descriptorPools = {};              // A map holding the current vk::SharedDescriptorPool per deviceID
};

/**
 * A manager for handling Vulkan Commands
 *
 * This class facilitates everything around using Command Buffers in Vulkan.
 * It can explicitly bind Buffers and Images, begin and end recording into
 * the Command Buffer, submit it and wait for it to finish executing.
 *
 * @note This class follows the Singleton approach. For each logical device
 * exactly one instance will be created.
 * @warning This class is only internally synchronized for initialization, any
 * further synchronization must be ensured by the caller.
 */
class VulkanCommandManager final
{

public:
    VulkanCommandManager(const VulkanCommandManager&) = delete;               // Delete implicit copy constructor
    VulkanCommandManager& operator=(const VulkanCommandManager&) = delete;    // Delete implicit assignment constructor

    /**
     * Gets a pointer to the static global instance
     *
     * @param[in] deviceID The deviceID for which the command buffer is
     * requested.
     *
     * @exceptsafe no-throw Will never throw an exception. If an instance for the
     * requested deviceID does not exist, it will be created.
     *
     * @return A pointer to an initialized instance of VulkanManager
     */
    static VulkanCommandManager* getInstance(uint32_t deviceID);

    /**
     * Begins recording into the command buffer
     *
     * @return Returns a pointer to this for method chaining
     *
     * @throw VulkanException is thrown on any occurring Vulkan error.
     */
    VulkanCommandManager* begin();

    /**
     * Ends recording into the command buffer
     *
     * @return Returns a pointer to this for method chaining
     *
     * @throw VulkanException is thrown on any occurring Vulkan error.
     */
    VulkanCommandManager* end();

    /**
     * Adds a Vulkan standard operation (like vkCopyBuffer) to the command
     * chain
     *
     * @param baseFunc A lamda function, which takes vk::SharedCommandBuffer
     * as parameter to call the standard operation on
     *
     * @throw VulkanException is thrown on any occurring Vulkan error.
     *
     * @return Returns a pointer to this for method chaining
     */
    VulkanCommandManager* standardOp(const std::function<void(vk::SharedCommandBuffer cmdBuffer)>& baseFunc);

    /**
     * Transfers an image layout as a pipeline barrier
     *
     * @tparam T The underlying data type
     *
     * @param[in] in_img The underlying image to operate on
     * @param[in] targetLayout The layout to transition into
     * @param[in] mipLevelStart The mip level to start on
     * @param[in] mipLevelEnd The mip level to end on
     *
     */
    template<typename T>
    VulkanCommandManager* transferImageLayout(const VulkanImage<T>& in_img, const vk::ImageLayout& targetLayout, uint32_t mipLevelStart, uint32_t mipLevelEnd);

    /**
     * Adds a buffer to be bound for the command buffer
     *
     * @param[in] buffer The buffer instance to be bound
     * @param[in] descriptorType The type of buffer
     *
     * @return Returns a pointer to this for method chaining
     *
     * @exceptsafe no-throw This method is guaranteed to not throw a runtime
     * exception.
     *
     * @note The buffer binding index will be determined by the sequence the
     * buffers are added by calling bind(). Starts at index 0 and goes up with
     * each call to bind(). The caller must ensure that the sequence of calls to
     * bind() matches the declaration in the GLSL shader.
     */
    template <typename Type>
    VulkanCommandManager* bind(const VulkanMemoryBase<Type>& buffer, const vk::DescriptorType& descriptorType);

    /**
     * Adds a buffer to be bound for the command buffer
     *
     * @param[in] buffer The buffer instance to be bound
     * @param[in] descriptorType The type of buffer
     * @param[in] mipLevel The miplevel to bind
     *
     * @return Returns a pointer to this for method chaining
     *
     * @throw std::runtime_error Throws if the buffer is not a vk::SharedImage
     * or the descriptor type is not vk::DescriptorType::eStorageImage
     *
     * @note The buffer binding index will be determined by the sequence the
     * buffers are added by calling bind(). Starts at index 0 and goes up with
     * each call to bind(). The caller must ensure that the sequence of calls to
     * bind() matches the declaration in the GLSL shader.
     * @warning This function does not work with buffers or images where the
     * descriptor type is vk::DescriptorType::eSampledImage!
     */
    template <typename Type>
    VulkanCommandManager* bind(const VulkanMemoryBase<Type>& buffer, const vk::DescriptorType& descriptorType, uint32_t mipLevel);

    /**
     * Adds push constants to be bound to the command buffer
     *
     * @tparam PushConstants The type of Push Constants which will be submitted
     * @param pushConstants A reference to initialized and preset PushConstants
     *
     * @return Returns a pointer to this for method chaining
     *
     * @exceptsafe no-throw This method is guaranteed to not throw a runtime
     * exception.
     *
     * @note The function will hold an internal pointer to the pushConstants
     * parameter. The value will not be copied until submit() is called.
     *
     * @warning A subsequent call of pushConstant() before calling submit() will
     * overwrite the previous value.
     * @warning The caller is responsible for making sure that the
     * pushConstants object outlives calls to submit(). Otherwise, the command
     * buffer submission will receive a null or a dangling pointer.
     * @warning The caller is responsible for making sure that the
     * pushConstants objects memory location will not change until submit() is
     * called. A reason for this could be implicit moves.
     * @warning Heap-allocated objects/members or pointers cannot be used as
     * push constants, because Vulkan passes by value, not by reference.
     * This technically excludes any dynamically sized arrays or container
     * types. However, one is free to use fixed size C-style arrays like this:
     *
     * These are all valid push constants
     *
     *     float fixedSizeArray[8];
     *
     *     struct PushConstants {
     *          float a;
     *          float b;
     *          int c;
     *          const char name[8];
     *          float values[4];
     *     }
     *
     */
    template<typename PushConstants>
    VulkanCommandManager* pushConstant(const PushConstants& pushConstants);

    /**
     * Resets the underlying command buffer, pipeline, etc.
     *
     * @throw VulkanException is thrown on any occurring Vulkan error.
     *
     * @return Returns a pointer to this for method chaining
     */
    VulkanCommandManager* reset();

    /**
     * Submits the command buffer in its current configuration for execution
     *
     * @return Returns a pointer to this for method chaining
     *
     * @throw VulkanException is thrown on any occurring Vulkan error.
     *
     * @warning The caller has to ensure that the current configuration is in
     * a valid state. This function performs no input validation.
     */
    VulkanCommandManager* submit();

    /**
     * Dispatches the current configuration into the command buffer
     *
     * @return Returns a pointer to this for method chaining
     *
     * @throw VulkanException is thrown on any occurring Vulkan error.
     *
     * @warning The caller has to ensure that the current configuration is in
     * a valid state. This function performs no input validation.
     */
    VulkanCommandManager* dispatch();

    /**
     * Sets the shader code to be used by the command buffer
     *
     * @param shaderModule A pair holding a constant pointer to a valid SPIR-V
     * module and its size in bytes
     *
     * @return Returns a pointer to this for method chaining
     *
     * @exceptsafe no-throw This method is guaranteed to not throw a runtime
     * exception.
     *
     * @note [ALICEVISION INTERNAL]: The shader module can easily be retrieved
     * using the ALICEVISION_GET_SPIRV_SHADER helper macro.
     */
    VulkanCommandManager* shader(const std::pair<const uint32_t*, size_t>& shaderModule);

    /**
     * Sets the shader code to be used by the command buffer
     *
     * @param shaderModule A vector holding valid shader code
     *
     * @return Returns a pointer to this for method chaining
     *
     * @exceptsafe no-throw This method is guaranteed to not throw a runtime
     * exception.
     *
     * @note [ALICEVISION INTERNAL]: The shader module can easily be retrieved
     * using the ALICEVISION_GET_SPIRV_SHADER helper macro.
     */
    VulkanCommandManager* shader(const std::vector<uint8_t>& shaderModule);

    /**
     * Sets the workgroup size to be used by the command buffer
     *
     * @param workgroups A vk::Extent3D specifying the workgroup size
     *
     * @exceptsafe no-throw This method is guaranteed to not throw a runtime
     * exception.
     *
     * @return Returns a pointer to this for method chaining
     */
    VulkanCommandManager* workgroups(const vk::Extent3D& workgroups);

    /**
     * Waits for the command buffer to finish executing
     *
     * @return Returns a pointer to this for method chaining
     *
     * @throw VulkanException is thrown on any occurring Vulkan error.
     *
     * @note If no buffer is in execution, this function immediately returns.
     * @note Waiting follows a FIFO pattern. All submitted command buffers will
     * be awaited in order of their submission.
     *
     * @warning If this method throws, it received either a timeout or the
     * underlying device was lost. The Fence will be removed internally and
     * cannot be awaited again.
     */
    VulkanCommandManager* wait();

private:
    explicit VulkanCommandManager(const uint32_t deviceID) : m_deviceID(deviceID) {}  // Internal Constructor
    ~VulkanCommandManager() = default;                                                // Internal Destructor

    static std::mutex m_initMutex;                                       // Mutex used for initialization
    static std::unordered_map<uint32_t, VulkanCommandManager*> m_these;  // Static Member holding a pointer to selves

    /**
     *
     *
     */
    struct IBOUNDMEM
    {
        virtual const bool isVkImage() const = 0;
        virtual const bool isVkBuffer() const = 0;
        virtual const vk::SharedImage getVkImage() const = 0;
        virtual const vk::SharedBuffer getVkBuffer() const = 0;
        virtual const vk::SharedSampler getSampler() const = 0;
        virtual const vk::SharedImageView getImageViewForAllMipMapLevels() const = 0;
        virtual const vk::SharedImageView getImageViewForMipLevel(uint32_t mipLevel) const = 0;
        virtual const vk::ImageLayout getCurrentImageLayout() const = 0;
        virtual const size_t getMemorySize() const = 0;
        virtual const unsigned int getBindingIndex() const = 0;
        virtual const vk::DescriptorType getDescriptorType() const = 0;
        virtual const int getMipLevel() const = 0;
        virtual ~IBOUNDMEM() = default;
    };
    /**
     * An internal struct for associating VulkanMemoryBase instances with
     * a size, a binding index, and a descriptor type
     */
    template <typename Type>
    struct BoundMemory : IBOUNDMEM
    {
        const VulkanMemoryBase<Type>& m_memory;     // A reference to the actual memory
        const size_t m_memorySize;                  // Size in bytes of the memory
        const unsigned int m_bindingIndex;          // The binding index for this memory
        const vk::DescriptorType m_descriptorType;  // The descriptor type for this memory
        const int m_mipLevel = -1;                  // The mip level for this memory, if it is a vk::SharedImage. Unused if -1.

        const bool isVkImage() const override { return m_memory.isVkImage(); }
        const bool isVkBuffer() const override { return m_memory.isVkBuffer(); }
        const vk::SharedImage getVkImage() const override { return m_memory.getVkImage().value(); }
        const vk::SharedBuffer getVkBuffer() const override { return m_memory.getVkBuffer().value(); }
        const vk::SharedSampler getSampler() const override { return m_memory.getSampler().value(); }
        const vk::SharedImageView getImageViewForAllMipMapLevels() const override { return m_memory.getImageViewForAllMipMapLevels().value(); }
        const vk::SharedImageView getImageViewForMipLevel(uint32_t mipLevel) const override { return m_memory.getImageViewForMipMapLevel(mipLevel).value(); }
        const vk::ImageLayout getCurrentImageLayout() const override { return m_memory.getCurrentImageLayout().value(); }
        const size_t getMemorySize() const override { return m_memory.getByteSize(); }
        const unsigned int getBindingIndex() const override { return m_bindingIndex; }
        const vk::DescriptorType getDescriptorType() const override { return m_descriptorType; }
        const int getMipLevel() const override { return m_mipLevel; }

        explicit BoundMemory(
            const VulkanMemoryBase<Type>& memory,
            size_t memorySize,
            unsigned int bindingIndex,
            vk::DescriptorType descriptorType,
            int mipLevel = -1)
        : m_memory(memory)
        , m_memorySize(memorySize)
        , m_bindingIndex(bindingIndex)
        , m_descriptorType(descriptorType)
        , m_mipLevel(mipLevel)
    {}
    };

    const uint32_t m_deviceID = UINT32_MAX;                                                      // The deviceID of this instance
    vk::Extent3D m_workgroupSize = vk::Extent3D(0u, 0u, 0u);                 // A member variable for storing the current workgroup size
    std::pair<const uint32_t*, size_t> m_ShaderModule = std::make_pair(nullptr, 0);              // A member variable for storing a reference to the current SPIR-V code
    std::vector<std::unique_ptr<IBOUNDMEM>> m_boundMemoryInstances = {};                         // A member variable for storing bound VulkanMemoryBase instances
    std::pair<const void*, size_t> m_pushConstantTemporaryStorage = std::make_pair(nullptr, 0);  // A member variable for storing Push Constant data of different types
};

#include <AVGPU/Vulkan/impl/pushconstant.inl>
#include <AVGPU/Vulkan/impl/commandmanager.inl>
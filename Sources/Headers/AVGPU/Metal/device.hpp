// This file is part of the AliceVision project.
// Copyright (c) 2025 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#import <Metal/Metal.hpp>

#include <map>
#include <mutex>
#include <span>

namespace aliceVision {
namespace gpu {

// Concept of Resources
template<class RT>
concept ResourceType = std::is_same_v<RT, NS::SharedPtr<MTL::Buffer>> || std::is_same_v<RT, NS::SharedPtr<MTL::Texture>>;

// Short Types
using MTLBuffer = NS::SharedPtr<MTL::Buffer>;
using MTLTexture = NS::SharedPtr<MTL::Texture>;

// Forward declare MTLCommandManager
class MTLCommandManager;

template<ResourceType Resource, typename Type>
class MTLSharedResource
{
    // Make MTLResourceManager a friend, so it can construct instances
    friend class MTLResourceManager;

    // Make MTLSharedResource a friend, so it can construct instances
    template<ResourceType ResourceInner, typename TypeInner>
    friend class MTLSharedResource;

  public:
    MTLSharedResource() = default;
    Resource getResource() const;

    void copyFromHost(const Type* data, uint64_t count, uint64_t srcOffset, uint64_t dstOffset);

    template<ResourceType InnerRT>
    void copyFromResource(const MTLSharedResource<InnerRT, Type>& resource, uint64_t srcOffset, uint64_t dstOffset);

    std::shared_ptr<std::span<Type>> getData() const;

    const MTL::Size getTextureDimensions() const requires std::is_same_v<Resource, MTLTexture>;
    const uint64_t getBytesPadded() const requires std::is_same_v<Resource, MTLTexture>;
    const uint64_t getBytesUnpadded() const requires std::is_same_v<Resource, MTLTexture>;
    const uint64_t getBytes() const requires std::is_same_v<Resource, MTLBuffer>;
    const uint64_t getResourceDeviceID() const;

  private:
    explicit MTLSharedResource(NS::SharedPtr<MTL::Device> device, uint64_t count, bool sharedWithCPU);
    explicit MTLSharedResource(NS::SharedPtr<MTL::Device> device,
                               MTL::TextureType texType,
                               MTL::Size dim,
                               MTL::PixelFormat pixFormat,
                               uint64_t mipmapLevels,
                               bool sharedWithCPU);

    void syncCPU(uint64_t byteStart, uint64_t byteEnd) const;

    bool m_isCPUShared;
    bool m_needsExplicitSync;
    NS::SharedPtr<MTL::Device> m_device;
    Resource m_resource;
};

class MTLResourceManager
{
    // Make MTLDeviceManager a friend, so it can construct instances
    friend class MTLDeviceManager;

  public:
    template<typename Type>
    MTLSharedResource<MTLBuffer, Type> createBuffer(uint64_t count, bool sharedWithCPU) const;
    template<typename Type>
    MTLSharedResource<MTLTexture, Type> createTexture1D(MTL::PixelFormat pixFormat, uint64_t width, uint64_t mipmapLevels, bool sharedWithCPU) const;
    template<typename Type>
    MTLSharedResource<MTLTexture, Type> createTexture2D(MTL::PixelFormat pixFormat,
                                                        uint64_t width,
                                                        uint64_t height,
                                                        uint64_t mipmapLevels,
                                                        bool sharedWithCPU) const;
    template<typename Type>
    MTLSharedResource<MTLTexture, Type> createTexture3D(MTL::PixelFormat pixFormat,
                                                        uint64_t width,
                                                        uint64_t height,
                                                        uint64_t depth,
                                                        uint64_t mipmapLevels,
                                                        bool sharedWithCPU) const;

  private:
    explicit MTLResourceManager(NS::SharedPtr<MTL::Device> device);

    NS::SharedPtr<MTL::Device> m_device;
};

class MTLDeviceManager
{
  public:
    static MTLDeviceManager* getInstance();

    const std::unordered_map<uint64_t, NS::SharedPtr<MTL::Device>>& getDevices() const;
    NS::SharedPtr<MTL::Device> getDevice(uint64_t deviceID) const;
    NS::SharedPtr<MTL::CommandQueue> getQueue(uint64_t deviceID) const;
    MTLResourceManager* getResourceManager(uint64_t deviceID) const;
    MTLCommandManager* getCommandManager(uint64_t deviceID) const;
    const uint64_t getPriorityDeviceID() const;
    void getDeviceMemoryInfo(uint64_t deviceID, double& availableMB, double& usedMB, double& totalMB) const;
    void logDeviceMemoryInfo(uint64_t deviceID) const;

  private:
    MTLDeviceManager();  // Internal Constructor

    static std::mutex m_initMutex;    // Mutex used for initialization
    static MTLDeviceManager* m_this;  // Static Member holding a pointer to self

    std::unordered_map<uint64_t, NS::SharedPtr<MTL::Device>> m_devices;       // A map holding deviceIDs and devices
    std::unordered_map<uint64_t, NS::SharedPtr<MTL::CommandQueue>> m_queues;  // A map holding deviceIDs and queues
    std::unordered_map<uint64_t, std::unique_ptr<MTLResourceManager>>
      m_resourceManagers;  // A map holding deviceIDs and the relevant Resource Manager instance
    std::unordered_map<uint64_t, std::unique_ptr<MTLCommandManager>>
      m_commandManagers;  // A map holding deviceIDs and the relevant Command Manager instance
};

}  // namespace gpu
}  // namespace aliceVision

#include <AVGPU/Metal/Impl/device.inl>

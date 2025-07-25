// This file is part of the AliceVision project.
// Copyright (c) 2025 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <iostream>
#include <sstream>

#include <AVSystem/Logger.hpp>
#include <AVGPU/Vulkan/exception.hpp>

template<typename T>
VulkanBuffer<T> VulkanMemoryBase<T>::createBuffer(
    uint32_t deviceID,
    uint32_t size,
    bool shaderWritable,
    bool hostVisible)
{
    return VulkanBuffer<T>(deviceID, size, shaderWritable, hostVisible);
}

template<typename T>
VulkanImage<T> VulkanMemoryBase<T>::create1DImage(
    uint32_t deviceID,
    uint32_t width,
    vk::Format imageFormat,
    uint32_t mipmapLevels,
    bool shaderWritable,
    bool hostVisible)
{
    return VulkanImage<T>(deviceID, width, imageFormat, mipmapLevels, shaderWritable, hostVisible);
}

template<typename T>
VulkanImage<T> VulkanMemoryBase<T>::create2DImage(
    uint32_t deviceID,
    uint32_t width,
    uint32_t height,
    vk::Format imageFormat,
    uint32_t mipmapLevels,
    bool shaderWritable,
    bool hostVisible)
{
    return VulkanImage<T>(deviceID, width, height, imageFormat, mipmapLevels, shaderWritable, hostVisible);
}

template<typename T>
VulkanImage<T> VulkanMemoryBase<T>::create3DImage(
    uint32_t deviceID,
    uint32_t width,
    uint32_t height,
    uint32_t depth,
    vk::Format imageFormat,
    uint32_t mipmapLevels,
    bool shaderWritable,
    bool hostVisible)
{
    return VulkanImage<T>(deviceID, width, height, depth, imageFormat, mipmapLevels, shaderWritable, hostVisible);
}

template<typename T>
std::shared_ptr<T[]> VulkanMemoryBase<T>::getData() const
{
    auto deviceID = this->INTERN_getDeviceID();

    // Create the data pointer
    T* data = static_cast<T*>(this->INTERN_getData());

    std::shared_ptr<VmaAllocation_T> allocationMaybe = nullptr;

    if(isVkImage()) {
        const VulkanMemory<vk::SharedImage, T>* derived = static_cast<const VulkanMemory<vk::SharedImage, T>*>(this);
        if(derived->m_stagingBuffer != nullptr)
            allocationMaybe = std::dynamic_pointer_cast<VulkanBuffer<T>>(derived->m_stagingBuffer)->m_allocation;
    } else {
        const VulkanMemory<vk::SharedBuffer, T>* derived = static_cast<const VulkanMemory<vk::SharedBuffer, T>*>(this);
        if(derived->m_stagingBuffer != nullptr)
            allocationMaybe = std::dynamic_pointer_cast<VulkanBuffer<T>>(derived->m_stagingBuffer)->m_allocation;
    }

    // We did not use a staging buffer
    if(allocationMaybe == nullptr) {
        allocationMaybe = static_cast<const VulkanMemory<vk::SharedImage, T>*>(this)->m_allocation;
    }

    return std::shared_ptr<T[]>(data, [deviceID, allocationMaybe](T* ptr) {
        if (allocationMaybe != nullptr) {
            vmaUnmapMemory(
                VulkanManager::getInstance()->m_allocators.at(deviceID).get(),
                allocationMaybe.get()
            );
        }
    });
}

template<BufferType BufferType, typename Type>
uint32_t VulkanMemory<BufferType, Type>::getDeviceID() const
{
    return this->m_deviceID;
}

template<BufferType BufferType, typename Type>
constexpr bool VulkanMemory<BufferType, Type>::isVkImage() const
{
    return std::is_same_v<BufferType, vk::SharedImage>;
}

template<BufferType BufferType, typename Type>
constexpr bool VulkanMemory<BufferType, Type>::isVkBuffer() const
{
    return std::is_same_v<BufferType, vk::SharedBuffer>;
}

template<BufferType BufferType, typename Type>
std::optional<vk::SharedImage> VulkanMemory<BufferType, Type>::getVkImage() const
{
    if constexpr (std::is_same_v<BufferType, vk::SharedImage>) {
        return std::optional<vk::SharedImage>(this->m_bufferObject);
    }
    return std::nullopt;
}

template<BufferType BufferType, typename Type>
std::optional<vk::SharedBuffer> VulkanMemory<BufferType, Type>::getVkBuffer() const
{
    if constexpr (std::is_same_v<BufferType, vk::SharedBuffer>) {
        return std::optional<vk::SharedBuffer>(this->m_bufferObject);
    }
    return std::nullopt;
}

template<BufferType BufferType, typename Type>
uint32_t VulkanMemory<BufferType, Type>::getByteSize() const
{
    return this->m_allocationByteSize;
}

template<BufferType BufferType, typename Type>
uint32_t VulkanMemory<BufferType, Type>::getSize() const
{
    return this->m_elementCount;
}

template<BufferType BufferType, typename Type>
std::optional<vk::Extent3D> VulkanMemory<BufferType, Type>::getDimensionSizes() const
{
    if constexpr (std::is_same_v<BufferType, vk::SharedImage>) {
        const auto& derived = static_cast<const VulkanImage<Type>&>(*this);
        return derived.m_extent;
    }
    return std::nullopt;
}

template<BufferType BufferType, typename Type>
std::optional<vk::ImageType> VulkanMemory<BufferType, Type>::getDimensions() const
{
    if constexpr (std::is_same_v<BufferType, vk::SharedImage>) {
        const auto& derived = static_cast<const VulkanImage<Type>&>(*this);
        return derived.m_type;
    }
    return std::nullopt;
}

template<BufferType BufferType, typename Type>
std::optional<vk::ImageLayout> VulkanMemory<BufferType, Type>::getCurrentImageLayout() const
{
    if constexpr (std::is_same_v<BufferType, vk::SharedImage>) {
        const auto derived = static_cast<const VulkanImage<Type>&>(*this);
        return std::optional<vk::ImageLayout>(derived.m_layout);
    }
    return std::nullopt;
}

template<BufferType BufferType, typename Type>
std::optional<vk::Format> VulkanMemory<BufferType, Type>::getImageFormat() const
{
    if constexpr (std::is_same_v<BufferType, vk::SharedImage>) {
        return this->m_format;
    }
    return std::nullopt;
}

template<BufferType BufferType, typename Type>
std::optional<uint32_t> VulkanMemory<BufferType, Type>::getMipmapLevelCount() const
{
    if constexpr (std::is_same_v<BufferType, vk::SharedImage>) {
        const auto derived = static_cast<const VulkanImage<Type>&>(*this);
        return std::optional<uint32_t>(derived.m_mipmapLevels);
    }
    return std::nullopt;
}

template<BufferType BufferType, typename Type>
std::optional<vk::SharedSampler> VulkanMemory<BufferType, Type>::getSampler() const
{
    if constexpr (std::is_same_v<BufferType, vk::SharedImage>) {
        return this->m_sampler;
    }
    return std::nullopt;
}

template<BufferType BufferType, typename Type>
std::optional<vk::SharedImageView> VulkanMemory<BufferType, Type>::getImageViewForMipMapLevel(uint32_t mipmapLevel) const
{
    if constexpr (std::is_same_v<BufferType, vk::SharedImage>) {
        if(this->m_imageViews.contains(mipmapLevel))
            return this->m_imageViews.at(mipmapLevel);
    }
    return std::nullopt;
}

template<BufferType BufferType, typename Type>
std::optional<vk::SharedImageView> VulkanMemory<BufferType, Type>::getImageViewForAllMipMapLevels() const
{
    if constexpr (std::is_same_v<BufferType, vk::SharedImage>) {
        return this->m_imageViewAllMipMaps;
    }
    return std::nullopt;
}

template<BufferType BufferType, typename Type>
void VulkanMemory<BufferType, Type>::copyFromAllocation(const VulkanMemoryBase<Type>& source, uint32_t sourceOffset, uint32_t destinationOffset) const
{
    if(this->isVkImage() && !source.isVkImage()) {
        throw std::runtime_error("Attempted to VulkanMemory::copyFromAllocation for copying a buffer into an image! Use VulkanMemory::copyImageFromBuffer instead!");
    }
    if constexpr (std::is_same_v<BufferType, vk::SharedBuffer>) {
        auto derived = static_cast<const VulkanBuffer<Type>*>(this);
        if(this->INTERN_getDeviceID() == source.INTERN_getDeviceID()) {
            derived->INTERN_copyFromSameDevice(source, sourceOffset, destinationOffset);
        } else {
            derived->INTERN_copyFromForeignDevice(source, sourceOffset, destinationOffset);
        }
    } else {
        auto derived = static_cast<const VulkanImage<Type>*>(this);
        if(this->INTERN_getDeviceID() == source.INTERN_getDeviceID()) {
            derived->INTERN_copyFromSameDevice(source, sourceOffset, destinationOffset);
        } else {
            derived->INTERN_copyFromForeignDevice(source, sourceOffset, destinationOffset);
        }
    }
}

template<BufferType BufferType, typename Type>
void VulkanMemory<BufferType, Type>::copyImageFromBuffer(const VulkanMemoryBase<Type>& source, uint32_t sourceOffset, uint32_t destinationOffset) const
{
    if constexpr (std::is_same_v<BufferType, vk::SharedImage>) {
        auto derived = static_cast<const VulkanImage<Type>*>(this);
        if(this->INTERN_getDeviceID() == source.INTERN_getDeviceID()) {
            derived->INTERN_copyImageFromSameDevice(source, sourceOffset, destinationOffset);
        } else {
            derived->INTERN_copyImageFromForeignDevice(source, sourceOffset, destinationOffset);
        }
    } else {
        throw std::runtime_error("Attempted to call VulkanMemory<BufferType, Type>::copyImageFromBuffer on a VulkanMemory instance which is specialized with vk::SharedBuffer. This function can only be called on VulkanMemory<vk::SharedImage, Type>!");
    }
}

template<BufferType BufferType, typename Type>
uint32_t VulkanMemory<BufferType, Type>::INTERN_getDeviceID() const
{
    return this->m_deviceID;
}

template<BufferType BufferType, typename Type>
VulkanMemory<BufferType, Type>::VulkanMemory(const uint32_t deviceID)
: m_deviceID(deviceID) {}

template<BufferType BufferType, typename Type>
void* VulkanMemory<BufferType, Type>::INTERN_getData() const
{
    // If this is host visible, we can directly use it and get a mapping
    // Images cansnot be mapped directly, so we always need a staging buffer
    void* mappedData;
    if(this->m_hostVisible && this->isVkBuffer()) {
        mappedData = ALICEVISION_THROW_ON_VULKAN_ERROR<void*>(vmaMapMemory, "Failed to map buffer data in the host!", VulkanManager::getInstance()->m_allocators.at(this->m_deviceID).get(), this->m_allocation.get());
        ALICEVISION_THROW_ON_VULKAN_ERROR<>(vmaInvalidateAllocation, "Failed to invalidate allocation!", VulkanManager::getInstance()->m_allocators.at(this->m_deviceID).get(), this->m_allocation.get(), 0, this->getByteSize());
    } else { // Else use a staging buffer
        if(this->isVkBuffer()) {
            this->m_stagingBuffer = std::make_shared<VulkanBuffer<Type>>(VulkanMemoryBase<Type>::createBuffer(this->m_deviceID, this->getSize(), false, true));
        } else {
            switch(this->getDimensions().value()) {
                case vk::ImageType::e1D: {
                    this->m_stagingBuffer = std::make_shared<VulkanBuffer<Type>>(VulkanMemoryBase<Type>::createBuffer(this->m_deviceID, this->getDimensionSizes().value().width, false, true));
                    break;
                }
                case vk::ImageType::e2D: {
                    this->m_stagingBuffer = std::make_shared<VulkanBuffer<Type>>(VulkanMemoryBase<Type>::createBuffer(this->m_deviceID, this->getDimensionSizes().value().width * this->getDimensionSizes().value().height, false, true));
                    break;
                }
                case vk::ImageType::e3D: {
                    this->m_stagingBuffer = std::make_shared<VulkanBuffer<Type>>(VulkanMemoryBase<Type>::createBuffer(this->m_deviceID, this->getDimensionSizes().value().width * this->getDimensionSizes().value().height * this->getDimensionSizes().value().depth, false, true));
                    break;
                }
                default: {
                    throw std::runtime_error("Unknown image type!");
                }
            }
        }
        // Copy into staging buffer
        this->m_stagingBuffer->copyFromAllocation(*this, 0, 0);
        mappedData = ALICEVISION_THROW_ON_VULKAN_ERROR<void*>(vmaMapMemory, "Failed to map buffer data in the host!", VulkanManager::getInstance()->m_allocators.at(this->m_deviceID).get(), this->m_stagingBuffer->m_allocation.get());
        ALICEVISION_THROW_ON_VULKAN_ERROR<>(vmaInvalidateAllocation, "Failed to invalidate allocation!", VulkanManager::getInstance()->m_allocators.at(this->m_deviceID).get(), this->m_stagingBuffer->m_allocation.get(), 0, this->m_stagingBuffer->getByteSize());
    }
    return mappedData;
}

template<typename Type>
VulkanBuffer<Type>::VulkanBuffer(
    const uint32_t deviceID,
    const uint32_t size,
    const bool shaderWritable,
    const bool hostVisible)
: VulkanMemory<vk::SharedBuffer, Type>(deviceID)
{
    // Calculate the sizing requirements
    const uint32_t sizingRequirementBytes = sizeof(Type) * size;
    // Create the buffer info
    vk::BufferCreateInfo bufferCreateInfo({}, sizingRequirementBytes, {
        vk::BufferUsageFlagBits::eTransferDst
        | vk::BufferUsageFlagBits::eTransferSrc
        | vk::BufferUsageFlagBits::eUniformBuffer
    }, vk::SharingMode::eExclusive);
    if(shaderWritable)
        bufferCreateInfo.usage |= vk::BufferUsageFlagBits::eStorageBuffer;
    // Create the allocation info
    VmaAllocationCreateInfo allocationCreateInfo = {};
    if(hostVisible)
        allocationCreateInfo.usage = VMA_MEMORY_USAGE_AUTO;
    allocationCreateInfo.priority = 1.0f;
    if(hostVisible)
        allocationCreateInfo.flags = VMA_ALLOCATION_CREATE_HOST_ACCESS_SEQUENTIAL_WRITE_BIT;
    if(hostVisible)
        allocationCreateInfo.requiredFlags = static_cast<VkMemoryPropertyFlagBits>(vk::MemoryPropertyFlagBits::eHostVisible);
    auto [buffer, allocation, allocationInfo] = ALICEVISION_THROW_ON_VULKAN_ERROR<VkBuffer, VmaAllocation, VmaAllocationInfo>(vmaCreateBuffer, "Failed to create Vulkan buffer!", VulkanManager::getInstance()->m_allocators.at(this->m_deviceID).get(), &*bufferCreateInfo, &allocationCreateInfo);
    // TODO: Figure out if it safe to destroy it like this, or if we actually need to get the Buffer ourselves
    // Assign the member variables
    // We need to explicitly copy m_deviceID, because otherwise it won't be usable in the destructor of m_allocation.
    this->m_allocation = std::shared_ptr<VmaAllocation_T>(allocation, [&, deviceID = this->m_deviceID](VmaAllocation alloc){ vmaFreeMemory(VulkanManager::getInstance()->m_allocators.at(deviceID).get(), alloc); });
    this->m_bufferObject = vk::SharedBuffer(vk::Buffer(buffer), VulkanManager::getInstance()->m_devs.at(this->m_deviceID));
    this->m_allocationByteSize = allocationInfo.size;
    this->m_elementCount = size;
    this->m_hostVisible = hostVisible;
    this->m_shaderWritable = shaderWritable;
}

template<typename Type>
VulkanImage<Type>::VulkanImage(
    uint32_t deviceID,
    uint32_t width,
    vk::Format imageFormat,
    uint32_t mipmapLevels,
    bool shaderWritable,
    bool hostVisible)
: VulkanMemory<vk::SharedImage, Type>(deviceID)
{
    // Create the extent
    this->m_extent = vk::Extent3D(width, 1, 1);
    // The image format
    this->m_format = imageFormat;
    // The image type
    this->m_type = vk::ImageType::e1D;
    // Create the image info
    vk::ImageCreateInfo imageCreateInfo({}, this->m_type, this->m_format, this->m_extent, mipmapLevels, 1, vk::SampleCountFlagBits::e1, vk::ImageTiling::eOptimal, {
        vk::ImageUsageFlagBits::eTransferDst
        | vk::ImageUsageFlagBits::eTransferSrc
        | vk::ImageUsageFlagBits::eSampled
    }, vk::SharingMode::eExclusive, {}, vk::ImageLayout::eUndefined);
    if(shaderWritable)
        imageCreateInfo.usage |= vk::ImageUsageFlagBits::eStorage;
    // Check if we can actually allocate the requested amount of mipmaps
    // TODO: Investigate
    // const vk::ImageFormatProperties formatProperties = ALICEVISION_THROW_ON_VULKAN_HPP_ERROR(VulkanManager::getInstance()->m_physDevs.at(this->m_deviceID)->getImageFormatProperties(imageCreateInfo.format, imageCreateInfo.imageType, imageCreateInfo.tiling, imageCreateInfo.usage, imageCreateInfo.flags), "Failed to get format properties!");
    // if(mipmapLevels > formatProperties.maxMipLevels) {
    //     std::stringstream s;
    //     s << "Cannot create image with " << mipmapLevels << " mipmap levels! Device only allows " << formatProperties.maxMipLevels << " mipmap levels in this image configuration.";
    //     throw std::runtime_error(s.str());
    // }
    // Create the allocation info
    VmaAllocationCreateInfo allocationCreateInfo = {};
    if(hostVisible)
        allocationCreateInfo.usage = VMA_MEMORY_USAGE_AUTO;
    allocationCreateInfo.priority = 1.0f;
    if(hostVisible)
        allocationCreateInfo.flags = VMA_ALLOCATION_CREATE_HOST_ACCESS_SEQUENTIAL_WRITE_BIT;
    if(hostVisible)
        allocationCreateInfo.requiredFlags = static_cast<VkMemoryPropertyFlagBits>(vk::MemoryPropertyFlagBits::eHostVisible);
    auto [image, allocation, allocationInfo] = ALICEVISION_THROW_ON_VULKAN_ERROR<VkImage, VmaAllocation, VmaAllocationInfo>(vmaCreateImage, "Failed to create Vulkan image!", VulkanManager::getInstance()->m_allocators.at(this->m_deviceID).get(), &*imageCreateInfo, &allocationCreateInfo);
    this->m_allocation = std::shared_ptr<VmaAllocation_T>(allocation, [&, deviceID = this->m_deviceID](VmaAllocation alloc){ vmaFreeMemory(VulkanManager::getInstance()->m_allocators.at(deviceID).get(), alloc); });
    this->m_bufferObject = vk::SharedImage(vk::Image(image), VulkanManager::getInstance()->m_devs.at(this->m_deviceID));
    this->m_allocationByteSize = allocationInfo.size;
    this->m_elementCount = this->m_extent.width * this->m_extent.height * this->m_extent.depth;
    this->m_hostVisible = hostVisible;
    this->m_shaderWritable = shaderWritable;
    this->m_layout = imageCreateInfo.initialLayout;
    this->m_mipmapLevels = mipmapLevels;
    // Create the vk::SharedImageViews for the mipmap levels
    for(uint32_t i = 0; i < mipmapLevels; i++) {
        const vk::ImageViewCreateInfo imageViewInfo({}, *this->m_bufferObject, vk::ImageViewType::e1D, this->m_format, { vk::ComponentSwizzle::eIdentity, vk::ComponentSwizzle::eIdentity, vk::ComponentSwizzle::eIdentity, vk::ComponentSwizzle::eIdentity }, { vk::ImageAspectFlagBits::eColor, i, 1, 0, 1 });
        this->m_imageViews.insert(std::make_pair(i, vk::SharedImageView(ALICEVISION_THROW_ON_VULKAN_HPP_ERROR(VulkanManager::getInstance()->m_devs.at(this->m_deviceID)->createImageView(imageViewInfo), "Failed to create image view!"), VulkanManager::getInstance()->m_devs.at(this->m_deviceID))));
    }
    // Create the image view for all mipmap levels in one
    const vk::ImageViewCreateInfo imageViewInfo({}, *this->m_bufferObject, vk::ImageViewType::e1D, this->m_format, { vk::ComponentSwizzle::eIdentity, vk::ComponentSwizzle::eIdentity, vk::ComponentSwizzle::eIdentity, vk::ComponentSwizzle::eIdentity }, { vk::ImageAspectFlagBits::eColor, 0, this->m_mipmapLevels, 0, 1 });
    this->m_imageViewAllMipMaps = vk::SharedImageView(ALICEVISION_THROW_ON_VULKAN_HPP_ERROR(VulkanManager::getInstance()->m_devs.at(this->m_deviceID)->createImageView(imageViewInfo), "Failed to create image view!"), VulkanManager::getInstance()->m_devs.at(this->m_deviceID));
    // Create the sampler
    constexpr vk::SamplerCreateInfo samplerCreateInfo({}, vk::Filter::eLinear, vk::Filter::eLinear, vk::SamplerMipmapMode::eLinear, vk::SamplerAddressMode::eClampToEdge, vk::SamplerAddressMode::eClampToEdge, vk::SamplerAddressMode::eClampToEdge, 0.f, vk::False, 0.f, vk::False, vk::CompareOp::eNever, 0.f, vk::LodClampNone, vk::BorderColor::eIntOpaqueBlack, vk::False);
    this->m_sampler = vk::SharedSampler(ALICEVISION_THROW_ON_VULKAN_HPP_ERROR(VulkanManager::getInstance()->m_devs.at(this->m_deviceID)->createSampler(samplerCreateInfo), "Failed to create sampler!"), VulkanManager::getInstance()->m_devs.at(this->m_deviceID));
}

template<typename Type>
VulkanImage<Type>::VulkanImage(
    uint32_t deviceID,
    uint32_t width,
    uint32_t height,
    vk::Format imageFormat,
    uint32_t mipmapLevels,
    bool shaderWritable,
    bool hostVisible)
: VulkanMemory<vk::SharedImage, Type>(deviceID)
{
    // Create the extent
    this->m_extent = vk::Extent3D(width, height, 1);
    // The image format
    this->m_format = imageFormat;
    // The image type
    this->m_type = vk::ImageType::e2D;
    // Create the image info
    vk::ImageCreateInfo imageCreateInfo({}, this->m_type, this->m_format, this->m_extent, mipmapLevels, 1, vk::SampleCountFlagBits::e1, vk::ImageTiling::eOptimal, {
        vk::ImageUsageFlagBits::eTransferDst
        | vk::ImageUsageFlagBits::eTransferSrc
        | vk::ImageUsageFlagBits::eSampled
    }, vk::SharingMode::eExclusive, {}, vk::ImageLayout::eUndefined);
    if(shaderWritable)
        imageCreateInfo.usage |= vk::ImageUsageFlagBits::eStorage;
    // Check if we can actually allocate the requested amount of mipmaps
    // TODO: Investigate
    // const vk::ImageFormatProperties formatProperties = ALICEVISION_THROW_ON_VULKAN_HPP_ERROR(VulkanManager::getInstance()->m_physDevs.at(this->m_deviceID)->getImageFormatProperties(imageCreateInfo.format, imageCreateInfo.imageType, imageCreateInfo.tiling, imageCreateInfo.usage, imageCreateInfo.flags), "Failed to get format properties!");
    // if(mipmapLevels > formatProperties.maxMipLevels) {
    //     std::stringstream s;
    //     s << "Cannot create image with " << mipmapLevels << " mipmap levels! Device only allows " << formatProperties.maxMipLevels << " mipmap levels in this image configuration.";
    //     throw std::runtime_error(s.str());
    // }
    // Create the allocation info
    VmaAllocationCreateInfo allocationCreateInfo = {};
    if(hostVisible)
        allocationCreateInfo.usage = VMA_MEMORY_USAGE_AUTO;
    allocationCreateInfo.priority = 1.0f;
    if(hostVisible)
        allocationCreateInfo.flags = VMA_ALLOCATION_CREATE_HOST_ACCESS_SEQUENTIAL_WRITE_BIT;
    if(hostVisible)
        allocationCreateInfo.requiredFlags = static_cast<VkMemoryPropertyFlagBits>(vk::MemoryPropertyFlagBits::eHostVisible);
    auto [image, allocation, allocationInfo] = ALICEVISION_THROW_ON_VULKAN_ERROR<VkImage, VmaAllocation, VmaAllocationInfo>(vmaCreateImage, "Failed to create Vulkan image!", VulkanManager::getInstance()->m_allocators.at(this->m_deviceID).get(), &*imageCreateInfo, &allocationCreateInfo);
    this->m_allocation = std::shared_ptr<VmaAllocation_T>(allocation, [&, deviceID = this->m_deviceID](VmaAllocation alloc){ vmaFreeMemory(VulkanManager::getInstance()->m_allocators.at(deviceID).get(), alloc); });
    this->m_bufferObject = vk::SharedImage(vk::Image(image), VulkanManager::getInstance()->m_devs.at(this->m_deviceID));
    this->m_allocationByteSize = allocationInfo.size;
    this->m_elementCount = this->m_extent.width * this->m_extent.height * this->m_extent.depth;
    this->m_hostVisible = hostVisible;
    this->m_shaderWritable = shaderWritable;
    this->m_layout = imageCreateInfo.initialLayout;
    this->m_mipmapLevels = mipmapLevels;
    // Create the vk::SharedImageViews for the mipmap levels
    for(uint32_t i = 0; i < mipmapLevels; i++) {
        const vk::ImageViewCreateInfo imageViewInfo({}, *this->m_bufferObject, vk::ImageViewType::e2D, this->m_format, { vk::ComponentSwizzle::eIdentity, vk::ComponentSwizzle::eIdentity, vk::ComponentSwizzle::eIdentity, vk::ComponentSwizzle::eIdentity }, { vk::ImageAspectFlagBits::eColor, i, 1, 0, 1 });
        this->m_imageViews.insert(std::make_pair(i, vk::SharedImageView(ALICEVISION_THROW_ON_VULKAN_HPP_ERROR(VulkanManager::getInstance()->m_devs.at(this->m_deviceID)->createImageView(imageViewInfo), "Failed to create image view!"), VulkanManager::getInstance()->m_devs.at(this->m_deviceID))));
    }
    // Create the image view for all mipmap levels in one
    const vk::ImageViewCreateInfo imageViewInfo({}, *this->m_bufferObject, vk::ImageViewType::e2D, this->m_format, { vk::ComponentSwizzle::eIdentity, vk::ComponentSwizzle::eIdentity, vk::ComponentSwizzle::eIdentity, vk::ComponentSwizzle::eIdentity }, { vk::ImageAspectFlagBits::eColor, 0, this->m_mipmapLevels, 0, 1 });
    this->m_imageViewAllMipMaps = vk::SharedImageView(ALICEVISION_THROW_ON_VULKAN_HPP_ERROR(VulkanManager::getInstance()->m_devs.at(this->m_deviceID)->createImageView(imageViewInfo), "Failed to create image view!"), VulkanManager::getInstance()->m_devs.at(this->m_deviceID));
    // Create the sampler
    constexpr vk::SamplerCreateInfo samplerCreateInfo({}, vk::Filter::eLinear, vk::Filter::eLinear, vk::SamplerMipmapMode::eLinear, vk::SamplerAddressMode::eClampToEdge, vk::SamplerAddressMode::eClampToEdge, vk::SamplerAddressMode::eClampToEdge, 0.f, vk::False, 0.f, vk::False, vk::CompareOp::eNever, 0.f, vk::LodClampNone, vk::BorderColor::eIntOpaqueBlack, vk::False);
    this->m_sampler = vk::SharedSampler(ALICEVISION_THROW_ON_VULKAN_HPP_ERROR(VulkanManager::getInstance()->m_devs.at(this->m_deviceID)->createSampler(samplerCreateInfo), "Failed to create sampler!"), VulkanManager::getInstance()->m_devs.at(this->m_deviceID));
}

template<typename Type>
VulkanImage<Type>::VulkanImage(
    uint32_t deviceID,
    uint32_t width,
    uint32_t height,
    uint32_t depth,
    vk::Format imageFormat,
    uint32_t mipmapLevels,
    bool shaderWritable,
    bool hostVisible)
: VulkanMemory<vk::SharedImage, Type>(deviceID)
{
    // Create the extent
    this->m_extent = vk::Extent3D(width, height, depth);
    // The image format
    this->m_format = imageFormat;
    // The image type
    this->m_type = vk::ImageType::e3D;
    // Create the image info
    vk::ImageCreateInfo imageCreateInfo({}, this->m_type, this->m_format, this->m_extent, mipmapLevels, 1, vk::SampleCountFlagBits::e1, vk::ImageTiling::eOptimal, {
        vk::ImageUsageFlagBits::eTransferDst
        | vk::ImageUsageFlagBits::eTransferSrc
        | vk::ImageUsageFlagBits::eSampled
    }, vk::SharingMode::eExclusive, {}, vk::ImageLayout::eUndefined);
    if(shaderWritable)
        imageCreateInfo.usage |= vk::ImageUsageFlagBits::eStorage;
    // Check if we can actually allocate the requested amount of mipmaps
    // TODO: Investigate
    // const vk::ImageFormatProperties formatProperties = ALICEVISION_THROW_ON_VULKAN_HPP_ERROR(VulkanManager::getInstance()->m_physDevs.at(this->m_deviceID)->getImageFormatProperties(imageCreateInfo.format, imageCreateInfo.imageType, imageCreateInfo.tiling, imageCreateInfo.usage, imageCreateInfo.flags), "Failed to get format properties!");
    // if(mipmapLevels > formatProperties.maxMipLevels) {
    //     std::stringstream s;
    //     s << "Cannot create image with " << mipmapLevels << " mipmap levels! Device only allows " << formatProperties.maxMipLevels << " mipmap levels in this image configuration.";
    //     throw std::runtime_error(s.str());
    // }
    // Create the allocation info
    VmaAllocationCreateInfo allocationCreateInfo = {};
    if(hostVisible)
        allocationCreateInfo.usage = VMA_MEMORY_USAGE_AUTO;
    allocationCreateInfo.priority = 1.0f;
    if(hostVisible)
        allocationCreateInfo.flags = VMA_ALLOCATION_CREATE_HOST_ACCESS_SEQUENTIAL_WRITE_BIT;
    if(hostVisible)
        allocationCreateInfo.requiredFlags = static_cast<VkMemoryPropertyFlagBits>(vk::MemoryPropertyFlagBits::eHostVisible);
    auto [image, allocation, allocationInfo] = ALICEVISION_THROW_ON_VULKAN_ERROR<VkImage, VmaAllocation, VmaAllocationInfo>(vmaCreateImage, "Failed to create Vulkan image!", VulkanManager::getInstance()->m_allocators.at(this->m_deviceID).get(), &*imageCreateInfo, &allocationCreateInfo);
    this->m_allocation = std::shared_ptr<VmaAllocation_T>(allocation, [&, deviceID = this->m_deviceID](VmaAllocation alloc){ vmaFreeMemory(VulkanManager::getInstance()->m_allocators.at(deviceID).get(), alloc); });
    this->m_bufferObject = vk::SharedImage(vk::Image(image), VulkanManager::getInstance()->m_devs.at(this->m_deviceID));
    this->m_allocationByteSize = allocationInfo.size;
    this->m_elementCount = this->m_extent.width * this->m_extent.height * this->m_extent.depth;
    this->m_hostVisible = hostVisible;
    this->m_shaderWritable = shaderWritable;
    this->m_layout = imageCreateInfo.initialLayout;
    this->m_mipmapLevels = mipmapLevels;
    // Create the vk::SharedImageViews for the mipmap levels
    for(uint32_t i = 0; i < mipmapLevels; i++) {
        const vk::ImageViewCreateInfo imageViewInfo({}, *this->m_bufferObject, vk::ImageViewType::e3D, this->m_format, { vk::ComponentSwizzle::eIdentity, vk::ComponentSwizzle::eIdentity, vk::ComponentSwizzle::eIdentity, vk::ComponentSwizzle::eIdentity }, { vk::ImageAspectFlagBits::eColor, i, 1, 0, 1 });
        this->m_imageViews.insert(std::make_pair(i, vk::SharedImageView(ALICEVISION_THROW_ON_VULKAN_HPP_ERROR(VulkanManager::getInstance()->m_devs.at(this->m_deviceID)->createImageView(imageViewInfo), "Failed to create image view!"), VulkanManager::getInstance()->m_devs.at(this->m_deviceID))));
    }
    // Create the image view for all mipmap levels in one
    const vk::ImageViewCreateInfo imageViewInfo({}, *this->m_bufferObject, vk::ImageViewType::e3D, this->m_format, { vk::ComponentSwizzle::eIdentity, vk::ComponentSwizzle::eIdentity, vk::ComponentSwizzle::eIdentity, vk::ComponentSwizzle::eIdentity }, { vk::ImageAspectFlagBits::eColor, 0, this->m_mipmapLevels, 0, 1 });
    this->m_imageViewAllMipMaps = vk::SharedImageView(ALICEVISION_THROW_ON_VULKAN_HPP_ERROR(VulkanManager::getInstance()->m_devs.at(this->m_deviceID)->createImageView(imageViewInfo), "Failed to create image view!"), VulkanManager::getInstance()->m_devs.at(this->m_deviceID));
    // Create the sampler
    constexpr vk::SamplerCreateInfo samplerCreateInfo({}, vk::Filter::eLinear, vk::Filter::eLinear, vk::SamplerMipmapMode::eLinear, vk::SamplerAddressMode::eClampToEdge, vk::SamplerAddressMode::eClampToEdge, vk::SamplerAddressMode::eClampToEdge, 0.f, vk::False, 0.f, vk::False, vk::CompareOp::eNever, 0.f, vk::LodClampNone, vk::BorderColor::eIntOpaqueBlack, vk::False);
    this->m_sampler = vk::SharedSampler(ALICEVISION_THROW_ON_VULKAN_HPP_ERROR(VulkanManager::getInstance()->m_devs.at(this->m_deviceID)->createSampler(samplerCreateInfo), "Failed to create sampler!"), VulkanManager::getInstance()->m_devs.at(this->m_deviceID));
}

template<typename Type>
void VulkanBuffer<Type>::INTERN_copyFromSameDevice(const VulkanMemoryBase<Type>& source, uint32_t sourceOffset, uint32_t destinationOffset) const
{
    // Use a fast in-device memory copy
    // TODO: Check sizing requirements
    if(source.isVkBuffer())
        VulkanCommandManager::getInstance(this->m_deviceID)
            ->wait()
            ->reset()
            ->begin()
            ->standardOp([&](vk::SharedCommandBuffer cmdBuffer){ cmdBuffer->copyBuffer(*source.getVkBuffer().value(), *this->getVkBuffer().value(), { vk::BufferCopy(sourceOffset, destinationOffset, source.getByteSize() - sourceOffset) }); })
            ->end()
            ->submit()
            ->wait();
    if(source.isVkImage())
        VulkanCommandManager::getInstance(this->m_deviceID)
            ->wait()
            ->reset()
            ->begin()
            ->standardOp([&](vk::SharedCommandBuffer cmdBuffer) {
                constexpr vk::ImageSubresourceRange imageSubresourceRange( vk::ImageAspectFlagBits::eColor, 0, 1, 0, 1 );
                const vk::ImageMemoryBarrier imageMemoryBarrier({vk::AccessFlagBits::eNone}, {vk::AccessFlagBits::eTransferRead}, source.getCurrentImageLayout().value(), vk::ImageLayout::eTransferSrcOptimal, {}, {}, *source.getVkImage().value(), imageSubresourceRange);
                cmdBuffer->pipelineBarrier({vk::PipelineStageFlagBits::eAllCommands}, {vk::PipelineStageFlagBits::eAllCommands}, {}, {}, {}, { imageMemoryBarrier });
                const auto& derived = static_cast<const VulkanImage<Type>&>(source);
                derived.m_layout = vk::ImageLayout::eTransferSrcOptimal;
            })
            // TODO: Add ability to specify image offsets
            ->standardOp([&](vk::SharedCommandBuffer cmdBuffer){ cmdBuffer->copyImageToBuffer(*source.getVkImage().value(), source.getCurrentImageLayout().value(), *this->getVkBuffer().value(), { vk::BufferImageCopy(destinationOffset, 0, 0, vk::ImageSubresourceLayers({vk::ImageAspectFlagBits::eColor}, 0, 0, 1), vk::Offset3D(0, 0, 0), source.getDimensionSizes().value()) }); })
            ->end()
            ->submit()
            ->wait();
}

template<typename Type>
void VulkanBuffer<Type>::INTERN_copyFromForeignDevice(const VulkanMemoryBase<Type>& source, uint32_t sourceOffset, uint32_t destinationOffset) const
{
    // Create two staging buffers and copy
    VulkanBuffer stagingBufferSrc = VulkanBuffer<Type>(source.INTERN_getDeviceID(), source.getSize(), false, true);
    stagingBufferSrc.INTERN_copyFromSameDevice(source, 0, 0);
    VulkanBuffer stagingBufferDst = VulkanBuffer<Type>(this->INTERN_getDeviceID(), source.getSize(), false, true);
    // Get a host allocated array
    std::shared_ptr<Type[]> hostArray = stagingBufferSrc.getData();
    // Copy into the destination staging buffer
    stagingBufferDst.copyFromHostToBuffer(hostArray.get(), stagingBufferSrc.getSize(), 0, 0);
    // Copy from destination staging buffer into this
    this->INTERN_copyFromSameDevice(stagingBufferDst, sourceOffset, destinationOffset);
}

template<typename Type>
void VulkanBuffer<Type>::copyFromHostToBuffer(const Type* data, uint32_t count, uint32_t sourceOffset, uint32_t destinationOffset) const
{
    // If this is host visible, copy directly
    if(this->m_hostVisible) {
        void* mappedData = ALICEVISION_THROW_ON_VULKAN_ERROR<void*>(vmaMapMemory, "Failed to map memory!", VulkanManager::getInstance()->m_allocators.at(this->m_deviceID).get(), this->m_allocation.get());
        memcpy(static_cast<Type*>(mappedData) + destinationOffset / sizeof(Type), data + sourceOffset / sizeof(Type), count * sizeof(Type));
        ALICEVISION_THROW_ON_VULKAN_ERROR<>(vmaFlushAllocation, "Failed to flush allocation!", VulkanManager::getInstance()->m_allocators.at(this->m_deviceID).get(), this->m_allocation.get(), 0, this->getByteSize());
        vmaUnmapMemory(VulkanManager::getInstance()->m_allocators.at(this->m_deviceID).get(), this->m_allocation.get());
    } else { // Staging Buffer is required
        VulkanBuffer stagingBuffer = VulkanBuffer<Type>(this->m_deviceID, count, false, true);
        stagingBuffer.copyFromHostToBuffer(data, count, sourceOffset, destinationOffset);
        this->INTERN_copyFromSameDevice(stagingBuffer, 0, 0);
    }
}

template<typename Type>
void VulkanBuffer<Type>::copyFromHostToImage(const Type* data, uint32_t count, uint32_t sourceOffset, uint32_t destinationOffset) const
{
    throw std::runtime_error("This method cannot be called on a VulkanMemory instance which is specialized with vk::SharedBuffer!");
}

template<typename Type>
void VulkanImage<Type>::copyFromHostToBuffer(const Type* data, uint32_t count, uint32_t sourceOffset, uint32_t destinationOffset) const
{
    ALICEVISION_THROW_ERROR("VulkanMemory::copyFromHostToBuffer cannot be called on a VulkanImage!");
    // Need to create a staging buffer, copy into it, and then transfer to image
    // VulkanBuffer<Type> stagingBuffer = VulkanBuffer<Type>(this->m_deviceID, count, false, true);
    // stagingBuffer.copyFromHostToBuffer(data, count, sourceOffset, destinationOffset);
    // this->copyImageFromBuffer(stagingBuffer, 0, 0);
}

template<typename Type>
void VulkanImage<Type>::copyFromHostToImage(const Type* data, uint32_t count, uint32_t sourceOffset, uint32_t destinationOffset) const
{
    // Need to copy into staging buffer
    VulkanBuffer<Type> stagingBuffer = VulkanMemoryBase<Type>::createBuffer(this->m_deviceID, this->getSize(), false, true);
    stagingBuffer.copyFromHostToBuffer(data, count, 0, 0);
    this->INTERN_copyImageFromSameDevice(stagingBuffer, sourceOffset, destinationOffset);
}

template<typename Type>
void VulkanImage<Type>::INTERN_copyFromSameDevice(const VulkanMemoryBase<Type>& source, uint32_t sourceOffset, uint32_t destinationOffset) const
{
    // Use a fast in-device memory copy
    // TODO: Check sizing requirements
    if(!source.isVkImage())
        throw std::runtime_error("INTERN_copyFromSameDevice on a VulkanImage<T> is only valid if the source is a VulkanImage<T> as well!");
    VulkanCommandManager::getInstance(this->m_deviceID)
        ->wait()
        ->reset()
        ->begin()
        ->standardOp([&](vk::SharedCommandBuffer cmdBuffer) {
            const vk::ImageSubresourceRange imageSubresourceRange( vk::ImageAspectFlagBits::eColor, 0, source.getMipmapLevelCount().value(), 0, 1 );
            const vk::ImageMemoryBarrier imageMemoryBarrier({vk::AccessFlagBits::eNone}, {vk::AccessFlagBits::eTransferRead}, vk::ImageLayout::eUndefined, vk::ImageLayout::eTransferSrcOptimal, {}, {}, *source.getVkImage().value(), imageSubresourceRange);
            cmdBuffer->pipelineBarrier({vk::PipelineStageFlagBits::eAllCommands}, {vk::PipelineStageFlagBits::eAllCommands}, {}, {}, {}, { imageMemoryBarrier });
            const auto& derived = static_cast<const VulkanImage&>(source);
            derived.m_layout = vk::ImageLayout::eTransferSrcOptimal;
        })
        ->standardOp([&](vk::SharedCommandBuffer cmdBuffer) {
            const vk::ImageSubresourceRange imageSubresourceRange( vk::ImageAspectFlagBits::eColor, 0, this->m_mipmapLevels, 0, 1 );
            const vk::ImageMemoryBarrier imageMemoryBarrier({vk::AccessFlagBits::eTransferWrite}, {vk::AccessFlagBits::eTransferWrite}, vk::ImageLayout::eUndefined, vk::ImageLayout::eTransferDstOptimal, {}, {}, *this->getVkImage().value(), imageSubresourceRange);
            cmdBuffer->pipelineBarrier({vk::PipelineStageFlagBits::eAllCommands}, {vk::PipelineStageFlagBits::eAllCommands}, {}, {}, {}, { imageMemoryBarrier });
            const auto derived = static_cast<const VulkanImage*>(this);
            derived->m_layout = vk::ImageLayout::eTransferDstOptimal;
        })
        // TODO: Add ability to specify image offsets
        ->standardOp([&](vk::SharedCommandBuffer cmdBuffer) {
            // Create a range for each source mip level
            std::vector<vk::ImageCopy> regions;
            for(uint32_t i=0; i < source.getMipmapLevelCount().value(); i++) {
                const vk::Extent3D scaledExtent = {
                    std::max(1u, source.getDimensionSizes().value().width >> i),
                    std::max(1u, source.getDimensionSizes().value().height >> i),
                    std::max(1u, source.getDimensionSizes().value().depth >> i),
                };
                regions.emplace_back(vk::ImageSubresourceLayers({vk::ImageAspectFlagBits::eColor}, i, 0, 1), 0, vk::ImageSubresourceLayers({vk::ImageAspectFlagBits::eColor}, i, 0, 1), 0, scaledExtent);
            }
            cmdBuffer->copyImage(*source.getVkImage().value(), source.getCurrentImageLayout().value(), *this->getVkImage().value(), this->getCurrentImageLayout().value(), regions);
        })
        ->end()
        ->submit()
        ->wait();
}

template<typename Type>
void VulkanImage<Type>::INTERN_copyFromForeignDevice(const VulkanMemoryBase<Type>& source, uint32_t sourceOffset, uint32_t destinationOffset) const
{
    // Need to copy into staging buffer first
    auto sourceData = source.getData();
    VulkanBuffer<Type> stagingBuffer = VulkanMemoryBase<Type>::createBuffer(this->m_deviceID, source.getByteSize(), false, true);
    stagingBuffer.copyFromHostToBuffer(sourceData.get(), source.getSize(), 0, 0);
    // Now copy into this
    this->INTERN_copyImageFromSameDevice(stagingBuffer, sourceOffset, destinationOffset);
}

template<typename Type>
void VulkanImage<Type>::INTERN_copyImageFromSameDevice(const VulkanMemoryBase<Type>& source, uint32_t sourceOffset, uint32_t destinationOffset) const
{
    if(source.isVkImage())
        this->INTERN_copyFromSameDevice(source, sourceOffset, destinationOffset);
    else
        VulkanCommandManager::getInstance(this->m_deviceID)
            ->wait()
            ->reset()
            ->begin()
            ->standardOp([&](vk::SharedCommandBuffer cmdBuffer) {
                constexpr vk::ImageSubresourceRange imageSubresourceRange( vk::ImageAspectFlagBits::eColor, 0, 1, 0, 1 );
                const vk::ImageMemoryBarrier imageMemoryBarrier({vk::AccessFlagBits::eNone}, {vk::AccessFlagBits::eTransferWrite}, this->getCurrentImageLayout().value(), vk::ImageLayout::eTransferDstOptimal, {}, {}, *this->getVkImage().value(), imageSubresourceRange);
                cmdBuffer->pipelineBarrier({vk::PipelineStageFlagBits::eAllCommands}, {vk::PipelineStageFlagBits::eAllCommands}, {}, {}, {}, { imageMemoryBarrier });
                this->m_layout = vk::ImageLayout::eTransferDstOptimal;
            })
            ->standardOp([&](vk::SharedCommandBuffer cmdBuffer) {
                cmdBuffer->copyBufferToImage(*source.getVkBuffer().value(), *this->m_bufferObject, this->m_layout, { vk::BufferImageCopy(0, 0, 0, vk::ImageSubresourceLayers(vk::ImageAspectFlagBits::eColor, 0, 0, 1), vk::Offset3D(0, 0, 0), this->m_extent) });
            })
            ->end()
            ->submit()
            ->wait();
}

template<typename Type>
void VulkanImage<Type>::INTERN_copyImageFromForeignDevice(const VulkanMemoryBase<Type>& source, uint32_t sourceOffset, uint32_t destinationOffset) const
{
    // Create a staging buffer
    auto sourceData = source.getData();
    VulkanBuffer<Type> stagingBuffer = VulkanMemoryBase<Type>::createBuffer(this->m_deviceID, source.getSize(), false, true);
    stagingBuffer.copyFromHostToBuffer(sourceData.get(), source.getSize(), 0, 0);
    this->INTERN_copyImageFromSameDevice(stagingBuffer, sourceOffset, destinationOffset);
}

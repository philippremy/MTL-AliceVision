//
// Created by Philipp Remy on 16.07.25.
//

#pragma once

#include <CoreFoundation/CoreFoundation.h>

#include <AVSystem/Logger.hpp>

namespace aliceVision {
namespace gpu {

template<typename Type>
MTLSharedResource<MTLBuffer, Type> MTLResourceManager::createBuffer(const uint64_t count, const bool sharedWithCPU) const
{
    return MTLSharedResource<MTLBuffer, Type>(this->m_device, count, sharedWithCPU);
}

template<typename Type>
MTLSharedResource<MTLTexture, Type> MTLResourceManager::createTexture1D(const MTL::PixelFormat pixFormat,
                                                                        const uint64_t width,
                                                                        const uint64_t mipmapLevels,
                                                                        const bool sharedWithCPU) const
{
    return MTLSharedResource<MTLTexture, Type>(this->m_device, MTL::TextureType1D, MTL::Size(width, 1, 1), pixFormat, mipmapLevels, sharedWithCPU);
}

template<typename Type>
MTLSharedResource<MTLTexture, Type> MTLResourceManager::createTexture2D(const MTL::PixelFormat pixFormat,
                                                                        const uint64_t width,
                                                                        const uint64_t height,
                                                                        const uint64_t mipmapLevels,
                                                                        const bool sharedWithCPU) const
{
    return MTLSharedResource<MTLTexture, Type>(
      this->m_device, MTL::TextureType2D, MTL::Size(width, height, 1), pixFormat, mipmapLevels, sharedWithCPU);
}

template<typename Type>
MTLSharedResource<MTLTexture, Type> MTLResourceManager::createTexture3D(const MTL::PixelFormat pixFormat,
                                                                        const uint64_t width,
                                                                        const uint64_t height,
                                                                        const uint64_t depth,
                                                                        const uint64_t mipmapLevels,
                                                                        const bool sharedWithCPU) const
{
    return MTLSharedResource<MTLTexture, Type>(
      this->m_device, MTL::TextureType3D, MTL::Size(width, height, depth), pixFormat, mipmapLevels, sharedWithCPU);
}

template<ResourceType Resource, typename Type>
MTLSharedResource<Resource, Type>::MTLSharedResource(NS::SharedPtr<MTL::Device> device, const uint64_t count, const bool sharedWithCPU)
  : m_isCPUShared(sharedWithCPU),
    m_device(device)
{
    // Create the buffer options
    this->m_needsExplicitSync = false;
    MTL::ResourceOptions options;
    if (sharedWithCPU)
    {
        NS::SharedPtr<NS::String> name = NS::TransferPtr(device->name());
        const CFStringRef apple = CFStringCreateWithCString(kCFAllocatorDefault, "Apple", kCFStringEncodingUTF8);
        const CFRange strRange = CFStringFind(reinterpret_cast<CFStringRef>(name.get()), apple, 0);
        if (strRange.length == 0 && strRange.location == kCFNotFound)
        {
            options = MTL::ResourceStorageModeManaged;
        }
        else
        {
            options = MTL::ResourceStorageModeShared;
            this->m_needsExplicitSync = true;
        }
        CFRelease(apple);
    }
    else
    {
        options = MTL::ResourceStorageModePrivate;
    }
    // Create the buffer
    this->m_resource = NS::TransferPtr(this->m_device->newBuffer(sizeof(Type) * count, options));
}

template<ResourceType Resource, typename Type>
MTLSharedResource<Resource, Type>::MTLSharedResource(NS::SharedPtr<MTL::Device> device,
                                                     const MTL::TextureType texType,
                                                     const MTL::Size dim,
                                                     const MTL::PixelFormat pixFormat,
                                                     const uint64_t mipmapLevels,
                                                     const bool sharedWithCPU)
  : m_isCPUShared(sharedWithCPU),
    m_device(device)
{
    // Create the buffer options
    this->m_needsExplicitSync = false;
    MTL::ResourceOptions options;
    if (sharedWithCPU)
    {
        NS::SharedPtr<NS::String> name = NS::TransferPtr(device->name());
        const CFStringRef apple = CFStringCreateWithCString(kCFAllocatorDefault, "Apple", kCFStringEncodingUTF8);
        const CFRange strRange = CFStringFind(reinterpret_cast<CFStringRef>(name.get()), apple, 0);
        printf("");
        if (strRange.length == 0 && strRange.location == kCFNotFound)
        {
            options = MTL::ResourceStorageModeManaged;
        }
        else
        {
            options = MTL::ResourceStorageModeShared;
            this->m_needsExplicitSync = true;
        }
        CFRelease(apple);
    }
    else
    {
        options = MTL::ResourceStorageModePrivate;
    }

    // Create the texture descriptor
    NS::SharedPtr<MTL::TextureDescriptor> texDescriptor = NS::TransferPtr(MTL::TextureDescriptor::alloc()->init());
    texDescriptor->setTextureType(texType);
    texDescriptor->setPixelFormat(pixFormat);
    texDescriptor->setWidth(dim.width);
    texDescriptor->setHeight(dim.height);
    texDescriptor->setDepth(dim.depth);
    texDescriptor->setMipmapLevelCount(mipmapLevels);
    texDescriptor->setSampleCount(1);
    texDescriptor->setResourceOptions(options);
    texDescriptor->setAllowGPUOptimizedContents(true);
    texDescriptor->setUsage(MTL::TextureUsageShaderRead | MTL::TextureUsageShaderWrite);

    // Create the texture
    this->m_resource = NS::TransferPtr(this->m_device->newTexture(texDescriptor.get()));
}

template<ResourceType Resource, typename Type>
void MTLSharedResource<Resource, Type>::syncCPU(const uint64_t byteStart, const uint64_t byteEnd) const
{
    this->m_resource->didModifyRange(NS::Range(byteStart, byteEnd));
}

template<ResourceType Resource, typename Type>
Resource MTLSharedResource<Resource, Type>::getResource() const
{
    return this->m_resource;
}

template<ResourceType Resource, typename Type>
void MTLSharedResource<Resource, Type>::copyFromHost(const Type* data, uint64_t count, uint64_t srcOffset, uint64_t dstOffset)
{
    if constexpr (std::is_same_v<Resource, MTLBuffer>)  // Buffer
    {
        if (this->m_isCPUShared && !this->m_needsExplicitSync)  // CPU shared, can be mapped and copied and does not need any sync
        {
            Type* contents = static_cast<Type*>(static_cast<MTLBuffer>(this->m_resource)->contents());
            memcpy(contents + dstOffset, data + srcOffset, count * sizeof(Type));
        }
        else if (this->m_isCPUShared && this->m_needsExplicitSync)  // CPU shared, needs explicit sync
        {
            Type* contents = static_cast<Type*>(static_cast<MTLBuffer>(this->m_resource)->contents());
            memcpy(contents + dstOffset, data + srcOffset, count * sizeof(Type));
            this->syncCPU(dstOffset, dstOffset + sizeof(Type) * count);
        }
        else  // Needs staging buffer
        {
            // Create staging buffer that matches the size of the current buffer
            NS::SharedPtr<MTL::Buffer> stagingBuffer =
              NS::TransferPtr(this->m_device->newBuffer(static_cast<const void*>(data + srcOffset), sizeof(Type) * count, MTL::ResourceStorageModeShared));
            // Create a command buffer for copying
            // TODO: Is this retained? Not clear. Likely.
            NS::SharedPtr<MTL::CommandBuffer> cmdBuffer =
              NS::TransferPtr(MTLDeviceManager::getInstance()->getQueue(this->m_device->registryID())->commandBuffer());
            NS::SharedPtr<MTL::BlitCommandEncoder> blitEncoder = NS::TransferPtr(cmdBuffer->blitCommandEncoder());
            blitEncoder->copyFromBuffer(stagingBuffer.get(), 0, this->m_resource.get(), dstOffset, stagingBuffer->length());
            blitEncoder->endEncoding();
            cmdBuffer->commit();
            cmdBuffer->waitUntilCompleted();
        }
    }
    else  // Texture
    {
        if (this->m_isCPUShared && !this->m_needsExplicitSync)  // CPU shared, can be mapped and copied and does not need any sync
        {
            MTL::Region region;
            MTL::Size texSize = MTL::Size(static_cast<MTLTexture>(this->m_resource)->width(),
                                          static_cast<MTLTexture>(this->m_resource)->height(),
                                          static_cast<MTLTexture>(this->m_resource)->depth());
            switch (static_cast<MTLTexture>(this->m_resource)->textureType())
            {
                case MTL::TextureType1D:
                {
                    region = MTL::Region::Make1D(dstOffset, count);
                    break;
                }
                case MTL::TextureType2D:
                {
                    uint64_t x = dstOffset % texSize.width;
                    uint64_t y = dstOffset / texSize.width;

                    // Number of rows we'll need to fill
                    uint64_t lastOffset = dstOffset + count - 1;
                    uint64_t lastY = lastOffset / texSize.width;

                    uint64_t height = lastY - y + 1;
                    uint64_t width = texSize.width;  // default full row width

                    // If it's only one row, use `count` directly
                    if (height == 1)
                    {
                        width = count;
                    }

                    region = MTL::Region::Make2D(x, y, width, height);
                    break;
                }
                case MTL::TextureType3D:
                {
                    // Compute (x, y, z) from flat dstOffset
                    uint64_t xyArea = texSize.width * texSize.height;
                    uint64_t z = dstOffset / xyArea;
                    uint64_t y = (dstOffset % xyArea) / texSize.width;
                    uint64_t x = dstOffset % texSize.width;

                    // Compute total 3D volume covered
                    uint64_t lastOffset = dstOffset + count - 1;
                    uint64_t lastZ = lastOffset / xyArea;

                    uint64_t depth = lastZ - z + 1;
                    uint64_t height = texSize.height;
                    uint64_t width = texSize.width;

                    // If it fits in a single slice and row, narrow the copy size
                    if (depth == 1)
                    {
                        height = ((dstOffset + count) / texSize.width) - y + 1;
                        if (height == 1)
                        {
                            width = count;
                        }
                    }

                    region = MTL::Region::Make3D(x, y, z, width, height, depth);

                    break;
                }
                default:
                    ALICEVISION_THROW_ERROR("Unsupported texture type");
            }
            // TODO: Figure out if bytesPerRow should be 0.
            const uint64_t bytesPerImage = static_cast<MTLTexture>(this->m_resource)->textureType() == MTL::TextureType3D ? (texSize.width * texSize.height) * sizeof(Type) : 0;
            static_cast<MTLTexture>(this->m_resource)->replaceRegion(region, 0, 0, static_cast<const void*>(data), this->m_resource->width() * sizeof(Type), bytesPerImage);
        }
        else if (this->m_isCPUShared && this->m_needsExplicitSync)  // CPU shared, needs explicit sync
        {
            MTL::Region region;
            MTL::Size texSize = MTL::Size(static_cast<MTLTexture>(this->m_resource)->width(),
                                          static_cast<MTLTexture>(this->m_resource)->height(),
                                          static_cast<MTLTexture>(this->m_resource)->depth());
            switch (static_cast<MTLTexture>(this->m_resource)->textureType())
            {
                case MTL::TextureType1D:
                {
                    region = MTL::Region::Make1D(dstOffset, count);
                    break;
                }
                case MTL::TextureType2D:
                {
                    uint64_t x = dstOffset % texSize.width;
                    uint64_t y = dstOffset / texSize.width;

                    // Number of rows we'll need to fill
                    uint64_t lastOffset = dstOffset + count;
                    uint64_t lastY = lastOffset / texSize.width;

                    uint64_t height = lastY - y;
                    uint64_t width = texSize.width;  // default full row width

                    // If it's only one row, use `count` directly
                    if (height == 1)
                    {
                        width = count;
                    }

                    region = MTL::Region::Make2D(x, y, width, height);
                    break;
                }
                case MTL::TextureType3D:
                {
                    // Compute (x, y, z) from flat dstOffset
                    uint64_t xyArea = texSize.width * texSize.height;
                    uint64_t z = dstOffset / xyArea;
                    uint64_t y = (dstOffset % xyArea) / texSize.width;
                    uint64_t x = dstOffset % texSize.width;

                    // Compute total 3D volume covered
                    uint64_t lastOffset = dstOffset + count;
                    uint64_t lastZ = lastOffset / xyArea;

                    uint64_t depth = lastZ - z;
                    uint64_t height = texSize.height;
                    uint64_t width = texSize.width;

                    // If it fits in a single slice and row, narrow the copy size
                    if (depth == 1)
                    {
                        height = ((dstOffset + count) / texSize.width) - y + 1;
                        if (height == 1)
                        {
                            width = count;
                        }
                    }

                    region = MTL::Region::Make3D(x, y, z, width, height, depth);

                    break;
                }
                default:
                    ALICEVISION_THROW_ERROR("Unsupported texture type");
            }
            // TODO: Figure out if bytesPerRow should be 0.
            // NOTE: replaceRegion automatically synchronizes to the GPU
            static_cast<MTLTexture>(this->m_resource)->replaceRegion(region, 0, static_cast<const void*>(data), this->m_resource->width() * sizeof(Type));
        }
        else  // Needs staging buffer
        {
            // Create a staging texture
            MTLSharedResource stagingTex;
            const MTLTexture destTex = static_cast<MTLTexture>(this->m_resource);
            switch (destTex->textureType())
            {
                case MTL::TextureType1D:
                {
                    stagingTex = MTLDeviceManager::getInstance()
                                   ->getResourceManager(this->m_device->registryID())
                                   ->template createTexture1D<Type>(destTex->pixelFormat(), destTex->width(), 1, true);
                    break;
                }
                case MTL::TextureType2D:
                {
                    stagingTex = MTLDeviceManager::getInstance()
                                   ->getResourceManager(this->m_device->registryID())
                                   ->template createTexture2D<Type>(destTex->pixelFormat(), destTex->width(), destTex->height(), 1, true);
                    break;
                }
                case MTL::TextureType3D:
                {
                    stagingTex = MTLDeviceManager::getInstance()
                                   ->getResourceManager(this->m_device->registryID())
                                   ->template createTexture3D<Type>(destTex->pixelFormat(), destTex->width(), destTex->height(), destTex->depth(), 1, true);
                    break;
                }
                default:
                    ALICEVISION_THROW_ERROR("Unsupported texture type");
            }
            stagingTex.copyFromHost(data, count, srcOffset, dstOffset);
            // Create a command buffer for copying
            // TODO: Is this retained? Not clear.
            NS::SharedPtr<MTL::CommandBuffer> cmdBuffer =
              NS::TransferPtr(MTLDeviceManager::getInstance()->getQueue(this->m_device->registryID())->commandBuffer());
            NS::SharedPtr<MTL::BlitCommandEncoder> blitEncoder = NS::TransferPtr(cmdBuffer->blitCommandEncoder());
            blitEncoder->copyFromTexture(stagingTex.getResource().get(), static_cast<MTLTexture>(this->m_resource).get());
            blitEncoder->endEncoding();
            cmdBuffer->commit();
            cmdBuffer->waitUntilCompleted();
        }
    }
}

template<ResourceType Resource, typename Type>
template<ResourceType InnerRT>
void MTLSharedResource<Resource, Type>::copyFromResource(const MTLSharedResource<InnerRT, Type>& resource, uint64_t srcOffset, uint64_t dstOffset)
{
    // Copying into buffer
    if constexpr (std::is_same_v<Resource, MTLBuffer>)
    {
        // We are copying from another buffer
        if constexpr (std::is_same_v<InnerRT, MTLBuffer>)
        {
            // Same device, copy directly
            if (this->m_device->registryID() == resource.m_device->registryID())
            {
                NS::SharedPtr<MTL::CommandBuffer> cmdBuffer =
                  NS::TransferPtr(MTLDeviceManager::getInstance()->getQueue(this->m_device->registryID())->commandBuffer());
                NS::SharedPtr<MTL::BlitCommandEncoder> blitEncoder = NS::TransferPtr(cmdBuffer->blitCommandEncoder());
                blitEncoder->copyFromBuffer(resource.getResource().get(),
                                            srcOffset * sizeof(Type),
                                            this->m_resource.get(),
                                            dstOffset * sizeof(Type),
                                            resource.m_resource->length() - (dstOffset * sizeof(Type)));
                blitEncoder->endEncoding();
                cmdBuffer->commit();
                cmdBuffer->waitUntilCompleted();
                return;
            }
            else if (this->m_device->peerGroupID() == resource.m_device->peerGroupID())  // Same peer group, copy with remote view
            {
                const MTLBuffer remoteBuffer =
                  NS::TransferPtr(static_cast<MTLBuffer>(resource.getResource())->newRemoteBufferViewForDevice(this->m_device.get()));
                NS::SharedPtr<MTL::CommandBuffer> cmdBuffer =
                  NS::TransferPtr(MTLDeviceManager::getInstance()->getQueue(this->m_device->registryID())->commandBuffer());
                NS::SharedPtr<MTL::BlitCommandEncoder> blitEncoder = NS::TransferPtr(cmdBuffer->blitCommandEncoder());
                blitEncoder->copyFromBuffer(remoteBuffer.get(),
                                            srcOffset * sizeof(Type),
                                            this->m_resource.get(),
                                            dstOffset * sizeof(Type),
                                            resource.m_resource->length() - (dstOffset * sizeof(Type)));
                blitEncoder->endEncoding();
                cmdBuffer->commit();
                cmdBuffer->waitUntilCompleted();
                return;
            }
            else  // Device is completely different, so we need to copy via host
            {
                std::shared_ptr<std::span<Type>> hostData = resource.getData();
                const uint64_t count = resource.getResource()->length() / sizeof(Type);
                this->copyFromHost(hostData->data(), count, srcOffset, dstOffset);
                return;
            }
        }
        else  // We are copying from a texture
        {
            // Same device, copy directly
            if (this->m_device->registryID() == resource.m_device->registryID())
            {
                NS::SharedPtr<MTL::CommandBuffer> cmdBuffer =
                  NS::TransferPtr(MTLDeviceManager::getInstance()->getQueue(this->m_device->registryID())->commandBuffer());
                NS::SharedPtr<MTL::BlitCommandEncoder> blitEncoder = NS::TransferPtr(cmdBuffer->blitCommandEncoder());
                // Make the origin
                MTL::Origin srcOrigin;
                switch (resource.getResource()->textureType())
                {
                    case MTL::TextureType1D:
                    {
                        srcOrigin = MTL::Origin(srcOffset, 0, 0);
                        break;
                    }
                    case MTL::TextureType2D:
                    {
                        srcOrigin = MTL::Origin(srcOffset % resource.getResource()->width(), srcOffset / resource.getResource()->width(), 0);
                        break;
                    }
                    case MTL::TextureType3D:
                    {
                        uint32_t width = resource.getResource()->width();
                        uint32_t height = resource.getResource()->height();

                        uint32_t sliceSize = width * height;
                        uint32_t z = srcOffset / sliceSize;
                        uint32_t rem2D = srcOffset % sliceSize;
                        uint32_t y = rem2D / width;
                        uint32_t x = rem2D % width;

                        srcOrigin = MTL::Origin(x, y, z);
                        break;
                    }
                    default:
                        ALICEVISION_THROW_ERROR("Unsupported texture type");
                }
                // Make the size
                MTL::Size srcSize;
                switch (resource.getResource()->textureType())
                {
                    case MTL::TextureType1D:
                    {
                        srcSize = MTL::Size(resource.getResource()->width() - srcOffset, 1, 1);
                        break;
                    }
                    case MTL::TextureType2D:
                    {
                        uint32_t width = resource.getResource()->width();
                        uint32_t height = resource.getResource()->height();

                        // Compute (x, y) origin
                        uint32_t x = srcOffset % width;
                        uint32_t y = srcOffset / width;

                        srcOrigin = MTL::Origin(x, y, 0);

                        // Calculate how many texels to copy
                        uint32_t remaining = width * height;

                        // First row span
                        uint32_t firstRowCount = std::min(width - x, remaining);
                        remaining -= firstRowCount;

                        uint32_t fullRows = remaining / width;
                        uint32_t tailCount = remaining % width;

                        // Height span
                        uint32_t heightSpan = 1 + fullRows + (tailCount > 0 ? 1 : 0);

                        // Width span: full row if more than one row, otherwise just the tail
                        uint32_t widthSpan = (heightSpan == 1) ? firstRowCount : width;

                        srcSize = MTL::Size(widthSpan, heightSpan, 1);
                        break;
                    }
                    case MTL::TextureType3D:
                    {
                        uint32_t width = resource.getResource()->width();
                        uint32_t height = resource.getResource()->height();
                        uint32_t depth = resource.getResource()->depth();

                        uint64_t sliceSize = width * height;

                        uint64_t z = srcOffset / sliceSize;
                        uint64_t rem2D = srcOffset % sliceSize;
                        uint64_t y = rem2D / width;
                        uint64_t x = rem2D % width;

                        srcOrigin = MTL::Origin(x, y, z);

                        // Compute size
                        uint64_t remaining = width * height * depth;

                        // First row of first slice
                        uint64_t firstRowCount = std::min(width - x, remaining);
                        remaining -= firstRowCount;

                        // Compute how many full 2D slices (width × height) we cover
                        uint32_t fullSlices = remaining / sliceSize;
                        remaining %= sliceSize;

                        // Compute how many full rows in the last partial slice
                        uint32_t fullRows = remaining / width;
                        uint32_t tailCount = remaining % width;

                        // Now calculate actual spans
                        uint32_t depthSpan = 1 + fullSlices + ((fullRows > 0 || tailCount > 0) ? 1 : 0);
                        uint32_t heightSpan = (depthSpan == 1) ? (1 + fullRows + (tailCount > 0 ? 1 : 0)) : height;
                        uint32_t widthSpan = (depthSpan == 1 && heightSpan == 1) ? firstRowCount : width;

                        srcSize = MTL::Size(widthSpan, heightSpan, depthSpan);
                        break;
                    }
                    default:
                        ALICEVISION_THROW_ERROR("Unsupported texture type");
                }
                blitEncoder->copyFromTexture(
                  resource.getResource().get(),
                  0,
                  0,
                  srcOrigin,
                  srcSize,
                  this->m_resource.get(),
                  dstOffset * sizeof(Type),
                  resource.getResource()->width() * sizeof(Type),
                  resource.getResource()->textureType() == MTL::TextureType3D
                    ? (resource.getResource()->width() * resource.getResource()->height() * resource.getResource()->depth()) * sizeof(Type)
                    : 0);
                blitEncoder->endEncoding();
                cmdBuffer->commit();
                cmdBuffer->waitUntilCompleted();
                return;
            }
            else if (this->m_device->peerGroupID() == resource.m_device->peerGroupID())  // Same peer group, copy with remote view
            {
                const MTLTexture remoteTexture =
                  NS::TransferPtr(static_cast<MTLTexture>(resource.getResource())->newRemoteTextureViewForDevice(this->m_device.get()));
                NS::SharedPtr<MTL::CommandBuffer> cmdBuffer =
                  NS::TransferPtr(MTLDeviceManager::getInstance()->getQueue(this->m_device->registryID())->commandBuffer());
                NS::SharedPtr<MTL::BlitCommandEncoder> blitEncoder = NS::TransferPtr(cmdBuffer->blitCommandEncoder());
                // Make the origin
                MTL::Origin srcOrigin;
                switch (remoteTexture->textureType())
                {
                    case MTL::TextureType1D:
                    {
                        srcOrigin = MTL::Origin(srcOffset, 0, 0);
                        break;
                    }
                    case MTL::TextureType2D:
                    {
                        srcOrigin = MTL::Origin(srcOffset % remoteTexture->width(), srcOffset / remoteTexture->width(), 0);
                        break;
                    }
                    case MTL::TextureType3D:
                    {
                        uint32_t width = remoteTexture->width();
                        uint32_t height = remoteTexture->height();

                        uint32_t sliceSize = width * height;
                        uint32_t z = srcOffset / sliceSize;
                        uint32_t rem2D = srcOffset % sliceSize;
                        uint32_t y = rem2D / width;
                        uint32_t x = rem2D % width;

                        srcOrigin = MTL::Origin(x, y, z);
                        break;
                    }
                    default:
                        ALICEVISION_THROW_ERROR("Unsupported texture type");
                }
                // Make the size
                MTL::Size srcSize;
                switch (remoteTexture->textureType())
                {
                    case MTL::TextureType1D:
                    {
                        srcSize = MTL::Size(remoteTexture->width() - srcOffset, 1, 1);
                        break;
                    }
                    case MTL::TextureType2D:
                    {
                        uint32_t width = remoteTexture->width();
                        uint32_t height = remoteTexture->height();

                        // Compute (x, y) origin
                        uint32_t x = srcOffset % width;
                        uint32_t y = srcOffset / width;

                        srcOrigin = MTL::Origin(x, y, 0);

                        // Calculate how many texels to copy
                        uint32_t remaining = width * height;

                        // First row span
                        uint32_t firstRowCount = std::min(width - x, remaining);
                        remaining -= firstRowCount;

                        uint32_t fullRows = remaining / width;
                        uint32_t tailCount = remaining % width;

                        // Height span
                        uint32_t heightSpan = 1 + fullRows + (tailCount > 0 ? 1 : 0);

                        // Width span: full row if more than one row, otherwise just the tail
                        uint32_t widthSpan = (heightSpan == 1) ? firstRowCount : width;

                        srcSize = MTL::Size(widthSpan, heightSpan, 1);
                        break;
                    }
                    case MTL::TextureType3D:
                    {
                        uint32_t width = remoteTexture->width();
                        uint32_t height = remoteTexture->height();
                        uint32_t depth = remoteTexture->depth();

                        uint64_t sliceSize = width * height;

                        uint64_t z = srcOffset / sliceSize;
                        uint64_t rem2D = srcOffset % sliceSize;
                        uint64_t y = rem2D / width;
                        uint64_t x = rem2D % width;

                        srcOrigin = MTL::Origin(x, y, z);

                        // Compute size
                        uint64_t remaining = width * height * depth;

                        // First row of first slice
                        uint64_t firstRowCount = std::min(width - x, remaining);
                        remaining -= firstRowCount;

                        // Compute how many full 2D slices (width × height) we cover
                        uint32_t fullSlices = remaining / sliceSize;
                        remaining %= sliceSize;

                        // Compute how many full rows in the last partial slice
                        uint32_t fullRows = remaining / width;
                        uint32_t tailCount = remaining % width;

                        // Now calculate actual spans
                        uint32_t depthSpan = 1 + fullSlices + ((fullRows > 0 || tailCount > 0) ? 1 : 0);
                        uint32_t heightSpan = (depthSpan == 1) ? (1 + fullRows + (tailCount > 0 ? 1 : 0)) : height;
                        uint32_t widthSpan = (depthSpan == 1 && heightSpan == 1) ? firstRowCount : width;

                        srcSize = MTL::Size(widthSpan, heightSpan, depthSpan);
                        break;
                    }
                    default:
                        ALICEVISION_THROW_ERROR("Unsupported texture type");
                }
                blitEncoder->copyFromTexture(remoteTexture.get(),
                                             0,
                                             0,
                                             srcOrigin,
                                             srcSize,
                                             this->m_resource.get(),
                                             dstOffset * sizeof(Type),
                                             0,
                                             remoteTexture->textureType() == MTL::TextureType3D
                                               ? (remoteTexture->width() * remoteTexture->height() * remoteTexture->depth())
                                               : 0);
                blitEncoder->endEncoding();
                cmdBuffer->commit();
                cmdBuffer->waitUntilCompleted();
            }
            else  // Completely foreign device, must copy over from host
            {
                std::shared_ptr<std::span<Type>> hostData = resource.getData();
                const uint64_t count = resource.getResource()->width() * resource.getResource()->height() * resource.getResource()->depth();
                this->copyFromHost(hostData->data(), count, srcOffset, dstOffset);
                return;
            }
        }
    }
    else  // Copying to Texture
    {
        if constexpr (std::is_same_v<InnerRT, MTLBuffer>)  // Copying from Buffer
        {
            if (this->m_device->registryID() == resource.m_device->registryID())  // Copy from same device, easy
            {
                NS::SharedPtr<MTL::CommandBuffer> cmdBuffer =
                  NS::TransferPtr(MTLDeviceManager::getInstance()->getQueue(this->m_device->registryID())->commandBuffer());
                NS::SharedPtr<MTL::BlitCommandEncoder> blitEncoder = NS::TransferPtr(cmdBuffer->blitCommandEncoder());
                // Create destination origin
                MTL::Origin dstOrigin;
                switch (this->getResource()->textureType())
                {
                    case MTL::TextureType1D:
                    {
                        dstOrigin = MTL::Origin(dstOffset, 0, 0);
                        break;
                    }
                    case MTL::TextureType2D:
                    {
                        dstOrigin = MTL::Origin(dstOffset % this->getResource()->width(), dstOffset / this->getResource()->width(), 0);
                        break;
                    }
                    case MTL::TextureType3D:
                    {
                        uint32_t width = this->getResource()->width();
                        uint32_t height = this->getResource()->height();

                        uint32_t sliceSize = width * height;
                        uint32_t z = dstOffset / sliceSize;
                        uint32_t rem2D = dstOffset % sliceSize;
                        uint32_t y = rem2D / width;
                        uint32_t x = rem2D % width;

                        dstOrigin = MTL::Origin(x, y, z);
                        break;
                    }
                    default:
                        ALICEVISION_THROW_ERROR("Unsupported texture type");
                }
                blitEncoder->copyFromBuffer(resource.getResource().get(),
                                            srcOffset * sizeof(Type),
                                            resource.getResource().get()->length() / this->m_resource->height(),
                                            this->m_resource->width() * this->m_resource->height() * sizeof(Type),
                                            MTL::Size(this->m_resource->width(), this->m_resource->height(), this->m_resource->depth()),
                                            this->m_resource.get(),
                                            0,
                                            0,
                                            dstOrigin);
                blitEncoder->endEncoding();
                cmdBuffer->commit();
                cmdBuffer->waitUntilCompleted();
                return;
            }
            else if (this->m_device->peerGroupID() == resource.m_device->peerGroupID())  // Copy from same peer group, use remote view
            {
                const MTLBuffer remoteBuffer =
                  NS::TransferPtr(static_cast<MTLBuffer>(resource.getResource())->newRemoteBufferViewForDevice(this->m_device.get()));
                NS::SharedPtr<MTL::CommandBuffer> cmdBuffer =
                  NS::TransferPtr(MTLDeviceManager::getInstance()->getQueue(this->m_device->registryID())->commandBuffer());
                NS::SharedPtr<MTL::BlitCommandEncoder> blitEncoder = NS::TransferPtr(cmdBuffer->blitCommandEncoder());
                // Create size
                MTL::Size srcSize;
                switch (this->getResource()->textureType())
                {
                    case MTL::TextureType1D:
                    {
                        srcSize = MTL::Size(remoteBuffer->length() / sizeof(Type) - srcOffset, 1, 1);
                        break;
                    }
                    case MTL::TextureType2D:
                    {
                        int width = (int)std::ceil(std::sqrt(remoteBuffer->length() / sizeof(Type) - srcOffset));
                        int height = (remoteBuffer->length() / sizeof(Type) - srcOffset + width - 1) / width;
                        srcSize = MTL::Size(width, height, 1);
                        break;
                    }
                    case MTL::TextureType3D:
                    {
                        int cubeRoot = (int)std::ceil(std::cbrt(remoteBuffer->length() / sizeof(Type) - srcOffset));
                        int bestW = 1, bestH = 1, bestD = remoteBuffer->length() / sizeof(Type) - srcOffset;

                        int minVolume = std::numeric_limits<int>::max();

                        for (int w = 1; w <= cubeRoot; ++w) {
                            for (int h = w; h <= cubeRoot; ++h) { // h >= w to avoid permutations
                                int d = ((remoteBuffer->length() / sizeof(Type) - srcOffset) + w * h - 1) / (w * h);
                                int volume = w * h * d;
                                if (volume < minVolume) {
                                    minVolume = volume;
                                    bestW = w;
                                    bestH = h;
                                    bestD = d;
                                }
                            }
                        }
                        srcSize = MTL::Size(bestW, bestH, bestD);
                        break;
                    }
                    default:
                        ALICEVISION_THROW_ERROR("Unsupported texture type");
                }
                // Create destination origin
                MTL::Origin dstOrigin;
                switch (this->getResource()->textureType())
                {
                    case MTL::TextureType1D:
                    {
                        dstOrigin = MTL::Origin(dstOffset, 0, 0);
                        break;
                    }
                    case MTL::TextureType2D:
                    {
                        dstOrigin = MTL::Origin(dstOffset % this->getResource()->width(), dstOffset / this->getResource()->width(), 0);
                        break;
                    }
                    case MTL::TextureType3D:
                    {
                        uint32_t width = this->getResource()->width();
                        uint32_t height = this->getResource()->height();

                        uint32_t sliceSize = width * height;
                        uint32_t z = dstOffset / sliceSize;
                        uint32_t rem2D = dstOffset % sliceSize;
                        uint32_t y = rem2D / width;
                        uint32_t x = rem2D % width;

                        dstOrigin = MTL::Origin(x, y, z);
                        break;
                    }
                    default:
                        ALICEVISION_THROW_ERROR("Unsupported texture type");
                }
                blitEncoder->copyFromBuffer(
                  remoteBuffer.get(), srcOffset * sizeof(Type), 0, 0, srcSize, this->m_resource.get(), 0, 0, dstOrigin);
                blitEncoder->endEncoding();
                cmdBuffer->commit();
                cmdBuffer->waitUntilCompleted();
                return;
            }
            else  // Completely different device, copy via host
            {
                std::shared_ptr<std::span<Type>> hostData = resource.getData();
                const uint64_t count = resource.getResource()->length() / sizeof(Type);
                this->copyFromHost(hostData->data(), count, srcOffset, dstOffset);
                return;
            }
        }
        else  // Copying from Texture
        {
            if (this->m_device->registryID() == resource.m_device->registryID())  // Copy from same device, easy
            {
                NS::SharedPtr<MTL::CommandBuffer> cmdBuffer =
                  NS::TransferPtr(MTLDeviceManager::getInstance()->getQueue(this->m_device->registryID())->commandBuffer());
                NS::SharedPtr<MTL::BlitCommandEncoder> blitEncoder = NS::TransferPtr(cmdBuffer->blitCommandEncoder());
                // Make the origin
                MTL::Origin srcOrigin;
                switch (resource.getResource()->textureType())
                {
                    case MTL::TextureType1D:
                    {
                        srcOrigin = MTL::Origin(srcOffset, 0, 0);
                        break;
                    }
                    case MTL::TextureType2D:
                    {
                        srcOrigin = MTL::Origin(srcOffset % resource.getResource()->width(), srcOffset / resource.getResource()->width(), 0);
                        break;
                    }
                    case MTL::TextureType3D:
                    {
                        uint32_t width = resource.getResource()->width();
                        uint32_t height = resource.getResource()->height();

                        uint32_t sliceSize = width * height;
                        uint32_t z = srcOffset / sliceSize;
                        uint32_t rem2D = srcOffset % sliceSize;
                        uint32_t y = rem2D / width;
                        uint32_t x = rem2D % width;

                        srcOrigin = MTL::Origin(x, y, z);
                        break;
                    }
                    default:
                        ALICEVISION_THROW_ERROR("Unsupported texture type");
                }
                // Make the size
                MTL::Size srcSize;
                switch (resource.getResource()->textureType())
                {
                    case MTL::TextureType1D:
                    {
                        srcSize = MTL::Size(resource.getResource()->width() - srcOffset, 1, 1);
                        break;
                    }
                    case MTL::TextureType2D:
                    {
                        uint32_t width = resource.getResource()->width();
                        uint32_t height = resource.getResource()->height();

                        // Compute (x, y) origin
                        uint32_t x = srcOffset % width;
                        uint32_t y = srcOffset / width;

                        srcOrigin = MTL::Origin(x, y, 0);

                        // Calculate how many texels to copy
                        uint32_t remaining = width * height;

                        // First row span
                        uint32_t firstRowCount = std::min(width - x, remaining);
                        remaining -= firstRowCount;

                        uint32_t fullRows = remaining / width;
                        uint32_t tailCount = remaining % width;

                        // Height span
                        uint32_t heightSpan = 1 + fullRows + (tailCount > 0 ? 1 : 0);

                        // Width span: full row if more than one row, otherwise just the tail
                        uint32_t widthSpan = (heightSpan == 1) ? firstRowCount : width;

                        srcSize = MTL::Size(widthSpan, heightSpan, 1);
                        break;
                    }
                    case MTL::TextureType3D:
                    {
                        uint32_t width = resource.getResource()->width();
                        uint32_t height = resource.getResource()->height();
                        uint32_t depth = resource.getResource()->depth();

                        uint64_t sliceSize = width * height;

                        uint64_t z = srcOffset / sliceSize;
                        uint64_t rem2D = srcOffset % sliceSize;
                        uint64_t y = rem2D / width;
                        uint64_t x = rem2D % width;

                        srcOrigin = MTL::Origin(x, y, z);

                        // Compute size
                        uint64_t remaining = width * height * depth;

                        // First row of first slice
                        uint64_t firstRowCount = std::min(width - x, remaining);
                        remaining -= firstRowCount;

                        // Compute how many full 2D slices (width × height) we cover
                        uint32_t fullSlices = remaining / sliceSize;
                        remaining %= sliceSize;

                        // Compute how many full rows in the last partial slice
                        uint32_t fullRows = remaining / width;
                        uint32_t tailCount = remaining % width;

                        // Now calculate actual spans
                        uint32_t depthSpan = 1 + fullSlices + ((fullRows > 0 || tailCount > 0) ? 1 : 0);
                        uint32_t heightSpan = (depthSpan == 1) ? (1 + fullRows + (tailCount > 0 ? 1 : 0)) : height;
                        uint32_t widthSpan = (depthSpan == 1 && heightSpan == 1) ? firstRowCount : width;

                        srcSize = MTL::Size(widthSpan, heightSpan, depthSpan);
                        break;
                    }
                    default:
                        ALICEVISION_THROW_ERROR("Unsupported texture type");
                }
                // Create destination origin
                MTL::Origin dstOrigin;
                switch (this->getResource()->textureType())
                {
                    case MTL::TextureType1D:
                    {
                        dstOrigin = MTL::Origin(dstOffset, 0, 0);
                        break;
                    }
                    case MTL::TextureType2D:
                    {
                        dstOrigin = MTL::Origin(dstOffset % this->getResource()->width(), dstOffset / this->getResource()->width(), 0);
                        break;
                    }
                    case MTL::TextureType3D:
                    {
                        uint32_t width = this->getResource()->width();
                        uint32_t height = this->getResource()->height();

                        uint32_t sliceSize = width * height;
                        uint32_t z = dstOffset / sliceSize;
                        uint32_t rem2D = dstOffset % sliceSize;
                        uint32_t y = rem2D / width;
                        uint32_t x = rem2D % width;

                        dstOrigin = MTL::Origin(x, y, z);
                        break;
                    }
                    default:
                        ALICEVISION_THROW_ERROR("Unsupported texture type");
                }
                blitEncoder->copyFromTexture(resource.getResource().get(), 0, 0, srcOrigin, srcSize, this->m_resource.get(), 0, 0, dstOrigin);
                blitEncoder->endEncoding();
                cmdBuffer->commit();
                cmdBuffer->waitUntilCompleted();
                return;
            }
            else if (this->m_device->peerGroupID() == resource.m_device->peerGroupID())  // Copy from same peer group, use remote view
            {
                const MTLTexture remoteTexture =
                  NS::TransferPtr(static_cast<MTLTexture>(resource.getResource())->newRemoteTextureViewForDevice(this->m_device.get()));
                NS::SharedPtr<MTL::CommandBuffer> cmdBuffer =
                  NS::TransferPtr(MTLDeviceManager::getInstance()->getQueue(this->m_device->registryID())->commandBuffer());
                NS::SharedPtr<MTL::BlitCommandEncoder> blitEncoder = NS::TransferPtr(cmdBuffer->blitCommandEncoder());
                // Make the origin
                MTL::Origin srcOrigin;
                switch (remoteTexture->textureType())
                {
                    case MTL::TextureType1D:
                    {
                        srcOrigin = MTL::Origin(srcOffset, 0, 0);
                        break;
                    }
                    case MTL::TextureType2D:
                    {
                        srcOrigin = MTL::Origin(srcOffset % remoteTexture->width(), srcOffset / remoteTexture->width(), 0);
                        break;
                    }
                    case MTL::TextureType3D:
                    {
                        uint32_t width = remoteTexture->width();
                        uint32_t height = remoteTexture->height();

                        uint32_t sliceSize = width * height;
                        uint32_t z = srcOffset / sliceSize;
                        uint32_t rem2D = srcOffset % sliceSize;
                        uint32_t y = rem2D / width;
                        uint32_t x = rem2D % width;

                        srcOrigin = MTL::Origin(x, y, z);
                        break;
                    }
                    default:
                        ALICEVISION_THROW_ERROR("Unsupported texture type");
                }
                // Make the size
                MTL::Size srcSize;
                switch (resource.getResource()->textureType())
                {
                    case MTL::TextureType1D:
                    {
                        srcSize = MTL::Size(remoteTexture->width() - srcOffset, 1, 1);
                        break;
                    }
                    case MTL::TextureType2D:
                    {
                        uint32_t width = remoteTexture->width();
                        uint32_t height = remoteTexture->height();

                        // Compute (x, y) origin
                        uint32_t x = srcOffset % width;
                        uint32_t y = srcOffset / width;

                        srcOrigin = MTL::Origin(x, y, 0);

                        // Calculate how many texels to copy
                        uint32_t remaining = width * height;

                        // First row span
                        uint32_t firstRowCount = std::min(width - x, remaining);
                        remaining -= firstRowCount;

                        uint32_t fullRows = remaining / width;
                        uint32_t tailCount = remaining % width;

                        // Height span
                        uint32_t heightSpan = 1 + fullRows + (tailCount > 0 ? 1 : 0);

                        // Width span: full row if more than one row, otherwise just the tail
                        uint32_t widthSpan = (heightSpan == 1) ? firstRowCount : width;

                        srcSize = MTL::Size(widthSpan, heightSpan, 1);
                        break;
                    }
                    case MTL::TextureType3D:
                    {
                        uint32_t width = remoteTexture->width();
                        uint32_t height = remoteTexture->height();
                        uint32_t depth = remoteTexture->depth();

                        uint64_t sliceSize = width * height;

                        uint64_t z = srcOffset / sliceSize;
                        uint64_t rem2D = srcOffset % sliceSize;
                        uint64_t y = rem2D / width;
                        uint64_t x = rem2D % width;

                        srcOrigin = MTL::Origin(x, y, z);

                        // Compute size
                        uint64_t remaining = width * height * depth;

                        // First row of first slice
                        uint64_t firstRowCount = std::min(width - x, remaining);
                        remaining -= firstRowCount;

                        // Compute how many full 2D slices (width × height) we cover
                        uint32_t fullSlices = remaining / sliceSize;
                        remaining %= sliceSize;

                        // Compute how many full rows in the last partial slice
                        uint32_t fullRows = remaining / width;
                        uint32_t tailCount = remaining % width;

                        // Now calculate actual spans
                        uint32_t depthSpan = 1 + fullSlices + ((fullRows > 0 || tailCount > 0) ? 1 : 0);
                        uint32_t heightSpan = (depthSpan == 1) ? (1 + fullRows + (tailCount > 0 ? 1 : 0)) : height;
                        uint32_t widthSpan = (depthSpan == 1 && heightSpan == 1) ? firstRowCount : width;

                        srcSize = MTL::Size(widthSpan, heightSpan, depthSpan);
                        break;
                    }
                    default:
                        ALICEVISION_THROW_ERROR("Unsupported texture type");
                }
                // Create destination origin
                MTL::Origin dstOrigin;
                switch (this->getResource()->textureType())
                {
                    case MTL::TextureType1D:
                    {
                        dstOrigin = MTL::Origin(dstOffset, 0, 0);
                        break;
                    }
                    case MTL::TextureType2D:
                    {
                        dstOrigin = MTL::Origin(dstOffset % this->getResource()->width(), dstOffset / this->getResource()->width(), 0);
                        break;
                    }
                    case MTL::TextureType3D:
                    {
                        uint32_t width = this->getResource()->width();
                        uint32_t height = this->getResource()->height();

                        uint32_t sliceSize = width * height;
                        uint32_t z = dstOffset / sliceSize;
                        uint32_t rem2D = dstOffset % sliceSize;
                        uint32_t y = rem2D / width;
                        uint32_t x = rem2D % width;

                        dstOrigin = MTL::Origin(x, y, z);
                        break;
                    }
                    default:
                        ALICEVISION_THROW_ERROR("Unsupported texture type");
                }
                blitEncoder->copyFromTexture(remoteTexture.get(), 0, 0, srcOrigin, srcSize, this->m_resource.get(), 0, 0, dstOrigin);
                blitEncoder->endEncoding();
                cmdBuffer->commit();
                cmdBuffer->waitUntilCompleted();
                return;
            }
            else  // Completely different device, copy via host
            {
                std::shared_ptr<std::span<Type>> hostData = resource.getData();
                const uint64_t count = resource.getResource()->width() * resource.getResource()->height() * resource.getResource()->depth();
                this->copyFromHost(hostData->data(), count, srcOffset, dstOffset);
                return;
            }
        }
    }
}

template<ResourceType Resource, typename Type>
std::shared_ptr<std::span<Type>> MTLSharedResource<Resource, Type>::getData() const
{
    if constexpr (std::is_same_v<Resource, MTLBuffer>)  // Buffer
    {
        if (this->m_isCPUShared && !this->m_needsExplicitSync)
        {
            // This can be mapped forever, as it shares the same address space and is always implicitly synced
            auto* ptr = static_cast<Type*>(static_cast<MTLBuffer>(this->m_resource)->contents());

            auto size = this->m_resource->length() / sizeof(Type);

            return std::make_shared<std::span<Type>>(ptr, size);
        }
        else if (this->m_isCPUShared && this->m_needsExplicitSync)
        {
            // We can map, but we need to explicitly synchronize the resource first
            NS::SharedPtr<MTL::CommandBuffer> cmdBuffer =
              NS::TransferPtr(MTLDeviceManager::getInstance()->getQueue(this->m_device->registryID())->commandBuffer());
            NS::SharedPtr<MTL::BlitCommandEncoder> blitEncoder = NS::TransferPtr(cmdBuffer->blitCommandEncoder());
            blitEncoder->synchronizeResource(this->m_resource.get());
            blitEncoder->endEncoding();
            cmdBuffer->commit();
            cmdBuffer->waitUntilCompleted();
            return std::make_shared<std::span<Type>>(static_cast<Type*>(static_cast<MTLBuffer>(this->m_resource)->contents()),
                                                     this->m_resource->length() / sizeof(Type));
        }
        else
        {
            // A staging buffer is needed
            // We explicitly retain a copy of the NS::SharedPtr in the destructor, so the user dos not have to release it explicitly
            MTLSharedResource stagingBuffer = MTLDeviceManager::getInstance()
                                                ->getResourceManager(this->m_device->registryID())
                                                ->template createBuffer<Type>(this->m_resource->length() / sizeof(Type), true);
            stagingBuffer.copyFromResource(*this, 0, 0);
            return std::shared_ptr<std::span<Type>>(
                new std::span<Type>(*stagingBuffer.getData()),
                [retainedStagingBuffer = stagingBuffer](std::span<Type>*) {
                    (void)retainedStagingBuffer;
                });
        }
    }
    else  // Texture
    {
        if (this->m_isCPUShared)
        {
            const MTLTexture tex = static_cast<MTLTexture>(this->m_resource);
            Type* hostData = static_cast<Type*>(malloc(sizeof(Type) * (tex->width() * tex->height() * tex->depth())));
            MTL::Region region;
            switch (tex->textureType())
            {
                case MTL::TextureType1D:
                {
                    region = MTL::Region::Make1D(0, tex->width());
                    break;
                }
                case MTL::TextureType2D:
                {
                    region = MTL::Region::Make2D(0, 0, tex->width(), tex->height());
                    break;
                }
                case MTL::TextureType3D:
                {
                    region = MTL::Region::Make3D(0, 0, 0, tex->width(), tex->height(), tex->depth());
                    break;
                }
                default:
                    ALICEVISION_THROW_ERROR("Unsupported texture type");
            }
            NS::SharedPtr<MTL::CommandBuffer> cmdBuffer =
              NS::TransferPtr(MTLDeviceManager::getInstance()->getQueue(this->m_device->registryID())->commandBuffer());
            NS::SharedPtr<MTL::BlitCommandEncoder> blitEncoder = NS::TransferPtr(cmdBuffer->blitCommandEncoder());
            blitEncoder->synchronizeResource(this->m_resource.get());
            blitEncoder->endEncoding();
            cmdBuffer->commit();
            cmdBuffer->waitUntilCompleted();
            tex->getBytes(static_cast<void*>(hostData), tex->width() * sizeof(Type), region, 0);
            return std::shared_ptr<std::span<Type>>(new std::span<Type>(hostData, tex->width() * tex->height() * tex->depth()), [hostData](std::span<Type>* ptr) {
                delete ptr;
                std::free(hostData);
            });
        }
        else
        {
            // Create a staging texture
            MTLSharedResource stagingTex;
            const MTLTexture srcTex = static_cast<MTLTexture>(this->m_resource);
            switch (srcTex->textureType())
            {
                case MTL::TextureType1D:
                {
                    stagingTex = MTLDeviceManager::getInstance()
                                   ->getResourceManager(this->m_device->registryID())
                                   ->template createTexture1D<Type>(srcTex->pixelFormat(), srcTex->width(), 1, true);
                    break;
                }
                case MTL::TextureType2D:
                {
                    stagingTex = MTLDeviceManager::getInstance()
                                   ->getResourceManager(this->m_device->registryID())
                                   ->template createTexture2D<Type>(srcTex->pixelFormat(), srcTex->width(), srcTex->height(), 1, true);
                    break;
                }
                case MTL::TextureType3D:
                {
                    stagingTex = MTLDeviceManager::getInstance()
                                   ->getResourceManager(this->m_device->registryID())
                                   ->template createTexture3D<Type>(srcTex->pixelFormat(), srcTex->width(), srcTex->height(), srcTex->depth(), 1, true);
                    break;
                }
                default:
                    ALICEVISION_THROW_ERROR("Unsupported texture type");
            }
            stagingTex.copyFromResource(*this, 0, 0);
            return std::shared_ptr<std::span<Type>>(
              new std::span<Type>(stagingTex.getData()->data(),
                                  stagingTex.getResource()->width() * stagingTex.getResource()->height() * stagingTex.getResource()->depth()),
                                  [retainedStagingTex = stagingTex](std::span<Type>* ptr) { (void)retainedStagingTex; });
        }
    }
}

template<ResourceType Resource, typename Type>
const MTL::Size MTLSharedResource<Resource, Type>::getTextureDimensions() const
    requires std::is_same_v<Resource, NS::SharedPtr<MTL::Texture>>
{
    const MTL::Size dimensions(
        this->m_resource->width(),
        this->m_resource->height(),
        this->m_resource->depth()
    );
    return dimensions;
}

template<ResourceType Resource, typename Type>
const uint64_t MTLSharedResource<Resource, Type>::getBytesPadded() const
    requires std::is_same_v<Resource, NS::SharedPtr<MTL::Texture>>
{
    return this->m_resource->allocatedSize();
}

template<ResourceType Resource, typename Type>
const uint64_t MTLSharedResource<Resource, Type>::getBytesUnpadded() const
    requires std::is_same_v<Resource, NS::SharedPtr<MTL::Texture>>
{
    // width * height * depth * sizeof(Type)
    const MTL::Size dim = this->getTextureDimensions();
    return (dim.width * dim.height * dim.depth) * sizeof(Type);
}

template<ResourceType Resource, typename Type>
const uint64_t MTLSharedResource<Resource, Type>::getBytes() const
    requires std::is_same_v<Resource, NS::SharedPtr<MTL::Buffer>>
{
    return this->m_resource->allocatedSize();
}

template<ResourceType Resource, typename Type>
const uint64_t MTLSharedResource<Resource, Type>::getResourceDeviceID() const
{
    return this->m_device->registryID();
}


}  // namespace gpu
}  // namespace aliceVision

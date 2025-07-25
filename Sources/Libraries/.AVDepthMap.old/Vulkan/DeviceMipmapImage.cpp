// This file is part of the AliceVision project.
// Copyright (c) 2023 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <AVNumeric/numeric.hpp>

#include <AVDepthMap/Vulkan/DeviceMipmapImage.hpp>
#include <AVDepthMap/Vulkan/DeviceGaussianArray.hpp>
#include <AVDepthMap/Vulkan/DivUp.hpp>
#include <AVDepthMapVulkanKernels/AVDepthMapVulkanKernels.hpp>

namespace aliceVision {
namespace depthMap {

void DeviceMipmapImage::fill(const VulkanImage<VulkanRGBA>& in_img, int minDownscale, int maxDownscale)
{
    // update private members
    _minDownscale = minDownscale;
    _maxDownscale = maxDownscale;
    _width = in_img.getDimensionSizes().value().width;
    _height = in_img.getDimensionSizes().value().height;

    std::shared_ptr<VulkanImage<VulkanRGBA>> img_dmpPtr = std::make_shared<VulkanImage<VulkanRGBA>>(in_img);

    // downscale device-sided full-size input image buffer to min downscale
    if (minDownscale > 1)
    {
        // create downscaled image buffer
        const size_t downscaledWidth = size_t(divideRoundUp(int(_width), int(minDownscale)));
        const size_t downscaledHeight = size_t(divideRoundUp(int(_height), int(minDownscale)));
        std::shared_ptr<VulkanImage<VulkanRGBA>> downscaledImg_dmpPtr = std::make_shared<VulkanImage<VulkanRGBA>>(VulkanMemoryBase<VulkanRGBA>::create2DImage(in_img.getDeviceID(), downscaledWidth, downscaledHeight, in_img.getImageFormat().value(), in_img.getMipmapLevelCount().value(), true, false));

        // downscale with gaussian blur the full-size image texture
        const int gaussianFilterRadius = minDownscale;

        // Vulkan integers are 32-bit
        struct PushConstants
        {
            int32_t downscale;
            int32_t gaussianRadius;
        };

        const auto pc = PushConstants{minDownscale, gaussianFilterRadius};

        // Create and submit the Vulkan command
        const auto cmdManager = VulkanCommandManager::getInstance(img_dmpPtr->getDeviceID());
        cmdManager->wait()
            ->reset()
            ->begin()
#if defined(ALICEVISION_DEPTHMAP_TEXTURE_USE_UCHAR)
            ->shader(ALICEVISION_LOOKUP_SPIRV_MODULE(DownscaleWithGaussianBlur_uchar))
#elif defined(ALICEVISION_DEPTHMAP_TEXTURE_USE_HALF)
            ->shader(ALICEVISION_LOOKUP_SPIRV_MODULE(DownscaleWithGaussianBlur_half))
#else
            ->shader(ALICEVISION_LOOKUP_SPIRV_MODULE(DownscaleWithGaussianBlur_float))
#endif
            ->pushConstant(&pc)
            ->bind(*img_dmpPtr, vk::DescriptorType::eSampledImage)
            ->bind(*downscaledImg_dmpPtr, vk::DescriptorType::eStorageImage)
            ->bind(DeviceGaussianArray::getInstance(img_dmpPtr->getDeviceID())->getGaussianArrayOffsetBuffer(), vk::DescriptorType::eUniformBuffer)
            ->bind(DeviceGaussianArray::getInstance(img_dmpPtr->getDeviceID())->getGaussianArrayBuffer(), vk::DescriptorType::eUniformBuffer)
            ->transferImageLayout(*img_dmpPtr, vk::ImageLayout::eShaderReadOnlyOptimal, 0, 1)
            ->transferImageLayout(*downscaledImg_dmpPtr, vk::ImageLayout::eGeneral, 0, 1)
            ->workgroups(vk::Extent3D(divUp(downscaledImg_dmpPtr->getDimensionSizes().value().width, 32), divUp(downscaledImg_dmpPtr->getDimensionSizes().value().height, 2), 1))
            ->dispatch()
            ->end()
            ->submit()
            ->wait();

        // use downscaled image buffer as input full-size image buffer
        img_dmpPtr.swap(downscaledImg_dmpPtr);
    }

    // in-place color conversion into CIELAB
    const auto cmdManager = VulkanCommandManager::getInstance(img_dmpPtr->getDeviceID());
    cmdManager->wait()
        ->reset()
        ->begin()
#if defined(ALICEVISION_DEPTHMAP_TEXTURE_USE_UCHAR)
        ->shader(ALICEVISION_LOOKUP_SPIRV_MODULE(RGB2LAB_uchar))
#elif defined(ALICEVISION_DEPTHMAP_TEXTURE_USE_HALF)
        ->shader(ALICEVISION_LOOKUP_SPIRV_MODULE(RGB2LAB_half))
#else
        ->shader(ALICEVISION_LOOKUP_SPIRV_MODULE(RGB2LAB_float))
#endif
        ->bind(*img_dmpPtr, vk::DescriptorType::eStorageImage)
        ->transferImageLayout(*img_dmpPtr, vk::ImageLayout::eGeneral, 0, 1)
        ->workgroups(vk::Extent3D(divUp(img_dmpPtr->getDimensionSizes().value().width, 32), divUp(img_dmpPtr->getDimensionSizes().value().height, 2), 1))
        ->dispatch()
        ->end()
        ->submit()
        ->wait();

    // Initialize the mip levels
    for(uint32_t i=1; i < img_dmpPtr->getMipmapLevelCount().value(); ++i) {

        // Vulkan integers are 32-bit
        struct PushConstants
        {
            int32_t radius;
            uint32_t out_mipLevel;
        };

        // Radius is currently hard-coded to 2
        const auto pc = PushConstants{2, i};

        cmdManager->reset()
            ->begin()
#if defined(ALICEVISION_DEPTHMAP_TEXTURE_USE_UCHAR)
            ->shader(ALICEVISION_LOOKUP_SPIRV_MODULE(CreateMipmapLevel_uchar))
#elif defined(ALICEVISION_DEPTHMAP_TEXTURE_USE_HALF)
            ->shader(ALICEVISION_LOOKUP_SPIRV_MODULE(CreateMipmapLevel_half))
#else
            ->shader(ALICEVISION_LOOKUP_SPIRV_MODULE(CreateMipmapLevel_float))
#endif
            ->pushConstant(pc)
            ->bind(*img_dmpPtr, vk::DescriptorType::eSampledImage)
            ->bind(*img_dmpPtr, vk::DescriptorType::eStorageImage, i)
            ->transferImageLayout(*img_dmpPtr, vk::ImageLayout::eShaderReadOnlyOptimal, i - 1, i)
            ->transferImageLayout(*img_dmpPtr, vk::ImageLayout::eGeneral, i, i + 1)
            ->workgroups(vk::Extent3D(divUp(img_dmpPtr->getDimensionSizes().value().width, 16), divUp(img_dmpPtr->getDimensionSizes().value().height, 16), 1))
            ->dispatch()
            ->end()
            ->submit()
            ->wait();
    }

    // Set the downscaled image as the image member in the class
    this->_mipmappedImage = img_dmpPtr;
}

float DeviceMipmapImage::getLevel(unsigned int downscale) const
{
    // check given downscale
    if (downscale < _minDownscale || downscale > _maxDownscale)
        ALICEVISION_THROW_ERROR("Cannot get device mipmap image level (downscale: " << downscale << ")");

    return log2(float(downscale) / float(_minDownscale));
}

vk::Extent2D DeviceMipmapImage::getDimensions(unsigned int downscale) const
{
    // check given downscale
    if (downscale < _minDownscale || downscale > _maxDownscale)
        ALICEVISION_THROW_ERROR("Cannot get device mipmap image level dimensions (downscale: " << downscale << ")");

    return vk::Extent2D(divideRoundUp(int(_width), int(downscale)), divideRoundUp(int(_height), int(downscale)));
}

std::pair<vk::SharedImageView, vk::SharedSampler> DeviceMipmapImage::getTextureObject() const
{
    return std::make_pair(this->_mipmappedImage->getImageViewForAllMipMapLevels().value(), this->_mipmappedImage->getSampler().value());
}

}  // namespace depthMap
}  // namespace aliceVision

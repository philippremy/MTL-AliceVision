// This file is part of the AliceVision project.
// Copyright (c) 2023 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <AVDepthMap/Metal/host/DeviceMipmapImage.hpp>

#include <AVNumeric/numeric.hpp>
#include <AVDepthMap/Metal/imageProcessing/deviceColorConversion_host.hpp>
#include <AVDepthMap/Metal/imageProcessing/deviceGaussianFilter_host.hpp>
#include <AVDepthMap/Metal/imageProcessing/deviceMipmappedArray_host.hpp>

namespace aliceVision {
namespace depthMap {

void DeviceMipmapImage::fill(const MTLHostMemoryHeap<MTLRGBA, 2>& in_img_hmh, int minDownscale, int maxDownscale, const uint64_t deviceID)
{
    // update private members
    _minDownscale = minDownscale;
    _maxDownscale = maxDownscale;
    _width = in_img_hmh.getSize().x();
    _height = in_img_hmh.getSize().y();
    _levels = std::log2(maxDownscale / minDownscale) + 1;

    // allocate the device-sided full-size input image buffer
    auto img_dmpPtr = std::make_shared<MTLDeviceMemoryPitched<MTLRGBA, 2>>(in_img_hmh.getSize(), deviceID, false, "DeviceMipmapImage full-size input image buffer");

    // copy the host-sided full-size input image buffer onto the device-sided image buffer
    img_dmpPtr->copyFrom(in_img_hmh);

    // downscale device-sided full-size input image buffer to min downscale
    if (minDownscale > 1)
    {
        // create downscaled image buffer
        const size_t downscaledWidth = size_t(divideRoundUp(int(_width), int(minDownscale)));
        const size_t downscaledHeight = size_t(divideRoundUp(int(_height), int(minDownscale)));
        auto downscaledImg_dmpPtr = std::make_shared<MTLDeviceMemoryPitched<MTLRGBA, 2>>(MTLSize<2>(downscaledWidth, downscaledHeight), deviceID, false, "DeviceMipmapImage downscaled input image buffer");

        // Create wrapper class
        MTLMipmappedTexture<MTLRGBA> fullSizeImg_tex = MTLMipmappedTexture<MTLRGBA>(img_dmpPtr->getBuffer(), img_dmpPtr->getUnitsInDim(0), img_dmpPtr->getUnitsInDim(1), 1);

        // downscale with gaussian blur the full-size image texture
        const int gaussianFilterRadius = minDownscale;
        mtl_downscaleWithGaussianBlur(*downscaledImg_dmpPtr, fullSizeImg_tex, minDownscale, gaussianFilterRadius, deviceID);

        // use downscaled image buffer as input full-size image buffer
        img_dmpPtr.swap(downscaledImg_dmpPtr);
    }

    // in-place color conversion into CIELAB
    mtl_rgb2lab(*img_dmpPtr, deviceID);

    // Create the mipmapped texture
    this->_textureObject = MTLMipmappedTexture<MTLRGBA>(img_dmpPtr->getUnitsInDim(0), img_dmpPtr->getUnitsInDim(1), _levels, deviceID);

    mtl_createMipmappedArrayFromImage(&this->_textureObject, *img_dmpPtr, _levels, deviceID);
}

float DeviceMipmapImage::getLevel(unsigned int downscale) const
{
    // check given downscale
    if (downscale < _minDownscale || downscale > _maxDownscale)
        ALICEVISION_THROW_ERROR("Cannot get device mipmap image level (downscale: " << downscale << ")");

    return log2(float(downscale) / float(_minDownscale));
}

MTLSize<2> DeviceMipmapImage::getDimensions(unsigned int downscale) const
{
    // check given downscale
    if (downscale < _minDownscale || downscale > _maxDownscale)
        ALICEVISION_THROW_ERROR("Cannot get device mipmap image level dimensions (downscale: " << downscale << ")");

    return MTLSize<2>(divideRoundUp(int(_width), int(downscale)), divideRoundUp(int(_height), int(downscale)));
}

}  // namespace depthMap
}  // namespace aliceVision

//
// Created by Philipp Remy on 18.07.25.
//

#include "AVDepthMap/Metal/DeviceCache.hpp"

#include <AVDepthMap/Metal/CreateMipmapLevel_PushConstants.hpp>
#include <AVDepthMap/Metal/DeviceMipmapImage.hpp>
#include <AVDepthMap/Metal/DownscaleWithGaussianBlur_PushConstants.hpp>
#include <AVGPU/Metal/command.hpp>
#include <AVNumeric/numeric.hpp>

namespace aliceVision {
namespace depthMap {

using namespace gpu;

DeviceMipmapImage::~DeviceMipmapImage()
{
}

void DeviceMipmapImage::fill(const uint64_t deviceID, const MTLSharedResource<MTLTexture, MTLRGBA>& in_img_hmh, int minDownscale, int maxDownscale)
{
    // Get image dims
    const auto imgDims = in_img_hmh.getTextureDimensions();
    // update private members
    _minDownscale = minDownscale;
    _maxDownscale = maxDownscale;
    _width = imgDims.width;
    _height = imgDims.height;
    _levels = std::log2(maxDownscale / minDownscale) + 1;

    // allocate the device-sided full-size input image buffer
    auto img_dmpPtr = std::make_unique<MTLSharedResource<MTLTexture, MTLRGBA>>(in_img_hmh);

    // Get resource manager
    const auto resMng = MTLDeviceManager::getInstance()->getResourceManager(in_img_hmh.getResourceDeviceID());
    auto cmdManager = MTLDeviceManager::getInstance()->getCommandManager(in_img_hmh.getResourceDeviceID());

    // Get the gaussian array resources
    const auto gaussianArrayWrapper = DeviceCache::getInstance().getGaussianArrayWrapper(deviceID);
    const auto& gaussianOffsetArray = gaussianArrayWrapper->getGaussianArrayOffsetBuffer();
    const auto& gaussianArray = gaussianArrayWrapper->getGaussianArrayBuffer();

    // downscale device-sided full-size input image buffer to min downscale
    if (minDownscale > 1)
    {
        // create downscaled image buffer
        const size_t downscaledWidth = size_t(divideRoundUp(int(_width), int(minDownscale)));
        const size_t downscaledHeight = size_t(divideRoundUp(int(_height), int(minDownscale)));
        auto downscaledImg_dmpPtr = std::make_unique<MTLSharedResource<MTLTexture, MTLRGBA>>(resMng->createTexture2D<MTLRGBA>(MTLPixelBaseTypeFormat, downscaledWidth, downscaledHeight, _levels, false));
        const auto downscaledImageDims = downscaledImg_dmpPtr->getTextureDimensions();

        // downscale with gaussian blur the full-size image texture
        const int gaussianFilterRadius = minDownscale;

        // Create the command pipeline
        cmdManager
        ->loadLibrary("AVDepthMapMetalKernels", "org.aliceVision.AVDepthMap")
        ->reset()
        ->commandBuffer()
        ->pipeline("aliceVision::depthMap::DownscaleWithGaussianBlur", "AVDepthMapMetalKernels")
        ->commandEncoder()
        ->bind(*downscaledImg_dmpPtr, 0)
        ->bind(in_img_hmh, 1)
        ->createSampler(0)
        ->bind(gaussianOffsetArray, 1)
        ->bind(gaussianArray, 2)
        ->pushConstants(DownscaleWithGaussianBlur_PushConstants{minDownscale, gaussianFilterRadius})
        ->dispatchDimensions(MTL::Size(downscaledImageDims.width, downscaledImageDims.height, 1), MTL::Size(32, 2, 1))
        ->endRecording()
        ->commitCommands()
        ->waitAll();

        // use downscaled image buffer as input full-size image buffer
        img_dmpPtr.swap(downscaledImg_dmpPtr);
    }

    // in-place color conversion into CIELAB
    // Create the command pipeline

    // Image dims
    const auto img_dims = img_dmpPtr->getTextureDimensions();

    cmdManager
    ->loadLibrary("AVDepthMapMetalKernels", "org.aliceVision.AVDepthMap")
    ->reset()
    ->commandBuffer()
    ->pipeline("aliceVision::depthMap::RGB2LAB", "AVDepthMapMetalKernels")
    ->commandEncoder()
    ->bind(*img_dmpPtr, 0)
    ->dispatchDimensions(MTL::Size(img_dims.width, img_dims.height, 1), MTL::Size(32, 2, 1))
    ->endRecording()
    ->commitCommands()
    ->waitAll();

    // Start with mip level 0
    MTL::Size mipLevelSize = img_dims;

    // Set up command manager
    cmdManager
    ->loadLibrary("AVDepthMapMetalKernels", "org.aliceVision.AVDepthMap")
    ->reset()
    ->commandBuffer()
    ->pipeline("aliceVision::depthMap::CreateMipmapLevel", "AVDepthMapMetalKernels");

    // Create mip levels
    for(size_t l = 1; l < _levels; ++l)
    {
        // current level width/height
        mipLevelSize.width  /= 2;
        mipLevelSize.height /= 2;

        // Append each command encoder to the command buffer
        // TODO: Figure out if binding twice is safe
        cmdManager
        ->commandEncoder()
        ->bind(*img_dmpPtr, 0)
        ->createSampler(0)
        ->bind(*img_dmpPtr, 1)
        ->bind(gaussianOffsetArray, 1)
        ->bind(gaussianArray, 2)
        ->pushConstants(CreateMipmapLevel_PushConstants{2, static_cast<unsigned int>(l)})
        ->dispatchDimensions(MTL::Size(mipLevelSize.width, mipLevelSize.height, 1), MTL::Size(16, 16, 1))
        ->endRecording();
    }

    // Commit and wait for completion
    cmdManager
    ->commitCommands()
    ->waitAll();

    // Store the texture
    this->_metalTexture.swap(img_dmpPtr);
}

float DeviceMipmapImage::getLevel(unsigned int downscale) const
{
    // check given downscale
    if (downscale < _minDownscale || downscale > _maxDownscale)
        ALICEVISION_THROW_ERROR("Cannot get device mipmap image level (downscale: " << downscale << ")");

    return log2(float(downscale) / float(_minDownscale));
}

const MTL::Size DeviceMipmapImage::getDimensions(unsigned int downscale) const
{
    // check given downscale
    if (downscale < _minDownscale || downscale > _maxDownscale)
        ALICEVISION_THROW_ERROR("Cannot get device mipmap image level dimensions (downscale: " << downscale << ")");

    return MTL::Size(divideRoundUp(int(_width), int(downscale)), divideRoundUp(int(_height), int(downscale)), 1);
}

const MTLSharedResource<MTLTexture, MTLRGBA>& DeviceMipmapImage::getTexture() const
{
    return *this->_metalTexture;
}

}  // namespace depthMap
}  // namespace aliceVision

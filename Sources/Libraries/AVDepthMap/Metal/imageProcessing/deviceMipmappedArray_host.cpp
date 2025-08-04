// This file is part of the AliceVision project.
// Copyright (c) 2025 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <AVDepthMap/Metal/host/DeviceCache.hpp>
#include <AVDepthMap/Metal/imageProcessing/deviceGaussianFilter_host.hpp>
#include <AVDepthMap/Metal/util/KernelArgs.hpp>

namespace aliceVision {
namespace depthMap {

void mtl_createMipmappedArrayFromImage(MTLMipmappedTexture<MTLRGBA>* out_mipmappedArrayPtr,
                                       const MTLDeviceMemoryPitched<MTLRGBA, 2>& in_img_dmp,
                                       const unsigned int levels,
                                       uint64_t deviceID)
{

    MTLCommandManager* cmdMng = DeviceManager::getInstance().getCommandManager(deviceID);

    const MTLSize<2>& in_imgSize = in_img_dmp.getSize();

    // Copy the data into the first mip level
    const auto queue = DeviceManager::getInstance().getCommandQueue(deviceID);
    const auto commandBuffer = queue->commandBuffer();
    const auto blitCommandEncoder = commandBuffer->blitCommandEncoder();
    blitCommandEncoder->copyFromBuffer(in_img_dmp.getBuffer(), 0, out_mipmappedArrayPtr->getBuffer(), 0, in_img_dmp.getBytesPadded());
    blitCommandEncoder->endEncoding();
    commandBuffer->commit();
    commandBuffer->waitUntilCompleted();

    // initialize each mipmapped array level from level 0
    size_t width  = in_imgSize.x();
    size_t height = in_imgSize.y();

    // Get gauss arrays
    const MTL::Buffer* gaussOffsetBuffer = DeviceCache::getInstance().requestGaussianOffsetBuffer(deviceID);
    const MTL::Buffer* gaussArrayBuffer = DeviceCache::getInstance().requestGaussianArrayBuffer(deviceID);

    cmdMng
    ->reset()
    ->loadLibrary("AVDepthMapMTLKernels", "AVDepthMap")
    ->commandBuffer()
    ->pipeline("aliceVision::depthMap::createMipmappedArrayLevel_kernel", "AVDepthMapMTLKernels");

    for(size_t l = 1; l < levels; ++l)
    {
        // current level width/height
        width  /= 2;
        height /= 2;

        const createMipmappedArrayLevel_kernel_PC pc = createMipmappedArrayLevel_kernel_PC{
            2,
            static_cast<unsigned int>(width),
            static_cast<unsigned int>(height),
            static_cast<unsigned int>(l) - 1,
            static_cast<unsigned int>(l),
            static_cast<unsigned int>(in_img_dmp.getUnitsInDim(0)),
            static_cast<unsigned int>(in_img_dmp.getUnitsInDim(1)),
            levels
        };

        cmdMng
        ->commandEncoder()
        ->bind(out_mipmappedArrayPtr->getBuffer(), 0)
        ->bind(gaussOffsetBuffer, 1)
        ->bind(gaussArrayBuffer, 2)
        ->pushConstants(pc)
        ->dispatchDimensions({width, height, 1}, {16, 16, 1})
        ->endRecording();
        // ->commitCommands();

    }

    cmdMng
    ->commitCommands();

}

}
}

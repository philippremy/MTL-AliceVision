// This file is part of the AliceVision project.
// Copyright (c) 2025 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <AVDepthMap/Metal/device/DeviceGaussianFilter.hpp>
#include <AVDepthMap/Metal/host/DeviceCache.hpp>
#include <AVDepthMap/Metal/imageProcessing/deviceGaussianFilter_host.hpp>
#include <AVDepthMap/Metal/util/KernelArgs.hpp>

namespace aliceVision {
namespace depthMap {

void mtl_downscaleWithGaussianBlur(MTLDeviceMemoryPitched<MTLRGBA, 2>& out_downscaledImg_dmp,
                                   const MTLMipmappedTexture<MTLRGBA>& in_img_tex,
                                   int downscale,
                                   int gaussRadius,
                                   uint64_t deviceID)
{
    MTLCommandManager* cmdMng = DeviceManager::getInstance().getCommandManager(deviceID);

    const downscaleWithGaussianBlur_kernel_PC pc = downscaleWithGaussianBlur_kernel_PC{
        static_cast<unsigned int>(in_img_tex.getWidth(0)),
        static_cast<unsigned int>(in_img_tex.getHeight(0)),
        static_cast<unsigned int>(in_img_tex.getLevelCount()),
        static_cast<int>(out_downscaledImg_dmp.getPitch()),
        static_cast<unsigned int>(out_downscaledImg_dmp.getSize().x()),
        static_cast<unsigned int>(out_downscaledImg_dmp.getSize().y()),
        downscale,
        gaussRadius
    };

    // Get gauss arrays
    const MTL::Buffer* gaussOffsetBuffer = DeviceCache::getInstance().requestGaussianOffsetBuffer(deviceID);
    const MTL::Buffer* gaussArrayBuffer = DeviceCache::getInstance().requestGaussianArrayBuffer(deviceID);

    cmdMng
    ->reset()
    ->loadLibrary("AVDepthMapMTLKernels", "AVDepthMap")
    ->commandBuffer()
    ->pipeline("aliceVision::depthMap::downscaleWithGaussianBlur_kernel", "AVDepthMapMTLKernels")
    ->commandEncoder()
    ->bind(in_img_tex.getBuffer(), 0)
    ->bind(out_downscaledImg_dmp.getBuffer(), 1)
    ->bind(gaussOffsetBuffer, 2)
    ->bind(gaussArrayBuffer, 3)
    ->pushConstants(pc)
    ->dispatchDimensions({out_downscaledImg_dmp.getSize().x(), out_downscaledImg_dmp.getSize().y(), 1}, {32, 2, 1})
    ->endRecording()
    ->commitCommands();
    // ->waitAll();
}

void mtl_createConstantGaussianArray(DeviceGaussianFilterManager& mng, uint64_t deviceID, int scales)
{
    if(scales >= MAX_CONSTANT_GAUSS_SCALES)
    {
        throw std::runtime_error( "Programming error: too few scales pre-computed for Gaussian kernels. Enlarge and recompile." );
    }

    int   h_gaussianArrayOffset[MAX_CONSTANT_GAUSS_SCALES];
    float h_gaussianArray[MAX_CONSTANT_GAUSS_MEM_SIZE];

    int sumSizes = 0;

    for(int scale = 0; scale < MAX_CONSTANT_GAUSS_SCALES; ++scale)
    {
        h_gaussianArrayOffset[scale] = sumSizes;
        const int radius = scale + 1;
        const int size = 2 * radius + 1;
        sumSizes += size;
    }

    if(sumSizes >= MAX_CONSTANT_GAUSS_MEM_SIZE)
    {
        throw std::runtime_error( "Programming error: too little memory allocated for "
            + std::to_string(MAX_CONSTANT_GAUSS_SCALES) + " Gaussian kernels. Enlarge and recompile." );
    }

    for(int scale = 0; scale < MAX_CONSTANT_GAUSS_SCALES; ++scale)
    {
        const int radius = scale + 1;
        const float delta  = 1.0f;
        const int size   = 2 * radius + 1;

        for(int idx = 0; idx < size; idx++)
        {
            int x = idx - radius;
            h_gaussianArray[h_gaussianArrayOffset[scale]+idx] = expf(-(x * x) / (2 * delta * delta));
        }
    }

    // Create Metal Buffer
    auto device = DeviceManager::getInstance().getDevice(deviceID);

    NS::SharedPtr<MTL::Buffer> gaussOffsetBuffer = NS::TransferPtr(device->newBuffer(h_gaussianArrayOffset, MAX_CONSTANT_GAUSS_SCALES * sizeof(int), MTL::ResourceStorageModeManaged));
    NS::SharedPtr<MTL::Buffer> gaussArrayBuffer = NS::TransferPtr(device->newBuffer(h_gaussianArray, MAX_CONSTANT_GAUSS_MEM_SIZE * sizeof(float), MTL::ResourceStorageModeManaged));
    gaussOffsetBuffer->didModifyRange(NS::Range(0, MAX_CONSTANT_GAUSS_SCALES * sizeof(int)));
    gaussArrayBuffer->didModifyRange(NS::Range(0, MAX_CONSTANT_GAUSS_MEM_SIZE * sizeof(float)));

    // Insert into Manager
    mng.insertOffsetBuffer(gaussOffsetBuffer);
    mng.insertGaussianArrayBuffer(gaussArrayBuffer);
}

}
}

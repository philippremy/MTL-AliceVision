// This file is part of the AliceVision project.
// Copyright (c) 2023 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <AVDepthMap/NormalMapEstimator.hpp>

#include <AVSystem/Logger.hpp>
#include <AVSystem/Timer.hpp>
#include <AVUtils/filesIO.hpp>
#include <AVMVSUtils/fileIO.hpp>
#include <AVMVSUtils/mapIO.hpp>
#include <AVDepthMap/DepthMapUtils.hpp>

#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_CUDA)
#include <AVDepthMap/cuda/host/utils.hpp>
#include <AVDepthMap/cuda/host/DeviceCache.hpp>
#include <AVDepthMap/cuda/planeSweeping/deviceDepthSimilarityMap.hpp>
#elif ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_VULKAN)
    #include <AVDepthMap/Vulkan/DeviceCache.hpp>
    #include <AVDepthMap/Vulkan/DivUp.hpp>
    #include <AVDepthMapVulkanKernels/AVDepthMapVulkanKernels.hpp>
#elif ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_METAL)
    #include <AVDepthMap/Metal/DeviceCache.hpp>
    #include <AVGPU/Metal/device.hpp>
    #include <AVGPU/Metal/command.hpp>
#else
    #error No backend for DepthMap computations was selected!
#endif

#include "AVDepthMap/Metal/DepthSimMapComputeNormal_PushConstants.hpp"

#include <filesystem>

namespace fs = std::filesystem;

namespace aliceVision {
namespace depthMap {

#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_CUDA)

NormalMapEstimator::NormalMapEstimator(const mvsUtils::MultiViewParams& mp)
  : _mp(mp)
{}

void NormalMapEstimator::compute(int cudaDeviceId, const std::vector<int>& cams)
{
    // set the device to use for GPU executions
    // the CUDA runtime API is thread-safe, it maintains per-thread state about the current device
    setCudaDeviceId(cudaDeviceId);

    DeviceCache& deviceCache = DeviceCache::getInstance();
    deviceCache.build(0, 1);  // 0 mipmap image, 1 camera parameters

    for (const int rc : cams)
    {
        const std::string normalMapFilepath = getFileNameFromIndex(_mp, rc, mvsUtils::EFileType::normalMapFiltered);

        if (!utils::exists(normalMapFilepath))
        {
            const system::Timer timer;

            ALICEVISION_LOG_INFO("Compute normal map (rc: " << rc << ")");

            // add R camera parameters to the device cache (device constant memory)
            // no additional downscale applied, we are working at input depth map resolution
            deviceCache.addCameraParams(rc, 1 /*downscale*/, _mp);

            // get R camera parameters id in device constant memory array
            const int rcDeviceCameraParamsId = deviceCache.requestCameraParamsId(rc, 1 /*downscale*/, _mp);

            // read input depth map
            image::Image<float> in_depthMap;
            mvsUtils::readMap(rc, _mp, mvsUtils::EFileType::depthMapFiltered, in_depthMap);

            // get input depth map width / height
            const int width = in_depthMap.width();
            const int height = in_depthMap.height();

            // default tile parameters, no tiles
            const mvsUtils::TileParams tileParams;

            // fullsize roi
            const ROI roi(0, _mp.getWidth(rc), 0, _mp.getHeight(rc));

            // copy input depth map into depth/sim map in device memory
            // note: we don't need similarity for normal map computation
            //       we use depth/sim map in order to avoid code duplication
            CudaDeviceMemoryPitched<float2, 2> in_depthSimMap_dmp({size_t(width), size_t(height)});
            {
                CudaHostMemoryHeap<float2, 2> in_depthSimMap_hmh(in_depthSimMap_dmp.getSize());

                for (int x = 0; x < width; ++x)
                    for (int y = 0; y < height; ++y)
                        in_depthSimMap_hmh(size_t(x), size_t(y)) = make_float2(in_depthMap(y, x), 1.f);

                in_depthSimMap_dmp.copyFrom(in_depthSimMap_hmh);
            }

            // allocate normal map buffer in device memory
            CudaDeviceMemoryPitched<float3, 2> out_normalMap_dmp(in_depthSimMap_dmp.getSize());

            // compute normal map
            cuda_depthSimMapComputeNormal(out_normalMap_dmp, in_depthSimMap_dmp, rcDeviceCameraParamsId, 1 /*step*/, roi, 0 /*stream*/);

            // write output normal map
            writeNormalMapFiltered(rc, _mp, tileParams, roi, out_normalMap_dmp);

            ALICEVISION_LOG_INFO("Compute normal map (rc: " << rc << ") done in: " << timer.elapsedMs() << " ms.");
        }
    }

    // device cache contains CUDA objects
    // this objects should be destroyed before the end of the program (i.e. the end of the CUDA context)
    DeviceCache::getInstance().clear();
}

#elif ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_VULKAN)

NormalMapEstimator::NormalMapEstimator(const mvsUtils::MultiViewParams& mp)
  : _mp(mp)
{}

void NormalMapEstimator::compute(uint32_t deviceID, const std::vector<int>& cams)
{
    DeviceCache& deviceCache = DeviceCache::getInstance();
    deviceCache.build(deviceID, 0, 1);  // 0 mipmap image, 1 camera parameters

    for (const int rc : cams)
    {
        const std::string normalMapFilepath = getFileNameFromIndex(_mp, rc, mvsUtils::EFileType::normalMapFiltered);

        if (!utils::exists(normalMapFilepath))
        {
            const system::Timer timer;

            ALICEVISION_LOG_INFO("Compute normal map (rc: " << rc << ")");

            // add R camera parameters to the device cache (device constant memory)
            // no additional downscale applied, we are working at input depth map resolution
            deviceCache.addCameraParams(deviceID, rc, 1 /*downscale*/, _mp);

            // get R camera parameters id in device constant memory array
            const int rcDeviceCameraParamsId = deviceCache.requestCameraParamsId(deviceID, rc, 1 /*downscale*/, _mp);

            // read input depth map
            image::Image<float> in_depthMap;
            mvsUtils::readMap(rc, _mp, mvsUtils::EFileType::depthMapFiltered, in_depthMap);

            // get input depth map width / height
            const int width = in_depthMap.width();
            const int height = in_depthMap.height();

            // default tile parameters, no tiles
            const mvsUtils::TileParams tileParams;

            // fullsize roi
            const ROI roi(0, _mp.getWidth(rc), 0, _mp.getHeight(rc));

            // copy input depth map into depth/sim map in device memory
            // note: we don't need similarity for normal map computation
            //       we use depth/sim map in order to avoid code duplication
            VulkanImage<float2> in_depthSimMap_dmp = VulkanMemoryBase<float2>::create2DImage(deviceID, width, height, vk::Format::eR32G32Sfloat, 1, true, false);
            {
                std::vector<float2> hostData = {};
                hostData.reserve(width * height);

                for (int x = 0; x < width; ++x)
                    for (int y = 0; y < height; ++y)
                        hostData[y * width + x] = float2{in_depthMap(y, x), 1.f};

                in_depthSimMap_dmp.copyFromHostToImage(hostData.data(), hostData.size(), 0, 0);
            }

            // allocate normal map buffer in device memory
            VulkanImage<float4> out_normalMap_dmp = VulkanMemoryBase<float4>::create2DImage(deviceID, in_depthSimMap_dmp.getDimensionSizes().value().width, in_depthSimMap_dmp.getDimensionSizes().value().height, vk::Format::eR32G32B32A32Sfloat, 1, true, false);

            struct PushConstants
            {
                int TWsh;
                int rcDeviceCameraParamsId;
                int stepXY;
                ROI roi;
            };

            auto const pc = PushConstants{3, rcDeviceCameraParamsId, 1, roi};

            // compute normal map
            VulkanCommandManager::getInstance(deviceID)
                ->wait()
                ->reset()
                ->begin()
                ->shader(ALICEVISION_LOOKUP_SPIRV_MODULE(DepthSimMapComputeNormal))
                ->pushConstant(pc)
                ->bind(out_normalMap_dmp, vk::DescriptorType::eStorageImage)
                ->bind(in_depthSimMap_dmp, vk::DescriptorType::eStorageImage)
                ->bind(DeviceCameraParamsArray::getInstance(deviceID)->getCameraParamsArray(), vk::DescriptorType::eUniformBuffer)
                ->transferImageLayout(out_normalMap_dmp, vk::ImageLayout::eGeneral, 0, 1)
                ->transferImageLayout(in_depthSimMap_dmp, vk::ImageLayout::eGeneral, 0, 1)
                ->workgroups(vk::Extent3D(divUp(roi.width(), 8), divUp(roi.height(), 8), 1))
                ->dispatch()
                ->end()
                ->submit()
                ->wait();

            // write output normal map
            writeNormalMapFiltered(rc, _mp, tileParams, roi, out_normalMap_dmp);

            ALICEVISION_LOG_INFO("Compute normal map (rc: " << rc << ") done in: " << timer.elapsedMs() << " ms.");
        }
    }

    // device cache contains CUDA objects
    // this objects should be destroyed before the end of the program (i.e. the end of the CUDA context)
    DeviceCache::getInstance().clear(deviceID);
}

#elif ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_METAL)

using namespace gpu;

NormalMapEstimator::NormalMapEstimator(const mvsUtils::MultiViewParams& mp)
  : _mp(mp)
{}

void NormalMapEstimator::compute(const uint64_t mtlDeviceID, const std::vector<int>& cams)
{
    DeviceCache& deviceCache = DeviceCache::getInstance();
    deviceCache.build(mtlDeviceID, 0, 1, std::nullopt);  // 0 mipmap image, 1 camera parameters

    for (const int rc : cams)
    {
        const std::string normalMapFilepath = getFileNameFromIndex(_mp, rc, mvsUtils::EFileType::normalMapFiltered);

        if (!utils::exists(normalMapFilepath))
        {
            const system::Timer timer;

            ALICEVISION_LOG_INFO("Compute normal map (rc: " << rc << ")");

            // add R camera parameters to the device cache (device constant memory)
            // no additional downscale applied, we are working at input depth map resolution
            deviceCache.addCameraParams(mtlDeviceID, rc, 1 /*downscale*/, _mp);

            // get R camera parameters id in device constant memory array
            const int rcDeviceCameraParamsId = deviceCache.requestCameraParamsId(mtlDeviceID, rc, 1 /*downscale*/, _mp);

            // read input depth map
            image::Image<float> in_depthMap;
            mvsUtils::readMap(rc, _mp, mvsUtils::EFileType::depthMapFiltered, in_depthMap);

            // get input depth map width / height
            const int width = in_depthMap.width();
            const int height = in_depthMap.height();

            // default tile parameters, no tiles
            const mvsUtils::TileParams tileParams;

            // fullsize roi
            const ROI roi(0, _mp.getWidth(rc), 0, _mp.getHeight(rc));

            // copy input depth map into depth/sim map in device memory
            // note: we don't need similarity for normal map computation
            //       we use depth/sim map in order to avoid code duplication
            std::vector<float4> in_depthSimMap_hmh;
            in_depthSimMap_hmh.resize(width * height);
            for (int x = 0; x < width; ++x)
                for (int y = 0; y < height; ++y)
                    in_depthSimMap_hmh[y * width + x] = float4{in_depthMap(y, x), 1.f, 0.f, 0.f};

            // Get Resource Manager
            const auto resMng = MTLDeviceManager::getInstance()->getResourceManager(mtlDeviceID);

            MTLSharedResource<MTLTexture, float4> in_depthSimMap_dmp = resMng->createTexture2D<float4>(MTL::PixelFormatRGBA32Float, width, height, 1, false);
            in_depthSimMap_dmp.copyFromHost(in_depthSimMap_hmh.data(), in_depthSimMap_hmh.size(), 0, 0);

            // allocate normal map buffer in device memory
            MTLSharedResource<MTLTexture, float4> out_normalMap_dmp = resMng->createTexture2D<float4>(MTL::PixelFormatRGBA32Float, width, height, 1, false);

            // Get Command Manager
            auto cmdMng = MTLDeviceManager::getInstance()->getCommandManager(mtlDeviceID);

            // Get Device Camera Params
            const auto deviceCameraParamsArrayWrapper = DeviceCache::getInstance().getDeviceCameraParamsArrayWrapper(mtlDeviceID);
            const auto& deviceCameraParamsArrayBuffer = deviceCameraParamsArrayWrapper->getDeviceCameraParamsBuffer();

            cmdMng
            ->loadLibrary("AVDepthMapMetalKernels", "org.aliceVision.AVDepthMap")
            ->reset()
            ->commandBuffer()
            ->pipeline("aliceVision::depthMap::DepthSimMapComputeNormal", "AVDepthMapMetalKernels")
            ->commandEncoder()
            ->bind(out_normalMap_dmp, 0)
            ->bind(in_depthSimMap_dmp, 1)
            ->bind(deviceCameraParamsArrayBuffer, 1)
            ->pushConstants(DepthSimMapComputeNormal_PushConstants{rcDeviceCameraParamsId, 1, 3, roi})
            ->dispatchDimensions(MTL::Size(roi.width(), roi.height(), 1), MTL::Size(8, 8, 1))
            ->endRecording()
            ->commitCommands()
            ->waitAll();

            // write output normal map
            writeNormalMapFiltered(rc, _mp, tileParams, roi, out_normalMap_dmp);

            ALICEVISION_LOG_INFO("Compute normal map (rc: " << rc << ") done in: " << timer.elapsedMs() << " ms.");
        }
    }

    // device cache contains CUDA objects
    // this objects should be destroyed before the end of the program (i.e. the end of the CUDA context)
    DeviceCache::getInstance().clear(mtlDeviceID);
}

#else
    #error No backend for DepthMap computations was selected!
#endif

}  // namespace depthMap
}  // namespace aliceVision

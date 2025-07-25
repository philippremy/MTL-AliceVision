// This file is part of the AliceVision project.
// Copyright (c) 2025 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <AVDepthMap/Metal/host/DeviceCache.hpp>
#include <AVDepthMap/Metal/host/DeviceCommandManager.hpp>
#include <AVDepthMap/Metal/host/DeviceManager.hpp>
#include <AVDepthMap/Metal/planeSweeping/deviceDepthSimilarityMap_host.hpp>
#include <AVDepthMap/Metal/util/KernelArgs.hpp>

namespace aliceVision {
namespace depthMap {

/**
 * @brief Copy depth and default from input depth/sim map to another depth/sim map.
 * @param[out] out_depthSimMap_dmp the output depth/sim map
 * @param[in] in_depthSimMap_dmp the input depth/sim map to copy
 * @param[in] defaultSim the default similarity value to copy
 * @param[in] stream the stream for gpu execution
 */
void mtl_depthSimMapCopyDepthOnly(MTLDeviceMemoryPitched<float2, 2>& out_depthSimMap_dmp,
                                  const MTLDeviceMemoryPitched<float2, 2>& in_depthSimMap_dmp,
                                  float defaultSim,
                                  uint64_t deviceID)
{
    // get output map dimensions
    const MTLSize<2>& depthSimMapDim = out_depthSimMap_dmp.getSize();

    MTLCommandManager* cmdMng = DeviceManager::getInstance().getCommandManager(deviceID);

    const depthSimMapCopyDepthOnly_kernel_PC pc = depthSimMapCopyDepthOnly_kernel_PC{
        static_cast<int>(out_depthSimMap_dmp.getPitch()),
        static_cast<int>(in_depthSimMap_dmp.getPitch()),
        static_cast<unsigned int>(depthSimMapDim.x()),
        static_cast<unsigned int>(depthSimMapDim.y()),
        defaultSim
    };

    cmdMng
    ->reset()
    ->loadLibrary("AVDepthMapMTLKernels", "AVDepthMap")
    ->commandBuffer()
    ->pipeline("aliceVision::depthMap::depthSimMapCopyDepthOnly_kernel", "AVDepthMapMTLKernels")
    ->commandEncoder()
    ->bind(out_depthSimMap_dmp.getBuffer(), 0)
    ->bind(in_depthSimMap_dmp.getBuffer(), 1)
    ->pushConstants(pc)
    ->dispatchDimensions({depthSimMapDim.x(), depthSimMapDim.y(), 1}, {16, 16, 1})
    ->endRecording()
    ->commitCommands()
    ->waitAll();
};

/**
 * @brief Upscale the given normal map.
 * @param[out] out_upscaledMap_dmp the output upscaled normal map
 * @param[in] in_map_dmp the normal map to upscaled
 * @param[in] roi the 2d region of interest
 * @param[in] stream the stream for gpu execution
 */
void mtl_normalMapUpscale(MTLDeviceMemoryPitched<float3, 2>& out_upscaledMap_dmp,
                          const MTLDeviceMemoryPitched<float3, 2>& in_map_dmp,
                          const ROI& roi,
                          uint64_t deviceID)
{
    // compute upscale ratio
    const MTLSize<2>& out_mapDim = out_upscaledMap_dmp.getSize();
    const MTLSize<2>& in_mapDim = in_map_dmp.getSize();
    const float ratio = float(in_mapDim.x()) / float(out_mapDim.x());

    MTLCommandManager* cmdMng = DeviceManager::getInstance().getCommandManager(deviceID);

    const mapUpscale_kernel_PC pc = mapUpscale_kernel_PC{
        static_cast<int>(out_upscaledMap_dmp.getPitch()),
        static_cast<int>(in_map_dmp.getPitch()),
        ratio,
        roi
    };

    cmdMng
    ->reset()
    ->loadLibrary("AVDepthMapMTLKernels", "AVDepthMap")
    ->commandBuffer()
    ->pipeline("aliceVision::depthMap::mapUpscale_kernel", "AVDepthMapMTLKernels")
    ->commandEncoder()
    ->bind(out_upscaledMap_dmp.getBuffer(), 0)
    ->bind(in_map_dmp.getBuffer(), 1)
    ->pushConstants(pc)
    ->dispatchDimensions({roi.width(), roi.height(), 1}, {16, 16, 1})
    ->endRecording()
    ->commitCommands()
    ->waitAll();
};

/**
 * @brief Smooth thickness map with adjacent pixels.
 * @param[in,out] inout_depthThicknessMap_dmp the depth/thickness map
 * @param[in] sgmParams the Semi Global Matching parameters
 * @param[in] refineParams the Refine parameters
 * @param[in] roi the 2d region of interest
 * @param[in] stream the stream for gpu execution
 */
void mtl_depthThicknessSmoothThickness(MTLDeviceMemoryPitched<float2, 2>& inout_depthThicknessMap_dmp,
                                       const SgmParams& sgmParams,
                                       const RefineParams& refineParams,
                                       const ROI& roi,
                                       uint64_t deviceID)
{
    const int sgmScaleStep = sgmParams.scale * sgmParams.stepXY;
    const int refineScaleStep = refineParams.scale * refineParams.stepXY;

    // min/max number of Refine samples in SGM thickness area
    const float minNbRefineSamples = 2.f;
    const float maxNbRefineSamples = max(sgmScaleStep / float(refineScaleStep), minNbRefineSamples);

    // min/max SGM thickness inflate factor
    const float minThicknessInflate = refineParams.halfNbDepths / maxNbRefineSamples;
    const float maxThicknessInflate = refineParams.halfNbDepths / minNbRefineSamples;

    MTLCommandManager* cmdMng = DeviceManager::getInstance().getCommandManager(deviceID);

    const depthThicknessMapSmoothThickness_kernel_PC pc = depthThicknessMapSmoothThickness_kernel_PC{
        static_cast<int>(inout_depthThicknessMap_dmp.getPitch()),
        minThicknessInflate,
        maxThicknessInflate,
        roi
    };

    cmdMng
    ->reset()
    ->loadLibrary("AVDepthMapMTLKernels", "AVDepthMap")
    ->commandBuffer()
    ->pipeline("aliceVision::depthMap::depthThicknessMapSmoothThickness_kernel", "AVDepthMapMTLKernels")
    ->commandEncoder()
    ->bind(inout_depthThicknessMap_dmp.getBuffer(), 0)
    ->pushConstants(pc)
    ->dispatchDimensions({roi.width(), roi.height(), 1}, {8, 8, 1})
    ->endRecording()
    ->commitCommands()
    ->waitAll();
};

/**
 * @brief Upscale the given depth/thickness map, filter masked pixels and compute pixSize from thickness.
 * @param[out] out_upscaledDepthPixSizeMap_dmp the output upscaled depth/pixSize map
 * @param[in] in_sgmDepthThicknessMap_dmp the input SGM depth/thickness map
 * @param[in] rcDeviceCameraParamsId the R camera parameters id for array in device constant memory
 * @param[in] rcDeviceMipmapImage the R mipmap image in device memory container
 * @param[in] refineParams the Refine parameters
 * @param[in] roi the 2d region of interest
 * @param[in] stream the stream for gpu execution
 */
void mtl_computeSgmUpscaledDepthPixSizeMap(MTLDeviceMemoryPitched<float2, 2>& out_upscaledDepthPixSizeMap_dmp,
                                           const MTLDeviceMemoryPitched<float2, 2>& in_sgmDepthThicknessMap_dmp,
                                           const int rcDeviceCameraParamsId,
                                           const DeviceMipmapImage& rcDeviceMipmapImage,
                                           const RefineParams& refineParams,
                                           const ROI& roi,
                                           uint64_t deviceID)
{
    // compute upscale ratio
    const MTLSize<2>& out_mapDim = out_upscaledDepthPixSizeMap_dmp.getSize();
    const MTLSize<2>& in_mapDim = in_sgmDepthThicknessMap_dmp.getSize();
    const float ratio = float(in_mapDim.x()) / float(out_mapDim.x());

    // get R mipmap image level and dimensions
    const float rcMipmapLevel = rcDeviceMipmapImage.getLevel(refineParams.scale);
    const MTLSize<2> rcLevelDim = rcDeviceMipmapImage.getDimensions(refineParams.scale);

    MTLCommandManager* cmdMng = DeviceManager::getInstance().getCommandManager(deviceID);

    // Get DeviceCameraParams Buffer
    const MTL::Buffer* deviceCameraParamsBuffer = DeviceCache::getInstance().requestDeviceCameraParamsBuffer(deviceID);

    // kernel execution
    if(refineParams.interpolateMiddleDepth)
    {
        const computeSgmUpscaledDepthPixSizeMap_bilinear_kernel_PC pc = computeSgmUpscaledDepthPixSizeMap_bilinear_kernel_PC{
            static_cast<int>(out_upscaledDepthPixSizeMap_dmp.getPitch()),
            static_cast<int>(in_sgmDepthThicknessMap_dmp.getPitch()),
            rcDeviceCameraParamsId,
            static_cast<unsigned int>(rcDeviceMipmapImage.getLevel0Width()),
            static_cast<unsigned int>(rcDeviceMipmapImage.getLevel0Height()),
            static_cast<unsigned int>(rcDeviceMipmapImage.getLevelCount()),
            static_cast<unsigned int>(rcLevelDim.x()),
            static_cast<unsigned int>(rcLevelDim.y()),
            rcMipmapLevel,
            refineParams.stepXY,
            refineParams.halfNbDepths,
            ratio,
            roi
        };

        cmdMng
        ->reset()
        ->loadLibrary("AVDepthMapMTLKernels", "AVDepthMap")
        ->commandBuffer()
        ->pipeline("aliceVision::depthMap::computeSgmUpscaledDepthPixSizeMap_bilinear_kernel", "AVDepthMapMTLKernels")
        ->commandEncoder()
        ->bind(out_upscaledDepthPixSizeMap_dmp.getBuffer(), 0)
        ->bind(in_sgmDepthThicknessMap_dmp.getBuffer(), 1)
        ->bind(rcDeviceMipmapImage.getTextureObject(), 2)
        ->bind(deviceCameraParamsBuffer, 3)
        ->pushConstants(pc)
        ->dispatchDimensions({roi.width(), roi.height(), 1}, {16, 16, 1})
        ->endRecording()
        ->commitCommands()
        ->waitAll();
    }
    else
    {
        const computeSgmUpscaledDepthPixSizeMap_nearestNeighbor_kernel_PC pc = computeSgmUpscaledDepthPixSizeMap_nearestNeighbor_kernel_PC{
            static_cast<int>(out_upscaledDepthPixSizeMap_dmp.getPitch()),
            static_cast<int>(in_sgmDepthThicknessMap_dmp.getPitch()),
            rcDeviceCameraParamsId,
            static_cast<unsigned int>(rcDeviceMipmapImage.getLevel0Width()),
            static_cast<unsigned int>(rcDeviceMipmapImage.getLevel0Height()),
            static_cast<unsigned int>(rcDeviceMipmapImage.getLevelCount()),
            static_cast<unsigned int>(rcLevelDim.x()),
            static_cast<unsigned int>(rcLevelDim.y()),
            rcMipmapLevel,
            refineParams.stepXY,
            refineParams.halfNbDepths,
            ratio,
            roi
        };

        cmdMng
        ->reset()
        ->loadLibrary("AVDepthMapMTLKernels", "AVDepthMap")
        ->commandBuffer()
        ->pipeline("aliceVision::depthMap::computeSgmUpscaledDepthPixSizeMap_nearestNeighbor_kernel", "AVDepthMapMTLKernels")
        ->commandEncoder()
        ->bind(out_upscaledDepthPixSizeMap_dmp.getBuffer(), 0)
        ->bind(in_sgmDepthThicknessMap_dmp.getBuffer(), 1)
        ->bind(rcDeviceMipmapImage.getTextureObject(), 2)
        ->bind(deviceCameraParamsBuffer, 3)
        ->pushConstants(pc)
        ->dispatchDimensions({roi.width(), roi.height(), 1}, {16, 16, 1})
        ->endRecording()
        ->commitCommands()
        ->waitAll();
    }
};

/**
 * @brief Compute the normal map from the depth/sim map (only depth is used).
 * @param[out] out_normalMap_dmp the output normal map
 * @param[in] in_depthSimMap_dmp the input depth/sim map (only depth is used)
 * @param[in] rcDeviceCameraParamsId the R camera parameters id for array in device constant memory
 * @param[in] stepXY the input depth/sim map stepXY factor
 * @param[in] roi the 2d region of interest
 * @param[in] stream the stream for gpu execution
 */
void mtl_depthSimMapComputeNormal(MTLDeviceMemoryPitched<float3, 2>& out_normalMap_dmp,
                                  const MTLDeviceMemoryPitched<float2, 2>& in_depthSimMap_dmp,
                                  const int rcDeviceCameraParamsId,
                                  const int stepXY,
                                  const ROI& roi,
                                  uint64_t deviceID)
{
    MTLCommandManager* cmdMng = DeviceManager::getInstance().getCommandManager(deviceID);

    // Get DeviceCameraParams Buffer
    const MTL::Buffer* deviceCameraParamsBuffer = DeviceCache::getInstance().requestDeviceCameraParamsBuffer(deviceID);

    const depthSimMapComputeNormal_kernel_PC pc = depthSimMapComputeNormal_kernel_PC{
        3,
        static_cast<int>(out_normalMap_dmp.getPitch()),
        static_cast<int>(in_depthSimMap_dmp.getPitch()),
        rcDeviceCameraParamsId,
        stepXY,
        roi
    };

    cmdMng
    ->reset()
    ->loadLibrary("AVDepthMapMTLKernels", "AVDepthMap")
    ->commandBuffer()
    ->pipeline("aliceVision::depthMap::depthSimMapComputeNormal_kernel", "AVDepthMapMTLKernels")
    ->commandEncoder()
    ->bind(out_normalMap_dmp.getBuffer(), 0)
    ->bind(in_depthSimMap_dmp.getBuffer(), 1)
    ->bind(deviceCameraParamsBuffer, 2)
    ->pushConstants(pc)
    ->dispatchDimensions({roi.width(), roi.height(), 1}, {8, 8, 1})
    ->endRecording()
    ->commitCommands()
    ->waitAll();
};

/**
 * @brief Optimize a depth/sim map with the refineFused depth/sim map and the SGM depth/pixSize map.
 * @param[out] out_optimizeDepthSimMap_dmp the output optimized depth/sim map
 * @param[in,out] inout_imgVariance_dmp the image variance buffer
 * @param[in,out] inout_tmpOptDepthMap_dmp the temporary optimized depth map buffer
 * @param[in] in_sgmDepthPixSizeMap_dmp the input SGM upscaled depth/pixSize map
 * @param[in] in_refineDepthSimMap_dmp the input refined and fused depth/sim map
 * @param[in] rcDeviceCameraParamsId the R camera parameters id for array in device constant memory
 * @param[in] rcDeviceMipmapImage the R mipmap image in device memory container
 * @param[in] refineParams the Refine parameters
 * @param[in] roi the 2d region of interest
 * @param[in] stream the stream for gpu execution
 */
void mtl_depthSimMapOptimizeGradientDescent(MTLDeviceMemoryPitched<float2, 2>& out_optimizeDepthSimMap_dmp,
                                            MTLDeviceMemoryPitched<float, 2>& inout_imgVariance_dmp,
                                            MTLDeviceMemoryPitched<float, 2>& inout_tmpOptDepthMap_dmp,
                                            const MTLDeviceMemoryPitched<float2, 2>& in_sgmDepthPixSizeMap_dmp,
                                            const MTLDeviceMemoryPitched<float2, 2>& in_refineDepthSimMap_dmp,
                                            const int rcDeviceCameraParamsId,
                                            const DeviceMipmapImage& rcDeviceMipmapImage,
                                            const RefineParams& refineParams,
                                            const ROI& roi,
                                            uint64_t deviceID)
{

    // get R mipmap image level and dimensions
    const float rcMipmapLevel = rcDeviceMipmapImage.getLevel(refineParams.scale);
    const MTLSize<2> rcLevelDim = rcDeviceMipmapImage.getDimensions(refineParams.scale);

    // initialize depth/sim map optimized with SGM depth/pixSize map
    out_optimizeDepthSimMap_dmp.copyFrom(in_sgmDepthPixSizeMap_dmp);

    MTLCommandManager* cmdMng = DeviceManager::getInstance().getCommandManager(deviceID);

    // Get DeviceCameraParams Buffer
    const MTL::Buffer* deviceCameraParamsBuffer = DeviceCache::getInstance().requestDeviceCameraParamsBuffer(deviceID);

    const optimize_varLofLABtoW_kernel_PC pc = optimize_varLofLABtoW_kernel_PC{
        static_cast<int>(inout_imgVariance_dmp.getPitch()),
        static_cast<unsigned int>(rcDeviceMipmapImage.getLevel0Width()),
        static_cast<unsigned int>(rcDeviceMipmapImage.getLevel0Height()),
        static_cast<unsigned int>(rcDeviceMipmapImage.getLevelCount()),
        static_cast<unsigned int>(rcLevelDim.x()),
        static_cast<unsigned int>(rcLevelDim.y()),
        rcMipmapLevel,
        refineParams.stepXY,
        roi
    };

    cmdMng
    ->reset()
    ->loadLibrary("AVDepthMapMTLKernels", "AVDepthMap")
    ->commandBuffer()
    ->pipeline("aliceVision::depthMap::optimize_varLofLABtoW_kernel", "AVDepthMapMTLKernels")
    ->commandEncoder()
    ->bind(inout_imgVariance_dmp.getBuffer(), 0)
    ->bind(rcDeviceMipmapImage.getTextureObject(), 1)
    ->pushConstants(pc)
    ->dispatchDimensions({roi.width(), roi.height(), 1}, {32, 2, 1})
    ->endRecording()
    ->commitCommands()
    ->waitAll();

    for(int iter = 0; iter < refineParams.optimizationNbIterations; ++iter) // default nb iterations is 100
    {
        // copy depths values from out_depthSimMapOptimized_dmp to inout_tmpOptDepthMap_dmp
        const optimize_getOptDeptMapFromOptDepthSimMap_kernel_PC pc2 = optimize_getOptDeptMapFromOptDepthSimMap_kernel_PC{
            static_cast<int>(inout_tmpOptDepthMap_dmp.getPitch()),
            static_cast<int>(out_optimizeDepthSimMap_dmp.getPitch()),
            roi
        };

        cmdMng
        ->reset()
        ->loadLibrary("AVDepthMapMTLKernels", "AVDepthMap")
        ->commandBuffer()
        ->pipeline("aliceVision::depthMap::optimize_getOptDeptMapFromOptDepthSimMap_kernel", "AVDepthMapMTLKernels")
        ->commandEncoder()
        ->bind(inout_tmpOptDepthMap_dmp.getBuffer(), 0)
        ->bind(out_optimizeDepthSimMap_dmp.getBuffer(), 1)
        ->pushConstants(pc2)
        ->dispatchDimensions({roi.width(), roi.height(), 1}, {16, 16, 1})
        ->endRecording()
        ->commitCommands()
        ->waitAll();

        // copy depths values from out_depthSimMapOptimized_dmp to inout_tmpOptDepthMap_dmp
        const optimize_depthSimMap_kernel_PC pc3 = optimize_depthSimMap_kernel_PC{
            static_cast<int>(out_optimizeDepthSimMap_dmp.getPitch()),
            static_cast<int>(in_sgmDepthPixSizeMap_dmp.getPitch()),
            static_cast<int>(in_refineDepthSimMap_dmp.getPitch()),
            rcDeviceCameraParamsId,
            static_cast<unsigned int>(inout_imgVariance_dmp.getUnitsInDim(0)),
            static_cast<unsigned int>(inout_imgVariance_dmp.getUnitsInDim(1)),
            1,
            static_cast<unsigned int>(inout_tmpOptDepthMap_dmp.getUnitsInDim(0)),
            static_cast<unsigned int>(inout_tmpOptDepthMap_dmp.getUnitsInDim(1)),
            1,
            iter,
            roi
        };

        cmdMng
        ->reset()
        ->loadLibrary("AVDepthMapMTLKernels", "AVDepthMap")
        ->commandBuffer()
        ->pipeline("aliceVision::depthMap::optimize_depthSimMap_kernel", "AVDepthMapMTLKernels")
        ->commandEncoder()
        ->bind(out_optimizeDepthSimMap_dmp.getBuffer(), 0)
        ->bind(in_sgmDepthPixSizeMap_dmp.getBuffer(), 1)
        ->bind(in_refineDepthSimMap_dmp.getBuffer(), 2)
        ->bind(inout_imgVariance_dmp.getBuffer(), 3)
        ->bind(inout_tmpOptDepthMap_dmp.getBuffer(), 4)
        ->bind(deviceCameraParamsBuffer, 5)
        ->pushConstants(pc3)
        ->dispatchDimensions({roi.width(), roi.height(), 1}, {16, 16, 1})
        ->endRecording()
        ->commitCommands()
        ->waitAll();
    }
};

}  // namespace depthMap
}  // namespace aliceVision

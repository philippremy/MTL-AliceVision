// This file is part of the AliceVision project.
// Copyright (c) 2022 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <AVMVSData/ROI.hpp>
#include <AVDepthMap/SgmParams.hpp>
#include <AVDepthMap/RefineParams.hpp>
#include <AVDepthMap/Metal/host/memory.hpp>
#include <AVDepthMap/Metal/host/DeviceMipmapImage.hpp>
#include <AVDepthMap/Metal/util/MetalTypes.hpp>

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
                                  uint64_t deviceID);

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
                          uint64_t deviceID);

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
                                       uint64_t deviceID);

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
                                           uint64_t deviceID);

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
                                  uint64_t deviceID);

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
                                            uint64_t deviceID);

}  // namespace depthMap
}  // namespace aliceVision

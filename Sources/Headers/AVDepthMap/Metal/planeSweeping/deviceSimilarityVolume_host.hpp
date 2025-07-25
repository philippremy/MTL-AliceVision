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
 * @brief Add similarity values from a given volume to another given volume.
 * @param[in,out] inout_volume_dmp the input/output similarity volume in device memory
 * @param[in] in_volume_dmp the input similarity volume in device memory
 * @param[in] stream the stream for gpu execution
 */
void mtl_volumeAdd(MTLDeviceMemoryPitched<TSimRefine, 3>& inout_volume_dmp,
                   const MTLDeviceMemoryPitched<TSimRefine, 3>& in_volume_dmp,
                   uint64_t deviceID);

/**
 * @brief Update second best similarity volume uninitialized values with first best volume values.
 * @param[in] in_volBestSim_dmp the best similarity volume in device memory
 * @param[out] inout_volSecBestSim_dmp the second best similarity volume in device memory
 * @param[in] stream the stream for gpu execution
 */
void mtl_volumeUpdateUninitializedSimilarity(const MTLDeviceMemoryPitched<TSim, 3>& in_volBestSim_dmp,
                                             MTLDeviceMemoryPitched<TSim, 3>& inout_volSecBestSim_dmp,
                                             uint64_t deviceID);

/**
 * @brief Compute the best / second best similarity volume for the given RC / TC.
 * @param[out] out_volBestSim_dmp the best similarity volume in device memory
 * @param[out] out_volSecBestSim_dmp the second best similarity volume in device memory
 * @param[in] in_depths_dmp the R camera depth list in device memory
 * @param[in] rcDeviceCameraParamsId the R camera parameters id for array in device constant memory
 * @param[in] tcDeviceCameraParamsId the T camera parameters id for array in device constant memory
 * @param[in] rcDeviceMipmapImage the R mipmap image in device memory container
 * @param[in] tcDeviceMipmapImage the T mipmap image in device memory container
 * @param[in] sgmParams the Semi Global Matching parameters
 * @param[in] depthRange the volume depth range to compute
 * @param[in] roi the 2d region of interest
 * @param[in] stream the stream for gpu execution
 */
void mtl_volumeComputeSimilarity(MTLDeviceMemoryPitched<TSim, 3>& out_volBestSim_dmp,
                                 MTLDeviceMemoryPitched<TSim, 3>& out_volSecBestSim_dmp,
                                 const MTLDeviceMemoryPitched<float, 2>& in_depths_dmp,
                                 const int rcDeviceCameraParamsId,
                                 const int tcDeviceCameraParamsId,
                                 const DeviceMipmapImage& rcDeviceMipmapImage,
                                 const DeviceMipmapImage& tcDeviceMipmapImage,
                                 const SgmParams& sgmParams,
                                 const Range& depthRange,
                                 const ROI& roi,
                                 uint64_t deviceID);

/**
 * @brief Refine the best similarity volume for the given RC / TC.
 * @param[out] inout_volSim_dmp the similarity volume in device memory
 * @param[in] in_sgmDepthPixSizeMap_dmp the SGM upscaled depth/pixSize map (useful to get middle depth) in device memory
 * @param[in] in_sgmNormalMap_dmpPtr (or nullptr) the SGM upscaled normal map in device memory
 * @param[in] rcDeviceCameraParamsId the R camera parameters id for array in device constant memory
 * @param[in] tcDeviceCameraParamsId the T camera parameters id for array in device constant memory
 * @param[in] rcDeviceMipmapImage the R mipmap image in device memory container
 * @param[in] tcDeviceMipmapImage the T mipmap image in device memory container
 * @param[in] refineParams the Refine parameters
 * @param[in] depthRange the volume depth range to compute
 * @param[in] roi the 2d region of interest
 * @param[in] stream the stream for gpu execution
 */
void mtl_volumeRefineSimilarity(MTLDeviceMemoryPitched<TSimRefine, 3>& inout_volSim_dmp,
                                const MTLDeviceMemoryPitched<float2, 2>& in_sgmDepthPixSizeMap_dmp,
                                const MTLDeviceMemoryPitched<float3, 2>* in_sgmNormalMap_dmpPtr,
                                const int rcDeviceCameraParamsId,
                                const int tcDeviceCameraParamsId,
                                const DeviceMipmapImage& rcDeviceMipmapImage,
                                const DeviceMipmapImage& tcDeviceMipmapImage,
                                const RefineParams& refineParams,
                                const Range& depthRange,
                                const ROI& roi,
                                uint64_t deviceID);

/**
 * @brief Filter / Optimize the given similarity volume
 * @param[out] out_volSimFiltered_dmp the output similarity volume in device memory
 * @param[in,out] inout_volSliceAccA_dmp the volume slice first accumulation buffer in device memory
 * @param[in,out] inout_volSliceAccB_dmp the volume slice second accumulation buffer in device memory
 * @param[in,out] inout_volAxisAcc_dmp the volume axisaccumulation buffer in device memory
 * @param[in] in_volSim_dmp the input similarity volume in device memory
 * @param[in] rcDeviceMipmapImage the R mipmap image in device memory container
 * @param[in] sgmParams the Semi Global Matching parameters
 * @param[in] lastDepthIndex the R camera last depth index
 * @param[in] roi the 2d region of interest
 * @param[in] stream the stream for gpu execution
 */
void mtl_volumeOptimize(MTLDeviceMemoryPitched<TSim, 3>& out_volSimFiltered_dmp,
                        MTLDeviceMemoryPitched<TSimAcc, 2>& inout_volSliceAccA_dmp,
                        MTLDeviceMemoryPitched<TSimAcc, 2>& inout_volSliceAccB_dmp,
                        MTLDeviceMemoryPitched<TSimAcc, 2>& inout_volAxisAcc_dmp,
                        const MTLDeviceMemoryPitched<TSim, 3>& in_volSim_dmp,
                        const DeviceMipmapImage& rcDeviceMipmapImage,
                        const SgmParams& sgmParams,
                        const int lastDepthIndex,
                        const ROI& roi,
                        uint64_t deviceID);

/**
 * @brief Retrieve the best depth/sim in the given similarity volume.
 * @param[out] out_sgmDepthThicknessMap_dmp the output depth/thickness map in device memory
 * @param[out] out_sgmDepthSimMap_dmp the output best depth/sim map in device memory
 * @param[in] in_depths_dmp the R camera depth list in device memory
 * @param[in] in_volSim_dmp the input similarity volume in device memory
 * @param[in] rcDeviceCameraParamsId the R camera parameters id for array in device constant memory
 * @param[in] sgmParams the Semi Global Matching parameters
 * @param[in] depthRange the volume depth range to compute
 * @param[in] roi the 2d region of interest
 * @param[in] stream the stream for gpu execution
 */
void mtl_volumeRetrieveBestDepth(MTLDeviceMemoryPitched<float2, 2>& out_sgmDepthThicknessMap_dmp,
                                 MTLDeviceMemoryPitched<float2, 2>* out_sgmDepthSimMap_dmp,
                                 const MTLDeviceMemoryPitched<float, 2>& in_depths_dmp,
                                 const MTLDeviceMemoryPitched<TSim, 3>& in_volSim_dmp,
                                 const int rcDeviceCameraParamsId,
                                 const SgmParams& sgmParams,
                                 const Range& depthRange,
                                 const ROI& roi,
                                 uint64_t deviceID);

/**
 * @brief Retrieve the best depth/sim in the given refined similarity volume.
 * @param[out] out_refineDepthSimMap_dmp the output refined and fused depth/sim map in device memory
 * @param[in] in_sgmDepthPixSizeMap_dmp the SGM upscaled depth/pixSize map (useful to get middle depth) in device memory
 * @param[in] in_volSim_dmp the similarity volume in device memory
 * @param[in] refineParams the Refine parameters
 * @param[in] depthRange the volume depth range to compute
 * @param[in] roi the 2d region of interest
 * @param[in] stream the stream for gpu execution
 */
void mtl_volumeRefineBestDepth(MTLDeviceMemoryPitched<float2, 2>& out_refineDepthSimMap_dmp,
                               const MTLDeviceMemoryPitched<float2, 2>& in_sgmDepthPixSizeMap_dmp,
                               const MTLDeviceMemoryPitched<TSimRefine, 3>& in_volSim_dmp,
                               const RefineParams& refineParams,
                               const ROI& roi,
                               uint64_t deviceID);

}  // namespace depthMap
}  // namespace aliceVision

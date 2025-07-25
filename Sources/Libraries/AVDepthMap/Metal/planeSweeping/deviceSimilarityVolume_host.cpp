// This file is part of the AliceVision project.
// Copyright (c) 2025 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <AVDepthMap/Metal/host/DeviceCache.hpp>
#include <AVDepthMap/Metal/host/DeviceCommandManager.hpp>
#include <AVDepthMap/Metal/host/DeviceManager.hpp>
#include <AVDepthMap/Metal/host/patchPattern.hpp>
#include <AVDepthMap/Metal/planeSweeping/deviceSimilarityVolume_host.hpp>
#include <AVDepthMap/Metal/util/KernelArgs.hpp>

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
                   uint64_t deviceID)
{
    // get input/output volume dimensions
    const MTLSize<3>& volDim = inout_volume_dmp.getSize();

    MTLCommandManager* cmdMng = DeviceManager::getInstance().getCommandManager(deviceID);

    const volume_add_kernel_PC pc = volume_add_kernel_PC{
        static_cast<int>(inout_volume_dmp.getBytesPaddedUpToDim(1)),
        static_cast<int>(inout_volume_dmp.getBytesPaddedUpToDim(0)),
        static_cast<int>(in_volume_dmp.getBytesPaddedUpToDim(1)),
        static_cast<int>(in_volume_dmp.getBytesPaddedUpToDim(0)),
        static_cast<unsigned int>(volDim.x()),
        static_cast<unsigned int>(volDim.y())
    };

    cmdMng
    ->reset()
    ->loadLibrary("AVDepthMapMTLKernels", "AVDepthMap")
    ->commandBuffer()
    ->pipeline("aliceVision::depthMap::volume_add_kernel", "AVDepthMapMTLKernels")
    ->commandEncoder()
    ->bind(inout_volume_dmp.getBuffer(), 0)
    ->bind(in_volume_dmp.getBuffer(), 1)
    ->pushConstants(pc)
    ->dispatchDimensions({volDim.x(), volDim.y(), volDim.z()}, {32, 4, 1})
    ->endRecording()
    ->commitCommands()
    ->waitAll();
}

/**
 * @brief Update second best similarity volume uninitialized values with first best volume values.
 * @param[in] in_volBestSim_dmp the best similarity volume in device memory
 * @param[out] inout_volSecBestSim_dmp the second best similarity volume in device memory
 * @param[in] stream the stream for gpu execution
 */
void mtl_volumeUpdateUninitializedSimilarity(const MTLDeviceMemoryPitched<TSim, 3>& in_volBestSim_dmp,
                                             MTLDeviceMemoryPitched<TSim, 3>& inout_volSecBestSim_dmp,
                                             uint64_t deviceID)
{
    assert(in_volBestSim_dmp.getSize() == inout_volSecBestSim_dmp.getSize());

    // get input/output volume dimensions
    const MTLSize<3>& volDim = inout_volSecBestSim_dmp.getSize();

    MTLCommandManager* cmdMng = DeviceManager::getInstance().getCommandManager(deviceID);

    const volume_updateUninitialized_kernel_PC pc = volume_updateUninitialized_kernel_PC{
        static_cast<int>(inout_volSecBestSim_dmp.getBytesPaddedUpToDim(1)),
        static_cast<int>(inout_volSecBestSim_dmp.getBytesPaddedUpToDim(0)),
        static_cast<int>(in_volBestSim_dmp.getBytesPaddedUpToDim(1)),
        static_cast<int>(in_volBestSim_dmp.getBytesPaddedUpToDim(0)),
        static_cast<unsigned int>(volDim.x()),
        static_cast<unsigned int>(volDim.y())
    };

    cmdMng
    ->reset()
    ->loadLibrary("AVDepthMapMTLKernels", "AVDepthMap")
    ->commandBuffer()
    ->pipeline("aliceVision::depthMap::volume_updateUninitialized_kernel", "AVDepthMapMTLKernels")
    ->commandEncoder()
    ->bind(inout_volSecBestSim_dmp.getBuffer(), 0)
    ->bind(in_volBestSim_dmp.getBuffer(), 1)
    ->pushConstants(pc)
    ->dispatchDimensions({volDim.x(), volDim.y(), volDim.z()}, {32, 1, 1})
    ->endRecording()
    ->commitCommands()
    ->waitAll();
}

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
                                 uint64_t deviceID)
{
    // get mipmap images level and dimensions
    const float rcMipmapLevel = rcDeviceMipmapImage.getLevel(sgmParams.scale);
    const MTLSize<2> rcLevelDim = rcDeviceMipmapImage.getDimensions(sgmParams.scale);
    const MTLSize<2> tcLevelDim = tcDeviceMipmapImage.getDimensions(sgmParams.scale);

    MTLCommandManager* cmdMng = DeviceManager::getInstance().getCommandManager(deviceID);

    const volume_computeSimilarity_kernel_PC pc = volume_computeSimilarity_kernel_PC{
        static_cast<int>(out_volBestSim_dmp.getBytesPaddedUpToDim(1)),
        static_cast<int>(out_volBestSim_dmp.getBytesPaddedUpToDim(0)),
        static_cast<int>(out_volSecBestSim_dmp.getBytesPaddedUpToDim(1)),
        static_cast<int>(out_volSecBestSim_dmp.getBytesPaddedUpToDim(0)),
        static_cast<int>(in_depths_dmp.getBytesPaddedUpToDim(0)),
        rcDeviceCameraParamsId,
        tcDeviceCameraParamsId,
        static_cast<unsigned int>(rcDeviceMipmapImage.getLevel0Width()),
        static_cast<unsigned int>(rcDeviceMipmapImage.getLevel0Height()),
        static_cast<unsigned int>(rcDeviceMipmapImage.getLevelCount()),
        static_cast<unsigned int>(tcDeviceMipmapImage.getLevel0Width()),
        static_cast<unsigned int>(tcDeviceMipmapImage.getLevel0Height()),
        static_cast<unsigned int>(tcDeviceMipmapImage.getLevelCount()),
        static_cast<unsigned int>(rcLevelDim.x()),
        static_cast<unsigned int>(rcLevelDim.y()),
        static_cast<unsigned int>(tcLevelDim.x()),
        static_cast<unsigned int>(tcLevelDim.y()),
        rcMipmapLevel,
        sgmParams.stepXY,
        sgmParams.wsh,
        (1.f / float(sgmParams.gammaC)), // inverted gammaC
        (1.f / float(sgmParams.gammaP)), // inverted gammaP
        sgmParams.useConsistentScale,
        sgmParams.useCustomPatchPattern,
        depthRange,
        roi
    };

    // Get DeviceCameraParams Buffer
    const MTL::Buffer* deviceCameraParamsBuffer = DeviceCache::getInstance().requestDeviceCameraParamsBuffer(deviceID);

    // Get Custom Patch Pattern
    const DevicePatchPattern customPatchPattern = CustomPatchPattern::getInstance().getDevicePatchPattern(deviceID).value_or(DevicePatchPattern());

    cmdMng
    ->reset()
    ->loadLibrary("AVDepthMapMTLKernels", "AVDepthMap")
    ->commandBuffer()
    ->pipeline("aliceVision::depthMap::volume_computeSimilarity_kernel", "AVDepthMapMTLKernels")
    ->commandEncoder()
    ->bind(out_volBestSim_dmp.getBuffer(), 0)
    ->bind(out_volSecBestSim_dmp.getBuffer(), 1)
    ->bind(in_depths_dmp.getBuffer(), 2)
    ->bind(rcDeviceMipmapImage.getTextureObject(), 3)
    ->bind(tcDeviceMipmapImage.getTextureObject(), 4)
    ->bind(deviceCameraParamsBuffer, 5)
    ->constant(customPatchPattern, 6)
    ->pushConstants(pc)
    ->dispatchDimensions({roi.width(), roi.height(), depthRange.size()}, {32, 1, 1})
    ->endRecording()
    ->commitCommands()
    ->waitAll();
}

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
                                uint64_t deviceID)
{
    // get mipmap images level and dimensions
    const float rcMipmapLevel = rcDeviceMipmapImage.getLevel(refineParams.scale);
    const MTLSize<2> rcLevelDim = rcDeviceMipmapImage.getDimensions(refineParams.scale);
    const MTLSize<2> tcLevelDim = tcDeviceMipmapImage.getDimensions(refineParams.scale);

    MTLCommandManager* cmdMng = DeviceManager::getInstance().getCommandManager(deviceID);

    const volume_refineSimilarity_kernel_PC pc = volume_refineSimilarity_kernel_PC{
        in_sgmNormalMap_dmpPtr != nullptr,
        static_cast<int>(inout_volSim_dmp.getBytesPaddedUpToDim(1)),
        static_cast<int>(inout_volSim_dmp.getBytesPaddedUpToDim(0)),
        static_cast<int>(in_sgmDepthPixSizeMap_dmp.getBytesPaddedUpToDim(0)),
        static_cast<int>((in_sgmNormalMap_dmpPtr == nullptr) ? 0 : in_sgmNormalMap_dmpPtr->getBytesPaddedUpToDim(0)),
        rcDeviceCameraParamsId,
        tcDeviceCameraParamsId,
        static_cast<unsigned int>(rcDeviceMipmapImage.getLevel0Width()),
        static_cast<unsigned int>(rcDeviceMipmapImage.getLevel0Height()),
        static_cast<unsigned int>(rcDeviceMipmapImage.getLevelCount()),
        static_cast<unsigned int>(tcDeviceMipmapImage.getLevel0Width()),
        static_cast<unsigned int>(tcDeviceMipmapImage.getLevel0Height()),
        static_cast<unsigned int>(tcDeviceMipmapImage.getLevelCount()),
        static_cast<unsigned int>(rcLevelDim.x()),
        static_cast<unsigned int>(rcLevelDim.y()),
        static_cast<unsigned int>(tcLevelDim.x()),
        static_cast<unsigned int>(tcLevelDim.y()),
        rcMipmapLevel,
        static_cast<int>(inout_volSim_dmp.getSize().z()),
        refineParams.stepXY,
        refineParams.wsh,
        (1.f / float(refineParams.gammaC)), // inverted gammaC
        (1.f / float(refineParams.gammaP)), // inverted gammaP
        refineParams.useConsistentScale,
        refineParams.useCustomPatchPattern,
        depthRange,
        roi
    };

    // Get DeviceCameraParams Buffer
    const MTL::Buffer* deviceCameraParamsBuffer = DeviceCache::getInstance().requestDeviceCameraParamsBuffer(deviceID);

    // Get Custom Patch Pattern
    const DevicePatchPattern customPatchPattern = CustomPatchPattern::getInstance().getDevicePatchPattern(deviceID).value_or(DevicePatchPattern());

    // Create dummy binding if needed
    MTLDeviceMemoryPitched<float3, 2> in_sgmNormalMap_dmp_DUMMY;
    if (in_sgmNormalMap_dmpPtr == nullptr)
    {
        in_sgmNormalMap_dmp_DUMMY = MTLDeviceMemoryPitched<float3, 2>(MTLSize<2>(1, 1), deviceID, false);
    }

    cmdMng
    ->reset()
    ->loadLibrary("AVDepthMapMTLKernels", "AVDepthMap")
    ->commandBuffer()
    ->pipeline("aliceVision::depthMap::volume_refineSimilarity_kernel", "AVDepthMapMTLKernels")
    ->commandEncoder()
    ->bind(inout_volSim_dmp.getBuffer(), 0)
    ->bind(in_sgmDepthPixSizeMap_dmp.getBuffer(), 1)
    ->bind((in_sgmNormalMap_dmpPtr == nullptr) ? in_sgmNormalMap_dmp_DUMMY.getBuffer() : in_sgmNormalMap_dmpPtr->getBuffer(), 2)
    ->bind(rcDeviceMipmapImage.getTextureObject(), 3)
    ->bind(tcDeviceMipmapImage.getTextureObject(), 4)
    ->bind(deviceCameraParamsBuffer, 5)
    ->constant(customPatchPattern, 6)
    ->pushConstants(pc)
    ->dispatchDimensions({roi.width(), roi.height(), depthRange.size()}, {32, 1, 1})
    ->endRecording()
    ->commitCommands()
    ->waitAll();
}

void mtl_volumeAggregatePath(MTLDeviceMemoryPitched<TSim, 3>& out_volAgr_dmp,
                             MTLDeviceMemoryPitched<TSimAcc, 2>& inout_volSliceAccA_dmp,
                             MTLDeviceMemoryPitched<TSimAcc, 2>& inout_volSliceAccB_dmp,
                             MTLDeviceMemoryPitched<TSimAcc, 2>& inout_volAxisAcc_dmp,
                             const MTLDeviceMemoryPitched<TSim, 3>& in_volSim_dmp,
                             const DeviceMipmapImage& rcDeviceMipmapImage,
                             const MTLSize<2>& rcLevelDim,
                             const float rcMipmapLevel,
                             const MTLSize<3>& axisT,
                             const SgmParams& sgmParams,
                             const int lastDepthIndex,
                             const int filteringIndex,
                             const bool invY,
                             const ROI& roi,
                             uint64_t deviceID)
{
    MTLSize<3> volDim = in_volSim_dmp.getSize();
    volDim[2] = lastDepthIndex; // override volume depth, use rc depth list last index

    const size_t volDimX = volDim[axisT[0]];
    const size_t volDimY = volDim[axisT[1]];
    const size_t volDimZ = volDim[axisT[2]];

    const int3 volDim_ = make_int3(volDim[0], volDim[1], volDim[2]);
    const int3 axisT_ = make_int3(axisT[0], axisT[1], axisT[2]);
    const int ySign = (invY ? -1 : 1);

    MTLDeviceMemoryPitched<TSimAcc, 2>* xzSliceForY_dmpPtr   = &inout_volSliceAccA_dmp; // Y slice
    MTLDeviceMemoryPitched<TSimAcc, 2>* xzSliceForYm1_dmpPtr = &inout_volSliceAccB_dmp; // Y-1 slice
    MTLDeviceMemoryPitched<TSimAcc, 2>* bestSimInYm1_dmpPtr  = &inout_volAxisAcc_dmp;   // best sim score along the Y axis for each Z value

    MTLCommandManager* cmdMng = DeviceManager::getInstance().getCommandManager(deviceID);

    const volume_getVolumeXZSlice_kernel_PC pc = volume_getVolumeXZSlice_kernel_PC{
        static_cast<int>(xzSliceForYm1_dmpPtr->getPitch()),
        static_cast<int>(in_volSim_dmp.getBytesPaddedUpToDim(1)),
        static_cast<int>(in_volSim_dmp.getBytesPaddedUpToDim(0)),
        volDim_,
        axisT_,
        0 /* Y = 0 */
    };

    // Copy the first XZ plane (at Y=0) from 'in_volSim_dmp' into 'xzSliceForYm1_dmpPtr'
    cmdMng
    ->reset()
    ->loadLibrary("AVDepthMapMTLKernels", "AVDepthMap")
    ->commandBuffer()
    ->pipeline("aliceVision::depthMap::volume_getVolumeXZSlice_kernel", "AVDepthMapMTLKernels")
    ->commandEncoder()
    ->bind(xzSliceForYm1_dmpPtr->getBuffer(), 0)
    ->bind(in_volSim_dmp.getBuffer(), 1)
    ->pushConstants(pc)
    ->dispatchDimensions({volDimX, volDimZ, 1}, {8, 8, 1})
    ->endRecording()
    ->commitCommands()
    ->waitAll();

    const volume_initVolumeYSlice_kernel_PC pc2 = volume_initVolumeYSlice_kernel_PC{
        static_cast<int>(out_volAgr_dmp.getBytesPaddedUpToDim(1)),
        static_cast<int>(out_volAgr_dmp.getBytesPaddedUpToDim(0)),
        volDim_,
        axisT_,
        0,
        255
    };

    // Set the first Z plane from 'out_volAgr_dmp' to 255
    cmdMng
    ->reset()
    ->loadLibrary("AVDepthMapMTLKernels", "AVDepthMap")
    ->commandBuffer()
    ->pipeline("aliceVision::depthMap::volume_initVolumeYSlice_kernel", "AVDepthMapMTLKernels")
    ->commandEncoder()
    ->bind(out_volAgr_dmp.getBuffer(), 0)
    ->pushConstants(pc2)
    ->dispatchDimensions({volDimX, volDimZ, 1}, {8, 8, 1})
    ->endRecording()
    ->commitCommands()
    ->waitAll();

    for(int iy = 1; iy < volDimY; ++iy)
    {
        const int y = invY ? volDimY - 1 - iy : iy;

        const volume_computeBestZInSlice_kernel_PC pc3 = volume_computeBestZInSlice_kernel_PC{
            static_cast<int>(xzSliceForYm1_dmpPtr->getPitch()),
            static_cast<int>(volDimX),
            static_cast<int>(volDimZ)
        };

        // For each column: compute the best score
        // Foreach x:
        //   bestSimInYm1[x] = min(d_xzSliceForY[1:height])
        // Set the first Z plane from 'out_volAgr_dmp' to 255
        cmdMng
        ->reset()
        ->loadLibrary("AVDepthMapMTLKernels", "AVDepthMap")
        ->commandBuffer()
        ->pipeline("aliceVision::depthMap::volume_computeBestZInSlice_kernel", "AVDepthMapMTLKernels")
        ->commandEncoder()
        ->bind(xzSliceForYm1_dmpPtr->getBuffer(), 0)
        ->bind(bestSimInYm1_dmpPtr->getBuffer(), 1)
        ->pushConstants(pc3)
        ->dispatchDimensions({volDimX, 1, 1}, {64, 1, 1})
        ->endRecording()
        ->commitCommands()
        ->waitAll();

        // Copy the 'z' plane from 'in_volSim_dmp' into 'xzSliceForY'
        const volume_getVolumeXZSlice_kernel_PC pc4 = volume_getVolumeXZSlice_kernel_PC{
            static_cast<int>(xzSliceForY_dmpPtr->getPitch()),
            static_cast<int>(in_volSim_dmp.getBytesPaddedUpToDim(1)),
            static_cast<int>(in_volSim_dmp.getBytesPaddedUpToDim(0)),
            volDim_,
            axisT_,
            y
        };

        // Copy the first XZ plane (at Y=0) from 'in_volSim_dmp' into 'xzSliceForYm1_dmpPtr'
        cmdMng
        ->reset()
        ->loadLibrary("AVDepthMapMTLKernels", "AVDepthMap")
        ->commandBuffer()
        ->pipeline("aliceVision::depthMap::volume_getVolumeXZSlice_kernel", "AVDepthMapMTLKernels")
        ->commandEncoder()
        ->bind(xzSliceForY_dmpPtr->getBuffer(), 0)
        ->bind(in_volSim_dmp.getBuffer(), 1)
        ->pushConstants(pc4)
        ->dispatchDimensions({volDimX, volDimZ, 1}, {8, 8, 1})
        ->endRecording()
        ->commitCommands()
        ->waitAll();

        // Copy the 'z' plane from 'in_volSim_dmp' into 'xzSliceForY'
        const volume_agregateCostVolumeAtXinSlices_kernel_PC pc5 = volume_agregateCostVolumeAtXinSlices_kernel_PC{
            static_cast<unsigned int>(rcDeviceMipmapImage.getLevel0Width()),
            static_cast<unsigned int>(rcDeviceMipmapImage.getLevel0Height()),
            static_cast<unsigned int>(rcDeviceMipmapImage.getLevelCount()),
            static_cast<unsigned int>(rcLevelDim.x()),
            static_cast<unsigned int>(rcLevelDim.y()),
            rcMipmapLevel,
            static_cast<int>(xzSliceForY_dmpPtr->getPitch()),
            static_cast<int>(xzSliceForYm1_dmpPtr->getPitch()),
            static_cast<int>(out_volAgr_dmp.getBytesPaddedUpToDim(1)),
            static_cast<int>(out_volAgr_dmp.getBytesPaddedUpToDim(0)),
            volDim_,
            axisT_,
            static_cast<float>(sgmParams.stepXY),
            y,
            static_cast<float>(sgmParams.p1),
            static_cast<float>(sgmParams.p2Weighting),
            ySign,
            filteringIndex,
            roi
        };

        // Copy the first XZ plane (at Y=0) from 'in_volSim_dmp' into 'xzSliceForYm1_dmpPtr'
        cmdMng
        ->reset()
        ->loadLibrary("AVDepthMapMTLKernels", "AVDepthMap")
        ->commandBuffer()
        ->pipeline("aliceVision::depthMap::volume_agregateCostVolumeAtXinSlices_kernel", "AVDepthMapMTLKernels")
        ->commandEncoder()
        ->bind(rcDeviceMipmapImage.getTextureObject(), 0)
        ->bind(xzSliceForY_dmpPtr->getBuffer(), 1)
        ->bind(xzSliceForYm1_dmpPtr->getBuffer(), 2)
        ->bind(bestSimInYm1_dmpPtr->getBuffer(), 3)
        ->bind(out_volAgr_dmp.getBuffer(), 4)
        ->pushConstants(pc5)
        ->dispatchDimensions({volDimX, volDimZ, 1}, {64, 1, 1})
        ->endRecording()
        ->commitCommands()
        ->waitAll();

        std::swap(xzSliceForYm1_dmpPtr, xzSliceForY_dmpPtr);
    }
}

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
                        uint64_t deviceID)
{
    // get R mipmap image level and dimensions
    const float rcMipmapLevel = rcDeviceMipmapImage.getLevel(sgmParams.scale);
    const MTLSize<2> rcLevelDim = rcDeviceMipmapImage.getDimensions(sgmParams.scale);

    // update aggregation volume
    int npaths = 0;
    const auto updateAggrVolume = [&](const MTLSize<3>& axisT, bool invX)
    {
        mtl_volumeAggregatePath(out_volSimFiltered_dmp,
                                 inout_volSliceAccA_dmp,
                                 inout_volSliceAccB_dmp,
                                 inout_volAxisAcc_dmp,
                                 in_volSim_dmp,
                                 rcDeviceMipmapImage,
                                 rcLevelDim,
                                 rcMipmapLevel,
                                 axisT,
                                 sgmParams,
                                 lastDepthIndex,
                                 npaths,
                                 invX,
                                 roi,
                                 deviceID);
        npaths++;
    };

    // filtering is done on the last axis
    const std::map<char, MTLSize<3>> mapAxes = {
        {'X', {1, 0, 2}}, // XYZ -> YXZ
        {'Y', {0, 1, 2}}, // XYZ
    };

    for(char axis : sgmParams.filteringAxes)
    {
        const MTLSize<3>& axisT = mapAxes.at(axis);
        updateAggrVolume(axisT, false); // without transpose
        updateAggrVolume(axisT, true);  // with transpose of the last axis
    }
}

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
                                 uint64_t deviceID)
{
    // constant kernel inputs
    const int scaleStep = sgmParams.scale * sgmParams.stepXY;
    const float thicknessMultFactor = 1.f + float(sgmParams.depthThicknessInflate);
    const float maxSimilarity = float(sgmParams.maxSimilarity) * 254.f; // convert from (0, 1) to (0, 254)

    MTLCommandManager* cmdMng = DeviceManager::getInstance().getCommandManager(deviceID);

    // Get DeviceCameraParams Buffer
    const MTL::Buffer* deviceCameraParamsBuffer = DeviceCache::getInstance().requestDeviceCameraParamsBuffer(deviceID);

    const volume_retrieveBestDepth_kernel_PC pc = volume_retrieveBestDepth_kernel_PC{
        out_sgmDepthSimMap_dmp != nullptr,
        static_cast<int>(out_sgmDepthThicknessMap_dmp.getPitch()),
        out_sgmDepthSimMap_dmp == nullptr ? 0 : static_cast<int>(out_sgmDepthThicknessMap_dmp.getPitch()),
        static_cast<int>(in_depths_dmp.getBytesPaddedUpToDim(0)),
        static_cast<int>(in_volSim_dmp.getBytesPaddedUpToDim(1)),
        static_cast<int>(in_volSim_dmp.getBytesPaddedUpToDim(0)),
        rcDeviceCameraParamsId,
        static_cast<int>(in_volSim_dmp.getSize().z()),
        scaleStep,
        thicknessMultFactor,
        maxSimilarity,
        depthRange,
        roi
    };

    // Create dummy binding if needed
    MTLDeviceMemoryPitched<float2, 2> out_sgmDepthSimMap_dmp_DUMMY;
    if (out_sgmDepthSimMap_dmp == nullptr)
    {
        out_sgmDepthSimMap_dmp_DUMMY = MTLDeviceMemoryPitched<float2, 2>(MTLSize<2>(1, 1), deviceID, false);
    }

    cmdMng
    ->reset()
    ->loadLibrary("AVDepthMapMTLKernels", "AVDepthMap")
    ->commandBuffer()
    ->pipeline("aliceVision::depthMap::volume_retrieveBestDepth_kernel", "AVDepthMapMTLKernels")
    ->commandEncoder()
    ->bind(out_sgmDepthThicknessMap_dmp.getBuffer(), 0)
    ->bind(out_sgmDepthSimMap_dmp == nullptr ? out_sgmDepthSimMap_dmp_DUMMY.getBuffer() : out_sgmDepthSimMap_dmp->getBuffer(), 1)
    ->bind(in_depths_dmp.getBuffer(), 2)
    ->bind(in_volSim_dmp.getBuffer(), 3)
    ->bind(deviceCameraParamsBuffer, 4)
    ->pushConstants(pc)
    ->dispatchDimensions({roi.width(), roi.height(), depthRange.size()}, {32, 1, 1})
    ->endRecording()
    ->commitCommands()
    ->waitAll();
}

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
                               uint64_t deviceID)
{
    // constant kernel inputs
    const int halfNbSamples = refineParams.nbSubsamples * refineParams.halfNbDepths;
    const float twoTimesSigmaPowerTwo = float(2.0 * refineParams.sigma * refineParams.sigma);

    MTLCommandManager* cmdMng = DeviceManager::getInstance().getCommandManager(deviceID);

    const volume_refineBestDepth_kernel_PC pc = volume_refineBestDepth_kernel_PC{
        static_cast<int>(out_refineDepthSimMap_dmp.getPitch()),
        static_cast<int>(in_sgmDepthPixSizeMap_dmp.getPitch()),
        static_cast<int>(in_volSim_dmp.getBytesPaddedUpToDim(1)),
        static_cast<int>(in_volSim_dmp.getBytesPaddedUpToDim(0)),
        static_cast<int>(in_volSim_dmp.getSize().z()),
        refineParams.nbSubsamples,  // number of samples between two depths
        halfNbSamples,              // number of samples (in front and behind mid depth)
        refineParams.halfNbDepths,  // number of depths  (in front and behind mid depth)
        twoTimesSigmaPowerTwo,
        roi
    };

    cmdMng
    ->reset()
    ->loadLibrary("AVDepthMapMTLKernels", "AVDepthMap")
    ->commandBuffer()
    ->pipeline("aliceVision::depthMap::volume_refineBestDepth_kernel", "AVDepthMapMTLKernels")
    ->commandEncoder()
    ->bind(out_refineDepthSimMap_dmp.getBuffer(), 0)
    ->bind(in_sgmDepthPixSizeMap_dmp.getBuffer(), 1)
    ->bind(in_volSim_dmp.getBuffer(), 2)
    ->pushConstants(pc)
    ->dispatchDimensions({roi.width(), roi.height(), 1}, {32, 1, 1})
    ->endRecording()
    ->commitCommands()
    ->waitAll();
}

}  // namespace depthMap
}  // namespace aliceVision

// This file is part of the AliceVision project.
// Copyright (c) 2025 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <metal_stdlib>
using namespace metal;

#include <AVDepthMap/Metal/device/matrix.hpp>
#include <AVDepthMap/Metal/device/Patch.hpp>
#include <AVDepthMap/Metal/util/MetalTypes.hpp>
#include <AVDepthMap/Metal/util/KernelArgs.hpp>

namespace aliceVision {
namespace depthMap {

inline void move3DPointByRcPixSize(thread float3& p,
                                   constant const DeviceCameraParams& rcDeviceCamParams,
                                   const float rcPixSize)
{
    float3 rpv = p - rcDeviceCamParams.C;
    normalize(rpv);
    p = p + rpv * rcPixSize;
}
    
inline void move3DPointByRcPixSize(device float3& p,
                                   constant const DeviceCameraParams& rcDeviceCamParams,
                                   const float rcPixSize)
{
    float3 rpv = p - rcDeviceCamParams.C;
    normalize(rpv);
    p = p + rpv * rcPixSize;
}

inline void volume_computePatch(thread Patch& patch,
                                constant const DeviceCameraParams& rcDeviceCamParams,
                                constant const DeviceCameraParams& tcDeviceCamParams,
                                const float fpPlaneDepth,
                                thread const float2& pix)
{
    patch.p = get3DPointForPixelAndFrontoParellePlaneRC(rcDeviceCamParams, pix, fpPlaneDepth);
    patch.d = computePixSize(rcDeviceCamParams, patch.p);
    computeRotCSEpip(patch, rcDeviceCamParams, tcDeviceCamParams);
}

inline float depthPlaneToDepth(constant const DeviceCameraParams& deviceCamParams,
                               const float fpPlaneDepth,
                               thread const float2& pix)
{
    DeviceCameraParams deviceCamParams_tmp = deviceCamParams;
    const float3 planep = deviceCamParams_tmp.C + deviceCamParams_tmp.ZVect * fpPlaneDepth;
    float3 v = M3x3mulV2(deviceCamParams_tmp.iP, pix);
    normalize(v);
    float3 p = linePlaneIntersect(deviceCamParams_tmp.C, v, planep, deviceCamParams_tmp.ZVect);
    return size(deviceCamParams_tmp.C - p);
}

kernel void volume_add_kernel(device TSimRefine* inout_volume_d [[buffer(0)]],
                              device const TSimRefine* in_volume_d [[buffer(1)]],
                              constant const volume_add_kernel_PC& kArgs [[buffer(30)]],
                              const uint3 gid [[thread_position_in_grid]],
                              const uint3 bid [[threadgroup_position_in_grid]])
{
    const unsigned int vx = gid.x;
    const unsigned int vy = gid.y;
    const unsigned int vz = bid.z;

    if(vx >= kArgs.volDimX || vy >= kArgs.volDimY)
        return;

    device TSimRefine* outSimPtr = get3DBufferAt(inout_volume_d, kArgs.inout_volume_s, kArgs.inout_volume_p, vx, vy, vz);

#ifdef TSIM_REFINE_USE_HALF
    // note: using built-in half addition can give bad results on some gpus
    //*outSimPtr = __hadd(*outSimPtr, *get3DBufferAt(in_volume_d, in_volume_s, in_volume_p, vx, vy, vz));
    *outSimPtr = half(float(*outSimPtr) + float(*get3DBufferAt(in_volume_d, kArgs.in_volume_s, kArgs.in_volume_p, vx, vy, vz))); // perform the addition in float
#else
    *outSimPtr += *get3DBufferAt(in_volume_d, kArgs.in_volume_s, kArgs.in_volume_p, vx, vy, vz);
#endif
}

kernel void volume_updateUninitialized_kernel(device TSim* inout_volume2nd_d [[buffer(0)]],
                                              device const TSim* in_volume1st_d [[buffer(1)]],
                                              constant const volume_updateUninitialized_kernel_PC& kArgs [[buffer(30)]],
                                              const uint3 gid [[thread_position_in_grid]],
                                              const uint3 bid [[threadgroup_position_in_grid]])
{
    const unsigned int vx = gid.x;
    const unsigned int vy = gid.y;
    const unsigned int vz = bid.z;

    if(vx >= kArgs.volDimX || vy >= kArgs.volDimY)
        return;

    // input/output second best similarity value
    device TSim* inout_simPtr = get3DBufferAt(inout_volume2nd_d, kArgs.inout_volume2nd_s, kArgs.inout_volume2nd_p, vx, vy, vz);

    if(*inout_simPtr >= 255.f) // invalid or uninitialized similarity value
    {
        // update second best similarity value with first best similarity value
        *inout_simPtr = *get3DBufferAt(in_volume1st_d, kArgs.in_volume1st_s, kArgs.in_volume1st_p, vx, vy, vz);
    }
}

kernel void volume_computeSimilarity_kernel(device TSim* out_volume1st_d [[buffer(0)]],
                                            device TSim* out_volume2nd_d [[buffer(1)]],
                                            device const float* in_depths_d [[buffer(2)]],
                                            device MTLRGBA* rcMipmapImage_tex [[buffer(3)]], // As per CUDA implementation, use Bilinear filtering and normalized coordinates
                                            device MTLRGBA* tcMipmapImage_tex [[buffer(4)]], // As per CUDA implementation, use Bilinear filtering and normalized coordinates
                                            constant const DeviceCameraParams* constantCameraParametersArray_d [[buffer(5)]],
                                            constant const DevicePatchPattern& constantDevicePatchPattern_d [[buffer(6)]],
                                            constant const volume_computeSimilarity_kernel_PC& kArgs [[buffer(30)]],
                                            const uint3 gid [[thread_position_in_grid]],
                                            const uint3 bid [[threadgroup_position_in_grid]])
{
    const unsigned int roiX = gid.x;
    const unsigned int roiY = gid.y;
    const unsigned int roiZ = bid.z;

    if(roiX >= kArgs.roi.width() || roiY >= kArgs.roi.height()) // no need to check roiZ
        return;

    // R and T camera parameters
    constant const DeviceCameraParams& rcDeviceCamParams = constantCameraParametersArray_d[kArgs.rcDeviceCameraParamsId];
    constant const DeviceCameraParams& tcDeviceCamParams = constantCameraParametersArray_d[kArgs.tcDeviceCameraParamsId];

    // corresponding volume coordinates
    const unsigned int vx = roiX;
    const unsigned int vy = roiY;
    const unsigned int vz = kArgs.depthRange.begin + roiZ;

    // corresponding image coordinates
    const float x = float(kArgs.roi.x.begin + vx) * float(kArgs.stepXY);
    const float y = float(kArgs.roi.y.begin + vy) * float(kArgs.stepXY);

    // corresponding depth plane
    const float depthPlane = *get2DBufferAt(in_depths_d, kArgs.in_depths_p, size_t(vz), 0);

    // compute patch
    Patch patch;
    volume_computePatch(patch, rcDeviceCamParams, tcDeviceCamParams, depthPlane, make_float2(x, y));

    // we do not need positive and filtered similarity values
    constexpr bool invertAndFilter = false;

    float fsim = INFINITY;

    // Create mipmapped texture wrappers
    MTLMipmappedTexture<MTLRGBA> rcMipmapImage_tex_wrapper = MTLMipmappedTexture<MTLRGBA>(rcMipmapImage_tex, kArgs.rcLevel0Width, kArgs.rcLevel0Height, kArgs.rcLevelCount);
    MTLMipmappedTexture<MTLRGBA> tcMipmapImage_tex_wrapper = MTLMipmappedTexture<MTLRGBA>(tcMipmapImage_tex, kArgs.tcLevel0Width, kArgs.tcLevel0Height, kArgs.tcLevelCount);
    
    // compute patch similarity
    if(kArgs.useCustomPatchPattern)
    {
        fsim = compNCCby3DptsYK_customPatchPattern<invertAndFilter>(rcDeviceCamParams,
                                                                    tcDeviceCamParams,
                                                                    constantDevicePatchPattern_d,
                                                                    rcMipmapImage_tex_wrapper, // As per CUDA implementation, use Bilinear filtering and normalized coordinates
                                                                    tcMipmapImage_tex_wrapper, // As per CUDA implementation, use Bilinear filtering and normalized coordinates
                                                                    kArgs.rcSgmLevelWidth,
                                                                    kArgs.rcSgmLevelHeight,
                                                                    kArgs.tcSgmLevelWidth,
                                                                    kArgs.tcSgmLevelHeight,
                                                                    kArgs.rcMipmapLevel,
                                                                    kArgs.invGammaC,
                                                                    kArgs.invGammaP,
                                                                    kArgs.useConsistentScale,
                                                                    patch);
    }
    else
    {
        fsim = compNCCby3DptsYK<invertAndFilter>(rcDeviceCamParams,
                                                 tcDeviceCamParams,
                                                 rcMipmapImage_tex_wrapper, // As per CUDA implementation, use Bilinear filtering and normalized coordinates
                                                 tcMipmapImage_tex_wrapper, // As per CUDA implementation, use Bilinear filtering and normalized coordinates
                                                 kArgs.rcSgmLevelWidth,
                                                 kArgs.rcSgmLevelHeight,
                                                 kArgs.tcSgmLevelWidth,
                                                 kArgs.tcSgmLevelHeight,
                                                 kArgs.rcMipmapLevel,
                                                 kArgs.wsh,
                                                 kArgs.invGammaC,
                                                 kArgs.invGammaP,
                                                 kArgs.useConsistentScale,
                                                 patch);
    }

    if(fsim == INFINITY) // invalid similarity
    {
      fsim = 255.0f; // 255 is the invalid similarity value
    }
    else // valid similarity
    {
      // remap similarity value
      constexpr const float fminVal = -1.0f;
      constexpr const float fmaxVal = 1.0f;
      constexpr const float fmultiplier = 1.0f / (fmaxVal - fminVal);

      fsim = (fsim - fminVal) * fmultiplier;

#ifdef TSIM_USE_FLOAT
      // no clamp
#else
      fsim = fmin(1.0f, fmax(0.0f, fsim));
#endif
      // convert from (0, 1) to (0, 254)
      // needed to store in the volume in uchar
      // 255 is reserved for the similarity initialization, i.e. undefined values
      fsim *= 254.0f;
    }

    device TSim* fsim_1st = get3DBufferAt(out_volume1st_d, kArgs.out_volume1st_s, kArgs.out_volume1st_p, size_t(vx), size_t(vy), size_t(vz));
    device TSim* fsim_2nd = get3DBufferAt(out_volume2nd_d, kArgs.out_volume2nd_s, kArgs.out_volume2nd_p, size_t(vx), size_t(vy), size_t(vz));

    if(fsim < *fsim_1st)
    {
        *fsim_2nd = *fsim_1st;
        *fsim_1st = TSim(fsim);
    }
    else if(fsim < *fsim_2nd)
    {
        *fsim_2nd = TSim(fsim);
    }
}

kernel void volume_refineSimilarity_kernel(device TSimRefine* inout_volSim_d [[buffer(0)]],
                                           device const float2* in_sgmDepthPixSizeMap_d [[buffer(1)]],
                                           device const float3* in_sgmNormalMap_d [[buffer(2)]],
                                           device MTLRGBA* rcMipmapImage_tex [[buffer(3)]],
                                           device MTLRGBA* tcMipmapImage_tex [[buffer(4)]],
                                           constant const DeviceCameraParams* constantCameraParametersArray_d [[buffer(5)]],
                                           constant const DevicePatchPattern& constantDevicePatchPattern_d [[buffer(6)]],
                                           constant const volume_refineSimilarity_kernel_PC& kArgs [[buffer(30)]],
                                           const uint3 gid [[thread_position_in_grid]],
                                           const uint3 bid [[threadgroup_position_in_grid]])
{
    const unsigned int roiX = gid.x;
    const unsigned int roiY = gid.y;
    const unsigned int roiZ = bid.z;

    if(roiX >= kArgs.roi.width() || roiY >= kArgs.roi.height()) // no need to check roiZ
        return;

    // R and T camera parameters
    constant const DeviceCameraParams& rcDeviceCamParams = constantCameraParametersArray_d[kArgs.rcDeviceCameraParamsId];
    constant const DeviceCameraParams& tcDeviceCamParams = constantCameraParametersArray_d[kArgs.tcDeviceCameraParamsId];

    // corresponding volume and depth/sim map coordinates
    const unsigned int vx = roiX;
    const unsigned int vy = roiY;
    const unsigned int vz = kArgs.depthRange.begin + roiZ;

    // corresponding image coordinates
    const float x = float(kArgs.roi.x.begin + vx) * float(kArgs.stepXY);
    const float y = float(kArgs.roi.y.begin + vy) * float(kArgs.stepXY);

    // corresponding input sgm depth/pixSize (middle depth)
    const float2 in_sgmDepthPixSize = *get2DBufferAt(in_sgmDepthPixSizeMap_d, kArgs.in_sgmDepthPixSizeMap_p, vx, vy);

    // sgm depth (middle depth) invalid or masked
    if(in_sgmDepthPixSize.x <= 0.0f)
        return;

    // initialize rc 3d point at sgm depth (middle depth)
    float3 p = get3DPointForPixelAndDepthFromRC(rcDeviceCamParams, make_float2(x, y), in_sgmDepthPixSize.x);

    // compute relative depth index offset from z center
    const int relativeDepthIndexOffset = vz - ((kArgs.volDimZ - 1) / 2);

    if(relativeDepthIndexOffset != 0)
    {
        // not z center
        // move rc 3d point by relative depth index offset * sgm pixSize
        const float pixSizeOffset = relativeDepthIndexOffset * in_sgmDepthPixSize.y; // input sgm pixSize
        move3DPointByRcPixSize(p, rcDeviceCamParams, pixSizeOffset);
    }

    // compute patch
    Patch patch;
    patch.p = p;
    patch.d = computePixSize(rcDeviceCamParams, p);

    // computeRotCSEpip
    {
      // vector from the reference camera to the 3d point
      float3 v1 = rcDeviceCamParams.C - patch.p;
      // vector from the target camera to the 3d point
      float3 v2 = tcDeviceCamParams.C - patch.p;
      normalize(v1);
      normalize(v2);

      // y has to be orthogonal to the epipolar plane
      // n has to be on the epipolar plane
      // x has to be on the epipolar plane

      patch.y = cross(v1, v2);
      normalize(patch.y);

      if(kArgs.useNormalMap) // initialize patch normal from input normal map
      {
        patch.n = *get2DBufferAt(in_sgmNormalMap_d, kArgs.in_sgmNormalMap_p, vx, vy);
      }
      else // initialize patch normal from v1 & v2
      {
        patch.n = (v1 + v2) / 2.0f;
        normalize(patch.n);
      }

      patch.x = cross(patch.y, patch.n);
      normalize(patch.x);
    }

    // we need positive and filtered similarity values
    constexpr bool invertAndFilter = true;

    float fsimInvertedFiltered = INFINITY;

    // Create mipmapped texture wrappers
    MTLMipmappedTexture<MTLRGBA> rcMipmapImage_tex_wrapper = MTLMipmappedTexture<MTLRGBA>(rcMipmapImage_tex, kArgs.rcLevel0Width, kArgs.rcLevel0Height, kArgs.rcLevelCount);
    MTLMipmappedTexture<MTLRGBA> tcMipmapImage_tex_wrapper = MTLMipmappedTexture<MTLRGBA>(tcMipmapImage_tex, kArgs.tcLevel0Width, kArgs.tcLevel0Height, kArgs.tcLevelCount);
    
    // compute similarity
    if(kArgs.useCustomPatchPattern)
    {
        fsimInvertedFiltered = compNCCby3DptsYK_customPatchPattern<invertAndFilter>(rcDeviceCamParams,
                                                                                    tcDeviceCamParams,
                                                                                    constantDevicePatchPattern_d,
                                                                                    rcMipmapImage_tex_wrapper,
                                                                                    tcMipmapImage_tex_wrapper,
                                                                                    kArgs.rcRefineLevelWidth,
                                                                                    kArgs.rcRefineLevelHeight,
                                                                                    kArgs.tcRefineLevelWidth,
                                                                                    kArgs.tcRefineLevelHeight,
                                                                                    kArgs.rcMipmapLevel,
                                                                                    kArgs.invGammaC,
                                                                                    kArgs.invGammaP,
                                                                                    kArgs.useConsistentScale,
                                                                                    patch);
    }
    else
    {
        fsimInvertedFiltered = compNCCby3DptsYK<invertAndFilter>(rcDeviceCamParams,
                                                                 tcDeviceCamParams,
                                                                 rcMipmapImage_tex_wrapper,
                                                                 tcMipmapImage_tex_wrapper,
                                                                 kArgs.rcRefineLevelWidth,
                                                                 kArgs.rcRefineLevelHeight,
                                                                 kArgs.tcRefineLevelWidth,
                                                                 kArgs.tcRefineLevelHeight,
                                                                 kArgs.rcMipmapLevel,
                                                                 kArgs.wsh,
                                                                 kArgs.invGammaC,
                                                                 kArgs.invGammaP,
                                                                 kArgs.useConsistentScale,
                                                                 patch);
    }

    if(fsimInvertedFiltered == INFINITY) // invalid similarity
    {
        // do nothing
        return;
    }

    // get output similarity pointer
    device TSimRefine* outSimPtr = get3DBufferAt(inout_volSim_d, kArgs.inout_volSim_s, kArgs.inout_volSim_p, vx, vy, vz);

    // add the output similarity value
#ifdef TSIM_REFINE_USE_HALF
    // note: using built-in half addition can give bad results on some gpus
    //*outSimPtr = __hadd(*outSimPtr, TSimRefine(fsimInvertedFiltered));
    //*outSimPtr = __hadd(*outSimPtr, __float2half(fsimInvertedFiltered));
    *outSimPtr = half(float(*outSimPtr) + fsimInvertedFiltered); // perform the addition in float
#else
    *outSimPtr += TSimRefine(fsimInvertedFiltered);
#endif
}

kernel void volume_retrieveBestDepth_kernel(device float2* out_sgmDepthThicknessMap_d [[buffer(0)]],
                                            device float2* out_sgmDepthSimMap_d [[buffer(1)]],
                                            device const float* in_depths_d [[buffer(2)]],
                                            device const TSim* in_volSim_d [[buffer(3)]],
                                            constant const DeviceCameraParams* constantCameraParametersArray_d [[buffer(4)]],
                                            constant const volume_retrieveBestDepth_kernel_PC& kArgs [[buffer(30)]],
                                            const uint3 gid [[thread_position_in_grid]])
{
    const unsigned int vx = gid.x;
    const unsigned int vy = gid.y;

    if(vx >= kArgs.roi.width() || vy >= kArgs.roi.height())
        return;

    // R camera parameters
    constant const DeviceCameraParams& rcDeviceCamParams = constantCameraParametersArray_d[kArgs.rcDeviceCameraParamsId];

    // corresponding image coordinates
    const float2 pix{float((kArgs.roi.x.begin + vx) * kArgs.scaleStep), float((kArgs.roi.y.begin + vy) * kArgs.scaleStep)};

    // corresponding output depth/thickness pointer
    device float2* out_bestDepthThicknessPtr = get2DBufferAt(out_sgmDepthThicknessMap_d, kArgs.out_sgmDepthThicknessMap_p, vx, vy);

    // corresponding output depth/sim pointer or nullptr
    device float2* out_bestDepthSimPtr = (!kArgs.useDepthSimMap) ? nullptr : get2DBufferAt(out_sgmDepthSimMap_d, kArgs.out_sgmDepthSimMap_p, vx, vy);

    // find the best depth plane index for the current pixel
    // the best depth plane has the best similarity value
    // - best possible similarity value is 0
    // - worst possible similarity value is 254
    // - invalid similarity value is 255
    float bestSim = 255.f;
    int bestZIdx = -1;

    for(unsigned int vz = kArgs.depthRange.begin; vz < kArgs.depthRange.end; ++vz)
    {
      const float simAtZ = *get3DBufferAt(in_volSim_d, kArgs.in_volSim_s, kArgs.in_volSim_p, vx, vy, vz);

      if(simAtZ < bestSim)
      {
        bestSim = simAtZ;
        bestZIdx = vz;
      }
    }

    // filtering out invalid values and values with a too bad score (above the user maximum similarity threshold)
    // note: this helps to reduce following calculations and also the storage volume of the depth maps.
    if((bestZIdx == -1) || (bestSim > kArgs.maxSimilarity))
    {
        out_bestDepthThicknessPtr->x = -1.f; // invalid depth
        out_bestDepthThicknessPtr->y = -1.f; // invalid thickness

        if(out_bestDepthSimPtr != nullptr)
        {
            out_bestDepthSimPtr->x = -1.f; // invalid depth
            out_bestDepthSimPtr->y =  1.f; // worst similarity value
        }
        return;
    }

    // find best depth plane previous and next indexes
    const int bestZIdx_m1 = max(0, bestZIdx - 1);           // best depth plane previous index
    const int bestZIdx_p1 = min(kArgs.volDimZ - 1, bestZIdx + 1); // best depth plane next index

    // get best best depth current, previous and next plane depth values
    // note: float3 struct is useful for depth interpolation
    float3 depthPlanes;
    depthPlanes.x = *get2DBufferAt(in_depths_d, kArgs.in_depths_p, bestZIdx_m1, 0);  // best depth previous plane
    depthPlanes.y = *get2DBufferAt(in_depths_d, kArgs.in_depths_p, bestZIdx, 0);     // best depth plane
    depthPlanes.z = *get2DBufferAt(in_depths_d, kArgs.in_depths_p, bestZIdx_p1, 0);  // best depth next plane

    const float bestDepth    = depthPlaneToDepth(rcDeviceCamParams, depthPlanes.y, pix); // best depth
    const float bestDepth_m1 = depthPlaneToDepth(rcDeviceCamParams, depthPlanes.x, pix); // previous best depth
    const float bestDepth_p1 = depthPlaneToDepth(rcDeviceCamParams, depthPlanes.z, pix); // next best depth

#ifdef ALICEVISION_DEPTHMAP_RETRIEVE_BEST_Z_INTERPOLATION
    // with depth/sim interpolation
    // note: disable by default

    float3 sims;
    sims.x = *get3DBufferAt(in_volSim_d, kArgs.in_volSim_s, kArgs.in_volSim_p, vx, vy, bestZIdx_m1);
    sims.y = bestSim;
    sims.z = *get3DBufferAt(in_volSim_d, kArgs.in_volSim_s, kArgs.in_volSim_p, vx, vy, bestZIdx_p1);

    // convert sims from (0, 255) to (-1, +1)
    sims.x = (sims.x / 255.0f) * 2.0f - 1.0f;
    sims.y = (sims.y / 255.0f) * 2.0f - 1.0f;
    sims.z = (sims.z / 255.0f) * 2.0f - 1.0f;

    // interpolation between the 3 depth planes candidates
    const float refinedDepthPlane = refineDepthSubPixel(depthPlanes, sims);

    const float out_bestDepth = depthPlaneToDepth(rcDeviceCamParams, refinedDepthPlane, pix);
    const float out_bestSim = sims.y;
#else
    // without depth interpolation
    const float out_bestDepth = bestDepth;
    const float out_bestSim = (bestSim / 255.0f) * 2.0f - 1.0f; // convert from (0, 255) to (-1, +1)
#endif

    // compute output best depth thickness
    // thickness is the maximum distance between output best depth and previous or next depth
    // thickness can be inflate with thicknessMultFactor
    const float out_bestDepthThickness = max(bestDepth_p1 - out_bestDepth, out_bestDepth - bestDepth_m1) * kArgs.thicknessMultFactor;

    // write output depth/thickness
    out_bestDepthThicknessPtr->x = out_bestDepth;
    out_bestDepthThicknessPtr->y = out_bestDepthThickness;

    if(out_sgmDepthSimMap_d != nullptr)
    {
        // write output depth/sim
        out_bestDepthSimPtr->x = out_bestDepth;
        out_bestDepthSimPtr->y = out_bestSim;
    }
}


kernel void volume_refineBestDepth_kernel(device float2* out_refineDepthSimMap_d [[buffer(0)]],
                                          device const float2* in_sgmDepthPixSizeMap_d [[buffer(1)]],
                                          device const TSimRefine* in_volSim_d [[buffer(2)]],
                                          constant const volume_refineBestDepth_kernel_PC& kArgs [[buffer(30)]],
                                          const uint3 gid [[thread_position_in_grid]])
{
    const unsigned int vx = gid.x;
    const unsigned int vy = gid.y;

    if(vx >= kArgs.roi.width() || vy >= kArgs.roi.height())
        return;

    // corresponding input sgm depth/pixSize (middle depth)
    const float2 in_sgmDepthPixSize = *get2DBufferAt(in_sgmDepthPixSizeMap_d, kArgs.in_sgmDepthPixSizeMap_p, vx, vy);

    // corresponding output depth/sim pointer
    device float2* out_bestDepthSimPtr = get2DBufferAt(out_refineDepthSimMap_d, kArgs.out_refineDepthSimMap_p, vx, vy);

    // sgm depth (middle depth) invalid or masked
    if(in_sgmDepthPixSize.x <= 0.0f)
    {
        out_bestDepthSimPtr->x = in_sgmDepthPixSize.x;  // -1 (invalid) or -2 (masked)
        out_bestDepthSimPtr->y = 1.0f;                  // similarity between (-1, +1)
        return;
    }

    // find best z sample per pixel
    float bestSampleSim = 0.f;      // all sample sim <= 0.f
    int bestSampleOffsetIndex = 0;  // default is middle depth (SGM)

    // sliding gaussian window
    for(int sample = -kArgs.halfNbSamples; sample <= kArgs.halfNbSamples; ++sample)
    {
        float sampleSim = 0.f;

        for(int vz = 0; vz < kArgs.volDimZ; ++vz)
        {
            const int rz = (vz - kArgs.halfNbDepths);    // relative depth index offset
            const int zs = rz * kArgs.samplesPerPixSize; // relative sample offset

            // get the inverted similarity sum value
            // best value is the HIGHEST
            // worst value is 0
            const float invSimSum = *get3DBufferAt(in_volSim_d, kArgs.in_volSim_s, kArgs.in_volSim_p, vx, vy, vz);

            // reverse the inverted similarity sum value
            // best value is the LOWEST
            // worst value is 0
            const float simSum = -invSimSum;

            // apply gaussian
            // see: https://www.desmos.com/calculator/ribalnoawq
            sampleSim += simSum * expf(-((zs - sample) * (zs - sample)) / kArgs.twoTimesSigmaPowerTwo);
        }

        if(sampleSim < bestSampleSim)
        {
            bestSampleOffsetIndex = sample;
            bestSampleSim = sampleSim;
        }
    }

    // compute sample size
    const float sampleSize = in_sgmDepthPixSize.y / kArgs.samplesPerPixSize; // input sgm pixSize / samplesPerPixSize

    // compute sample size offset from z center
    const float sampleSizeOffset = bestSampleOffsetIndex * sampleSize;

    // compute best depth
    // input sgm depth (middle depth) + sample size offset from z center
    const float bestDepth = in_sgmDepthPixSize.x + sampleSizeOffset;

    // write output best depth/sim
    out_bestDepthSimPtr->x = bestDepth;
    out_bestDepthSimPtr->y = bestSampleSim;
}

kernel void volume_initVolumeYSlice_kernel(device TSim* volume_d [[buffer(0)]], constant const volume_initVolumeYSlice_kernel_PC& kArgs [[buffer(30)]], const uint3 gid [[thread_position_in_grid]])
{
    const int x = gid.x;
    const int z = gid.y;

    int3 v;
    int temp[3];
    temp[kArgs.axisT.x] = x;
    temp[kArgs.axisT.y] = kArgs.y;
    temp[kArgs.axisT.z] = z;
    v = make_int3(temp[0], temp[1], temp[2]);
    
    int volDim_temp[3];
    volDim_temp[0] = kArgs.volDim.x;
    volDim_temp[1] = kArgs.volDim.y;
    volDim_temp[2] = kArgs.volDim.z;

    if ((x >= 0) && (x < volDim_temp[kArgs.axisT.x]) && (z >= 0) && (z < volDim_temp[kArgs.axisT.z]))
    {
        device TSim* volume_zyx = get3DBufferAt(volume_d, kArgs.volume_s, kArgs.volume_p, v.x, v.y, v.z);
        *volume_zyx = kArgs.cst;
    }
}

kernel void volume_getVolumeXZSlice_kernel(device TSimAcc* slice_d [[buffer(0)]],
                                           device const TSim* volume_d [[buffer(1)]],
                                           constant const volume_getVolumeXZSlice_kernel_PC& kArgs [[buffer(30)]],
                                           const uint3 gid [[thread_position_in_grid]])
{
    const int x = gid.x;
    const int z = gid.y;

    int3 v;
    int temp[3];
    temp[kArgs.axisT.x] = x;
    temp[kArgs.axisT.y] = kArgs.y;
    temp[kArgs.axisT.z] = z;
    v = make_int3(temp[0], temp[1], temp[2]);
    
    int volDim_temp[3];
    volDim_temp[0] = kArgs.volDim.x;
    volDim_temp[1] = kArgs.volDim.y;
    volDim_temp[2] = kArgs.volDim.z;

    if (x >= volDim_temp[kArgs.axisT.x] || z >= volDim_temp[kArgs.axisT.z])
      return;

    device const TSim* volume_xyz = get3DBufferAt(volume_d, kArgs.volume_s, kArgs.volume_p, v);
    device TSimAcc* slice_xz = get2DBufferAt(slice_d, kArgs.slice_p, x, z);
    *slice_xz = (TSimAcc)(*volume_xyz);
}

kernel void volume_computeBestZInSlice_kernel(device TSimAcc* xzSlice_d [[buffer(0)]], device TSimAcc* ySliceBestInColCst_d [[buffer(1)]], constant const volume_computeBestZInSlice_kernel_PC& kArgs [[buffer(30)]], const uint3 gid [[thread_position_in_grid]])
{
    const int x = gid.x;

    if(x >= kArgs.volDimX)
        return;

    TSimAcc bestCst = *get2DBufferAt(xzSlice_d, kArgs.xzSlice_p, x, 0);

    for(int z = 1; z < kArgs.volDimZ; ++z)
    {
        const TSimAcc cst = *get2DBufferAt(xzSlice_d, kArgs.xzSlice_p, x, z);
        bestCst = cst < bestCst ? cst : bestCst;  // min(cst, bestCst);
    }
    ySliceBestInColCst_d[x] = bestCst;
}

/**
 * @param[inout] xySliceForZ input similarity plane
 * @param[in] xySliceForZM1
 * @param[in] xSliceBestInColCst
 * @param[out] volSimT output similarity volume
 */
kernel void volume_agregateCostVolumeAtXinSlices_kernel(device MTLRGBA* rcMipmapImage_tex [[buffer(0)]], // As per CUDA implementation, use Bilinear filtering and normalized coordinates
                                                        device TSimAcc* xzSliceForY_d [[buffer(1)]],
                                                        device const TSimAcc* xzSliceForYm1_d [[buffer(2)]],
                                                        device const TSimAcc* bestSimInYm1_d [[buffer(3)]],
                                                        device TSim* volAgr_d [[buffer(4)]],
                                                        constant const volume_agregateCostVolumeAtXinSlices_kernel_PC& kArgs [[buffer(30)]],
                                                        const uint3 gid [[thread_position_in_grid]])
{
    const int x = gid.x;
    const int z = gid.y;

    int3 v;
    int temp[3];
    temp[kArgs.axisT.x] = x;
    temp[kArgs.axisT.y] = kArgs.y;
    temp[kArgs.axisT.z] = z;
    v = make_int3(temp[0], temp[1], temp[2]);
    
    int volDim_temp[3];
    volDim_temp[0] = kArgs.volDim.x;
    volDim_temp[1] = kArgs.volDim.y;
    volDim_temp[2] = kArgs.volDim.z;

    if (x >= volDim_temp[kArgs.axisT.x] || z >= kArgs.volDim.z)
        return;

    // find texture offset
    const int beginX = (kArgs.axisT.x == 0) ? kArgs.roi.x.begin : kArgs.roi.y.begin;
    const int beginY = (kArgs.axisT.x == 0) ? kArgs.roi.y.begin : kArgs.roi.x.begin;

    device TSimAcc* sim_xz = get2DBufferAt(xzSliceForY_d, kArgs.xzSliceForY_p, x, z);
    float pathCost = 255.0f;

    if((z >= 1) && (z < kArgs.volDim.z - 1))
    {
        float P2 = 0;

        if(kArgs._P2 < 0)
        {
          // _P2 convention: use negative value to skip the use of deltaC.
          P2 = abs(kArgs._P2);
        }
        else
        {
          const int imX0 = (beginX + v.x) * kArgs.step; // current
          const int imY0 = (beginY + v.y) * kArgs.step;

          const int imX1 = imX0 - kArgs.ySign * kArgs.step * (kArgs.axisT.y == 0); // M1
          const int imY1 = imY0 - kArgs.ySign * kArgs.step * (kArgs.axisT.y == 1);
            
          // Create mipmap texture wrapper
          MTLMipmappedTexture<MTLRGBA> rcMipmapImage_tex_wrapper = MTLMipmappedTexture<MTLRGBA>(rcMipmapImage_tex, kArgs.rcLevel0Width, kArgs.rcLevel0Height, kArgs.rcLevelCount);

          const MTLRGBA gcr0 = rcMipmapImage_tex_wrapper.tex2DLodNorm<FilterMode::Bilinear>((float(imX0) + 0.5f) / float(kArgs.rcSgmLevelWidth), (float(imY0) + 0.5f) / float(kArgs.rcSgmLevelHeight), kArgs.rcMipmapLevel);
          const MTLRGBA gcr1 = rcMipmapImage_tex_wrapper.tex2DLodNorm<FilterMode::Bilinear>((float(imX1) + 0.5f) / float(kArgs.rcSgmLevelWidth), (float(imY1) + 0.5f) / float(kArgs.rcSgmLevelHeight), kArgs.rcMipmapLevel);
        
          float4 gcr0_f(gcr0.x, gcr0.y, gcr0.z, gcr0.w);
          float4 gcr1_f(gcr1.x, gcr1.y, gcr1.z, gcr1.w);
            
          const float deltaC = euclideanDist3(gcr0_f, gcr1_f);

          // sigmoid f(x) = i + (a - i) * (1 / ( 1 + e^(10 * (x - P2) / w)))
          // see: https://www.desmos.com/calculator/1qvampwbyx
          // best values found from tests: i = 80, a = 255, w = 80, P2 = 100
          // historical values: i = 15, a = 255, w = 80, P2 = 20
          P2 = sigmoid(80.f, 255.f, 80.f, kArgs._P2, deltaC);
        }

        const TSimAcc bestCostInColM1 = bestSimInYm1_d[x];
        const TSimAcc pathCostMDM1 = *get2DBufferAt(xzSliceForYm1_d, kArgs.xzSliceForYm1_p, x, z - 1); // M1: minus 1 over depths
        const TSimAcc pathCostMD   = *get2DBufferAt(xzSliceForYm1_d, kArgs.xzSliceForYm1_p, x, z);
        const TSimAcc pathCostMDP1 = *get2DBufferAt(xzSliceForYm1_d, kArgs.xzSliceForYm1_p, x, z + 1); // P1: plus 1 over depths
        const float minCost = multi_fminf(pathCostMD, pathCostMDM1 + kArgs.P1, pathCostMDP1 + kArgs.P1, bestCostInColM1 + P2);

        // if 'pathCostMD' is the minimal value of the depth
        pathCost = (*sim_xz) + minCost - bestCostInColM1;
    }

    // fill the current slice with the new similarity score
    *sim_xz = TSimAcc(pathCost);

#ifndef TSIM_USE_FLOAT
    // clamp if TSim = uchar (TSimAcc = unsigned int)
    pathCost = fmin(255.0f, fmax(0.0f, pathCost));
#endif

    // aggregate into the final output
    device TSim* volume_xyz = get3DBufferAt(volAgr_d, kArgs.volAgr_s, kArgs.volAgr_p, v.x, v.y, v.z);
    const float val = (float(*volume_xyz) * float(kArgs.filteringIndex) + pathCost) / float(kArgs.filteringIndex + 1);
    *volume_xyz = TSim(val);
}

} // namespace depthMap
} // namespace aliceVision

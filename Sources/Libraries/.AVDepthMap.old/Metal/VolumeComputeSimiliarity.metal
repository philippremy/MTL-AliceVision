//
//  VolumeComputeSimiliarity.metal
//  AVGPUServices
//
//  Created by Philipp Remy on 14.07.25.
//

#include <metal_stdlib>
using namespace metal;

#include <AVDepthMap/Metal/Types.hpp>
#include <AVDepthMap/Metal/DeviceCameraParams.hpp>
#include <AVDepthMap/Metal/DevicePatchPattern.hpp>
#include <AVDepthMap/Metal/VolumeComputeSimiliarity_PushConstants.hpp>

#include <AVDepthMap/Metal/Patch.metal>

namespace aliceVision {
namespace depthMap {

inline void volume_computePatch(thread Patch& patch,
                                constant DeviceCameraParams& rcDeviceCamParams,
                                constant DeviceCameraParams& tcDeviceCamParams,
                                const float fpPlaneDepth,
                                thread const float2& pix)
{
    patch.p = get3DPointForPixelAndFrontoParellePlaneRC(rcDeviceCamParams, pix, fpPlaneDepth);
    patch.d = computePixSize(rcDeviceCamParams, patch.p);
    computeRotCSEpip(patch, rcDeviceCamParams, tcDeviceCamParams);
}

[[kernel]]
kernel void VolumeComputeSimiliarity(constant VolumeComputeSimiliarity_PushConstants& pushConstants [[buffer(0)]],
                                     texture3d<TSim, access::read_write> out_volume1st_d [[texture(0)]],
                                     texture3d<TSim, access::read_write> out_volume2nd_d [[texture(1)]],
                                     texture2d<float, access::read> in_depths_d [[texture(2)]],
                                     texture2d<MTLPixelBaseType, access::sample> rcMipmapImage_tex [[texture(3)]],
                                     texture2d<MTLPixelBaseType, access::sample> tcMipmapImage_tex [[texture(4)]],
                                     sampler rcMipmapImage_samp [[sampler(0)]],
                                     sampler tcMipmapImage_samp [[sampler(1)]],
                                     constant DeviceCameraParams* constantCameraParametersArray_d [[buffer(1)]],
                                     constant DevicePatchPattern* constantPatchPattern_d [[buffer(2)]],
                                     uint3 gid [[thread_position_in_grid]])
{
    
    uint roiX = gid.x;
    uint roiY = gid.y;
    uint roiZ = gid.z;
    
    if(roiX >= pushConstants.roi.width() || roiY >= pushConstants.roi.height()) // no need to check roiZ
        return;
    
    // R and T camera parameters
    constant DeviceCameraParams& rcDeviceCamParams = constantCameraParametersArray_d[pushConstants.rcDeviceCameraParamsId];
    constant DeviceCameraParams& tcDeviceCamParams = constantCameraParametersArray_d[pushConstants.tcDeviceCameraParamsId];
    
    // corresponding volume coordinates
    const unsigned int vx = roiX;
    const unsigned int vy = roiY;
    const unsigned int vz = pushConstants.depthRange.begin + roiZ;
    
    // corresponding image coordinates
    const float x = float(pushConstants.roi.x.begin + vx) * float(pushConstants.stepXY);
    const float y = float(pushConstants.roi.y.begin + vy) * float(pushConstants.stepXY);
    
    // corresponding depth plane
    const float depthPlane = in_depths_d.read(uint2(vz, 0)).x;

    // compute patch
    Patch patch;
    volume_computePatch(patch, rcDeviceCamParams, tcDeviceCamParams, depthPlane, float2(x, y));
    
    // we do not need positive and filtered similarity values
    constexpr bool invertAndFilter = false;

    float fsim = INFINITY;
    
    // compute patch similarity
    if(pushConstants.useCustomPatchPattern)
    {
        fsim = compNCCby3DptsYK_customPatchPattern<invertAndFilter>(rcDeviceCamParams,
                                                                    tcDeviceCamParams,
                                                                    *constantPatchPattern_d,
                                                                    rcMipmapImage_tex,
                                                                    tcMipmapImage_tex,
                                                                    rcMipmapImage_samp,
                                                                    tcMipmapImage_samp,
                                                                    pushConstants.rcSgmLevelWidth,
                                                                    pushConstants.rcSgmLevelHeight,
                                                                    pushConstants.tcSgmLevelWidth,
                                                                    pushConstants.tcSgmLevelHeight,
                                                                    pushConstants.rcMipmapLevel,
                                                                    pushConstants.invGammaC,
                                                                    pushConstants.invGammaP,
                                                                    pushConstants.useConsistentScale,
                                                                    patch);
    }
    else
    {
        fsim = compNCCby3DptsYK<invertAndFilter>(rcDeviceCamParams,
                                                 tcDeviceCamParams,
                                                 rcMipmapImage_tex,
                                                 tcMipmapImage_tex,
                                                 rcMipmapImage_samp,
                                                 tcMipmapImage_samp,
                                                 pushConstants.rcSgmLevelWidth,
                                                 pushConstants.rcSgmLevelHeight,
                                                 pushConstants.tcSgmLevelWidth,
                                                 pushConstants.tcSgmLevelHeight,
                                                 pushConstants.rcMipmapLevel,
                                                 pushConstants.wsh,
                                                 pushConstants.invGammaC,
                                                 pushConstants.invGammaP,
                                                 pushConstants.useConsistentScale,
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
      fsim = min(1.0f, max(0.0f, fsim));
#endif
      // convert from (0, 1) to (0, 254)
      // needed to store in the volume in uchar
      // 255 is reserved for the similarity initialization, i.e. undefined values
      fsim *= 254.0f;
    }
    
    TSim fsim_1st = out_volume1st_d.read(uint3(vx, vy, vz)).x;
    TSim fsim_2nd = out_volume2nd_d.read(uint3(vx, vy, vz)).x;
    
    if(fsim < fsim_1st)
    {
        fsim_2nd = fsim_1st;
        out_volume2nd_d.write(fsim_1st, uint3(vx, vy, vz));
        fsim_1st = TSim(fsim);
        out_volume1st_d.write(fsim, uint3(vx, vy, vz));
    }
    else if(fsim < fsim_2nd)
    {
        fsim_2nd = TSim(fsim);
        out_volume2nd_d.write(fsim, uint3(vx, vy, vz));
    }
    
}

}
}
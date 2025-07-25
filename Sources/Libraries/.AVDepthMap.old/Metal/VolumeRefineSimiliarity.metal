//
//  VolumeRefineSimilarity.metal
//  AVGPUServices
//
//  Created by Philipp Remy on 16.07.25.
//

#include <metal_stdlib>
using namespace metal;

#include <AVDepthMap/Metal/Types.hpp>
#include <AVDepthMap/Metal/DeviceCameraParams.hpp>
#include <AVDepthMap/Metal/DevicePatchPattern.hpp>
#include <AVDepthMap/Metal/VolumeRefineSimiliarity_PushConstants.hpp>

#include <AVDepthMap/Metal/Patch.metal>

namespace aliceVision {
namespace depthMap {

inline void move3DPointByRcPixSize(thread float3& p,
                                   constant DeviceCameraParams& rcDeviceCamParams,
                                   const float rcPixSize)
{
    float3 rpv = p - rcDeviceCamParams.C;
    rpv = normalize(rpv);
    p = p + rpv * rcPixSize;
}

[[kernel]]
kernel void VolumeRefineSimiliarity(constant VolumeRefineSimiliarity_PushConstants& pushConstants [[buffer(0)]],
                                    texture3d<TSimRefine, access::read_write> inout_volSim_d [[texture(0)]],
                                    texture2d<float, access::read> in_sgmDepthPixSizeMap_d [[texture(1)]],
                                    texture2d<float, access::read> in_sgmNormalMap_d [[texture(2)]],
                                    texture2d<MTLPixelBaseType, access::sample> rcMipmapImage_tex [[texture(3)]],
                                    texture2d<MTLPixelBaseType, access::sample> tcMipmapImage_tex [[texture(4)]],
                                    sampler rcMipmapImage_samp [[sampler(0)]],
                                    sampler tcMipmapImage_samp [[sampler(1)]],
                                    constant DeviceCameraParams* constantCameraParametersArray_d [[buffer(1)]],
                                    constant DevicePatchPattern* constantPatchPattern_d [[buffer(2)]],
                                    uint3 gid [[thread_position_in_grid]],
                                    uint3 bit [[threadgroup_position_in_grid]])
{
 
    const unsigned int roiX = gid.x;
    const unsigned int roiY = gid.y;
    const unsigned int roiZ = bit.z;

    if(roiX >= pushConstants.roi.width() || roiY >= pushConstants.roi.height()) // no need to check roiZ
        return;

    // R and T camera parameters
    constant DeviceCameraParams& rcDeviceCamParams = constantCameraParametersArray_d[pushConstants.rcDeviceCameraParamsId];
    constant DeviceCameraParams& tcDeviceCamParams = constantCameraParametersArray_d[pushConstants.tcDeviceCameraParamsId];

    // corresponding volume and depth/sim map coordinates
    const unsigned int vx = roiX;
    const unsigned int vy = roiY;
    const unsigned int vz = pushConstants.depthRange.begin + roiZ;

    // corresponding image coordinates
    const float x = float(pushConstants.roi.x.begin + vx) * float(pushConstants.stepXY);
    const float y = float(pushConstants.roi.y.begin + vy) * float(pushConstants.stepXY);

    // corresponding input sgm depth/pixSize (middle depth)
    const float2 in_sgmDepthPixSize = in_sgmDepthPixSizeMap_d.read(uint2(vx, vy)).xy;

    // sgm depth (middle depth) invalid or masked
    if(in_sgmDepthPixSize.x <= 0.0f)
        return;

    // initialize rc 3d point at sgm depth (middle depth)
    float3 p = get3DPointForPixelAndDepthFromRC(rcDeviceCamParams, float2(x, y), in_sgmDepthPixSize.x);
    
    // compute relative depth index offset from z center
    const int relativeDepthIndexOffset = vz - ((pushConstants.volDimZ - 1) / 2);
    
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
      v1 = normalize(v1);
      v2 = normalize(v2);

      // y has to be orthogonal to the epipolar plane
      // n has to be on the epipolar plane
      // x has to be on the epipolar plane

      patch.y = cross(v1, v2);
      patch.y = normalize(patch.y);

      if(pushConstants.hasNormalMap) // initialize patch normal from input normal map
      {
          patch.n = in_sgmNormalMap_d.read(uint2(vx, vy)).xyz;
      } 
      else // initialize patch normal from v1 & v2
      {
          patch.n = (v1 + v2) / 2.0f;
          patch.n = normalize(patch.n);
      }

      patch.x = cross(patch.y, patch.n);
      patch.x = normalize(patch.x);
    }
    
    // we need positive and filtered similarity values
    constexpr bool invertAndFilter = true;

    float fsimInvertedFiltered = INFINITY;
    
    // compute similarity
    if(pushConstants.useCustomPatchPattern)
    {
        fsimInvertedFiltered = compNCCby3DptsYK_customPatchPattern<invertAndFilter>(rcDeviceCamParams,
                                                                                    tcDeviceCamParams,
                                                                                    *constantPatchPattern_d,
                                                                                    rcMipmapImage_tex,
                                                                                    tcMipmapImage_tex,
                                                                                    rcMipmapImage_samp,
                                                                                    tcMipmapImage_samp,
                                                                                    pushConstants.rcRefineLevelWidth,
                                                                                    pushConstants.rcRefineLevelHeight,
                                                                                    pushConstants.tcRefineLevelWidth,
                                                                                    pushConstants.tcRefineLevelHeight,
                                                                                    pushConstants.rcMipmapLevel,
                                                                                    pushConstants.invGammaC,
                                                                                    pushConstants.invGammaP,
                                                                                    pushConstants.useConsistentScale,
                                                                                    patch);
    }
    else
    {
        fsimInvertedFiltered = compNCCby3DptsYK<invertAndFilter>(rcDeviceCamParams,
                                                                 tcDeviceCamParams,
                                                                 rcMipmapImage_tex,
                                                                 tcMipmapImage_tex,
                                                                 rcMipmapImage_samp,
                                                                 tcMipmapImage_samp,
                                                                 pushConstants.rcRefineLevelWidth,
                                                                 pushConstants.rcRefineLevelHeight,
                                                                 pushConstants.tcRefineLevelWidth,
                                                                 pushConstants.tcRefineLevelHeight,
                                                                 pushConstants.rcMipmapLevel,
                                                                 pushConstants.wsh,
                                                                 pushConstants.invGammaC,
                                                                 pushConstants.invGammaP,
                                                                 pushConstants.useConsistentScale,
                                                                 patch);
    }

    if(fsimInvertedFiltered == INFINITY) // invalid similarity
    {
        // do nothing
        return;
    }
    
    // get output similarity pointer
    vec<TSimRefine, 4> outSimRead = inout_volSim_d.read(uint3(vx, vy, vz));
    
    // add the output similarity value
    outSimRead.x += TSimRefine(fsimInvertedFiltered);
    
    inout_volSim_d.write(outSimRead, uint3(vx, vy, vz));
    
}

}
}
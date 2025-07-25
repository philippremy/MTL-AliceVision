//
//  DepthMapSimMapComputeNormal.metal
//  AVGPUServices
//
//  Created by Philipp Remy on 14.07.25.
//

#include <metal_stdlib>
using namespace metal;

#include <AVDepthMap/Metal/Patch.metal>
#include <AVDepthMap/Metal/Eig33.metal>

#include <AVDepthMap/Metal/Types.hpp>
#include <AVDepthMap/Metal/DepthSimMapComputeNormal_PushConstants.hpp>
#include <AVDepthMap/Metal/DeviceCameraParams.hpp>

namespace aliceVision {
namespace depthMap {

static inline float orientedPointPlaneDistanceNormalizedNormal(thread const float3& point,
                                                               thread const float3& planePoint,
                                                               thread const float3& planeNormalNormalized)
{
    return (dot(point, planeNormalNormalized) - dot(planePoint, planeNormalNormalized));
}

[[kernel]]
kernel void DepthSimMapComputeNormal(constant DepthSimMapComputeNormal_PushConstants& pushConstants [[buffer(0)]],
                                     texture2d<float, access::read_write> out_normalMap_d [[texture(0)]],
                                     texture2d<float, access::read> in_depthSimMap_d [[texture(1)]],
                                     constant DeviceCameraParams* constantCameraParametersArray_d [[buffer(1)]],
                                     uint3 gid [[thread_position_in_grid]])
{
    
    const uint roiX = gid.x;
    const uint roiY = gid.y;
    
    if(roiX >= pushConstants.roi.width() || roiY >= pushConstants.roi.height())
        return;
    
    // R camera parameters
    constant DeviceCameraParams& rcDeviceCamParams = constantCameraParametersArray_d[pushConstants.rcDeviceCameraParamsId];
    
    // corresponding image coordinates
    const unsigned int x = (pushConstants.roi.x.begin + roiX) * (unsigned int)(pushConstants.stepXY);
    const unsigned int y = (pushConstants.roi.y.begin + roiY) * (unsigned int)(pushConstants.stepXY);
    
    // corresponding input depth
    const float in_depth = in_depthSimMap_d.read(uint2(roiX, roiY)).x; // use only depth

    // corresponding output normal
    float4 out_normal = out_normalMap_d.read(uint2(roiX, roiY));

    // no depth
    if(in_depth <= 0.0f)
    {
        out_normalMap_d.write(float4(-1.f, -1.f, -1.f, -1.f), uint2(roiX, roiY));
        return;
    }

    const float3 p = get3DPointForPixelAndDepthFromRC(rcDeviceCamParams, float2(float(x), float(y)), in_depth);
    const float pixSize = length(p - get3DPointForPixelAndDepthFromRC(rcDeviceCamParams, float2(float(x + 1), float(y)), in_depth));
    
    mtl_stat3d s3d = mtl_stat3d();
    
#pragma unroll
    for(int yp = -pushConstants.TWsh; yp <= pushConstants.TWsh; ++yp)
    {
        const int roiYp = int(roiY) + yp;
        if(roiYp < 0)
            continue;

#pragma unroll
        for(int xp = -pushConstants.TWsh; xp <= pushConstants.TWsh; ++xp)
        {
            const int roiXp = int(roiX) + xp;
            if(roiXp < 0)
                continue;

            const float depthP = in_depthSimMap_d.read(uint2(roiXp, roiYp)).x; // use only depth

            if((depthP > 0.0f) && (fabs(depthP - in_depth) < 30.0f * pixSize))
            {
                const float w = 1.0f;
                const float2 pixP = float2(float(int(x) + xp), float(int(y) + yp));
                const float3 pP = get3DPointForPixelAndDepthFromRC(rcDeviceCamParams, pixP, depthP);
                s3d.update(pP, w);
            }
        }
    }
    
    float3 pp = p;
    float3 nn = float3(-1.f, -1.f, -1.f);

    if(!s3d.computePlaneByPCA(pp, nn))
    {
        out_normalMap_d.write(float4(-1.f, -1.f, -1.f, out_normal.w), uint2(roiX, roiY));
        return;
    }

    float3 nc = rcDeviceCamParams.C - p;
    nc = normalize(nc);

    if(orientedPointPlaneDistanceNormalizedNormal(pp + nn, pp, nc) < 0.0f)
    {
        nn.x = -nn.x;
        nn.y = -nn.y;
        nn.z = -nn.z;
    }
    
    out_normalMap_d.write(float4(nn, out_normal.w), uint2(roiX, roiY));
    
}

}
}
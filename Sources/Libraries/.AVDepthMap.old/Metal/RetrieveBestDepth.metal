//
//  RetrieveBestDepth.metal
//  AVGPUServices
//
//  Created by Philipp Remy on 16.07.25.
//

#include <metal_stdlib>
using namespace metal;

#include <AVDepthMap/Metal/Types.hpp>
#include <AVDepthMap/Metal/DeviceCameraParams.hpp>
#include <AVDepthMap/Metal/RetrieveBestDepth_PushConstants.hpp>

#include <AVDepthMap/Metal/Matrix.metal>
#include <AVDepthMap/Metal/Patch.metal>

namespace aliceVision {
namespace depthMap {

inline float depthPlaneToDepth(constant DeviceCameraParams& deviceCamParams,
                               const float fpPlaneDepth,
                               thread const float2& pix)
{
    const float3 planep = deviceCamParams.C + deviceCamParams.ZVect * fpPlaneDepth;
    float3 v = M3x3mulV2(deviceCamParams.iP, pix);
    normalize(v);
    float3 p = linePlaneIntersect(deviceCamParams.C, v, planep, deviceCamParams.ZVect);
    return length(deviceCamParams.C - p);
}

[[kernel]]
kernel void RetrieveBestDepth(constant RetrieveBestDepth_PushConstants& pushConstants [[buffer(0)]],
                              texture2d<float, access::read_write> out_sgmDepthThicknessMap_d [[texture(0)]],
                              texture2d<float, access::read_write> out_sgmDepthSimMap_d [[texture(1)]],
                              texture2d<float, access::read> in_depths_d [[texture(2)]],
                              texture3d<TSim, access::read> in_volSim_d [[texture(3)]],
                              constant DeviceCameraParams* constantCameraParametersArray_d [[buffer(1)]],
                              uint3 gid [[thread_position_in_grid]])
{
    
    const unsigned int vx = gid.x;
    const unsigned int vy = gid.y;

    if(vx >= pushConstants.roi.width() || vy >= pushConstants.roi.height())
        return;
    
    // R camera parameters
    constant DeviceCameraParams& rcDeviceCamParams = constantCameraParametersArray_d[pushConstants.rcDeviceCameraParamsId];

    // corresponding image coordinates
    const float2 pix{float((pushConstants.roi.x.begin + vx) * pushConstants.scaleStep), float((pushConstants.roi.y.begin + vy) * pushConstants.scaleStep)};
    
    // corresponding output depth/thickness pointer
    float4 out_bestDepthThicknessRead = out_sgmDepthThicknessMap_d.read(uint2(vx, vy));

    // corresponding output depth/sim pointer or nullptr
    float4 out_bestDepthSimReadMaybe;
    thread float4* out_bestDepthSimRead = nullptr;
    if(pushConstants.hasDepthSimTex) {
        out_bestDepthSimReadMaybe = out_sgmDepthSimMap_d.read(uint2(vx, vy));
        out_bestDepthSimRead = &out_bestDepthSimReadMaybe;
    }

    // find the best depth plane index for the current pixel
    // the best depth plane has the best similarity value
    // - best possible similarity value is 0
    // - worst possible similarity value is 254
    // - invalid similarity value is 255
    float bestSim = 255.f;
    int bestZIdx = -1;
    
    for(uint vz = pushConstants.depthRange.begin; vz < pushConstants.depthRange.end; ++vz)
    {
        const float simAtZ = in_volSim_d.read(uint3(vx, vy, vz)).x;

        if(simAtZ < bestSim)
        {
            bestSim = simAtZ;
            bestZIdx = vz;
        }
    }
    
    // filtering out invalid values and values with a too bad score (above the user maximum similarity threshold)
    // note: this helps to reduce following calculations and also the storage volume of the depth maps.
    if((bestZIdx == -1) || (bestSim > pushConstants.maxSimilarity))
    {
        out_bestDepthThicknessRead.x = -1.f; // invalid depth
        out_bestDepthThicknessRead.y = -1.f; // invalid thickness

        if(out_bestDepthSimRead != nullptr)
        {
            out_bestDepthSimRead->x = -1.f; // invalid depth
            out_bestDepthSimRead->y =  1.f; // worst similarity value
            out_sgmDepthSimMap_d.write(*out_bestDepthSimRead, uint2(vx, vy));
        }
        
        out_sgmDepthThicknessMap_d.write(out_bestDepthThicknessRead, uint2(vx, vy));
        
        return;
    }
    
    // find best depth plane previous and next indexes
    const int bestZIdx_m1 = max(0, bestZIdx - 1);           // best depth plane previous index
    const int bestZIdx_p1 = min(pushConstants.volDimZ - 1, bestZIdx + 1); // best depth plane next index

    // get best best depth current, previous and next plane depth values
    // note: float3 struct is useful for depth interpolation
    float3 depthPlanes;
    depthPlanes.x = in_depths_d.read(uint2(bestZIdx_m1, 0)).x;  // best depth previous plane
    depthPlanes.y = in_depths_d.read(uint2(bestZIdx, 0)).x;     // best depth plane
    depthPlanes.z = in_depths_d.read(uint2(bestZIdx_p1, 0)).x;  // best depth next plane

    const float bestDepth    = depthPlaneToDepth(rcDeviceCamParams, depthPlanes.y, pix); // best depth
    const float bestDepth_m1 = depthPlaneToDepth(rcDeviceCamParams, depthPlanes.x, pix); // previous best depth
    const float bestDepth_p1 = depthPlaneToDepth(rcDeviceCamParams, depthPlanes.z, pix); // next best depth
    
#ifdef ALICEVISION_DEPTHMAP_RETRIEVE_BEST_Z_INTERPOLATION
    // with depth/sim interpolation
    // note: disable by default

    float3 sims;
    sims.x = in_volSim_d.read(uint3(vx, vy, bestZIdx_m1)).x;
    sims.y = bestSim;
    sims.z = in_volSim_d(uint3(vx, vy, bestZIdx_p1));

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
    const float out_bestDepthThickness = max(bestDepth_p1 - out_bestDepth, out_bestDepth - bestDepth_m1) * pushConstants.thicknessMultFactor;

    // write output depth/thickness
    out_bestDepthThicknessRead.x = out_bestDepth;
    out_bestDepthThicknessRead.y = out_bestDepthThickness;

    if(out_bestDepthSimRead != nullptr)
    {
        // write output depth/sim
        out_bestDepthSimRead->x = out_bestDepth;
        out_bestDepthSimRead->y = out_bestSim;
        out_sgmDepthSimMap_d.write(*out_bestDepthSimRead, uint2(vx, vy));
    }
    
    out_sgmDepthThicknessMap_d.write(out_bestDepthThicknessRead, uint2(vx, vy));
    
}

}
}
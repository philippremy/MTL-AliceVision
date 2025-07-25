//
//  OptimizeDepthSimMap.metal
//  AVGPUServices
//
//  Created by Philipp Remy on 16.07.25.
//

#include <metal_stdlib>
using namespace metal;

#include <AVDepthMap/Metal/DeviceCameraParams.hpp>
#include <AVDepthMap/Metal/OptimizeDepthSimMap_PushConstants.hpp>

#include <AVDepthMap/Metal/Matrix.metal>
#include <AVDepthMap/Metal/Patch.metal>

namespace aliceVision {
namespace depthMap {

/**
 * @return (smoothStep, energy)
 */
float2 getCellSmoothStepEnergy(constant DeviceCameraParams& rcDeviceCamParams,
                               const texture2d<float, access::sample> in_depth_tex,
                               const sampler in_depth_samp,
                               thread const float2& cell0,
                               thread const float2& offsetRoi)
{
    float2 out = float2(0.0f, 180.0f);

    // get pixel depth from the depth texture
    // note: we do not use 0.5f offset because in_depth_tex use nearest neighbor interpolation
    const float d0 = in_depth_tex.sample(in_depth_samp, float2(cell0.x, cell0.y)).x;

    // early exit: depth is <= 0
    if(d0 <= 0.0f)
        return out;

    // consider the neighbor pixels
    const float2 cellL = cell0 + float2( 0.f, -1.f); // Left
    const float2 cellR = cell0 + float2( 0.f,  1.f); // Right
    const float2 cellU = cell0 + float2(-1.f,  0.f); // Up
    const float2 cellB = cell0 + float2( 1.f,  0.f); // Bottom

    // get associated depths from depth texture
    // note: we do not use 0.5f offset because in_depth_tex use nearest neighbor interpolation
    const float dL = in_depth_tex.sample(in_depth_samp, float2(cellL.x, cellL.y)).x;
    const float dR = in_depth_tex.sample(in_depth_samp, float2(cellR.x, cellR.y)).x;
    const float dU = in_depth_tex.sample(in_depth_samp, float2(cellU.x, cellU.y)).x;
    const float dB = in_depth_tex.sample(in_depth_samp, float2(cellB.x, cellB.y)).x;

    // get associated 3D points
    const float3 p0 = get3DPointForPixelAndDepthFromRC(rcDeviceCamParams, cell0 + offsetRoi, d0);
    const float3 pL = get3DPointForPixelAndDepthFromRC(rcDeviceCamParams, cellL + offsetRoi, dL);
    const float3 pR = get3DPointForPixelAndDepthFromRC(rcDeviceCamParams, cellR + offsetRoi, dR);
    const float3 pU = get3DPointForPixelAndDepthFromRC(rcDeviceCamParams, cellU + offsetRoi, dU);
    const float3 pB = get3DPointForPixelAndDepthFromRC(rcDeviceCamParams, cellB + offsetRoi, dB);

    // compute the average point based on neighbors (cg)
    float3 cg = float3(0.0f, 0.0f, 0.0f);
    float n = 0.0f;

    if(dL > 0.0f) { cg = cg + pL; n++; }
    if(dR > 0.0f) { cg = cg + pR; n++; }
    if(dU > 0.0f) { cg = cg + pU; n++; }
    if(dB > 0.0f) { cg = cg + pB; n++; }

    // if we have at least one valid depth
    if(n > 1.0f)
    {
        cg = cg / n; // average of x, y, depth
        float3 vcn = rcDeviceCamParams.C - p0;
        normalize(vcn);
        // pS: projection of cg on the line from p0 to camera
        const float3 pS = closestPointToLine3D(cg, p0, vcn);
        // keep the depth difference between pS and p0 as the smoothing step
        out.x = length(rcDeviceCamParams.C - pS) - d0;
    }

    float e = 0.0f;
    n = 0.0f;

    if(dL > 0.0f && dR > 0.0f)
    {
        // large angle between neighbors == flat area => low energy
        // small angle between neighbors == non-flat area => high energy
        e = max(e, (180.0f - angleBetwABandAC(p0, pL, pR)));
        n++;
    }
    if(dU > 0.0f && dB > 0.0f)
    {
        e = max(e, (180.0f - angleBetwABandAC(p0, pU, pB)));
        n++;
    }
    // the higher the energy, the less flat the area
    if(n > 0.0f)
        out.y = e;

    return out;
}

[[kernel]]
kernel void OptimizeDepthSimMap(constant OptimizeDepthSimMap_PushConstants& pushConstants [[buffer(0)]],
                                texture2d<float, access::read_write> out_optimizeDepthSimMap_d [[texture(0)]],
                                texture2d<float, access::read> in_sgmDepthPixSizeMap_d [[texture(1)]],
                                texture2d<float, access::read> in_refineDepthSimMap_d [[texture(2)]],
                                texture2d<float, access::sample> imgVariance_tex [[texture(3)]],
                                sampler imgVariance_samp [[sampler(0)]],
                                texture2d<float, access::sample> depth_tex [[texture(4)]],
                                sampler depth_samp [[sampler(1)]],
                                constant DeviceCameraParams* constantCameraParametersArray_d [[buffer(1)]],
                                uint3 gid [[thread_position_in_grid]])
{
    
    // roi and imgVariance_tex, depth_tex coordinates
    const unsigned int roiX = gid.x;
    const unsigned int roiY = gid.y;

    if(roiX >= pushConstants.roi.width() || roiY >= pushConstants.roi.height())
        return;

    // R camera parameters
    constant DeviceCameraParams& rcDeviceCamParams = constantCameraParametersArray_d[pushConstants.rcDeviceCameraParamsId];
    
    // SGM upscale (rough) depth/pixSize
    const float4 sgmDepthPixSize = in_sgmDepthPixSizeMap_d.read(uint2(roiX, roiY));
    const float sgmDepth = sgmDepthPixSize.x;
    const float sgmPixSize = sgmDepthPixSize.y;

    // refined and fused (fine) depth/sim
    const float4 refineDepthSim = in_refineDepthSimMap_d.read(uint2(roiX, roiY));
    const float refineDepth = refineDepthSim.x;
    const float refineSim = refineDepthSim.y;

    // output optimized depth/sim
    float4 out_optDepthSimRead = out_optimizeDepthSimMap_d.read(uint2(roiX, roiY));
    float2 out_optDepthSim = (pushConstants.iter == 0) ? float2(sgmDepth, refineSim) : out_optDepthSimRead.xy;
    const float depthOpt = out_optDepthSim.x;
    
    if (depthOpt > 0.0f)
    {
        const float2 depthSmoothStepEnergy = getCellSmoothStepEnergy(rcDeviceCamParams, depth_tex, depth_samp, {float(roiX), float(roiY)}, {float(pushConstants.roi.x.begin), float(pushConstants.roi.y.begin)}); // (smoothStep, energy)
        float stepToSmoothDepth = depthSmoothStepEnergy.x;
        stepToSmoothDepth = copysign(min(abs(stepToSmoothDepth), sgmPixSize / 10.0f), stepToSmoothDepth);
        const float depthEnergy = depthSmoothStepEnergy.y; // max angle with neighbors
        float stepToFineDM = refineDepth - depthOpt; // distance to refined/noisy input depth map
        stepToFineDM = copysign(min(abs(stepToFineDM), sgmPixSize / 10.0f), stepToFineDM);

        const float stepToRoughDM = sgmDepth - depthOpt; // distance to smooth/robust input depth map
        const float imgColorVariance = imgVariance_tex.sample(imgVariance_samp, float2(float(roiX), float(roiY))).x; // do not use 0.5f offset because imgVariance_tex use nearest neighbor interpolation
        const float colorVarianceThresholdForSmoothing = 20.0f;
        const float angleThresholdForSmoothing = 30.0f; // 30

        // https://www.desmos.com/calculator/kob9lxs9qf
        const float weightedColorVariance = sigmoid2(5.0f, angleThresholdForSmoothing, 40.0f, colorVarianceThresholdForSmoothing, imgColorVariance);

        // https://www.desmos.com/calculator/jwhpjq6ppj
        const float fineSimWeight = sigmoid(0.0f, 1.0f, 0.7f, -0.7f, refineSim);

        // if geometry variation is bigger than color variation => the fineDM is considered noisy

        // if depthEnergy > weightedColorVariance   => energyLowerThanVarianceWeight=0 => smooth
        // else:                                    => energyLowerThanVarianceWeight=1 => use fineDM
        // weightedColorVariance max value is 30, so if depthEnergy > 30 (which means depthAngle < 150�) energyLowerThanVarianceWeight will be 0
        // https://www.desmos.com/calculator/jzbweilb85
        const float energyLowerThanVarianceWeight = sigmoid(0.0f, 1.0f, 30.0f, weightedColorVariance, depthEnergy); // TODO: 30 => 60

        // https://www.desmos.com/calculator/ilsk7pthvz
        const float closeToRoughWeight = 1.0f - sigmoid(0.0f, 1.0f, 10.0f, 17.0f, abs(stepToRoughDM / sgmPixSize)); // TODO: 10 => 30

        // f(z) = c1 * s1(z_rought - z)^2 + c2 * s2(z-z_fused)^2 + coeff3 * s3*(z-z_smooth)^2

        const float depthOptStep = closeToRoughWeight * stepToRoughDM + // distance to smooth/robust input depth map
                                   (1.0f - closeToRoughWeight) * (energyLowerThanVarianceWeight * fineSimWeight * stepToFineDM + // distance to refined/noisy
                                                                 (1.0f - energyLowerThanVarianceWeight) * stepToSmoothDepth); // max angle in current depthMap

        out_optDepthSim.x = depthOpt + depthOptStep;

        out_optDepthSim.y = (1.0f - closeToRoughWeight) * (energyLowerThanVarianceWeight * fineSimWeight * refineSim + (1.0f - energyLowerThanVarianceWeight) * (depthEnergy / 20.0f));
    }
    
    out_optimizeDepthSimMap_d.write(float4(out_optDepthSim, out_optDepthSimRead.zw), uint2(roiX, roiY));
    
}

}
}
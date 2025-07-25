//
//  AggregateCostVolumeAtXInSlices.metal
//  AVGPUServices
//
//  Created by Philipp Remy on 14.07.25.
//

#include <metal_stdlib>
using namespace metal;

#include <AVDepthMap/Metal/Types.hpp>
#include <AVDepthMap/Metal/AggregateCostVolumeAtXInSlices_PushConstants.hpp>

#include <AVDepthMap/Metal/Color.metal>
#include <AVDepthMap/Metal/Matrix.metal>
#include <AVDepthMap/Metal/Buffer.metal>

namespace aliceVision {
namespace depthMap {

[[kernel]]
kernel void AggregateConstVolumeAtXInSlices(constant AggregateCostVolumeAtXInSlices_PushConstants& pushConstants [[buffer(0)]],
                                            texture2d<MTLPixelBaseType, access::sample> rcMipmapImage_tex [[texture(0)]],
                                            sampler rcMipmapImage_samp [[sampler(0)]],
                                            texture2d<TSimAcc, access::read_write> xzSliceForY_d [[texture(1)]],
                                            texture2d<TSimAcc, access::read> xzSliceForYm1_d [[texture(2)]],
                                            texture2d<TSimAcc, access::read> bestSimInYm1_d [[texture(3)]],
                                            texture3d<TSim, access::read_write> volAgr_d [[texture(4)]],
                                            uint3 gid [[thread_position_in_grid]])
{
    
    const uint x = gid.x;
    const uint z = gid.y;
    
    int3 v;
    v[pushConstants.axisT.x] = x;
    v[pushConstants.axisT.y] = pushConstants.y;
    v[pushConstants.axisT.z] = z;
    
    if (int(x) >= pushConstants.volDim[pushConstants.axisT.x] || int(z) >= pushConstants.volDim.z)
        return;
    
    // find texture offset
    const int beginX = (pushConstants.axisT.x == 0) ? pushConstants.roi.x.begin : pushConstants.roi.y.begin;
    const int beginY = (pushConstants.axisT.x == 0) ? pushConstants.roi.y.begin : pushConstants.roi.x.begin;
    
    TSimAcc sim_xz = xzSliceForY_d.read(x, z).x;
    float pathCost = 255.0f;
    
    if((z >= 1) && (int(z) < pushConstants.volDim.z - 1))
    {
        float P2 = 0;
        
        if(pushConstants._P2 < 0)
        {
            // _P2 convention: use negative value to skip the use of deltaC.
            P2 = metal::abs(pushConstants._P2);
        }
        else
        {
            const int imX0 = (beginX + v.x) * pushConstants.step; // current
            const int imY0 = (beginY + v.y) * pushConstants.step;
            
            const int imX1 = imX0 - pushConstants.ySign * pushConstants.step * (pushConstants.axisT.y == 0); // M1
            const int imY1 = imY0 - pushConstants.ySign * pushConstants.step * (pushConstants.axisT.y == 1);
            
            const half4 gcr0 = rcMipmapImage_tex.sample(rcMipmapImage_samp, float2((float(imX0) + 0.5f) / float(pushConstants.rcSgmLevelWidth), (float(imY0) + 0.5f) / float(pushConstants.rcSgmLevelHeight)), level(pushConstants.rcMipmapLevel));
            const half4 gcr1 = rcMipmapImage_tex.sample(rcMipmapImage_samp, float2((float(imX1) + 0.5f) / float(pushConstants.rcSgmLevelWidth), (float(imY1) + 0.5f) / float(pushConstants.rcSgmLevelHeight)), level(pushConstants.rcMipmapLevel));
            const float deltaC = euclideanDist3(gcr0, gcr1);
            
            // sigmoid f(x) = i + (a - i) * (1 / ( 1 + e^(10 * (x - P2) / w)))
            // see: https://www.desmos.com/calculator/1qvampwbyx
            // best values found from tests: i = 80, a = 255, w = 80, P2 = 100
            // historical values: i = 15, a = 255, w = 80, P2 = 20
            P2 = sigmoid(80.f, 255.f, 80.f, pushConstants._P2, deltaC);
        }
        
        const TSimAcc bestCostInColM1 = bestSimInYm1_d.read(uint2(x, 0)).x;
        const TSimAcc pathCostMDM1 = xzSliceForYm1_d.read(uint2(x, z - 1)).x; // M1: minus 1 over depths
        const TSimAcc pathCostMD = xzSliceForYm1_d.read(uint2(x, z)).x;
        const TSimAcc pathCostMDP1 = xzSliceForYm1_d.read(uint2(x, z + 1)).x; // P1: plus 1 over depths
        const float minCost = multi_fminf(pathCostMD, pathCostMDM1 + pushConstants.P1, pathCostMDP1 + pushConstants.P1, bestCostInColM1 + P2);

        // if 'pathCostMD' is the minimal value of the depth
        pathCost = (sim_xz) + minCost - bestCostInColM1;
        
    }
    
    // fill the current slice with the new similarity score
    xzSliceForY_d.write(TSimAcc(pathCost), uint2(x, z));

#ifndef TSIM_USE_FLOAT
    // clamp if TSim = uchar (TSimAcc = unsigned int)
    pathCost = min(255.0f, max(0.0f, pathCost));
#endif

    // aggregate into the final output
    TSim volume_xyz = volAgr_d.read(uint3(v.x, v.y, v.z)).x;
    const float val = (float(volume_xyz) * float(pushConstants.filteringIndex) + pathCost) / float(pushConstants.filteringIndex + 1);
    volAgr_d.write(TSim(val), uint3(v.x, v.y, v.z));

}

}
}
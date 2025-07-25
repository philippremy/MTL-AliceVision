//
//  RefineBestDepth.metal
//  AVGPUServices
//
//  Created by Philipp Remy on 16.07.25.
//

#include <metal_stdlib>
using namespace metal;

#include <AVDepthMap/Metal/Types.hpp>
#include <AVDepthMap/Metal/RefineBestDepth_PushConstants.hpp>

namespace aliceVision {
namespace depthMap {

[[kernel]]
kernel void RefineBestDepth(constant RefineBestDepth_PushConstants& pushConstants [[buffer(0)]],
                            texture2d<float, access::read_write> out_refineDepthSimMap_d [[texture(0)]],
                            texture2d<float, access::read> in_sgmDepthPixSizeMap_d [[texture(1)]],
                            texture3d<TSimRefine, access::read> in_volSim_d [[texture(2)]],
                            uint3 gid [[thread_position_in_grid]])
{
    
    const unsigned int vx = gid.x;
    const unsigned int vy = gid.y;

    if(vx >= pushConstants.roi.width() || vy >= pushConstants.roi.height())
        return;
    
    // corresponding input sgm depth/pixSize (middle depth)
    const float2 in_sgmDepthPixSize = in_sgmDepthPixSizeMap_d.read(uint2(vx, vy)).xy;

    // corresponding output depth/sim pointer
    float4 out_bestDepthSimRead = out_refineDepthSimMap_d.read(uint2(vx, vy));
    
    // sgm depth (middle depth) invalid or masked
    if(in_sgmDepthPixSize.x <= 0.0f)
    {
        out_bestDepthSimRead.x = in_sgmDepthPixSize.x;  // -1 (invalid) or -2 (masked)
        out_bestDepthSimRead.y = 1.0f;                  // similarity between (-1, +1)
        out_refineDepthSimMap_d.write(out_bestDepthSimRead, uint2(vx, vy));
        return;
    }

    // find best z sample per pixel
    float bestSampleSim = 0.f;      // all sample sim <= 0.f
    int bestSampleOffsetIndex = 0;  // default is middle depth (SGM)
    
    // sliding gaussian window
    #pragma unroll
    for(int sample = -pushConstants.halfNbSamples; sample <= pushConstants.halfNbSamples; ++sample)
    {
        float sampleSim = 0.f;

        for(int vz = 0; vz < pushConstants.volDimZ; ++vz)
        {
            const int rz = (vz - pushConstants.halfNbDepths);    // relative depth index offset
            const int zs = rz * pushConstants.samplesPerPixSize; // relative sample offset

            // get the inverted similarity sum value
            // best value is the HIGHEST
            // worst value is 0
            const float invSimSum = in_volSim_d.read(uint3(vx, vy, vz)).x;

            // reverse the inverted similarity sum value
            // best value is the LOWEST
            // worst value is 0
            const float simSum = -invSimSum;

            // apply gaussian
            // see: https://www.desmos.com/calculator/ribalnoawq
            sampleSim += simSum * exp(-((zs - sample) * (zs - sample)) / pushConstants.twoTimesSigmaPowerTwo);
        }

        if(sampleSim < bestSampleSim)
        {
            bestSampleOffsetIndex = sample;
            bestSampleSim = sampleSim;
        }
    }
    
    // compute sample size
    const float sampleSize = in_sgmDepthPixSize.y / pushConstants.samplesPerPixSize; // input sgm pixSize / samplesPerPixSize

    // compute sample size offset from z center
    const float sampleSizeOffset = bestSampleOffsetIndex * sampleSize;

    // compute best depth
    // input sgm depth (middle depth) + sample size offset from z center
    const float bestDepth = in_sgmDepthPixSize.x + sampleSizeOffset;

    // write output best depth/sim
    out_bestDepthSimRead.x = bestDepth;
    out_bestDepthSimRead.y = bestSampleSim;
    out_refineDepthSimMap_d.write(out_bestDepthSimRead, uint2(vx, vy));
    
}

}
}
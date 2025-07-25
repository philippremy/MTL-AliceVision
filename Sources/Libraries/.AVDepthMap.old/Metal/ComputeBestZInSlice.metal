//
//  ComputeBestZInSlice.metal
//  AVGPUServices
//
//  Created by Philipp Remy on 14.07.25.
//

#include <metal_stdlib>
using namespace metal;

#include <AVDepthMap/Metal/Types.hpp>
#include <AVDepthMap/Metal/ComputeBestZInSlice_PushConstants.hpp>

namespace aliceVision {
namespace depthMap {

[[kernel]]
kernel void ComputeBestZInSlice(constant ComputeBestZInSlice_PushConstants& pushConstants [[buffer(0)]],
                                texture2d<TSimAcc, access::read> xzSlice_d [[texture(0)]],
                                texture2d<TSimAcc, access::read_write> ySliceBestInColCst_d [[texture(1)]],
                                uint3 gid [[thread_position_in_grid]])
{
    
    uint x = gid.x;
    
    if(int(x) >= pushConstants.volDimX)
        return;
    
    TSimAcc bestCst = xzSlice_d.read(uint2(x, 0)).x;
    
    for(int z = 1; z < pushConstants.volDimZ; ++z)
    {
        const TSimAcc cst = xzSlice_d.read(uint2(x, z)).x;
        bestCst = cst < bestCst ? cst : bestCst;  // min(cst, bestCst);
    }
    
    ySliceBestInColCst_d.write(bestCst, uint2(x, 0));
    
}

}
}

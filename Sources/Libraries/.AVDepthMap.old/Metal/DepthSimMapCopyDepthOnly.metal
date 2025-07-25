//
//  DepthSimMapCopyDepthOnly.metal
//  AVGPUServices
//
//  Created by Philipp Remy on 16.07.25.
//

#include <metal_stdlib>
using namespace metal;

#include <AVDepthMap/Metal/DepthSimMapCopyDepthOnly_PushConstants.hpp>

namespace aliceVision {
namespace depthMap {

[[kernel]]
kernel void DepthSimMapCopyDepthOnly(constant DepthSimMapCopyDepthOnly_PushConstants& pushConstants [[buffer(0)]],
                                     texture2d<float, access::read_write> out_deptSimMap_d [[texture(0)]],
                                     texture2d<float, access::read> in_depthSimMap_d [[texture(1)]],
                                     uint3 gid [[thread_position_in_grid]])
{
    
    const unsigned int x = gid.x;
    const unsigned int y = gid.y;
    
    if(x >= pushConstants.width || y >= pushConstants.height)
        return;
    
    const float4 out_depthSim = out_deptSimMap_d.read(uint2(x, y));
    out_deptSimMap_d.write(float4(in_depthSimMap_d.read(uint2(x, y)).x, pushConstants.defaultSim, out_depthSim.zw), uint2(x, y));
    
}

}
}

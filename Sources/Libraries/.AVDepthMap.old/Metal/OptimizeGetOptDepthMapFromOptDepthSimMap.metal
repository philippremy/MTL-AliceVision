//
//  OptimizeGetOptDepthMapFromOptDepthSimMap.metal
//  AVGPUServices
//
//  Created by Philipp Remy on 16.07.25.
//

#include <metal_stdlib>
using namespace metal;

#include <AVDepthMap/Metal/OptimizeGetOptDepthMapFromOptDepthSimMap_PushConstants.hpp>

namespace aliceVision {
namespace depthMap {

[[kernel]]
kernel void OptimizeGetOptDepthMapFromOptDepthSimMap(constant OptimizeGetOptDepthMapFromOptDepthSimMap_PushConstants& pushConstants [[buffer(0)]],
                                                     texture2d<float, access::read_write> out_tmpOptDepthMap_d [[texture(0)]],
                                                     texture2d<float, access::read> in_optDepthSimMap_d [[texture(1)]],
                                                     uint3 gid [[thread_position_in_grid]])
{
    
    // roi and depth/sim map part coordinates
    const unsigned int roiX = gid.x;
    const unsigned int roiY = gid.y;

    if(roiX >= pushConstants.roi.width() || roiY >= pushConstants.roi.height())
        return;
    
    float4 out = out_tmpOptDepthMap_d.read(uint2(roiX, roiY));
    float in = in_optDepthSimMap_d.read(roiX, roiY).x; // depth
    out.x = in;
    out_tmpOptDepthMap_d.write(out, uint2(roiX, roiY));
    
}

}
}

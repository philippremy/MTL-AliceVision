//
//  MapUpscale.metal
//  AVGPUServices
//
//  Created by Philipp Remy on 16.07.25.
//

#include <metal_stdlib>
using namespace metal;

#include <AVDepthMap/Metal/Types.hpp>
#include <AVDepthMap/Metal/MapUpscale_PushConstants.hpp>

namespace aliceVision {
namespace depthMap {

[[kernel]]
kernel void MapUpscale(constant MapUpscale_PushConstants& pushConstants [[buffer(0)]],
                       texture2d<MTLPixelBaseType, access::read_write> out_upscaledMap_d [[texture(0)]],
                       texture2d<MTLPixelBaseType, access::read> in_map_d [[texture(1)]],
                       uint3 gid [[thread_position_in_grid]])
{
    
    const unsigned int x = gid.x;
    const unsigned int y = gid.y;

    if(x >= pushConstants.roi.width() || y >= pushConstants.roi.height())
        return;

    const float ox = (float(x) - 0.5f) * pushConstants.ratio;
    const float oy = (float(y) - 0.5f) * pushConstants.ratio;

    // nearest neighbor, no interpolation
    const int xp = min(int(floor(ox + 0.5)), int(pushConstants.roi.width()  * pushConstants.ratio) - 1);
    const int yp = min(int(floor(oy + 0.5)), int(pushConstants.roi.height() * pushConstants.ratio) - 1);

    // write output upscaled map
    MTLRGBA out = out_upscaledMap_d.read(uint2(x, y));
    MTLRGBA in = in_map_d.read(uint2(xp, yp));
    out = in;
    out_upscaledMap_d.write(out, uint2(x, y));
    
}

}
}

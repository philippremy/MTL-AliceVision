//
//  RGB2LAB.metal
//  AVGPUServices
//
//  Created by Philipp Remy on 16.07.25.
//

#include <metal_stdlib>
using namespace metal;

#include <AVDepthMap/Metal/Types.hpp>

#include <AVDepthMap/Metal/Color.metal>

namespace aliceVision {
namespace depthMap {

[[kernel]]
kernel void RGB2LAB(texture2d<MTLPixelBaseType, access::read_write> inout_img_d [[texture(0)]],
                    uint3 gid [[thread_position_in_grid]])
{
        
    const unsigned int x = gid.x;
    const unsigned int y = gid.y;

    if((x >= inout_img_d.get_width()) || (y >= inout_img_d.get_height()))
        return;

    // corresponding input CudaRGBA
    MTLRGBA rgb = inout_img_d.read(uint2(x, y));

    // CudaRGBA (uchar4 or float4) in range (0, 255)
    // rgb2xyz needs RGB in range (0, 1)
    constexpr float d = 1 / 255.f;

    // compute output CIELAB
    // RGB(0, 255) to XYZ(0, 1) to CIELAB(0, 255)
    float3 flab = xyz2lab(rgb2xyz(float3(float(rgb.x) * d, float(rgb.y) * d, float(rgb.z) * d)));

    // write output CIELAB
    rgb.x = flab.x;
    rgb.y = flab.y;
    rgb.z = flab.z;
    
    inout_img_d.write(rgb, uint2(x, y));
    
}

}
}
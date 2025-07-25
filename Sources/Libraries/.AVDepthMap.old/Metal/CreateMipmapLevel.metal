//
//  CreateMipmapLevel.metal
//  AVGPUServices
//
//  Created by Philipp Remy on 14.07.25.
//

#include <metal_stdlib>
using namespace metal;

#include <AVDepthMap/Metal/Types.hpp>
#include <AVDepthMap/Metal/CreateMipmapLevel_PushConstants.hpp>

#include <AVDepthMap/Metal/Buffer.metal>

namespace aliceVision {
namespace depthMap {

[[kernel]]
kernel void CreateMipmapLevel(constant CreateMipmapLevel_PushConstants& pushConstants [[buffer(0)]],
                              texture2d<MTLPixelBaseType, access::sample> in_image_tex [[texture(0)]],
                              sampler in_image_samp [[sampler(0)]],
                              texture2d<MTLPixelBaseType, access::write> out_mipmappedArrayLevel [[texture(1)]],
                              constant int* d_gaussianArrayOffset [[buffer(1)]],
                              constant float* d_gaussianArray [[buffer(2)]],
                              uint3 gid [[thread_position_in_grid]])
{
    
    uint x = gid.x;
    uint y = gid.y;
    
    uint2 outImageSize = uint2(out_mipmappedArrayLevel.get_width(), out_mipmappedArrayLevel.get_height());
    
    if(x >= outImageSize.x || y >= outImageSize.y)
        return;
    
    const float px = 1.f / float(outImageSize.x);
    const float py = 1.f / float(outImageSize.y);

    MTLRGBA sumColor = MTLRGBA(0.0f, 0.0f, 0.0f, 0.0f);
    float sumFactor = 0.0f;
    
    for(int i = -pushConstants.radius; i <= pushConstants.radius; i++)
    {
        for(int j = -pushConstants.radius; j <= pushConstants.radius; j++)
        {
            // domain factor
            const float factor = getGauss(d_gaussianArrayOffset, d_gaussianArray, 1, i + pushConstants.radius) * getGauss(d_gaussianArrayOffset, d_gaussianArray, 1, j + pushConstants.radius);

            // normalized coordinates
            float2 uv = float2((float(x + j) + 0.5) * float(px), (float(y + i) + 0.5) * float(py));

            // current pixel color
            const MTLRGBA color = in_image_tex.sample(in_image_samp, uv, level(pushConstants.out_mipLevel - 1));

            // sum color
            sumColor = sumColor + color * factor;

            // sum factor
            sumFactor += factor;
        }
    }
    
    const MTLRGBA color = sumColor / sumFactor;
    
#ifdef ALICEVISION_DEPTHMAP_TEXTURE_USE_UCHAR
    // convert color to unsigned char
    MTLRGBA out;
    out.x = MTLPixelBaseType(color.x);
    out.y = MTLPixelBaseType(color.y);
    out.z = MTLPixelBaseType(color.z);
    out.w = MTLPixelBaseType(color.w);

    // write output color
    out_mipmappedArrayLevel.write(out, uint2(x, y));
#else // texture use float4 or half4
    #ifdef ALICEVISION_DEPTHMAP_TEXTURE_USE_HALF
        // convert color to half
        out_mipmappedArrayLevel.write(color, uint2(x, y));
    #else // texture use float4
        // write output color
        out_mipmappedArrayLevel.write(color, uint2(x, y));
    #endif // ALICEVISION_DEPTHMAP_TEXTURE_USE_HALF
#endif // ALICEVISION_DEPTHMAP_TEXTURE_USE_UCHAR
    
}

}
}

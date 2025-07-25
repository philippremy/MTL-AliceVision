//
//  DownscaleWithGaussianBlur.metal
//  AVGPUServices
//
//  Created by Philipp Remy on 16.07.25.
//

#include <metal_stdlib>
using namespace metal;

#include <AVDepthMap/Metal/Types.hpp>
#include <AVDepthMap/Metal/DownscaleWithGaussianBlur_PushConstants.hpp>

#include <AVDepthMap/Metal/Buffer.metal>

namespace aliceVision {
namespace depthMap {

[[kernel]]
kernel void DownscaleWithGaussianBlur(constant DownscaleWithGaussianBlur_PushConstants& pushConstants [[buffer(0)]],
                                      texture2d<MTLPixelBaseType, access::sample> in_img_tex [[texture(0)]],
                                      sampler in_img_samp [[sampler(0)]],
                                      texture2d<MTLPixelBaseType, access::read_write> out_downscaledImg_d [[texture(1)]],
                                      constant int* d_gaussianArrayOffset [[buffer(1)]],
                                      constant float* d_gaussianArray [[buffer(2)]],
                                      uint3 gid [[thread_position_in_grid]])
{

    const unsigned int x = gid.x;
    const unsigned int y = gid.y;
    
    if((x < out_downscaledImg_d.get_width()) && (y < out_downscaledImg_d.get_height()))
    {
        const float s = float(pushConstants.downscale) * 0.5f;

        MTLRGBA accPix = MTLRGBA(0.0f, 0.0f, 0.0f, 0.0f);
        float sumFactor = 0.0f;

        #pragma unroll
        for(int i = -pushConstants.gaussRadius; i <= pushConstants.gaussRadius; i++)
        {
            #pragma unroll
            for(int j = -pushConstants.gaussRadius; j <= pushConstants.gaussRadius; j++)
            {
                const MTLRGBA curPix = in_img_tex.sample(in_img_samp, float2(float(x * pushConstants.downscale + j) + s, float(y * pushConstants.downscale + i) + s));
                const float factor = getGauss(d_gaussianArrayOffset, d_gaussianArray, pushConstants.downscale - 1, i + pushConstants.gaussRadius) *
                                     getGauss(d_gaussianArrayOffset, d_gaussianArray, pushConstants.downscale - 1, j + pushConstants.gaussRadius); // domain factor

                accPix = accPix + curPix * factor;
                sumFactor += factor;
            }
        }

        MTLRGBA out = out_downscaledImg_d.read(uint2(x, y));
        out.x = accPix.x / sumFactor;
        out.y = accPix.y / sumFactor;
        out.z = accPix.z / sumFactor;
        out.w = accPix.w / sumFactor;
        out_downscaledImg_d.write(out, uint2(x, y));
    }
    
}

}
}
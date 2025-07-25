//
//  OptimizeVarLofLABtoW.metal
//  AVGPUServices
//
//  Created by Philipp Remy on 16.07.25.
//

#include <metal_stdlib>
using namespace metal;

#include <AVDepthMap/Metal/Types.hpp>
#include <AVDepthMap/Metal/OptimizeVarLofLABtoW_PushConstants.hpp>

namespace aliceVision {
namespace depthMap {

[[kernel]]
kernel void OptimizeVarLofLABtoW(constant OptimizeVarLofLABtoW_PushConstants& pushConstants [[buffer(0)]],
                                 texture2d<float, access::read_write> out_varianceMap_d [[texture(0)]],
                                 texture2d<MTLPixelBaseType, access::sample> rcMipmapImage_tex [[texture(1)]],
                                 sampler rcMipmapImage_samp [[sampler(0)]],
                                 uint3 gid [[thread_position_in_grid]])
{
    
    // roi and varianceMap coordinates
    const unsigned int roiX = gid.x;
    const unsigned int roiY = gid.y;

    if(roiX >= pushConstants.roi.width() || roiY >= pushConstants.roi.height())
        return;

    // corresponding image coordinates
    const float x = float(pushConstants.roi.x.begin + roiX) * float(pushConstants.stepXY);
    const float y = float(pushConstants.roi.y.begin + roiY) * float(pushConstants.stepXY);

    // compute inverse width / height
    // note: useful to compute p1 / m1 normalized coordinates
    const float invLevelWidth  = 1.f / float(pushConstants.rcLevelWidth);
    const float invLevelHeight = 1.f / float(pushConstants.rcLevelHeight);

    // compute gradient size of L
    // note: we use 0.5f offset because rcTex texture use interpolation;
    const float xM1 = rcMipmapImage_tex.sample(rcMipmapImage_samp, float2(((x + 1.f) + 0.5f) * invLevelWidth, ((y + 0.f) + 0.5f) * invLevelHeight), pushConstants.rcMipmapLevel).x;
    const float xP1 = rcMipmapImage_tex.sample(rcMipmapImage_samp, float2(((x - 1.f) + 0.5f) * invLevelWidth, ((y + 0.f) + 0.5f) * invLevelHeight), pushConstants.rcMipmapLevel).x;
    const float yM1 = rcMipmapImage_tex.sample(rcMipmapImage_samp, float2(((x + 0.f) + 0.5f) * invLevelWidth, ((y - 1.f) + 0.5f) * invLevelHeight), pushConstants.rcMipmapLevel).x;
    const float yP1 = rcMipmapImage_tex.sample(rcMipmapImage_samp, float2(((x + 0.f) + 0.5f) * invLevelWidth, ((y + 1.f) + 0.5f) * invLevelHeight), pushConstants.rcMipmapLevel).x;

    const float2 g = float2(xM1 - xP1, yM1 - yP1); // TODO: not divided by 2?
    const float grad = length(g);

    // write output
    float4 out = out_varianceMap_d.read(uint2(roiX, roiY));
    out.x = grad;
    out_varianceMap_d.write(out, uint2(roiX, roiY));
    
}

}
}

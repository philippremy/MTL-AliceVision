//
//  ComputeSgmUpscaledDepthPixSizeMap_NearestNeighbor.metal
//  AVGPUServices
//
//  Created by Philipp Remy on 16.07.25.
//

#include <metal_stdlib>
using namespace metal;

#include <AVDepthMap/Metal/Types.hpp>
#include <AVDepthMap/Metal/DeviceCameraParams.hpp>
#include <AVDepthMap/Metal/ComputeSgmUpscaledDepthPixSizeMap_NearestNeighbor_PushConstants.hpp>

namespace aliceVision {
namespace depthMap {

[[kernel]]
kernel void ComputeSgmUpscaledDepthPixSizeMap_NearestNeighbor(constant ComputeSgmUpscaledDepthPixSizeMap_NearestNeighbor_PushConstants& pushConstants [[buffer(0)]],
                                                              texture2d<float, access::read_write> out_upscaledDepthPixSizeMap_d [[texture(0)]],
                                                              texture2d<float, access::read> in_sgmDepthThicknessMap_d [[texture(1)]],
                                                              texture2d<MTLPixelBaseType, access::sample> rcMipmapImage_tex [[texture(2)]],
                                                              sampler rcMipmapImage_samp [[sampler(0)]],
                                                              constant DeviceCameraParams* constantCameraParametersArray_d [[buffer(1)]],
                                                              uint3 gid [[thread_position_in_grid]])
{
    const unsigned int roiX = gid.x;
    const unsigned int roiY = gid.y;

    if(roiX >= pushConstants.roi.width() || roiY >= pushConstants.roi.height())
        return;

    // corresponding image coordinates
    const unsigned int x = (pushConstants.roi.x.begin + roiX) * (unsigned int)(pushConstants.stepXY);
    const unsigned int y = (pushConstants.roi.y.begin + roiY) * (unsigned int)(pushConstants.stepXY);

    // corresponding output upscaled depth/pixSize map
    float4 out_depthPixSize = out_upscaledDepthPixSizeMap_d.read(uint2(roiX, roiY));

    // filter masked pixels (alpha < 0.9f)
    if(rcMipmapImage_tex.sample(rcMipmapImage_samp, float2((float(x) + 0.5f) / float(pushConstants.rcLevelWidth), (float(y) + 0.5f) / float(pushConstants.rcLevelHeight)), level(pushConstants.rcMipmapLevel)).w < 0.9f)
    {
        out_upscaledDepthPixSizeMap_d.write(float4(float2(-2.f, 0.f), out_depthPixSize.zw), uint2(roiX, roiY));
        return;
    }

    // find corresponding depth/thickness
    // nearest neighbor, no interpolation
    const float oy = (float(roiY) - 0.5f) * pushConstants.ratio;
    const float ox = (float(roiX) - 0.5f) * pushConstants.ratio;

    int xp = floor(ox + 0.5);
    int yp = floor(oy + 0.5);

    xp = min(xp, int(pushConstants.roi.width()  * pushConstants.ratio) - 1);
    yp = min(yp, int(pushConstants.roi.height() * pushConstants.ratio) - 1);

    const float4 out_depthThickness = in_sgmDepthThicknessMap_d.read(uint2(xp, yp));

#ifdef ALICEVISION_DEPTHMAP_COMPUTE_PIXSIZEMAP
    // R camera parameters
    constant DeviceCameraParams& rcDeviceCamParams = constantCameraParametersArray_d[pushConstants.rcDeviceCameraParamsId];

    // get rc 3d point
    const float3 p = get3DPointForPixelAndDepthFromRC(rcDeviceCamParams, float2(float(x), float(y)), out_depthThickness.x);

    // compute and write rc 3d point pixSize
    const float out_pixSize = computePixSize(rcDeviceCamParams, p);
#else
    // compute pixSize from depth thickness
    const float out_pixSize = out_depthThickness.y / pushConstants.halfNbDepths;
#endif

    // write output depth/pixSize
    out_upscaledDepthPixSizeMap_d.write(float4(out_depthThickness.x, out_pixSize, out_depthPixSize.wz), uint2(roiX, roiY));
}

}
}
#pragma clang diagnostic ignored "-Wmissing-prototypes"

#include <metal_stdlib>
#include <simd/simd.h>

using namespace metal;

struct Range
{
    uint begin;
    uint end;
};

struct ROI
{
    Range x;
    Range y;
};

struct Range_1
{
    uint begin;
    uint end;
};

struct ROI_1
{
    Range_1 x;
    Range_1 y;
};

struct PushConstants
{
    uint rcLevelWidth;
    uint rcLevelHeight;
    float rcMipmapLevel;
    int stepXY;
    ROI_1 roi;
};

constant uint3 gl_WorkGroupSize [[maybe_unused]] = uint3(32u, 2u, 1u);

struct spvDescriptorSetBuffer0
{
    texture2d<float> rcMipmapImage_tex [[id(0)]];
    sampler rcMipmapImage_texSmplr [[id(1)]];
    texture2d<float, access::read_write> out_varianceMap_d [[id(2)]];
};

static inline __attribute__((always_inline))
uint Range_size(thread const Range& range)
{
    return range.end - range.begin;
}

static inline __attribute__((always_inline))
uint ROI_width(thread const ROI& roi)
{
    Range param = roi.x;
    return Range_size(param);
}

static inline __attribute__((always_inline))
uint ROI_height(thread const ROI& roi)
{
    Range param = roi.y;
    return Range_size(param);
}

kernel void main0(constant spvDescriptorSetBuffer0& spvDescriptorSet0 [[buffer(0)]], constant PushConstants& pushConstants [[buffer(1)]], uint3 gl_GlobalInvocationID [[thread_position_in_grid]])
{
    uint roiX = gl_GlobalInvocationID.x;
    uint roiY = gl_GlobalInvocationID.y;
    ROI param;
    param.x.begin = pushConstants.roi.x.begin;
    param.x.end = pushConstants.roi.x.end;
    param.y.begin = pushConstants.roi.y.begin;
    param.y.end = pushConstants.roi.y.end;
    bool _83 = roiX >= ROI_width(param);
    bool _105;
    if (!_83)
    {
        ROI param_1;
        param_1.x.begin = pushConstants.roi.x.begin;
        param_1.x.end = pushConstants.roi.x.end;
        param_1.y.begin = pushConstants.roi.y.begin;
        param_1.y.end = pushConstants.roi.y.end;
        _105 = roiY >= ROI_height(param_1);
    }
    else
    {
        _105 = _83;
    }
    if (_105)
    {
        return;
    }
    float x = float(pushConstants.roi.x.begin + roiX) * float(pushConstants.stepXY);
    float y = float(pushConstants.roi.y.begin + roiY) * float(pushConstants.stepXY);
    float invLevelWidth = 1.0 / float(pushConstants.rcLevelWidth);
    float invLevelHeight = 1.0 / float(pushConstants.rcLevelHeight);
    float xM1 = spvDescriptorSet0.rcMipmapImage_tex.sample(spvDescriptorSet0.rcMipmapImage_texSmplr, float2(((x - 1.0) + 0.5) * invLevelWidth, ((y + 0.0) + 0.5) * invLevelHeight), level(pushConstants.rcMipmapLevel)).x;
    float xP1 = spvDescriptorSet0.rcMipmapImage_tex.sample(spvDescriptorSet0.rcMipmapImage_texSmplr, float2(((x + 1.0) + 0.5) * invLevelWidth, ((y + 0.0) + 0.5) * invLevelHeight), level(pushConstants.rcMipmapLevel)).x;
    float yM1 = spvDescriptorSet0.rcMipmapImage_tex.sample(spvDescriptorSet0.rcMipmapImage_texSmplr, float2(((x + 0.0) + 0.5) * invLevelWidth, ((y - 1.0) + 0.5) * invLevelHeight), level(pushConstants.rcMipmapLevel)).x;
    float yP1 = spvDescriptorSet0.rcMipmapImage_tex.sample(spvDescriptorSet0.rcMipmapImage_texSmplr, float2(((x + 0.0) + 0.5) * invLevelWidth, ((y + 1.0) + 0.5) * invLevelHeight), level(pushConstants.rcMipmapLevel)).x;
    float2 g = float2(xM1 - xP1, yM1 - yP1);
    float grad = length(g);
    spvDescriptorSet0.out_varianceMap_d.fence();
    float4 outp = spvDescriptorSet0.out_varianceMap_d.read(uint2(int2(int(roiX), int(roiY))));
    outp.x = grad;
    spvDescriptorSet0.out_varianceMap_d.write(outp, uint2(int2(int(roiX), int(roiY))));
}


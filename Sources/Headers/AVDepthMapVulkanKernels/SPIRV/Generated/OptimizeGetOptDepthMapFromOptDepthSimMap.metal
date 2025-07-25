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
    ROI_1 roi;
};

constant uint3 gl_WorkGroupSize [[maybe_unused]] = uint3(16u, 16u, 1u);

struct spvDescriptorSetBuffer0
{
    texture2d<float, access::read_write> out_tmpOptDepthMap_d [[id(0)]];
    texture2d<float> in_optDepthSimMap_d [[id(1)]];
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
    bool _81 = roiX >= ROI_width(param);
    bool _103;
    if (!_81)
    {
        ROI param_1;
        param_1.x.begin = pushConstants.roi.x.begin;
        param_1.x.end = pushConstants.roi.x.end;
        param_1.y.begin = pushConstants.roi.y.begin;
        param_1.y.end = pushConstants.roi.y.end;
        _103 = roiY >= ROI_height(param_1);
    }
    else
    {
        _103 = _81;
    }
    if (_103)
    {
        return;
    }
    spvDescriptorSet0.out_tmpOptDepthMap_d.fence();
    float4 outp = spvDescriptorSet0.out_tmpOptDepthMap_d.read(uint2(int2(int(roiX), int(roiY))));
    outp.x = spvDescriptorSet0.in_optDepthSimMap_d.read(uint2(int2(int(roiX), int(roiY)))).x;
    spvDescriptorSet0.out_tmpOptDepthMap_d.write(outp, uint2(int2(int(roiX), int(roiY))));
}


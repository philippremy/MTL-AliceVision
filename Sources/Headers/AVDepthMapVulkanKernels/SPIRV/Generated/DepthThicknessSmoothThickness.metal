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
    float minThicknessInflate;
    float maxThicknessInflate;
    ROI_1 roi;
};

constant uint3 gl_WorkGroupSize [[maybe_unused]] = uint3(8u, 8u, 1u);

struct spvDescriptorSetBuffer0
{
    texture2d<float, access::read_write> inout_depthThicknessMap_d [[id(0)]];
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
    spvDescriptorSet0.inout_depthThicknessMap_d.fence();
    float4 inout_depthThicknessLoaded = spvDescriptorSet0.inout_depthThicknessMap_d.read(uint2(int2(int(roiX), int(roiY))));
    float2 inout_depthThickness = inout_depthThicknessLoaded.xy;
    if (inout_depthThickness.x <= 0.0)
    {
        return;
    }
    float minThickness = pushConstants.minThicknessInflate * inout_depthThickness.y;
    float maxThickness = pushConstants.maxThicknessInflate * inout_depthThickness.y;
    float sumCenterDepthDist = 0.0;
    int nbValidPatchPixels = 0;
    ROI param_2;
    ROI param_3;
    for (int yp = -1; yp <= 1; yp++)
    {
        for (int xp = -1; xp <= 1; xp++)
        {
            int roiXp = int(roiX) + xp;
            int roiYp = int(roiY) + yp;
            bool _186 = ((xp == 0) && (yp == 0)) || (roiXp < 0);
            bool _209;
            if (!_186)
            {
                param_2.x.begin = pushConstants.roi.x.begin;
                param_2.x.end = pushConstants.roi.x.end;
                param_2.y.begin = pushConstants.roi.y.begin;
                param_2.y.end = pushConstants.roi.y.end;
                _209 = uint(roiXp) >= ROI_width(param_2);
            }
            else
            {
                _209 = _186;
            }
            bool _212 = _209 || (roiYp < 0);
            bool _235;
            if (!_212)
            {
                param_3.x.begin = pushConstants.roi.x.begin;
                param_3.x.end = pushConstants.roi.x.end;
                param_3.y.begin = pushConstants.roi.y.begin;
                param_3.y.end = pushConstants.roi.y.end;
                _235 = uint(roiYp) >= ROI_height(param_3);
            }
            else
            {
                _235 = _212;
            }
            if (_235)
            {
                continue;
            }
            spvDescriptorSet0.inout_depthThicknessMap_d.fence();
            float4 in_depthThicknessPatchLoaded = spvDescriptorSet0.inout_depthThicknessMap_d.read(uint2(int2(roiXp, roiYp)));
            float2 in_depthThicknessPatch = in_depthThicknessPatchLoaded.xy;
            if (in_depthThicknessPatch.x > 0.0)
            {
                float depthDistance = abs(inout_depthThickness.x - in_depthThicknessPatch.x);
                sumCenterDepthDist += fast::max(minThickness, fast::min(maxThickness, depthDistance));
                nbValidPatchPixels++;
            }
        }
    }
    if (nbValidPatchPixels < 3)
    {
        return;
    }
    inout_depthThickness.y = sumCenterDepthDist / float(nbValidPatchPixels);
    spvDescriptorSet0.inout_depthThicknessMap_d.write(float4(inout_depthThickness, inout_depthThicknessLoaded.z, inout_depthThicknessLoaded.w), uint2(int2(int(roiX), int(roiY))));
}


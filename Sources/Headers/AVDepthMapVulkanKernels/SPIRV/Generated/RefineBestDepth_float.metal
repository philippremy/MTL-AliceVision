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
    int volDimZ;
    int samplesPerPixSize;
    int halfNbSamples;
    int halfNbDepths;
    float twoTimesSigmaPowerTwo;
    ROI_1 roi;
};

constant uint3 gl_WorkGroupSize [[maybe_unused]] = uint3(32u, 32u, 1u);

struct spvDescriptorSetBuffer0
{
    texture2d<float> in_sgmDepthPixSizeMap_d [[id(0)]];
    texture2d<float, access::read_write> out_refineDepthSimMap_d [[id(1)]];
    texture3d<float> in_volSim_d [[id(2)]];
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
    uint vx = gl_GlobalInvocationID.x;
    uint vy = gl_GlobalInvocationID.y;
    ROI param;
    param.x.begin = pushConstants.roi.x.begin;
    param.x.end = pushConstants.roi.x.end;
    param.y.begin = pushConstants.roi.y.begin;
    param.y.end = pushConstants.roi.y.end;
    bool _83 = vx >= ROI_width(param);
    bool _105;
    if (!_83)
    {
        ROI param_1;
        param_1.x.begin = pushConstants.roi.x.begin;
        param_1.x.end = pushConstants.roi.x.end;
        param_1.y.begin = pushConstants.roi.y.begin;
        param_1.y.end = pushConstants.roi.y.end;
        _105 = vy >= ROI_height(param_1);
    }
    else
    {
        _105 = _83;
    }
    if (_105)
    {
        return;
    }
    float2 in_sgmDepthPixSize = spvDescriptorSet0.in_sgmDepthPixSizeMap_d.read(uint2(int2(int(vx), int(vy)))).xy;
    spvDescriptorSet0.out_refineDepthSimMap_d.fence();
    float4 out_bestDepthSimPtr = spvDescriptorSet0.out_refineDepthSimMap_d.read(uint2(int2(int(vx), int(vy))));
    if (in_sgmDepthPixSize.x <= 0.0)
    {
        out_bestDepthSimPtr.x = in_sgmDepthPixSize.x;
        out_bestDepthSimPtr.y = 1.0;
        spvDescriptorSet0.out_refineDepthSimMap_d.write(out_bestDepthSimPtr, uint2(int2(int(vx), int(vy))));
        return;
    }
    float bestSampleSim = 0.0;
    int bestSampleOffsetIndex = 0;
    int _163 = -pushConstants.halfNbSamples;
    for (int samp = _163; samp <= pushConstants.halfNbSamples; samp++)
    {
        float sampleSim = 0.0;
        for (int vz = 0; vz < pushConstants.volDimZ; vz++)
        {
            int rz = vz - pushConstants.halfNbDepths;
            int zs = rz * pushConstants.samplesPerPixSize;
            float invSimSum = spvDescriptorSet0.in_volSim_d.read(uint3(int3(int(vx), int(vy), vz))).x;
            float simSum = -invSimSum;
            sampleSim += (simSum * exp(float(-((zs - samp) * (zs - samp))) / pushConstants.twoTimesSigmaPowerTwo));
        }
        if (sampleSim < bestSampleSim)
        {
            bestSampleOffsetIndex = samp;
            bestSampleSim = sampleSim;
        }
    }
    float sampleSize = in_sgmDepthPixSize.y / float(pushConstants.samplesPerPixSize);
    float sampleSizeOffset = float(bestSampleOffsetIndex) * sampleSize;
    float bestDepth = in_sgmDepthPixSize.x + sampleSizeOffset;
    out_bestDepthSimPtr.x = bestDepth;
    out_bestDepthSimPtr.y = bestSampleSim;
    spvDescriptorSet0.out_refineDepthSimMap_d.write(out_bestDepthSimPtr, uint2(int2(int(vx), int(vy))));
}


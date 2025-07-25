#pragma clang diagnostic ignored "-Wmissing-prototypes"

#include <metal_stdlib>
#include <simd/simd.h>

using namespace metal;

struct GaussianArray
{
    float gaussian[128];
};

struct GaussianArrayOffset
{
    int offsets[10];
};

struct PushConstants
{
    int radius;
    uint out_mipLevel;
};

constant uint3 gl_WorkGroupSize [[maybe_unused]] = uint3(16u, 16u, 1u);

struct spvDescriptorSetBuffer0
{
    constant GaussianArray* d_gaussianArray [[id(0)]];
    constant GaussianArrayOffset* d_gaussianArrayOffset [[id(1)]];
    texture2d<float, access::write> out_mipmappedArrayLevel [[id(2)]];
    texture2d<float> in_image [[id(3)]];
    sampler in_imageSmplr [[id(4)]];
};

static inline __attribute__((always_inline))
float getGauss(thread const int& scale, thread const int& idx, constant GaussianArray& d_gaussianArray, constant GaussianArrayOffset& d_gaussianArrayOffset)
{
    return d_gaussianArray.gaussian[d_gaussianArrayOffset.offsets[scale] + idx];
}

kernel void main0(constant spvDescriptorSetBuffer0& spvDescriptorSet0 [[buffer(0)]], constant PushConstants& pushConstants [[buffer(1)]], uint3 gl_GlobalInvocationID [[thread_position_in_grid]])
{
    uint x = gl_GlobalInvocationID.x;
    uint y = gl_GlobalInvocationID.y;
    int2 outImageSize = int2(spvDescriptorSet0.out_mipmappedArrayLevel.get_width(), spvDescriptorSet0.out_mipmappedArrayLevel.get_height());
    bool _63 = x >= uint(outImageSize.x);
    bool _72;
    if (!_63)
    {
        _72 = y >= uint(outImageSize.y);
    }
    else
    {
        _72 = _63;
    }
    if (_72)
    {
        return;
    }
    float px = 1.0 / float(outImageSize.x);
    float py = 1.0 / float(outImageSize.y);
    float4 sumColor = float4(0.0);
    float sumFactor = 0.0;
    int _101 = -pushConstants.radius;
    for (int i = _101; i <= pushConstants.radius; i++)
    {
        int _114 = -pushConstants.radius;
        for (int j = _114; j <= pushConstants.radius; j++)
        {
            int param = 1;
            int param_1 = i + pushConstants.radius;
            int param_2 = 1;
            int param_3 = j + pushConstants.radius;
            float factor = getGauss(param, param_1, (*spvDescriptorSet0.d_gaussianArray), (*spvDescriptorSet0.d_gaussianArrayOffset)) * getGauss(param_2, param_3, (*spvDescriptorSet0.d_gaussianArray), (*spvDescriptorSet0.d_gaussianArrayOffset));
            float2 uv = float2((float(x + uint(j)) + 0.5) * px, (float(y + uint(i)) + 0.5) * py);
            float4 color = spvDescriptorSet0.in_image.sample(spvDescriptorSet0.in_imageSmplr, uv, level(float(pushConstants.out_mipLevel - 1u)));
            sumColor += (color * factor);
            sumFactor += factor;
        }
    }
    float4 color_1 = sumColor / float4(sumFactor);
    spvDescriptorSet0.out_mipmappedArrayLevel.write(color_1, uint2(int2(int(x), int(y))));
}


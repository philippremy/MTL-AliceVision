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
    int downscale;
    int gaussRadius;
};

constant uint3 gl_WorkGroupSize [[maybe_unused]] = uint3(32u, 2u, 1u);

struct spvDescriptorSetBuffer0
{
    constant GaussianArray* d_gaussianArray [[id(0)]];
    constant GaussianArrayOffset* d_gaussianArrayOffset [[id(1)]];
    texture2d<float, access::write> out_downscaledImg_d [[id(2)]];
    texture2d<float> in_img_tex [[id(3)]];
    sampler in_img_texSmplr [[id(4)]];
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
    int2 outImageSize = int2(spvDescriptorSet0.out_downscaledImg_d.get_width(), spvDescriptorSet0.out_downscaledImg_d.get_height());
    bool _63 = x < uint(outImageSize.x);
    bool _71;
    if (_63)
    {
        _71 = y < uint(outImageSize.y);
    }
    else
    {
        _71 = _63;
    }
    if (_71)
    {
        float s = float(pushConstants.downscale) * 0.5;
        half4 accPix = half4(half(0.0));
        half sumFactor = half(0.0);
        int _97 = -pushConstants.gaussRadius;
        for (int i = _97; i <= pushConstants.gaussRadius; i++)
        {
            int _110 = -pushConstants.gaussRadius;
            for (int j = _110; j <= pushConstants.gaussRadius; j++)
            {
                float2 pixelCoord = float2(float((x * uint(pushConstants.downscale)) + uint(j)) + s, float((y * uint(pushConstants.downscale)) + uint(i)) + s);
                int2 texel = int2(pixelCoord);
                half4 curPix = half4(spvDescriptorSet0.in_img_tex.read(uint2(texel), 0));
                int param = pushConstants.downscale - 1;
                int param_1 = i + pushConstants.gaussRadius;
                int param_2 = pushConstants.downscale - 1;
                int param_3 = j + pushConstants.gaussRadius;
                float factor = getGauss(param, param_1, (*spvDescriptorSet0.d_gaussianArray), (*spvDescriptorSet0.d_gaussianArrayOffset)) * getGauss(param_2, param_3, (*spvDescriptorSet0.d_gaussianArray), (*spvDescriptorSet0.d_gaussianArrayOffset));
                accPix += (curPix * half(factor));
                sumFactor += half(factor);
            }
        }
        half4 outPix = half4(half(0.0));
        outPix = accPix / half4(sumFactor);
        spvDescriptorSet0.out_downscaledImg_d.write(float4(outPix), uint2(int2(int(x), int(y))));
    }
}


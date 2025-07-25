#pragma clang diagnostic ignored "-Wmissing-prototypes"

#include <metal_stdlib>
#include <simd/simd.h>

using namespace metal;

struct spvDescriptorSetBuffer0
{
    texture2d<float, access::read_write> inout_img_dmp [[id(0)]];
};

static inline __attribute__((always_inline))
half3 rgb2xyz(half3 c)
{
    return half3(half(((0.41245639324188232421875 * float(c.x)) + (0.3575761020183563232421875 * float(c.y))) + (0.180437505245208740234375 * float(c.z))), half(((0.21267290413379669189453125 * float(c.x)) + (0.715152204036712646484375 * float(c.y))) + (0.072175003588199615478515625 * float(c.z))), half(((0.01933390088379383087158203125 * float(c.x)) + (0.119191996753215789794921875 * float(c.y))) + (0.950304090976715087890625 * float(c.z))));
}

static inline __attribute__((always_inline))
half3 xyz2lab(half3 c)
{
    half3 r = half3(half(float(c.x) / 0.950469970703125), c.y, half(float(c.z) / 1.08882999420166015625));
    float _91;
    if (float(r.x) > 0.008856452070176601409912109375)
    {
        _91 = powr(float(r.x), 0.3333333432674407958984375);
    }
    else
    {
        _91 = ((903.29632568359375 * float(r.x)) + 16.0) / 116.0;
    }
    float _115;
    if (float(r.y) > 0.008856452070176601409912109375)
    {
        _115 = powr(float(r.y), 0.3333333432674407958984375);
    }
    else
    {
        _115 = ((903.29632568359375 * float(r.y)) + 16.0) / 116.0;
    }
    float _135;
    if (float(r.z) > 0.008856452070176601409912109375)
    {
        _135 = powr(float(r.z), 0.3333333432674407958984375);
    }
    else
    {
        _135 = ((903.29632568359375 * float(r.z)) + 16.0) / 116.0;
    }
    half3 f = half3(half(_91), half(_115), half(_135));
    half3 outVal = half3(half((116.0 * float(f.y)) - 16.0), half(500.0 * float(f.x - f.y)), half(200.0 * float(f.y - f.z)));
    outVal *= half(2.548828125);
    return outVal;
}

kernel void main0(constant spvDescriptorSetBuffer0& spvDescriptorSet0 [[buffer(0)]], uint3 gl_GlobalInvocationID [[thread_position_in_grid]])
{
    uint x = gl_GlobalInvocationID.x;
    uint y = gl_GlobalInvocationID.y;
    int2 outImageSize = int2(spvDescriptorSet0.inout_img_dmp.get_width(), spvDescriptorSet0.inout_img_dmp.get_height());
    bool _209 = x >= uint(outImageSize.x);
    bool _218;
    if (!_209)
    {
        _218 = y >= uint(outImageSize.y);
    }
    else
    {
        _218 = _209;
    }
    if (_218)
    {
        return;
    }
    spvDescriptorSet0.inout_img_dmp.fence();
    half4 rgba = half4(spvDescriptorSet0.inout_img_dmp.read(uint2(int2(int(x), int(y)))));
    half3 flab = xyz2lab(rgb2xyz(half3(rgba.x * half(0.0039215087890625), rgba.y * half(0.0039215087890625), rgba.z * half(0.0039215087890625))));
    spvDescriptorSet0.inout_img_dmp.write(float4(half4(flab, rgba.w)), uint2(int2(int(x), int(y))));
}


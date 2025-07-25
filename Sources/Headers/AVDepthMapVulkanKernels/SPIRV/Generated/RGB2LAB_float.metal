#pragma clang diagnostic ignored "-Wmissing-prototypes"

#include <metal_stdlib>
#include <simd/simd.h>

using namespace metal;

struct spvDescriptorSetBuffer0
{
    texture2d<float, access::read_write> inout_img_dmp [[id(0)]];
};

static inline __attribute__((always_inline))
float3 rgb2xyz(float3 c)
{
    return float3(((0.41245639324188232421875 * c.x) + (0.3575761020183563232421875 * c.y)) + (0.180437505245208740234375 * c.z), ((0.21267290413379669189453125 * c.x) + (0.715152204036712646484375 * c.y)) + (0.072175003588199615478515625 * c.z), ((0.01933390088379383087158203125 * c.x) + (0.119191996753215789794921875 * c.y)) + (0.950304090976715087890625 * c.z));
}

static inline __attribute__((always_inline))
float3 xyz2lab(float3 c)
{
    float3 r = float3(c.x / 0.950469970703125, c.y, c.z / 1.08882999420166015625);
    float _72;
    if (r.x > 0.008856452070176601409912109375)
    {
        _72 = powr(r.x, 0.3333333432674407958984375);
    }
    else
    {
        _72 = ((903.29632568359375 * r.x) + 16.0) / 116.0;
    }
    float _92;
    if (r.y > 0.008856452070176601409912109375)
    {
        _92 = powr(r.y, 0.3333333432674407958984375);
    }
    else
    {
        _92 = ((903.29632568359375 * r.y) + 16.0) / 116.0;
    }
    float _108;
    if (r.z > 0.008856452070176601409912109375)
    {
        _108 = powr(r.z, 0.3333333432674407958984375);
    }
    else
    {
        _108 = ((903.29632568359375 * r.z) + 16.0) / 116.0;
    }
    float3 f = float3(_72, _92, _108);
    float3 outVal = float3((116.0 * f.y) - 16.0, 500.0 * (f.x - f.y), 200.0 * (f.y - f.z));
    outVal *= 2.5499999523162841796875;
    return outVal;
}

kernel void main0(constant spvDescriptorSetBuffer0& spvDescriptorSet0 [[buffer(0)]], uint3 gl_GlobalInvocationID [[thread_position_in_grid]])
{
    uint x = gl_GlobalInvocationID.x;
    uint y = gl_GlobalInvocationID.y;
    int2 outImageSize = int2(spvDescriptorSet0.inout_img_dmp.get_width(), spvDescriptorSet0.inout_img_dmp.get_height());
    bool _173 = x >= uint(outImageSize.x);
    bool _182;
    if (!_173)
    {
        _182 = y >= uint(outImageSize.y);
    }
    else
    {
        _182 = _173;
    }
    if (_182)
    {
        return;
    }
    spvDescriptorSet0.inout_img_dmp.fence();
    float4 rgba = spvDescriptorSet0.inout_img_dmp.read(uint2(int2(int(x), int(y))));
    float3 flab = xyz2lab(rgb2xyz(float3(rgba.x * 0.0039215688593685626983642578125, rgba.y * 0.0039215688593685626983642578125, rgba.z * 0.0039215688593685626983642578125)));
    spvDescriptorSet0.inout_img_dmp.write(float4(flab, rgba.w), uint2(int2(int(x), int(y))));
}


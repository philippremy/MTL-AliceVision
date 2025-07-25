#include <metal_stdlib>
#include <simd/simd.h>

using namespace metal;

constant uint3 gl_WorkGroupSize [[maybe_unused]] = uint3(32u, 32u, 1u);

struct spvDescriptorSetBuffer0
{
    texture3d<float, access::read_write> inout_volume2nd_d [[id(0)]];
    texture3d<float> in_volume1st_d [[id(1)]];
};

kernel void main0(constant spvDescriptorSetBuffer0& spvDescriptorSet0 [[buffer(0)]], uint3 gl_GlobalInvocationID [[thread_position_in_grid]])
{
    uint vx = gl_GlobalInvocationID.x;
    uint vy = gl_GlobalInvocationID.y;
    uint vz = gl_GlobalInvocationID.z;
    int3 volumeSize = int3(spvDescriptorSet0.inout_volume2nd_d.get_width(), spvDescriptorSet0.inout_volume2nd_d.get_height(), spvDescriptorSet0.inout_volume2nd_d.get_depth());
    bool _40 = vx >= uint(volumeSize.x);
    bool _49;
    if (!_40)
    {
        _49 = vy >= uint(volumeSize.y);
    }
    else
    {
        _49 = _40;
    }
    if (_49)
    {
        return;
    }
    spvDescriptorSet0.inout_volume2nd_d.fence();
    float4 inout_sim = spvDescriptorSet0.inout_volume2nd_d.read(uint3(int3(int(vx), int(vy), int(vz))));
    if (inout_sim.x >= 255.0)
    {
        float in_sim = spvDescriptorSet0.in_volume1st_d.read(uint3(int3(int(vx), int(vy), int(vz)))).x;
        spvDescriptorSet0.inout_volume2nd_d.write(float4(in_sim, inout_sim.y, inout_sim.z, inout_sim.w), uint3(int3(int(vx), int(vy), int(vz))));
    }
}


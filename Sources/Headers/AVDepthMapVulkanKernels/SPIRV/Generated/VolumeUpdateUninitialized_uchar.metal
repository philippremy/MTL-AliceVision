#include <metal_stdlib>
#include <simd/simd.h>

using namespace metal;

constant uint3 gl_WorkGroupSize [[maybe_unused]] = uint3(32u, 32u, 1u);

struct spvDescriptorSetBuffer0
{
    texture3d<uint, access::read_write> inout_volume2nd_d [[id(0)]];
    texture3d<uint> in_volume1st_d [[id(1)]];
};

kernel void main0(constant spvDescriptorSetBuffer0& spvDescriptorSet0 [[buffer(0)]], uint3 gl_GlobalInvocationID [[thread_position_in_grid]])
{
    uint vx = gl_GlobalInvocationID.x;
    uint vy = gl_GlobalInvocationID.y;
    uint vz = gl_GlobalInvocationID.z;
    int3 volumeSize = int3(spvDescriptorSet0.inout_volume2nd_d.get_width(), spvDescriptorSet0.inout_volume2nd_d.get_height(), spvDescriptorSet0.inout_volume2nd_d.get_depth());
    bool _39 = vx >= uint(volumeSize.x);
    bool _48;
    if (!_39)
    {
        _48 = vy >= uint(volumeSize.y);
    }
    else
    {
        _48 = _39;
    }
    if (_48)
    {
        return;
    }
    spvDescriptorSet0.inout_volume2nd_d.fence();
    uint4 inout_sim = spvDescriptorSet0.inout_volume2nd_d.read(uint3(int3(int(vx), int(vy), int(vz))));
    if (float(inout_sim.x) >= 255.0)
    {
        uint in_sim = spvDescriptorSet0.in_volume1st_d.read(uint3(int3(int(vx), int(vy), int(vz)))).x;
        spvDescriptorSet0.inout_volume2nd_d.write(uint4(in_sim, inout_sim.y, inout_sim.z, inout_sim.w), uint3(int3(int(vx), int(vy), int(vz))));
    }
}


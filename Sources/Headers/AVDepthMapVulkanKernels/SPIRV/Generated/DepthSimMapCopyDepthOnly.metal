#include <metal_stdlib>
#include <simd/simd.h>

using namespace metal;

struct PushConstants
{
    float defaultSim;
};

constant uint3 gl_WorkGroupSize [[maybe_unused]] = uint3(16u, 16u, 1u);

struct spvDescriptorSetBuffer0
{
    texture2d<float, access::read_write> out_deptSimMap_d [[id(0)]];
    texture2d<float> in_depthSimMap_d [[id(1)]];
};

kernel void main0(constant spvDescriptorSetBuffer0& spvDescriptorSet0 [[buffer(0)]], constant PushConstants& pushConstants [[buffer(1)]], uint3 gl_GlobalInvocationID [[thread_position_in_grid]])
{
    uint x = gl_GlobalInvocationID.x;
    uint y = gl_GlobalInvocationID.y;
    int2 out_imageWidth = int2(spvDescriptorSet0.out_deptSimMap_d.get_width(), spvDescriptorSet0.out_deptSimMap_d.get_height());
    bool _36 = x >= uint(out_imageWidth.x);
    bool _45;
    if (!_36)
    {
        _45 = y >= uint(out_imageWidth.y);
    }
    else
    {
        _45 = _36;
    }
    if (_45)
    {
        return;
    }
    spvDescriptorSet0.out_deptSimMap_d.fence();
    float4 out_depthSim = spvDescriptorSet0.out_deptSimMap_d.read(uint2(int2(int(x), int(y))));
    out_depthSim.x = spvDescriptorSet0.in_depthSimMap_d.read(uint2(int2(int(x), int(y)))).x;
    out_depthSim.y = pushConstants.defaultSim;
    spvDescriptorSet0.out_deptSimMap_d.write(out_depthSim, uint2(int2(int(x), int(y))));
}


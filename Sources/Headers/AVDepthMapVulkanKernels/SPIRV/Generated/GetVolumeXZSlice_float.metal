#pragma clang diagnostic ignored "-Wmissing-prototypes"
#pragma clang diagnostic ignored "-Wmissing-braces"

#include <metal_stdlib>
#include <simd/simd.h>

using namespace metal;

template<typename T, size_t Num>
struct spvUnsafeArray
{
    T elements[Num ? Num : 1];
    
    thread T& operator [] (size_t pos) thread
    {
        return elements[pos];
    }
    constexpr const thread T& operator [] (size_t pos) const thread
    {
        return elements[pos];
    }
    
    device T& operator [] (size_t pos) device
    {
        return elements[pos];
    }
    constexpr const device T& operator [] (size_t pos) const device
    {
        return elements[pos];
    }
    
    constexpr const constant T& operator [] (size_t pos) const constant
    {
        return elements[pos];
    }
    
    threadgroup T& operator [] (size_t pos) threadgroup
    {
        return elements[pos];
    }
    constexpr const threadgroup T& operator [] (size_t pos) const threadgroup
    {
        return elements[pos];
    }
};

struct PushConstants
{
    packed_int3 axisT;
    packed_int3 volDim;
    int y;
};

constant uint3 gl_WorkGroupSize [[maybe_unused]] = uint3(8u, 8u, 1u);

struct spvDescriptorSetBuffer0
{
    texture3d<float> volume_d [[id(0)]];
    texture2d<float, access::write> slice_d [[id(1)]];
};

kernel void main0(constant spvDescriptorSetBuffer0& spvDescriptorSet0 [[buffer(0)]], constant PushConstants& pushConstants [[buffer(1)]], uint3 gl_GlobalInvocationID [[thread_position_in_grid]])
{
    uint x = gl_GlobalInvocationID.x;
    uint z = gl_GlobalInvocationID.y;
    spvUnsafeArray<int, 3> temp;
    temp[pushConstants.axisT[0u]] = int(x);
    temp[pushConstants.axisT[1u]] = pushConstants.y;
    temp[pushConstants.axisT[2u]] = int(z);
    int3 v = int3(temp[0], temp[1], temp[2]);
    bool _66 = int(x) >= pushConstants.volDim[pushConstants.axisT[0u]];
    bool _77;
    if (!_66)
    {
        _77 = int(z) >= pushConstants.volDim[pushConstants.axisT[2u]];
    }
    else
    {
        _77 = _66;
    }
    if (_77)
    {
        return;
    }
    float4 volume_xyz = spvDescriptorSet0.volume_d.read(uint3(v));
    spvDescriptorSet0.slice_d.write(volume_xyz, uint2(int2(int(x), int(z))));
}


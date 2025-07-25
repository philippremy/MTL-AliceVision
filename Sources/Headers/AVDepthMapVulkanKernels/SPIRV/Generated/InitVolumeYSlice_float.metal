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
    float cst;
};

struct spvDescriptorSetBuffer0
{
    texture3d<float, access::read_write> volume_d [[id(0)]];
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
    bool _62 = x >= 0u;
    bool _72;
    if (_62)
    {
        _72 = x < uint(pushConstants.volDim[pushConstants.axisT[0u]]);
    }
    else
    {
        _72 = _62;
    }
    bool _75 = _72 && (z >= 0u);
    bool _85;
    if (_75)
    {
        _85 = z < uint(pushConstants.volDim[pushConstants.axisT[2u]]);
    }
    else
    {
        _85 = _75;
    }
    if (_85)
    {
        spvDescriptorSet0.volume_d.fence();
        float4 volume_zyx = spvDescriptorSet0.volume_d.read(uint3(v));
        volume_zyx.x = pushConstants.cst;
        spvDescriptorSet0.volume_d.write(volume_zyx, uint3(v));
    }
}


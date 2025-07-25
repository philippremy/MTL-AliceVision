#include <metal_stdlib>
#include <simd/simd.h>

using namespace metal;

struct PushConstants
{
    int volDimX;
    int volDimZ;
};

constant uint3 gl_WorkGroupSize [[maybe_unused]] = uint3(64u, 1u, 1u);

struct spvDescriptorSetBuffer0
{
    texture2d<uint> xzSlice_d [[id(0)]];
    texture2d<uint, access::write> ySliceBestInColCst_d [[id(1)]];
};

kernel void main0(constant spvDescriptorSetBuffer0& spvDescriptorSet0 [[buffer(0)]], constant PushConstants& pushConstants [[buffer(1)]], uint3 gl_GlobalInvocationID [[thread_position_in_grid]])
{
    uint x = gl_GlobalInvocationID.x;
    if (x >= uint(pushConstants.volDimX))
    {
        return;
    }
    uint4 bestCst = spvDescriptorSet0.xzSlice_d.read(uint2(int2(int(x), 0)));
    for (int z = 1; z < pushConstants.volDimZ; z++)
    {
        uint4 cst = spvDescriptorSet0.xzSlice_d.read(uint2(int2(int(x), z)));
        bestCst = select(bestCst, cst, bool4(cst.x < bestCst.x));
    }
    spvDescriptorSet0.ySliceBestInColCst_d.write(bestCst, uint2(int2(int(x), 0)));
}


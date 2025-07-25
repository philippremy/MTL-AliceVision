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

struct Range
{
    uint begin;
    uint end;
};

struct ROI
{
    Range x;
    Range y;
};

struct Range_1
{
    uint begin;
    uint end;
};

struct ROI_1
{
    Range_1 x;
    Range_1 y;
};

struct PushConstants
{
    uint rcSgmLevelWidth;
    uint rcSgmLevelHeight;
    float rcMipmapLevel;
    packed_int3 volDim;
    packed_int3 axisT;
    float _step;
    int y;
    float P1;
    float _P2;
    int ySign;
    int filteringIndex;
    ROI_1 roi;
};

constant uint3 gl_WorkGroupSize [[maybe_unused]] = uint3(64u, 1u, 1u);

struct spvDescriptorSetBuffer0
{
    texture2d<uint, access::read_write> xzSliceForY_d [[id(0)]];
    texture2d<float> rcMipmapImage_tex [[id(1)]];
    sampler rcMipmapImage_texSmplr [[id(2)]];
    texture2d<uint> bestSimInYm1_d [[id(3)]];
    texture2d<uint> xzSliceForYm1_d [[id(4)]];
    texture3d<uint, access::read_write> volAgr_d [[id(5)]];
};

static inline __attribute__((always_inline))
float euclideanDist3(float4 x1, float4 x2)
{
    return length(float3(x1.x - x2.x, x1.y - x2.y, x1.z - x2.z));
}

static inline __attribute__((always_inline))
float sigmoid(thread const float& zeroVal, thread const float& endVal, thread const float& sigwidth, thread const float& sigMid, thread const float& xval)
{
    return zeroVal + ((endVal - zeroVal) * (1.0 / (1.0 + exp(10.0 * ((xval - sigMid) / sigwidth)))));
}

static inline __attribute__((always_inline))
float multi_fminf(thread const float& a, thread const float& b, thread const float& c, thread const float& d)
{
    return fast::min(fast::min(fast::min(a, b), c), d);
}

kernel void main0(constant spvDescriptorSetBuffer0& spvDescriptorSet0 [[buffer(0)]], constant PushConstants& pushConstants [[buffer(1)]], uint3 gl_GlobalInvocationID [[thread_position_in_grid]])
{
    uint x = gl_GlobalInvocationID.x;
    uint z = gl_GlobalInvocationID.y;
    spvUnsafeArray<int, 3> temp;
    temp[pushConstants.axisT[0u]] = int(x);
    temp[pushConstants.axisT[1u]] = pushConstants.y;
    temp[pushConstants.axisT[2u]] = int(z);
    int3 v = int3(temp[0], temp[1], temp[2]);
    bool _469 = int(x) >= pushConstants.volDim[pushConstants.axisT[0u]];
    bool _478;
    if (!_469)
    {
        _478 = int(z) >= pushConstants.volDim[2u];
    }
    else
    {
        _478 = _469;
    }
    if (_478)
    {
        return;
    }
    int _486;
    if (pushConstants.axisT[0u] == 0)
    {
        _486 = int(pushConstants.roi.x.begin);
    }
    else
    {
        _486 = int(pushConstants.roi.y.begin);
    }
    int beginX = _486;
    int _502;
    if (pushConstants.axisT[0u] == 0)
    {
        _502 = int(pushConstants.roi.y.begin);
    }
    else
    {
        _502 = int(pushConstants.roi.x.begin);
    }
    int beginY = _502;
    spvDescriptorSet0.xzSliceForY_d.fence();
    uint4 sim_xz = spvDescriptorSet0.xzSliceForY_d.read(uint2(int2(int(x), int(z))));
    float pathCost = 255.0;
    bool _530 = z >= 1u;
    bool _539;
    if (_530)
    {
        _539 = z < uint(pushConstants.volDim[2u] - 1);
    }
    else
    {
        _539 = _530;
    }
    if (_539)
    {
        float P2 = 0.0;
        if (pushConstants._P2 < 0.0)
        {
            P2 = abs(pushConstants._P2);
        }
        else
        {
            int imX0 = (beginX + v.x) * int(pushConstants._step);
            int imY0 = (beginY + v.y) * int(pushConstants._step);
            int imX1 = imX0 - ((pushConstants.ySign * int(pushConstants._step)) * int(pushConstants.axisT[1u] == 0));
            int imY1 = imY0 - ((pushConstants.ySign * int(pushConstants._step)) * int(pushConstants.axisT[1u] == 1));
            float4 gcr0 = spvDescriptorSet0.rcMipmapImage_tex.sample(spvDescriptorSet0.rcMipmapImage_texSmplr, float2((float(imX0) + 0.5) / float(pushConstants.rcSgmLevelWidth), (float(imY0) + 0.5) / float(pushConstants.rcSgmLevelHeight)), level(pushConstants.rcMipmapLevel));
            float4 gcr1 = spvDescriptorSet0.rcMipmapImage_tex.sample(spvDescriptorSet0.rcMipmapImage_texSmplr, float2((float(imX1) + 0.5) / float(pushConstants.rcSgmLevelWidth), (float(imY1) + 0.5) / float(pushConstants.rcSgmLevelHeight)), level(pushConstants.rcMipmapLevel));
            float deltaC = euclideanDist3(gcr0, gcr1);
            float param = 80.0;
            float param_1 = 255.0;
            float param_2 = 80.0;
            float param_3 = pushConstants._P2;
            float param_4 = deltaC;
            P2 = sigmoid(param, param_1, param_2, param_3, param_4);
        }
        uint bestCostInColM1 = spvDescriptorSet0.bestSimInYm1_d.read(uint2(int2(int(x), 0))).x;
        uint pathCostMDM1 = spvDescriptorSet0.xzSliceForYm1_d.read(uint2(int2(int(x), int(z - 1u)))).x;
        uint pathCostMD = spvDescriptorSet0.xzSliceForYm1_d.read(uint2(int2(int(x), int(z)))).x;
        uint pathCostMDP1 = spvDescriptorSet0.xzSliceForYm1_d.read(uint2(int2(int(x), int(z + 1u)))).x;
        float param_5 = float(pathCostMD);
        float param_6 = float(pathCostMDM1) + pushConstants.P1;
        float param_7 = float(pathCostMDP1) + pushConstants.P1;
        float param_8 = float(bestCostInColM1) + P2;
        float minCost = multi_fminf(param_5, param_6, param_7, param_8);
        pathCost = (float(sim_xz.x) + minCost) - float(bestCostInColM1);
    }
    sim_xz.x = uint(int(pathCost));
    spvDescriptorSet0.xzSliceForY_d.write(sim_xz, uint2(int2(int(x), int(z))));
    pathCost = float(int(fast::min(255.0, fast::max(0.0, pathCost))));
    spvDescriptorSet0.volAgr_d.fence();
    uint4 volume_xyz = spvDescriptorSet0.volAgr_d.read(uint3(v));
    float val = ((float(volume_xyz.x) * float(pushConstants.filteringIndex)) + pathCost) / float(pushConstants.filteringIndex + 1);
    spvDescriptorSet0.volAgr_d.write(uint4(uint(val), volume_xyz.y, volume_xyz.z, volume_xyz.w), uint3(v));
}


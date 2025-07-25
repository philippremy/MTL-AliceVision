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

struct DeviceCameraParams
{
    spvUnsafeArray<float, 12> P;
    spvUnsafeArray<float, 9> iP;
    spvUnsafeArray<float, 9> R;
    spvUnsafeArray<float, 9> iR;
    spvUnsafeArray<float, 9> K;
    spvUnsafeArray<float, 9> iK;
    float3 C;
    float3 XVect;
    float3 YVect;
    float3 ZVect;
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
    int rcDeviceCameraParamsId;
    int volDimZ;
    int scaleStep;
    float thicknessMultFactor;
    float maxSimilarity;
    Range_1 depthRange;
    ROI_1 roi;
};

struct DeviceCameraParams_1
{
    float P[12];
    float iP[9];
    float R[9];
    float iR[9];
    float K[9];
    float iK[9];
    packed_float3 C;
    packed_float3 XVect;
    packed_float3 YVect;
    packed_float3 ZVect;
};

struct DeviceCameraParamsConstant
{
    DeviceCameraParams_1 cameraParams[100];
};

constant uint3 gl_WorkGroupSize [[maybe_unused]] = uint3(32u, 32u, 1u);

struct spvDescriptorSetBuffer0
{
    constant DeviceCameraParamsConstant* constantCameraParametersArray_d [[id(0)]];
    texture3d<float> in_volSim_d [[id(1)]];
    texture2d<float, access::write> out_sgmDepthThicknessMap_d [[id(2)]];
    texture2d<float, access::write> out_sgmDepthSimMap_d [[id(3)]];
    texture2d<float> in_depths_d [[id(4)]];
};

static inline __attribute__((always_inline))
uint Range_size(thread const Range& range)
{
    return range.end - range.begin;
}

static inline __attribute__((always_inline))
uint ROI_width(thread const ROI& roi)
{
    Range param = roi.x;
    return Range_size(param);
}

static inline __attribute__((always_inline))
uint ROI_height(thread const ROI& roi)
{
    Range param = roi.y;
    return Range_size(param);
}

static inline __attribute__((always_inline))
float3 M3x3mulV2(spvUnsafeArray<float, 9> M3x3, float2 V)
{
    return float3(((M3x3[0] * V.x) + (M3x3[3] * V.y)) + M3x3[6], ((M3x3[1] * V.x) + (M3x3[4] * V.y)) + M3x3[7], ((M3x3[2] * V.x) + (M3x3[5] * V.y)) + M3x3[8]);
}

static inline __attribute__((always_inline))
float3 linePlaneIntersect(float3 linePoint, float3 lineVect, float3 planePoint, float3 planeNormal)
{
    float k = (dot(planePoint, planeNormal) - dot(planeNormal, linePoint)) / dot(planeNormal, lineVect);
    return linePoint + (lineVect * k);
}

static inline __attribute__((always_inline))
float depthPlaneToDepth(DeviceCameraParams deviceCamParams, float fpPlaneDepth, float2 pix)
{
    float3 planep = deviceCamParams.C + (deviceCamParams.ZVect * fpPlaneDepth);
    float3 v = M3x3mulV2(deviceCamParams.iP, pix);
    v = fast::normalize(v);
    float3 p = linePlaneIntersect(deviceCamParams.C, v, planep, deviceCamParams.ZVect);
    return length(deviceCamParams.C - p);
}

kernel void main0(constant spvDescriptorSetBuffer0& spvDescriptorSet0 [[buffer(0)]], constant PushConstants& pushConstants [[buffer(1)]], uint3 gl_GlobalInvocationID [[thread_position_in_grid]])
{
    uint vx = gl_GlobalInvocationID.x;
    uint vy = gl_GlobalInvocationID.y;
    ROI param;
    param.x.begin = pushConstants.roi.x.begin;
    param.x.end = pushConstants.roi.x.end;
    param.y.begin = pushConstants.roi.y.begin;
    param.y.end = pushConstants.roi.y.end;
    bool _401 = vx >= ROI_width(param);
    bool _423;
    if (!_401)
    {
        ROI param_1;
        param_1.x.begin = pushConstants.roi.x.begin;
        param_1.x.end = pushConstants.roi.x.end;
        param_1.y.begin = pushConstants.roi.y.begin;
        param_1.y.end = pushConstants.roi.y.end;
        _423 = vy >= ROI_height(param_1);
    }
    else
    {
        _423 = _401;
    }
    if (_423)
    {
        return;
    }
    DeviceCameraParams rcDeviceCamParams;
    rcDeviceCamParams.P[0] = (*spvDescriptorSet0.constantCameraParametersArray_d).cameraParams[pushConstants.rcDeviceCameraParamsId].P[0];
    rcDeviceCamParams.P[1] = (*spvDescriptorSet0.constantCameraParametersArray_d).cameraParams[pushConstants.rcDeviceCameraParamsId].P[1];
    rcDeviceCamParams.P[2] = (*spvDescriptorSet0.constantCameraParametersArray_d).cameraParams[pushConstants.rcDeviceCameraParamsId].P[2];
    rcDeviceCamParams.P[3] = (*spvDescriptorSet0.constantCameraParametersArray_d).cameraParams[pushConstants.rcDeviceCameraParamsId].P[3];
    rcDeviceCamParams.P[4] = (*spvDescriptorSet0.constantCameraParametersArray_d).cameraParams[pushConstants.rcDeviceCameraParamsId].P[4];
    rcDeviceCamParams.P[5] = (*spvDescriptorSet0.constantCameraParametersArray_d).cameraParams[pushConstants.rcDeviceCameraParamsId].P[5];
    rcDeviceCamParams.P[6] = (*spvDescriptorSet0.constantCameraParametersArray_d).cameraParams[pushConstants.rcDeviceCameraParamsId].P[6];
    rcDeviceCamParams.P[7] = (*spvDescriptorSet0.constantCameraParametersArray_d).cameraParams[pushConstants.rcDeviceCameraParamsId].P[7];
    rcDeviceCamParams.P[8] = (*spvDescriptorSet0.constantCameraParametersArray_d).cameraParams[pushConstants.rcDeviceCameraParamsId].P[8];
    rcDeviceCamParams.P[9] = (*spvDescriptorSet0.constantCameraParametersArray_d).cameraParams[pushConstants.rcDeviceCameraParamsId].P[9];
    rcDeviceCamParams.P[10] = (*spvDescriptorSet0.constantCameraParametersArray_d).cameraParams[pushConstants.rcDeviceCameraParamsId].P[10];
    rcDeviceCamParams.P[11] = (*spvDescriptorSet0.constantCameraParametersArray_d).cameraParams[pushConstants.rcDeviceCameraParamsId].P[11];
    rcDeviceCamParams.iP[0] = (*spvDescriptorSet0.constantCameraParametersArray_d).cameraParams[pushConstants.rcDeviceCameraParamsId].iP[0];
    rcDeviceCamParams.iP[1] = (*spvDescriptorSet0.constantCameraParametersArray_d).cameraParams[pushConstants.rcDeviceCameraParamsId].iP[1];
    rcDeviceCamParams.iP[2] = (*spvDescriptorSet0.constantCameraParametersArray_d).cameraParams[pushConstants.rcDeviceCameraParamsId].iP[2];
    rcDeviceCamParams.iP[3] = (*spvDescriptorSet0.constantCameraParametersArray_d).cameraParams[pushConstants.rcDeviceCameraParamsId].iP[3];
    rcDeviceCamParams.iP[4] = (*spvDescriptorSet0.constantCameraParametersArray_d).cameraParams[pushConstants.rcDeviceCameraParamsId].iP[4];
    rcDeviceCamParams.iP[5] = (*spvDescriptorSet0.constantCameraParametersArray_d).cameraParams[pushConstants.rcDeviceCameraParamsId].iP[5];
    rcDeviceCamParams.iP[6] = (*spvDescriptorSet0.constantCameraParametersArray_d).cameraParams[pushConstants.rcDeviceCameraParamsId].iP[6];
    rcDeviceCamParams.iP[7] = (*spvDescriptorSet0.constantCameraParametersArray_d).cameraParams[pushConstants.rcDeviceCameraParamsId].iP[7];
    rcDeviceCamParams.iP[8] = (*spvDescriptorSet0.constantCameraParametersArray_d).cameraParams[pushConstants.rcDeviceCameraParamsId].iP[8];
    rcDeviceCamParams.R[0] = (*spvDescriptorSet0.constantCameraParametersArray_d).cameraParams[pushConstants.rcDeviceCameraParamsId].R[0];
    rcDeviceCamParams.R[1] = (*spvDescriptorSet0.constantCameraParametersArray_d).cameraParams[pushConstants.rcDeviceCameraParamsId].R[1];
    rcDeviceCamParams.R[2] = (*spvDescriptorSet0.constantCameraParametersArray_d).cameraParams[pushConstants.rcDeviceCameraParamsId].R[2];
    rcDeviceCamParams.R[3] = (*spvDescriptorSet0.constantCameraParametersArray_d).cameraParams[pushConstants.rcDeviceCameraParamsId].R[3];
    rcDeviceCamParams.R[4] = (*spvDescriptorSet0.constantCameraParametersArray_d).cameraParams[pushConstants.rcDeviceCameraParamsId].R[4];
    rcDeviceCamParams.R[5] = (*spvDescriptorSet0.constantCameraParametersArray_d).cameraParams[pushConstants.rcDeviceCameraParamsId].R[5];
    rcDeviceCamParams.R[6] = (*spvDescriptorSet0.constantCameraParametersArray_d).cameraParams[pushConstants.rcDeviceCameraParamsId].R[6];
    rcDeviceCamParams.R[7] = (*spvDescriptorSet0.constantCameraParametersArray_d).cameraParams[pushConstants.rcDeviceCameraParamsId].R[7];
    rcDeviceCamParams.R[8] = (*spvDescriptorSet0.constantCameraParametersArray_d).cameraParams[pushConstants.rcDeviceCameraParamsId].R[8];
    rcDeviceCamParams.iR[0] = (*spvDescriptorSet0.constantCameraParametersArray_d).cameraParams[pushConstants.rcDeviceCameraParamsId].iR[0];
    rcDeviceCamParams.iR[1] = (*spvDescriptorSet0.constantCameraParametersArray_d).cameraParams[pushConstants.rcDeviceCameraParamsId].iR[1];
    rcDeviceCamParams.iR[2] = (*spvDescriptorSet0.constantCameraParametersArray_d).cameraParams[pushConstants.rcDeviceCameraParamsId].iR[2];
    rcDeviceCamParams.iR[3] = (*spvDescriptorSet0.constantCameraParametersArray_d).cameraParams[pushConstants.rcDeviceCameraParamsId].iR[3];
    rcDeviceCamParams.iR[4] = (*spvDescriptorSet0.constantCameraParametersArray_d).cameraParams[pushConstants.rcDeviceCameraParamsId].iR[4];
    rcDeviceCamParams.iR[5] = (*spvDescriptorSet0.constantCameraParametersArray_d).cameraParams[pushConstants.rcDeviceCameraParamsId].iR[5];
    rcDeviceCamParams.iR[6] = (*spvDescriptorSet0.constantCameraParametersArray_d).cameraParams[pushConstants.rcDeviceCameraParamsId].iR[6];
    rcDeviceCamParams.iR[7] = (*spvDescriptorSet0.constantCameraParametersArray_d).cameraParams[pushConstants.rcDeviceCameraParamsId].iR[7];
    rcDeviceCamParams.iR[8] = (*spvDescriptorSet0.constantCameraParametersArray_d).cameraParams[pushConstants.rcDeviceCameraParamsId].iR[8];
    rcDeviceCamParams.K[0] = (*spvDescriptorSet0.constantCameraParametersArray_d).cameraParams[pushConstants.rcDeviceCameraParamsId].K[0];
    rcDeviceCamParams.K[1] = (*spvDescriptorSet0.constantCameraParametersArray_d).cameraParams[pushConstants.rcDeviceCameraParamsId].K[1];
    rcDeviceCamParams.K[2] = (*spvDescriptorSet0.constantCameraParametersArray_d).cameraParams[pushConstants.rcDeviceCameraParamsId].K[2];
    rcDeviceCamParams.K[3] = (*spvDescriptorSet0.constantCameraParametersArray_d).cameraParams[pushConstants.rcDeviceCameraParamsId].K[3];
    rcDeviceCamParams.K[4] = (*spvDescriptorSet0.constantCameraParametersArray_d).cameraParams[pushConstants.rcDeviceCameraParamsId].K[4];
    rcDeviceCamParams.K[5] = (*spvDescriptorSet0.constantCameraParametersArray_d).cameraParams[pushConstants.rcDeviceCameraParamsId].K[5];
    rcDeviceCamParams.K[6] = (*spvDescriptorSet0.constantCameraParametersArray_d).cameraParams[pushConstants.rcDeviceCameraParamsId].K[6];
    rcDeviceCamParams.K[7] = (*spvDescriptorSet0.constantCameraParametersArray_d).cameraParams[pushConstants.rcDeviceCameraParamsId].K[7];
    rcDeviceCamParams.K[8] = (*spvDescriptorSet0.constantCameraParametersArray_d).cameraParams[pushConstants.rcDeviceCameraParamsId].K[8];
    rcDeviceCamParams.iK[0] = (*spvDescriptorSet0.constantCameraParametersArray_d).cameraParams[pushConstants.rcDeviceCameraParamsId].iK[0];
    rcDeviceCamParams.iK[1] = (*spvDescriptorSet0.constantCameraParametersArray_d).cameraParams[pushConstants.rcDeviceCameraParamsId].iK[1];
    rcDeviceCamParams.iK[2] = (*spvDescriptorSet0.constantCameraParametersArray_d).cameraParams[pushConstants.rcDeviceCameraParamsId].iK[2];
    rcDeviceCamParams.iK[3] = (*spvDescriptorSet0.constantCameraParametersArray_d).cameraParams[pushConstants.rcDeviceCameraParamsId].iK[3];
    rcDeviceCamParams.iK[4] = (*spvDescriptorSet0.constantCameraParametersArray_d).cameraParams[pushConstants.rcDeviceCameraParamsId].iK[4];
    rcDeviceCamParams.iK[5] = (*spvDescriptorSet0.constantCameraParametersArray_d).cameraParams[pushConstants.rcDeviceCameraParamsId].iK[5];
    rcDeviceCamParams.iK[6] = (*spvDescriptorSet0.constantCameraParametersArray_d).cameraParams[pushConstants.rcDeviceCameraParamsId].iK[6];
    rcDeviceCamParams.iK[7] = (*spvDescriptorSet0.constantCameraParametersArray_d).cameraParams[pushConstants.rcDeviceCameraParamsId].iK[7];
    rcDeviceCamParams.iK[8] = (*spvDescriptorSet0.constantCameraParametersArray_d).cameraParams[pushConstants.rcDeviceCameraParamsId].iK[8];
    rcDeviceCamParams.C = float3((*spvDescriptorSet0.constantCameraParametersArray_d).cameraParams[pushConstants.rcDeviceCameraParamsId].C);
    rcDeviceCamParams.XVect = float3((*spvDescriptorSet0.constantCameraParametersArray_d).cameraParams[pushConstants.rcDeviceCameraParamsId].XVect);
    rcDeviceCamParams.YVect = float3((*spvDescriptorSet0.constantCameraParametersArray_d).cameraParams[pushConstants.rcDeviceCameraParamsId].YVect);
    rcDeviceCamParams.ZVect = float3((*spvDescriptorSet0.constantCameraParametersArray_d).cameraParams[pushConstants.rcDeviceCameraParamsId].ZVect);
    float2 pix = float2(float((pushConstants.roi.x.begin + vx) * uint(pushConstants.scaleStep)), float((pushConstants.roi.y.begin + vy) * uint(pushConstants.scaleStep)));
    float bestSim = 255.0;
    int bestZIdx = -1;
    int _613 = int(pushConstants.depthRange.begin);
    for (int vz = _613; vz < int(pushConstants.depthRange.end); vz++)
    {
        float simAtZ = spvDescriptorSet0.in_volSim_d.read(uint3(int3(int(vx), int(vy), vz))).x;
        if (simAtZ < bestSim)
        {
            bestSim = simAtZ;
            bestZIdx = vz;
        }
    }
    bool _649 = bestZIdx == (-1);
    bool _658;
    if (!_649)
    {
        _658 = bestSim > pushConstants.maxSimilarity;
    }
    else
    {
        _658 = _649;
    }
    if (_658)
    {
        spvDescriptorSet0.out_sgmDepthThicknessMap_d.write(float4(-1.0, -1.0, 0.0, 0.0), uint2(int2(int(vx), int(vy))));
        spvDescriptorSet0.out_sgmDepthSimMap_d.write(float4(-1.0, -1.0, 0.0, 0.0), uint2(int2(int(vx), int(vy))));
        return;
    }
    int bestZIdx_m1 = max(0, (bestZIdx - 1));
    int bestZIdx_p1 = min((pushConstants.volDimZ - 1), (bestZIdx + 1));
    float3 depthPlanes = float3(0.0);
    depthPlanes.x = spvDescriptorSet0.in_depths_d.read(uint2(int2(bestZIdx_m1, 0))).x;
    depthPlanes.y = spvDescriptorSet0.in_depths_d.read(uint2(int2(bestZIdx, 0))).x;
    depthPlanes.z = spvDescriptorSet0.in_depths_d.read(uint2(int2(bestZIdx_p1, 0))).x;
    float bestDepth = depthPlaneToDepth(rcDeviceCamParams, depthPlanes.y, pix);
    float bestDepth_m1 = depthPlaneToDepth(rcDeviceCamParams, depthPlanes.x, pix);
    float bestDepth_p1 = depthPlaneToDepth(rcDeviceCamParams, depthPlanes.z, pix);
    float out_bestDepth = bestDepth;
    float out_bestSim = ((bestSim / 255.0) * 2.0) - 1.0;
    float out_bestDepthThickness = fast::max(bestDepth_p1 - out_bestDepth, out_bestDepth - bestDepth_m1) * pushConstants.thicknessMultFactor;
    spvDescriptorSet0.out_sgmDepthThicknessMap_d.write(float4(out_bestDepth, out_bestDepthThickness, 0.0, 0.0), uint2(int2(int(vx), int(vy))));
    spvDescriptorSet0.out_sgmDepthSimMap_d.write(float4(out_bestDepth, out_bestSim, 0.0, 0.0), uint2(int2(int(vx), int(vy))));
}


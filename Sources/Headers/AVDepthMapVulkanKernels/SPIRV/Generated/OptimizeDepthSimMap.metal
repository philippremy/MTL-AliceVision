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

struct SimStat
{
    float xsum;
    float ysum;
    float xxsum;
    float yysum;
    float xysum;
    float count;
    float wsum;
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

struct Patch
{
    float3 p;
    float3 n;
    float3 x;
    float3 y;
    float d;
};

struct DevicePatchPatternSubpart
{
    int nbCoordinates;
    float level0;
    float downscale;
    float weight;
    short isCircle;
    int wsh;
    spvUnsafeArray<float2, 24> coordinates;
};

struct DevicePatchPattern
{
    int nbSubparts;
    spvUnsafeArray<DevicePatchPatternSubpart, 4> subparts;
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
    int rcDeviceCameraParamsId;
    int iter;
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

constant uint3 gl_WorkGroupSize [[maybe_unused]] = uint3(16u, 16u, 1u);

struct spvDescriptorSetBuffer0
{
    constant DeviceCameraParamsConstant* constantCameraParametersArray_d [[id(0)]];
    texture2d<float> in_sgmDepthPixSizeMap_d [[id(1)]];
    texture2d<float> in_refineDepthSimMap_d [[id(2)]];
    texture2d<float, access::read_write> out_optimizeDepthSimMap_d [[id(3)]];
    texture2d<float> depth_tex [[id(4)]];
    sampler depth_texSmplr [[id(5)]];
    texture2d<float> imgVariance_tex [[id(6)]];
    sampler imgVariance_texSmplr [[id(7)]];
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
float3 get3DPointForPixelAndDepthFromRC(DeviceCameraParams deviceCamParams, float2 pix, thread const float& depth)
{
    float3 rpvIn = M3x3mulV2(deviceCamParams.iP, pix);
    float3 rpvOut = fast::normalize(rpvIn);
    return deviceCamParams.C + (rpvOut * depth);
}

static inline __attribute__((always_inline))
float3 closestPointToLine3D(float3 point, float3 linePoint, float3 lineVectNormalized)
{
    return linePoint + (lineVectNormalized * dot(lineVectNormalized, point - linePoint));
}

static inline __attribute__((always_inline))
float angleBetwABandAC(float3 A, float3 B, float3 C)
{
    float3 V1 = B - A;
    float3 V2 = C - A;
    V1 = fast::normalize(V1);
    V2 = fast::normalize(V2);
    float x = ((V1.x * V2.x) + (V1.y * V2.y)) + (V1.z * V2.z);
    float a = acos(x);
    a = isinf(a) ? 0.0 : a;
    return abs(a) / 0.01745329238474369049072265625;
}

static inline __attribute__((always_inline))
float2 getCellSmoothStepEnergy(DeviceCameraParams rcDeviceCamParams, texture2d<float> in_depth_tex, sampler in_depth_texSmplr, float2 cell0, float2 offsetRoi)
{
    float2 outp = float2(0.0, 180.0);
    float d0 = in_depth_tex.sample(in_depth_texSmplr, float2(cell0), level(0.0)).x;
    if (d0 <= 0.0)
    {
        return outp;
    }
    float2 cellL = cell0 + float2(0.0, -1.0);
    float2 cellR = cell0 + float2(0.0, 1.0);
    float2 cellU = cell0 + float2(-1.0, 0.0);
    float2 cellB = cell0 + float2(1.0, 0.0);
    float dL = in_depth_tex.sample(in_depth_texSmplr, float2(cellL.x, cellL.y), level(0.0)).x;
    float dR = in_depth_tex.sample(in_depth_texSmplr, float2(cellR.x, cellR.y), level(0.0)).x;
    float dU = in_depth_tex.sample(in_depth_texSmplr, float2(cellU.x, cellU.y), level(0.0)).x;
    float dB = in_depth_tex.sample(in_depth_texSmplr, float2(cellB.x, cellB.y), level(0.0)).x;
    float param = d0;
    float3 p0 = get3DPointForPixelAndDepthFromRC(rcDeviceCamParams, cell0 + offsetRoi, param);
    float param_1 = dL;
    float3 pL = get3DPointForPixelAndDepthFromRC(rcDeviceCamParams, cellL + offsetRoi, param_1);
    float param_2 = dR;
    float3 pR = get3DPointForPixelAndDepthFromRC(rcDeviceCamParams, cellR + offsetRoi, param_2);
    float param_3 = dU;
    float3 pU = get3DPointForPixelAndDepthFromRC(rcDeviceCamParams, cellU + offsetRoi, param_3);
    float param_4 = dB;
    float3 pB = get3DPointForPixelAndDepthFromRC(rcDeviceCamParams, cellB + offsetRoi, param_4);
    float3 cg = float3(0.0);
    float n = 0.0;
    if (dL > 0.0)
    {
        cg += pL;
        n += 1.0;
    }
    if (dR > 0.0)
    {
        cg += pR;
        n += 1.0;
    }
    if (dU > 0.0)
    {
        cg += pU;
        n += 1.0;
    }
    if (dB > 0.0)
    {
        cg += pB;
        n += 1.0;
    }
    if (n > 1.0)
    {
        cg /= float3(n);
        float3 vcn = rcDeviceCamParams.C - p0;
        vcn = fast::normalize(vcn);
        float3 pS = closestPointToLine3D(cg, p0, vcn);
        outp.x = length(rcDeviceCamParams.C - pS) - d0;
    }
    float e = 0.0;
    n = 0.0;
    if ((dL > 0.0) && (dR > 0.0))
    {
        e = fast::max(e, 180.0 - angleBetwABandAC(p0, pL, pR));
        n += 1.0;
    }
    if ((dU > 0.0) && (dB > 0.0))
    {
        e = fast::max(e, 180.0 - angleBetwABandAC(p0, pU, pB));
        n += 1.0;
    }
    if (n > 0.0)
    {
        outp.y = e;
    }
    return outp;
}

static inline __attribute__((always_inline))
float copysignf(thread const float& x, thread const float& y)
{
    return abs(x) * sign(y);
}

static inline __attribute__((always_inline))
float sigmoid2(thread const float& zeroVal, thread const float& endVal, thread const float& sigwidth, thread const float& sigMid, thread const float& xval)
{
    return zeroVal + ((endVal - zeroVal) * (1.0 / (1.0 + exp(10.0 * ((sigMid - xval) / sigwidth)))));
}

static inline __attribute__((always_inline))
float sigmoid(thread const float& zeroVal, thread const float& endVal, thread const float& sigwidth, thread const float& sigMid, thread const float& xval)
{
    return zeroVal + ((endVal - zeroVal) * (1.0 / (1.0 + exp(10.0 * ((xval - sigMid) / sigwidth)))));
}

kernel void main0(constant spvDescriptorSetBuffer0& spvDescriptorSet0 [[buffer(0)]], constant PushConstants& pushConstants [[buffer(1)]], uint3 gl_GlobalInvocationID [[thread_position_in_grid]])
{
    uint roiX = gl_GlobalInvocationID.x;
    uint roiY = gl_GlobalInvocationID.y;
    ROI param;
    param.x.begin = pushConstants.roi.x.begin;
    param.x.end = pushConstants.roi.x.end;
    param.y.begin = pushConstants.roi.y.begin;
    param.y.end = pushConstants.roi.y.end;
    bool _1767 = roiX >= ROI_width(param);
    bool _1789;
    if (!_1767)
    {
        ROI param_1;
        param_1.x.begin = pushConstants.roi.x.begin;
        param_1.x.end = pushConstants.roi.x.end;
        param_1.y.begin = pushConstants.roi.y.begin;
        param_1.y.end = pushConstants.roi.y.end;
        _1789 = roiY >= ROI_height(param_1);
    }
    else
    {
        _1789 = _1767;
    }
    if (_1789)
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
    float2 sgmDepthPixSize = spvDescriptorSet0.in_sgmDepthPixSizeMap_d.read(uint2(int2(int(roiX), int(roiY)))).xy;
    float sgmDepth = sgmDepthPixSize.x;
    float sgmPixSize = sgmDepthPixSize.y;
    float2 refineDepthSim = spvDescriptorSet0.in_refineDepthSimMap_d.read(uint2(int2(int(roiX), int(roiY)))).xy;
    float refineDepth = refineDepthSim.x;
    float refineSim = refineDepthSim.y;
    spvDescriptorSet0.out_optimizeDepthSimMap_d.fence();
    float4 out_optDepthSimPtr = spvDescriptorSet0.out_optimizeDepthSimMap_d.read(uint2(int2(int(roiX), int(roiY))));
    float2 _1997;
    if (pushConstants.iter == 0)
    {
        _1997 = float2(sgmDepth, refineSim);
    }
    else
    {
        _1997 = out_optDepthSimPtr.xy;
    }
    float2 out_optDepthSim = _1997;
    float depthOpt = out_optDepthSim.x;
    if (depthOpt > 0.0)
    {
        float2 depthSmoothStepEnergy = getCellSmoothStepEnergy(rcDeviceCamParams, spvDescriptorSet0.depth_tex, spvDescriptorSet0.depth_texSmplr, float2(float(roiX), float(roiY)), float2(float(pushConstants.roi.x.begin), float(pushConstants.roi.y.begin)));
        float stepToSmoothDepth = depthSmoothStepEnergy.x;
        float param_2 = fast::min(abs(stepToSmoothDepth), sgmPixSize / 10.0);
        float param_3 = stepToSmoothDepth;
        stepToSmoothDepth = copysignf(param_2, param_3);
        float depthEnergy = depthSmoothStepEnergy.y;
        float stepToFineDM = refineDepth - depthOpt;
        float param_4 = fast::min(abs(stepToFineDM), sgmPixSize / 10.0);
        float param_5 = stepToFineDM;
        stepToFineDM = copysignf(param_4, param_5);
        float stepToRoughDM = sgmDepth - depthOpt;
        float imgColorVariance = spvDescriptorSet0.imgVariance_tex.sample(spvDescriptorSet0.imgVariance_texSmplr, float2(float(roiX), float(roiY)), level(0.0)).x;
        float param_6 = 5.0;
        float param_7 = 30.0;
        float param_8 = 40.0;
        float param_9 = 20.0;
        float param_10 = imgColorVariance;
        float weightedColorVariance = sigmoid2(param_6, param_7, param_8, param_9, param_10);
        float param_11 = 0.0;
        float param_12 = 1.0;
        float param_13 = 0.699999988079071044921875;
        float param_14 = -0.699999988079071044921875;
        float param_15 = refineSim;
        float fineSimWeight = sigmoid(param_11, param_12, param_13, param_14, param_15);
        float param_16 = 0.0;
        float param_17 = 1.0;
        float param_18 = 30.0;
        float param_19 = weightedColorVariance;
        float param_20 = depthEnergy;
        float energyLowerThanVarianceWeight = sigmoid(param_16, param_17, param_18, param_19, param_20);
        float param_21 = 0.0;
        float param_22 = 1.0;
        float param_23 = 10.0;
        float param_24 = 17.0;
        float param_25 = abs(stepToRoughDM / sgmPixSize);
        float closeToRoughWeight = 1.0 - sigmoid(param_21, param_22, param_23, param_24, param_25);
        float depthOptStep = (closeToRoughWeight * stepToRoughDM) + ((1.0 - closeToRoughWeight) * (((energyLowerThanVarianceWeight * fineSimWeight) * stepToFineDM) + ((1.0 - energyLowerThanVarianceWeight) * stepToSmoothDepth)));
        out_optDepthSim.x = depthOpt + depthOptStep;
        out_optDepthSim.y = (1.0 - closeToRoughWeight) * (((energyLowerThanVarianceWeight * fineSimWeight) * refineSim) + ((1.0 - energyLowerThanVarianceWeight) * (depthEnergy / 20.0)));
    }
    out_optDepthSimPtr.x = out_optDepthSim.x;
    out_optDepthSimPtr.y = out_optDepthSim.y;
    spvDescriptorSet0.out_optimizeDepthSimMap_d.write(out_optDepthSimPtr, uint2(int2(int(roiX), int(roiY))));
}


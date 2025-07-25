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
    int tcDeviceCameraParamsId;
    uint rcRefineLevelWidth;
    uint rcRefineLevelHeight;
    uint tcRefineLevelWidth;
    uint tcRefineLevelHeight;
    float rcMipmapLevel;
    int volDimZ;
    int stepXY;
    int wsh;
    float invGammaC;
    float invGammaP;
    uint useConsistentScale;
    uint useCustomPatchPattern;
    uint useNormalMap;
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

struct DevicePatchPatternSubpart_1
{
    int nbCoordinates;
    float level0;
    float downscale;
    float weight;
    uint isCircle;
    int wsh;
    packed_float2 coordinates[24];
};

struct DevicePatchPattern_1
{
    int nbSubparts;
    DevicePatchPatternSubpart_1 subparts[4];
};

struct DevicePatchPatternConstant
{
    DevicePatchPattern_1 patchPattern;
};

constant uint3 gl_WorkGroupSize [[maybe_unused]] = uint3(32u, 32u, 1u);

struct spvDescriptorSetBuffer0
{
    constant DeviceCameraParamsConstant* constantCameraParametersArray_d [[id(0)]];
    texture2d<float> in_sgmDepthPixSizeMap_d [[id(1)]];
    texture2d<float> in_sgmNormalMap_d [[id(2)]];
    texture2d<float> rcMipmapImage_tex [[id(3)]];
    sampler rcMipmapImage_texSmplr [[id(4)]];
    texture2d<float> tcMipmapImage_tex [[id(5)]];
    sampler tcMipmapImage_texSmplr [[id(6)]];
    constant DevicePatchPatternConstant* constantPatchPattern_d [[id(7)]];
    texture3d<float, access::read_write> inout_volSim_d [[id(8)]];
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
void move3DPointByRcPixSize(thread float3& p, DeviceCameraParams rcDeviceCamParams, float rcPixSize)
{
    float3 rpv = p - rcDeviceCamParams.C;
    rpv = fast::normalize(rpv);
    p += (rpv * rcPixSize);
}

static inline __attribute__((always_inline))
float3 M3x4mulV3(spvUnsafeArray<float, 12> M3x4, float3 V)
{
    return float3((((M3x4[0] * V.x) + (M3x4[3] * V.y)) + (M3x4[6] * V.z)) + M3x4[9], (((M3x4[1] * V.x) + (M3x4[4] * V.y)) + (M3x4[7] * V.z)) + M3x4[10], (((M3x4[2] * V.x) + (M3x4[5] * V.y)) + (M3x4[8] * V.z)) + M3x4[11]);
}

static inline __attribute__((always_inline))
float2 project3DPoint(spvUnsafeArray<float, 12> M3x4, float3 V)
{
    float3 p = M3x4mulV3(M3x4, V);
    float pzInv = 1.0 / p.z;
    return float2(p.x * pzInv, p.y * pzInv);
}

static inline __attribute__((always_inline))
float pointLineDistance3D(float3 point, float3 linePoint, float3 lineVectNormalized)
{
    return length(cross(lineVectNormalized, linePoint - point));
}

static inline __attribute__((always_inline))
float computePixSize(DeviceCameraParams deviceCamParams, float3 p)
{
    float2 rp = project3DPoint(deviceCamParams.P, p);
    float2 rp1 = rp + float2(1.0, 0.0);
    float3 refvect = M3x3mulV2(deviceCamParams.iP, rp1);
    refvect = fast::normalize(refvect);
    return pointLineDistance3D(p, deviceCamParams.C, refvect);
}

static inline __attribute__((always_inline))
float dist(float3 a, float3 b)
{
    return length(a - b);
}

static inline __attribute__((always_inline))
void computeRcTcMipmapLevels(thread float& out_rcMipmapLevel, thread float& out_tcMipmapLevel, float mipmapLevel, DeviceCameraParams rcDeviceCamParams, DeviceCameraParams tcDeviceCamParams, float2 rp0, float2 tp0, float3 p0)
{
    float rcDepth = length(rcDeviceCamParams.C - p0);
    float tcDepth = length(tcDeviceCamParams.C - p0);
    float2 rp1 = rp0 + float2(1.0, 0.0);
    float2 tp1 = tp0 + float2(1.0, 0.0);
    float3 rpv = M3x3mulV2(rcDeviceCamParams.iP, rp1);
    rpv = fast::normalize(rpv);
    float3 prp1 = rcDeviceCamParams.C + (rpv * rcDepth);
    float3 tpv = M3x3mulV2(tcDeviceCamParams.iP, tp1);
    tpv = fast::normalize(tpv);
    float3 ptp1 = tcDeviceCamParams.C + (tpv * tcDepth);
    float rcDist = dist(p0, prp1);
    float tcDist = dist(p0, ptp1);
    float distFactor = rcDist / tcDist;
    if (distFactor < 1.0)
    {
        out_tcMipmapLevel = mipmapLevel - log2(1.0 / distFactor);
        if (out_tcMipmapLevel < 0.0)
        {
            out_rcMipmapLevel = mipmapLevel + abs(out_tcMipmapLevel);
            out_tcMipmapLevel = 0.0;
        }
    }
    else
    {
        out_rcMipmapLevel = mipmapLevel;
        out_tcMipmapLevel = mipmapLevel + log2(distFactor);
    }
}

static inline __attribute__((always_inline))
float euclideanDist3(float4 x1, float4 x2)
{
    return length(float3(x1.x - x2.x, x1.y - x2.y, x1.z - x2.z));
}

static inline __attribute__((always_inline))
float CostYKfromLab(float4 c1, float4 c2, float invGammaC)
{
    float deltaC = euclideanDist3(c1, c2);
    return exp(-(deltaC * invGammaC));
}

static inline __attribute__((always_inline))
void SimStat_update(thread SimStat& sst, float gx, float gy, thread const float& w)
{
    sst.wsum += w;
    sst.count += 1.0;
    sst.xsum += (w * gx);
    sst.ysum += (w * gy);
    sst.xxsum += ((w * gx) * gx);
    sst.yysum += ((w * gy) * gy);
    sst.xysum += ((w * gx) * gy);
}

static inline __attribute__((always_inline))
float CostYKfromLab(int dx, int dy, float4 c1, float4 c2, float invGammaC, float invGammaP)
{
    float deltaC = euclideanDist3(c1, c2);
    deltaC *= invGammaC;
    float deltaP = sqrt(float((dx * dx) + (dy * dy)));
    deltaP *= invGammaP;
    deltaC += deltaP;
    return exp(-deltaC);
}

static inline __attribute__((always_inline))
float SimStat_getVarianceXYW(thread const SimStat& sst)
{
    return (sst.xysum - ((sst.xsum * sst.ysum) / sst.wsum)) / sst.wsum;
}

static inline __attribute__((always_inline))
float SimStat_getVarianceXW(thread const SimStat& sst)
{
    return (sst.xxsum - ((sst.xsum * sst.xsum) / sst.wsum)) / sst.wsum;
}

static inline __attribute__((always_inline))
float SimStat_getVarianceYW(thread const SimStat& sst)
{
    return (sst.yysum - ((sst.ysum * sst.ysum) / sst.wsum)) / sst.wsum;
}

static inline __attribute__((always_inline))
float SimStat_computeWSim(thread SimStat& sst)
{
    SimStat param = sst;
    sst = param;
    SimStat param_1 = sst;
    sst = param_1;
    SimStat param_2 = sst;
    sst = param_2;
    float rawSim = SimStat_getVarianceXYW(param) / sqrt(SimStat_getVarianceXW(param_1) * SimStat_getVarianceYW(param_2));
    if (rawSim == as_type<float>(0x7f800000u /* inf */))
    {
        return -rawSim;
    }
    else
    {
        return 1.0;
    }
}

static inline __attribute__((always_inline))
float sigmoid(thread const float& zeroVal, thread const float& endVal, thread const float& sigwidth, thread const float& sigMid, thread const float& xval)
{
    return zeroVal + ((endVal - zeroVal) * (1.0 / (1.0 + exp(10.0 * ((xval - sigMid) / sigwidth)))));
}

static inline __attribute__((always_inline))
float compNCCby3DptsYK_customPatchPattern(DeviceCameraParams rcDeviceCamParams, DeviceCameraParams tcDeviceCamParams, texture2d<float> rcMipmapImage_tex, sampler rcMipmapImage_texSmplr, texture2d<float> tcMipmapImage_tex, sampler tcMipmapImage_texSmplr, uint rcLevelWidth, uint rcLevelHeight, uint tcLevelWidth, uint tcLevelHeight, float mipmapLevel, float invGammaC, float invGammaP, bool useConsistentScale, Patch pat, bool TInvertAndFilter, DevicePatchPattern constantPatchPattern_d)
{
    float2 rp = project3DPoint(rcDeviceCamParams.P, pat.p);
    float2 tp = project3DPoint(tcDeviceCamParams.P, pat.p);
    bool _1059 = rp.x < 2.0;
    bool _1069;
    if (!_1059)
    {
        _1069 = rp.x > (float(rcLevelWidth - 1u) - 2.0);
    }
    else
    {
        _1069 = _1059;
    }
    bool _1076;
    if (!_1069)
    {
        _1076 = tp.x < 2.0;
    }
    else
    {
        _1076 = _1069;
    }
    bool _1086;
    if (!_1076)
    {
        _1086 = tp.x > (float(tcLevelWidth - 1u) - 2.0);
    }
    else
    {
        _1086 = _1076;
    }
    bool _1093;
    if (!_1086)
    {
        _1093 = rp.y < 2.0;
    }
    else
    {
        _1093 = _1086;
    }
    bool _1103;
    if (!_1093)
    {
        _1103 = rp.y > (float(rcLevelHeight - 1u) - 2.0);
    }
    else
    {
        _1103 = _1093;
    }
    bool _1110;
    if (!_1103)
    {
        _1110 = tp.y < 2.0;
    }
    else
    {
        _1110 = _1103;
    }
    bool _1120;
    if (!_1110)
    {
        _1120 = tp.y > (float(tcLevelHeight - 1u) - 2.0);
    }
    else
    {
        _1120 = _1110;
    }
    if (_1120)
    {
        return as_type<float>(0x7f800000u /* inf */);
    }
    float rcInvLevelWidth = 1.0 / float(rcLevelWidth);
    float rcInvLevelHeight = 1.0 / float(rcLevelHeight);
    float tcInvLevelWidth = 1.0 / float(tcLevelWidth);
    float tcInvLevelHeight = 1.0 / float(tcLevelHeight);
    float rcAlpha = rcMipmapImage_tex.sample(rcMipmapImage_texSmplr, float2((rp.x + 0.5) * rcInvLevelWidth, (rp.y + 0.5) * rcInvLevelHeight), level(mipmapLevel)).w;
    float tcAlpha = tcMipmapImage_tex.sample(tcMipmapImage_texSmplr, float2((tp.x + 0.5) * tcInvLevelWidth, (tp.y + 0.5) * tcInvLevelHeight), level(mipmapLevel)).w;
    if ((rcAlpha < 229.5) || (tcAlpha < 102.0))
    {
        return as_type<float>(0x7f800000u /* inf */);
    }
    float rcMipmapLevel = mipmapLevel;
    float tcMipmapLevel = mipmapLevel;
    if (useConsistentScale)
    {
        float param = rcMipmapLevel;
        float param_1 = tcMipmapLevel;
        computeRcTcMipmapLevels(param, param_1, mipmapLevel, rcDeviceCamParams, tcDeviceCamParams, rp, tp, pat.p);
        rcMipmapLevel = param;
        tcMipmapLevel = param_1;
    }
    float fsim = 0.0;
    float wsum = 0.0;
    for (int s = 0; s < constantPatchPattern_d.nbSubparts; s++)
    {
        SimStat sst = SimStat{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
        DevicePatchPattern indexable = constantPatchPattern_d;
        DevicePatchPatternSubpart subpart = indexable.subparts[s];
        float4 rcCenterColor = rcMipmapImage_tex.sample(rcMipmapImage_texSmplr, float2((rp.x + 0.5) * rcInvLevelWidth, (rp.y + 0.5) * rcInvLevelHeight), level(rcMipmapLevel + subpart.level0));
        float4 tcCenterColor = tcMipmapImage_tex.sample(tcMipmapImage_texSmplr, float2((tp.x + 0.5) * tcInvLevelWidth, (tp.y + 0.5) * tcInvLevelHeight), level(tcMipmapLevel + subpart.level0));
        if (bool(subpart.isCircle))
        {
            for (int c = 0; c < subpart.nbCoordinates; c++)
            {
                float2 relativeCoord = subpart.coordinates[c];
                float3 p = (pat.p + (pat.x * (pat.d * relativeCoord.x))) + (pat.y * (pat.d * relativeCoord.y));
                float2 rpc = project3DPoint(rcDeviceCamParams.P, p);
                float2 tpc = project3DPoint(tcDeviceCamParams.P, p);
                float4 rcPatchCoordColor = rcMipmapImage_tex.sample(rcMipmapImage_texSmplr, float2((rpc.x + 0.5) * rcInvLevelWidth, (rpc.y + 0.5) * rcInvLevelHeight), level(rcMipmapLevel + subpart.level0));
                float4 tcPatchCoordColor = tcMipmapImage_tex.sample(tcMipmapImage_texSmplr, float2((tpc.x + 0.5) * tcInvLevelWidth, (tpc.y + 0.5) * tcInvLevelHeight), level(tcMipmapLevel + subpart.level0));
                float w = CostYKfromLab(rcCenterColor, rcPatchCoordColor, invGammaC) * CostYKfromLab(tcCenterColor, tcPatchCoordColor, invGammaC);
                SimStat param_2 = sst;
                float param_3 = w;
                SimStat_update(param_2, rcPatchCoordColor.x, tcPatchCoordColor.x, param_3);
                sst = param_2;
            }
        }
        else
        {
            int _1346 = -subpart.wsh;
            for (int yp = _1346; yp <= subpart.wsh; yp++)
            {
                int _1359 = -subpart.wsh;
                for (int xp = _1359; xp <= subpart.wsh; xp++)
                {
                    float3 p_1 = (pat.p + (pat.x * ((pat.d * float(xp)) * subpart.downscale))) + (pat.y * ((pat.d * float(yp)) * subpart.downscale));
                    float2 rpc_1 = project3DPoint(rcDeviceCamParams.P, p_1);
                    float2 tpc_1 = project3DPoint(tcDeviceCamParams.P, p_1);
                    float4 rcPatchCoordColor_1 = rcMipmapImage_tex.sample(rcMipmapImage_texSmplr, float2((rpc_1.x + 0.5) * rcInvLevelWidth, (rpc_1.y + 0.5) * rcInvLevelHeight), level(rcMipmapLevel + subpart.level0));
                    float4 tcPatchCoordColor_1 = tcMipmapImage_tex.sample(tcMipmapImage_texSmplr, float2((tpc_1.x + 0.5) * tcInvLevelWidth, (tpc_1.y + 0.5) * tcInvLevelHeight), level(tcMipmapLevel + subpart.level0));
                    float w_1 = CostYKfromLab(xp, yp, rcCenterColor, rcPatchCoordColor_1, invGammaC, invGammaP) * CostYKfromLab(xp, yp, tcCenterColor, tcPatchCoordColor_1, invGammaC, invGammaP);
                    SimStat param_4 = sst;
                    float param_5 = w_1;
                    SimStat_update(param_4, rcPatchCoordColor_1.x, tcPatchCoordColor_1.x, param_5);
                    sst = param_4;
                }
            }
        }
        SimStat param_6 = sst;
        float _1464 = SimStat_computeWSim(param_6);
        sst = param_6;
        float fsimSubpart = _1464;
        if (fsimSubpart < 0.0)
        {
            if (TInvertAndFilter)
            {
                float param_7 = 0.0;
                float param_8 = 1.0;
                float param_9 = 0.699999988079071044921875;
                float param_10 = -0.699999988079071044921875;
                float param_11 = fsimSubpart;
                float fsimInverted = sigmoid(param_7, param_8, param_9, param_10, param_11);
                fsim += (fsimInverted * subpart.weight);
            }
            else
            {
                fsim += (fsimSubpart * subpart.weight);
            }
            wsum += subpart.weight;
        }
    }
    if (wsum == 0.0)
    {
        return as_type<float>(0x7f800000u /* inf */);
    }
    if (TInvertAndFilter)
    {
        return fsim;
    }
    return fsim / wsum;
}

static inline __attribute__((always_inline))
float compNCCby3DptsYK(DeviceCameraParams rcDeviceCamParams, DeviceCameraParams tcDeviceCamParams, texture2d<float> rcMipmapImage_tex, sampler rcMipmapImage_texSmplr, texture2d<float> tcMipmapImage_tex, sampler tcMipmapImage_texSmplr, uint rcLevelWidth, uint rcLevelHeight, uint tcLevelWidth, uint tcLevelHeight, float mipmapLevel, int wsh, float invGammaC, float invGammaP, bool useConsistentScale, Patch pat, bool TInvertAndFilter)
{
    float2 rp = project3DPoint(rcDeviceCamParams.P, pat.p);
    float2 tp = project3DPoint(tcDeviceCamParams.P, pat.p);
    float dd = float(wsh) + 2.0;
    bool _779 = rp.x < dd;
    bool _790;
    if (!_779)
    {
        _790 = rp.x > (float(rcLevelWidth - 1u) - dd);
    }
    else
    {
        _790 = _779;
    }
    bool _798;
    if (!_790)
    {
        _798 = tp.x < dd;
    }
    else
    {
        _798 = _790;
    }
    bool _809;
    if (!_798)
    {
        _809 = tp.x > (float(tcLevelWidth - 1u) - dd);
    }
    else
    {
        _809 = _798;
    }
    bool _817;
    if (!_809)
    {
        _817 = rp.y < dd;
    }
    else
    {
        _817 = _809;
    }
    bool _828;
    if (!_817)
    {
        _828 = rp.y > (float(rcLevelHeight - 1u) - dd);
    }
    else
    {
        _828 = _817;
    }
    bool _836;
    if (!_828)
    {
        _836 = tp.y < dd;
    }
    else
    {
        _836 = _828;
    }
    bool _847;
    if (!_836)
    {
        _847 = tp.y > (float(tcLevelHeight - 1u) - dd);
    }
    else
    {
        _847 = _836;
    }
    if (_847)
    {
        return as_type<float>(0x7f800000u /* inf */);
    }
    float rcInvLevelWidth = 1.0 / float(rcLevelWidth);
    float rcInvLevelHeight = 1.0 / float(rcLevelHeight);
    float tcInvLevelWidth = 1.0 / float(tcLevelWidth);
    float tcInvLevelHeight = 1.0 / float(tcLevelHeight);
    float rcMipmapLevel = mipmapLevel;
    float tcMipmapLevel = mipmapLevel;
    if (useConsistentScale)
    {
        float param = rcMipmapLevel;
        float param_1 = tcMipmapLevel;
        computeRcTcMipmapLevels(param, param_1, mipmapLevel, rcDeviceCamParams, tcDeviceCamParams, rp, tp, pat.p);
        rcMipmapLevel = param;
        tcMipmapLevel = param_1;
    }
    SimStat sst = SimStat{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    float4 rcCenterColor = rcMipmapImage_tex.sample(rcMipmapImage_texSmplr, float2((rp.x + 0.5) * rcInvLevelWidth, (rp.y + 0.5) * rcInvLevelHeight), level(rcMipmapLevel));
    float4 tcCenterColor = tcMipmapImage_tex.sample(tcMipmapImage_texSmplr, float2((tp.x + 0.5) * tcInvLevelWidth, (tp.y + 0.5) * tcInvLevelHeight), level(tcMipmapLevel));
    bool _915 = rcCenterColor.w < 229.5;
    bool _923;
    if (!_915)
    {
        _923 = tcCenterColor.w < 102.0;
    }
    else
    {
        _923 = _915;
    }
    if (_923)
    {
        return as_type<float>(0x7f800000u /* inf */);
    }
    int _929 = -wsh;
    for (int yp = _929; yp <= wsh; yp++)
    {
        int _938 = -wsh;
        for (int xp = _938; xp <= wsh; xp++)
        {
            float3 p = (pat.p + (pat.x * (pat.d * float(xp)))) + (pat.y * (pat.d * float(yp)));
            float2 rpc = project3DPoint(rcDeviceCamParams.P, p);
            float2 tpc = project3DPoint(tcDeviceCamParams.P, p);
            float4 rcPatchCoordColor = rcMipmapImage_tex.sample(rcMipmapImage_texSmplr, float2((rpc.x + 0.5) * rcInvLevelWidth, (rpc.y + 0.5) * rcInvLevelHeight), level(rcMipmapLevel));
            float4 tcPatchCoordColor = tcMipmapImage_tex.sample(tcMipmapImage_texSmplr, float2((tpc.x + 0.5) * tcInvLevelWidth, (tpc.y + 0.5) * tcInvLevelHeight), level(tcMipmapLevel));
            float w = CostYKfromLab(xp, yp, rcCenterColor, rcPatchCoordColor, invGammaC, invGammaP) * CostYKfromLab(xp, yp, tcCenterColor, tcPatchCoordColor, invGammaC, invGammaP);
            SimStat param_2 = sst;
            float param_3 = w;
            SimStat_update(param_2, rcPatchCoordColor.x, tcPatchCoordColor.x, param_3);
            sst = param_2;
        }
    }
    if (TInvertAndFilter)
    {
        SimStat param_4 = sst;
        float _1031 = SimStat_computeWSim(param_4);
        sst = param_4;
        float fsim = _1031;
        float param_5 = 0.0;
        float param_6 = 1.0;
        float param_7 = 0.699999988079071044921875;
        float param_8 = -0.699999988079071044921875;
        float param_9 = fsim;
        return sigmoid(param_5, param_6, param_7, param_8, param_9);
    }
    SimStat param_10 = sst;
    float _1045 = SimStat_computeWSim(param_10);
    sst = param_10;
    return _1045;
}

kernel void main0(constant spvDescriptorSetBuffer0& spvDescriptorSet0 [[buffer(0)]], constant PushConstants& pushConstants [[buffer(1)]], uint3 gl_GlobalInvocationID [[thread_position_in_grid]], uint3 gl_WorkGroupID [[threadgroup_position_in_grid]])
{
    uint roiX = gl_GlobalInvocationID.x;
    uint roiY = gl_GlobalInvocationID.y;
    uint roiZ = gl_WorkGroupID.z;
    ROI param;
    param.x.begin = pushConstants.roi.x.begin;
    param.x.end = pushConstants.roi.x.end;
    param.y.begin = pushConstants.roi.y.begin;
    param.y.end = pushConstants.roi.y.end;
    bool _1581 = roiX >= ROI_width(param);
    bool _1603;
    if (!_1581)
    {
        ROI param_1;
        param_1.x.begin = pushConstants.roi.x.begin;
        param_1.x.end = pushConstants.roi.x.end;
        param_1.y.begin = pushConstants.roi.y.begin;
        param_1.y.end = pushConstants.roi.y.end;
        _1603 = roiY >= ROI_height(param_1);
    }
    else
    {
        _1603 = _1581;
    }
    if (_1603)
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
    DeviceCameraParams tcDeviceCamParams;
    tcDeviceCamParams.P[0] = (*spvDescriptorSet0.constantCameraParametersArray_d).cameraParams[pushConstants.tcDeviceCameraParamsId].P[0];
    tcDeviceCamParams.P[1] = (*spvDescriptorSet0.constantCameraParametersArray_d).cameraParams[pushConstants.tcDeviceCameraParamsId].P[1];
    tcDeviceCamParams.P[2] = (*spvDescriptorSet0.constantCameraParametersArray_d).cameraParams[pushConstants.tcDeviceCameraParamsId].P[2];
    tcDeviceCamParams.P[3] = (*spvDescriptorSet0.constantCameraParametersArray_d).cameraParams[pushConstants.tcDeviceCameraParamsId].P[3];
    tcDeviceCamParams.P[4] = (*spvDescriptorSet0.constantCameraParametersArray_d).cameraParams[pushConstants.tcDeviceCameraParamsId].P[4];
    tcDeviceCamParams.P[5] = (*spvDescriptorSet0.constantCameraParametersArray_d).cameraParams[pushConstants.tcDeviceCameraParamsId].P[5];
    tcDeviceCamParams.P[6] = (*spvDescriptorSet0.constantCameraParametersArray_d).cameraParams[pushConstants.tcDeviceCameraParamsId].P[6];
    tcDeviceCamParams.P[7] = (*spvDescriptorSet0.constantCameraParametersArray_d).cameraParams[pushConstants.tcDeviceCameraParamsId].P[7];
    tcDeviceCamParams.P[8] = (*spvDescriptorSet0.constantCameraParametersArray_d).cameraParams[pushConstants.tcDeviceCameraParamsId].P[8];
    tcDeviceCamParams.P[9] = (*spvDescriptorSet0.constantCameraParametersArray_d).cameraParams[pushConstants.tcDeviceCameraParamsId].P[9];
    tcDeviceCamParams.P[10] = (*spvDescriptorSet0.constantCameraParametersArray_d).cameraParams[pushConstants.tcDeviceCameraParamsId].P[10];
    tcDeviceCamParams.P[11] = (*spvDescriptorSet0.constantCameraParametersArray_d).cameraParams[pushConstants.tcDeviceCameraParamsId].P[11];
    tcDeviceCamParams.iP[0] = (*spvDescriptorSet0.constantCameraParametersArray_d).cameraParams[pushConstants.tcDeviceCameraParamsId].iP[0];
    tcDeviceCamParams.iP[1] = (*spvDescriptorSet0.constantCameraParametersArray_d).cameraParams[pushConstants.tcDeviceCameraParamsId].iP[1];
    tcDeviceCamParams.iP[2] = (*spvDescriptorSet0.constantCameraParametersArray_d).cameraParams[pushConstants.tcDeviceCameraParamsId].iP[2];
    tcDeviceCamParams.iP[3] = (*spvDescriptorSet0.constantCameraParametersArray_d).cameraParams[pushConstants.tcDeviceCameraParamsId].iP[3];
    tcDeviceCamParams.iP[4] = (*spvDescriptorSet0.constantCameraParametersArray_d).cameraParams[pushConstants.tcDeviceCameraParamsId].iP[4];
    tcDeviceCamParams.iP[5] = (*spvDescriptorSet0.constantCameraParametersArray_d).cameraParams[pushConstants.tcDeviceCameraParamsId].iP[5];
    tcDeviceCamParams.iP[6] = (*spvDescriptorSet0.constantCameraParametersArray_d).cameraParams[pushConstants.tcDeviceCameraParamsId].iP[6];
    tcDeviceCamParams.iP[7] = (*spvDescriptorSet0.constantCameraParametersArray_d).cameraParams[pushConstants.tcDeviceCameraParamsId].iP[7];
    tcDeviceCamParams.iP[8] = (*spvDescriptorSet0.constantCameraParametersArray_d).cameraParams[pushConstants.tcDeviceCameraParamsId].iP[8];
    tcDeviceCamParams.R[0] = (*spvDescriptorSet0.constantCameraParametersArray_d).cameraParams[pushConstants.tcDeviceCameraParamsId].R[0];
    tcDeviceCamParams.R[1] = (*spvDescriptorSet0.constantCameraParametersArray_d).cameraParams[pushConstants.tcDeviceCameraParamsId].R[1];
    tcDeviceCamParams.R[2] = (*spvDescriptorSet0.constantCameraParametersArray_d).cameraParams[pushConstants.tcDeviceCameraParamsId].R[2];
    tcDeviceCamParams.R[3] = (*spvDescriptorSet0.constantCameraParametersArray_d).cameraParams[pushConstants.tcDeviceCameraParamsId].R[3];
    tcDeviceCamParams.R[4] = (*spvDescriptorSet0.constantCameraParametersArray_d).cameraParams[pushConstants.tcDeviceCameraParamsId].R[4];
    tcDeviceCamParams.R[5] = (*spvDescriptorSet0.constantCameraParametersArray_d).cameraParams[pushConstants.tcDeviceCameraParamsId].R[5];
    tcDeviceCamParams.R[6] = (*spvDescriptorSet0.constantCameraParametersArray_d).cameraParams[pushConstants.tcDeviceCameraParamsId].R[6];
    tcDeviceCamParams.R[7] = (*spvDescriptorSet0.constantCameraParametersArray_d).cameraParams[pushConstants.tcDeviceCameraParamsId].R[7];
    tcDeviceCamParams.R[8] = (*spvDescriptorSet0.constantCameraParametersArray_d).cameraParams[pushConstants.tcDeviceCameraParamsId].R[8];
    tcDeviceCamParams.iR[0] = (*spvDescriptorSet0.constantCameraParametersArray_d).cameraParams[pushConstants.tcDeviceCameraParamsId].iR[0];
    tcDeviceCamParams.iR[1] = (*spvDescriptorSet0.constantCameraParametersArray_d).cameraParams[pushConstants.tcDeviceCameraParamsId].iR[1];
    tcDeviceCamParams.iR[2] = (*spvDescriptorSet0.constantCameraParametersArray_d).cameraParams[pushConstants.tcDeviceCameraParamsId].iR[2];
    tcDeviceCamParams.iR[3] = (*spvDescriptorSet0.constantCameraParametersArray_d).cameraParams[pushConstants.tcDeviceCameraParamsId].iR[3];
    tcDeviceCamParams.iR[4] = (*spvDescriptorSet0.constantCameraParametersArray_d).cameraParams[pushConstants.tcDeviceCameraParamsId].iR[4];
    tcDeviceCamParams.iR[5] = (*spvDescriptorSet0.constantCameraParametersArray_d).cameraParams[pushConstants.tcDeviceCameraParamsId].iR[5];
    tcDeviceCamParams.iR[6] = (*spvDescriptorSet0.constantCameraParametersArray_d).cameraParams[pushConstants.tcDeviceCameraParamsId].iR[6];
    tcDeviceCamParams.iR[7] = (*spvDescriptorSet0.constantCameraParametersArray_d).cameraParams[pushConstants.tcDeviceCameraParamsId].iR[7];
    tcDeviceCamParams.iR[8] = (*spvDescriptorSet0.constantCameraParametersArray_d).cameraParams[pushConstants.tcDeviceCameraParamsId].iR[8];
    tcDeviceCamParams.K[0] = (*spvDescriptorSet0.constantCameraParametersArray_d).cameraParams[pushConstants.tcDeviceCameraParamsId].K[0];
    tcDeviceCamParams.K[1] = (*spvDescriptorSet0.constantCameraParametersArray_d).cameraParams[pushConstants.tcDeviceCameraParamsId].K[1];
    tcDeviceCamParams.K[2] = (*spvDescriptorSet0.constantCameraParametersArray_d).cameraParams[pushConstants.tcDeviceCameraParamsId].K[2];
    tcDeviceCamParams.K[3] = (*spvDescriptorSet0.constantCameraParametersArray_d).cameraParams[pushConstants.tcDeviceCameraParamsId].K[3];
    tcDeviceCamParams.K[4] = (*spvDescriptorSet0.constantCameraParametersArray_d).cameraParams[pushConstants.tcDeviceCameraParamsId].K[4];
    tcDeviceCamParams.K[5] = (*spvDescriptorSet0.constantCameraParametersArray_d).cameraParams[pushConstants.tcDeviceCameraParamsId].K[5];
    tcDeviceCamParams.K[6] = (*spvDescriptorSet0.constantCameraParametersArray_d).cameraParams[pushConstants.tcDeviceCameraParamsId].K[6];
    tcDeviceCamParams.K[7] = (*spvDescriptorSet0.constantCameraParametersArray_d).cameraParams[pushConstants.tcDeviceCameraParamsId].K[7];
    tcDeviceCamParams.K[8] = (*spvDescriptorSet0.constantCameraParametersArray_d).cameraParams[pushConstants.tcDeviceCameraParamsId].K[8];
    tcDeviceCamParams.iK[0] = (*spvDescriptorSet0.constantCameraParametersArray_d).cameraParams[pushConstants.tcDeviceCameraParamsId].iK[0];
    tcDeviceCamParams.iK[1] = (*spvDescriptorSet0.constantCameraParametersArray_d).cameraParams[pushConstants.tcDeviceCameraParamsId].iK[1];
    tcDeviceCamParams.iK[2] = (*spvDescriptorSet0.constantCameraParametersArray_d).cameraParams[pushConstants.tcDeviceCameraParamsId].iK[2];
    tcDeviceCamParams.iK[3] = (*spvDescriptorSet0.constantCameraParametersArray_d).cameraParams[pushConstants.tcDeviceCameraParamsId].iK[3];
    tcDeviceCamParams.iK[4] = (*spvDescriptorSet0.constantCameraParametersArray_d).cameraParams[pushConstants.tcDeviceCameraParamsId].iK[4];
    tcDeviceCamParams.iK[5] = (*spvDescriptorSet0.constantCameraParametersArray_d).cameraParams[pushConstants.tcDeviceCameraParamsId].iK[5];
    tcDeviceCamParams.iK[6] = (*spvDescriptorSet0.constantCameraParametersArray_d).cameraParams[pushConstants.tcDeviceCameraParamsId].iK[6];
    tcDeviceCamParams.iK[7] = (*spvDescriptorSet0.constantCameraParametersArray_d).cameraParams[pushConstants.tcDeviceCameraParamsId].iK[7];
    tcDeviceCamParams.iK[8] = (*spvDescriptorSet0.constantCameraParametersArray_d).cameraParams[pushConstants.tcDeviceCameraParamsId].iK[8];
    tcDeviceCamParams.C = float3((*spvDescriptorSet0.constantCameraParametersArray_d).cameraParams[pushConstants.tcDeviceCameraParamsId].C);
    tcDeviceCamParams.XVect = float3((*spvDescriptorSet0.constantCameraParametersArray_d).cameraParams[pushConstants.tcDeviceCameraParamsId].XVect);
    tcDeviceCamParams.YVect = float3((*spvDescriptorSet0.constantCameraParametersArray_d).cameraParams[pushConstants.tcDeviceCameraParamsId].YVect);
    tcDeviceCamParams.ZVect = float3((*spvDescriptorSet0.constantCameraParametersArray_d).cameraParams[pushConstants.tcDeviceCameraParamsId].ZVect);
    uint vx = roiX;
    uint vy = roiY;
    uint vz = pushConstants.depthRange.begin + roiZ;
    float x = float(pushConstants.roi.x.begin + vx) * float(pushConstants.stepXY);
    float y = float(pushConstants.roi.y.begin + vy) * float(pushConstants.stepXY);
    float2 in_sgmDepthPixSize = spvDescriptorSet0.in_sgmDepthPixSizeMap_d.read(uint2(int2(int(vx), int(vy)))).xy;
    if (in_sgmDepthPixSize.x <= 0.0)
    {
        return;
    }
    float param_2 = in_sgmDepthPixSize.x;
    float3 p = get3DPointForPixelAndDepthFromRC(rcDeviceCamParams, float2(x, y), param_2);
    int relativeDepthIndexOffset = int(vz) - ((pushConstants.volDimZ - 1) / 2);
    if (relativeDepthIndexOffset != 0)
    {
        float pixSizeOffset = float(relativeDepthIndexOffset) * in_sgmDepthPixSize.y;
        float3 param_3 = p;
        move3DPointByRcPixSize(param_3, rcDeviceCamParams, pixSizeOffset);
        p = param_3;
    }
    Patch pat = Patch{ float3(0.0), float3(0.0), float3(0.0), float3(0.0), 0.0 };
    pat.p = p;
    pat.d = computePixSize(rcDeviceCamParams, p);
    float3 v1 = rcDeviceCamParams.C - pat.p;
    float3 v2 = tcDeviceCamParams.C - pat.p;
    v1 = fast::normalize(v1);
    v2 = fast::normalize(v2);
    pat.y = cross(v1, v2);
    pat.y = fast::normalize(pat.y);
    if (pushConstants.useNormalMap != 0u)
    {
        pat.n = spvDescriptorSet0.in_sgmNormalMap_d.read(uint2(int2(int(vx), int(vy)))).xyz;
    }
    else
    {
        pat.n = (v1 + v2) / float3(2.0);
        pat.n = fast::normalize(pat.n);
    }
    pat.x = cross(pat.y, pat.n);
    pat.x = fast::normalize(pat.x);
    float fsimInvertedFiltered = as_type<float>(0x7f800000u /* inf */);
    if (pushConstants.useCustomPatchPattern != 0u)
    {
        DevicePatchPattern arg;
        arg.nbSubparts = (*spvDescriptorSet0.constantPatchPattern_d).patchPattern.nbSubparts;
        arg.subparts[0].nbCoordinates = (*spvDescriptorSet0.constantPatchPattern_d).patchPattern.subparts[0].nbCoordinates;
        arg.subparts[0].level0 = (*spvDescriptorSet0.constantPatchPattern_d).patchPattern.subparts[0].level0;
        arg.subparts[0].downscale = (*spvDescriptorSet0.constantPatchPattern_d).patchPattern.subparts[0].downscale;
        arg.subparts[0].weight = (*spvDescriptorSet0.constantPatchPattern_d).patchPattern.subparts[0].weight;
        arg.subparts[0].isCircle = short((*spvDescriptorSet0.constantPatchPattern_d).patchPattern.subparts[0].isCircle != 0u);
        arg.subparts[0].wsh = (*spvDescriptorSet0.constantPatchPattern_d).patchPattern.subparts[0].wsh;
        arg.subparts[0].coordinates[0] = float2((*spvDescriptorSet0.constantPatchPattern_d).patchPattern.subparts[0].coordinates[0]);
        arg.subparts[0].coordinates[1] = float2((*spvDescriptorSet0.constantPatchPattern_d).patchPattern.subparts[0].coordinates[1]);
        arg.subparts[0].coordinates[2] = float2((*spvDescriptorSet0.constantPatchPattern_d).patchPattern.subparts[0].coordinates[2]);
        arg.subparts[0].coordinates[3] = float2((*spvDescriptorSet0.constantPatchPattern_d).patchPattern.subparts[0].coordinates[3]);
        arg.subparts[0].coordinates[4] = float2((*spvDescriptorSet0.constantPatchPattern_d).patchPattern.subparts[0].coordinates[4]);
        arg.subparts[0].coordinates[5] = float2((*spvDescriptorSet0.constantPatchPattern_d).patchPattern.subparts[0].coordinates[5]);
        arg.subparts[0].coordinates[6] = float2((*spvDescriptorSet0.constantPatchPattern_d).patchPattern.subparts[0].coordinates[6]);
        arg.subparts[0].coordinates[7] = float2((*spvDescriptorSet0.constantPatchPattern_d).patchPattern.subparts[0].coordinates[7]);
        arg.subparts[0].coordinates[8] = float2((*spvDescriptorSet0.constantPatchPattern_d).patchPattern.subparts[0].coordinates[8]);
        arg.subparts[0].coordinates[9] = float2((*spvDescriptorSet0.constantPatchPattern_d).patchPattern.subparts[0].coordinates[9]);
        arg.subparts[0].coordinates[10] = float2((*spvDescriptorSet0.constantPatchPattern_d).patchPattern.subparts[0].coordinates[10]);
        arg.subparts[0].coordinates[11] = float2((*spvDescriptorSet0.constantPatchPattern_d).patchPattern.subparts[0].coordinates[11]);
        arg.subparts[0].coordinates[12] = float2((*spvDescriptorSet0.constantPatchPattern_d).patchPattern.subparts[0].coordinates[12]);
        arg.subparts[0].coordinates[13] = float2((*spvDescriptorSet0.constantPatchPattern_d).patchPattern.subparts[0].coordinates[13]);
        arg.subparts[0].coordinates[14] = float2((*spvDescriptorSet0.constantPatchPattern_d).patchPattern.subparts[0].coordinates[14]);
        arg.subparts[0].coordinates[15] = float2((*spvDescriptorSet0.constantPatchPattern_d).patchPattern.subparts[0].coordinates[15]);
        arg.subparts[0].coordinates[16] = float2((*spvDescriptorSet0.constantPatchPattern_d).patchPattern.subparts[0].coordinates[16]);
        arg.subparts[0].coordinates[17] = float2((*spvDescriptorSet0.constantPatchPattern_d).patchPattern.subparts[0].coordinates[17]);
        arg.subparts[0].coordinates[18] = float2((*spvDescriptorSet0.constantPatchPattern_d).patchPattern.subparts[0].coordinates[18]);
        arg.subparts[0].coordinates[19] = float2((*spvDescriptorSet0.constantPatchPattern_d).patchPattern.subparts[0].coordinates[19]);
        arg.subparts[0].coordinates[20] = float2((*spvDescriptorSet0.constantPatchPattern_d).patchPattern.subparts[0].coordinates[20]);
        arg.subparts[0].coordinates[21] = float2((*spvDescriptorSet0.constantPatchPattern_d).patchPattern.subparts[0].coordinates[21]);
        arg.subparts[0].coordinates[22] = float2((*spvDescriptorSet0.constantPatchPattern_d).patchPattern.subparts[0].coordinates[22]);
        arg.subparts[0].coordinates[23] = float2((*spvDescriptorSet0.constantPatchPattern_d).patchPattern.subparts[0].coordinates[23]);
        arg.subparts[1].nbCoordinates = (*spvDescriptorSet0.constantPatchPattern_d).patchPattern.subparts[1].nbCoordinates;
        arg.subparts[1].level0 = (*spvDescriptorSet0.constantPatchPattern_d).patchPattern.subparts[1].level0;
        arg.subparts[1].downscale = (*spvDescriptorSet0.constantPatchPattern_d).patchPattern.subparts[1].downscale;
        arg.subparts[1].weight = (*spvDescriptorSet0.constantPatchPattern_d).patchPattern.subparts[1].weight;
        arg.subparts[1].isCircle = short((*spvDescriptorSet0.constantPatchPattern_d).patchPattern.subparts[1].isCircle != 0u);
        arg.subparts[1].wsh = (*spvDescriptorSet0.constantPatchPattern_d).patchPattern.subparts[1].wsh;
        arg.subparts[1].coordinates[0] = float2((*spvDescriptorSet0.constantPatchPattern_d).patchPattern.subparts[1].coordinates[0]);
        arg.subparts[1].coordinates[1] = float2((*spvDescriptorSet0.constantPatchPattern_d).patchPattern.subparts[1].coordinates[1]);
        arg.subparts[1].coordinates[2] = float2((*spvDescriptorSet0.constantPatchPattern_d).patchPattern.subparts[1].coordinates[2]);
        arg.subparts[1].coordinates[3] = float2((*spvDescriptorSet0.constantPatchPattern_d).patchPattern.subparts[1].coordinates[3]);
        arg.subparts[1].coordinates[4] = float2((*spvDescriptorSet0.constantPatchPattern_d).patchPattern.subparts[1].coordinates[4]);
        arg.subparts[1].coordinates[5] = float2((*spvDescriptorSet0.constantPatchPattern_d).patchPattern.subparts[1].coordinates[5]);
        arg.subparts[1].coordinates[6] = float2((*spvDescriptorSet0.constantPatchPattern_d).patchPattern.subparts[1].coordinates[6]);
        arg.subparts[1].coordinates[7] = float2((*spvDescriptorSet0.constantPatchPattern_d).patchPattern.subparts[1].coordinates[7]);
        arg.subparts[1].coordinates[8] = float2((*spvDescriptorSet0.constantPatchPattern_d).patchPattern.subparts[1].coordinates[8]);
        arg.subparts[1].coordinates[9] = float2((*spvDescriptorSet0.constantPatchPattern_d).patchPattern.subparts[1].coordinates[9]);
        arg.subparts[1].coordinates[10] = float2((*spvDescriptorSet0.constantPatchPattern_d).patchPattern.subparts[1].coordinates[10]);
        arg.subparts[1].coordinates[11] = float2((*spvDescriptorSet0.constantPatchPattern_d).patchPattern.subparts[1].coordinates[11]);
        arg.subparts[1].coordinates[12] = float2((*spvDescriptorSet0.constantPatchPattern_d).patchPattern.subparts[1].coordinates[12]);
        arg.subparts[1].coordinates[13] = float2((*spvDescriptorSet0.constantPatchPattern_d).patchPattern.subparts[1].coordinates[13]);
        arg.subparts[1].coordinates[14] = float2((*spvDescriptorSet0.constantPatchPattern_d).patchPattern.subparts[1].coordinates[14]);
        arg.subparts[1].coordinates[15] = float2((*spvDescriptorSet0.constantPatchPattern_d).patchPattern.subparts[1].coordinates[15]);
        arg.subparts[1].coordinates[16] = float2((*spvDescriptorSet0.constantPatchPattern_d).patchPattern.subparts[1].coordinates[16]);
        arg.subparts[1].coordinates[17] = float2((*spvDescriptorSet0.constantPatchPattern_d).patchPattern.subparts[1].coordinates[17]);
        arg.subparts[1].coordinates[18] = float2((*spvDescriptorSet0.constantPatchPattern_d).patchPattern.subparts[1].coordinates[18]);
        arg.subparts[1].coordinates[19] = float2((*spvDescriptorSet0.constantPatchPattern_d).patchPattern.subparts[1].coordinates[19]);
        arg.subparts[1].coordinates[20] = float2((*spvDescriptorSet0.constantPatchPattern_d).patchPattern.subparts[1].coordinates[20]);
        arg.subparts[1].coordinates[21] = float2((*spvDescriptorSet0.constantPatchPattern_d).patchPattern.subparts[1].coordinates[21]);
        arg.subparts[1].coordinates[22] = float2((*spvDescriptorSet0.constantPatchPattern_d).patchPattern.subparts[1].coordinates[22]);
        arg.subparts[1].coordinates[23] = float2((*spvDescriptorSet0.constantPatchPattern_d).patchPattern.subparts[1].coordinates[23]);
        arg.subparts[2].nbCoordinates = (*spvDescriptorSet0.constantPatchPattern_d).patchPattern.subparts[2].nbCoordinates;
        arg.subparts[2].level0 = (*spvDescriptorSet0.constantPatchPattern_d).patchPattern.subparts[2].level0;
        arg.subparts[2].downscale = (*spvDescriptorSet0.constantPatchPattern_d).patchPattern.subparts[2].downscale;
        arg.subparts[2].weight = (*spvDescriptorSet0.constantPatchPattern_d).patchPattern.subparts[2].weight;
        arg.subparts[2].isCircle = short((*spvDescriptorSet0.constantPatchPattern_d).patchPattern.subparts[2].isCircle != 0u);
        arg.subparts[2].wsh = (*spvDescriptorSet0.constantPatchPattern_d).patchPattern.subparts[2].wsh;
        arg.subparts[2].coordinates[0] = float2((*spvDescriptorSet0.constantPatchPattern_d).patchPattern.subparts[2].coordinates[0]);
        arg.subparts[2].coordinates[1] = float2((*spvDescriptorSet0.constantPatchPattern_d).patchPattern.subparts[2].coordinates[1]);
        arg.subparts[2].coordinates[2] = float2((*spvDescriptorSet0.constantPatchPattern_d).patchPattern.subparts[2].coordinates[2]);
        arg.subparts[2].coordinates[3] = float2((*spvDescriptorSet0.constantPatchPattern_d).patchPattern.subparts[2].coordinates[3]);
        arg.subparts[2].coordinates[4] = float2((*spvDescriptorSet0.constantPatchPattern_d).patchPattern.subparts[2].coordinates[4]);
        arg.subparts[2].coordinates[5] = float2((*spvDescriptorSet0.constantPatchPattern_d).patchPattern.subparts[2].coordinates[5]);
        arg.subparts[2].coordinates[6] = float2((*spvDescriptorSet0.constantPatchPattern_d).patchPattern.subparts[2].coordinates[6]);
        arg.subparts[2].coordinates[7] = float2((*spvDescriptorSet0.constantPatchPattern_d).patchPattern.subparts[2].coordinates[7]);
        arg.subparts[2].coordinates[8] = float2((*spvDescriptorSet0.constantPatchPattern_d).patchPattern.subparts[2].coordinates[8]);
        arg.subparts[2].coordinates[9] = float2((*spvDescriptorSet0.constantPatchPattern_d).patchPattern.subparts[2].coordinates[9]);
        arg.subparts[2].coordinates[10] = float2((*spvDescriptorSet0.constantPatchPattern_d).patchPattern.subparts[2].coordinates[10]);
        arg.subparts[2].coordinates[11] = float2((*spvDescriptorSet0.constantPatchPattern_d).patchPattern.subparts[2].coordinates[11]);
        arg.subparts[2].coordinates[12] = float2((*spvDescriptorSet0.constantPatchPattern_d).patchPattern.subparts[2].coordinates[12]);
        arg.subparts[2].coordinates[13] = float2((*spvDescriptorSet0.constantPatchPattern_d).patchPattern.subparts[2].coordinates[13]);
        arg.subparts[2].coordinates[14] = float2((*spvDescriptorSet0.constantPatchPattern_d).patchPattern.subparts[2].coordinates[14]);
        arg.subparts[2].coordinates[15] = float2((*spvDescriptorSet0.constantPatchPattern_d).patchPattern.subparts[2].coordinates[15]);
        arg.subparts[2].coordinates[16] = float2((*spvDescriptorSet0.constantPatchPattern_d).patchPattern.subparts[2].coordinates[16]);
        arg.subparts[2].coordinates[17] = float2((*spvDescriptorSet0.constantPatchPattern_d).patchPattern.subparts[2].coordinates[17]);
        arg.subparts[2].coordinates[18] = float2((*spvDescriptorSet0.constantPatchPattern_d).patchPattern.subparts[2].coordinates[18]);
        arg.subparts[2].coordinates[19] = float2((*spvDescriptorSet0.constantPatchPattern_d).patchPattern.subparts[2].coordinates[19]);
        arg.subparts[2].coordinates[20] = float2((*spvDescriptorSet0.constantPatchPattern_d).patchPattern.subparts[2].coordinates[20]);
        arg.subparts[2].coordinates[21] = float2((*spvDescriptorSet0.constantPatchPattern_d).patchPattern.subparts[2].coordinates[21]);
        arg.subparts[2].coordinates[22] = float2((*spvDescriptorSet0.constantPatchPattern_d).patchPattern.subparts[2].coordinates[22]);
        arg.subparts[2].coordinates[23] = float2((*spvDescriptorSet0.constantPatchPattern_d).patchPattern.subparts[2].coordinates[23]);
        arg.subparts[3].nbCoordinates = (*spvDescriptorSet0.constantPatchPattern_d).patchPattern.subparts[3].nbCoordinates;
        arg.subparts[3].level0 = (*spvDescriptorSet0.constantPatchPattern_d).patchPattern.subparts[3].level0;
        arg.subparts[3].downscale = (*spvDescriptorSet0.constantPatchPattern_d).patchPattern.subparts[3].downscale;
        arg.subparts[3].weight = (*spvDescriptorSet0.constantPatchPattern_d).patchPattern.subparts[3].weight;
        arg.subparts[3].isCircle = short((*spvDescriptorSet0.constantPatchPattern_d).patchPattern.subparts[3].isCircle != 0u);
        arg.subparts[3].wsh = (*spvDescriptorSet0.constantPatchPattern_d).patchPattern.subparts[3].wsh;
        arg.subparts[3].coordinates[0] = float2((*spvDescriptorSet0.constantPatchPattern_d).patchPattern.subparts[3].coordinates[0]);
        arg.subparts[3].coordinates[1] = float2((*spvDescriptorSet0.constantPatchPattern_d).patchPattern.subparts[3].coordinates[1]);
        arg.subparts[3].coordinates[2] = float2((*spvDescriptorSet0.constantPatchPattern_d).patchPattern.subparts[3].coordinates[2]);
        arg.subparts[3].coordinates[3] = float2((*spvDescriptorSet0.constantPatchPattern_d).patchPattern.subparts[3].coordinates[3]);
        arg.subparts[3].coordinates[4] = float2((*spvDescriptorSet0.constantPatchPattern_d).patchPattern.subparts[3].coordinates[4]);
        arg.subparts[3].coordinates[5] = float2((*spvDescriptorSet0.constantPatchPattern_d).patchPattern.subparts[3].coordinates[5]);
        arg.subparts[3].coordinates[6] = float2((*spvDescriptorSet0.constantPatchPattern_d).patchPattern.subparts[3].coordinates[6]);
        arg.subparts[3].coordinates[7] = float2((*spvDescriptorSet0.constantPatchPattern_d).patchPattern.subparts[3].coordinates[7]);
        arg.subparts[3].coordinates[8] = float2((*spvDescriptorSet0.constantPatchPattern_d).patchPattern.subparts[3].coordinates[8]);
        arg.subparts[3].coordinates[9] = float2((*spvDescriptorSet0.constantPatchPattern_d).patchPattern.subparts[3].coordinates[9]);
        arg.subparts[3].coordinates[10] = float2((*spvDescriptorSet0.constantPatchPattern_d).patchPattern.subparts[3].coordinates[10]);
        arg.subparts[3].coordinates[11] = float2((*spvDescriptorSet0.constantPatchPattern_d).patchPattern.subparts[3].coordinates[11]);
        arg.subparts[3].coordinates[12] = float2((*spvDescriptorSet0.constantPatchPattern_d).patchPattern.subparts[3].coordinates[12]);
        arg.subparts[3].coordinates[13] = float2((*spvDescriptorSet0.constantPatchPattern_d).patchPattern.subparts[3].coordinates[13]);
        arg.subparts[3].coordinates[14] = float2((*spvDescriptorSet0.constantPatchPattern_d).patchPattern.subparts[3].coordinates[14]);
        arg.subparts[3].coordinates[15] = float2((*spvDescriptorSet0.constantPatchPattern_d).patchPattern.subparts[3].coordinates[15]);
        arg.subparts[3].coordinates[16] = float2((*spvDescriptorSet0.constantPatchPattern_d).patchPattern.subparts[3].coordinates[16]);
        arg.subparts[3].coordinates[17] = float2((*spvDescriptorSet0.constantPatchPattern_d).patchPattern.subparts[3].coordinates[17]);
        arg.subparts[3].coordinates[18] = float2((*spvDescriptorSet0.constantPatchPattern_d).patchPattern.subparts[3].coordinates[18]);
        arg.subparts[3].coordinates[19] = float2((*spvDescriptorSet0.constantPatchPattern_d).patchPattern.subparts[3].coordinates[19]);
        arg.subparts[3].coordinates[20] = float2((*spvDescriptorSet0.constantPatchPattern_d).patchPattern.subparts[3].coordinates[20]);
        arg.subparts[3].coordinates[21] = float2((*spvDescriptorSet0.constantPatchPattern_d).patchPattern.subparts[3].coordinates[21]);
        arg.subparts[3].coordinates[22] = float2((*spvDescriptorSet0.constantPatchPattern_d).patchPattern.subparts[3].coordinates[22]);
        arg.subparts[3].coordinates[23] = float2((*spvDescriptorSet0.constantPatchPattern_d).patchPattern.subparts[3].coordinates[23]);
        fsimInvertedFiltered = compNCCby3DptsYK_customPatchPattern(rcDeviceCamParams, tcDeviceCamParams, spvDescriptorSet0.rcMipmapImage_tex, spvDescriptorSet0.rcMipmapImage_texSmplr, spvDescriptorSet0.tcMipmapImage_tex, spvDescriptorSet0.tcMipmapImage_texSmplr, pushConstants.rcRefineLevelWidth, pushConstants.rcRefineLevelHeight, pushConstants.tcRefineLevelWidth, pushConstants.tcRefineLevelHeight, pushConstants.rcMipmapLevel, pushConstants.invGammaC, pushConstants.invGammaP, pushConstants.useConsistentScale != 0u, pat, true, arg);
    }
    else
    {
        fsimInvertedFiltered = compNCCby3DptsYK(rcDeviceCamParams, tcDeviceCamParams, spvDescriptorSet0.rcMipmapImage_tex, spvDescriptorSet0.rcMipmapImage_texSmplr, spvDescriptorSet0.tcMipmapImage_tex, spvDescriptorSet0.tcMipmapImage_texSmplr, pushConstants.rcRefineLevelWidth, pushConstants.rcRefineLevelHeight, pushConstants.tcRefineLevelWidth, pushConstants.tcRefineLevelHeight, pushConstants.rcMipmapLevel, pushConstants.wsh, pushConstants.invGammaC, pushConstants.invGammaP, pushConstants.useConsistentScale != 0u, pat, true);
    }
    if (isinf(fsimInvertedFiltered))
    {
        return;
    }
    spvDescriptorSet0.inout_volSim_d.fence();
    half4 outSimPtr = half4(spvDescriptorSet0.inout_volSim_d.read(uint3(int3(int(vx), int(vy), int(vz)))));
    outSimPtr.x = half(float(outSimPtr.x) + fsimInvertedFiltered);
    spvDescriptorSet0.inout_volSim_d.write(float4(outSimPtr), uint3(int3(int(vx), int(vy), int(vz))));
}


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

struct Stat3D
{
    float xsum;
    float ysum;
    float zsum;
    float xxsum;
    float yysum;
    float zzsum;
    float xysum;
    float xzsum;
    float yzsum;
    float count;
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
    int TWsh;
    int rcDeviceCameraParamsId;
    int stepXY;
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

constant uint3 gl_WorkGroupSize [[maybe_unused]] = uint3(8u, 8u, 1u);

struct spvDescriptorSetBuffer0
{
    constant DeviceCameraParamsConstant* constantCameraParametersArray_d [[id(0)]];
    texture2d<float> in_depthSimMap_d [[id(1)]];
    texture2d<float, access::read_write> out_normalMap_d [[id(2)]];
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
void Stat3D_update(thread Stat3D& stat3D, float3 p, float w)
{
    stat3D.xxsum += (p.x * p.x);
    stat3D.yysum += (p.y * p.y);
    stat3D.zzsum += (p.z * p.z);
    stat3D.xysum += (p.x * p.y);
    stat3D.xzsum += (p.x * p.z);
    stat3D.yzsum += (p.y * p.z);
    stat3D.xsum += p.x;
    stat3D.ysum += p.y;
    stat3D.zsum += p.z;
    stat3D.count += w;
}

static inline __attribute__((always_inline))
void tred2(thread spvUnsafeArray<float, 3>& V0, thread spvUnsafeArray<float, 3>& V1, thread spvUnsafeArray<float, 3>& V2, thread spvUnsafeArray<float, 3>& d, thread spvUnsafeArray<float, 3>& e)
{
    spvUnsafeArray<spvUnsafeArray<float, 3>, 3> V;
    V[0][0] = V0[0];
    V[0][1] = V0[1];
    V[0][2] = V0[2];
    V[1][0] = V1[0];
    V[1][1] = V1[1];
    V[1][2] = V1[2];
    V[2][0] = V2[0];
    V[2][1] = V2[1];
    V[2][2] = V2[2];
    int j = 0;
    for (; j < 3; j++)
    {
        d[j] = V[2][j];
    }
    int i = 2;
    float h;
    int k;
    float g;
    for (; i > 0; i--)
    {
        float scale = 0.0;
        h = 0.0;
        k = 0;
        for (; k < i; k++)
        {
            scale += abs(d[k]);
        }
        if (scale == 0.0)
        {
            e[i] = d[i - 1];
            j = 0;
            for (; j < i; j++)
            {
                d[j] = V[i - 1][j];
                V[i][j] = 0.0;
                V[j][i] = 0.0;
            }
        }
        else
        {
            k = 0;
            for (; k < i; k++)
            {
                d[k] /= scale;
                h += (d[k] * d[k]);
            }
            float f = d[i - 1];
            g = sqrt(h);
            if (f > 0.0)
            {
                g = -g;
            }
            e[i] = scale * g;
            h -= (f * g);
            d[i - 1] = f - g;
            j = 0;
            for (; j < i; j++)
            {
                e[j] = 0.0;
            }
            j = 0;
            for (; j < i; j++)
            {
                f = d[j];
                V[j][i] = f;
                g = e[j] + (V[j][j] * f);
                k = j + 1;
                for (; k <= (i - 1); k++)
                {
                    g += (V[k][j] * d[k]);
                    e[k] += (V[k][j] * f);
                }
                e[j] = g;
            }
            f = 0.0;
            j = 0;
            for (; j < i; j++)
            {
                e[j] /= h;
                f += (e[j] * d[j]);
            }
            float hh = f / (h + h);
            j = 0;
            for (; j < i; j++)
            {
                e[j] -= (hh * d[j]);
            }
            j = 0;
            for (; j < i; j++)
            {
                f = d[j];
                g = e[j];
                k = j;
                for (; k <= (i - 1); k++)
                {
                    V[k][j] -= ((f * e[k]) + (g * d[k]));
                }
                d[j] = V[i - 1][j];
                V[i][j] = 0.0;
            }
        }
        d[i] = h;
    }
    i = 0;
    for (; i < 2; i++)
    {
        V[2][i] = V[i][i];
        V[i][i] = 1.0;
        h = d[i + 1];
        if (h != 0.0)
        {
            k = 0;
            for (; k <= i; k++)
            {
                d[k] = V[k][i + 1] / h;
            }
            j = 0;
            for (; j <= i; j++)
            {
                g = 0.0;
                k = 0;
                for (; k <= i; k++)
                {
                    g += (V[k][i + 1] * V[k][j]);
                }
                k = 0;
                for (; k <= i; k++)
                {
                    V[k][j] -= (g * d[k]);
                }
            }
        }
        k = 0;
        for (; k <= i; k++)
        {
            V[k][i + 1] = 0.0;
        }
    }
    j = 0;
    for (; j < 3; j++)
    {
        d[j] = V[2][j];
        V[2][j] = 0.0;
    }
    V[2][2] = 1.0;
    e[0] = 0.0;
    V0[0] = V[0][0];
    V0[1] = V[0][1];
    V0[2] = V[0][2];
    V1[0] = V[1][0];
    V1[1] = V[1][1];
    V1[2] = V[1][2];
    V2[0] = V[2][0];
    V2[1] = V[2][1];
    V2[2] = V[2][2];
}

static inline __attribute__((always_inline))
float hypot2(thread const float& x, thread const float& y)
{
    return sqrt((x * x) + (y * y));
}

static inline __attribute__((always_inline))
void tql2(thread spvUnsafeArray<float, 3>& V0, thread spvUnsafeArray<float, 3>& V1, thread spvUnsafeArray<float, 3>& V2, thread spvUnsafeArray<float, 3>& d, thread spvUnsafeArray<float, 3>& e)
{
    spvUnsafeArray<spvUnsafeArray<float, 3>, 3> V;
    V[0][0] = V0[0];
    V[0][1] = V0[1];
    V[0][2] = V0[2];
    V[1][0] = V1[0];
    V[1][1] = V1[1];
    V[1][2] = V1[2];
    V[2][0] = V2[0];
    V[2][1] = V2[1];
    V[2][2] = V2[2];
    int i = 1;
    for (; i < 3; i++)
    {
        e[i - 1] = e[i];
    }
    e[2] = 0.0;
    float f = 0.0;
    float tst1 = 0.0;
    float eps = 2.2204460492503130808472633361816e-16;
    int l = 0;
    float p;
    int k;
    for (; l < 3; l++)
    {
        tst1 = fast::max(tst1, abs(d[l]) + abs(e[l]));
        int m = l;
        while (m < 3)
        {
            if (abs(e[m]) <= (eps * tst1))
            {
                break;
            }
            m++;
        }
        if (m > l)
        {
            int iter = 0;
            do
            {
                iter++;
                float g = d[l];
                p = (d[l + 1] - g) / (2.0 * e[l]);
                float param = p;
                float param_1 = 1.0;
                float r = hypot2(param, param_1);
                if (p < 0.0)
                {
                    r = -r;
                }
                d[l] = e[l] / (p + r);
                d[l + 1] = e[l] * (p + r);
                float dl1 = d[l + 1];
                float h = g - d[l];
                i = l + 2;
                for (; i < 3; i++)
                {
                    d[i] -= h;
                }
                f += h;
                p = d[m];
                float c = 1.0;
                float c2 = c;
                float c3 = c;
                float el1 = e[l + 1];
                float s = 0.0;
                float s2 = 0.0;
                i = m - 1;
                for (; i >= l; i--)
                {
                    c3 = c2;
                    c2 = c;
                    s2 = s;
                    g = c * e[i];
                    h = c * p;
                    float param_2 = p;
                    float param_3 = e[i];
                    r = hypot2(param_2, param_3);
                    e[i + 1] = s * r;
                    s = e[i] / r;
                    c = p / r;
                    p = (c * d[i]) - (s * g);
                    d[i + 1] = h + (s * ((c * g) + (s * d[i])));
                    k = 0;
                    for (; k < 3; k++)
                    {
                        h = V[k][i + 1];
                        V[k][i + 1] = (s * V[k][i]) + (c * h);
                        V[k][i] = (c * V[k][i]) - (s * h);
                    }
                }
                p = (((((-s) * s2) * c3) * el1) * e[l]) / dl1;
                e[l] = s * p;
                d[l] = c * p;
            } while (abs(e[l]) > (eps * tst1));
        }
        d[l] += f;
        e[l] = 0.0;
    }
    i = 0;
    for (; i < 2; i++)
    {
        k = i;
        p = d[i];
        int j = i + 1;
        for (; j < 3; j++)
        {
            if (d[j] < p)
            {
                k = j;
                p = d[j];
            }
        }
        if (k != i)
        {
            d[k] = d[i];
            d[i] = p;
            j = 0;
            for (; j < 3; j++)
            {
                p = V[j][i];
                V[j][i] = V[j][k];
                V[j][k] = p;
            }
        }
    }
    V0[0] = V[0][0];
    V0[1] = V[0][1];
    V0[2] = V[0][2];
    V1[0] = V[1][0];
    V1[1] = V[1][1];
    V1[2] = V[1][2];
    V2[0] = V[2][0];
    V2[1] = V[2][1];
    V2[2] = V[2][2];
}

static inline __attribute__((always_inline))
void eigen_decomposition(thread const spvUnsafeArray<spvUnsafeArray<float, 3>, 3>& A, thread spvUnsafeArray<float, 3>& V0, thread spvUnsafeArray<float, 3>& V1, thread spvUnsafeArray<float, 3>& V2, thread spvUnsafeArray<float, 3>& d)
{
    V0[0] = A[0][0];
    V0[1] = A[0][1];
    V0[2] = A[0][2];
    V1[0] = A[1][0];
    V1[1] = A[1][1];
    V1[2] = A[1][2];
    V2[0] = A[2][0];
    V2[1] = A[2][1];
    V2[2] = A[2][2];
    spvUnsafeArray<float, 3> param = V0;
    spvUnsafeArray<float, 3> param_1 = V1;
    spvUnsafeArray<float, 3> param_2 = V2;
    spvUnsafeArray<float, 3> param_3 = d;
    spvUnsafeArray<float, 3> e;
    spvUnsafeArray<float, 3> param_4 = e;
    tred2(param, param_1, param_2, param_3, param_4);
    V0 = param;
    V1 = param_1;
    V2 = param_2;
    d = param_3;
    e = param_4;
    spvUnsafeArray<float, 3> param_5 = V0;
    spvUnsafeArray<float, 3> param_6 = V1;
    spvUnsafeArray<float, 3> param_7 = V2;
    spvUnsafeArray<float, 3> param_8 = d;
    spvUnsafeArray<float, 3> param_9 = e;
    tql2(param_5, param_6, param_7, param_8, param_9);
    V0 = param_5;
    V1 = param_6;
    V2 = param_7;
    d = param_8;
    e = param_9;
}

static inline __attribute__((always_inline))
void Stat3D_getEigenVectorsDesc(thread const Stat3D& stat3D, thread float3& cg, thread float3& v1, thread float3& v2, thread float3& v3, thread float& d1, thread float& d2, thread float& d3)
{
    float xmean = stat3D.xsum / stat3D.count;
    float ymean = stat3D.ysum / stat3D.count;
    float zmean = stat3D.zsum / stat3D.count;
    spvUnsafeArray<spvUnsafeArray<float, 3>, 3> A;
    A[0][0] = (((stat3D.xxsum - (stat3D.xsum * xmean)) - (stat3D.xsum * xmean)) + ((xmean * xmean) * stat3D.count)) / stat3D.count;
    A[0][1] = (((stat3D.xysum - (stat3D.ysum * xmean)) - (stat3D.xsum * ymean)) + ((xmean * ymean) * stat3D.count)) / stat3D.count;
    A[0][2] = (((stat3D.xzsum - (stat3D.zsum * xmean)) - (stat3D.xsum * zmean)) + ((xmean * zmean) * stat3D.count)) / stat3D.count;
    A[1][0] = (((stat3D.xysum - (stat3D.xsum * ymean)) - (stat3D.ysum * xmean)) + ((ymean * xmean) * stat3D.count)) / stat3D.count;
    A[1][1] = (((stat3D.yysum - (stat3D.ysum * ymean)) - (stat3D.ysum * ymean)) + ((ymean * ymean) * stat3D.count)) / stat3D.count;
    A[1][2] = (((stat3D.yzsum - (stat3D.zsum * ymean)) - (stat3D.ysum * zmean)) + ((ymean * zmean) * stat3D.count)) / stat3D.count;
    A[2][0] = (((stat3D.xzsum - (stat3D.xsum * zmean)) - (stat3D.zsum * xmean)) + ((zmean * xmean) * stat3D.count)) / stat3D.count;
    A[2][1] = (((stat3D.yzsum - (stat3D.ysum * zmean)) - (stat3D.zsum * ymean)) + ((zmean * ymean) * stat3D.count)) / stat3D.count;
    A[2][2] = (((stat3D.zzsum - (stat3D.zsum * zmean)) - (stat3D.zsum * zmean)) + ((zmean * zmean) * stat3D.count)) / stat3D.count;
    spvUnsafeArray<spvUnsafeArray<float, 3>, 3> param = A;
    spvUnsafeArray<spvUnsafeArray<float, 3>, 3> V;
    spvUnsafeArray<float, 3> param_1 = V[0];
    spvUnsafeArray<float, 3> param_2 = V[1];
    spvUnsafeArray<float, 3> param_3 = V[2];
    spvUnsafeArray<float, 3> d;
    spvUnsafeArray<float, 3> param_4 = d;
    eigen_decomposition(param, param_1, param_2, param_3, param_4);
    A = param;
    V[0] = param_1;
    V[1] = param_2;
    V[2] = param_3;
    d = param_4;
    v1 = float3(V[0][2], V[1][2], V[2][2]);
    v1 = fast::normalize(v1);
    v2 = float3(V[0][1], V[1][1], V[2][1]);
    v2 = fast::normalize(v2);
    v3 = float3(V[0][0], V[1][0], V[2][0]);
    v3 = fast::normalize(v3);
    cg.x = stat3D.xsum / stat3D.count;
    cg.y = stat3D.ysum / stat3D.count;
    cg.z = stat3D.zsum / stat3D.count;
    d1 = d[2];
    d2 = d[1];
    d3 = d[0];
}

static inline __attribute__((always_inline))
bool Stat3D_computePlaneByPCA(thread Stat3D& stat3D, thread float3& p, thread float3& n)
{
    if (stat3D.count < 3.0)
    {
        return false;
    }
    Stat3D param = stat3D;
    float3 cg;
    float3 param_1 = cg;
    float3 v1;
    float3 param_2 = v1;
    float3 v2;
    float3 param_3 = v2;
    float3 v3;
    float3 param_4 = v3;
    float d1;
    float param_5 = d1;
    float d2;
    float param_6 = d2;
    float d3;
    float param_7 = d3;
    Stat3D_getEigenVectorsDesc(param, param_1, param_2, param_3, param_4, param_5, param_6, param_7);
    stat3D = param;
    cg = param_1;
    v1 = param_2;
    v2 = param_3;
    v3 = param_4;
    d1 = param_5;
    d2 = param_6;
    d3 = param_7;
    p = cg;
    n = v3;
    return true;
}

static inline __attribute__((always_inline))
float orientedPointPlaneDistanceNormalizedNormal(float3 point, float3 planePoint, float3 planeNormalNormalized)
{
    return dot(point, planeNormalNormalized) - dot(planePoint, planeNormalNormalized);
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
    bool _3054 = roiX >= ROI_width(param);
    bool _3076;
    if (!_3054)
    {
        ROI param_1;
        param_1.x.begin = pushConstants.roi.x.begin;
        param_1.x.end = pushConstants.roi.x.end;
        param_1.y.begin = pushConstants.roi.y.begin;
        param_1.y.end = pushConstants.roi.y.end;
        _3076 = roiY >= ROI_height(param_1);
    }
    else
    {
        _3076 = _3054;
    }
    if (_3076)
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
    uint x = (pushConstants.roi.x.begin + roiX) * uint(pushConstants.stepXY);
    uint y = (pushConstants.roi.y.begin + roiY) * uint(pushConstants.stepXY);
    float in_depth = spvDescriptorSet0.in_depthSimMap_d.read(uint2(int2(int(roiX), int(roiY)))).x;
    spvDescriptorSet0.out_normalMap_d.fence();
    float4 out_normal = spvDescriptorSet0.out_normalMap_d.read(uint2(int2(int(roiX), int(roiY))));
    if (in_depth <= 0.0)
    {
        spvDescriptorSet0.out_normalMap_d.write(float4(-1.0), uint2(int2(int(roiX), int(roiY))));
        return;
    }
    float param_2 = in_depth;
    float3 p = get3DPointForPixelAndDepthFromRC(rcDeviceCamParams, float2(float(x), float(y)), param_2);
    float param_3 = in_depth;
    float pixSize = length(p - get3DPointForPixelAndDepthFromRC(rcDeviceCamParams, float2(float(x + 1u), float(y)), param_3));
    Stat3D stat3D = Stat3D{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    int _3321 = -pushConstants.TWsh;
    for (int yp = _3321; yp <= pushConstants.TWsh; yp++)
    {
        int roiYp = int(roiY) + yp;
        if (roiYp < 0)
        {
            continue;
        }
        int _3344 = -pushConstants.TWsh;
        for (int xp = _3344; xp <= pushConstants.TWsh; xp++)
        {
            int roiXp = int(roiX) + xp;
            if (roiXp < 0)
            {
                continue;
            }
            float depthP = spvDescriptorSet0.in_depthSimMap_d.read(uint2(int2(roiXp, roiYp))).x;
            bool _3372 = depthP > 0.0;
            bool _3383;
            if (_3372)
            {
                _3383 = abs(depthP - in_depth) < (30.0 * pixSize);
            }
            else
            {
                _3383 = _3372;
            }
            if (_3383)
            {
                float2 pixP = float2(float(int(x) + xp), float(int(y) + yp));
                float param_4 = depthP;
                float3 pP = get3DPointForPixelAndDepthFromRC(rcDeviceCamParams, pixP, param_4);
                Stat3D param_5 = stat3D;
                Stat3D_update(param_5, pP, 1.0);
                stat3D = param_5;
            }
        }
    }
    float3 pp = p;
    float3 nn = float3(-1.0);
    Stat3D param_6 = stat3D;
    float3 param_7 = pp;
    float3 param_8 = nn;
    bool _3423 = Stat3D_computePlaneByPCA(param_6, param_7, param_8);
    stat3D = param_6;
    pp = param_7;
    nn = param_8;
    if (!_3423)
    {
        spvDescriptorSet0.out_normalMap_d.write(float4(-1.0), uint2(int2(int(roiX), int(roiY))));
        return;
    }
    float3 nc = rcDeviceCamParams.C - p;
    nc = fast::normalize(nc);
    if (orientedPointPlaneDistanceNormalizedNormal(pp + nn, pp, nc) < 0.0)
    {
        nn.x = -nn.x;
        nn.y = -nn.y;
        nn.z = -nn.z;
    }
    spvDescriptorSet0.out_normalMap_d.write(float4(nn, -1.0), uint2(int2(int(roiX), int(roiY))));
}


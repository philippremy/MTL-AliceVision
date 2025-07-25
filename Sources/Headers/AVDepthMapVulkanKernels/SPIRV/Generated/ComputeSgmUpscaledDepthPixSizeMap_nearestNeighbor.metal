#pragma clang diagnostic ignored "-Wmissing-prototypes"

#include <metal_stdlib>
#include <simd/simd.h>

using namespace metal;

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
    uint rcLevelWidth;
    uint rcLevelHeight;
    float rcMipmapLevel;
    int stepXY;
    int halfNbDepths;
    float ratio;
    ROI_1 roi;
};

constant uint3 gl_WorkGroupSize [[maybe_unused]] = uint3(16u, 16u, 1u);

struct DeviceCameraParams
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
    DeviceCameraParams cameraParams[100];
};

struct spvDescriptorSetBuffer0
{
    texture2d<float, access::read_write> out_upscaledDepthPixSizeMap_d [[id(0)]];
    texture2d<float> rcMipmapImage_tex [[id(1)]];
    sampler rcMipmapImage_texSmplr [[id(2)]];
    texture2d<float> in_sgmDepthThicknessMap_d [[id(3)]];
    constant DeviceCameraParamsConstant* constantCameraParametersArray_d [[id(4)]];
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

kernel void main0(constant spvDescriptorSetBuffer0& spvDescriptorSet0 [[buffer(0)]], constant PushConstants& pushConstants [[buffer(1)]], uint3 gl_GlobalInvocationID [[thread_position_in_grid]])
{
    uint roiX = gl_GlobalInvocationID.x;
    uint roiY = gl_GlobalInvocationID.y;
    ROI param;
    param.x.begin = pushConstants.roi.x.begin;
    param.x.end = pushConstants.roi.x.end;
    param.y.begin = pushConstants.roi.y.begin;
    param.y.end = pushConstants.roi.y.end;
    bool _83 = roiX >= ROI_width(param);
    bool _105;
    if (!_83)
    {
        ROI param_1;
        param_1.x.begin = pushConstants.roi.x.begin;
        param_1.x.end = pushConstants.roi.x.end;
        param_1.y.begin = pushConstants.roi.y.begin;
        param_1.y.end = pushConstants.roi.y.end;
        _105 = roiY >= ROI_height(param_1);
    }
    else
    {
        _105 = _83;
    }
    if (_105)
    {
        return;
    }
    uint x = (pushConstants.roi.x.begin + roiX) * uint(pushConstants.stepXY);
    uint y = (pushConstants.roi.y.begin + roiY) * uint(pushConstants.stepXY);
    spvDescriptorSet0.out_upscaledDepthPixSizeMap_d.fence();
    float4 out_depthPixSize = spvDescriptorSet0.out_upscaledDepthPixSizeMap_d.read(uint2(int2(int(roiX), int(roiY))));
    if (spvDescriptorSet0.rcMipmapImage_tex.sample(spvDescriptorSet0.rcMipmapImage_texSmplr, float2(int2(int((float(x) + 0.5) / float(pushConstants.rcLevelWidth)), int((float(y) + 0.5) / float(pushConstants.rcLevelHeight)))), level(pushConstants.rcMipmapLevel)).w < 0.89999997615814208984375)
    {
        spvDescriptorSet0.out_upscaledDepthPixSizeMap_d.write(float4(-2.0, 0.0, out_depthPixSize.z, out_depthPixSize.w), uint2(int2(int(roiX), int(roiY))));
        return;
    }
    float oy = (float(roiY) - 0.5) * pushConstants.ratio;
    float ox = (float(roiX) - 0.5) * pushConstants.ratio;
    int xp = int(floor(ox + 0.5));
    int yp = int(floor(oy + 0.5));
    ROI param_2;
    param_2.x.begin = pushConstants.roi.x.begin;
    param_2.x.end = pushConstants.roi.x.end;
    param_2.y.begin = pushConstants.roi.y.begin;
    param_2.y.end = pushConstants.roi.y.end;
    xp = min(xp, (int(float(ROI_width(param_2)) * pushConstants.ratio) - 1));
    ROI param_3;
    param_3.x.begin = pushConstants.roi.x.begin;
    param_3.x.end = pushConstants.roi.x.end;
    param_3.y.begin = pushConstants.roi.y.begin;
    param_3.y.end = pushConstants.roi.y.end;
    yp = min(yp, (int(float(ROI_height(param_3)) * pushConstants.ratio) - 1));
    float4 out_depthThickness = spvDescriptorSet0.in_sgmDepthThicknessMap_d.read(uint2(int2(xp, yp)));
    float out_pixSize = out_depthThickness.y / float(pushConstants.halfNbDepths);
    out_depthPixSize.x = out_depthThickness.x;
    out_depthPixSize.y = out_pixSize;
    spvDescriptorSet0.out_upscaledDepthPixSizeMap_d.write(out_depthPixSize, uint2(int2(int(roiX), int(roiY))));
}


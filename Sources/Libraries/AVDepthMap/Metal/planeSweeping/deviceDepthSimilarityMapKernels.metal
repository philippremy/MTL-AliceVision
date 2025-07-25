// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <AVDepthMap/Metal/device/buffer.hpp>
#include <AVDepthMap/Metal/device/matrix.hpp>
#include <AVDepthMap/Metal/device/Patch.hpp>
#include <AVDepthMap/Metal/device/eig33.hpp>
#include <AVDepthMap/Metal/device/DeviceCameraParams.hpp>
#include <AVDepthMap/Metal/util/KernelArgs.hpp>
#include <AVDepthMap/Metal/util/MTLMipmappedTexture.hpp>

// compute per pixel pixSize instead of using Sgm depth thickness
//#define ALICEVISION_DEPTHMAP_COMPUTE_PIXSIZEMAP

namespace aliceVision {
namespace depthMap {

/**
 * @return (smoothStep, energy)
 */
float2 getCellSmoothStepEnergy(constant const DeviceCameraParams& rcDeviceCamParams,
                               thread const MTLMipmappedTexture<float>& in_depth_tex, // As per CUDA implementation, this uses Nearest Neighbor and no normalized coordinates
                               thread const float2& cell0,
                               thread const float2& offsetRoi)
{
    float2 out = make_float2(0.0f, 180.0f);

    // get pixel depth from the depth texture
    // note: we do not use 0.5f offset because in_depth_tex use nearest neighbor interpolation
    const float d0 = in_depth_tex.tex2DLod(cell0.x, cell0.y);

    // early exit: depth is <= 0
    if(d0 <= 0.0f)
        return out;

    // consider the neighbor pixels
    const float2 cellL = cell0 + make_float2( 0.f, -1.f); // Left
    const float2 cellR = cell0 + make_float2( 0.f,  1.f); // Right
    const float2 cellU = cell0 + make_float2(-1.f,  0.f); // Up
    const float2 cellB = cell0 + make_float2( 1.f,  0.f); // Bottom

    // get associated depths from depth texture
    // note: we do not use 0.5f offset because in_depth_tex use nearest neighbor interpolation
    const float dL = in_depth_tex.tex2DLod(cellL.x, cellL.y);
    const float dR = in_depth_tex.tex2DLod(cellR.x, cellR.y);
    const float dU = in_depth_tex.tex2DLod(cellU.x, cellU.y);
    const float dB = in_depth_tex.tex2DLod(cellB.x, cellB.y);

    // get associated 3D points
    const float3 p0 = get3DPointForPixelAndDepthFromRC(rcDeviceCamParams, cell0 + offsetRoi, d0);
    const float3 pL = get3DPointForPixelAndDepthFromRC(rcDeviceCamParams, cellL + offsetRoi, dL);
    const float3 pR = get3DPointForPixelAndDepthFromRC(rcDeviceCamParams, cellR + offsetRoi, dR);
    const float3 pU = get3DPointForPixelAndDepthFromRC(rcDeviceCamParams, cellU + offsetRoi, dU);
    const float3 pB = get3DPointForPixelAndDepthFromRC(rcDeviceCamParams, cellB + offsetRoi, dB);

    // compute the average point based on neighbors (cg)
    float3 cg = make_float3(0.0f, 0.0f, 0.0f);
    float n = 0.0f;

    if(dL > 0.0f) { cg = cg + pL; n++; }
    if(dR > 0.0f) { cg = cg + pR; n++; }
    if(dU > 0.0f) { cg = cg + pU; n++; }
    if(dB > 0.0f) { cg = cg + pB; n++; }

    // if we have at least one valid depth
    if(n > 1.0f)
    {
        cg = cg / n; // average of x, y, depth
        float3 vcn = rcDeviceCamParams.C - p0;
        normalize(vcn);
        // pS: projection of cg on the line from p0 to camera
        const float3 pS = closestPointToLine3D(cg, p0, vcn);
        // keep the depth difference between pS and p0 as the smoothing step
        out.x = size(rcDeviceCamParams.C - pS) - d0;
    }

    float e = 0.0f;
    n = 0.0f;

    if(dL > 0.0f && dR > 0.0f)
    {
        // large angle between neighbors == flat area => low energy
        // small angle between neighbors == non-flat area => high energy
        e = fmax(e, (180.0f - angleBetwABandAC(p0, pL, pR)));
        n++;
    }
    if(dU > 0.0f && dB > 0.0f)
    {
        e = fmax(e, (180.0f - angleBetwABandAC(p0, pU, pB)));
        n++;
    }
    // the higher the energy, the less flat the area
    if(n > 0.0f)
        out.y = e;

    return out;
}
    
static inline float orientedPointPlaneDistanceNormalizedNormal(thread const float3& point,
                                                               thread const float3& planePoint,
                                                               thread const float3& planeNormalNormalized)
{
    return (dot(point, planeNormalNormalized) - dot(planePoint, planeNormalNormalized));
}

kernel void depthSimMapCopyDepthOnly_kernel(device float2* out_deptSimMap_d [[buffer(0)]],
                                            device const float2* in_depthSimMap_d [[buffer(1)]],
                                            constant const depthSimMapCopyDepthOnly_kernel_PC& kArgs [[buffer(30)]],
                                            const uint3 gid [[thread_position_in_grid]])
{
    const unsigned int x = gid.x;
    const unsigned int y = gid.y;

    if(x >= kArgs.width || y >= kArgs.height)
        return;

    // write output
    device float2* out_depthSim = get2DBufferAt(out_deptSimMap_d, kArgs.out_deptSimMap_p, x, y);
    out_depthSim->x = get2DBufferAt(in_depthSimMap_d, kArgs.in_depthSimMap_p, x, y)->x;
    out_depthSim->y = kArgs.defaultSim;
}

kernel void mapUpscale_kernel(device float3* out_upscaledMap_d [[buffer(0)]],
                              device float3* in_map_d [[buffer(1)]],
                              constant const mapUpscale_kernel_PC& kArgs [[buffer(30)]],
                              const uint3 gid [[thread_position_in_grid]])
{
    const unsigned int x = gid.x;
    const unsigned int y = gid.y;

    if(x >= kArgs.roi.width() || y >= kArgs.roi.height())
        return;

    const float ox = (float(x) - 0.5f) * kArgs.ratio;
    const float oy = (float(y) - 0.5f) * kArgs.ratio;

    // nearest neighbor, no interpolation
    const int xp = min(int(floor(ox + 0.5)), int(kArgs.roi.width()  * kArgs.ratio) - 1);
    const int yp = min(int(floor(oy + 0.5)), int(kArgs.roi.height() * kArgs.ratio) - 1);

    // write output upscaled map
    *get2DBufferAt(out_upscaledMap_d, kArgs.out_upscaledMap_p, x, y) = *get2DBufferAt(in_map_d, kArgs.in_map_p, xp, yp);
}

kernel void depthThicknessMapSmoothThickness_kernel(device float2* inout_depthThicknessMap_d [[buffer(0)]], 
                                                    constant const depthThicknessMapSmoothThickness_kernel_PC& kArgs [[buffer(30)]],
                                                    const uint3 gid [[thread_position_in_grid]])
{
    const unsigned int roiX = gid.x;
    const unsigned int roiY = gid.y;

    if(roiX >= kArgs.roi.width() || roiY >= kArgs.roi.height())
        return;

    // corresponding output depth/thickness (depth unchanged)
    device float2* inout_depthThickness = get2DBufferAt(inout_depthThicknessMap_d, kArgs.inout_depthThicknessMap_p, roiX, roiY);

    // depth invalid or masked
    if(inout_depthThickness->x <= 0.0f)
        return;

    const float minThickness = kArgs.minThicknessInflate * inout_depthThickness->y;
    const float maxThickness = kArgs.maxThicknessInflate * inout_depthThickness->y;

    // compute average depth distance to the center pixel
    float sumCenterDepthDist = 0.f;
    int nbValidPatchPixels = 0;

    // patch 3x3
    for(int yp = -1; yp <= 1; ++yp)
    {
        for(int xp = -1; xp <= 1; ++xp)
        {
            // compute patch coordinates
            const int roiXp = int(roiX) + xp;
            const int roiYp = int(roiY) + yp;

            if((xp == 0 && yp == 0) ||                           // avoid pixel center
               roiXp < 0 || roiXp >= (int)kArgs.roi.width() ||   // avoid pixel outside the ROI
               roiYp < 0 || roiYp >= (int)kArgs.roi.height())    // avoid pixel outside the ROI
            {
                continue;
            }

            // corresponding path depth/thickness
            const float2 in_depthThicknessPatch = *get2DBufferAt(inout_depthThicknessMap_d, kArgs.inout_depthThicknessMap_p, roiXp, roiYp);

            // patch depth valid
            if(in_depthThicknessPatch.x > 0.0f)
            {
                const float depthDistance = abs(inout_depthThickness->x - in_depthThicknessPatch.x);
                sumCenterDepthDist += max(minThickness, min(maxThickness, depthDistance)); // clamp (minThickness, maxThickness)
                ++nbValidPatchPixels;
            }
        }
    }

    // we require at least 3 valid patch pixels (over 8)
    if(nbValidPatchPixels < 3)
        return;

    // write output smooth thickness
    inout_depthThickness->y = sumCenterDepthDist / nbValidPatchPixels;
}
    
kernel void computeSgmUpscaledDepthPixSizeMap_nearestNeighbor_kernel(device float2* out_upscaledDepthPixSizeMap_d [[buffer(0)]],
                                                                     device const float2* in_sgmDepthThicknessMap_d [[buffer(1)]],
                                                                     device MTLRGBA* rcMipmapImage_tex [[buffer(2)]], // As per CUDA implementation, this uses Bilinear Filtering and normalized coordinates
                                                                     constant const DeviceCameraParams* constantCameraParametersArray_d [[buffer(3)]],
                                                                     constant const computeSgmUpscaledDepthPixSizeMap_nearestNeighbor_kernel_PC& kArgs [[buffer(30)]],
                                                                     const uint3 gid [[thread_position_in_grid]])
{
    const unsigned int roiX = gid.x;
    const unsigned int roiY = gid.y;

    if(roiX >= kArgs.roi.width() || roiY >= kArgs.roi.height())
        return;

    // corresponding image coordinates
    const unsigned int x = (kArgs.roi.x.begin + roiX) * (unsigned int)(kArgs.stepXY);
    const unsigned int y = (kArgs.roi.y.begin + roiY) * (unsigned int)(kArgs.stepXY);

    // corresponding output upscaled depth/pixSize map
    device float2* out_depthPixSize = get2DBufferAt(out_upscaledDepthPixSizeMap_d, kArgs.out_upscaledDepthPixSizeMap_p, roiX, roiY);

    // filter masked pixels (alpha < 0.9f)
    // NOTE: Metal workaround: First create a MipmapTexture wrapper
    MTLMipmappedTexture<MTLRGBA> rcMipmapImage_tex_wrapper = MTLMipmappedTexture<MTLRGBA>(rcMipmapImage_tex, kArgs.rcLevel0Width, kArgs.rcLevel0Height, kArgs.rcLevelCount);
    if(rcMipmapImage_tex_wrapper.tex2DLodNorm<FilterMode::Bilinear>((float(x) + 0.5f) / float(kArgs.rcLevelWidth), (float(y) + 0.5f) / float(kArgs.rcLevelHeight), kArgs.rcMipmapLevel).w < 0.9f)
    {
        *out_depthPixSize = make_float2(-2.f, 0.f);
        return;
    }

    // find corresponding depth/thickness
    // nearest neighbor, no interpolation
    const float oy = (float(roiY) - 0.5f) * kArgs.ratio;
    const float ox = (float(roiX) - 0.5f) * kArgs.ratio;

    int xp = floor(ox + 0.5);
    int yp = floor(oy + 0.5);

    xp = min(xp, int(kArgs.roi.width()  * kArgs.ratio) - 1);
    yp = min(yp, int(kArgs.roi.height() * kArgs.ratio) - 1);

    const float2 out_depthThickness = *get2DBufferAt(in_sgmDepthThicknessMap_d, kArgs.in_sgmDepthThicknessMap_p, xp, yp);

#ifdef ALICEVISION_DEPTHMAP_COMPUTE_PIXSIZEMAP
    // R camera parameters
    constant const DeviceCameraParams& rcDeviceCamParams = constantCameraParametersArray_d[kArgs.rcDeviceCameraParamsId];

    // get rc 3d point
    const float3 p = get3DPointForPixelAndDepthFromRC(rcDeviceCamParams, make_float2(float(x), float(y)), out_depthThickness.x);

    // compute and write rc 3d point pixSize
    const float out_pixSize = computePixSize(rcDeviceCamParams, p);
#else
    // compute pixSize from depth thickness
    const float out_pixSize = out_depthThickness.y / kArgs.halfNbDepths;
#endif

    // write output depth/pixSize
    out_depthPixSize->x = out_depthThickness.x;
    out_depthPixSize->y = out_pixSize;
}

kernel void computeSgmUpscaledDepthPixSizeMap_bilinear_kernel(device float2* out_upscaledDepthPixSizeMap_d [[buffer(0)]],
                                                              device const float2* in_sgmDepthThicknessMap_d [[buffer(1)]],
                                                              device MTLRGBA* rcMipmapImage_tex [[buffer(2)]], // As per CUDA implementation, this uses Bilinear Filtering and normalized coordinates
                                                              constant const DeviceCameraParams* constantCameraParametersArray_d [[buffer(3)]],
                                                              constant const computeSgmUpscaledDepthPixSizeMap_bilinear_kernel_PC& kArgs [[buffer(30)]],
                                                              const uint3 gid [[thread_position_in_grid]])
{
    const unsigned int roiX = gid.x;
    const unsigned int roiY = gid.y;

    if(roiX >= kArgs.roi.width() || roiY >= kArgs.roi.height())
        return;

    // corresponding image coordinates
    const unsigned int x = (kArgs.roi.x.begin + roiX) * (unsigned int)(kArgs.stepXY);
    const unsigned int y = (kArgs.roi.y.begin + roiY) * (unsigned int)(kArgs.stepXY);

    // corresponding output upscaled depth/pixSize map
    device float2* out_depthPixSize = get2DBufferAt(out_upscaledDepthPixSizeMap_d, kArgs.out_upscaledDepthPixSizeMap_p, roiX, roiY);

    // filter masked pixels with alpha
    // NOTE: Metal workaround: First create a MipmapTexture wrapper
    MTLMipmappedTexture<MTLRGBA> rcMipmapImage_tex_wrapper = MTLMipmappedTexture<MTLRGBA>(rcMipmapImage_tex, kArgs.rcLevel0Width, kArgs.rcLevel0Height, kArgs.rcLevelCount);
    if(rcMipmapImage_tex_wrapper.tex2DLodNorm<FilterMode::Bilinear>((float(x) + 0.5f) / float(kArgs.rcLevelWidth), (float(y) + 0.5f) / float(kArgs.rcLevelHeight), kArgs.rcMipmapLevel).w < ALICEVISION_DEPTHMAP_RC_MIN_ALPHA)
    {
        *out_depthPixSize = make_float2(-2.f, 0.f);
        return;
    }

    // find adjacent pixels
    const float oy = (float(roiY) - 0.5f) * kArgs.ratio;
    const float ox = (float(roiX) - 0.5f) * kArgs.ratio;

    int xp = floor(ox);
    int yp = floor(oy);

    xp = min(xp, int(kArgs.roi.width()  * kArgs.ratio) - 2);
    yp = min(yp, int(kArgs.roi.height() * kArgs.ratio) - 2);

    const float2 lu = *get2DBufferAt(in_sgmDepthThicknessMap_d, kArgs.in_sgmDepthThicknessMap_p, xp, yp);
    const float2 ru = *get2DBufferAt(in_sgmDepthThicknessMap_d, kArgs.in_sgmDepthThicknessMap_p, xp + 1, yp);
    const float2 rd = *get2DBufferAt(in_sgmDepthThicknessMap_d, kArgs.in_sgmDepthThicknessMap_p, xp + 1, yp + 1);
    const float2 ld = *get2DBufferAt(in_sgmDepthThicknessMap_d, kArgs.in_sgmDepthThicknessMap_p, xp, yp + 1);

    // find corresponding depth/thickness
    float2 out_depthThickness;

    if(lu.x <= 0.0f || ru.x <= 0.0f || rd.x <= 0.0f || ld.x <= 0.0f)
    {
        // at least one corner depth is invalid
        // average the other corners to get a proper depth/thickness
        float2 sumDepthThickness = {0.0f, 0.0f};
        int count = 0;

        if(lu.x > 0.0f)
        {
            sumDepthThickness = sumDepthThickness + lu;
            ++count;
        }
        if(ru.x > 0.0f)
        {
            sumDepthThickness = sumDepthThickness + ru;
            ++count;
        }
        if(rd.x > 0.0f)
        {
            sumDepthThickness = sumDepthThickness + rd;
            ++count;
        }
        if(ld.x > 0.0f)
        {
            sumDepthThickness = sumDepthThickness + ld;
            ++count;
        }
        if(count != 0)
        {
            out_depthThickness = {sumDepthThickness.x / float(count), sumDepthThickness.y / float(count)};
        }
        else
        {
            // invalid depth
            *out_depthPixSize = {-1.0f, 1.0f};
            return;
        }
    }
    else
    {
        // bilinear interpolation
        const float ui = ox - float(xp);
        const float vi = oy - float(yp);
        const float2 u = lu + (ru - lu) * ui;
        const float2 d = ld + (rd - ld) * ui;
        out_depthThickness = u + (d - u) * vi;
    }

#ifdef ALICEVISION_DEPTHMAP_COMPUTE_PIXSIZEMAP
    // R camera parameters
    constant const DeviceCameraParams& rcDeviceCamParams = constantCameraParametersArray_d[kArgs.rcDeviceCameraParamsId];

    // get rc 3d point
    const float3 p = get3DPointForPixelAndDepthFromRC(rcDeviceCamParams, make_float2(float(x), float(y)), out_depthThickness.x);

    // compute and write rc 3d point pixSize
    const float out_pixSize = computePixSize(rcDeviceCamParams, p);
#else
    // compute pixSize from depth thickness
    const float out_pixSize = out_depthThickness.y / kArgs.halfNbDepths;
#endif

    // write output depth/pixSize
    out_depthPixSize->x = out_depthThickness.x;
    out_depthPixSize->y = out_pixSize;
}

kernel void depthSimMapComputeNormal_kernel(device float3* out_normalMap_d [[buffer(0)]],
                                            device const float2* in_depthSimMap_d [[buffer(1)]],
                                            constant const DeviceCameraParams* constantCameraParametersArray_d [[buffer(2)]],
                                            constant const depthSimMapComputeNormal_kernel_PC& kArgs [[buffer(30)]],
                                            const uint3 gid [[thread_position_in_grid]])
{
    const unsigned int roiX = gid.x;
    const unsigned int roiY = gid.y;

    if(roiX >= kArgs.roi.width() || roiY >= kArgs.roi.height())
        return;

    // R camera parameters
    constant const DeviceCameraParams& rcDeviceCamParams = constantCameraParametersArray_d[kArgs.rcDeviceCameraParamsId];

    // corresponding image coordinates
    const unsigned int x = (kArgs.roi.x.begin + roiX) * (unsigned int)(kArgs.stepXY);
    const unsigned int y = (kArgs.roi.y.begin + roiY) * (unsigned int)(kArgs.stepXY);

    // corresponding input depth
    const float in_depth = get2DBufferAt(in_depthSimMap_d, kArgs.in_depthSimMap_p, roiX, roiY)->x; // use only depth

    // corresponding output normal
    device float3* out_normal = get2DBufferAt(out_normalMap_d, kArgs.out_normalMap_p, roiX, roiY);

    // no depth
    if(in_depth <= 0.0f)
    {
        *out_normal = make_float3(-1.f, -1.f, -1.f);
        return;
    }

    const float3 p = get3DPointForPixelAndDepthFromRC(rcDeviceCamParams, make_float2(float(x), float(y)), in_depth);
    const float pixSize = size(p - get3DPointForPixelAndDepthFromRC(rcDeviceCamParams, make_float2(float(x + 1), float(y)), in_depth));

    mtl_stat3d s3d = mtl_stat3d();

    for(int yp = -kArgs.TWsh; yp <= kArgs.TWsh; ++yp)
    {
        const int roiYp = int(roiY) + yp;
        if(roiYp < 0)
            continue;

        for(int xp = -kArgs.TWsh; xp <= kArgs.TWsh; ++xp)
        {
            const int roiXp = int(roiX) + xp;
            if(roiXp < 0)
                continue;

            const float depthP = get2DBufferAt(in_depthSimMap_d, kArgs.in_depthSimMap_p, roiXp, roiYp)->x;  // use only depth

            if((depthP > 0.0f) && (fabs(depthP - in_depth) < 30.0f * pixSize))
            {
                const float w = 1.0f;
                const float2 pixP = make_float2(float(int(x) + xp), float(int(y) + yp));
                const float3 pP = get3DPointForPixelAndDepthFromRC(rcDeviceCamParams, pixP, depthP);
                s3d.update(pP, w);
            }
        }
    }

    float3 pp = p;
    float3 nn = make_float3(-1.f, -1.f, -1.f);

    if(!s3d.computePlaneByPCA(pp, nn))
    {
        *out_normal = make_float3(-1.f, -1.f, -1.f);
        return;
    }

    float3 nc = rcDeviceCamParams.C - p;
    normalize(nc);

    if(orientedPointPlaneDistanceNormalizedNormal(pp + nn, pp, nc) < 0.0f)
    {
        nn.x = -nn.x;
        nn.y = -nn.y;
        nn.z = -nn.z;
    }

    *out_normal = nn;
}

kernel void optimize_varLofLABtoW_kernel(device float* out_varianceMap_d [[buffer(0)]],
                                         device MTLRGBA* rcMipmapImage_tex [[buffer(1)]], // As per CUDA implementation, this uses Bilinear Filtering and normalized coordinates
                                         constant const optimize_varLofLABtoW_kernel_PC& kArgs [[buffer(30)]],
                                         const uint3 gid [[thread_position_in_grid]])
{
    // roi and varianceMap coordinates
    const unsigned int roiX = gid.x;
    const unsigned int roiY = gid.y;

    if(roiX >= kArgs.roi.width() || roiY >= kArgs.roi.height())
        return;

    // corresponding image coordinates
    const float x = float(kArgs.roi.x.begin + roiX) * float(kArgs.stepXY);
    const float y = float(kArgs.roi.y.begin + roiY) * float(kArgs.stepXY);

    // compute inverse width / height
    // note: useful to compute p1 / m1 normalized coordinates
    const float invLevelWidth  = 1.f / float(kArgs.rcLevelWidth);
    const float invLevelHeight = 1.f / float(kArgs.rcLevelHeight);

    // compute gradient size of L
    // note: we use 0.5f offset because rcTex texture use interpolation
    // NOTE: Create Metal Texture Wrapper
    MTLMipmappedTexture<MTLRGBA> rcMipmapImage_tex_wrapper = MTLMipmappedTexture<MTLRGBA>(rcMipmapImage_tex, kArgs.rcLevel0Width, kArgs.rcLevel0Height, kArgs.rcLevelCount);
    const half xM1 = rcMipmapImage_tex_wrapper.tex2DLodNorm<FilterMode::Bilinear>(((x - 1.f) + 0.5f) * invLevelWidth, ((y + 0.f) + 0.5f) * invLevelHeight, kArgs.rcMipmapLevel).x;
    const float xP1 = rcMipmapImage_tex_wrapper.tex2DLodNorm<FilterMode::Bilinear>(((x + 1.f) + 0.5f) * invLevelWidth, ((y + 0.f) + 0.5f) * invLevelHeight, kArgs.rcMipmapLevel).x;
    const float yM1 = rcMipmapImage_tex_wrapper.tex2DLodNorm<FilterMode::Bilinear>(((x + 0.f) + 0.5f) * invLevelWidth, ((y - 1.f) + 0.5f) * invLevelHeight, kArgs.rcMipmapLevel).x;
    const float yP1 = rcMipmapImage_tex_wrapper.tex2DLodNorm<FilterMode::Bilinear>(((x + 0.f) + 0.5f) * invLevelWidth, ((y + 1.f) + 0.5f) * invLevelHeight, kArgs.rcMipmapLevel).x;

    const float2 g = make_float2(xM1 - xP1, yM1 - yP1); // TODO: not divided by 2?
    const float grad = size(g);

    // write output
    *get2DBufferAt(out_varianceMap_d, kArgs.out_varianceMap_p, roiX, roiY) = grad;
}

kernel void optimize_getOptDeptMapFromOptDepthSimMap_kernel(device float* out_tmpOptDepthMap_d [[buffer(0)]],
                                                            device const float2* in_optDepthSimMap_d [[buffer(1)]],
                                                            constant const optimize_getOptDeptMapFromOptDepthSimMap_kernel_PC& kArgs [[buffer(30)]],
                                                            const uint3 gid [[thread_position_in_grid]])
{
    // roi and depth/sim map part coordinates
    const unsigned int roiX = gid.x;
    const unsigned int roiY = gid.y;

    if(roiX >= kArgs.roi.width() || roiY >= kArgs.roi.height())
        return;

    *get2DBufferAt(out_tmpOptDepthMap_d, kArgs.out_tmpOptDepthMap_p, roiX, roiY) = get2DBufferAt(in_optDepthSimMap_d, kArgs.in_optDepthSimMap_p, roiX, roiY)->x; // depth
}

kernel void optimize_depthSimMap_kernel(device float2* out_optimizeDepthSimMap_d [[buffer(0)]],        // output optimized depth/sim map
                                        device const float2* in_sgmDepthPixSizeMap_d [[buffer(1)]],    // input upscaled rough depth/pixSize map
                                        device const float2* in_refineDepthSimMap_d [[buffer(2)]],     // input fine depth/sim map
                                        device float* imgVariance_tex [[buffer(3)]], // As per CUDA implementation, this uses Nearest Neighbor and no normalized coordinates
                                        device float* depth_tex [[buffer(4)]], // As per CUDA implementation, this uses Nearest Neighbor and no normalized coordinates
                                        constant const DeviceCameraParams* constantCameraParametersArray_d [[buffer(5)]],
                                        constant const optimize_depthSimMap_kernel_PC& kArgs [[buffer(30)]],
                                        const uint3 gid [[thread_position_in_grid]])
{
    // roi and imgVariance_tex, depth_tex coordinates
    const unsigned int roiX = gid.x;
    const unsigned int roiY = gid.y;

    if(roiX >= kArgs.roi.width() || roiY >= kArgs.roi.height())
        return;

    // R camera parameters
    constant const DeviceCameraParams& rcDeviceCamParams = constantCameraParametersArray_d[kArgs.rcDeviceCameraParamsId];

    // SGM upscale (rough) depth/pixSize
    const float2 sgmDepthPixSize = *get2DBufferAt(in_sgmDepthPixSizeMap_d, kArgs.in_sgmDepthPixSizeMap_p, roiX, roiY);
    const float sgmDepth = sgmDepthPixSize.x;
    const float sgmPixSize = sgmDepthPixSize.y;

    // refined and fused (fine) depth/sim
    const float2 refineDepthSim = *get2DBufferAt(in_refineDepthSimMap_d, kArgs.in_refineDepthSimMap_p, roiX, roiY);
    const float refineDepth = refineDepthSim.x;
    const float refineSim = refineDepthSim.y;

    // output optimized depth/sim
    device float2* out_optDepthSimPtr = get2DBufferAt(out_optimizeDepthSimMap_d, kArgs.out_optimizeDepthSimMap_p, roiX, roiY);
    float2 out_optDepthSim = (kArgs.iter == 0) ? make_float2(sgmDepth, refineSim) : *out_optDepthSimPtr;
    const float depthOpt = out_optDepthSim.x;

    if (depthOpt > 0.0f)
    {
        // NOTE: Create Metal Texture Wrapper
        MTLMipmappedTexture<float> depth_tex_wrapper = MTLMipmappedTexture<float>(depth_tex, kArgs.depth_tex_texLevel0Width, kArgs.depth_tex_texLevel0Height, kArgs.depth_tex_texLevelCount);
        const float2 depthSmoothStepEnergy = getCellSmoothStepEnergy(rcDeviceCamParams, depth_tex_wrapper, {float(roiX), float(roiY)}, {float(kArgs.roi.x.begin), float(kArgs.roi.y.begin)}); // (smoothStep, energy)
        float stepToSmoothDepth = depthSmoothStepEnergy.x;
        stepToSmoothDepth = copysign(fmin(fabs(stepToSmoothDepth), sgmPixSize / 10.0f), stepToSmoothDepth);
        const float depthEnergy = depthSmoothStepEnergy.y; // max angle with neighbors
        float stepToFineDM = refineDepth - depthOpt; // distance to refined/noisy input depth map
        stepToFineDM = copysign(fmin(fabs(stepToFineDM), sgmPixSize / 10.0f), stepToFineDM);

        const float stepToRoughDM = sgmDepth - depthOpt; // distance to smooth/robust input depth map
        
        // NOTE: Create Metal Texture Wrapper
        MTLMipmappedTexture<float> imgVariance_tex_wrapper = MTLMipmappedTexture<float>(imgVariance_tex, kArgs.imgVariance_texLevel0Width, kArgs.imgVariance_texLevel0Height, kArgs.imgVariance_texLevelCount);
        const float imgColorVariance = imgVariance_tex_wrapper.sampleTex2D<FilterMode::NearestNeighbor>(float(roiX), float(roiY)); // do not use 0.5f offset because imgVariance_tex use nearest neighbor interpolation
        const float colorVarianceThresholdForSmoothing = 20.0f;
        const float angleThresholdForSmoothing = 30.0f; // 30

        // https://www.desmos.com/calculator/kob9lxs9qf
        const float weightedColorVariance = sigmoid2(5.0f, angleThresholdForSmoothing, 40.0f, colorVarianceThresholdForSmoothing, imgColorVariance);

        // https://www.desmos.com/calculator/jwhpjq6ppj
        const float fineSimWeight = sigmoid(0.0f, 1.0f, 0.7f, -0.7f, refineSim);

        // if geometry variation is bigger than color variation => the fineDM is considered noisy

        // if depthEnergy > weightedColorVariance   => energyLowerThanVarianceWeight=0 => smooth
        // else:                                    => energyLowerThanVarianceWeight=1 => use fineDM
        // weightedColorVariance max value is 30, so if depthEnergy > 30 (which means depthAngle < 150�) energyLowerThanVarianceWeight will be 0
        // https://www.desmos.com/calculator/jzbweilb85
        const float energyLowerThanVarianceWeight = sigmoid(0.0f, 1.0f, 30.0f, weightedColorVariance, depthEnergy); // TODO: 30 => 60

        // https://www.desmos.com/calculator/ilsk7pthvz
        const float closeToRoughWeight = 1.0f - sigmoid(0.0f, 1.0f, 10.0f, 17.0f, fabs(stepToRoughDM / sgmPixSize)); // TODO: 10 => 30

        // f(z) = c1 * s1(z_rought - z)^2 + c2 * s2(z-z_fused)^2 + coeff3 * s3*(z-z_smooth)^2

        const float depthOptStep = closeToRoughWeight * stepToRoughDM + // distance to smooth/robust input depth map
                                   (1.0f - closeToRoughWeight) * (energyLowerThanVarianceWeight * fineSimWeight * stepToFineDM + // distance to refined/noisy
                                                                 (1.0f - energyLowerThanVarianceWeight) * stepToSmoothDepth); // max angle in current depthMap

        out_optDepthSim.x = depthOpt + depthOptStep;

        out_optDepthSim.y = (1.0f - closeToRoughWeight) * (energyLowerThanVarianceWeight * fineSimWeight * refineSim + (1.0f - energyLowerThanVarianceWeight) * (depthEnergy / 20.0f));
    }

    *out_optDepthSimPtr = out_optDepthSim;
}

} // namespace depthMap
} // namespace aliceVision

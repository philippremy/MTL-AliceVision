// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <AVDepthMap/Metal/device/buffer.hpp>
#include <AVDepthMap/Metal/device/color.hpp>
#include <AVDepthMap/Metal/device/matrix.hpp>
#include <AVDepthMap/Metal/device/SimStat.hpp>
#include <AVDepthMap/Metal/device/DeviceCameraParams.hpp>
#include <AVDepthMap/Metal/device/DevicePatchPattern.hpp>
#include <AVDepthMap/Metal/util/MTLMipmappedTexture.hpp>

namespace aliceVision {
namespace depthMap {

struct Patch
{
    float3 p; //< 3d point
    float3 n; //< normal
    float3 x; //< x axis
    float3 y; //< y axis
    float d;  //< pixel size
};

inline void rotPointAroundVect(device float3& out, device float3& X, device float3& vect, int angle)
{
    float ux, uy, uz, vx, vy, vz, wx, wy, wz, sa, ca, x, y, z, u, v, w;

    float sizeX = sqrt(dot(X, X));
    x = X.x / sizeX;
    y = X.y / sizeX;
    z = X.z / sizeX;
    u = vect.x;
    v = vect.y;
    w = vect.z;

    /*Rotate the point (x,y,z) around the vector (u,v,w)*/
    ux = u * x;
    uy = u * y;
    uz = u * z;
    vx = v * x;
    vy = v * y;
    vz = v * z;
    wx = w * x;
    wy = w * y;
    wz = w * z;
    sa = sin((float)angle * (M_PI_F / 180.0f));
    ca = cos((float)angle * (M_PI_F / 180.0f));
    x = u * (ux + vy + wz) + (x * (v * v + w * w) - u * (vy + wz)) * ca + (-wy + vz) * sa;
    y = v * (ux + vy + wz) + (y * (u * u + w * w) - v * (ux + wz)) * ca + (wx - uz) * sa;
    z = w * (ux + vy + wz) + (z * (u * u + v * v) - w * (ux + vy)) * ca + (-vx + uy) * sa;

    u = sqrt(x * x + y * y + z * z);
    x /= u;
    y /= u;
    z /= u;

    out.x = x * sizeX;
    out.y = y * sizeX;
    out.z = z * sizeX;
}

inline void rotPointAroundVect(thread float3& out, thread float3& X, thread float3& vect, int angle)
{
    float ux, uy, uz, vx, vy, vz, wx, wy, wz, sa, ca, x, y, z, u, v, w;

    float sizeX = sqrt(dot(X, X));
    x = X.x / sizeX;
    y = X.y / sizeX;
    z = X.z / sizeX;
    u = vect.x;
    v = vect.y;
    w = vect.z;

    /*Rotate the point (x,y,z) around the vector (u,v,w)*/
    ux = u * x;
    uy = u * y;
    uz = u * z;
    vx = v * x;
    vy = v * y;
    vz = v * z;
    wx = w * x;
    wy = w * y;
    wz = w * z;
    sa = sin((float)angle * (M_PI_F / 180.0f));
    ca = cos((float)angle * (M_PI_F / 180.0f));
    x = u * (ux + vy + wz) + (x * (v * v + w * w) - u * (vy + wz)) * ca + (-wy + vz) * sa;
    y = v * (ux + vy + wz) + (y * (u * u + w * w) - v * (ux + wz)) * ca + (wx - uz) * sa;
    z = w * (ux + vy + wz) + (z * (u * u + v * v) - w * (ux + vy)) * ca + (-vx + uy) * sa;

    u = sqrt(x * x + y * y + z * z);
    x /= u;
    y /= u;
    z /= u;

    out.x = x * sizeX;
    out.y = y * sizeX;
    out.z = z * sizeX;
}

inline void rotatePatch(thread Patch& ptch, int rx, int ry)
{
    float3 n, y, x;

    // rotate patch around x axis by angle rx
    rotPointAroundVect(n, ptch.n, ptch.x, rx);
    rotPointAroundVect(y, ptch.y, ptch.x, rx);
    ptch.n = n;
    ptch.y = y;

    // rotate new patch around y axis by angle ry
    rotPointAroundVect(n, ptch.n, ptch.y, ry);
    rotPointAroundVect(x, ptch.x, ptch.y, ry);
    ptch.n = n;
    ptch.x = x;
}

inline void movePatch(device Patch& ptch, int pt)
{
    // float3 v = ptch.p-rC;
    // normalize(v);
    float3 v = ptch.n;

    float d = ptch.d * (float)pt;
    float3 p = ptch.p + v * d;
    ptch.p = p;
}

inline void movePatch(thread Patch& ptch, int pt)
{
    // float3 v = ptch.p-rC;
    // normalize(v);
    float3 v = ptch.n;

    float d = ptch.d * (float)pt;
    float3 p = ptch.p + v * d;
    ptch.p = p;
}

inline void computeRotCS(device float3& xax, device float3& yax, device float3& n)
{
    xax.x = -n.y + n.z; // get any cross product
    xax.y = +n.x + n.z;
    xax.z = -n.x - n.y;
    if(fabs(xax.x) < 0.0000001f && fabs(xax.y) < 0.0000001f && fabs(xax.z) < 0.0000001f)
    {
        xax.x = -n.y - n.z; // get any cross product (complementar)
        xax.y = +n.x - n.z;
        xax.z = +n.x + n.y;
    };
    normalize(xax);
    yax = cross(n, xax);
}

inline void computeRotCS(thread float3& xax, thread float3& yax, thread float3& n)
{
    xax.x = -n.y + n.z; // get any cross product
    xax.y = +n.x + n.z;
    xax.z = -n.x - n.y;
    if(fabs(xax.x) < 0.0000001f && fabs(xax.y) < 0.0000001f && fabs(xax.z) < 0.0000001f)
    {
        xax.x = -n.y - n.z; // get any cross product (complementar)
        xax.y = +n.x - n.z;
        xax.z = +n.x + n.y;
    };
    normalize(xax);
    yax = cross(n, xax);
}

inline void computeRotCSEpip(thread Patch& ptch,
                             constant const DeviceCameraParams& rcDeviceCamParams,
                             constant const DeviceCameraParams& tcDeviceCamParams)
{
    // Vector from the reference camera to the 3d point
    float3 v1 = rcDeviceCamParams.C - ptch.p;
    // Vector from the target camera to the 3d point
    float3 v2 = tcDeviceCamParams.C - ptch.p;
    normalize(v1);
    normalize(v2);

    // y has to be orthogonal to the epipolar plane
    // n has to be on the epipolar plane
    // x has to be on the epipolar plane

    ptch.y = cross(v1, v2);
    normalize(ptch.y); // TODO: v1 & v2 are already normalized

    ptch.n = (v1 + v2) / 2.0f; // IMPORTANT !!!
    normalize(ptch.n); // TODO: v1 & v2 are already normalized
    // ptch.n = sg_s_r.ZVect; //IMPORTANT !!!

    ptch.x = cross(ptch.y, ptch.n);
    normalize(ptch.x);
}

inline void computeRotCSEpip(device Patch& ptch,
                             constant const DeviceCameraParams& rcDeviceCamParams,
                             constant const DeviceCameraParams& tcDeviceCamParams)
{
    // Vector from the reference camera to the 3d point
    float3 v1 = rcDeviceCamParams.C - ptch.p;
    // Vector from the target camera to the 3d point
    float3 v2 = tcDeviceCamParams.C - ptch.p;
    normalize(v1);
    normalize(v2);

    // y has to be orthogonal to the epipolar plane
    // n has to be on the epipolar plane
    // x has to be on the epipolar plane

    ptch.y = cross(v1, v2);
    normalize(ptch.y); // TODO: v1 & v2 are already normalized

    ptch.n = (v1 + v2) / 2.0f; // IMPORTANT !!!
    normalize(ptch.n); // TODO: v1 & v2 are already normalized
    // ptch.n = sg_s_r.ZVect; //IMPORTANT !!!

    ptch.x = cross(ptch.y, ptch.n);
    normalize(ptch.x);
}

inline float computePixSize(constant const DeviceCameraParams& deviceCamParams, thread const float3& p)
{
    DeviceCameraParams deviceCamParams_tmp = deviceCamParams;
    const float2 rp = project3DPoint(deviceCamParams_tmp.P, p);
    const float2 rp1 = rp + make_float2(1.0f, 0.0f);

    float3 refvect = M3x3mulV2(deviceCamParams_tmp.iP, rp1);
    normalize(refvect);
    return pointLineDistance3D(p, deviceCamParams_tmp.C, refvect);
}

inline void getPixelFor3DPoint(thread float2& out, constant const DeviceCameraParams& deviceCamParams, thread const float3& X)
{
    DeviceCameraParams deviceCamParams_tmp = deviceCamParams;
    const float3 p = M3x4mulV3(deviceCamParams_tmp.P, X);

    if(p.z <= 0.0f)
        out = make_float2(-1.0f, -1.0f);
    else
        out = make_float2(p.x / p.z, p.y / p.z);
}

inline float3 get3DPointForPixelAndFrontoParellePlaneRC(constant const DeviceCameraParams& deviceCamParams, thread const float2& pix, float fpPlaneDepth)
{
    DeviceCameraParams deviceCamParams_tmp = deviceCamParams;
    const float3 planep = deviceCamParams.C + deviceCamParams.ZVect * fpPlaneDepth;
    float3 v = M3x3mulV2(deviceCamParams_tmp.iP, pix);
    normalize(v);
    return linePlaneIntersect(deviceCamParams_tmp.C, v, planep, deviceCamParams_tmp.ZVect);
}

inline float3 get3DPointForPixelAndDepthFromRC(constant const DeviceCameraParams& deviceCamParams, thread const float2& pix, float depth)
{
    DeviceCameraParams deviceCamParams_tmp = deviceCamParams;
    float3 rpv = M3x3mulV2(deviceCamParams_tmp.iP, pix);
    normalize(rpv);
    return deviceCamParams_tmp.C + rpv * depth;
}

inline float3 triangulateMatchRef(constant const DeviceCameraParams& rcDeviceCamParams,
                                  constant const DeviceCameraParams& tcDeviceCamParams,
                                  thread const float2& refpix,
                                  thread const float2& tarpix)
{
    DeviceCameraParams rcDeviceCamParams_tmp = rcDeviceCamParams;
    DeviceCameraParams tcDeviceCamParams_tmp = tcDeviceCamParams;
    float3 refvect = M3x3mulV2(rcDeviceCamParams_tmp.iP, refpix);
    normalize(refvect);
    const float3 refpoint = refvect + rcDeviceCamParams.C;

    float3 tarvect = M3x3mulV2(tcDeviceCamParams_tmp.iP, tarpix);
    normalize(tarvect);
    const float3 tarpoint = tarvect + tcDeviceCamParams.C;

    float k, l;
    float3 lli1, lli2;

    lineLineIntersect(&k, &l, &lli1, &lli2, rcDeviceCamParams_tmp.C, refpoint, tcDeviceCamParams_tmp.C, tarpoint);

    return rcDeviceCamParams.C + refvect * k;
}

/**
 * @brief Subpixel refine by Stereo Matching with Color-Weighted Correlation, Hierarchical Belief Propagation,
 *        and Occlusion Handling Qingxiong pami08.
 *
 * @note Quadratic polynomial interpolation is used to approximate the cost function between
 *       three discrete depth candidates: d, dA, and dB.
 *
 * @see https://pubmed.ncbi.nlm.nih.gov/19147877/
 *
 * @param[in] depths the 3 depths candidates (depth-1, depth, depth+1)
 * @param[in] sims the similarity of the 3 depths candidates
 *
 * @return refined depth value
 */
inline float refineDepthSubPixel(device const float3& depths, device const float3& sims)
{
    // TODO: get formula back from paper as it has been lost by encoding.
    // d is the discrete depth with the minimal cost, dA ? d A 1, and dB ? d B 1. The cost function is approximated as
    // f?x? ? ax2 B bx B c.

    float simM1 = sims.x;
    float sim = sims.y;
    float simP1 = sims.z;
    simM1 = (simM1 + 1.0f) / 2.0f;
    sim = (sim + 1.0f) / 2.0f;
    simP1 = (simP1 + 1.0f) / 2.0f;

    // sim is supposed to be the best one (so the smallest one)
    if((simM1 < sim) || (simP1 < sim))
        return depths.y; // return the input

    float dispStep = -((simP1 - simM1) / (2.0f * (simP1 + simM1 - 2.0f * sim)));

    float floatDepthM1 = depths.x;
    float floatDepthP1 = depths.z;

    //-1 : floatDepthM1
    // 0 : floatDepth
    //+1 : floatDepthP1
    // linear function fit
    // f(x)=a*x+b
    // floatDepthM1=-a+b
    // floatDepthP1= a+b
    // a = b - floatDepthM1
    // floatDepthP1=2*b-floatDepthM1
    float b = (floatDepthP1 + floatDepthM1) / 2.0f;
    float a = b - floatDepthM1;

    float interpDepth = a * dispStep + b;

    // Ensure that the interpolated value is isfinite  (i.e. neither infinite nor NaN)
    if(!isfinite(interpDepth) || interpDepth <= 0.0f)
        return depths.y; // return the input

    return interpDepth;
}

/**
 * @brief Subpixel refine by Stereo Matching with Color-Weighted Correlation, Hierarchical Belief Propagation,
 *        and Occlusion Handling Qingxiong pami08.
 *
 * @note Quadratic polynomial interpolation is used to approximate the cost function between
 *       three discrete depth candidates: d, dA, and dB.
 *
 * @see https://pubmed.ncbi.nlm.nih.gov/19147877/
 *
 * @param[in] depths the 3 depths candidates (depth-1, depth, depth+1)
 * @param[in] sims the similarity of the 3 depths candidates
 *
 * @return refined depth value
 */
inline float refineDepthSubPixel(thread const float3& depths, thread const float3& sims)
{
    // TODO: get formula back from paper as it has been lost by encoding.
    // d is the discrete depth with the minimal cost, dA ? d A 1, and dB ? d B 1. The cost function is approximated as
    // f?x? ? ax2 B bx B c.

    float simM1 = sims.x;
    float sim = sims.y;
    float simP1 = sims.z;
    simM1 = (simM1 + 1.0f) / 2.0f;
    sim = (sim + 1.0f) / 2.0f;
    simP1 = (simP1 + 1.0f) / 2.0f;

    // sim is supposed to be the best one (so the smallest one)
    if((simM1 < sim) || (simP1 < sim))
        return depths.y; // return the input

    float dispStep = -((simP1 - simM1) / (2.0f * (simP1 + simM1 - 2.0f * sim)));

    float floatDepthM1 = depths.x;
    float floatDepthP1 = depths.z;

    //-1 : floatDepthM1
    // 0 : floatDepth
    //+1 : floatDepthP1
    // linear function fit
    // f(x)=a*x+b
    // floatDepthM1=-a+b
    // floatDepthP1= a+b
    // a = b - floatDepthM1
    // floatDepthP1=2*b-floatDepthM1
    float b = (floatDepthP1 + floatDepthM1) / 2.0f;
    float a = b - floatDepthM1;

    float interpDepth = a * dispStep + b;

    // Ensure that the interpolated value is isfinite  (i.e. neither infinite nor NaN)
    if(!isfinite(interpDepth) || interpDepth <= 0.0f)
        return depths.y; // return the input

    return interpDepth;
}

/**
 * @brief Subpixel refine by Stereo Matching with Color-Weighted Correlation, Hierarchical Belief Propagation,
 *        and Occlusion Handling Qingxiong pami08.
 *
 * @note Quadratic polynomial interpolation is used to approximate the cost function between
 *       three discrete depth candidates: d, dA, and dB.
 *
 * @see https://pubmed.ncbi.nlm.nih.gov/19147877/
 *
 * @param[in] depths the 3 depths candidates (depth-1, depth, depth+1)
 * @param[in] sims the similarity of the 3 depths candidates
 *
 * @return refined depth value
 */
inline float refineDepthSubPixel(constant const float3& depths, constant const float3& sims)
{
    // TODO: get formula back from paper as it has been lost by encoding.
    // d is the discrete depth with the minimal cost, dA ? d A 1, and dB ? d B 1. The cost function is approximated as
    // f?x? ? ax2 B bx B c.

    float simM1 = sims.x;
    float sim = sims.y;
    float simP1 = sims.z;
    simM1 = (simM1 + 1.0f) / 2.0f;
    sim = (sim + 1.0f) / 2.0f;
    simP1 = (simP1 + 1.0f) / 2.0f;

    // sim is supposed to be the best one (so the smallest one)
    if((simM1 < sim) || (simP1 < sim))
        return depths.y; // return the input

    float dispStep = -((simP1 - simM1) / (2.0f * (simP1 + simM1 - 2.0f * sim)));

    float floatDepthM1 = depths.x;
    float floatDepthP1 = depths.z;

    //-1 : floatDepthM1
    // 0 : floatDepth
    //+1 : floatDepthP1
    // linear function fit
    // f(x)=a*x+b
    // floatDepthM1=-a+b
    // floatDepthP1= a+b
    // a = b - floatDepthM1
    // floatDepthP1=2*b-floatDepthM1
    float b = (floatDepthP1 + floatDepthM1) / 2.0f;
    float a = b - floatDepthM1;

    float interpDepth = a * dispStep + b;

    // Ensure that the interpolated value is isfinite  (i.e. neither infinite nor NaN)
    if(!isfinite(interpDepth) || interpDepth <= 0.0f)
        return depths.y; // return the input

    return interpDepth;
}

inline void computeRcTcMipmapLevels(thread float& out_rcMipmapLevel,
                                    thread float& out_tcMipmapLevel,
                                    const float mipmapLevel,
                                    constant const DeviceCameraParams& rcDeviceCamParams,
                                    constant const DeviceCameraParams& tcDeviceCamParams,
                                    thread const float2& rp0,
                                    thread const float2& tp0,
                                    thread const float3& p0)
{
    DeviceCameraParams rcDeviceCamParams_tmp = rcDeviceCamParams;
    DeviceCameraParams tcDeviceCamParams_tmp = tcDeviceCamParams;
    // get p0 depth from the R camera
    const float rcDepth = size(rcDeviceCamParams.C - p0);

    // get p0 depth from the T camera
    const float tcDepth = size(tcDeviceCamParams.C - p0);

    // get R p0 corresponding pixel + 1x
    const float2 rp1 = rp0 + make_float2(1.f, 0.f);

    // get T p0 corresponding pixel + 1x
    const float2 tp1 = tp0 + make_float2(1.f, 0.f);

    // get rp1 3d point
    float3 rpv = M3x3mulV2(rcDeviceCamParams_tmp.iP, rp1);
    normalize(rpv);
    const float3 prp1 = rcDeviceCamParams.C + rpv * rcDepth;

    // get tp1 3d point
    float3 tpv = M3x3mulV2(tcDeviceCamParams_tmp.iP, tp1);
    normalize(tpv);
    const float3 ptp1 = tcDeviceCamParams.C + tpv * tcDepth;

    // compute 3d distance between p0 and rp1 3d point
    const float rcDist = dist(p0, prp1);

    // compute 3d distance between p0 and tp1 3d point
    const float tcDist = dist(p0, ptp1);

    // compute Rc/Tc distance factor
    const float distFactor = rcDist / tcDist;

    // set output R and T mipmap level
    if(distFactor < 1.f)
    {
        // T camera has a lower resolution (1 Rc pixSize < 1 Tc pixSize)
        out_tcMipmapLevel = mipmapLevel - log2(1.f / distFactor);

        if(out_tcMipmapLevel < 0.f)
        {
          out_rcMipmapLevel = mipmapLevel + abs(out_tcMipmapLevel);
          out_tcMipmapLevel = 0.f;
        }
    }
    else
    {
        // T camera has a higher resolution (1 Rc pixSize > 1 Tc pixSize)
        out_rcMipmapLevel = mipmapLevel;
        out_tcMipmapLevel = mipmapLevel + log2(distFactor);
    }
}

inline int angleBetwUnitV1andUnitV2(device float3& V1, device float3& V2)
{
    return (int)fabs(acos(V1.x * V2.x + V1.y * V2.y + V1.z * V2.z) / (M_PI_F / 180.0f));
}

inline int angleBetwUnitV1andUnitV2(thread float3& V1, thread float3& V2)
{
    return (int)fabs(acos(V1.x * V2.x + V1.y * V2.y + V1.z * V2.z) / (M_PI_F / 180.0f));
}

inline int angleBetwUnitV1andUnitV2(constant float3& V1, constant float3& V2)
{
    return (int)fabs(acos(V1.x * V2.x + V1.y * V2.y + V1.z * V2.z) / (M_PI_F / 180.0f));
}

/*
__device__ inline float getRefCamPixSize(Patch &ptch)
{
        float2 rp = project3DPoint(sg_s_r.P,ptch.p);

        float minstep=10000000.0f;
        for (int i=0;i<4;i++) {
                float2 pix = rp;
                if (i==0) {pix.x += 1.0f;};
                if (i==1) {pix.x -= 1.0f;};
                if (i==2) {pix.y += 1.0f;};
                if (i==3) {pix.y -= 1.0f;};
                float3 vect = M3x3mulV2(sg_s_r.iP,pix);
                float3 lpi = linePlaneIntersect(sg_s_r.C, vect, ptch.p, ptch.n);
                float step = dist(lpi,ptch.p);
                minstep = fminf(minstep,step);
        };

        return minstep;
}

__device__ inline float getTarCamPixSize(Patch &ptch)
{
        float2 tp = project3DPoint(sg_s_t.P,ptch.p);

        float minstep=10000000.0f;
        for (int i=0;i<4;i++) {
                float2 pix = tp;
                if (i==0) {pix.x += 1.0f;};
                if (i==1) {pix.x -= 1.0f;};
                if (i==2) {pix.y += 1.0f;};
                if (i==3) {pix.y -= 1.0f;};
                float3 vect = M3x3mulV2(sg_s_t.iP,pix);
                float3 lpi = linePlaneIntersect(sg_s_t.C, vect, ptch.p, ptch.n);
                float step = dist(lpi,ptch.p);
                minstep = fminf(minstep,step);
        };

        return minstep;
}

__device__ inline float getPatchPixSize(Patch &ptch)
{
        return fmaxf(getTarCamPixSize(ptch),getRefCamPixSize(ptch));
}
*/

inline void computeHomography(thread float* out_H,
                              constant const DeviceCameraParams& rcDeviceCamParams,
                              constant const DeviceCameraParams& tcDeviceCamParams,
                              thread const float3& in_p,
                              thread const float3& in_n)
{
    DeviceCameraParams rcDeviceCamParams_tmp = rcDeviceCamParams;
    DeviceCameraParams tcDeviceCamParams_tmp = tcDeviceCamParams;
    // hartley zisserman second edition p.327 (13.2)
    const float3 _tl = make_float3(0.0, 0.0, 0.0) - M3x3mulV3(rcDeviceCamParams_tmp.R, rcDeviceCamParams_tmp.C);
    const float3 _tr = make_float3(0.0, 0.0, 0.0) - M3x3mulV3(tcDeviceCamParams_tmp.R, tcDeviceCamParams_tmp.C);

    const float3 p = M3x3mulV3(rcDeviceCamParams_tmp.R, (in_p - rcDeviceCamParams_tmp.C));
    float3 n = M3x3mulV3(rcDeviceCamParams_tmp.R, in_n);
    normalize(n);
    const float d = -dot(n, p);

    float RrT[9];
    M3x3transpose(RrT, rcDeviceCamParams_tmp.R);

    float tmpRr[9];
    M3x3mulM3x3(tmpRr, tcDeviceCamParams_tmp.R, RrT);
    const float3 tr = _tr - M3x3mulV3(tmpRr, _tl);

    float tmp[9];
    float tmp1[9];
    outerMultiply(tmp, tr, n / d);
    M3x3minusM3x3(tmp, tmpRr, tmp);
    M3x3mulM3x3(tmp1, tcDeviceCamParams_tmp.K, tmp);
    M3x3mulM3x3(tmp, tmp1, rcDeviceCamParams_tmp.iK);

    out_H[0] = tmp[0];
    out_H[1] = tmp[1];
    out_H[2] = tmp[2];
    out_H[3] = tmp[3];
    out_H[4] = tmp[4];
    out_H[5] = tmp[5];
    out_H[6] = tmp[6];
    out_H[7] = tmp[7];
    out_H[8] = tmp[8];
}

/*
__device__ static float compNCCbyH(const DeviceCameraParams& rcDeviceCamParams,
                                   const DeviceCameraParams& tcDeviceCamParams,
                                   const Patch& ptch,
                                   int wsh)
{
    // get R and T image 2d coordinates from patch center 3d point
    const float2 rp = project3DPoint(rcDeviceCamParams.P, patch.p);
    //const float2 tp = project3DPoint(tcDeviceCamParams.P, patch.p);

    float H[9];
    computeHomography(H, rcDeviceCamParams, tcDeviceCamParams, ptch.p, ptch.n);

    simStat sst = simStat();

    for(int xp = -wsh; xp <= wsh; ++xp)
    {
        for(int yp = -wsh; yp <= wsh; ++yp)
        {
            float2 rpc;
            float2 tpc;
            rpc.x = rp.x + (float)xp;
            rpc.y = rp.y + (float)yp;
            tpc = V2M3x3mulV2(H, rpc);

            float2 g;
            g.x = 255.0f * tex2D(rtex, rpc.x + 0.5f, rpc.y + 0.5f);
            g.y = 255.0f * tex2D(ttex, tpc.x + 0.5f, tpc.y + 0.5f);
            sst.update(g);
        }
    }

    sst.computeSim();

    return sst.sim;
}
*/

/**
 * @brief Compute Normalized Cross-Correlation of a full square patch at given half-width.
 *
 * @tparam TInvertAndFilter invert and filter output similarity value
 *
 * @param[in] rcDeviceCameraParamsId the R camera parameters in device constant memory array
 * @param[in] tcDeviceCameraParamsId the T camera parameters in device constant memory array
 * @param[in] rcMipmapImage_tex the R camera mipmap image texture
 * @param[in] tcMipmapImage_tex the T camera mipmap image texture
 * @param[in] rcLevelWidth the R camera image width at given mipmapLevel
 * @param[in] rcLevelHeight the R camera image height at given mipmapLevel
 * @param[in] tcLevelWidth the T camera image width at given mipmapLevel
 * @param[in] tcLevelHeight the T camera image height at given mipmapLevel
 * @param[in] mipmapLevel the workflow current mipmap level (e.g. SGM=1.f, Refine=0.f)
 * @param[in] wsh the half-width of the patch
 * @param[in] invGammaC the inverted strength of grouping by color similarity
 * @param[in] invGammaP the inverted strength of grouping by proximity
 * @param[in] useConsistentScale enable consistent scale patch comparison
 * @param[in] tcLevelWidth the T camera image width at given mipmapLevel
 * @param[in] patch the input patch struct
 *
 * @return similarity value in range (-1.f, 0.f) or (0.f, 1.f) if TinvertAndFilter enabled
 *         special cases:
 *          -> infinite similarity value: 1
 *          -> invalid/uninitialized/masked similarity: CUDART_INF_F
 */
template<bool TInvertAndFilter>
inline float compNCCby3DptsYK(constant const DeviceCameraParams& rcDeviceCamParams,
                              constant const DeviceCameraParams& tcDeviceCamParams,
                              thread const MTLMipmappedTexture<MTLRGBA>& rcMipmapImage_tex, // As per CUDA implementation, use Bilinear filtering and normalized coordinates
                              thread const MTLMipmappedTexture<MTLRGBA>& tcMipmapImage_tex, // As per CUDA implementation, use Bilinear filtering and normalized coordinates
                              const unsigned int rcLevelWidth,
                              const unsigned int rcLevelHeight,
                              const unsigned int tcLevelWidth,
                              const unsigned int tcLevelHeight,
                              const float mipmapLevel,
                              const int wsh,
                              const float invGammaC,
                              const float invGammaP,
                              const bool useConsistentScale,
                              thread const Patch& patch)
{
    DeviceCameraParams rcDeviceCamParams_tmp = rcDeviceCamParams;
    DeviceCameraParams tcDeviceCamParams_tmp = tcDeviceCamParams;
    // get R and T image 2d coordinates from patch center 3d point
    const float2 rp = project3DPoint(rcDeviceCamParams_tmp.P, patch.p);
    const float2 tp = project3DPoint(tcDeviceCamParams_tmp.P, patch.p);

    // image 2d coordinates margin
    const float dd = wsh + 2.0f; // TODO: FACA

    // check R and T image 2d coordinates
    if((rp.x < dd) || (rp.x > float(rcLevelWidth  - 1) - dd) ||
       (tp.x < dd) || (tp.x > float(tcLevelWidth  - 1) - dd) ||
       (rp.y < dd) || (rp.y > float(rcLevelHeight - 1) - dd) ||
       (tp.y < dd) || (tp.y > float(tcLevelHeight - 1) - dd))
    {
        return INFINITY; // uninitialized
    }

    // compute inverse width / height
    // note: useful to compute normalized coordinates
    const float rcInvLevelWidth  = 1.f / float(rcLevelWidth);
    const float rcInvLevelHeight = 1.f / float(rcLevelHeight);
    const float tcInvLevelWidth  = 1.f / float(tcLevelWidth);
    const float tcInvLevelHeight = 1.f / float(tcLevelHeight);

    // initialize R and T mipmap image level at the given mipmap image level
    float rcMipmapLevel = mipmapLevel;
    float tcMipmapLevel = mipmapLevel;

    // update R and T mipmap image level in order to get consistent scale patch comparison
    if(useConsistentScale)
    {
        computeRcTcMipmapLevels(rcMipmapLevel, tcMipmapLevel, mipmapLevel, rcDeviceCamParams, tcDeviceCamParams, rp, tp, patch.p);
    }

    // create and initialize SimStat struct
    simStat sst;

    // compute patch center color (CIELAB) at R and T mipmap image level
    const MTLRGBA rcCenterColor = rcMipmapImage_tex.tex2DLodNorm<FilterMode::Bilinear>((rp.x + 0.5f) * rcInvLevelWidth, (rp.y + 0.5f) * rcInvLevelHeight, rcMipmapLevel);
    const MTLRGBA tcCenterColor = tcMipmapImage_tex.tex2DLodNorm<FilterMode::Bilinear>((tp.x + 0.5f) * tcInvLevelWidth, (tp.y + 0.5f) * tcInvLevelHeight, tcMipmapLevel);
    
    // Create alias
    float4 rcCenterColor_float(rcCenterColor.x, rcCenterColor.y, rcCenterColor.z, rcCenterColor.w);
    float4 tcCenterColor_float(tcCenterColor.x, tcCenterColor.y, tcCenterColor.z, tcCenterColor.w);

    // check the alpha values of the patch pixel center of the R and T cameras
    if(rcCenterColor.w < ALICEVISION_DEPTHMAP_RC_MIN_ALPHA || tcCenterColor.w < ALICEVISION_DEPTHMAP_TC_MIN_ALPHA)
    {
        return INFINITY; // masked
    }

    // compute patch (wsh*2+1)x(wsh*2+1)
    for(int yp = -wsh; yp <= wsh; ++yp)
    {
        for(int xp = -wsh; xp <= wsh; ++xp)
        {
            // get 3d point
            const float3 p = patch.p + patch.x * float(patch.d * float(xp)) + patch.y * float(patch.d * float(yp));

            // get R and T image 2d coordinates from 3d point
            const float2 rpc = project3DPoint(rcDeviceCamParams_tmp.P, p);
            const float2 tpc = project3DPoint(tcDeviceCamParams_tmp.P, p);

            // get R and T image color (CIELAB) from 2d coordinates
            const MTLRGBA rcPatchCoordColor = rcMipmapImage_tex.tex2DLodNorm<FilterMode::Bilinear>((rpc.x + 0.5f) * rcInvLevelWidth, (rpc.y + 0.5f) * rcInvLevelHeight, rcMipmapLevel);
            const MTLRGBA tcPatchCoordColor = tcMipmapImage_tex.tex2DLodNorm<FilterMode::Bilinear>((tpc.x + 0.5f) * tcInvLevelWidth, (tpc.y + 0.5f) * tcInvLevelHeight, tcMipmapLevel);

            // Create alias
            float4 rcPatchCoordColor_float(rcPatchCoordColor.x, rcPatchCoordColor.y, rcPatchCoordColor.z, rcPatchCoordColor.w);
            float4 tcPatchCoordColor_float(tcPatchCoordColor.x, tcPatchCoordColor.y, tcPatchCoordColor.z, tcPatchCoordColor.w);
            
            // compute weighting based on:
            // - color difference to the center pixel of the patch:
            //    - low value (close to 0) means that the color is different from the center pixel (ie. strongly supported surface)
            //    - high value (close to 1) means that the color is close the center pixel (ie. uniform color)
            // - distance in image to the center pixel of the patch:
            //    - low value (close to 0) means that the pixel is close to the center of the patch
            //    - high value (close to 1) means that the pixel is far from the center of the patch
            const float w = CostYKfromLab(xp, yp, rcCenterColor_float, rcPatchCoordColor_float, invGammaC, invGammaP) * CostYKfromLab(xp, yp, tcCenterColor_float, tcPatchCoordColor_float, invGammaC, invGammaP);

            // update simStat
            sst.update(rcPatchCoordColor.x, tcPatchCoordColor.x, w);
        }
    }

    if(TInvertAndFilter)
    {
        // compute patch similarity
        const float fsim = sst.computeWSim();

        // invert and filter similarity
        // apply sigmoid see: https://www.desmos.com/calculator/skmhf1gpyf
        // best similarity value was -1, worst was 0
        // best similarity value is 1, worst is still 0
        return sigmoid(0.0f, 1.0f, 0.7f, -0.7f, fsim);
    }

    // compute output patch similarity
    return sst.computeWSim();
}

/**
 * @brief Compute Normalized Cross-Correlation of a patch with an user custom patch pattern.
 *
 * @tparam TInvertAndFilter invert and filter output similarity value
 *
 * @param[in] rcDeviceCameraParamsId the R camera parameters in device constant memory array
 * @param[in] tcDeviceCameraParamsId the T camera parameters in device constant memory array
 * @param[in] rcMipmapImage_tex the R camera mipmap image texture
 * @param[in] tcMipmapImage_tex the T camera mipmap image texture
 * @param[in] rcLevelWidth the R camera image width at given mipmapLevel
 * @param[in] rcLevelHeight the R camera image height at given mipmapLevel
 * @param[in] tcLevelWidth the T camera image width at given mipmapLevel
 * @param[in] tcLevelHeight the T camera image height at given mipmapLevel
 * @param[in] mipmapLevel the workflow current mipmap level (e.g. SGM=1.f, Refine=0.f)
 * @param[in] invGammaC the inverted strength of grouping by color similarity
 * @param[in] invGammaP the inverted strength of grouping by proximity
 * @param[in] useConsistentScale enable consistent scale patch comparison
 * @param[in] patch the input patch struct
 *
 * @return similarity value in range (-1.f, 0.f) or (0.f, 1.f) if TinvertAndFilter enabled
 *         special cases:
 *          -> infinite similarity value: 1
 *          -> invalid/uninitialized/masked similarity: CUDART_INF_F
 */
template<bool TInvertAndFilter>
inline float compNCCby3DptsYK_customPatchPattern(constant const DeviceCameraParams& rcDeviceCamParams,
                                                 constant const DeviceCameraParams& tcDeviceCamParams,
                                                 constant const DevicePatchPattern& constantPatchPattern,
                                                 thread const MTLMipmappedTexture<MTLRGBA>& rcMipmapImage_tex, // As per CUDA implementation, use Bilinear filtering and normalized coordinates
                                                 thread const MTLMipmappedTexture<MTLRGBA>& tcMipmapImage_tex, // As per CUDA implementation, use Bilinear filtering and normalized coordinates
                                                 const unsigned int rcLevelWidth,
                                                 const unsigned int rcLevelHeight,
                                                 const unsigned int tcLevelWidth,
                                                 const unsigned int tcLevelHeight,
                                                 const float mipmapLevel,
                                                 const float invGammaC,
                                                 const float invGammaP,
                                                 const bool useConsistentScale,
                                                 thread const Patch& patch)
{
    DeviceCameraParams rcDeviceCamParams_tmp = rcDeviceCamParams;
    DeviceCameraParams tcDeviceCamParams_tmp = tcDeviceCamParams;
    // get R and T image 2d coordinates from patch center 3d point
    const float2 rp = project3DPoint(rcDeviceCamParams_tmp.P, patch.p);
    const float2 tp = project3DPoint(tcDeviceCamParams_tmp.P, patch.p);

    // image 2d coordinates margin
    const float dd = 2.f; // TODO: proper wsh handling

    // check R and T image 2d coordinates
    if((rp.x < dd) || (rp.x > float(rcLevelWidth  - 1) - dd) ||
       (tp.x < dd) || (tp.x > float(tcLevelWidth  - 1) - dd) ||
       (rp.y < dd) || (rp.y > float(rcLevelHeight - 1) - dd) ||
       (tp.y < dd) || (tp.y > float(tcLevelHeight - 1) - dd))
    {
        return INFINITY; // uninitialized
    }

    // compute inverse width / height
    // note: useful to compute normalized coordinates
    const float rcInvLevelWidth  = 1.f / float(rcLevelWidth);
    const float rcInvLevelHeight = 1.f / float(rcLevelHeight);
    const float tcInvLevelWidth  = 1.f / float(tcLevelWidth);
    const float tcInvLevelHeight = 1.f / float(tcLevelHeight);

    // get patch center pixel alpha at the given mipmap image level
    const float rcAlpha = rcMipmapImage_tex.tex2DLodNorm<FilterMode::Bilinear>((rp.x + 0.5f) * rcInvLevelWidth, (rp.y + 0.5f) * rcInvLevelHeight, mipmapLevel).w; // alpha only
    const float tcAlpha = tcMipmapImage_tex.tex2DLodNorm<FilterMode::Bilinear>((tp.x + 0.5f) * tcInvLevelWidth, (tp.y + 0.5f) * tcInvLevelHeight, mipmapLevel).w; // alpha only

    // check the alpha values of the patch pixel center of the R and T cameras
    if(rcAlpha < ALICEVISION_DEPTHMAP_RC_MIN_ALPHA || tcAlpha < ALICEVISION_DEPTHMAP_TC_MIN_ALPHA)
    {
        return INFINITY; // masked
    }

    // initialize R and T mipmap image level at the given mipmap image level
    float rcMipmapLevel = mipmapLevel;
    float tcMipmapLevel = mipmapLevel;

    // update R and T mipmap image level in order to get consistent scale patch comparison
    if(useConsistentScale)
    {
        computeRcTcMipmapLevels(rcMipmapLevel, tcMipmapLevel, mipmapLevel, rcDeviceCamParams, tcDeviceCamParams, rp, tp, patch.p);
    }

    // output similarity initialization
    float fsim = 0.f;
    float wsum = 0.f;

    for(int s = 0; s < constantPatchPattern.nbSubparts; ++s)
    {
        // create and initialize patch subpart SimStat
        simStat sst;

        // get patch pattern subpart
        constant const DevicePatchPatternSubpart& subpart = constantPatchPattern.subparts[s];

        // compute patch center color (CIELAB) at subpart level resolution
        const MTLRGBA rcCenterColor = rcMipmapImage_tex.tex2DLodNorm<FilterMode::Bilinear>((rp.x + 0.5f) * rcInvLevelWidth, (rp.y + 0.5f) * rcInvLevelHeight, rcMipmapLevel + subpart.level);
        const MTLRGBA tcCenterColor = tcMipmapImage_tex.tex2DLodNorm<FilterMode::Bilinear>((tp.x + 0.5f) * tcInvLevelWidth, (tp.y + 0.5f) * tcInvLevelHeight, tcMipmapLevel + subpart.level);
        
        // Alias
        float4 rcCenterColor_float(rcCenterColor.x, rcCenterColor.y, rcCenterColor.z, rcCenterColor.w);
        float4 tcCenterColor_float(tcCenterColor.x, tcCenterColor.y, tcCenterColor.z, tcCenterColor.w);

        if(subpart.isCircle)
        {
            for(int c = 0; c < subpart.nbCoordinates; ++c)
            {
                // get patch relative coordinates
                constant const float2& relativeCoord = subpart.coordinates[c];

                // get 3d point from relative coordinates
                const float3 p = patch.p + patch.x * float(patch.d * relativeCoord.x) + patch.y * float(patch.d * relativeCoord.y);

                // get R and T image 2d coordinates from 3d point
                const float2 rpc = project3DPoint(rcDeviceCamParams_tmp.P, p);
                const float2 tpc = project3DPoint(tcDeviceCamParams_tmp.P, p);

                // get R and T image color (CIELAB) from 2d coordinates
                const MTLRGBA rcPatchCoordColor = rcMipmapImage_tex.tex2DLodNorm<FilterMode::Bilinear>((rpc.x + 0.5f) * rcInvLevelWidth, (rpc.y + 0.5f) * rcInvLevelHeight, rcMipmapLevel + subpart.level);
                const MTLRGBA tcPatchCoordColor = tcMipmapImage_tex.tex2DLodNorm<FilterMode::Bilinear>((tpc.x + 0.5f) * tcInvLevelWidth, (tpc.y + 0.5f) * tcInvLevelHeight, tcMipmapLevel + subpart.level);

                // Alias
                float4 rcPatchCoordColor_float(rcPatchCoordColor.x, rcPatchCoordColor.y, rcPatchCoordColor.z, rcPatchCoordColor.w);
                float4 tcPatchCoordColor_float(tcPatchCoordColor.x, tcPatchCoordColor.y, tcPatchCoordColor.z, tcPatchCoordColor.w);
                
                // compute weighting based on color difference to the center pixel of the patch:
                // - low value (close to 0) means that the color is different from the center pixel (ie. strongly supported surface)
                // - high value (close to 1) means that the color is close the center pixel (ie. uniform color)
                const float w = CostYKfromLab(rcCenterColor_float, rcPatchCoordColor_float, invGammaC) * CostYKfromLab(tcCenterColor_float, tcPatchCoordColor_float, invGammaC);

                // update simStat
                sst.update(rcPatchCoordColor.x, tcPatchCoordColor.x, w);
            }
        }
        else // full patch pattern
        {
            for(int yp = -subpart.wsh; yp <= subpart.wsh; ++yp)
            {
                for(int xp = -subpart.wsh; xp <= subpart.wsh; ++xp)
                {
                    // get 3d point
                    const float3 p = patch.p + patch.x * float(patch.d * float(xp) * subpart.downscale) + patch.y * float(patch.d * float(yp) * subpart.downscale);

                    // get R and T image 2d coordinates from 3d point
                    const float2 rpc = project3DPoint(rcDeviceCamParams_tmp.P, p);
                    const float2 tpc = project3DPoint(tcDeviceCamParams_tmp.P, p);

                    // get R and T image color (CIELAB) from 2d coordinates
                    const MTLRGBA rcPatchCoordColor = rcMipmapImage_tex.tex2DLodNorm<FilterMode::Bilinear>((rpc.x + 0.5f) * rcInvLevelWidth, (rpc.y + 0.5f) * rcInvLevelHeight, rcMipmapLevel + subpart.level);
                    const MTLRGBA tcPatchCoordColor = tcMipmapImage_tex.tex2DLodNorm<FilterMode::Bilinear>((tpc.x + 0.5f) * tcInvLevelWidth, (tpc.y + 0.5f) * tcInvLevelHeight, tcMipmapLevel + subpart.level);
                    
                    // Alias
                    // Alias
                    float4 rcPatchCoordColor_float(rcPatchCoordColor.x, rcPatchCoordColor.y, rcPatchCoordColor.z, rcPatchCoordColor.w);
                    float4 tcPatchCoordColor_float(tcPatchCoordColor.x, tcPatchCoordColor.y, tcPatchCoordColor.z, tcPatchCoordColor.w);

                    // compute weighting based on:
                    // - color difference to the center pixel of the patch:
                    //    - low value (close to 0) means that the color is different from the center pixel (ie. strongly supported surface)
                    //    - high value (close to 1) means that the color is close the center pixel (ie. uniform color)
                    // - distance in image to the center pixel of the patch:
                    //    - low value (close to 0) means that the pixel is close to the center of the patch
                    //    - high value (close to 1) means that the pixel is far from the center of the patch
                    const float w = CostYKfromLab(xp, yp, rcCenterColor_float, rcPatchCoordColor_float, invGammaC, invGammaP) * CostYKfromLab(xp, yp, tcCenterColor_float, tcPatchCoordColor_float, invGammaC, invGammaP);

                    // update simStat
                    sst.update(rcPatchCoordColor.x, tcPatchCoordColor.x, w);
                }
            }
        }

        // compute patch subpart similarity
        const float fsimSubpart = sst.computeWSim();

        // similarity value in range (-1.f, 0.f) or invalid
        if(fsimSubpart < 0.f)
        {
            // add patch pattern subpart similarity to patch similarity
            if(TInvertAndFilter)
            {
                // invert and filter similarity
                // apply sigmoid see: https://www.desmos.com/calculator/skmhf1gpyf
                // best similarity value was -1, worst was 0
                // best similarity value is 1, worst is still 0
                const float fsimInverted = sigmoid(0.0f, 1.0f, 0.7f, -0.7f, fsimSubpart);
                fsim += fsimInverted * subpart.weight;

            }
            else
            {
                // weight and add similarity
                fsim += fsimSubpart * subpart.weight;
            }

            // sum subpart weight
            wsum += subpart.weight;
        }
    }

    // invalid patch similarity
    if(wsum == 0.f)
    {
        return INFINITY;
    }

    if(TInvertAndFilter)
    {
        // for now, we do not average
        return fsim;
    }

    // output average similarity
    return (fsim / wsum);
}

} // namespace depthMap
} // namespace aliceVision

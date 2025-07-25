// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <AVDepthMap/Metal/util/MetalTypes.hpp>
#include <AVDepthMap/Metal/device/math.hpp>

// mn MATRIX ADDRESSING: mxy = x*n+y (x-row,y-col), (m-number of rows, n-number of columns)

namespace aliceVision {
namespace depthMap {

inline uchar4 float4_to_uchar4(device const float4& a)
{
    return make_uchar4((unsigned char)a.x, (unsigned char)a.y, (unsigned char)a.z, (unsigned char)a.w);
}

inline uchar4 float4_to_uchar4(thread const float4& a)
{
    return make_uchar4((unsigned char)a.x, (unsigned char)a.y, (unsigned char)a.z, (unsigned char)a.w);
}

inline uchar4 float4_to_uchar4(constant const float4& a)
{
    return make_uchar4((unsigned char)a.x, (unsigned char)a.y, (unsigned char)a.z, (unsigned char)a.w);
}

inline float4 uchar4_to_float4(device const uchar4& a)
{
    return make_float4((float)a.x, (float)a.y, (float)a.z, (float)a.w);
}

inline float4 uchar4_to_float4(thread const uchar4& a)
{
    return make_float4((float)a.x, (float)a.y, (float)a.z, (float)a.w);
}

inline float4 uchar4_to_float4(constant const uchar4& a)
{
    return make_float4((float)a.x, (float)a.y, (float)a.z, (float)a.w);
}

inline float dot(device const float3& a, device const float3& b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

inline float dot(thread const float3& a, thread const float3& b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

inline float dot(constant const float3& a, constant const float3& b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

inline float dot(device const float2& a, device const float2& b)
{
    return a.x * b.x + a.y * b.y;
}

inline float dot(thread const float2& a, thread const float2& b)
{
    return a.x * b.x + a.y * b.y;
}

inline float dot(constant const float2& a, constant const float2& b)
{
    return a.x * b.x + a.y * b.y;
}

inline float size(device const float3& a)
{
    return sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
}

inline float size(thread const float3& a)
{
    return sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
}

inline float size(constant const float3& a)
{
    return sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
}

inline float size(device const float2& a)
{
    return sqrt(a.x * a.x + a.y * a.y);
}

inline float size(thread const float2& a)
{
    return sqrt(a.x * a.x + a.y * a.y);
}

inline float size(constant const float2& a)
{
    return sqrt(a.x * a.x + a.y * a.y);
}

inline float dist(device const float3& a, device const float3& b)
{
    return size(a - b);
}

inline float dist(thread const float3& a, thread const float3& b)
{
    return size(a - b);
}

inline float dist(constant const float3& a, constant const float3& b)
{
    return size(a - b);
}

inline float dist(device const float2& a, device const float2& b)
{
    return size(a - b);
}

inline float dist(thread const float2& a, thread const float2& b)
{
    return size(a - b);
}

inline float dist(constant const float2& a, constant const float2& b)
{
    return size(a - b);
}

inline float3 cross(device const float3& a, device const float3& b)
{
    return make_float3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
}

inline float3 cross(thread const float3& a, thread const float3& b)
{
    return make_float3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
}

inline float3 cross(constant const float3& a, constant const float3& b)
{
    return make_float3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
}

inline void normalize(device float3& a)
{
    // without optimization
    // float d = sqrtf(dot(a, a));
    // a.x /= d;
    // a.y /= d;
    // a.z /= d;

    const float dInv = divide(1.0f, sqrt(dot(a, a)));
    a.x *= dInv;
    a.y *= dInv;
    a.z *= dInv;
}

inline void normalize(thread float3& a)
{
    // without optimization
    // float d = sqrtf(dot(a, a));
    // a.x /= d;
    // a.y /= d;
    // a.z /= d;

    const float dInv = divide(1.0f, sqrt(dot(a, a)));
    a.x *= dInv;
    a.y *= dInv;
    a.z *= dInv;
}

inline void normalize(device float2& a)
{
    // without optimization
    // float d = sqrtf(dot(a, a));
    // a.x /= d;
    // a.y /= d;

    const float dInv = divide(1.0f, sqrt(dot(a, a)));
    a.x *= dInv;
    a.y *= dInv;
}

inline void normalize(thread float2& a)
{
    // without optimization
    // float d = sqrtf(dot(a, a));
    // a.x /= d;
    // a.y /= d;

    const float dInv = divide(1.0f, sqrt(dot(a, a)));
    a.x *= dInv;
    a.y *= dInv;
}

inline float3 M3x3mulV3(device const float* M3x3, device const float3& V)
{
    return make_float3(M3x3[0] * V.x + M3x3[3] * V.y + M3x3[6] * V.z,
                       M3x3[1] * V.x + M3x3[4] * V.y + M3x3[7] * V.z,
                       M3x3[2] * V.x + M3x3[5] * V.y + M3x3[8] * V.z);
}

inline float3 M3x3mulV3(thread const float* M3x3, thread const float3& V)
{
    return make_float3(M3x3[0] * V.x + M3x3[3] * V.y + M3x3[6] * V.z,
                       M3x3[1] * V.x + M3x3[4] * V.y + M3x3[7] * V.z,
                       M3x3[2] * V.x + M3x3[5] * V.y + M3x3[8] * V.z);
}

inline float3 M3x3mulV2(device const float* M3x3, device const float2& V)
{
    return make_float3(M3x3[0] * V.x + M3x3[3] * V.y + M3x3[6],
                       M3x3[1] * V.x + M3x3[4] * V.y + M3x3[7],
                       M3x3[2] * V.x + M3x3[5] * V.y + M3x3[8]);
}

inline float3 M3x3mulV2(thread const float* M3x3, thread const float2& V)
{
    return make_float3(M3x3[0] * V.x + M3x3[3] * V.y + M3x3[6],
                       M3x3[1] * V.x + M3x3[4] * V.y + M3x3[7],
                       M3x3[2] * V.x + M3x3[5] * V.y + M3x3[8]);
}

inline float3 M3x4mulV3(device const float* M3x4, device const float3& V)
{
    return make_float3(M3x4[0] * V.x + M3x4[3] * V.y + M3x4[6] * V.z + M3x4[9],
                       M3x4[1] * V.x + M3x4[4] * V.y + M3x4[7] * V.z + M3x4[10],
                       M3x4[2] * V.x + M3x4[5] * V.y + M3x4[8] * V.z + M3x4[11]);
}

inline float3 M3x4mulV3(thread const float* M3x4, thread const float3& V)
{
    return make_float3(M3x4[0] * V.x + M3x4[3] * V.y + M3x4[6] * V.z + M3x4[9],
                       M3x4[1] * V.x + M3x4[4] * V.y + M3x4[7] * V.z + M3x4[10],
                       M3x4[2] * V.x + M3x4[5] * V.y + M3x4[8] * V.z + M3x4[11]);
}

inline float2 V2M3x3mulV2(device float* M3x3, device float2& V)
{
    const float d = M3x3[2] * V.x + M3x3[5] * V.y + M3x3[8];
    return make_float2((M3x3[0] * V.x + M3x3[3] * V.y + M3x3[6]) / d, (M3x3[1] * V.x + M3x3[4] * V.y + M3x3[7]) / d);
}

inline float2 V2M3x3mulV2(thread float* M3x3, thread float2& V)
{
    const float d = M3x3[2] * V.x + M3x3[5] * V.y + M3x3[8];
    return make_float2((M3x3[0] * V.x + M3x3[3] * V.y + M3x3[6]) / d, (M3x3[1] * V.x + M3x3[4] * V.y + M3x3[7]) / d);
}

inline float2 project3DPoint(device const float* M3x4, device const float3& V)
{
    // without optimization
    // const float3 p = M3x4mulV3(M3x4, V);
    // return make_float2(p.x / p.z, p.y / p.z);

    float3 p = M3x4mulV3(M3x4, V);
    const float pzInv = divide(1.0f, p.z);
    return make_float2(p.x * pzInv, p.y * pzInv);
}

inline float2 project3DPoint(thread const float* M3x4, thread const float3& V)
{
    // without optimization
    // const float3 p = M3x4mulV3(M3x4, V);
    // return make_float2(p.x / p.z, p.y / p.z);

    float3 p = M3x4mulV3(M3x4, V);
    const float pzInv = divide(1.0f, p.z);
    return make_float2(p.x * pzInv, p.y * pzInv);
}

inline void M3x3mulM3x3(device float* O3x3, device const float* A3x3, device const float* B3x3)
{
    O3x3[0] = A3x3[0] * B3x3[0] + A3x3[3] * B3x3[1] + A3x3[6] * B3x3[2];
    O3x3[3] = A3x3[0] * B3x3[3] + A3x3[3] * B3x3[4] + A3x3[6] * B3x3[5];
    O3x3[6] = A3x3[0] * B3x3[6] + A3x3[3] * B3x3[7] + A3x3[6] * B3x3[8];

    O3x3[1] = A3x3[1] * B3x3[0] + A3x3[4] * B3x3[1] + A3x3[7] * B3x3[2];
    O3x3[4] = A3x3[1] * B3x3[3] + A3x3[4] * B3x3[4] + A3x3[7] * B3x3[5];
    O3x3[7] = A3x3[1] * B3x3[6] + A3x3[4] * B3x3[7] + A3x3[7] * B3x3[8];

    O3x3[2] = A3x3[2] * B3x3[0] + A3x3[5] * B3x3[1] + A3x3[8] * B3x3[2];
    O3x3[5] = A3x3[2] * B3x3[3] + A3x3[5] * B3x3[4] + A3x3[8] * B3x3[5];
    O3x3[8] = A3x3[2] * B3x3[6] + A3x3[5] * B3x3[7] + A3x3[8] * B3x3[8];
}

inline void M3x3mulM3x3(thread float* O3x3, thread const float* A3x3, thread const float* B3x3)
{
    O3x3[0] = A3x3[0] * B3x3[0] + A3x3[3] * B3x3[1] + A3x3[6] * B3x3[2];
    O3x3[3] = A3x3[0] * B3x3[3] + A3x3[3] * B3x3[4] + A3x3[6] * B3x3[5];
    O3x3[6] = A3x3[0] * B3x3[6] + A3x3[3] * B3x3[7] + A3x3[6] * B3x3[8];

    O3x3[1] = A3x3[1] * B3x3[0] + A3x3[4] * B3x3[1] + A3x3[7] * B3x3[2];
    O3x3[4] = A3x3[1] * B3x3[3] + A3x3[4] * B3x3[4] + A3x3[7] * B3x3[5];
    O3x3[7] = A3x3[1] * B3x3[6] + A3x3[4] * B3x3[7] + A3x3[7] * B3x3[8];

    O3x3[2] = A3x3[2] * B3x3[0] + A3x3[5] * B3x3[1] + A3x3[8] * B3x3[2];
    O3x3[5] = A3x3[2] * B3x3[3] + A3x3[5] * B3x3[4] + A3x3[8] * B3x3[5];
    O3x3[8] = A3x3[2] * B3x3[6] + A3x3[5] * B3x3[7] + A3x3[8] * B3x3[8];
}

inline void M3x3minusM3x3(device float* O3x3, device float* A3x3, device float* B3x3)
{
    O3x3[0] = A3x3[0] - B3x3[0];
    O3x3[1] = A3x3[1] - B3x3[1];
    O3x3[2] = A3x3[2] - B3x3[2];
    O3x3[3] = A3x3[3] - B3x3[3];
    O3x3[4] = A3x3[4] - B3x3[4];
    O3x3[5] = A3x3[5] - B3x3[5];
    O3x3[6] = A3x3[6] - B3x3[6];
    O3x3[7] = A3x3[7] - B3x3[7];
    O3x3[8] = A3x3[8] - B3x3[8];
}

inline void M3x3minusM3x3(thread float* O3x3, thread float* A3x3, thread float* B3x3)
{
    O3x3[0] = A3x3[0] - B3x3[0];
    O3x3[1] = A3x3[1] - B3x3[1];
    O3x3[2] = A3x3[2] - B3x3[2];
    O3x3[3] = A3x3[3] - B3x3[3];
    O3x3[4] = A3x3[4] - B3x3[4];
    O3x3[5] = A3x3[5] - B3x3[5];
    O3x3[6] = A3x3[6] - B3x3[6];
    O3x3[7] = A3x3[7] - B3x3[7];
    O3x3[8] = A3x3[8] - B3x3[8];
}

inline void M3x3transpose(device float* O3x3, device const float* A3x3)
{
    O3x3[0] = A3x3[0];
    O3x3[1] = A3x3[3];
    O3x3[2] = A3x3[6];
    O3x3[3] = A3x3[1];
    O3x3[4] = A3x3[4];
    O3x3[5] = A3x3[7];
    O3x3[6] = A3x3[2];
    O3x3[7] = A3x3[5];
    O3x3[8] = A3x3[8];
}

inline void M3x3transpose(thread float* O3x3, thread const float* A3x3)
{
    O3x3[0] = A3x3[0];
    O3x3[1] = A3x3[3];
    O3x3[2] = A3x3[6];
    O3x3[3] = A3x3[1];
    O3x3[4] = A3x3[4];
    O3x3[5] = A3x3[7];
    O3x3[6] = A3x3[2];
    O3x3[7] = A3x3[5];
    O3x3[8] = A3x3[8];
}

inline void outerMultiply(device float* O3x3, device const float3& a, device const float3& b)
{
    O3x3[0] = a.x * b.x;
    O3x3[3] = a.x * b.y;
    O3x3[6] = a.x * b.z;
    O3x3[1] = a.y * b.x;
    O3x3[4] = a.y * b.y;
    O3x3[7] = a.y * b.z;
    O3x3[2] = a.z * b.x;
    O3x3[5] = a.z * b.y;
    O3x3[8] = a.z * b.z;
}

inline void outerMultiply(thread float* O3x3, thread const float3& a, thread const float3& b)
{
    O3x3[0] = a.x * b.x;
    O3x3[3] = a.x * b.y;
    O3x3[6] = a.x * b.z;
    O3x3[1] = a.y * b.x;
    O3x3[4] = a.y * b.y;
    O3x3[7] = a.y * b.z;
    O3x3[2] = a.z * b.x;
    O3x3[5] = a.z * b.y;
    O3x3[8] = a.z * b.z;
}

inline float3 linePlaneIntersect(device const float3& linePoint,
                                 device const float3& lineVect,
                                 device const float3& planePoint,
                                 device const float3& planeNormal)
{
    const float k = (dot(planePoint, planeNormal) - dot(planeNormal, linePoint)) / dot(planeNormal, lineVect);
    return linePoint + lineVect * k;
}

inline float3 linePlaneIntersect(thread const float3& linePoint,
                                 thread const float3& lineVect,
                                 thread const float3& planePoint,
                                 thread const float3& planeNormal)
{
    const float k = (dot(planePoint, planeNormal) - dot(planeNormal, linePoint)) / dot(planeNormal, lineVect);
    return linePoint + lineVect * k;
}

inline float3 linePlaneIntersect(constant const float3& linePoint,
                                 constant const float3& lineVect,
                                 constant const float3& planePoint,
                                 constant const float3& planeNormal)
{
    const float k = (dot(planePoint, planeNormal) - dot(planeNormal, linePoint)) / dot(planeNormal, lineVect);
    return linePoint + lineVect * k;
}

inline float3 closestPointOnPlaneToPoint(device const float3& point, device const float3& planePoint, device const float3& planeNormalNormalized)
{
    float3 planeNormalNormalized_tmp = planeNormalNormalized;
    return point - planeNormalNormalized * dot(planeNormalNormalized_tmp, point - planePoint);
}

inline float3 closestPointOnPlaneToPoint(thread const float3& point, thread const float3& planePoint, thread const float3& planeNormalNormalized)
{
    return point - planeNormalNormalized * dot(planeNormalNormalized, point - planePoint);
}

inline float3 closestPointOnPlaneToPoint(constant const float3& point, constant const float3& planePoint, constant const float3& planeNormalNormalized)
{
    float3 planeNormalNormalized_tmp = planeNormalNormalized;
    return point - planeNormalNormalized * dot(planeNormalNormalized_tmp, point - planePoint);
}

inline float3 closestPointToLine3D(device const float3& point, device const float3& linePoint, device const float3& lineVectNormalized)
{
    float3 lineVectNormalized_tmp = lineVectNormalized;
    return linePoint + lineVectNormalized * dot(lineVectNormalized_tmp, point - linePoint);
}

inline float3 closestPointToLine3D(thread const float3& point, thread const float3& linePoint, thread const float3& lineVectNormalized)
{
    return linePoint + lineVectNormalized * dot(lineVectNormalized, point - linePoint);
}

inline float3 closestPointToLine3D(constant const float3& point, constant const float3& linePoint, constant const float3& lineVectNormalized)
{
    float3 lineVectNormalized_tmp = lineVectNormalized;
    return linePoint + lineVectNormalized * dot(lineVectNormalized_tmp, point - linePoint);
}

inline float pointLineDistance3D(device const float3& point, device const float3& linePoint, device const float3& lineVectNormalized)
{
    float3 lineVectNormalized_tmp = lineVectNormalized;
    return size(cross(lineVectNormalized_tmp, linePoint - point));
}

inline float pointLineDistance3D(thread const float3& point, thread const float3& linePoint, thread const float3& lineVectNormalized)
{
    return size(cross(lineVectNormalized, linePoint - point));
}

inline float pointLineDistance3D(constant const float3& point, constant const float3& linePoint, constant const float3& lineVectNormalized)
{
    float3 lineVectNormalized_tmp = lineVectNormalized;
    return size(cross(lineVectNormalized_tmp, linePoint - point));
}

// v1,v2 dot not have to be normalized
inline float angleBetwV1andV2(device const float3& iV1, device const float3& iV2)
{
    float3 V1 = iV1;
    normalize(V1);

    float3 V2 = iV2;
    normalize(V2);

    return fabs(acos(V1.x * V2.x + V1.y * V2.y + V1.z * V2.z) / (M_PI_F / 180.0f));
}

// v1,v2 dot not have to be normalized
inline float angleBetwV1andV2(thread const float3& iV1, thread const float3& iV2)
{
    float3 V1 = iV1;
    normalize(V1);

    float3 V2 = iV2;
    normalize(V2);

    return fabs(acos(V1.x * V2.x + V1.y * V2.y + V1.z * V2.z) / (M_PI_F / 180.0f));
}

// v1,v2 dot not have to be normalized
inline float angleBetwV1andV2(constant const float3& iV1, constant const float3& iV2)
{
    float3 V1 = iV1;
    normalize(V1);

    float3 V2 = iV2;
    normalize(V2);

    return fabs(acos(V1.x * V2.x + V1.y * V2.y + V1.z * V2.z) / (M_PI_F / 180.0f));
}

inline float angleBetwABandAC(device const float3& A, device const float3& B, device const float3& C)
{
    float3 V1 = B - A;
    float3 V2 = C - A;

    normalize(V1);
    normalize(V2);

    const float x = float(V1.x * V2.x + V1.y * V2.y + V1.z * V2.z);
    float a = acos(x);
    a = isinf(a) ? 0.0 : a;
    return float(fabs(a) / (M_PI_F / 180.0));
}

inline float angleBetwABandAC(thread const float3& A, thread const float3& B, thread const float3& C)
{
    float3 V1 = B - A;
    float3 V2 = C - A;

    normalize(V1);
    normalize(V2);

    const float x = float(V1.x * V2.x + V1.y * V2.y + V1.z * V2.z);
    float a = acos(x);
    a = isinf(a) ? 0.0 : a;
    return float(fabs(a) / (M_PI_F / 180.0));
}

inline float angleBetwABandAC(constant const float3& A, constant const float3& B, constant const float3& C)
{
    float3 V1 = B - A;
    float3 V2 = C - A;

    normalize(V1);
    normalize(V2);

    const float x = float(V1.x * V2.x + V1.y * V2.y + V1.z * V2.z);
    float a = acos(x);
    a = isinf(a) ? 0.0 : a;
    return float(fabs(a) / (M_PI_F / 180.0));
}

/**
 * @brief Calculate the line segment PaPb that is the shortest route between two lines p1-p2 and p3-p4.
 *        Calculate also the values of mua and mub where:
 *          -> pa = p1 + mua (p2 - p1)
 *          -> pb = p3 + mub (p4 - p3)
 *
 * @note This a simple conversion to MATLAB of the C code posted by Paul Bourke at:
 *       http://astronomy.swin.edu.au/~pbourke/geometry/lineline3d/
 *       The author of this all too imperfect translation is Cristian Dima (csd@cmu.edu).
 *
 * @see https://web.archive.org/web/20060422045048/http://astronomy.swin.edu.au/~pbourke/geometry/lineline3d/
 */
inline float3 lineLineIntersect(device float* k,
                                device float* l,
                                device float3* lli1,
                                device float3* lli2,
                                device const float3& p1,
                                device const float3& p2,
                                device const float3& p3,
                                device const float3& p4)
{
    float d1343, d4321, d1321, d4343, d2121, denom, numer, p13[3], p43[3], p21[3], pa[3], pb[3], muab[2];

    p13[0] = p1.x - p3.x;
    p13[1] = p1.y - p3.y;
    p13[2] = p1.z - p3.z;

    p43[0] = p4.x - p3.x;
    p43[1] = p4.y - p3.y;
    p43[2] = p4.z - p3.z;

    /*
    if ((abs(p43[0])  < eps) & ...
        (abs(p43[1])  < eps) & ...
        (abs(p43[2])  < eps))
      error('Could not compute LineLineIntersect!');
    end
    */

    p21[0] = p2.x - p1.x;
    p21[1] = p2.y - p1.y;
    p21[2] = p2.z - p1.z;

    /*
    if ((abs(p21[0])  < eps) & ...
        (abs(p21[1])  < eps) & ...
        (abs(p21[2])  < eps))
      error('Could not compute LineLineIntersect!');
    end
    */

    d1343 = p13[0] * p43[0] + p13[1] * p43[1] + p13[2] * p43[2];
    d4321 = p43[0] * p21[0] + p43[1] * p21[1] + p43[2] * p21[2];
    d1321 = p13[0] * p21[0] + p13[1] * p21[1] + p13[2] * p21[2];
    d4343 = p43[0] * p43[0] + p43[1] * p43[1] + p43[2] * p43[2];
    d2121 = p21[0] * p21[0] + p21[1] * p21[1] + p21[2] * p21[2];

    denom = d2121 * d4343 - d4321 * d4321;

    /*
    if (abs(denom) < eps)
      error('Could not compute LineLineIntersect!');
    end
     */

    numer = d1343 * d4321 - d1321 * d4343;

    muab[0] = numer / denom;
    muab[1] = (d1343 + d4321 * muab[0]) / d4343;

    pa[0] = p1.x + muab[0] * p21[0];
    pa[1] = p1.y + muab[0] * p21[1];
    pa[2] = p1.z + muab[0] * p21[2];

    pb[0] = p3.x + muab[1] * p43[0];
    pb[1] = p3.y + muab[1] * p43[1];
    pb[2] = p3.z + muab[1] * p43[2];

    float3 S;
    S.x = (pa[0] + pb[0]) / 2.0;
    S.y = (pa[1] + pb[1]) / 2.0;
    S.z = (pa[2] + pb[2]) / 2.0;

    *k = muab[0];
    *l = muab[1];

    lli1->x = pa[0];
    lli1->y = pa[1];
    lli1->z = pa[2];

    lli2->x = pb[0];
    lli2->y = pb[1];
    lli2->z = pb[2];

    return S;
}

/**
 * @brief Calculate the line segment PaPb that is the shortest route between two lines p1-p2 and p3-p4.
 *        Calculate also the values of mua and mub where:
 *          -> pa = p1 + mua (p2 - p1)
 *          -> pb = p3 + mub (p4 - p3)
 *
 * @note This a simple conversion to MATLAB of the C code posted by Paul Bourke at:
 *       http://astronomy.swin.edu.au/~pbourke/geometry/lineline3d/
 *       The author of this all too imperfect translation is Cristian Dima (csd@cmu.edu).
 *
 * @see https://web.archive.org/web/20060422045048/http://astronomy.swin.edu.au/~pbourke/geometry/lineline3d/
 */
inline float3 lineLineIntersect(thread float* k,
                                thread float* l,
                                thread float3* lli1,
                                thread float3* lli2,
                                thread const float3& p1,
                                thread const float3& p2,
                                thread const float3& p3,
                                thread const float3& p4)
{
    float d1343, d4321, d1321, d4343, d2121, denom, numer, p13[3], p43[3], p21[3], pa[3], pb[3], muab[2];

    p13[0] = p1.x - p3.x;
    p13[1] = p1.y - p3.y;
    p13[2] = p1.z - p3.z;

    p43[0] = p4.x - p3.x;
    p43[1] = p4.y - p3.y;
    p43[2] = p4.z - p3.z;

    /*
    if ((abs(p43[0])  < eps) & ...
        (abs(p43[1])  < eps) & ...
        (abs(p43[2])  < eps))
      error('Could not compute LineLineIntersect!');
    end
    */

    p21[0] = p2.x - p1.x;
    p21[1] = p2.y - p1.y;
    p21[2] = p2.z - p1.z;

    /*
    if ((abs(p21[0])  < eps) & ...
        (abs(p21[1])  < eps) & ...
        (abs(p21[2])  < eps))
      error('Could not compute LineLineIntersect!');
    end
    */

    d1343 = p13[0] * p43[0] + p13[1] * p43[1] + p13[2] * p43[2];
    d4321 = p43[0] * p21[0] + p43[1] * p21[1] + p43[2] * p21[2];
    d1321 = p13[0] * p21[0] + p13[1] * p21[1] + p13[2] * p21[2];
    d4343 = p43[0] * p43[0] + p43[1] * p43[1] + p43[2] * p43[2];
    d2121 = p21[0] * p21[0] + p21[1] * p21[1] + p21[2] * p21[2];

    denom = d2121 * d4343 - d4321 * d4321;

    /*
    if (abs(denom) < eps)
      error('Could not compute LineLineIntersect!');
    end
     */

    numer = d1343 * d4321 - d1321 * d4343;

    muab[0] = numer / denom;
    muab[1] = (d1343 + d4321 * muab[0]) / d4343;

    pa[0] = p1.x + muab[0] * p21[0];
    pa[1] = p1.y + muab[0] * p21[1];
    pa[2] = p1.z + muab[0] * p21[2];

    pb[0] = p3.x + muab[1] * p43[0];
    pb[1] = p3.y + muab[1] * p43[1];
    pb[2] = p3.z + muab[1] * p43[2];

    float3 S;
    S.x = (pa[0] + pb[0]) / 2.0;
    S.y = (pa[1] + pb[1]) / 2.0;
    S.z = (pa[2] + pb[2]) / 2.0;

    *k = muab[0];
    *l = muab[1];

    lli1->x = pa[0];
    lli1->y = pa[1];
    lli1->z = pa[2];

    lli2->x = pb[0];
    lli2->y = pb[1];
    lli2->z = pb[2];

    return S;
}

/**
 * @brief Sigmoid function filtering
 * @note f(x) = min + (max-min) * \frac{1}{1 + e^{10 * (x - mid) / width}}
 * @see https://www.desmos.com/calculator/1qvampwbyx
 */
inline float sigmoid(float zeroVal, float endVal, float sigwidth, float sigMid, float xval)
{
    return zeroVal + (endVal - zeroVal) * (1.0f / (1.0f + expf(10.0f * ((xval - sigMid) / sigwidth))));
}

/**
 * @brief Sigmoid function filtering
 * @note f(x) = min + (max-min) * \frac{1}{1 + e^{10 * (mid - x) / width}}
 */
inline float sigmoid2(float zeroVal, float endVal, float sigwidth, float sigMid, float xval)
{
    return zeroVal + (endVal - zeroVal) * (1.0f / (1.0f + expf(10.0f * ((sigMid - xval) / sigwidth))));
}

} // namespace depthMap
} // namespace aliceVision


//
//  Color.metal
//  AVDepthMapMetalKernels
//
//  Created by Philipp Remy on 14.07.25.
//

#ifndef COLOR_h
#define COLOR_h

#include <metal_stdlib>
using namespace metal;

#include <AVDepthMap/Metal/Math.metal>
#include <AVDepthMap/Metal/Types.hpp>

namespace aliceVision {
namespace depthMap {

// for the R camera, image alpha should be at least 0.9f (computation area)
#define ALICEVISION_DEPTHMAP_RC_MIN_ALPHA (255.f * 0.9f) // texture range (0, 255)

// for the T camera, image alpha should be at least 0.4f (masking)
#define ALICEVISION_DEPTHMAP_TC_MIN_ALPHA (255.f * 0.4f) // texture range (0, 255)

// color Euclidean distance

/**
 * @brief Euclidean distance (float3)
 * @param[in] x1 the first pixel color
 * @param[in] x2 the second pixel color
 * @return distance
 */
inline float euclideanDist3(const float3 x1, const float3 x2)
{
    return sqrt((x1.x - x2.x) * (x1.x - x2.x) + (x1.y - x2.y) * (x1.y - x2.y) + (x1.z - x2.z) * (x1.z - x2.z));
}

/**
 * @brief Euclidean distance (float4, XYZ ignore W)
 * @param[in] x1 the first pixel color
 * @param[in] x2 the second pixel color
 * @return distance
 */
inline float euclideanDist3(const float4 x1, const float4 x2)
{
    return sqrt((x1.x - x2.x) * (x1.x - x2.x) + (x1.y - x2.y) * (x1.y - x2.y) + (x1.z - x2.z) * (x1.z - x2.z));
}

/**
 * @brief Euclidean distance (float3)
 * @param[in] x1 the first pixel color
 * @param[in] x2 the second pixel color
 * @return distance
 */
inline half euclideanDist3(const half3 x1, const half3 x2)
{
    return sqrt((x1.x - x2.x) * (x1.x - x2.x) + (x1.y - x2.y) * (x1.y - x2.y) + (x1.z - x2.z) * (x1.z - x2.z));
}

/**
 * @brief Euclidean distance (float4, XYZ ignore W)
 * @param[in] x1 the first pixel color
 * @param[in] x2 the second pixel color
 * @return distance
 */
inline half euclideanDist3(const half4 x1, const half4 x2)
{
    return sqrt((x1.x - x2.x) * (x1.x - x2.x) + (x1.y - x2.y) * (x1.y - x2.y) + (x1.z - x2.z) * (x1.z - x2.z));
}

// color conversion utils

/**
 * @brief sRGB (0..1) to linear RGB (0..1)
 * @param[in] c the float3 sRGB
 * @return float3 linear RGB
 */
inline float3 srgb2rgb(const float3 c)
{
    return float3(c.x <= 0.04045f ? c.x / 12.92f : pow((c.x + 0.055f) / 1.055f, 2.4f),
                  c.y <= 0.04045f ? c.y / 12.92f : pow((c.y + 0.055f) / 1.055f, 2.4f),
                  c.z <= 0.04045f ? c.z / 12.92f : pow((c.z + 0.055f) / 1.055f, 2.4f));
}

/**
 * @brief Linear RGB (0..1) to XZY (0..1) using sRGB primaries
 * @param[in] c the float3 Linear RGB
 * @return float3 XYZ
 */
inline float3 rgb2xyz(const float3 c)
{
    return float3(0.4124564f * c.x + 0.3575761f * c.y + 0.1804375f * c.z,
                       0.2126729f * c.x + 0.7151522f * c.y + 0.0721750f * c.z,
                       0.0193339f * c.x + 0.1191920f * c.y + 0.9503041f * c.z);
}

/**
 * @brief Linear RGB (0..1) to HSL (0..1)
 * @param[in] c the float3 Linear RGB
 * @return float3 HSL
 */
inline float3 rgb2hsl(constant float3& c)
{
    const float cmin = min(c.x, min(c.y, c.z));
    const float cmax = max(c.x, max(c.y, c.z));

    float h = 0.0f;
    if(cmin == cmax)
    { /* h = 0.0f; */
    }
    else if(cmax == c.x)
    {
        h = ((c.y - c.z) / (cmax - cmin) + 6.0f) / 6.0f;
        if(h >= 1.0f)
            h -= 1.0f;
    }
    else if(cmax == c.y)
    {
        h = ((c.z - c.x) / (cmax - cmin) + 2.0f) / 6.0f;
    }
    else /* if(cmax == c.z) */
    {
        h = ((c.x - c.y) / (cmax - cmin) + 4.0f) / 6.0f;
    }

    const float l = 0.5f * (cmin + cmax);

    float s = 0.0f;
    if(cmin == cmax)
    { /* s = 0.0f; */
    }
    else if(l <= 0.5f)
    {
        s = (cmax - cmin) / (2.0f * l);
    }
    else /* if(l > 0.5f) */
    {
        s = (cmax - cmin) / (2.0f - 2.0f * l);
    }

    return float3(h, s, l);
}

/**
 * @brief XYZ (0..1) to CIELAB (0..255) assuming D65 whitepoint
 * @param[in] c the float3 XYZ
 * @return float3 CIELAB
 */
inline float3 xyz2lab(const float3 c)
{
    // assuming whitepoint D65, XYZ=(0.95047, 1.00000, 1.08883)
    float3 r = float3(c.x / 0.95047f, c.y, c.z / 1.08883f);

    float3 f = float3((r.x > 216.0f / 24389.0f ? cbrt(r.x) : (24389.0f / 27.0f * r.x + 16.0f) / 116.0f),
                      (r.y > 216.0f / 24389.0f ? cbrt(r.y) : (24389.0f / 27.0f * r.y + 16.0f) / 116.0f),
                      (r.z > 216.0f / 24389.0f ? cbrt(r.z) : (24389.0f / 27.0f * r.z + 16.0f) / 116.0f));

    float3 out = float3(116.0f * f.y - 16.0f, 500.0f * (f.x - f.y), 200.0f * (f.y - f.z));

    // convert values to fit into 0..255 (could be out-of-range)
    // TODO FACA: use float textures, the values are out-of-range for a and b.
    out.x = out.x * 2.55f;
    out.y = out.y * 2.55f;
    out.z = out.z * 2.55f;
    return out;
}

/**
 * @brief RGB (uchar4) to gray (float)
 * @param[in] c the uchar4 RGB
 * @return float gray
 */
inline float rgb2gray(const uchar4 c)
{
    return 0.2989f * (float)c.x + 0.5870f * (float)c.y + 0.1140f * (float)c.z;
}

/**
 * @brief Adaptive support-weight approach for correspondence search
 *
 * @note "Adaptive Support-Weight Approach for Correspondence Search", Kuk-Jin Yoon, In So Kweon
 * @see http://koasas.kaist.ac.kr/bitstream/10203/21048/1/000235253300014.pdf
 *
 * @param[in] dx x-axis distance between the two pixels
 * @param[in] dy y-axis distance between the two pixels
 * @param[in] c1 the first patch pixel color (Lab 0..255)
 * @param[in] c2 the second patch pixel color (Lab 0..255)
 * @param[in] invGammaC the inverted strength of grouping by color similarity (gammaC: 5.5 / 105.5)
 * @param[in] invGammaP the inverted strength of grouping by proximity (gammaP: 8 / 4)
 * @return distance value
 */
inline float CostYKfromLab(const int dx,
                           const int dy,
                           const MTLRGBA c1,
                           const MTLRGBA c2,
                           const float invGammaC,
                           const float invGammaP)
{
    // const float deltaC = 0; // ignore colour difference

    //// AD in RGB
    // const float deltaC =
    //    fabsf(float(c1.x) - float(c2.x)) +
    //    fabsf(float(c1.y) - float(c2.y)) +
    //    fabsf(float(c1.z) - float(c2.z));

    //// euclidean distance in RGB
    // const float deltaC = euclideanDist3(
    //    uchar4_to_float3(c1),
    //    uchar4_to_float3(c2)
    //);

    //// euclidean distance in Lab, assuming sRGB
    // const float deltaC = euclideanDist3(
    //    xyz2lab(rgb2xyz(srgb2rgb(uchar4_to_float3(c1)))),
    //    xyz2lab(rgb2xyz(srgb2rgb(uchar4_to_float3(c2))))
    //);

    // euclidean distance in Lab, assuming linear RGB
    float deltaC = euclideanDist3(c1, c2);
    // const float deltaC = fmaxf(fabs(c1.x-c2.x),fmaxf(fabs(c1.y-c2.y),fabs(c1.z-c2.z)));

    deltaC *= invGammaC;

    // spatial distance to the center of the patch (in pixels)
    // without optimization
    float deltaP = sqrt(float(dx * dx + dy * dy));

    deltaP *= invGammaP;

    deltaC += deltaP;

    return exp(-deltaC); // Yoon & Kweon
    // return __expf(-(deltaC * deltaC / (2 * gammaC * gammaC))) * sqrtf(__expf(-(deltaP * deltaP / (2 * gammaP * gammaP)))); // DCB
    // return __expf(-((deltaC * deltaC / 2) * (invGammaC * invGammaC))) * sqrtf(__expf(-(((deltaP * deltaP / 2) * (invGammaP * invGammaP)))); // DCB
}


/**
 * @brief Adaptive support-weight approach for correspondence search
 *
 * @note "Adaptive Support-Weight Approach for Correspondence Search", Kuk-Jin Yoon, In So Kweon
 * @see http://koasas.kaist.ac.kr/bitstream/10203/21048/1/000235253300014.pdf
 *
 * @param[in] c1 the first patch pixel color (Lab 0..255)
 * @param[in] c2 the second patch pixel color (Lab 0..255)
 * @param[in] invGammaC the inverted strength of grouping by color similarity (gammaC: 5.5 / 105.5)
 * @return distance value
 */
inline float CostYKfromLab(const MTLRGBA c1, const MTLRGBA c2, const float invGammaC)
{
    // euclidean distance in Lab, assuming linear RGB
    const float deltaC = euclideanDist3(c1, c2);

    return exp(-(deltaC * invGammaC)); // Yoon & Kweon
}

}
}

#endif

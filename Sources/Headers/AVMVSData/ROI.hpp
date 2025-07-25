// This file is part of the AliceVision project.
// Copyright (c) 2022 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

// allows code sharing between NVCC and other compilers
#if defined(__NVCC__)
    #define CUDA_HOST_DEVICE __host__ __device__
    #define CUDA_HOST __host__
    #define CUDA_CEIL(f) ceil(f)
    #define CUDA_FLOOR(f) floor(f)
    #define CUDA_MIN(a, b) min(a, b)
    #define CUDA_MAX(a, b) max(a, b)
    #define CONSTANT_REF const
#elif defined(__METAL__)
    #include <metal_stdlib>
    using namespace metal;
    #define CUDA_HOST_DEVICE
    #define CUDA_HOST
    #define CUDA_CEIL(f) ceil(f)
    #define CUDA_FLOOR(f) floor(f)
    #define CUDA_MIN(a, b) min(a, b)
    #define CUDA_MAX(a, b) max(a, b)
    #define CONSTANT_REF constant
#else
    #define CUDA_HOST_DEVICE
    #define CUDA_HOST
    #define CUDA_CEIL(f) std::ceil(f)
    #define CUDA_FLOOR(f) std::floor(f)
    #define CUDA_MIN(a, b) std::min(a, b)
    #define CUDA_MAX(a, b) std::max(a, b)
    #define CONSTANT_REF const
    #include <algorithm>
    #include <cmath>
    #include <ostream>
#endif

namespace aliceVision {

/*
 * @struct Range
 * @brief Small CPU and GPU host / device struct describing a 1d range.
 */
struct Range
{
    unsigned int begin = 0;
    unsigned int end = 0;

    // default CONSTANT_REFructor
    Range() = default;

    /**
     * @brief Range CONSTANT_REFructor
     * @param[in] in_begin the range begin index
     * @param[in] in_end the range end index
     */
    CUDA_HOST_DEVICE Range(unsigned int in_begin, unsigned int in_end)
      : begin(in_begin),
        end(in_end)
    {}

    /**
     * @brief Return true if the given index is contained in the Range.
     * @param[in] i the given index
     * @return true if the given index point is contained in the Range
     */
    CUDA_HOST_DEVICE inline unsigned int size() CONSTANT_REF { return end - begin; }

    CUDA_HOST_DEVICE inline bool isEmpty() CONSTANT_REF { return begin >= end; }

    /**
     * @brief Return true if the given index is contained in the Range.
     * @param[in] i the given index
     * @return true if the given index point is contained in the Range
     */
    CUDA_HOST inline bool contains(unsigned int i) CONSTANT_REF { return ((begin <= i) && (end > i)); }
};

inline Range intersect(CONSTANT_REF Range& a, CONSTANT_REF Range& b) { return Range(CUDA_MAX(a.begin, b.begin), CUDA_MIN(a.end, b.end)); }

/*
 * @struct ROI
 * @brief Small CPU and GPU host / device struct describing a rectangular 2d region of interest.
 */
struct ROI
{
    Range x, y;

    // default CONSTANT_REFructor
    ROI() = default;

    /**
     * @brief ROI CONSTANT_REFructor
     * @param[in] in_beginX the range X begin index
     * @param[in] in_endX the range X end index
     * @param[in] in_beginY the range Y begin index
     * @param[in] in_endY the range Y end index
     */
    CUDA_HOST_DEVICE ROI(unsigned int in_beginX, unsigned int in_endX, unsigned int in_beginY, unsigned int in_endY)
      : x(in_beginX, in_endX),
        y(in_beginY, in_endY)
    {}

    /**
     * @brief ROI CONSTANT_REFructor
     * @param[in] in_rangeX the X index range
     * @param[in] in_rangeY the Y index range
     */
    CUDA_HOST_DEVICE ROI(Range in_rangeX, Range in_rangeY)
      : x(in_rangeX),
        y(in_rangeY)
    {}

    /**
     * @brief Get the ROI width
     * @return the X range size
     */
    CUDA_HOST_DEVICE inline unsigned int width() CONSTANT_REF { return x.size(); }

    /**
     * @brief Get the ROI height
     * @return the Y range size
     */
    CUDA_HOST_DEVICE inline unsigned int height() CONSTANT_REF { return y.size(); }

    CUDA_HOST_DEVICE inline bool isEmpty() CONSTANT_REF { return x.isEmpty() || y.isEmpty(); }

    /**
     * @brief Return true if the given 2d point is contained in the ROI.
     * @param[in] in_x the given 2d point X coordinate
     * @param[in] in_y the given 2d point Y coordinate
     * @return true if the given 2d point is contained in the ROI
     */
    CUDA_HOST inline bool contains(unsigned int in_x, unsigned int in_y) CONSTANT_REF { return (x.contains(in_x) && y.contains(in_y)); }
};

/**
 * @brief check if a given ROI is valid and can be contained in a given image
 * @param[in] roi the given ROI
 * @param[in] width the given image width
 * @param[in] height the given image height
 * @return true if valid
 */
CUDA_HOST inline bool checkImageROI(CONSTANT_REF ROI& roi, int width, int height)
{
    return ((roi.x.end <= (unsigned int)(width)) && (roi.x.begin < roi.x.end) && (roi.y.end <= (unsigned int)(height)) && (roi.y.begin < roi.y.end));
}

/**
 * @brief Downscale the given Range with the given downscale factor
 * @param[in] range the given Range
 * @param[in] downscale the downscale factor to apply
 * @return the downscaled Range
 */
CUDA_HOST inline Range downscaleRange(CONSTANT_REF Range& range, float downscale)
{
    return Range(CUDA_FLOOR(range.begin / downscale), CUDA_CEIL(range.end / downscale));
}

/**
 * @brief Upscale the given Range with the given upscale factor
 * @param[in] range the given Range
 * @param[in] upscale the upscale factor to apply
 * @return the upscaled Range
 */
CUDA_HOST inline Range upscaleRange(CONSTANT_REF Range& range, float upscale)
{
    return Range(CUDA_FLOOR(range.begin * upscale), CUDA_CEIL(range.end * upscale));
}

/**
 * @brief Inflate the given Range with the given factor
 * @param[in] range the given Range
 * @param[in] factor the inflate factor to apply
 * @return the inflated Range
 */
CUDA_HOST inline Range inflateRange(CONSTANT_REF Range& range, float factor)
{
    const float midRange = range.begin + (range.size() * 0.5f);
    const float inflateSize = range.size() * factor * 0.5f;
    return Range(CUDA_FLOOR(CUDA_MAX(midRange - inflateSize, 0.f)), CUDA_CEIL(midRange + inflateSize));
}

/**
 * @brief Downscale the given ROI with the given downscale factor
 * @param[in] roi the given ROI
 * @param[in] downscale the downscale factor to apply
 * @return the downscaled ROI
 */
CUDA_HOST inline ROI downscaleROI(CONSTANT_REF ROI& roi, float downscale) { return ROI(downscaleRange(roi.x, downscale), downscaleRange(roi.y, downscale)); }

/**
 * @brief Upscale the given ROI with the given upscale factor
 * @param[in] roi the given ROI
 * @param[in] upscale the upscale factor to apply
 * @return the upscaled ROI
 */
CUDA_HOST inline ROI upscaleROI(CONSTANT_REF ROI& roi, float upscale) { return ROI(upscaleRange(roi.x, upscale), upscaleRange(roi.y, upscale)); }

/**
 * @brief Inflate the given ROI with the given factor
 * @param[in] roi the given ROI
 * @param[in] factor the inflate factor to apply
 * @return the Inflated ROI
 */
CUDA_HOST inline ROI inflateROI(CONSTANT_REF ROI& roi, float factor) { return ROI(inflateRange(roi.x, factor), inflateRange(roi.y, factor)); }

inline ROI intersect(CONSTANT_REF ROI& a, CONSTANT_REF ROI& b) { return ROI(intersect(a.x, b.x), intersect(a.y, b.y)); }

#if !defined(__NVCC__) && !defined(__METAL__)
inline std::ostream& operator<<(std::ostream& os, CONSTANT_REF Range& range)
{
    os << range.begin << "-" << range.end;
    return os;
}
inline std::ostream& operator<<(std::ostream& os, CONSTANT_REF ROI& roi)
{
    os << "x: " << roi.x << ", y: " << roi.y;
    return os;
}
#endif

}  // namespace aliceVision


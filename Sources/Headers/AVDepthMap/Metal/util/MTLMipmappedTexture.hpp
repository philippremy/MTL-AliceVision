// This file is part of the AliceVision project.
// Copyright (c) 2025 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <AVDepthMap/Metal/util/MetalTypes.hpp>

#if defined(__METAL__)
    #include <metal_stdlib>
    using namespace metal;
    #define THREAD_ADDR_SPACE thread
#else
    #include <Metal/Metal.hpp>
    #include <AVSystem/Logger.hpp>
    #include <AVDepthMap/Metal/host/DeviceManager.hpp>
    #include <cassert>
    #define THREAD_ADDR_SPACE
#endif

enum class FilterMode
{
    NearestNeighbor,
    Bilinear,
};

/**
 *
 * @brief A helper class for managing and accessing mipmapped 2D textures in Metal an C++
 *
 * The class allows for texture like functionality for an arbitrary type. The miplevels are defined from
 * C++, where the instance is created and initialized with the requested amount of mip levels.
 * The mip levels are tightly packed, i.e, no padding is used between the mip levels.
 *
 */
template<class T>
class MTLMipmappedTexture
{
public:

    #if defined(__METAL__)
        MTLMipmappedTexture() = delete;
        explicit MTLMipmappedTexture(MTLDeviceAddrSpace T* rawBuffer, uint64_t level0Width, uint64_t level0Height, uint64_t levels)
        : _buffer(rawBuffer), _level0Width(level0Width), _level0Height(level0Height), _levels(levels)
        {}
    #endif

    uint64_t getWidth(uint64_t forLevel) const
    {
        assert(forLevel < _levels);
        #if defined(__METAL__)
        return max(1ull, _level0Width >> forLevel);
        #else
        return std::max(1ull, _level0Width >> forLevel);
        #endif
    }

    uint64_t getHeight(uint64_t forLevel) const
    {
        assert(forLevel < _levels);
        #if defined(__METAL__)
        return max(1ull, _level0Height >> forLevel);
        #else
        return std::max(1ull, _level0Height >> forLevel);
        #endif
    }

    uint64_t getLevelCount() const
    {
        return _levels;
    }

    MTLDeviceAddrSpace T& tex2DLod(uint64_t x, uint64_t y, uint64_t lod = 0) const
    {
        // Calculate offset
        uint64_t bufferOffsetForLevel = 0;
        for(uint64_t idx=0; idx < lod; idx++)
        {
            bufferOffsetForLevel += (getWidth(idx) * getHeight(idx));
        }
        return _buffer[bufferOffsetForLevel + (y * getWidth(lod) + x)];
    }

    template<FilterMode Filter>
    T tex2DLod(float x, float y, uint64_t lod = 0)
    {
        assert(lod < _levels);

        uint64_t width = getWidth(lod);
        uint64_t height = getHeight(lod);

        if (Filter == FilterMode::NearestNeighbor) {
            uint64_t bufferOffsetForLevel = 0;
            for (uint64_t idx = 0; idx < lod; idx++)
            {
                bufferOffsetForLevel += getWidth(idx) * getHeight(idx);
            }

            uint64_t xi = (uint64_t)round(x);
            uint64_t yi = (uint64_t)round(y);

            // Index into buffer
            return _buffer[bufferOffsetForLevel + yi * width + xi];

        }
        else {
            // Calculate integer coords of the 4 surrounding pixels
            int x0 = (int)floor(x);
            int y0 = (int)floor(y);
            int x1 = x0 + 1 < (int)width ? x0 + 1 : x0;
            int y1 = y0 + 1 < (int)height ? y0 + 1 : y0;

            // Fractional parts
            float fx = x - (float)x0;
            float fy = y - (float)y0;

            // Compute mip level offset
            uint64_t bufferOffsetForLevel = 0;
            for (uint64_t idx = 0; idx < lod; idx++)
                bufferOffsetForLevel += getWidth(idx) * getHeight(idx);

            // Compute pixel values (assumes T supports float ops)
            T c00 = _buffer[bufferOffsetForLevel + y0 * width + x0];
            T c10 = _buffer[bufferOffsetForLevel + y0 * width + x1];
            T c01 = _buffer[bufferOffsetForLevel + y1 * width + x0];
            T c11 = _buffer[bufferOffsetForLevel + y1 * width + x1];

            // Interpolate horizontally
            T c0 = c00 * (1.0f - fx) + c10 * fx;
            T c1 = c01 * (1.0f - fx) + c11 * fx;

            // Interpolate vertically
            T c = c0 * (1.0f - fy) + c1 * fy;

            return c;
        }
    }

    template<FilterMode Filter>
    T sampleTex2D(float u, float v, uint64_t lod = 0) const
    {
        assert(lod < _levels);

        uint64_t width = getWidth(lod);
        uint64_t height = getHeight(lod);

        if (Filter == FilterMode::NearestNeighbor) {
            // Convert normalized coords to pixel indices (nearest neighbor sampling)
            // Clamp u,v to [0,1] to avoid out-of-bounds (you could also add wrap modes)
            float u_clamped = fmin(fmax(u, 0.0f), 1.0f);
            float v_clamped = fmin(fmax(v, 0.0f), 1.0f);

            // Map [0,1] to [0,width-1] and [0,height-1]
            uint64_t x = (uint64_t)round(u_clamped * (float)(width - 1) + 0.5f); // rounding to nearest pixel
            uint64_t y = (uint64_t)round(v_clamped * (float)(height - 1) + 0.5f);

            uint64_t bufferOffsetForLevel = 0;
            for (uint64_t idx = 0; idx < lod; idx++)
            {
                bufferOffsetForLevel += getWidth(idx) * getHeight(idx);
            }

            // Index into buffer
            return _buffer[bufferOffsetForLevel + y * width + x];
        }
        else {
            // Clamp uv to [0, 1]
            float u_clamped = fmin(fmax(u, 0.0f), 1.0f);
            float v_clamped = fmin(fmax(v, 0.0f), 1.0f);

            // Map uv to pixel space [0, width-1], [0, height-1]
            float x = u_clamped * (float)(width - 1);
            float y = v_clamped * (float)(height - 1);

            // Calculate integer coords of the 4 surrounding pixels
            int x0 = (int)floor(x);
            int y0 = (int)floor(y);
            int x1 = x0 + 1 < (int)width ? x0 + 1 : x0;
            int y1 = y0 + 1 < (int)height ? y0 + 1 : y0;

            // Fractional parts
            float fx = x - (float)x0;
            float fy = y - (float)y0;

            // Compute mip level offset
            uint64_t bufferOffsetForLevel = 0;
            for (uint64_t idx = 0; idx < lod; idx++)
                bufferOffsetForLevel += getWidth(idx) * getHeight(idx);

            // Compute pixel values (assumes T supports float ops)
            T c00 = _buffer[bufferOffsetForLevel + y0 * width + x0];
            T c10 = _buffer[bufferOffsetForLevel + y0 * width + x1];
            T c01 = _buffer[bufferOffsetForLevel + y1 * width + x0];
            T c11 = _buffer[bufferOffsetForLevel + y1 * width + x1];

            // Interpolate horizontally
            T c0 = c00 * (1.0f - fx) + c10 * fx;
            T c1 = c01 * (1.0f - fx) + c11 * fx;

            // Interpolate vertically
            T c = c0 * (1.0f - fy) + c1 * fy;

            return c;
        }
    }

    template<FilterMode Filter>
    T tex2DLodNorm(float u, float v, float lod) const
    {
        // Clamp lod to valid mip levels
        float lod_clamped = fmin(fmax(lod, 0.0f), (float)(_levels - 1));

        // We can spare one sample step if the float is actually a valid integer lod level
        float _lodInt;
        #if defined(__METAL__)
        if(modf(lod_clamped, _lodInt) == 0.0f) // No decimals, valid integer number without rounding or clamping
        #else
        if(modf(lod_clamped, &_lodInt) == 0.0f) // No decimals, valid integer number without rounding or clamping
        #endif
        {
            return sampleTex2D<Filter>(u, v, uint64_t(lod_clamped));
        }

        uint64_t lod_floor = floor(lod_clamped);
        uint64_t lod_ceil = (lod_floor + 1 < _levels) ? lod_floor + 1 : lod_floor;
        float lod_frac = lod_clamped - (float)lod_floor;

        T sample0 = sampleTex2D<Filter>(u, v, lod_floor);
        T sample1 = sampleTex2D<Filter>(u, v, lod_ceil);

        // Linear interpolate between mip levels
        T finalSample = sample0 * (1.0f - lod_frac) + sample1 * lod_frac;

        return finalSample;
    }

    void writeLod(THREAD_ADDR_SPACE const T& val, uint64_t x, uint64_t y, uint64_t lod)
    {
        assert(lod < _levels);

        // Calculate offset
        uint64_t bufferOffsetForLevel = 0;
        for(uint64_t idx=0; idx < lod; idx++)
        {
            bufferOffsetForLevel += (getWidth(idx) * getHeight(idx));
        }
        _buffer[bufferOffsetForLevel + (y * getWidth(lod) + x)] = val;
    }

    #if !defined(__METAL__)

    MTLMipmappedTexture() = default;

    // Newly allocated
    explicit MTLMipmappedTexture(uint64_t level0Width, uint64_t level0Height, uint64_t mipLevels, uint64_t deviceID)
    : _level0Width(level0Width), _level0Height(level0Height), _levels(mipLevels)
    {
        // Calculate required buffer size in total
        uint64_t totalSize = 0;
        uint64_t prevWidth = level0Width;
        uint64_t prevHeight = level0Height;
        for (uint64_t idx = 0; idx < mipLevels; idx++)
        {
            totalSize += prevWidth * prevHeight;
            prevWidth /= 2;
            prevHeight /= 2;
        }
        totalSize *= sizeof(T);
        this->_buffer = NS::TransferPtr(aliceVision::depthMap::DeviceManager::getInstance().getDevice(deviceID)->newBuffer(totalSize, MTL::ResourceStorageModePrivate));
        ALICEVISION_LOG_TRACE("MTLMipmappedTexture Allocation: " << this->_buffer->length() << " B. Levels: " << _levels << " Dim: (" << _level0Width << ", " << _level0Height << ")");
    }

    // Interpreting existing data
    explicit MTLMipmappedTexture(MTL::Buffer* initalizedBuffer, uint64_t level0Width, uint64_t level0Height, uint64_t mipLevels)
    : _level0Width(level0Width), _level0Height(level0Height), _levels(mipLevels)
    {
        this->_buffer = NS::RetainPtr(initalizedBuffer);
    }

    ~MTLMipmappedTexture()
    {
        ALICEVISION_LOG_TRACE("Deallocated MTLMipmappedTexture (MTLBuffer) with byte size: " << _buffer->length() << " B");
    }

    MTL::Buffer* getBuffer() const { return _buffer.get(); }

    #endif

private:
    #if defined(__METAL__)
    MTLDeviceAddrSpace T* _buffer = nullptr;
    #else
    NS::SharedPtr<MTL::Buffer> _buffer = NS::SharedPtr<MTL::Buffer>();
    #endif
    uint64_t _level0Width;
    uint64_t _level0Height;
    uint64_t _levels;
};

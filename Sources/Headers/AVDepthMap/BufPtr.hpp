// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <AVDepthMap/Metal/util/MetalAddressSpace.hpp>

namespace aliceVision {
namespace depthMap {

template<typename T, MetalAddressSpace AddressSpace>
class BufPtr
{};

template<typename T>
class BufPtr<T, MetalAddressSpace::Constant>
{
public:
    BufPtr(METAL_QUALIFIER_CONSTANT T* ptr, size_t pitch)
      : _ptr((METAL_QUALIFIER_CONSTANT unsigned char*)ptr),
        _pitch(pitch)
    {}

    METAL_QUALIFIER_CONSTANT inline T* ptr() { return (METAL_QUALIFIER_CONSTANT T*)(_ptr); }
    METAL_QUALIFIER_CONSTANT inline T* row(size_t y) { return (METAL_QUALIFIER_CONSTANT T*)(_ptr + y * _pitch); }
    METAL_QUALIFIER_CONSTANT inline T& at(size_t x, size_t y) { return row(y)[x]; }

    METAL_QUALIFIER_CONSTANT inline const T* ptr() const { return (METAL_QUALIFIER_CONSTANT const T*)(_ptr); }
    METAL_QUALIFIER_CONSTANT inline const T* row(size_t y) const { return (METAL_QUALIFIER_CONSTANT const T*)(_ptr + y * _pitch); }
    METAL_QUALIFIER_CONSTANT inline const T& at(size_t x, size_t y) const { return row(y)[x]; }

private:
    BufPtr();
    BufPtr(METAL_QUALIFIER_CONSTANT const BufPtr&);
    METAL_QUALIFIER_CONSTANT BufPtr& operator*=(METAL_QUALIFIER_CONSTANT const BufPtr&);

    METAL_QUALIFIER_CONSTANT unsigned char* const _ptr;
    const size_t _pitch;
};

template<typename T>
class BufPtr<T, MetalAddressSpace::Device>
{
public:
    BufPtr(METAL_QUALIFIER_DEVICE T* ptr, size_t pitch)
      : _ptr((METAL_QUALIFIER_DEVICE unsigned char*)ptr),
        _pitch(pitch)
    {}

    METAL_QUALIFIER_DEVICE inline T* ptr() { return (METAL_QUALIFIER_DEVICE T*)(_ptr); }
    METAL_QUALIFIER_DEVICE inline T* row(size_t y) { return (METAL_QUALIFIER_DEVICE T*)(_ptr + y * _pitch); }
    METAL_QUALIFIER_DEVICE inline T& at(size_t x, size_t y) { return row(y)[x]; }

    METAL_QUALIFIER_DEVICE inline const T* ptr() const { return (METAL_QUALIFIER_DEVICE const T*)(_ptr); }
    METAL_QUALIFIER_DEVICE inline const T* row(size_t y) const { return (METAL_QUALIFIER_DEVICE const T*)(_ptr + y * _pitch); }
    METAL_QUALIFIER_DEVICE inline const T& at(size_t x, size_t y) const { return row(y)[x]; }

private:
    BufPtr();
    BufPtr(METAL_QUALIFIER_DEVICE const BufPtr&);
    METAL_QUALIFIER_DEVICE BufPtr& operator*=(METAL_QUALIFIER_DEVICE const BufPtr&);

    METAL_QUALIFIER_DEVICE unsigned char* const _ptr;
    const size_t _pitch;
};

// DEVICE
template<typename T>
static inline METAL_QUALIFIER_DEVICE T* get3DBufferAt_h(METAL_QUALIFIER_DEVICE T* ptr, size_t spitch, size_t pitch,
                                        size_t x, size_t y, size_t z,
                                        MetalAddressSpaceTag<MetalAddressSpace::Device>)
{
    return (METAL_QUALIFIER_DEVICE T*)(((METAL_QUALIFIER_DEVICE char*)ptr) + z * spitch + y * pitch) + x;
}

// CONSTANT
template<typename T>
static inline METAL_QUALIFIER_CONSTANT T* get3DBufferAt_h(METAL_QUALIFIER_CONSTANT T* ptr, size_t spitch, size_t pitch,
                                          size_t x, size_t y, size_t z,
                                          MetalAddressSpaceTag<MetalAddressSpace::Constant>)
{
    return (METAL_QUALIFIER_CONSTANT T*)(((METAL_QUALIFIER_CONSTANT char*)ptr) + z * spitch + y * pitch) + x;
}

// DEVICE
template<typename T>
static inline METAL_QUALIFIER_DEVICE const T* get3DBufferAt_h(METAL_QUALIFIER_DEVICE const T* ptr, size_t spitch, size_t pitch,
                                        size_t x, size_t y, size_t z,
                                        MetalAddressSpaceTag<MetalAddressSpace::Device>)
{
    return (METAL_QUALIFIER_DEVICE const T*)(((METAL_QUALIFIER_DEVICE const char*)ptr) + z * spitch + y * pitch) + x;
}

}  // namespace depthMap
}  // namespace aliceVision
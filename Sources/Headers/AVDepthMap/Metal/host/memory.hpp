// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <AVDepthMap/Metal/host/DeviceManager.hpp>
#include <AVSystem/Logger.hpp>

#include <stdio.h>
#include <stdlib.h>
#include <stdexcept>
#include <assert.h>
#include <sstream>
#include <iostream>
#include <vector>
#include <cstring>

#include <Metal/Metal.hpp>

namespace aliceVision {
namespace depthMap {

/*********************************************************************************
 * forward declarations
 *********************************************************************************/

template<class Type, unsigned Dim>
class MTLDeviceMemoryPitched;

/*********************************************************************************
 * CudaSizeBase
 *********************************************************************************/

template<unsigned Dim>
class MTLSizeBase
{
  public:
    MTLSizeBase()
    {
#pragma unroll
        for (int i = Dim; i--;)
            size[i] = 0;
    }
    inline size_t operator[](size_t i) const { return size[i]; }
    inline size_t& operator[](size_t i) { return size[i]; }
    inline MTLSizeBase operator+(const MTLSizeBase<Dim>& s) const
    {
        MTLSizeBase<Dim> r;

#pragma unroll
        for (size_t i = Dim; i--;)
            r[i] = (*this)[i] + s[i];

        return r;
    }
    inline MTLSizeBase operator-(const MTLSizeBase<Dim>& s) const
    {
        MTLSizeBase<Dim> r;

#pragma unroll
        for (size_t i = Dim; i--;)
            r[i] = (*this)[i] - s[i];

        return r;
    }

  protected:
    size_t size[Dim];
};

template<unsigned Dim>
bool operator==(const MTLSizeBase<Dim>& s1, const MTLSizeBase<Dim>& s2)
{
    for (int i = Dim; i--;)
        if (s1[i] != s2[i])
            return false;

    return true;
}

template<unsigned Dim>
bool operator!=(const MTLSizeBase<Dim>& s1, const MTLSizeBase<Dim>& s2)
{
    for (size_t i = Dim; i--;)
        if (s1[i] != s2[i])
            return true;

    return false;
}

/*********************************************************************************
 * CudaSize
 *********************************************************************************/

template<unsigned Dim>
class MTLSize : public MTLSizeBase<Dim>
{
    MTLSize() {}
};

template<>
class MTLSize<1> : public MTLSizeBase<1>
{
  public:
    MTLSize() {}
    explicit MTLSize(size_t s0) { size[0] = s0; }
};

template<>
class MTLSize<2> : public MTLSizeBase<2>
{
  public:
    MTLSize() {}
    MTLSize(size_t s0, size_t s1)
    {
        size[0] = s0;
        size[1] = s1;
    }

    inline size_t x() const { return size[0]; }
    inline size_t y() const { return size[1]; }
};

template<>
class MTLSize<3> : public MTLSizeBase<3>
{
  public:
    MTLSize() {}
    MTLSize(size_t s0, size_t s1, size_t s2)
    {
        size[0] = s0;
        size[1] = s1;
        size[2] = s2;
    }

    inline size_t x() const { return size[0]; }
    inline size_t y() const { return size[1]; }
    inline size_t z() const { return size[2]; }
};

template<unsigned Dim>
MTLSize<Dim> operator/(const MTLSize<Dim>& lhs, const float& rhs)
{
    if (rhs == 0)
        fprintf(stderr, "Division by zero!!\n");
    MTLSize<Dim> out = lhs;
    for (size_t i = 0; i < Dim; ++i)
        out[i] /= rhs;

    return out;
}

template<unsigned Dim>
MTLSize<Dim> operator-(const MTLSize<Dim>& lhs, const MTLSize<Dim>& rhs)
{
    MTLSize<Dim> out = lhs;
    for (size_t i = Dim; i--;)
        out[i] -= rhs[i];
    return out;
}

/*********************************************************************************
 * CudaMemorySizeBase
 *********************************************************************************/

template<class Type, unsigned Dim>
class MTLMemorySizeBase
{
    MTLSize<Dim> _size;
    size_t _pitch;

  public:
    MTLMemorySizeBase() {}

    explicit MTLMemorySizeBase(const MTLSize<Dim>& size)
      : _size(size),
        _pitch(size[0] * sizeof(Type))
    {}

    /* Initialize or change the contained _size value. As a
     * convenience for the many subclasses whose pitch is always
     * size[0] * sizeof(Type), true can be passed as second
     * parameter.
     */
    void setSize(const MTLSize<Dim>& size, bool computePitch)
    {
        _size = size;
        if (computePitch)
        {
            _pitch = size[0] * sizeof(Type);
        }
    }

    /* Return the Size struct.
     * It is best to use this as an opaque type.
     */
    inline const MTLSize<Dim>& getSize() const { return _size; }

    /* Return the byte size of dimension 0 with padding.
     * This function may return useless info until the
     * actual pitch has been initiated by the subclass.
     */
    inline size_t getPitch() const { return _pitch; }

    /* Return the number of bytes that are required by the data
     * contained in the subclass. The pitch may be different.
     * For many subclasses, getBytesUnpadded() == getBytesPadded()
     */
    inline size_t getBytesUnpadded() const
    {
        size_t prod = _size[0] * sizeof(Type);
        for (int i = 1; i < Dim; i++)
            prod *= _size[i];
        return prod;
    }

    /* Return the number of bytes that are required to contain
     * the data of the subclass. This considers the pitch of the
     * first dimension.
     * For many subclasses, getBytesUnpadded() == getBytesPadded()
     */
    inline size_t getBytesPadded() const
    {
        size_t prod = _pitch;
        for (int i = 1; i < Dim; i++)
            prod *= _size[i];
        return prod;
    }

    /* Returns the number of items that have been allocated,
     * ignoring padding.
     */
    inline size_t getUnitsTotal() const
    {
        size_t prod = _size[0];
        for (int i = 1; i < Dim; i++)
            prod *= _size[i];
        return prod;
    }

    /* Returns the number of items of class Type that is contained
     * in the given dimension. For dimensions >= Dim, return 1.
     */
    inline size_t getUnitsInDim(int dim) const { return (dim < Dim ? _size[dim] : 1); }

    /* For dim 0, return the pitch.
     * For all other dimensions, return the number of units in that dimension.
     */
    inline size_t getPaddedBytesInRow() const { return getPitch(); }

    /* For dim 0, return the number of meaning bytes.
     * For all other dimensions, return the number of units in that dimension.
     */
    inline size_t getUnpaddedBytesInRow() const { return _size[0] * sizeof(Type); }

    /* Return the number of bytes that are required for an n-dimensional
     * slice of the subclass, always starting at dimension 0.
     *
     * Note that "dim" itself is included in the computation.
     */
    inline size_t getBytesPaddedUpToDim(int dim) const
    {
        size_t prod = _pitch;
        for (int i = 1; i <= dim; i++)
            prod *= getUnitsInDim(i);
        return prod;
    }

  protected:
    /* Use to set the pitch when it has been returned by an allocation
     * function.
     */
    inline void setPitch(size_t pitch) { _pitch = pitch; }

    /* Allows the child class to pass the pitch to functions such as
     * CudaMallocPitched for initialization to the true value.
     */
    inline size_t& getPitchRef() { return _pitch; }
};

/*********************************************************************************
 * CudaHostMemoryHeap
 *********************************************************************************/

template<class Type, unsigned Dim>
class MTLHostMemoryHeap : public MTLMemorySizeBase<Type, Dim>
{
    Type* buffer = nullptr;

  public:
    MTLHostMemoryHeap()
      : buffer(nullptr)
    {}

    explicit MTLHostMemoryHeap(const MTLSize<Dim>& size)
      : buffer(nullptr)
    {
        allocate(size);
    }

    MTLHostMemoryHeap<Type, Dim>& operator=(const MTLHostMemoryHeap<Type, Dim>& rhs)
    {
        if (buffer != nullptr)
        {
            allocate(rhs.getSize());
        }
        else if (this->getSize() != rhs.getSize())
        {
            deallocate();
            allocate(rhs.getSize());
        }

        memcpy(buffer, rhs.buffer, rhs.getBytesPadded());
        return *this;
    }

    ~MTLHostMemoryHeap() { deallocate(); }

    void initBuffer() { memset(buffer, 0, this->getBytesPadded()); }

    // see below with copy() functions
    void copyFrom(const MTLDeviceMemoryPitched<Type, Dim>& src);

    inline Type* getBuffer() { return buffer; }
    inline const Type* getBuffer() const { return buffer; }
    inline Type& operator()(size_t x) { return buffer[x]; }
    inline const Type& operator()(size_t x) const { return buffer[x]; }
    inline Type& operator()(size_t x, size_t y) { return getRow(y)[x]; }
    inline const Type& operator()(size_t x, size_t y) const { return getRow(y)[x]; }

    inline unsigned char* getBytePtr() { return (unsigned char*)buffer; }
    inline const unsigned char* getBytePtr() const { return (unsigned char*)buffer; }

  private:
    inline Type* getRow(size_t row)
    {
        unsigned char* ptr = getBytePtr();
        ptr += row * this->getPitch();
        return (Type*)ptr;
    }
    inline const Type* getRow(size_t row) const
    {
        const unsigned char* ptr = getBytePtr();
        ptr += row * this->getPitch();
        return (Type*)ptr;
    }

  public:
    void allocate(const MTLSize<Dim>& size)
    {
        this->setSize(size, true);

        buffer = static_cast<Type*>(malloc(this->getBytesUnpadded()));
        if (!buffer)
            ALICEVISION_THROW_ERROR("Failed to allocate host memory in " << __FILE__ << " at " << __LINE__ << ", called from " << __FUNCTION__);
    }

    void deallocate()
    {
        if (buffer == nullptr)
            return;
        free(buffer);
        buffer = nullptr;
    }
};

/*********************************************************************************
 * CudaDeviceMemoryPitched
 *********************************************************************************/

template<class Type, unsigned Dim>
class MTLDeviceMemoryPitched : public MTLMemorySizeBase<Type, Dim>
{
    MTL::Buffer* buffer = nullptr;
    uint64_t deviceID;

  public:
    bool explicitSyncRequired = false;
    bool cpuVisible = false;

    MTLDeviceMemoryPitched()
      : buffer(nullptr)
    {}

    explicit MTLDeviceMemoryPitched(const MTLSize<Dim>& size, uint64_t deviceID, bool allocateCpuVisible) { allocate(size, deviceID, allocateCpuVisible); }

    explicit MTLDeviceMemoryPitched(const MTLHostMemoryHeap<Type, Dim>& rhs, uint64_t deviceID)
    {
        allocate(rhs.getSize(), deviceID, rhs.cpuVisible);
        copyFrom(rhs);
    }

    ~MTLDeviceMemoryPitched() { deallocate(); }

    MTLDeviceMemoryPitched<Type, Dim>& operator=(const MTLDeviceMemoryPitched<Type, Dim>& rhs)
    {
        if (buffer == nullptr)
        {
            allocate(rhs.getSize(), rhs.getDeviceID(), rhs.cpuVisible);
        }
        else if (this->getSize() != rhs.getSize())
        {
            deallocate();
            allocate(rhs.getSize(), rhs.getDeviceID(), rhs.cpuVisible);
        }
        copyFrom(rhs);
        return *this;
    }

    // see below with copy() functions
    void copyFrom(const MTLDeviceMemoryPitched<Type, Dim>& src);
    void copyFrom(const MTLHostMemoryHeap<Type, Dim>& src);
    void copyFrom(const Type* src, size_t sx, size_t sy);

    void copyTo(Type* dst, size_t sx, size_t sy) const;

    MTL::Buffer* getBuffer() const { return buffer; }

    Type& operator()(size_t x) { return buffer[x]; }

    Type& operator()(size_t x, size_t y)
    {
        Type* row = getRow(y);
        return row[x];
    }

    // TODO: Adapt to Metal
    inline unsigned char* getBytePtr() { return (unsigned char*)buffer; }

    // TODO: Adapt to Metal
    inline const unsigned char* getBytePtr() const { return (unsigned char*)buffer; }

    inline Type* getRow(size_t row)
    {
        unsigned char* ptr = getBytePtr();
        ptr += row * this->getPitch();
        return (Type*)ptr;
    }

    void allocate(const MTLSize<Dim>& size, uint64_t deviceID, bool allocateCpuVisible = false)
    {
        this->setSize(size, false);
        this->deviceID = deviceID;

        // Get Device
        MTL::Device* device = DeviceManager::getInstance().getDevice(deviceID);

        // Check if the device needs explicit synchronization
        MTL::ResourceOptions options = MTL::ResourceStorageModePrivate;
        if (allocateCpuVisible)
        {
            // All Apple GPUs which are unified don't need explicit sync
            if (device->supportsFamily(MTL::GPUFamilyApple1) ||
                device->supportsFamily(MTL::GPUFamilyApple2) ||
                device->supportsFamily(MTL::GPUFamilyApple3) ||
                device->supportsFamily(MTL::GPUFamilyApple4) ||
                device->supportsFamily(MTL::GPUFamilyApple5) ||
                device->supportsFamily(MTL::GPUFamilyApple6) ||
                device->supportsFamily(MTL::GPUFamilyApple7) ||
                device->supportsFamily(MTL::GPUFamilyApple8) ||
                device->supportsFamily(static_cast<MTL::GPUFamily>(1009))
                && device->hasUnifiedMemory())
            {
                options = MTL::ResourceStorageModeShared;
                explicitSyncRequired = false;
            }
            else
            {
                options = MTL::ResourceStorageModeManaged;
                explicitSyncRequired = true;
            }
            cpuVisible = true;
        }

        if (Dim == 2)
        {
            MTL::Buffer* _buffer = device->newBuffer(this->getUnpaddedBytesInRow() * (this->getUnitsInDim(1) * sizeof(Type)), options);
            _buffer->retain();
            buffer = _buffer;
            if (!buffer)
                ALICEVISION_THROW_ERROR("Failed to allocate Metal device memory in " << __FILE__ << " at " << __LINE__ << ", called from " << __FUNCTION__);
            // Pitch equals the width of the buffer
            this->setPitch(this->getUnpaddedBytesInRow());
        }
        else if (Dim == 3)
        {
            MTL::Buffer* _buffer = device->newBuffer(this->getUnpaddedBytesInRow() * (this->getUnitsInDim(1) * sizeof(Type)) * (this->getUnitsInDim(2) * sizeof(Type)), options);
            _buffer->retain();
            buffer = _buffer;
            if (!buffer)
                ALICEVISION_THROW_ERROR("Failed to allocate Metal device memory in " << __FILE__ << " at " << __LINE__ << ", called from " << __FUNCTION__);
            this->setPitch(this->getUnpaddedBytesInRow());

            ALICEVISION_LOG_DEBUG("GPU 3D allocation: " << this->getUnitsInDim(0) << "x" << this->getUnitsInDim(1) << "x" << this->getUnitsInDim(2)
                                                        << ", type size=" << sizeof(Type) << ", pitch=" << this->getPitch());
            ALICEVISION_LOG_DEBUG("                 : "
                                  << this->getBytesUnpadded() << ", padded=" << this->getBytesPadded()
                                  << ", wasted=" << this->getBytesPadded() - this->getBytesUnpadded() << ", wasted ratio="
                                  << ((this->getBytesPadded() - this->getBytesUnpadded()) / double(this->getBytesUnpadded())) * 100.0 << "%");
        }
        else
        {
            throw std::runtime_error("MTLDeviceMemoryPitched does not support " + std::to_string(Dim) + " dimensions.");
        }
    }

    void deallocate()
    {
        if (buffer == nullptr)
            return;

        buffer->release();

        buffer = nullptr;
    }

    uint64_t getDeviceID() const { return deviceID; }
};

/*********************************************************************************
 * CudaDeviceMemory
 *********************************************************************************/

template<class Type>
class MTLDeviceMemory : public MTLMemorySizeBase<Type, 1>
{
    MTL::Buffer* buffer = nullptr;
    uint64_t deviceID;

  public:
    bool explicitSyncRequired = false;
    bool cpuVisible = false;

    MTLDeviceMemory()
      : buffer(nullptr)
    {}

    explicit MTLDeviceMemory(const size_t size, uint64_t deviceID, bool allocateCpuVisible) { allocate(size, deviceID, allocateCpuVisible); }

    explicit inline MTLDeviceMemory(const MTLHostMemoryHeap<Type, 1>& rhs, uint64_t deviceID, bool allocateCpuVisible)
    {
        allocate(rhs.getSize(), deviceID, allocateCpuVisible);
        copy(*this, rhs);
    }

    // constructor with synchronous copy
    MTLDeviceMemory(const Type* inbuf, const size_t size, uint64_t deviceID, bool allocateCpuVisible)
    {
        allocate(size, deviceID, allocateCpuVisible);
        copyFrom(inbuf, size);
    }

    ~MTLDeviceMemory() { deallocate(); }

    MTLDeviceMemory<Type>& operator=(const MTLDeviceMemory<Type>& rhs)
    {
        if (buffer == nullptr)
        {
            allocate(rhs.getSize(), rhs.getDeviceID(), rhs.cpuVisible);
        }
        else if (this->getSize() != rhs.getSize())
        {
            deallocate();
            allocate(rhs.getSize(), rhs.getDeviceID(), rhs.cpuVisible);
        }
        copy(*this, rhs);
        return *this;
    }

    MTL::Buffer* getBuffer() { return buffer; }

    // TODO: Adapt to Metal
    unsigned char* getBytePtr() { return (unsigned char*)buffer; }

    // TODO: Adapt to Metal
    const unsigned char* getBytePtr() const { return (unsigned char*)buffer; }

    void allocate(const MTLSize<1>& size, uint64_t deviceID, bool allocateCpuVisible = false)
    {
        this->setSize(size, true);
        this->deviceID = deviceID;

        // Get Device
        MTL::Device* device = DeviceManager::getInstance().getDevice(deviceID);

        // Check if the device needs explicit synchronization
        MTL::ResourceOptions options = MTL::ResourceStorageModePrivate;
        if (allocateCpuVisible)
        {
            // All Apple GPUs which are unified don't need explicit sync
            if (device->supportsFamily(MTL::GPUFamilyApple1) ||
                device->supportsFamily(MTL::GPUFamilyApple2) ||
                device->supportsFamily(MTL::GPUFamilyApple3) ||
                device->supportsFamily(MTL::GPUFamilyApple4) ||
                device->supportsFamily(MTL::GPUFamilyApple5) ||
                device->supportsFamily(MTL::GPUFamilyApple6) ||
                device->supportsFamily(MTL::GPUFamilyApple7) ||
                device->supportsFamily(MTL::GPUFamilyApple8) ||
                device->supportsFamily(static_cast<MTL::GPUFamily>(1009))
                && device->hasUnifiedMemory())
            {
                options = MTL::ResourceStorageModeShared;
                explicitSyncRequired = false;
            }
            else
            {
                options = MTL::ResourceStorageModeManaged;
                explicitSyncRequired = true;
            }
            this->cpuVisible = true;
        }

        MTL::Buffer* _buffer = device->newBuffer(this->getBytesUnpadded(), options);
        _buffer->retain();
        buffer = _buffer;
        if (!buffer)
            ALICEVISION_THROW_ERROR("Failed to allocate Metal device memory in " << __FILE__ << " at " << __LINE__ << ", called from " << __FUNCTION__);
    }
    void allocate(const size_t size, uint64_t deviceID, bool allocateCpuVisible) { allocate(MTLSize<1>(size), deviceID, allocateCpuVisible); }

    void deallocate()
    {
        if (buffer == nullptr)
            return;

        buffer->release();

        buffer = nullptr;
    }

    void copyFrom(const Type* inbuf, const size_t num)
    {
        if (buffer->resourceOptions() == MTL::ResourceStorageModeShared)
            memcpy(buffer->contents(), inbuf, num * sizeof(Type));
        else if (buffer->resourceOptions() == MTL::ResourceStorageModeManaged)
        {
            memcpy(buffer->contents(), inbuf, num * sizeof(Type));
            buffer->didModifyRange(NS::Range(0, num * sizeof(Type)));
        }
        else
        {
            // Create a staging buffer
            MTLDeviceMemory stagingBuffer = MTLDeviceMemory();
            stagingBuffer.allocate(this->getSize(), this->getDeviceID(), true);
            *this = stagingBuffer;
        }
    }
};

/*********************************************************************************
 * copyFrom member functions
 *********************************************************************************/

template<class Type, unsigned Dim>
void MTLDeviceMemoryPitched<Type, Dim>::copyFrom(const MTLDeviceMemoryPitched<Type, Dim>& src)
{
    // Get blit encoder from Device Manager
    MTL::CommandBuffer* cmdBuffer = DeviceManager::getInstance().getCommandQueue(this->getDeviceID())->commandBuffer();
    MTL::BlitCommandEncoder* blitCmdEncoder = cmdBuffer->blitCommandEncoder();
    ALICEVISION_RUNTIME_ASSERT(this->getDeviceID() == src.getDeviceID(), "Buffer copies must be performed within the same device!");
    blitCmdEncoder->copyFromBuffer(src.getBuffer(), 0, this->getBuffer(), 0, src.getBuffer()->length());
    blitCmdEncoder->endEncoding();
    cmdBuffer->commit();
    cmdBuffer->waitUntilCompleted();
    blitCmdEncoder->release();
    cmdBuffer->release();
}

template<class Type, unsigned Dim>
void MTLDeviceMemoryPitched<Type, Dim>::copyFrom(const MTLHostMemoryHeap<Type, Dim>& src)
{
    if (this->cpuVisible && !this->explicitSyncRequired)
        memcpy(this->getBuffer()->contents(), src.getBuffer(), src.getUnitsTotal() * sizeof(Type));
    else if (this->cpuVisible && this->explicitSyncRequired)
    {
        memcpy(this->getBuffer()->contents(), src.getBuffer(), src.getUnitsTotal() * sizeof(Type));
        this->getBuffer()->didModifyRange(NS::Range(0, this->getUnitsTotal() * sizeof(Type)));
    }
    else
    {
        // Copy with staging buffer
        MTLDeviceMemoryPitched stagingBuffer = MTLDeviceMemoryPitched();
        stagingBuffer.allocate(this->getSize(), this->getDeviceID(), true);
        stagingBuffer.copyFrom(src);
        *this = stagingBuffer;
    }
}

template<class Type, unsigned Dim>
void MTLDeviceMemoryPitched<Type, Dim>::copyFrom(const Type* src, size_t sx, size_t sy)
{
    if (Dim == 2)
    {
        if (this->cpuVisible && !this->explicitSyncRequired)
            memcpy(this->getBuffer()->contents(), src, sx * sy * sizeof(Type));
        else if (this->cpuVisible && this->explicitSyncRequired)
        {
            memcpy(this->getBuffer()->contents(), src, sx * sy * sizeof(Type));
            this->getBuffer()->didModifyRange(NS::Range(0, sx * sy * sizeof(Type)));
        }
        else
        {
            // Copy with staging buffer
            MTLDeviceMemoryPitched stagingBuffer = MTLDeviceMemoryPitched();
            stagingBuffer.allocate(this->getSize(), this->getDeviceID(), true);
            stagingBuffer.copyFrom(src, sx, sy);
            *this = stagingBuffer;
        }
    }
}

template<class Type, unsigned Dim>
void MTLHostMemoryHeap<Type, Dim>::copyFrom(const MTLDeviceMemoryPitched<Type, Dim>& src)
{
    if (src.cpuVisible && !src.explicitSyncRequired)
        memcpy(this->getBuffer(), src.getBuffer()->contents(), src.getUnitsTotal() * sizeof(Type));
    else if (src.cpuVisible && src.explicitSyncRequired)
    {
        // Get blit encoder from Device Manager
        MTL::CommandBuffer* cmdBuffer = DeviceManager::getInstance().getCommandQueue(src.getDeviceID())->commandBuffer();
        MTL::BlitCommandEncoder* blitCmdEncoder = cmdBuffer->blitCommandEncoder();
        // Encode a synchronization
        blitCmdEncoder->synchronizeResource(src.getBuffer());
        blitCmdEncoder->endEncoding();
        cmdBuffer->commit();
        cmdBuffer->waitUntilCompleted();
        blitCmdEncoder->release();
        cmdBuffer->release();
        // Copy the data
        memcpy(this->getBuffer(), src.getBuffer()->contents(), src.getUnitsTotal() * sizeof(Type));
    }
    else
    {
        MTLDeviceMemoryPitched<Type, Dim> stagingBuffer = MTLDeviceMemoryPitched<Type, Dim>();
        stagingBuffer.allocate(src.getSize(), src.getDeviceID(), true);
        stagingBuffer = src;
        this->copyFrom(stagingBuffer);
    }
}

template<class Type, unsigned Dim>
void MTLDeviceMemoryPitched<Type, Dim>::copyTo(Type* dst, size_t sx, size_t sy) const
{
    if (Dim == 2)
    {
        if (this->cpuVisible && !this->explicitSyncRequired)
            memcpy(dst, this->getBuffer()->contents(), this->getUnitsTotal() * sizeof(Type));
        else if (this->cpuVisible && this->explicitSyncRequired)
        {
            // Get blit encoder from Device Manager
            MTL::CommandBuffer* cmdBuffer = DeviceManager::getInstance().getCommandQueue(this->getDeviceID())->commandBuffer();
            MTL::BlitCommandEncoder* blitCmdEncoder = cmdBuffer->blitCommandEncoder();
            // Encode a synchronization
            blitCmdEncoder->synchronizeResource(this->getBuffer());
            blitCmdEncoder->endEncoding();
            cmdBuffer->commit();
            cmdBuffer->waitUntilCompleted();
            blitCmdEncoder->release();
            cmdBuffer->release();
            // Copy the data
            memcpy(dst, this->getBuffer()->contents(), this->getUnitsTotal() * sizeof(Type));
        }
        else
        {
            MTLDeviceMemoryPitched<Type, Dim> stagingBuffer = MTLDeviceMemoryPitched<Type, Dim>();
            stagingBuffer.allocate(this->getSize(), this->getDeviceID(), true);
            stagingBuffer = *this;
            stagingBuffer.copyTo(dst, sx, sy);
        }
    }
}

/*********************************************************************************
 * copy functions
 *********************************************************************************/

template<class Type, unsigned Dim>
void copy(MTLHostMemoryHeap<Type, Dim>& dst, const MTLDeviceMemoryPitched<Type, Dim>& src)
{
    dst.copyFrom(src);
}

template<class Type>
void copy(MTLHostMemoryHeap<Type, 1>& _dst, const MTLDeviceMemory<Type>& _src)
{
    if (_src.cpuVisible && !_src.explicitSyncRequired)
        memcpy(_dst.getBuffer(), _src.getBuffer()->contents(), _src.getUnitsTotal() * sizeof(Type));
    else if (_src.cpuVisible && _src.explicitSyncRequired)
    {
        // Get blit encoder from Device Manager
        MTL::CommandBuffer* cmdBuffer = DeviceManager::getInstance().getCommandQueue(_src.getDeviceID())->commandBuffer();
        MTL::BlitCommandEncoder* blitCmdEncoder = cmdBuffer->blitCommandEncoder();
        // Encode a synchronization
        blitCmdEncoder->synchronizeResource(_src.getBuffer());
        blitCmdEncoder->endEncoding();
        cmdBuffer->commit();
        cmdBuffer->waitUntilCompleted();
        blitCmdEncoder->release();
        cmdBuffer->release();
        // Copy the data
        memcpy(_dst.getBuffer(), _src.getBuffer()->contents(), _src.getUnitsTotal() * sizeof(Type));
    }
    else
    {
        // Create staging buffer
        MTLDeviceMemory<Type> stagingBuffer = MTLDeviceMemory<Type>();
        stagingBuffer.allocate(_src.getSize(), _src.getDeviceID(), true);
        stagingBuffer = _src;
        copy(_dst, stagingBuffer);
    }
}

template<class Type, unsigned Dim>
void copy(MTLDeviceMemoryPitched<Type, Dim>& _dst, const MTLHostMemoryHeap<Type, Dim>& _src)
{
    _dst.copyFrom(_src);
}

template<class Type, unsigned Dim>
void copy(MTLDeviceMemoryPitched<Type, Dim>& _dst, const MTLDeviceMemoryPitched<Type, Dim>& _src)
{
    _dst.copyFrom(_src);
}

template<class Type>
void copy(MTLDeviceMemory<Type>& _dst, const MTLHostMemoryHeap<Type, 1>& _src)
{
    if (_dst.cpuVisible && !_dst.explicitSyncRequired)
        memcpy(_dst.getBuffer()->contents(), _src.getBuffer(), _src.getUnitsTotal() * sizeof(Type));
    else if (_dst.cpuVisible && _dst.explicitSyncRequired)
    {
        // Copy the data
        memcpy(_dst.getBuffer()->contents(), _src.getBuffer(), _src.getUnitsTotal() * sizeof(Type));
        _dst.getBuffer()->didModifyRange(NS::Range(0, _src.getUnitsTotal() * sizeof(Type)));
    }
    else
    {
        // Create staging buffer
        MTLDeviceMemory<Type> stagingBuffer = MTLDeviceMemory<Type>();
        stagingBuffer.allocate(_dst.getSize(), _dst.getDeviceID(), true);
        copy(stagingBuffer, _src);
        _dst = stagingBuffer;
    }
}

template<class Type>
void copy(MTLDeviceMemory<Type>& _dst, const Type* buffer, const size_t numelems)
{
    _dst.copyFrom(buffer, numelems);
}

template<class Type, unsigned Dim>
void copy(Type* _dst, size_t sx, size_t sy, const MTLDeviceMemoryPitched<Type, Dim>& _src)
{
    if (Dim == 2)
    {
        _src.copyTo(_dst, sx, sy);
    }
}

template<class Type, unsigned Dim>
void copy(MTLDeviceMemoryPitched<Type, Dim>& dst, const Type* src, size_t sx, size_t sy)
{
    dst.copyFrom(src, sx, sy);
}

template<class Type, unsigned Dim>
void copy(Type* _dst, size_t sx, size_t sy, size_t sz, const MTLDeviceMemoryPitched<Type, Dim>& _src)
{
    if (Dim >= 3)
    {
        if (_src.cpuVisible && !_src.explicitSyncRequired)
        {
            const uint8_t* srcBase = static_cast<const uint8_t*>(_src->contents());
            Type* dstBase = static_cast<Type*>(_dst);
            for (size_t z = 0; z < sz; ++z)
            {
                for (size_t y = 0; y < sy; ++y)
                {
                    const uint8_t* srcRow = srcBase + z * _src.getUnitsInDim(1) * _src.getPitch() + y * _src.getPitch();
                    Type* dstRow = dstBase + z * sy * sx + y * sx;
                    memcpy(dstRow, srcRow, sx * sizeof(Type));
                }
            }
        }
        else if (_src.cpuVisible && _src.explicitSyncRequired)
        {
            // Get blit encoder from Device Manager
            MTL::CommandBuffer* cmdBuffer = DeviceManager::getInstance().getCommandQueue(_src.getDeviceID())->commandBuffer();
            MTL::BlitCommandEncoder* blitCmdEncoder = cmdBuffer->blitCommandEncoder();
            // Encode a synchronization
            blitCmdEncoder->synchronizeResource(_src.getBuffer());
            blitCmdEncoder->endEncoding();
            cmdBuffer->commit();
            cmdBuffer->waitUntilCompleted();
            blitCmdEncoder->release();
            cmdBuffer->release();
            const uint8_t* srcBase = static_cast<const uint8_t*>(_src->contents());
            Type* dstBase = static_cast<Type*>(_dst);
            for (size_t z = 0; z < sz; ++z)
            {
                for (size_t y = 0; y < sy; ++y)
                {
                    const uint8_t* srcRow = srcBase + z * _src.getUnitsInDim(1) * _src.getPitch() + y * _src.getPitch();
                    Type* dstRow = dstBase + z * sy * sx + y * sx;
                    memcpy(dstRow, srcRow, sx * sizeof(Type));
                }
            }
        } else
        {
            // Create a staging buffer
            MTLDeviceMemoryPitched<Type, Dim> stagingBuffer = MTLDeviceMemoryPitched<Type, Dim>();
            stagingBuffer.allocate(_src.getSize(), _src.getDeviceID(), true);
            stagingBuffer = _src;
            copy(_dst, sx, sy, sz, stagingBuffer);
        }
    }
}

template<class Type, unsigned Dim>
void copy(MTLDeviceMemoryPitched<Type, Dim>& _dst, const Type* _src, size_t sx, size_t sy, size_t sz)
{
    if (Dim >= 3)
    {
        if (_dst.cpuVisible && !_dst.explicitSyncRequired)
        {
            uint8_t* dstBase = static_cast<uint8_t*>(_dst.getBuffer()->contents());
            const Type* srcBase = _src;
            const size_t pitch = _dst.getPitch();
            const size_t slicePitch = _dst.getUnitsInDim(1) * pitch;

            for (size_t z = 0; z < sz; ++z)
            {
                for (size_t y = 0; y < sy; ++y)
                {
                    uint8_t* dstRow = dstBase + z * slicePitch + y * pitch;
                    const Type* srcRow = srcBase + z * sy * sx + y * sx;
                    memcpy(dstRow, srcRow, sx * sizeof(Type));
                }
            }
        }
        else if (_dst.cpuVisible && _dst.explicitSyncRequired)
        {
            uint8_t* dstBase = static_cast<uint8_t*>(_dst.getBuffer()->contents());
            const Type* srcBase = _src;
            const size_t pitch = _dst.getPitch();
            const size_t slicePitch = _dst.getUnitsInDim(1) * pitch;

            for (size_t z = 0; z < sz; ++z)
            {
                for (size_t y = 0; y < sy; ++y)
                {
                    uint8_t* dstRow = dstBase + z * slicePitch + y * pitch;
                    const Type* srcRow = srcBase + z * sy * sx + y * sx;
                    memcpy(dstRow, srcRow, sx * sizeof(Type));
                }
            }
            _dst.getBuffer()->didModifyRange(NS::Range(0, _dst.getUnitsTotal() * sizeof(Type)));
        }
        else
        {
            // Create a staging buffer
            MTLDeviceMemoryPitched<Type, Dim> stagingBuffer = MTLDeviceMemoryPitched<Type, Dim>();
            stagingBuffer.allocate(_dst.getSize(), _dst.getDeviceID(), true);
            copy(stagingBuffer, _src, sx, sy, sz);
            _dst = stagingBuffer;
        }
    }
}

//
// /*
//  * @struct CudaTexture
//  * @brief Support class to maintain a buffer texture in gpu memory.
//  *
//  * @tparam Type the buffer type
//  *
//  * @tparam subpixelInterpolation enable subpixel interpolation
//  *   - can have a large performance impact on some graphic cards
//  *   - could be critical for quality during SGM in small resolution
//  *
//  * @tparam normalizedCoords enable normalized coordinates
//  *   - if true  addressed (x,y) in [0, 1]
//  *   - if false addressed (x,y) in [width, height]
//  */
// template<class Type, bool subpixelInterpolation, bool normalizedCoords>
// struct CudaTexture
// {
//     cudaTextureObject_t textureObj = 0;
//
//     CudaTexture(CudaDeviceMemoryPitched<Type, 2>& buffer_dmp)
//     {
//         cudaTextureDesc texDesc;
//         memset(&texDesc, 0, sizeof(cudaTextureDesc));
//         texDesc.normalizedCoords = normalizedCoords;
//         texDesc.addressMode[0] = cudaAddressModeClamp;
//         texDesc.addressMode[1] = cudaAddressModeClamp;
//         texDesc.addressMode[2] = cudaAddressModeClamp;
//         texDesc.readMode = cudaReadModeElementType;
//         texDesc.filterMode = (subpixelInterpolation) ? cudaFilterModeLinear : cudaFilterModePoint;
//
//         cudaResourceDesc resDesc;
//         resDesc.resType = cudaResourceTypePitch2D;
//         resDesc.res.pitch2D.desc = cudaCreateChannelDesc<Type>();
//         resDesc.res.pitch2D.devPtr = buffer_dmp.getBuffer();
//         resDesc.res.pitch2D.width = buffer_dmp.getSize()[0];
//         resDesc.res.pitch2D.height = buffer_dmp.getSize()[1];
//         resDesc.res.pitch2D.pitchInBytes = buffer_dmp.getPitch();
//
//         // create texture object
//         // note: we only have to do this once
//         CHECK_CUDA_RETURN_ERROR(cudaCreateTextureObject(&textureObj, &resDesc, &texDesc, nullptr));
//     }
//
//     ~CudaTexture() { CHECK_CUDA_RETURN_ERROR_NOEXCEPT(cudaDestroyTextureObject(textureObj)); }
// };
//
// struct CudaRGBATexture
// {
//     cudaTextureObject_t textureObj = 0;
//
//     CudaRGBATexture(CudaDeviceMemoryPitched<CudaRGBA, 2>& buffer_dmp)
//     {
//         cudaTextureDesc texDesc;
//         memset(&texDesc, 0, sizeof(cudaTextureDesc));
//         texDesc.normalizedCoords = false;
//         texDesc.addressMode[0] = cudaAddressModeClamp;
//         texDesc.addressMode[1] = cudaAddressModeClamp;
//         texDesc.addressMode[2] = cudaAddressModeClamp;
//
// #if defined(ALICEVISION_DEPTHMAP_TEXTURE_USE_UCHAR) && defined(ALICEVISION_DEPTHMAP_TEXTURE_USE_INTERPOLATION)
//         texDesc.readMode = cudaReadModeNormalizedFloat;  // uchar to float [0:1], see tex2d_float4 function
//
// #else
//         texDesc.readMode = cudaReadModeElementType;
// #endif
//
// #if defined ALICEVISION_DEPTHMAP_TEXTURE_USE_INTERPOLATION
//         texDesc.filterMode = cudaFilterModeLinear;
// #else
//         texDesc.filterMode = cudaFilterModePoint;
// #endif
//
//         cudaResourceDesc resDesc;
//         resDesc.resType = cudaResourceTypePitch2D;
//
// #ifdef ALICEVISION_DEPTHMAP_TEXTURE_USE_HALF
//         resDesc.res.pitch2D.desc = cudaCreateChannelDescHalf4();
// #else
//         resDesc.res.pitch2D.desc = cudaCreateChannelDesc<CudaRGBA>();
// #endif
//
//         resDesc.res.pitch2D.devPtr = buffer_dmp.getBuffer();
//         resDesc.res.pitch2D.width = buffer_dmp.getSize()[0];
//         resDesc.res.pitch2D.height = buffer_dmp.getSize()[1];
//         resDesc.res.pitch2D.pitchInBytes = buffer_dmp.getPitch();
//
//         // create texture object
//         // note: we only have to do this once
//         CHECK_CUDA_RETURN_ERROR(cudaCreateTextureObject(&textureObj, &resDesc, &texDesc, nullptr));
//     }
//
//     ~CudaRGBATexture() { CHECK_CUDA_RETURN_ERROR_NOEXCEPT(cudaDestroyTextureObject(textureObj)); }
// };

}  // namespace depthMap
}  // namespace aliceVision

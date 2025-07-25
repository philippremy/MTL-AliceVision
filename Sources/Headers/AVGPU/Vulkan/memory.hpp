// This file is part of the AliceVision project.
// Copyright (c) 2025 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <vulkan/vulkan.hpp>
#include <vulkan/vulkan_shared.hpp>

#include <AVGPU/Vulkan/device.hpp>

#include "vk_mem_alloc.h"

#include <memory>
#include <optional>

/**
 * Forward declare classes
 */
template<BufferType BufferType, typename Type>
class VulkanMemory;
template<typename Type>
class VulkanBuffer;
template<typename Type>
class VulkanImage;

/**
 * An abstract base class for handing around a Vulkan Memory Allocation
 *
 * This class merely acts as a type erasure utility to abstract away the
 * difference between a VkBuffer and a VkImage.
 */
template<typename Type>
class VulkanMemoryBase
{

/**
 * Make derived class a friend
 */
template<BufferType BufferType, typename TypeInner>
friend class VulkanMemory;
template<typename TypeInner>
friend class VulkanBuffer;
template<typename TypeInner>
friend class VulkanImage;

public:
    /**
     * Creates a new buffer accessible for Vulkan
     *
     * This function creates a VkBuffer for use by Vulkan. The allocation size
     * is determined by sizeof(T) * size. Therefore, it is similar to a
     * std::array.
     *
     * @tparam T The underlying data type bound to this buffer
     * @param deviceID The deviceID specifying which device to create the
     * buffer in
     * @param size The maximum size of the buffer, specified by element count
     * @param shaderWritable Whether this buffer should be writable from
     * shader code (i.e, GLSL code).
     * @param hostVisible Whether the underlying memory allocation should be
     * host visible
     *
     * @return A VulkanMemory<vk::SharedBuffer, T>, initially empty,
     * but readily allocated.
     *
     * @note Be sure whether you need a host visible buffer; this will likely
     * incur a performance hit, if not on an IGPU where system and GPU memory
     * are shared.
     * @note Once created, this allocation has a fixed size. If a bigger buffer
     * is required, one must allocate a new one and copy the new data into it.
     * @note If shaderWritable = false, this buffer will be created as a
     * Uniform Buffer Object. It will be faster, but cannot be written to by
     * shader code.
     *
     * @warning It is safe to create a writable buffer and later use it a
     * Uniform Buffer Object, but not the other way round. If a read-only
     * buffer is used as a Storage Buffer, this will result in undefined
     * behaviour.
     */
    static VulkanBuffer<Type> createBuffer(
        uint32_t deviceID,
        uint32_t size,
        bool shaderWritable = true,
        bool hostVisible = false);

    /**
     * Creates a new image accessible for Vulkan
     *
     * This function creates a VkImage for use by Vulkan. The allocation size
     * might be larger than width, height and depth combined; this is due to
     * stride, padding, etc. required by the actual implementation.
     *
     * @tparam T The underlying data type bound to this image
     * @param deviceID The deviceID specifying which device to create the
     * image in
     * @param width The width of the image
     * @param imageFormat The image format to be used for this image
     * @param mipmapLevels The count of mipmap levels to allocate
     * @param shaderWritable Whether this image should be writable from
     * shader code (i.e, GLSL code).
     * @param hostVisible Whether the underlying memory allocation should be
     * host visible
     *
     * @return A VulkanMemory<vk::SharedBuffer, T>, initially empty,
     * but readily allocated.
     *
     * @note Be sure whether you need a host visible buffer; this will likely
     * incur a performance hit, if not on an IGPU where system and GPU memory
     * are shared.
     * @note Once created, this allocation has a fixed size. If a bigger buffer
     * is required, one must allocate a new one and copy the new data into it.
     * @note If shaderWritable = false, this buffer will be created as a
     * Uniform Buffer Object. It will be faster, but cannot be written to by
     * shader code.
     *
     * @warning It is safe to create a writable buffer and later use it a
     * Uniform Buffer Object, but not the other way round. If a read-only
     * buffer is used as a Storage Buffer, this will result in undefined
     * behaviour.
     * @warning The mipmap levels are allocated, but not initialized. It is
     * the responsibility of the caller to later generate the appropriate
     * mipmap levels.
     */
    static VulkanImage<Type> create1DImage(
        uint32_t deviceID,
        uint32_t width,
        vk::Format imageFormat,
        uint32_t mipmapLevels,
        bool shaderWritable,
        bool hostVisible);

    /**
     * Creates a new image accessible for Vulkan
     *
     * This function creates a VkImage for use by Vulkan. The allocation size
     * might be larger than width, height and depth combined; this is due to
     * stride, padding, etc. required by the actual implementation.
     *
     * @tparam T The underlying data type bound to this image
     * @param deviceID The deviceID specifying which device to create the
     * image in
     * @param width The width of the image
     * @param height The height of the image
     * @param imageFormat The image format to be used for this image
     * @param mipmapLevels The count of mipmap levels to allocate
     * @param shaderWritable Whether this image should be writable from
     * shader code (i.e, GLSL code).
     * @param hostVisible Whether the underlying memory allocation should be
     * host visible
     *
     * @return A VulkanMemory<vk::SharedBuffer, T>, initially empty,
     * but readily allocated.
     *
     * @note Be sure whether you need a host visible buffer; this will likely
     * incur a performance hit, if not on an IGPU where system and GPU memory
     * are shared.
     * @note Once created, this allocation has a fixed size. If a bigger buffer
     * is required, one must allocate a new one and copy the new data into it.
     * @note If shaderWritable = false, this buffer will be created as a
     * Uniform Buffer Object. It will be faster, but cannot be written to by
     * shader code.
     *
     * @warning It is safe to create a writable buffer and later use it a
     * Uniform Buffer Object, but not the other way round. If a read-only
     * buffer is used as a Storage Buffer, this will result in undefined
     * behaviour.
     * @warning The mipmap levels are allocated, but not initialized. It is
     * the responsibility of the caller to later generate the appropriate
     * mipmap levels.
     */
    static VulkanImage<Type> create2DImage(
        uint32_t deviceID,
        uint32_t width,
        uint32_t height,
        vk::Format imageFormat,
        uint32_t mipmapLevels,
        bool shaderWritable,
        bool hostVisible);

    /**
     * Creates a new image accessible for Vulkan
     *
     * This function creates a VkImage for use by Vulkan. The allocation size
     * might be larger than width, height and depth combined; this is due to
     * stride, padding, etc. required by the actual implementation.
     *
     * @tparam T The underlying data type bound to this image
     * @param deviceID The deviceID specifying which device to create the
     * image in
     * @param width The width of the image
     * @param height The height of the image
     * @param depth The depth of the image
     * @param imageFormat The image format to be used for this image
     * @param mipmapLevels The count of mipmap levels to allocate
     * @param shaderWritable Whether this image should be writable from
     * shader code (i.e, GLSL code).
     * @param hostVisible Whether the underlying memory allocation should be
     * host visible
     *
     * @return A VulkanMemory<vk::SharedBuffer, T>, initially empty,
     * but readily allocated.
     *
     * @note Be sure whether you need a host visible buffer; this will likely
     * incur a performance hit, if not on an IGPU where system and GPU memory
     * are shared.
     * @note Once created, this allocation has a fixed size. If a bigger buffer
     * is required, one must allocate a new one and copy the new data into it.
     * @note If shaderWritable = false, this buffer will be created as a
     * Uniform Buffer Object. It will be faster, but cannot be written to by
     * shader code.
     *
     * @warning It is safe to create a writable buffer and later use it a
     * Uniform Buffer Object, but not the other way round. If a read-only
     * buffer is used as a Storage Buffer, this will result in undefined
     * behaviour.
     * @warning The mipmap levels are allocated, but not initialized. It is
     * the responsibility of the caller to later generate the appropriate
     * mipmap levels.
     */
    static VulkanImage<Type> create3DImage(
        uint32_t deviceID,
        uint32_t width,
        uint32_t height,
        uint32_t depth,
        vk::Format imageFormat,
        uint32_t mipmapLevels,
        bool shaderWritable,
        bool hostVisible);

    /**
     * Virtual default destructor
     */
    virtual ~VulkanMemoryBase() = default;

    /**
     * Returns the deviceID of this instance
     *
     * @return A uint32_t specifying the deviceID this buffer is associated
     * with
     *
     */
    [[nodiscard]] virtual uint32_t getDeviceID() const = 0;

    /**
     * Function to determine if the underlying allocation is bound to a VkImage
     *
     * @return A boolean indicating if the underlying allocation is a VkImage
     */
    [[nodiscard]] virtual constexpr bool isVkImage() const = 0;

    /**
     * Function to determine if the underlying allocation is bound to a
     * VkBuffer
     *
     * @return A boolean indicating if the underlying allocation is a VkBuffer
     */
    [[nodiscard]] virtual constexpr bool isVkBuffer() const = 0;

    /**
     * Function to retrieve the underlying shared vk::SharedImage handle
     *
     * @return A std::optional containing the vk::SharedImage of this
     * allocation
     *
     * @note This function returns the vk::SharedImage only if its subclass is
     * a vk::SharedImage specialization.
     */
    [[nodiscard]] virtual std::optional<vk::SharedImage> getVkImage() const = 0;
    /**
     * Function to retrieve the underlying shared vk::SharedBuffer handle
     *
     * @return A std::optional containing the vk::SharedBuffer of this
     * allocation
     *
     * @note This function returns the vk::SharedBuffer only if its subclass is
     * a vk::SharedBuffer specialization.
     */
    [[nodiscard]] virtual std::optional<vk::SharedBuffer> getVkBuffer() const = 0;

    /**
     * Returns the size of the underlying allocation
     *
     * @return An unsigned integer with the size of the underlying allocation
     */
    [[nodiscard]] virtual uint32_t getByteSize() const = 0;

    /**
     * Returns the total (i.e., maximum) count elements of the underlying
     * allocation.
     *
     * @return The count of elements in this allocation
     */
    [[nodiscard]] virtual uint32_t getSize() const = 0;

    /**
     * Returns the size of the dimensions of the underlying image
     *
     * @return A constant reference to a vk::Extent3D holding the dimension
     * sizes of the underlying image
     *
     * @note This function returns the image dimensions only if its subclass is
     * a vk::SharedImage specialization.
     */
    [[nodiscard]] virtual std::optional<vk::Extent3D> getDimensionSizes() const = 0;

    /**
     * Returns the type of image (how many dimensions it has)
     *
     * @return A vk::ImageType holding the dimension sizes of the underlying
     * image
     *
     * @note This function returns the image type only if its subclass is a
     * vk::SharedImage specialization.
     */
    [[nodiscard]] virtual std::optional<vk::ImageType> getDimensions() const = 0;

    /**
     * Returns the current layout of the image
     *
     * @return A vk::ImageLayout specifying the current image layout
     *
     * @note This function returns the image type only if its subclass is a
     * vk::SharedImage specialization.
     */
    [[nodiscard]] virtual std::optional<vk::ImageLayout> getCurrentImageLayout() const = 0;

    /**
     * Returns the format this image was initialized with
     *
     * @return A vk::Format specifying the current image layout
     *
     * @note This function returns the image type only if its subclass is a
     * vk::SharedImage specialization.
     */
    [[nodiscard]] virtual std::optional<vk::Format> getImageFormat() const = 0;

    /**
     * Returns the count of mipmap levels this image has
     *
     * @return An uint32_t holding the count of allocated mipmap levels
     *
     * @note This function returns the image type only if its subclass is a
     * vk::SharedImage specialization.
     */
    [[nodiscard]] virtual std::optional<uint32_t> getMipmapLevelCount() const = 0;

    /**
     * Returns the sampler for this image
     *
     * @return A vk::SharedSampler of the underlying vk::SharedImage
     *
     * @note This function returns the image type only if its subclass is a
     * vk::SharedImage specialization.
     */
    [[nodiscard]] virtual std::optional<vk::SharedSampler> getSampler() const = 0;

    /**
     * Returns the image view for this image for a mipmap level
     *
     * @param mipmapLevel The mipmap level for which the image view is
     * requested
     *
     * @return A vk::SharedImageView of the underlying vk::SharedImage of the
     * mipmap level
     *
     * @note This function returns the image type only if its subclass is a
     * vk::SharedImage specialization.
     */
    [[nodiscard]] virtual std::optional<vk::SharedImageView> getImageViewForMipMapLevel(uint32_t mipmapLevel) const = 0;

    /**
     * Returns the image view for this image for all mipmap levels
     *
     * @return A vk::SharedImageView of the underlying vk::SharedImage
     *
     * @note This function returns the image type only if its subclass is a
     * vk::SharedImage specialization.
     */
    [[nodiscard]] virtual std::optional<vk::SharedImageView> getImageViewForAllMipMapLevels() const = 0;

    /**
     * Copies data from one allocation into another allocation
     *
     * This function abstracts away the need to care about all different
     * situations which could technically occur while copying data in Vulkan.
     * It performs VkImage to VkImage, VkImage to VkBuffer and VkBuffer to
     * VkBuffer copies.
     *
     * @param source The source allocation
     * @param sourceOffset The offset from where the copy should start
     * @param destinationOffset The offset from where the copy should be
     * inserted
     *
     * @throw VulkanException Throws if any underlying Vulkan error occurred
     * @throw std::runtime_error Throws if the allocations don't have the
     * required size to perform the copy operation.
     * @throw std::runtime_error Throws if the destination (this) is a
     * vk::SharedImage and the source is not. For copying from a buffer into an
     * image use copyImageFromBuffer().
     *
     * @note This function will detect any cross-device operations and
     * host-device operations and will create the necessary staging buffers for
     * carrying out the copy operation. If possible, a fast copy command
     * will be used.
     *
     * @warning The caller has to ensure that the allocations are having the
     * required size to actually hold the data to copy.
     */
    virtual void copyFromAllocation(const VulkanMemoryBase& source, uint32_t sourceOffset, uint32_t destinationOffset) const = 0;

    /**
     * Copies data from a VkBuffer to a VkImage
     *
     * This function abstracts away the need to care about all different
     * situations which could technically occur while copying data in Vulkan.
     * It specifically (and only) performs VkBuffer to VkImage copies.
     *
     * @throw VulkanException Throws if any underlying Vulkan error occurred
     * @throw std::runtime_error Throws if the allocations don't have the
     * required size to perform the copy operation.
     * @throw std::runtime_error Throws if the destination (this) is a
     * vk::SharedImage and the source is not a vk::SharedBuffer. For other copy
     * operations see copyFromAllocation().
     *
     * @param source The source allocation
     * @param sourceOffset The offset from where the copy should start
     * @param destinationOffset The offset from where the copy should be
     * inserted
     *
     * @note This function will detect any cross-device operations and
     * host-device operations and will create the necessary staging buffers for
     * carrying out the copy operation. If possible, a fast copy command
     * will be used.
     *
     * @warning The caller has to ensure that the allocations are having the
     * required size to actually hold the data to copy.
     * @warning The caller has to ensure that this instance was created with
     * the correct vk::Format specified; otherwise the copy operation will
     * corrupt the data.
     */
    virtual void copyImageFromBuffer(const VulkanMemoryBase& source, uint32_t sourceOffset, uint32_t destinationOffset) const = 0;

    /**
     * Returns read only data from the underlying allocation
     *
     * @tparam T The data type stored in the underlying allocation
     * @return A shared pointer to the underlying buffer data.
     *
     * @note The underlying memory allocation is managed by the shared_ptr,
     * so freeing the memory by the caller is not allowed.
     */
    [[nodiscard]] std::shared_ptr<Type[]> getData() const;

protected:
    [[nodiscard]] virtual void* INTERN_getData() const = 0;
    [[nodiscard]] virtual uint32_t INTERN_getDeviceID() const = 0;
};

template<BufferType BufferType, typename Type>
class VulkanMemory : public VulkanMemoryBase<Type>
{

/**
 * Friend Class: VulkanMemoryBase and derived classes
 */
template <typename TypeInner>
friend class VulkanMemoryBase;
template<typename TypeInner>
friend class VulkanBuffer;
template<typename TypeInner>
friend class VulkanImage;

public:
    /**
     * Copies data from the host into the underlying buffer
     *
     * This function abstracts away the need to care about all different
     * situations which could technically occur while copying data in Vulkan.
     * It only copies into class specializations of vk::SharedBuffer.
     *
     * @param data The source data
     * @param count The count of elements to be copied
     * @param sourceOffset The offset from where the copy should start
     * @param destinationOffset The offset from where the copy should be
     * inserted
     *
     * @throw VulkanException Throws if any underlying Vulkan error occurred.
     * @throw std::runtime_error Throws if the allocations don't have the
     * required size to perform the copy operation.
     * @throw std::runtime_error Throws if the underlying buffer type is not a
     * vk::SharedBuffer.
     *
     * @note This function will detect any cross-device operations and
     * host-device operations and will create the necessary staging buffers for
     * carrying out the copy operation. If possible, a fast copy command
     * will be used.
     *
     * @warning The caller has to ensure that the allocations are having the
     * required size to actually hold the data to copy.
     * @warning Due to this function accepting any arbitrary data type, this
     * function can only be called on an instance specialization of
     * vk::SharedBuffer.
     */
    virtual void copyFromHostToBuffer(const Type* data, uint32_t count, uint32_t sourceOffset, uint32_t destinationOffset) const = 0;

    /**
     * Copies data from the host into the underlying image
     *
     * This function abstracts away the need to care about all different
     * situations which could technically occur while copying data in Vulkan.
     * It only copies into class specializations of vk::SharedImage.
     *
     * @param data The source data
     * @param count The count of elements to be copied
     * @param sourceOffset The offset from where the copy should start
     * @param destinationOffset The offset from where the copy should be
     * inserted
     *
     * @throw VulkanException Throws if any underlying Vulkan error occurred.
     * @throw std::runtime_error Throws if the allocations don't have the
     * required size to perform the copy operation.
     * @throw std::runtime_error Throws if the underlying buffer type is not a
     * vk::SharedImage.
     *
     * @note This function will create all necessary staging buffers to
     * complete the operation.
     *
     * @warning The caller has to ensure that the allocations are having the
     * required size to actually hold the data to copy.
     * @warning The caller has to ensure that this instance was created with
     * the correct vk::Format specified; otherwise the copy operation will
     * corrupt the data.
     */
    virtual void copyFromHostToImage(const Type* data, uint32_t count, uint32_t sourceOffset, uint32_t destinationOffset) const = 0;

    [[nodiscard]] uint32_t getDeviceID() const override;

    [[nodiscard]] constexpr bool isVkImage() const override;

    [[nodiscard]] constexpr bool isVkBuffer() const override;

    [[nodiscard]] std::optional<vk::SharedImage> getVkImage() const override;

    [[nodiscard]] std::optional<vk::SharedBuffer> getVkBuffer() const override;

    [[nodiscard]] uint32_t getByteSize() const override;

    [[nodiscard]] uint32_t getSize() const override;

    [[nodiscard]] std::optional<vk::ImageLayout> getCurrentImageLayout() const override;

    [[nodiscard]] std::optional<vk::Format> getImageFormat() const override;

    [[nodiscard]] std::optional<vk::Extent3D> getDimensionSizes() const override;

    [[nodiscard]] std::optional<vk::ImageType> getDimensions() const override;

    [[nodiscard]] std::optional<uint32_t> getMipmapLevelCount() const override;

    [[nodiscard]] std::optional<vk::SharedSampler> getSampler() const override;

    [[nodiscard]] std::optional<vk::SharedImageView> getImageViewForMipMapLevel(uint32_t mipmapLevel) const override;

    [[nodiscard]] std::optional<vk::SharedImageView> getImageViewForAllMipMapLevels() const override;

    void copyFromAllocation(const VulkanMemoryBase<Type>& source, uint32_t sourceOffset, uint32_t destinationOffset) const override;

    void copyImageFromBuffer(const VulkanMemoryBase<Type>& source, uint32_t sourceOffset, uint32_t destinationOffset) const override;

protected:
    /**
     * Explicit constructor for derived classes
     *
     * @param deviceID The deviceID associated with this VulkanMemory instance
     */
    explicit VulkanMemory(uint32_t deviceID);

    uint32_t m_deviceID = UINT32_MAX;             // The device ID associated with this Vulkan Memory

    uint32_t m_allocationByteSize = UINT32_MAX;   // The size of the allocation in bytes
    uint32_t m_elementCount = UINT32_MAX;         // The count of elements in this allocation
    bool m_hostVisible = false;                   // A boolean indicating if this allocation can be accessed by the host
    bool m_shaderWritable = false;                // A boolean indicating if this allocation is writable by shader code

    vk::Extent3D m_extent;              // The dimension sizes of the image
    vk::Format m_format;                // The image format used
    vk::ImageType m_type;               // The image type (dimension)
    uint32_t m_mipmapLevels;            // The amount of mipmap levels this image has
    mutable vk::ImageLayout m_layout;   // The current image layout (mutable, can and will change during the lifetime of the image)

    vk::SharedSampler m_sampler;                                       // A sampler sampling all mipmap levels
    std::unordered_map<uint32_t, vk::SharedImageView> m_imageViews;    // The VkImageViews associated with this image, mapped by LOD (i.e., mipmap level)
    vk::SharedImageView m_imageViewAllMipMaps;                         // The VkImageView associated wirh this image, for all mipmap levels

    std::shared_ptr<Type[]> m_data = nullptr;                    // A shared_ptr holding the current host mapping of this allocation
    std::shared_ptr<VmaAllocation_T> m_allocation = nullptr;     // A shared_ptr holding the actual handle to the VmaAllocation (custom deleter)
    BufferType m_bufferObject;                                   // The underlying buffer type, either vk::SharedBuffer or vk::SharedImage
    mutable std::shared_ptr<VulkanBuffer<Type>> m_stagingBuffer; // The current staging buffer for this allocation when getting data to the host (mutable, will be updated every time getData() is called)

private:
    virtual void INTERN_copyFromSameDevice(const VulkanMemoryBase<Type> &source, uint32_t sourceOffset, uint32_t destinationOffset) const = 0;
    virtual void INTERN_copyFromForeignDevice(const VulkanMemoryBase<Type> &source, uint32_t sourceOffset, uint32_t destinationOffset) const = 0;

    [[nodiscard]] uint32_t INTERN_getDeviceID() const override;
    [[nodiscard]] void* INTERN_getData() const override;
};

template<typename Type>
class VulkanBuffer final : public VulkanMemory<vk::SharedBuffer, Type>
{

template <typename TypeInner>
friend class VulkanMemoryBase;
template <BufferType BufferType, typename TypeInner>
friend class VulkanMemory;
template <typename TypeInner>
friend class VulkanImage;

public:
    void copyFromHostToBuffer(const Type* data, uint32_t count, uint32_t sourceOffset, uint32_t destinationOffset) const override;
    void copyFromHostToImage(const Type* data, uint32_t count, uint32_t sourceOffset, uint32_t destinationOffset) const override;

    explicit VulkanBuffer(
        uint32_t deviceID,
        uint32_t size,
        bool shaderWritable,
        bool hostVisible);

private:
    void INTERN_copyFromSameDevice(const VulkanMemoryBase<Type>& source, uint32_t sourceOffset, uint32_t destinationOffset) const override;
    void INTERN_copyFromForeignDevice(const VulkanMemoryBase<Type>& source, uint32_t sourceOffset, uint32_t destinationOffset) const override;

};

template<typename Type>
class VulkanImage final : public VulkanMemory<vk::SharedImage, Type>
{

template <typename TypeInner>
friend class VulkanMemoryBase;
template <BufferType BufferType, typename TypeInner>
friend class VulkanMemory;
template <typename TypeInner>
friend class VulkanBuffer;

// Make command manager a friend class so that it can update the current layout
friend class VulkanCommandManager;

public:
    void copyFromHostToBuffer(const Type* data, uint32_t count, uint32_t sourceOffset, uint32_t destinationOffset) const override;
    void copyFromHostToImage(const Type* data, uint32_t count, uint32_t sourceOffset, uint32_t destinationOffset) const override;

    explicit VulkanImage(
        uint32_t deviceID,
        uint32_t width,
        vk::Format imageFormat,
        uint32_t mipmapLevels,
        bool shaderWritable,
        bool hostVisible);

    explicit VulkanImage(
        uint32_t deviceID,
        uint32_t width,
        uint32_t height,
        vk::Format imageFormat,
        uint32_t mipmapLevels,
        bool shaderWritable,
        bool hostVisible);

    explicit VulkanImage(
        uint32_t deviceID,
        uint32_t width,
        uint32_t height,
        uint32_t depth,
        vk::Format imageFormat,
        uint32_t mipmapLevels,
        bool shaderWritable,
        bool hostVisible);

private:
    void INTERN_copyFromSameDevice(const VulkanMemoryBase<Type>& source, uint32_t sourceOffset, uint32_t destinationOffset) const override;
    void INTERN_copyFromForeignDevice(const VulkanMemoryBase<Type>& source, uint32_t sourceOffset, uint32_t destinationOffset) const override;

    void INTERN_copyImageFromSameDevice(const VulkanMemoryBase<Type>& source, uint32_t sourceOffset, uint32_t destinationOffset) const;
    void INTERN_copyImageFromForeignDevice(const VulkanMemoryBase<Type>& source, uint32_t sourceOffset, uint32_t destinationOffset) const;
};

#include <AVGPU/Vulkan/impl/memory.inl>

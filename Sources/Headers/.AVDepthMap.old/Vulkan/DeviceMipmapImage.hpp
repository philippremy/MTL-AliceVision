// This file is part of the AliceVision project.
// Copyright (c) 2023 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <AVGPU/Vulkan/memory.hpp>
#include <AVDepthMap/Vulkan/Types.hpp>

namespace aliceVision {
namespace depthMap {

/**
 * @class Device mipmap image
 * @brief Support class to maintain an image pyramid in gpu memory.
 */
class DeviceMipmapImage
{
  public:
  /**
   * Constructs a new wrapper class for managing a mipmapped Vulkan image
   *
   * @param in_img_hmh The image on which the mipmap levels should be generated in
   */
  DeviceMipmapImage() = default;

    // destructor
    ~DeviceMipmapImage() = default;

    // this class handles unique data, no copy constructor
    DeviceMipmapImage(DeviceMipmapImage const&) = delete;

    // this class handles unique data, no copy operator
    void operator=(DeviceMipmapImage const&) = delete;

    /**
     * @brief Update the DeviceMipmapImage from an host-sided image buffer.
     * @param[in] minDownscale the first downscale level of the mipmap image (level 0)
     * @param[in] maxDownscale the last downscale level of the mipmap image
     */
    void fill(const VulkanImage<VulkanRGBA>& in_img, int minDownscale, int maxDownscale);

    /**
     * @brief Get the corresponding mipmap image level of the given downscale
     * @note throw if the given downscale is not contained in the mipmap image
     * @return corresponding mipmap image level
     */
    float getLevel(unsigned int downscale) const;

    /**
     * @brief Get the corresponding mipmap image level dimensions (width, height) of the given downscale.
     * @note throw if the given downscale is not contained in the mipmap image
     * @return corresponding mipmap image downscale level dimensions
     */
    vk::Extent2D getDimensions(unsigned int downscale) const;

    /**
     * @brief Get device mipmap image minimum (first) downscale level.
     * @return first level downscale factor (must be power of two)
     */
    inline unsigned int getMinDownscale() const { return _minDownscale; }

    /**
     * @brief Get device mipmap image maximum (last) downscale level.
     * @return last level downscale factor (must be power of two)
     */
    inline unsigned int getMaxDownscale() const { return _maxDownscale; }

    /**
     * @brief Get device mipmap image texture object with normalized coordinates.
     * @note Normalized coordinates: texture coordinates (x,y) in [0, 1]
     * @return CUDA mipmapped array texture object with normalized coordinates
     */
    inline std::pair<vk::SharedImageView, vk::SharedSampler> getTextureObject() const;

    /**
     * @brief Returns the underlying image
     * @return A constant reference to the underlying VulkanImage<VulkanRGBA>
     */
    inline const VulkanImage<VulkanRGBA>& getImage() const { return *_mipmappedImage; }

  private:
    // private members

    std::shared_ptr<VulkanImage<VulkanRGBA>> _mipmappedImage = nullptr; //< The Vulkan Image with mip levels (stores ImageView an Sampler internally)
    unsigned int _minDownscale = 0;                                     //< the min downscale factor (must be power of two), first downscale level
    unsigned int _maxDownscale = 0;                                     //< the max downscale factor (must be power of two), last downscale level
    size_t _width = 0;                                                  //< original image buffer width (no downscale)
    size_t _height = 0;                                                 //< original image buffer height (no downscale)
};

}  // namespace depthMap
}  // namespace aliceVision

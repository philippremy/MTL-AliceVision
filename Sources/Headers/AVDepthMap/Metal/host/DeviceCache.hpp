// This file is part of the AliceVision project.
// Copyright (c) 2022 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <AVDepthMap/Metal/host/deviceCameraParams_host.hpp>
#include <AVDepthMap/Metal/host/DeviceMipmapImage.hpp>
#include <AVDepthMap/Metal/host/LRUCameraCache.hpp>
#include <AVDepthMap/Metal/imageProcessing/deviceGaussianFilter_host.hpp>
#include <AVMVSUtils/ImagesCache.hpp>
#include <AVMVSUtils/MultiViewParams.hpp>

#include <memory>

namespace aliceVision {
namespace depthMap {

/**
 * @class Device cache
 * @brief This singleton allows to access the current gpu cache.
 */
class DeviceCache
{
  public:
    static DeviceCache& getInstance()
    {
        static DeviceCache instance;
        return instance;
    }

    // Singleton, no copy constructor
    DeviceCache(DeviceCache const&) = delete;

    // Singleton, no copy operator
    void operator=(DeviceCache const&) = delete;

    /**
     * @brief Clear the current gpu device cache.
     */
    void clear(uint64_t deviceID);

    /**
     * @brief Build the current device cache.
     * @param[in] maxMipmapImages the maximum number of mipmap images in the current device cache
     * @param[in] maxCameraParams the maximum number of camera parameters in the current device cache
     */
    void build(int maxMipmapImages, int maxCameraParams, uint64_t deviceID);

    /**
     * @brief Add a mipmap image in current gpu device cache.
     * @param[in] camId the camera index in the ImagesCache / MultiViewParams
     * @param[in] minDownscale the min downscale factor
     * @param[in] maxDownscale the max downscale factor
     * @param[in,out] imageCache the image cache to get host-side data
     * @param[in] mp the multi-view parameters
     */
    void addMipmapImage(int camId,
                        int minDownscale,
                        int maxDownscale,
                        mvsUtils::ImagesCache<image::Image<image::RGBAfColor>>& imageCache,
                        const mvsUtils::MultiViewParams& mp,
                        uint64_t deviceID);

    /**
     * @brief Add a camera parameters structure in current gpu device cache.
     * @param[in] camId the camera index in the ImagesCache / MultiViewParams
     * @param[in] downscale the downscale to apply on gpu
     * @param[in,out] imageCache the image cache to get host-side data
     * @param[in] mp the multi-view parameters
     */
    void addCameraParams(int camId, int downscale, const mvsUtils::MultiViewParams& mp, uint64_t deviceID);

    /**
     * @brief Request a mipmap image in current gpu device cache.
     * @param[in] camId the camera index in the ImagesCache / MultiViewParams
     * @param[in] mp the multi-view parameters
     * @return DeviceMipmapImage
     */
    const DeviceMipmapImage& requestMipmapImage(int camId, const mvsUtils::MultiViewParams& mp, uint64_t deviceID);

    /**
     * @brief Request a camera parameters id in current gpu device cache.
     * @param[in] camId the camera index in the ImagesCache / MultiViewParams
     * @param[in] downscale the downscale to apply on gpu
     * @param[in] mp the multi-view parameters
     * @return Device camera parameters id in CUDA constant memory array
     */
    const int requestCameraParamsId(int camId, int downscale, const mvsUtils::MultiViewParams& mp, uint64_t deviceID);

    MTL::Buffer* requestGaussianOffsetBuffer(uint64_t deviceID);
    MTL::Buffer* requestGaussianArrayBuffer(uint64_t deviceID);
    MTL::Buffer* requestDeviceCameraParamsBuffer(uint64_t deviceID);

  private:
    // private members

    /*
     * @struct SingleDeviceCache
     * @brief This class keeps the cache data for a single gpu device.
     */
    struct SingleDeviceCache
    {
        SingleDeviceCache(int maxMipmapImages, int maxCameraParams, uint64_t deviceID);
        ~SingleDeviceCache() = default;

        // caches Least Recently Used
        LRUCameraIdCache mipmapCache;     //< device mipmap image id cached per (camera id)
        LRUCameraCache cameraParamCache;  //< device camera parameters id cached per (camera id, downscale)

        std::vector<std::unique_ptr<DeviceMipmapImage>> mipmaps;  //< cached device mipmap images
        DeviceGaussianFilterManager gaussMng;
        DeviceCameraParamsManager camParamMng;
    };
    std::map<uint64_t, std::unique_ptr<SingleDeviceCache>> _cachePerDevice;  //< Metal device ID, SingleDeviceCachePtr>

    // private methods

    // Singleton, private default constructor
    DeviceCache() = default;

    // Singleton, private default destructor
    ~DeviceCache() = default;

    /**
     * @brief Get the SingleDeviceCache associated to the current cudaDeviceId
     * @return SingleDeviceCache
     */
    SingleDeviceCache& getCurrentDeviceCache(uint64_t deviceID);
};

}  // namespace depthMap
}  // namespace aliceVision

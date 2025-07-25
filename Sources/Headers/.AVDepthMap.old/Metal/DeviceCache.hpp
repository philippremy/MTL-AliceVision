//
// Created by Philipp Remy on 18.07.25.
//

#ifndef DEVICECACHE_HPP
#define DEVICECACHE_HPP

#include <memory>

#include <AVMVSUtils/MultiViewParams.hpp>
#include <AVMVSUtils/ImagesCache.hpp>
#include <AVDepthMap/Metal/LRUCameraCache.hpp>
#include <AVDepthMap/Metal/DeviceMipmapImage.hpp>
#include <AVDepthMap/Metal/DeviceCameraParams.hpp>
#include <AVDepthMap/Metal/DeviceGaussianArray.hpp>
#include <AVDepthMap/Metal/DevicePatchPattern.hpp>

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
    void build(uint64_t deviceID, int maxMipmapImages, int maxCameraParams, std::optional<const CustomPatchPatternParams> patchPatternParams);

    /**
     * @brief Add a mipmap image in current gpu device cache.
     * @param[in] camId the camera index in the ImagesCache / MultiViewParams
     * @param[in] minDownscale the min downscale factor
     * @param[in] maxDownscale the max downscale factor
     * @param[in,out] imageCache the image cache to get host-side data
     * @param[in] mp the multi-view parameters
     */
    void addMipmapImage(uint64_t deviceID,
                        int camId,
                        int minDownscale,
                        int maxDownscale,
                        mvsUtils::ImagesCache<image::Image<image::RGBAfColor>>& imageCache,
                        const mvsUtils::MultiViewParams& mp);

    /**
     * @brief Add a camera parameters structure in current gpu device cache.
     * @param[in] camId the camera index in the ImagesCache / MultiViewParams
     * @param[in] downscale the downscale to apply on gpu
     * @param[in,out] imageCache the image cache to get host-side data
     * @param[in] mp the multi-view parameters
     */
    void addCameraParams(uint64_t deviceID, int camId, int downscale, const mvsUtils::MultiViewParams& mp);

    /**
     * @brief Request a mipmap image in current gpu device cache.
     * @param[in] camId the camera index in the ImagesCache / MultiViewParams
     * @param[in] mp the multi-view parameters
     * @return DeviceMipmapImage
     */
    const DeviceMipmapImage& requestMipmapImage(uint64_t deviceID, int camId, const mvsUtils::MultiViewParams& mp);

    /**
     * @brief Request a camera parameters id in current gpu device cache.
     * @param[in] camId the camera index in the ImagesCache / MultiViewParams
     * @param[in] downscale the downscale to apply on gpu
     * @param[in] mp the multi-view parameters
     * @return Device camera parameters id in CUDA constant memory array
     */
    const int requestCameraParamsId(uint64_t deviceID, int camId, int downscale, const mvsUtils::MultiViewParams& mp);

    const DeviceGaussianArray* getGaussianArrayWrapper(uint64_t deviceID) const;
    const DeviceCameraParamsArray* getDeviceCameraParamsArrayWrapper(uint64_t deviceID) const;
    const DevicePatchPatternArray* getDevicePatchPatternArrayWrapper(uint64_t deviceID) const;

  private:
    // private members

    /*
     * @struct SingleDeviceCache
     * @brief This class keeps the cache data for a single gpu device.
     */
    struct SingleDeviceCache
    {
        SingleDeviceCache(uint64_t deviceID, int maxMipmapImages, int maxCameraParams, std::optional<const CustomPatchPatternParams> patchPatternParams);
        ~SingleDeviceCache() = default;

        // caches Least Recently Used
        LRUCameraIdCache mipmapCache;     //< device mipmap image id cached per (camera id)
        LRUCameraCache cameraParamCache;  //< device camera parameters id cached per (camera id, downscale)

        std::vector<std::unique_ptr<DeviceMipmapImage>> mipmaps;  //< cached device mipmap images
        std::unique_ptr<DeviceGaussianArray> _gaussianArray;
        std::unique_ptr<DeviceCameraParamsArray> _deviceCameraParams;
        std::unique_ptr<DevicePatchPatternArray> _devicePatchPattern;
    };
    std::map<uint64_t, std::unique_ptr<SingleDeviceCache>> _cachePerDevice;  //< deviceID, SingleDeviceCachePtr>

    // private methods

    // Singleton, private default constructor
    DeviceCache() = default;

    // Singleton, private default destructor
    ~DeviceCache() = default;

    /**
     * @brief Get the SingleDeviceCache associated to the current cudaDeviceId
     * @return SingleDeviceCache
     */
    SingleDeviceCache& getCurrentDeviceCache(uint64_t deviceID) const;

    void fillDeviceCameraParameters(uint64_t deviceID, const DeviceCameraParams& cameraParameters_h, int deviceCameraParamsId);
};

}  // namespace depthMap
}  // namespace aliceVision

#endif //DEVICECACHE_HPP

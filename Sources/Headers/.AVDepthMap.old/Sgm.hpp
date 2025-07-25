// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <AV/config.hpp>

#include <AVMVSData/ROI.hpp>
#include <AVMVSUtils/MultiViewParams.hpp>
#include <AVMVSUtils/TileParams.hpp>
#include <AVDepthMap/Tile.hpp>
#include <AVDepthMap/RefineParams.hpp>
#include <AVDepthMap/SgmParams.hpp>
#include <AVDepthMap/SgmDepthList.hpp>

#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_CUDA)
    #include <aliceVision/depthMap/cuda/host/memory.hpp>
    #include <aliceVision/depthMap/cuda/planeSweeping/similarity.hpp>
#elif ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_VULKAN)
    #include <AVGPU/Vulkan/memory.hpp>
    #include <AVDepthMap/Vulkan/Types.hpp>
#elif ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_METAL)
    #include <simd/vector.h>
    using namespace simd;
    #include <AVGPU/Metal/device.hpp>
    #include <AVDepthMap/Metal/Types.hpp>
#else
    #error No backend for DepthMap computations selected!
#endif

#include <vector>
#include <string>

namespace aliceVision {
namespace depthMap {

#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_CUDA)

/**
 * @class Depth map estimation Semi-Global Matching
 * @brief Manages the calculation of the Semi-Global Matching step.
 */
class Sgm
{
  public:
    /**
     * @brief Sgm constructor.
     * @param[in] mp the multi-view parameters
     * @param[in] tileParams tile workflow parameters
     * @param[in] sgmParams the Semi Global Matching parameters
     * @param[in] computeDepthSimMap Enable final depth/sim map computation
     * @param[in] computeNormalMap Enable final normal map computation
     * @param[in] stream the stream for gpu execution
     */
    Sgm(const mvsUtils::MultiViewParams& mp,
        const mvsUtils::TileParams& tileParams,
        const SgmParams& sgmParams,
        bool computeDepthSimMap,
        bool computeNormalMap,
        cudaStream_t stream);

    // no default constructor
    Sgm() = delete;

    // default destructor
    ~Sgm() = default;

    // final depth/thickness map getter
    inline const CudaDeviceMemoryPitched<float2, 2>& getDeviceDepthThicknessMap() const { return _depthThicknessMap_dmp; }

    // final depth/similarity map getter (optional: could be empty)
    inline const CudaDeviceMemoryPitched<float2, 2>& getDeviceDepthSimMap() const { return _depthSimMap_dmp; }

    // final normal map getter (optional: could be empty)
    inline const CudaDeviceMemoryPitched<float3, 2>& getDeviceNormalMap() const { return _normalMap_dmp; }

    /**
     * @brief Get memory consumpyion in device memory.
     * @return device memory consumpyion (in MB)
     */
    double getDeviceMemoryConsumption() const;

    /**
     * @brief Get unpadded memory consumpyion in device memory.
     * @return unpadded device memory consumpyion (in MB)
     */
    double getDeviceMemoryConsumptionUnpadded() const;

    /**
     * @brief Compute for a single R camera the Semi-Global Matching.
     * @param[in] tile The given tile for SGM computation
     * @param[in] tileDepthList the tile SGM depth list
     */
    void sgmRc(const Tile& tile, const SgmDepthList& tileDepthList);

    /**
     * @brief Smooth SGM result thickness map
     * @note Important to be a proper Refine input parameter.
     * @param[in] tile The given tile for SGM computation
     * @param[in] refineParams the Refine parameters
     */
    void smoothThicknessMap(const Tile& tile, const RefineParams& refineParams);

  private:
    // private methods

    /**
     * @brief Compute for each RcTc the best / second best similarity volumes.
     * @param[in] tile The given tile for SGM computation
     * @param[in] tileDepthList the tile SGM depth list
     */
    void computeSimilarityVolumes(const Tile& tile, const SgmDepthList& tileDepthList);

    /**
     * @brief Optimize the given similarity volume.
     * @note  Filter on the 3D volume to weight voxels based on their neighborhood strongness.
     *        So it downweights local minimums that are not supported by their neighborhood.
     * @param[in] tile The given tile for SGM computation
     * @param[in] tileDepthList the tile SGM depth list
     */
    void optimizeSimilarityVolume(const Tile& tile, const SgmDepthList& tileDepthList);

    /**
     * @brief Retrieve the best depths in the given similarity volume.
     * @note  For each pixel, choose the voxel with the minimal similarity value.
     * @param[in] tile The given tile for SGM computation
     * @param[in] tileDepthList the tile SGM depth list
     */
    void retrieveBestDepth(const Tile& tile, const SgmDepthList& tileDepthList);

    /**
     * @brief Export volume alembic files and 9 points csv file.
     * @param[in] tile The given tile for SGM computation
     * @param[in] tileDepthList the tile SGM depth list
     * @param[in] in_volume_dmp the input volume
     * @param[in] name the export filename
     */
    void exportVolumeInformation(const Tile& tile,
                                 const SgmDepthList& tileDepthList,
                                 const CudaDeviceMemoryPitched<TSim, 3>& in_volume_dmp,
                                 const std::string& name) const;

    // private members

    const mvsUtils::MultiViewParams& _mp;     //< Multi-view parameters
    const mvsUtils::TileParams& _tileParams;  //< tile workflow parameters
    const SgmParams& _sgmParams;              //< Semi Global Matching parameters
    const bool _computeDepthSimMap;           //< needs to compute a final depth/sim map
    const bool _computeNormalMap;             //< needs to compute a final normal map

    // private members in device memory

    CudaHostMemoryHeap<float, 2> _depths_hmh;                   //< rc depth data host memory
    CudaDeviceMemoryPitched<float, 2> _depths_dmp;              //< rc depth data device memory
    CudaDeviceMemoryPitched<float2, 2> _depthThicknessMap_dmp;  //< rc result depth thickness map
    CudaDeviceMemoryPitched<float2, 2> _depthSimMap_dmp;        //< rc result depth/sim map
    CudaDeviceMemoryPitched<float3, 2> _normalMap_dmp;          //< rc normal map
    CudaDeviceMemoryPitched<TSim, 3> _volumeBestSim_dmp;        //< rc best similarity volume
    CudaDeviceMemoryPitched<TSim, 3> _volumeSecBestSim_dmp;     //< rc second best similarity volume
    CudaDeviceMemoryPitched<TSimAcc, 2> _volumeSliceAccA_dmp;   //< for optimization: volume accumulation slice A
    CudaDeviceMemoryPitched<TSimAcc, 2> _volumeSliceAccB_dmp;   //< for optimization: volume accumulation slice B
    CudaDeviceMemoryPitched<TSimAcc, 2> _volumeAxisAcc_dmp;     //< for optimization: volume accumulation axis
    cudaStream_t _stream;                                       //< stream for gpu execution
};

#elif ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_VULKAN)

/**
 * @class Depth map estimation Semi-Global Matching
 * @brief Manages the calculation of the Semi-Global Matching step.
 */
class Sgm
{
  public:
    /**
     * @brief Sgm constructor.
     * @param[in] mp the multi-view parameters
     * @param[in] tileParams tile workflow parameters
     * @param[in] sgmParams the Semi Global Matching parameters
     * @param[in] computeDepthSimMap Enable final depth/sim map computation
     * @param[in] computeNormalMap Enable final normal map computation
     * @param[in] deviceID The deviceID for execution
     */
    Sgm(const mvsUtils::MultiViewParams& mp,
        const mvsUtils::TileParams& tileParams,
        const SgmParams& sgmParams,
        bool computeDepthSimMap,
        bool computeNormalMap,
        uint32_t deviceID);

    // no default constructor
    Sgm() = delete;

    // default destructor
    ~Sgm() = default;

    // final depth/thickness map getter
    inline const VulkanMemoryBase<float2>& getDeviceDepthThicknessMap() const { return *_depthThicknessMap_dmp; }

    // final depth/similarity map getter (optional: could be empty)
    inline const VulkanMemoryBase<float2>& getDeviceDepthSimMap() const { return *_depthSimMap_dmp; }

    // final normal map getter (optional: could be empty)
    inline const VulkanMemoryBase<float4>& getDeviceNormalMap() const { return *_normalMap_dmp; }

    /**
     * @brief Get memory consumption in device memory.
     * @return device memory consumption (in MB)
     */
    double getDeviceMemoryConsumption() const;

    /**
     * THIS FUNCTION IS UNAVAILABLE IN VULKAN!
     *
     * @brief Get unpadded memory consumption in device memory.
     * @return unpadded device memory consumption (in MB)
     */
    // double getDeviceMemoryConsumptionUnpadded() const;

    /**
     * @brief Compute for a single R camera the Semi-Global Matching.
     * @param[in] tile The given tile for SGM computation
     * @param[in] tileDepthList the tile SGM depth list
     */
    void sgmRc(const Tile& tile, const SgmDepthList& tileDepthList);

    /**
     * @brief Smooth SGM result thickness map
     * @note Important to be a proper Refine input parameter.
     * @param[in] tile The given tile for SGM computation
     * @param[in] refineParams the Refine parameters
     */
    void smoothThicknessMap(const Tile& tile, const RefineParams& refineParams);

  private:
    // private methods

    /**
     * @brief Compute for each RcTc the best / second best similarity volumes.
     * @param[in] tile The given tile for SGM computation
     * @param[in] tileDepthList the tile SGM depth list
     */
    void computeSimilarityVolumes(const Tile& tile, const SgmDepthList& tileDepthList);

    /**
     * @brief Optimize the given similarity volume.
     * @note  Filter on the 3D volume to weight voxels based on their neighborhood strongness.
     *        So it downweights local minimums that are not supported by their neighborhood.
     * @param[in] tile The given tile for SGM computation
     * @param[in] tileDepthList the tile SGM depth list
     */
    void optimizeSimilarityVolume(const Tile& tile, const SgmDepthList& tileDepthList);

    /**
     * @brief Retrieve the best depths in the given similarity volume.
     * @note  For each pixel, choose the voxel with the minimal similarity value.
     * @param[in] tile The given tile for SGM computation
     * @param[in] tileDepthList the tile SGM depth list
     */
    void retrieveBestDepth(const Tile& tile, const SgmDepthList& tileDepthList);

    /**
     * @brief Export volume alembic files and 9 points csv file.
     * @param[in] tile The given tile for SGM computation
     * @param[in] tileDepthList the tile SGM depth list
     * @param[in] in_volume_dmp the input volume
     * @param[in] name the export filename
     */
    void exportVolumeInformation(const Tile& tile,
                                 const SgmDepthList& tileDepthList,
                                 const VulkanMemoryBase<TSim>& in_volume_dmp,
                                 const std::string& name) const;

    // private members

    const mvsUtils::MultiViewParams& _mp;     //< Multi-view parameters
    const mvsUtils::TileParams& _tileParams;  //< tile workflow parameters
    const SgmParams& _sgmParams;              //< Semi Global Matching parameters
    const bool _computeDepthSimMap;           //< needs to compute a final depth/sim map
    const bool _computeNormalMap;             //< needs to compute a final normal map

    // private members in device memory
    std::shared_ptr<VulkanImage<float>> _depths_dmp;              //< rc depth data
    std::shared_ptr<VulkanImage<float2>> _depthThicknessMap_dmp;  //< rc result depth thickness map
    std::shared_ptr<VulkanImage<float2>> _depthSimMap_dmp;        //< rc result depth/sim map
    // NOTE: THE ALPHA CHANNEL IS UNUSED! VULKAN DOES NOT SUPPORT RGB IMAGES!
    std::shared_ptr<VulkanImage<float4>> _normalMap_dmp;          //< rc normal map
    std::shared_ptr<VulkanImage<TSim>> _volumeBestSim_dmp;        //< rc best similarity volume
    std::shared_ptr<VulkanImage<TSim>> _volumeSecBestSim_dmp;     //< rc second best similarity volume
    std::shared_ptr<VulkanImage<TSimAcc>> _volumeSliceAccA_dmp;   //< for optimization: volume accumulation slice A
    std::shared_ptr<VulkanImage<TSimAcc>> _volumeSliceAccB_dmp;   //< for optimization: volume accumulation slice B
    std::shared_ptr<VulkanImage<TSimAcc>> _volumeAxisAcc_dmp;     //< for optimization: volume accumulation axis
    uint32_t _deviceID;                                           //< deviceID for execution
};

#elif ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_METAL)

using namespace gpu;

/**
 * @class Depth map estimation Semi-Global Matching
 * @brief Manages the calculation of the Semi-Global Matching step.
 */
class Sgm
{
  public:
    /**
     * @brief Sgm constructor.
     * @param[in] mp the multi-view parameters
     * @param[in] tileParams tile workflow parameters
     * @param[in] sgmParams the Semi Global Matching parameters
     * @param[in] computeDepthSimMap Enable final depth/sim map computation
     * @param[in] computeNormalMap Enable final normal map computation
     * @param[in] deviceID the deviceID for gpu execution
     */
    Sgm(const mvsUtils::MultiViewParams& mp,
        const mvsUtils::TileParams& tileParams,
        const SgmParams& sgmParams,
        const bool computeDepthSimMap,
        const bool computeNormalMap,
        uint64_t deviceID);

    // no default constructor
    Sgm() = delete;

    // default destructor
    ~Sgm() = default;

    // final depth/thickness map getter
    inline const MTLSharedResource<MTLTexture, float4>& getDeviceDepthThicknessMap() const { return _depthThicknessMap_dmp; }

    // final depth/similarity map getter (optional: could be empty)
    inline const MTLSharedResource<MTLTexture, float4>& getDeviceDepthSimMap() const { return _depthSimMap_dmp; }

    // final normal map getter (optional: could be empty)
    inline const MTLSharedResource<MTLTexture, float4>& getDeviceNormalMap() const { return _normalMap_dmp; }

    /**
     * @brief Get memory consumpyion in device memory.
     * @return device memory consumpyion (in MB)
     */
    double getDeviceMemoryConsumption() const;

    /**
     * @brief Get unpadded memory consumpyion in device memory.
     * @return unpadded device memory consumpyion (in MB)
     */
    double getDeviceMemoryConsumptionUnpadded() const;

    /**
     * @brief Compute for a single R camera the Semi-Global Matching.
     * @param[in] tile The given tile for SGM computation
     * @param[in] tileDepthList the tile SGM depth list
     */
    void sgmRc(const Tile& tile, const SgmDepthList& tileDepthList);

    /**
     * @brief Smooth SGM result thickness map
     * @note Important to be a proper Refine input parameter.
     * @param[in] tile The given tile for SGM computation
     * @param[in] refineParams the Refine parameters
     */
    void smoothThicknessMap(const Tile& tile, const RefineParams& refineParams);

  private:
    // private methods

    /**
     * @brief Compute for each RcTc the best / second best similarity volumes.
     * @param[in] tile The given tile for SGM computation
     * @param[in] tileDepthList the tile SGM depth list
     */
    void computeSimilarityVolumes(const Tile& tile, const SgmDepthList& tileDepthList);

    /**
     * @brief Optimize the given similarity volume.
     * @note  Filter on the 3D volume to weight voxels based on their neighborhood strongness.
     *        So it downweights local minimums that are not supported by their neighborhood.
     * @param[in] tile The given tile for SGM computation
     * @param[in] tileDepthList the tile SGM depth list
     */
    void optimizeSimilarityVolume(const Tile& tile, const SgmDepthList& tileDepthList);

    /**
     * @brief Retrieve the best depths in the given similarity volume.
     * @note  For each pixel, choose the voxel with the minimal similarity value.
     * @param[in] tile The given tile for SGM computation
     * @param[in] tileDepthList the tile SGM depth list
     */
    void retrieveBestDepth(const Tile& tile, const SgmDepthList& tileDepthList);

    /**
     * @brief Export volume alembic files and 9 points csv file.
     * @param[in] tile The given tile for SGM computation
     * @param[in] tileDepthList the tile SGM depth list
     * @param[in] in_volume_dmp the input volume
     * @param[in] name the export filename
     */
    void exportVolumeInformation(const Tile& tile,
                                 const SgmDepthList& tileDepthList,
                                 const MTLSharedResource<MTLTexture, TSim>& in_volume_dmp,
                                 const std::string& name) const;

    // private members

    const mvsUtils::MultiViewParams& _mp;     //< Multi-view parameters
    const mvsUtils::TileParams& _tileParams;  //< tile workflow parameters
    const SgmParams& _sgmParams;              //< Semi Global Matching parameters
    const bool _computeDepthSimMap;           //< needs to compute a final depth/sim map
    const bool _computeNormalMap;             //< needs to compute a final normal map

    // private members in device memory
    MTLSharedResource<MTLTexture, float> _depths_dmp;              //< rc depth data device memory
    MTLSharedResource<MTLTexture, float4> _depthThicknessMap_dmp;  //< rc result depth thickness map
    MTLSharedResource<MTLTexture, float4> _depthSimMap_dmp;        //< rc result depth/sim map
    MTLSharedResource<MTLTexture, float4> _depthSimMap_dmp_DUMMY;  //< rc result depth/sim map dummy texture
    MTLSharedResource<MTLTexture, float4> _normalMap_dmp;          //< rc normal map
    MTLSharedResource<MTLTexture, TSim> _volumeBestSim_dmp;        //< rc best similarity volume
    MTLSharedResource<MTLTexture, TSim> _volumeSecBestSim_dmp;     //< rc second best similarity volume
    MTLSharedResource<MTLTexture, TSimAcc> _volumeSliceAccA_dmp;   //< for optimization: volume accumulation slice A
    MTLSharedResource<MTLTexture, TSimAcc> _volumeSliceAccB_dmp;   //< for optimization: volume accumulation slice B
    MTLSharedResource<MTLTexture, TSimAcc> _volumeAxisAcc_dmp;     //< for optimization: volume accumulation axis
    uint64_t _deviceID;                                            //< deviceID for gpu execution
};

#else
    #error No backend for DepthMap computations selected!
#endif

}  // namespace depthMap
}  // namespace aliceVision

// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <AV/config.hpp>

#include <AVMVSData/ROI.hpp>
#include <AVMVSUtils//MultiViewParams.hpp>
#include <AVMVSUtils/TileParams.hpp>
#include <AVDepthMap/Tile.hpp>
#include <AVDepthMap/RefineParams.hpp>

#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_CUDA)
    #include <aliceVision/depthMap/cuda/host/memory.hpp>
    #include <aliceVision/depthMap/cuda/planeSweeping/similarity.hpp>
#elif ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_VULKAN)
    #include <AVGPU/Vulkan/memory.hpp>
    #include <AVDepthMap/Vulkan/Types.hpp>
#elif ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_METAL)
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
 * @class Depth map estimation Refine
 * @brief Manages the calculation of the Refine step.
 */
class Refine
{
  public:
    /**
     * @brief Refine constructor.
     * @param[in] mp the multi-view parameters
     * @param[in] tileParams tile workflow parameters
     * @param[in] refineParams the Refine parameters
     * @param[in] stream the stream for gpu execution
     */
    Refine(const mvsUtils::MultiViewParams& mp, const mvsUtils::TileParams& tileParams, const RefineParams& refineParams, cudaStream_t stream);

    // no default constructor
    Refine() = delete;

    // default destructor
    ~Refine() = default;

    // final depth/similarity map getter
    inline const CudaDeviceMemoryPitched<float2, 2>& getDeviceDepthSimMap() const { return _optimizedDepthSimMap_dmp; }

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
     * @brief Refine for a single R camera the Semi-Global Matching depth/sim map.
     * @param[in] tile The given tile for Refine computation
     * @param[in] in_sgmDepthThicknessMap_dmp the SGM result depth/thickness map in device memory
     * @param[in] in_sgmNormalMap_dmp the SGM result normal map in device memory (or empty)
     */
    void refineRc(const Tile& tile,
                  const CudaDeviceMemoryPitched<float2, 2>& in_sgmDepthThicknessMap_dmp,
                  const CudaDeviceMemoryPitched<float3, 2>& in_sgmNormalMap_dmp);

  private:
    // private methods

    /**
     * @brief Refine and fuse the given depth/sim map using volume strategy.
     * @param[in] tile The given tile for Refine computation
     */
    void refineAndFuseDepthSimMap(const Tile& tile);

    /**
     * @brief Optimize the refined depth/sim maps.
     * @param[in] tile The given tile for Refine computation
     */
    void optimizeDepthSimMap(const Tile& tile);

    /**
     * @brief Compute and write the normal map from the input depth/sim map.
     * @param[in] tile The given tile for Refine computation
     * @param[in] in_depthSimMap_dmp the input depth/sim map in device memory
     * @param[in] name the export filename
     */
    void computeAndWriteNormalMap(const Tile& tile, const CudaDeviceMemoryPitched<float2, 2>& in_depthSimMap_dmp, const std::string& name = "");

    /**
     * @brief Export volume cross alembic file and 9 points csv file.
     * @param[in] tile The given tile for Refine computation
     * @param[in] name the export filename
     */
    void exportVolumeInformation(const Tile& tile, const std::string& name) const;

    // private members

    const mvsUtils::MultiViewParams& _mp;     //< Multi-view parameters
    const mvsUtils::TileParams& _tileParams;  //< tile workflow parameters
    const RefineParams& _refineParams;        //< Refine parameters

    // private members in device memory

    CudaDeviceMemoryPitched<float2, 2> _sgmDepthPixSizeMap_dmp;    //< rc upscaled SGM depth/pixSize map
    CudaDeviceMemoryPitched<float2, 2> _refinedDepthSimMap_dmp;    //< rc refined and fused depth/sim map
    CudaDeviceMemoryPitched<float2, 2> _optimizedDepthSimMap_dmp;  //< rc optimized depth/sim map
    CudaDeviceMemoryPitched<float3, 2> _sgmNormalMap_dmp;          //< rc upscaled SGM normal map (for experimentation purposes)
    CudaDeviceMemoryPitched<float3, 2> _normalMap_dmp;             //< rc normal map (for debug / intermediate results purposes)
    CudaDeviceMemoryPitched<TSimRefine, 3> _volumeRefineSim_dmp;   //< rc refine similarity volume
    CudaDeviceMemoryPitched<float, 2> _optTmpDepthMap_dmp;         //< for color optimization: temporary depth map buffer
    CudaDeviceMemoryPitched<float, 2> _optImgVariance_dmp;         //< for color optimization: image variance buffer
    cudaStream_t _stream;                                          //< stream for gpu execution
};

#elif ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_VULKAN)

/**
 * @class Depth map estimation Refine
 * @brief Manages the calculation of the Refine step.
 */
class Refine
{
  public:
    /**
     * @brief Refine constructor.
     * @param[in] mp the multi-view parameters
     * @param[in] tileParams tile workflow parameters
     * @param[in] refineParams the Refine parameters
     * @param[in] deviceID the device ID for gpu execution
     */
    Refine(const mvsUtils::MultiViewParams& mp, const mvsUtils::TileParams& tileParams, const RefineParams& refineParams, uint32_t deviceID);

    // no default constructor
    Refine() = delete;

    // default destructor
    ~Refine() = default;

    // final depth/similarity map getter
    inline const VulkanImage<float2>& getDeviceDepthSimMap() const { return *_optimizedDepthSimMap_dmp; }

    /**
     * @brief Get memory consumpyion in device memory.
     * @return device memory consumpyion (in MB)
     */
    double getDeviceMemoryConsumption() const;

    /**
     * NOT AVAILABLE IN VULKAN
     *
     * @brief Get unpadded memory consumpyion in device memory.
     * @return unpadded device memory consumpyion (in MB)
     */
    // double getDeviceMemoryConsumptionUnpadded() const;

    /**
     * @brief Refine for a single R camera the Semi-Global Matching depth/sim map.
     * @param[in] tile The given tile for Refine computation
     * @param[in] in_sgmDepthThicknessMap_dmp the SGM result depth/thickness map in device memory
     * @param[in] in_sgmNormalMap_dmp the SGM result normal map in device memory (or empty)
     */
    void refineRc(const Tile& tile,
                  const VulkanImage<float2>& in_sgmDepthThicknessMap_dmp,
                  const VulkanImage<float4>& in_sgmNormalMap_dmp);

  private:
    // private methods

    /**
     * @brief Refine and fuse the given depth/sim map using volume strategy.
     * @param[in] tile The given tile for Refine computation
     */
    void refineAndFuseDepthSimMap(const Tile& tile);

    /**
     * @brief Optimize the refined depth/sim maps.
     * @param[in] tile The given tile for Refine computation
     */
    void optimizeDepthSimMap(const Tile& tile);

    /**
     * @brief Compute and write the normal map from the input depth/sim map.
     * @param[in] tile The given tile for Refine computation
     * @param[in] in_depthSimMap_dmp the input depth/sim map in device memory
     * @param[in] name the export filename
     */
    void computeAndWriteNormalMap(const Tile& tile, const VulkanImage<float2>& in_depthSimMap_dmp, const std::string& name = "");

    /**
     * @brief Export volume cross alembic file and 9 points csv file.
     * @param[in] tile The given tile for Refine computation
     * @param[in] name the export filename
     */
    void exportVolumeInformation(const Tile& tile, const std::string& name) const;

    // private members

    const mvsUtils::MultiViewParams& _mp;     //< Multi-view parameters
    const mvsUtils::TileParams& _tileParams;  //< tile workflow parameters
    const RefineParams& _refineParams;        //< Refine parameters

    // private members in device memory

    std::shared_ptr<VulkanImage<float2>> _sgmDepthPixSizeMap_dmp;    //< rc upscaled SGM depth/pixSize map
    std::shared_ptr<VulkanImage<float2>> _refinedDepthSimMap_dmp;    //< rc refined and fused depth/sim map
    std::shared_ptr<VulkanImage<float2>> _optimizedDepthSimMap_dmp;  //< rc optimized depth/sim map
    std::shared_ptr<VulkanImage<float4>> _sgmNormalMap_dmp;          //< rc upscaled SGM normal map (for experimentation purposes)
    std::shared_ptr<VulkanImage<float4>> _normalMap_dmp;             //< rc normal map (for debug / intermediate results purposes)
    std::shared_ptr<VulkanImage<TSimRefine>> _volumeRefineSim_dmp;   //< rc refine similarity volume
    std::shared_ptr<VulkanImage<float>> _optTmpDepthMap_dmp;         //< for color optimization: temporary depth map buffer
    std::shared_ptr<VulkanImage<float>> _optImgVariance_dmp;         //< for color optimization: image variance buffer
    uint32_t _deviceID;                                            //< deviceID for gpu execution
};

#elif ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_METAL)

using namespace gpu;

/**
 * @class Depth map estimation Refine
 * @brief Manages the calculation of the Refine step.
 */
class Refine
{
  public:
    /**
     * @brief Refine constructor.
     * @param[in] mp the multi-view parameters
     * @param[in] tileParams tile workflow parameters
     * @param[in] refineParams the Refine parameters
     * @param[in] stream the stream for gpu execution
     */
    Refine(const mvsUtils::MultiViewParams& mp, const mvsUtils::TileParams& tileParams, const RefineParams& refineParams, uint64_t deviceID);

    // no default constructor
    Refine() = delete;

    // default destructor
    ~Refine() = default;

    // final depth/similarity map getter
    inline const MTLSharedResource<MTLTexture, float4>& getDeviceDepthSimMap() const { return _optimizedDepthSimMap_dmp; }

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
     * @brief Refine for a single R camera the Semi-Global Matching depth/sim map.
     * @param[in] tile The given tile for Refine computation
     * @param[in] in_sgmDepthThicknessMap_dmp the SGM result depth/thickness map in device memory
     * @param[in] in_sgmNormalMap_dmp the SGM result normal map in device memory (or empty)
     */
    void refineRc(const Tile& tile,
                  const MTLSharedResource<MTLTexture, float4>& in_sgmDepthThicknessMap_dmp,
                  const MTLSharedResource<MTLTexture, float4>& in_sgmNormalMap_dmp);

  private:
    // private methods

    /**
     * @brief Refine and fuse the given depth/sim map using volume strategy.
     * @param[in] tile The given tile for Refine computation
     */
    void refineAndFuseDepthSimMap(const Tile& tile);

    /**
     * @brief Optimize the refined depth/sim maps.
     * @param[in] tile The given tile for Refine computation
     */
    void optimizeDepthSimMap(const Tile& tile);

    /**
     * @brief Compute and write the normal map from the input depth/sim map.
     * @param[in] tile The given tile for Refine computation
     * @param[in] in_depthSimMap_dmp the input depth/sim map in device memory
     * @param[in] name the export filename
     */
    void computeAndWriteNormalMap(const Tile& tile, const MTLSharedResource<MTLTexture, float4>& in_depthSimMap_dmp, const std::string& name = "");

    /**
     * @brief Export volume cross alembic file and 9 points csv file.
     * @param[in] tile The given tile for Refine computation
     * @param[in] name the export filename
     */
    void exportVolumeInformation(const Tile& tile, const std::string& name) const;

    // private members

    const mvsUtils::MultiViewParams& _mp;     //< Multi-view parameters
    const mvsUtils::TileParams& _tileParams;  //< tile workflow parameters
    const RefineParams& _refineParams;        //< Refine parameters

    // private members in device memory
    MTLSharedResource<MTLTexture, float4> _sgmDepthPixSizeMap_dmp;    //< rc upscaled SGM depth/pixSize map
    MTLSharedResource<MTLTexture, float4> _refinedDepthSimMap_dmp;    //< rc refined and fused depth/sim map
    MTLSharedResource<MTLTexture, float4> _optimizedDepthSimMap_dmp;  //< rc optimized depth/sim map
    MTLSharedResource<MTLTexture, float4> _sgmNormalMap_dmp;          //< rc upscaled SGM normal map (for experimentation purposes)
    MTLSharedResource<MTLTexture, float4> _sgmNormalMap_dmp_DUMMY;    //< rc upscaled SGM normal map (for experimentation purposes) dummy texture
    MTLSharedResource<MTLTexture, float4> _normalMap_dmp;             //< rc normal map (for debug / intermediate results purposes)
    MTLSharedResource<MTLTexture, TSimRefine> _volumeRefineSim_dmp;   //< rc refine similarity volume
    MTLSharedResource<MTLTexture, float> _optTmpDepthMap_dmp;         //< for color optimization: temporary depth map buffer
    MTLSharedResource<MTLTexture, float> _optImgVariance_dmp;         //< for color optimization: image variance buffer
    uint64_t _deviceID;                                            //< stream for gpu execution
};

#else
    #error No backend for DepthMap computations selected!
#endif

}  // namespace depthMap
}  // namespace aliceVision

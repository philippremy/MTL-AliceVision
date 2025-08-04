// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <AVMVSData/ROI.hpp>
#include <AVMVSUtils/MultiViewParams.hpp>
#include <AVMVSUtils/TileParams.hpp>
#include <AVDepthMap/Tile.hpp>
#include <AVDepthMap/RefineParams.hpp>
#include <AVDepthMap/Metal/host/memory.hpp>
#include <AVDepthMap/Metal/util/MetalTypes.hpp>

#include <vector>
#include <string>

namespace aliceVision {
namespace depthMap {

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
     * @param[in] deviceID the deviceID for gpu execution
     */
    Refine(const mvsUtils::MultiViewParams& mp, const mvsUtils::TileParams& tileParams, const RefineParams& refineParams, uint64_t deviceID);

    // no default constructor
    Refine() = delete;

    // default destructor
    ~Refine() = default;

    // final depth/similarity map getter
    inline const MTLDeviceMemoryPitched<float2, 2>& getDeviceDepthSimMap() const { return _optimizedDepthSimMap_dmp; }

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

    MTLSize<2> getDeviceDepthSimMapSize() const;

    /**
     * @brief Refine for a single R camera the Semi-Global Matching depth/sim map.
     * @param[in] tile The given tile for Refine computation
     * @param[in] in_sgmDepthThicknessMap_dmp the SGM result depth/thickness map in device memory
     * @param[in] in_sgmNormalMap_dmp the SGM result normal map in device memory (or empty)
     */
    void refineRc(const Tile& tile,
                  const MTLDeviceMemoryPitched<float2, 2>& in_sgmDepthThicknessMap_dmp,
                  const MTLDeviceMemoryPitched<float3, 2>& in_sgmNormalMap_dmp);

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
    void computeAndWriteNormalMap(const Tile& tile, const MTLDeviceMemoryPitched<float2, 2>& in_depthSimMap_dmp, const std::string& name = "");

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

    MTLDeviceMemoryPitched<float2, 2> _sgmDepthPixSizeMap_dmp;    //< rc upscaled SGM depth/pixSize map
    MTLDeviceMemoryPitched<float2, 2> _refinedDepthSimMap_dmp;    //< rc refined and fused depth/sim map
    MTLDeviceMemoryPitched<float2, 2> _optimizedDepthSimMap_dmp;  //< rc optimized depth/sim map
    MTLDeviceMemoryPitched<float3, 2> _sgmNormalMap_dmp;          //< rc upscaled SGM normal map (for experimentation purposes)
    MTLDeviceMemoryPitched<float3, 2> _normalMap_dmp;             //< rc normal map (for debug / intermediate results purposes)
    MTLDeviceMemoryPitched<TSimRefine, 3> _volumeRefineSim_dmp;   //< rc refine similarity volume
    MTLDeviceMemoryPitched<float, 2> _optTmpDepthMap_dmp;         //< for color optimization: temporary depth map buffer
    MTLDeviceMemoryPitched<float, 2> _optImgVariance_dmp;         //< for color optimization: image variance buffer
    uint64_t _deviceID;                                            //< stream for gpu execution
};

}  // namespace depthMap
}  // namespace aliceVision

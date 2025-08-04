// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <AVDepthMap/Refine.hpp>

#include <AV/omp.hpp>
#include <AVSystem/Logger.hpp>
#include <AVMVSData/Point2d.hpp>
#include <AVMVSData/Point3d.hpp>
#include <AVMVSUtils/fileIO.hpp>
#include <AVDepthMap/depthMapUtils.hpp>
#include <AVDepthMap/volumeIO.hpp>
#include <AVDepthMap/Metal/host/DeviceCache.hpp>
#include <AVDepthMap/Metal/planeSweeping/deviceDepthSimilarityMap_host.hpp>
#include <AVDepthMap/Metal/planeSweeping/deviceSimilarityVolume_host.hpp>

namespace aliceVision {
namespace depthMap {

Refine::Refine(const mvsUtils::MultiViewParams& mp, const mvsUtils::TileParams& tileParams, const RefineParams& refineParams, uint64_t deviceID)
  : _mp(mp),
    _tileParams(tileParams),
    _refineParams(refineParams),
    _deviceID(deviceID)
{
}

double Refine::getDeviceMemoryConsumption() const
{
    size_t bytes = 0;

    bytes += _sgmDepthPixSizeMap_dmp.getBytesPadded();
    bytes += _refinedDepthSimMap_dmp.getBytesPadded();
    bytes += _optimizedDepthSimMap_dmp.getBytesPadded();
    bytes += _sgmNormalMap_dmp.getBytesPadded();
    bytes += _normalMap_dmp.getBytesPadded();
    bytes += _volumeRefineSim_dmp.getBytesPadded();
    bytes += _optTmpDepthMap_dmp.getBytesPadded();
    bytes += _optImgVariance_dmp.getBytesPadded();

    return (double(bytes) / (1024.0 * 1024.0));
}

double Refine::getDeviceMemoryConsumptionUnpadded() const
{
    size_t bytes = 0;

    bytes += _sgmDepthPixSizeMap_dmp.getBytesUnpadded();
    bytes += _refinedDepthSimMap_dmp.getBytesUnpadded();
    bytes += _optimizedDepthSimMap_dmp.getBytesUnpadded();
    bytes += _sgmNormalMap_dmp.getBytesUnpadded();
    bytes += _normalMap_dmp.getBytesUnpadded();
    bytes += _volumeRefineSim_dmp.getBytesUnpadded();
    bytes += _optTmpDepthMap_dmp.getBytesUnpadded();
    bytes += _optImgVariance_dmp.getBytesUnpadded();

    return (double(bytes) / (1024.0 * 1024.0));
}

MTLSize<2> Refine::getDeviceDepthSimMapSize() const
{
    // get tile maximum dimensions
    const int downscale = _refineParams.scale * _refineParams.stepXY;
    const int maxTileWidth = divideRoundUp(_tileParams.bufferWidth, downscale);
    const int maxTileHeight = divideRoundUp(_tileParams.bufferHeight, downscale);

    // compute depth/sim map maximum dimensions
    const MTLSize<2> depthSimMapDim(maxTileWidth, maxTileHeight);
    return depthSimMapDim;
}

void Refine::refineRc(const Tile& tile,
                      const MTLDeviceMemoryPitched<float2, 2>& in_sgmDepthThicknessMap_dmp,
                      const MTLDeviceMemoryPitched<float3, 2>& in_sgmNormalMap_dmp)
{
    // get tile maximum dimensions
    const int downscale = _refineParams.scale * _refineParams.stepXY;
    const int maxTileWidth = divideRoundUp(_tileParams.bufferWidth, downscale);
    const int maxTileHeight = divideRoundUp(_tileParams.bufferHeight, downscale);

    // compute depth/sim map maximum dimensions
    const MTLSize<2> depthSimMapDim(maxTileWidth, maxTileHeight);

    // allocate depth/sim maps in device memory
    _sgmDepthPixSizeMap_dmp.allocate("rc upscaled SGM depth/pixSize map", depthSimMapDim, _deviceID, false);
    _refinedDepthSimMap_dmp.allocate("rc refined and fused depth/sim map", depthSimMapDim, _deviceID, false);
    _optimizedDepthSimMap_dmp.allocate("rc optimized depth/sim map", depthSimMapDim, _deviceID, false);

    // allocate SGM upscaled normal map in device memory
    if (_refineParams.useSgmNormalMap)
        _sgmNormalMap_dmp.allocate("rc upscaled SGM normal map (for experimentation purposes)", depthSimMapDim, _deviceID, false);

    // allocate normal map in device memory
    if (_refineParams.exportIntermediateNormalMaps)
        _normalMap_dmp.allocate("rc normal map (for debug / intermediate results purposes)", depthSimMapDim, _deviceID, false);

    // compute volume maximum dimensions
    const int nbDepthsToRefine = _refineParams.halfNbDepths * 2 + 1;
    const MTLSize<3> volDim(maxTileWidth, maxTileHeight, nbDepthsToRefine);

    // allocate refine volume in device memory
    _volumeRefineSim_dmp.allocate("rc refine similarity volume", volDim, _deviceID, false);

    // allocate depth/sim map optimization buffers
    if (_refineParams.useColorOptimization)
    {
        _optTmpDepthMap_dmp.allocate("for color optimization: temporary depth map buffer", depthSimMapDim, _deviceID, false);
        _optImgVariance_dmp.allocate("for color optimization: image variance buffer", depthSimMapDim, _deviceID, false);
    }

    const IndexT viewId = _mp.getViewId(tile.rc);

    ALICEVISION_LOG_INFO(tile << "Refine depth/sim map of view id: " << viewId << ", rc: " << tile.rc << " (" << (tile.rc + 1) << " / " << _mp.ncams
                              << ").");

    // compute upscaled SGM depth/pixSize map
    // compute upscaled SGM normal map
    {
        // downscale the region of interest
        const ROI downscaledRoi = downscaleROI(tile.roi, _refineParams.scale * _refineParams.stepXY);

        // get device cache instance
        DeviceCache& deviceCache = DeviceCache::getInstance();

        // get R device camera parameters id from cache
        const int rcDeviceCameraParamsId = deviceCache.requestCameraParamsId(tile.rc, _refineParams.scale, _mp, _deviceID);

        // get R device mipmap image from cache
        const DeviceMipmapImage& rcDeviceMipmapImage = deviceCache.requestMipmapImage(tile.rc, _mp, _deviceID);

        // compute upscaled SGM depth/pixSize map
        // - upscale SGM depth/thickness map
        // - filter masked pixels (alpha)
        // - compute pixSize from SGM thickness
        mtl_computeSgmUpscaledDepthPixSizeMap(
          _sgmDepthPixSizeMap_dmp, in_sgmDepthThicknessMap_dmp, rcDeviceCameraParamsId, rcDeviceMipmapImage, _refineParams, downscaledRoi, _deviceID);

        // export intermediate depth/pixSize map (if requested by user)
        if (_refineParams.exportIntermediateDepthSimMaps)
            writeDepthPixSizeMap(
              tile.rc, _mp, _tileParams, tile.roi, _sgmDepthPixSizeMap_dmp, _refineParams.scale, _refineParams.stepXY, "sgmUpscaled");

        // upscale SGM normal map (if needed)
        if (_refineParams.useSgmNormalMap && in_sgmNormalMap_dmp.getBuffer() != nullptr)
        {
            mtl_normalMapUpscale(_sgmNormalMap_dmp, in_sgmNormalMap_dmp, downscaledRoi, _deviceID);
        }
    }

    // refine and fuse depth/sim map
    if (_refineParams.useRefineFuse)
    {
        // refine and fuse with volume strategy
        refineAndFuseDepthSimMap(tile);
    }
    else
    {
        ALICEVISION_LOG_INFO(tile << "Refine and fuse depth/sim map volume disabled.");
        mtl_depthSimMapCopyDepthOnly(_refinedDepthSimMap_dmp, _sgmDepthPixSizeMap_dmp, 1.0f, _deviceID);
    }

    // export intermediate depth/sim map (if requested by user)
    if (_refineParams.exportIntermediateDepthSimMaps)
        writeDepthSimMap(tile.rc, _mp, _tileParams, tile.roi, _refinedDepthSimMap_dmp, _refineParams.scale, _refineParams.stepXY, "refinedFused");

    // export intermediate normal map (if requested by user)
    if (_refineParams.exportIntermediateNormalMaps)
        computeAndWriteNormalMap(tile, _refinedDepthSimMap_dmp, "refinedFused");

    // optimize depth/sim map
    if (_refineParams.useColorOptimization && _refineParams.optimizationNbIterations > 0)
    {
        optimizeDepthSimMap(tile);
    }
    else
    {
        ALICEVISION_LOG_INFO(tile << "Color optimize depth/sim map disabled.");
        _optimizedDepthSimMap_dmp.copyFrom(_refinedDepthSimMap_dmp);
    }

    // export intermediate normal map (if requested by user)
    if (_refineParams.exportIntermediateNormalMaps)
        computeAndWriteNormalMap(tile, _optimizedDepthSimMap_dmp);

    ALICEVISION_LOG_INFO(tile << "Refine depth/sim map done.");
}

void Refine::refineAndFuseDepthSimMap(const Tile& tile)
{
    ALICEVISION_LOG_INFO(tile << "Refine and fuse depth/sim map volume.");

    // downscale the region of interest
    const ROI downscaledRoi = downscaleROI(tile.roi, _refineParams.scale * _refineParams.stepXY);

    // get the depth range
    const Range depthRange(0, _volumeRefineSim_dmp.getSize().z());

    // initialize the similarity volume at 0
    // each tc filtered and inverted similarity value will be summed in this volume
    MTLHostMemoryHeap<TSimRefine, 3> volumeRefineSim_hmh(_volumeRefineSim_dmp.getSize());
    for (size_t idx=0; idx < _volumeRefineSim_dmp.getUnitsTotal(); idx++)
    {
        volumeRefineSim_hmh.getBuffer()[idx] = static_cast<TSimRefine>(0.f);
    }
    _volumeRefineSim_dmp.copyFrom(volumeRefineSim_hmh);
    volumeRefineSim_hmh.deallocate();

    // get device cache instance
    DeviceCache& deviceCache = DeviceCache::getInstance();

    // get R device camera parameters id from cache
    const int rcDeviceCameraParamsId = deviceCache.requestCameraParamsId(tile.rc, _refineParams.scale, _mp, _deviceID);

    // get R device mipmap image from cache
    const DeviceMipmapImage& rcDeviceMipmapImage = deviceCache.requestMipmapImage(tile.rc, _mp, _deviceID);

    // compute for each RcTc each similarity value for each depth to refine
    // sum the inverted / filtered similarity value, best value is the HIGHEST
    for (std::size_t tci = 0; tci < tile.refineTCams.size(); ++tci)
    {
        const int tc = tile.refineTCams.at(tci);

        // get T device camera parameters id from cache
        const int tcDeviceCameraParamsId = deviceCache.requestCameraParamsId(tc, _refineParams.scale, _mp, _deviceID);

        // get T device mipmap image from cache
        const DeviceMipmapImage& tcDeviceMipmapImage = deviceCache.requestMipmapImage(tc, _mp, _deviceID);

        ALICEVISION_LOG_DEBUG(tile << "Refine similarity volume:" << std::endl
                                   << "\t- rc: " << tile.rc << std::endl
                                   << "\t- tc: " << tc << " (" << (tci + 1) << "/" << tile.refineTCams.size() << ")" << std::endl
                                   << "\t- rc camera parameters id: " << rcDeviceCameraParamsId << std::endl
                                   << "\t- tc camera parameters id: " << tcDeviceCameraParamsId << std::endl
                                   << "\t- tile range x: [" << downscaledRoi.x.begin << " - " << downscaledRoi.x.end << "]" << std::endl
                                   << "\t- tile range y: [" << downscaledRoi.y.begin << " - " << downscaledRoi.y.end << "]" << std::endl);

        mtl_volumeRefineSimilarity(_volumeRefineSim_dmp,
                                    _sgmDepthPixSizeMap_dmp,
                                    (_refineParams.useSgmNormalMap) ? &_sgmNormalMap_dmp : nullptr,
                                    rcDeviceCameraParamsId,
                                    tcDeviceCameraParamsId,
                                    rcDeviceMipmapImage,
                                    tcDeviceMipmapImage,
                                    _refineParams,
                                    depthRange,
                                    downscaledRoi,
                                    _deviceID);
    }

    // export intermediate volume information (if requested by user)
    exportVolumeInformation(tile, "afterRefine");

    // retrieve the best depth/sim in the volume
    // compute sub-pixel sample using a sliding gaussian
    mtl_volumeRefineBestDepth(_refinedDepthSimMap_dmp, _sgmDepthPixSizeMap_dmp, _volumeRefineSim_dmp, _refineParams, downscaledRoi, _deviceID);

    ALICEVISION_LOG_INFO(tile << "Refine and fuse depth/sim map volume done.");
}

void Refine::optimizeDepthSimMap(const Tile& tile)
{
    ALICEVISION_LOG_INFO(tile << "Color optimize depth/sim map.");

    // downscale the region of interest
    const ROI downscaledRoi = downscaleROI(tile.roi, _refineParams.scale * _refineParams.stepXY);

    // get R device camera from cache
    DeviceCache& deviceCache = DeviceCache::getInstance();

    // get R device camera parameters id from cache
    const int rcDeviceCameraParamsId = deviceCache.requestCameraParamsId(tile.rc, _refineParams.scale, _mp, _deviceID);

    // get R device mipmap image from cache
    const DeviceMipmapImage& rcDeviceMipmapImage = deviceCache.requestMipmapImage(tile.rc, _mp, _deviceID);

    mtl_depthSimMapOptimizeGradientDescent(_optimizedDepthSimMap_dmp,  // output depth/sim map optimized
                                            _optImgVariance_dmp,        // image variance buffer pre-allocate
                                            _optTmpDepthMap_dmp,        // temporary depth map buffer pre-allocate
                                            _sgmDepthPixSizeMap_dmp,    // input SGM upscaled depth/pixSize map
                                            _refinedDepthSimMap_dmp,    // input refined and fused depth/sim map
                                            rcDeviceCameraParamsId,
                                            rcDeviceMipmapImage,
                                            _refineParams,
                                            downscaledRoi,
                                            _deviceID);

    ALICEVISION_LOG_INFO(tile << "Color optimize depth/sim map done.");
}

void Refine::computeAndWriteNormalMap(const Tile& tile, const MTLDeviceMemoryPitched<float2, 2>& in_depthSimMap_dmp, const std::string& name)
{
    // downscale the region of interest
    const ROI downscaledRoi = downscaleROI(tile.roi, _refineParams.scale * _refineParams.stepXY);

    // get R device camera parameters id from cache
    DeviceCache& deviceCache = DeviceCache::getInstance();
    const int rcDeviceCameraParamsId = deviceCache.requestCameraParamsId(tile.rc, _refineParams.scale, _mp, _deviceID);

    ALICEVISION_LOG_INFO(tile << "Refine compute normal map of view id: " << _mp.getViewId(tile.rc) << ", rc: " << tile.rc << " (" << (tile.rc + 1)
                              << " / " << _mp.ncams << ").");

    mtl_depthSimMapComputeNormal(_normalMap_dmp, in_depthSimMap_dmp, rcDeviceCameraParamsId, _refineParams.stepXY, downscaledRoi, _deviceID);

    writeNormalMap(tile.rc, _mp, _tileParams, tile.roi, _normalMap_dmp, _refineParams.scale, _refineParams.stepXY, name);
}

void Refine::exportVolumeInformation(const Tile& tile, const std::string& name) const
{
    if (!_refineParams.exportIntermediateCrossVolumes && !_refineParams.exportIntermediateVolume9pCsv)
    {
        // nothing to do
        return;
    }

    // get tile begin indexes (default no tile)
    int tileBeginX = -1;
    int tileBeginY = -1;

    if (tile.nbTiles > 1)
    {
        tileBeginX = tile.roi.x.begin;
        tileBeginY = tile.roi.y.begin;
    }

    // copy device similarity volume to host memory
    MTLHostMemoryHeap<TSimRefine, 3> volumeSim_hmh(_volumeRefineSim_dmp.getSize());
    volumeSim_hmh.copyFrom(_volumeRefineSim_dmp);

    // copy device SGM upscale depth/sim map to host memory
    MTLHostMemoryHeap<float2, 2> depthPixSizeMapSgmUpscale_hmh(_sgmDepthPixSizeMap_dmp.getSize());
    depthPixSizeMapSgmUpscale_hmh.copyFrom(_sgmDepthPixSizeMap_dmp);

    if (_refineParams.exportIntermediateCrossVolumes)
    {
        ALICEVISION_LOG_INFO(tile << "Export similarity volume cross (" << name << ").");

        const std::string volumeCrossPath = getFileNameFromIndex(_mp, tile.rc, mvsUtils::EFileType::volumeCross, "_" + name, tileBeginX, tileBeginY);

        exportSimilarityVolumeCross(volumeSim_hmh, depthPixSizeMapSgmUpscale_hmh, _mp, tile.rc, _refineParams, volumeCrossPath, tile.roi);

        ALICEVISION_LOG_INFO(tile << "Export similarity volume cross (" << name << ") done.");
    }

    if (_refineParams.exportIntermediateTopographicCutVolumes)
    {
        ALICEVISION_LOG_INFO(tile << "Export similarity volume topographic cut (" << name << ").");

        const std::string volumeCutPath =
          getFileNameFromIndex(_mp, tile.rc, mvsUtils::EFileType::volumeTopographicCut, "_" + name, tileBeginX, tileBeginY);

        exportSimilarityVolumeTopographicCut(volumeSim_hmh, depthPixSizeMapSgmUpscale_hmh, _mp, tile.rc, _refineParams, volumeCutPath, tile.roi);

        ALICEVISION_LOG_INFO(tile << "Export similarity volume topographic cut (" << name << ") done.");
    }

    if (_refineParams.exportIntermediateVolume9pCsv)
    {
        ALICEVISION_LOG_INFO(tile << "Export similarity volume 9 points CSV (" << name << ").");

        const std::string stats9Path = getFileNameFromIndex(_mp, tile.rc, mvsUtils::EFileType::stats9p, "_refine", tileBeginX, tileBeginY);

        exportSimilaritySamplesCSV(volumeSim_hmh, name, _refineParams, stats9Path, tile.roi);

        ALICEVISION_LOG_INFO(tile << "Export similarity volume 9 points CSV (" << name << ") done.");
    }
}

}  // namespace depthMap
}  // namespace aliceVision

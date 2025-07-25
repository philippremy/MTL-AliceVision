// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <AVDepthMap/Sgm.hpp>

#include <AVSystem/Logger.hpp>
#include <AVMVSUtils/fileIO.hpp>
#include <AVDepthMap/DepthMapUtils.hpp>
#include <AVDepthMap/VolumeIO.hpp>

#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_CUDA)
    #include <AVDepthMap/cuda/host/utils.hpp>
    #include <AVDepthMap/cuda/host/DeviceCache.hpp>
    #include <AVDepthMap/cuda/planeSweeping/deviceDepthSimilarityMap.hpp>
    #include <AVDepthMap/cuda/planeSweeping/deviceSimilarityVolume.hpp>
#elif ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_VULKAN)
    #include <AVGPU/Vulkan/memory.hpp>
    #include <AVGPU/Vulkan/device.hpp>
    #include <AVDepthMap/Vulkan/DeviceCache.hpp>
    #include <AVDepthMap/Vulkan/DeviceCameraParams.hpp>
    #include <AVDepthMap/Vulkan/DevicePatchPattern.hpp>
    #include <AVDepthMap/Vulkan/DivUp.hpp>
    #include <AVDepthMapVulkanKernels/AVDepthMapVulkanKernels.hpp>
    #include <AVDepthMapVulkanKernels/AVDepthMapVulkanKernels.hpp>
#elif ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_METAL)
    #include <AVDepthMap/Metal/DeviceCache.hpp>
    #include <AVDepthMap/Metal/DepthSimMapComputeNormal_PushConstants.hpp>
    #include <AVDepthMap/Metal/DepthThicknessSmoothThickness_PushConstants.hpp>
    #include <AVDepthMap/Metal/VolumeComputeSimiliarity_PushConstants.hpp>
    #include <AVDepthMap/Metal/AggregateCostVolumeAtXInSlices_PushConstants.hpp>
    #include <AVDepthMap/Metal/ComputeBestZInSlice_PushConstants.hpp>
    #include <AVDepthMap/Metal/GetVolumeXZSlice_PushConstants.hpp>
    #include <AVDepthMap/Metal/InitVolumeYSlice_PushConstants.hpp>
    #include <AVDepthMap/Metal/RetrieveBestDepth_PushConstants.hpp>
    #include <AVGPU/Metal/command.hpp>
#else
    #error No backend for DepthMap computations selected!
#endif

#include <sstream>

namespace aliceVision {
namespace depthMap {

#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_CUDA)

Sgm::Sgm(const mvsUtils::MultiViewParams& mp,
         const mvsUtils::TileParams& tileParams,
         const SgmParams& sgmParams,
         bool computeDepthSimMap,
         bool computeNormalMap,
         cudaStream_t stream)
  : _mp(mp),
    _tileParams(tileParams),
    _sgmParams(sgmParams),
    _computeDepthSimMap(computeDepthSimMap || sgmParams.exportIntermediateDepthSimMaps),
    _computeNormalMap(computeNormalMap || sgmParams.exportIntermediateNormalMaps),
    _stream(stream)
{
    // get tile maximum dimensions
    const int downscale = _sgmParams.scale * _sgmParams.stepXY;
    const int maxTileWidth = divideRoundUp(tileParams.bufferWidth, downscale);
    const int maxTileHeight = divideRoundUp(tileParams.bufferHeight, downscale);

    // compute map maximum dimensions
    const CudaSize<2> mapDim(maxTileWidth, maxTileHeight);

    // allocate depth list in device memory
    {
        const CudaSize<2> depthsDim(_sgmParams.maxDepths, 1);

        _depths_hmh.allocate(depthsDim);
        _depths_dmp.allocate(depthsDim);
    }

    // allocate depth thickness map in device memory
    _depthThicknessMap_dmp.allocate(mapDim);

    // allocate depth/sim map in device memory
    if (_computeDepthSimMap)
        _depthSimMap_dmp.allocate(mapDim);

    // allocate normal map in device memory
    if (_computeNormalMap)
        _normalMap_dmp.allocate(mapDim);

    // allocate similarity volumes in device memory
    {
        const CudaSize<3> volDim(maxTileWidth, maxTileHeight, _sgmParams.maxDepths);

        _volumeBestSim_dmp.allocate(volDim);
        _volumeSecBestSim_dmp.allocate(volDim);
    }

    // allocate similarity volume optimization buffers
    if (sgmParams.doSgmOptimizeVolume)
    {
        const size_t maxTileSide = std::max(maxTileWidth, maxTileHeight);

        _volumeSliceAccA_dmp.allocate(CudaSize<2>(maxTileSide, _sgmParams.maxDepths));
        _volumeSliceAccB_dmp.allocate(CudaSize<2>(maxTileSide, _sgmParams.maxDepths));
        _volumeAxisAcc_dmp.allocate(CudaSize<2>(maxTileSide, 1));
    }
}

double Sgm::getDeviceMemoryConsumption() const
{
    size_t bytes = 0;

    bytes += _depths_dmp.getBytesPadded();
    bytes += _depthThicknessMap_dmp.getBytesPadded();
    bytes += _depthSimMap_dmp.getBytesPadded();
    bytes += _normalMap_dmp.getBytesPadded();
    bytes += _volumeBestSim_dmp.getBytesPadded();
    bytes += _volumeSecBestSim_dmp.getBytesPadded();
    bytes += _volumeSliceAccA_dmp.getBytesPadded();
    bytes += _volumeSliceAccB_dmp.getBytesPadded();
    bytes += _volumeAxisAcc_dmp.getBytesPadded();

    return (double(bytes) / (1024.0 * 1024.0));
}

double Sgm::getDeviceMemoryConsumptionUnpadded() const
{
    size_t bytes = 0;

    bytes += _depths_dmp.getBytesUnpadded();
    bytes += _depthThicknessMap_dmp.getBytesUnpadded();
    bytes += _depthSimMap_dmp.getBytesUnpadded();
    bytes += _normalMap_dmp.getBytesUnpadded();
    bytes += _volumeBestSim_dmp.getBytesUnpadded();
    bytes += _volumeSecBestSim_dmp.getBytesUnpadded();
    bytes += _volumeSliceAccA_dmp.getBytesUnpadded();
    bytes += _volumeSliceAccB_dmp.getBytesUnpadded();
    bytes += _volumeAxisAcc_dmp.getBytesUnpadded();

    return (double(bytes) / (1024.0 * 1024.0));
}

void Sgm::sgmRc(const Tile& tile, const SgmDepthList& tileDepthList)
{
    const IndexT viewId = _mp.getViewId(tile.rc);

    ALICEVISION_LOG_INFO(tile << "SGM depth/thickness map of view id: " << viewId << ", rc: " << tile.rc << " (" << (tile.rc + 1) << " / "
                              << _mp.ncams << ").");

    // check SGM depth list and T cameras
    if (tile.sgmTCams.empty() || tileDepthList.getDepths().empty())
        ALICEVISION_THROW_ERROR(tile << "Cannot compute Semi-Global Matching, no depths or no T cameras (viewId: " << viewId << ").");

    // copy rc depth data in page-locked host memory
    for (int i = 0; i < tileDepthList.getDepths().size(); ++i)
        _depths_hmh(i, 0) = tileDepthList.getDepths()[i];

    // copy rc depth data in device memory
    _depths_dmp.copyFrom(_depths_hmh, _stream);

    // compute best sim and second best sim volumes
    computeSimilarityVolumes(tile, tileDepthList);

    // export intermediate volume information (if requested by user)
    exportVolumeInformation(tile, tileDepthList, _volumeSecBestSim_dmp, "beforeFiltering");

    // this is here for experimental purposes
    // to show how SGGC work on non optimized depthmaps
    // it must equals to true in normal case
    if (_sgmParams.doSgmOptimizeVolume)
    {
        optimizeSimilarityVolume(tile, tileDepthList);
    }
    else
    {
        // best sim volume is normally reuse to put optimized similarity
        _volumeBestSim_dmp.copyFrom(_volumeSecBestSim_dmp, _stream);
    }

    // export intermediate volume information (if requested by user)
    exportVolumeInformation(tile, tileDepthList, _volumeBestSim_dmp, "afterFiltering");

    // retrieve best depth
    retrieveBestDepth(tile, tileDepthList);

    // export intermediate depth/sim map (if requested by user)
    if (_sgmParams.exportIntermediateDepthSimMaps)
    {
        writeDepthSimMap(tile.rc, _mp, _tileParams, tile.roi, _depthSimMap_dmp, _sgmParams.scale, _sgmParams.stepXY, "sgm");
    }

    // compute normal map from depth/sim map if needed
    if (_computeNormalMap)
    {
        // downscale the region of interest
        const ROI downscaledRoi = downscaleROI(tile.roi, _sgmParams.scale * _sgmParams.stepXY);

        // get R device camera parameters id from cache
        DeviceCache& deviceCache = DeviceCache::getInstance();
        const int rcDeviceCameraParamsId = deviceCache.requestCameraParamsId(tile.rc, _sgmParams.scale, _mp);

        ALICEVISION_LOG_INFO(tile << "SGM compute normal map of view id: " << viewId << ", rc: " << tile.rc << " (" << (tile.rc + 1) << " / "
                                  << _mp.ncams << ").");

        cuda_depthSimMapComputeNormal(_normalMap_dmp, _depthSimMap_dmp, rcDeviceCameraParamsId, _sgmParams.stepXY, downscaledRoi, _stream);

        // export intermediate normal map (if requested by user)
        if (_sgmParams.exportIntermediateNormalMaps)
        {
            writeNormalMap(tile.rc, _mp, _tileParams, tile.roi, _normalMap_dmp, _sgmParams.scale, _sgmParams.stepXY, "sgm");
        }
    }

    ALICEVISION_LOG_INFO(tile << "SGM depth/thickness map done.");
}

void Sgm::smoothThicknessMap(const Tile& tile, const RefineParams& refineParams)
{
    ALICEVISION_LOG_INFO(tile << "SGM Smooth thickness map.");

    // downscale the region of interest
    const ROI downscaledRoi = downscaleROI(tile.roi, _sgmParams.scale * _sgmParams.stepXY);

    // in-place result thickness map smoothing with adjacent pixels
    cuda_depthThicknessSmoothThickness(_depthThicknessMap_dmp, _sgmParams, refineParams, downscaledRoi, _stream);

    ALICEVISION_LOG_INFO(tile << "SGM Smooth thickness map done.");
}

void Sgm::computeSimilarityVolumes(const Tile& tile, const SgmDepthList& tileDepthList)
{
    ALICEVISION_LOG_INFO(tile << "SGM Compute similarity volume.");

    // downscale the region of interest
    const ROI downscaledRoi = downscaleROI(tile.roi, _sgmParams.scale * _sgmParams.stepXY);

    // initialize the two similarity volumes at 255
    cuda_volumeInitialize(_volumeBestSim_dmp, 255.f, _stream);
    cuda_volumeInitialize(_volumeSecBestSim_dmp, 255.f, _stream);

    // get device cache instance
    DeviceCache& deviceCache = DeviceCache::getInstance();

    // get R device camera parameters id from cache
    const int rcDeviceCameraParamsId = deviceCache.requestCameraParamsId(tile.rc, _sgmParams.scale, _mp);

    // get R device mipmap image from cache
    const DeviceMipmapImage& rcDeviceMipmapImage = deviceCache.requestMipmapImage(tile.rc, _mp);

    // compute similarity volume per Rc Tc
    for (std::size_t tci = 0; tci < tile.sgmTCams.size(); ++tci)
    {
        const int tc = tile.sgmTCams.at(tci);

        const int firstDepth = tileDepthList.getDepthsTcLimits()[tci].x;
        const int lastDepth = firstDepth + tileDepthList.getDepthsTcLimits()[tci].y;

        const Range tcDepthRange(firstDepth, lastDepth);

        // get T device camera parameters id from cache
        const int tcDeviceCameraParamsId = deviceCache.requestCameraParamsId(tc, _sgmParams.scale, _mp);

        // get T device mipmap image from cache
        const DeviceMipmapImage& tcDeviceMipmapImage = deviceCache.requestMipmapImage(tc, _mp);

        ALICEVISION_LOG_DEBUG(tile << "Compute similarity volume:" << std::endl
                                   << "\t- rc: " << tile.rc << std::endl
                                   << "\t- tc: " << tc << " (" << (tci + 1) << "/" << tile.sgmTCams.size() << ")" << std::endl
                                   << "\t- rc camera parameters id: " << rcDeviceCameraParamsId << std::endl
                                   << "\t- tc camera parameters id: " << tcDeviceCameraParamsId << std::endl
                                   << "\t- tc first depth: " << firstDepth << std::endl
                                   << "\t- tc last depth: " << lastDepth << std::endl
                                   << "\t- tile range x: [" << downscaledRoi.x.begin << " - " << downscaledRoi.x.end << "]" << std::endl
                                   << "\t- tile range y: [" << downscaledRoi.y.begin << " - " << downscaledRoi.y.end << "]" << std::endl);

        cuda_volumeComputeSimilarity(_volumeBestSim_dmp,
                                     _volumeSecBestSim_dmp,
                                     _depths_dmp,
                                     rcDeviceCameraParamsId,
                                     tcDeviceCameraParamsId,
                                     rcDeviceMipmapImage,
                                     tcDeviceMipmapImage,
                                     _sgmParams,
                                     tcDepthRange,
                                     downscaledRoi,
                                     _stream);
    }

    // update second best uninitialized similarity volume values with first best similarity volume values
    // - allows to avoid the particular case with a single tc (second best volume has no valid similarity values)
    // - useful if a tc alone contributes to the calculation of a subpart of the similarity volume
    if (_sgmParams.updateUninitializedSim)  // should always be true, false for debug purposes
    {
        ALICEVISION_LOG_DEBUG(tile << "SGM Update uninitialized similarity volume values from best similarity volume.");

        cuda_volumeUpdateUninitializedSimilarity(_volumeBestSim_dmp, _volumeSecBestSim_dmp, _stream);
    }

    ALICEVISION_LOG_INFO(tile << "SGM Compute similarity volume done.");
}

void Sgm::optimizeSimilarityVolume(const Tile& tile, const SgmDepthList& tileDepthList)
{
    ALICEVISION_LOG_INFO(tile << "SGM Optimizing volume (filtering axes: " << _sgmParams.filteringAxes << ").");

    // downscale the region of interest
    const ROI downscaledRoi = downscaleROI(tile.roi, _sgmParams.scale * _sgmParams.stepXY);

    // get R device mipmap image from cache
    DeviceCache& deviceCache = DeviceCache::getInstance();
    const DeviceMipmapImage& rcDeviceMipmapImage = deviceCache.requestMipmapImage(tile.rc, _mp);

    cuda_volumeOptimize(_volumeBestSim_dmp,     // output volume (reuse best sim to put optimized similarity)
                        _volumeSliceAccA_dmp,   // slice A accumulation buffer pre-allocate
                        _volumeSliceAccB_dmp,   // slice B accumulation buffer pre-allocate
                        _volumeAxisAcc_dmp,     // axis accumulation buffer pre-allocate
                        _volumeSecBestSim_dmp,  // input volume
                        rcDeviceMipmapImage,
                        _sgmParams,
                        tileDepthList.getDepths().size(),
                        downscaledRoi,
                        _stream);

    ALICEVISION_LOG_INFO(tile << "SGM Optimizing volume done.");
}

void Sgm::retrieveBestDepth(const Tile& tile, const SgmDepthList& tileDepthList)
{
    ALICEVISION_LOG_INFO(tile << "SGM Retrieve best depth in volume.");

    // downscale the region of interest
    const ROI downscaledRoi = downscaleROI(tile.roi, _sgmParams.scale * _sgmParams.stepXY);

    // get depth range
    const Range depthRange(0, tileDepthList.getDepths().size());

    // get R device camera parameters id from cache
    DeviceCache& deviceCache = DeviceCache::getInstance();
    const int rcDeviceCameraParamsId = deviceCache.requestCameraParamsId(tile.rc, 1, _mp);

    cuda_volumeRetrieveBestDepth(_depthThicknessMap_dmp,  // output depth thickness map
                                 _depthSimMap_dmp,        // output depth/sim map (or empty)
                                 _depths_dmp,             // rc depth
                                 _volumeBestSim_dmp,      // second best sim volume optimized in best sim volume
                                 rcDeviceCameraParamsId,
                                 _sgmParams,
                                 depthRange,
                                 downscaledRoi,
                                 _stream);

    ALICEVISION_LOG_INFO(tile << "SGM Retrieve best depth in volume done.");
}

void Sgm::exportVolumeInformation(const Tile& tile,
                                  const SgmDepthList& tileDepthList,
                                  const CudaDeviceMemoryPitched<TSim, 3>& in_volume_dmp,
                                  const std::string& name) const
{
    if (!_sgmParams.exportIntermediateVolumes && !_sgmParams.exportIntermediateCrossVolumes && !_sgmParams.exportIntermediateVolume9pCsv)
    {
        // nothing to do
        return;
    }

    // get file tile begin indexes (default is single tile)
    int tileBeginX = -1;
    int tileBeginY = -1;

    if (tile.nbTiles > 1)
    {
        tileBeginX = tile.roi.x.begin;
        tileBeginY = tile.roi.y.begin;
    }

    // copy device similarity volume to host memory
    CudaHostMemoryHeap<TSim, 3> volumeSim_hmh(in_volume_dmp.getSize());
    volumeSim_hmh.copyFrom(in_volume_dmp);

    if (_sgmParams.exportIntermediateVolumes)
    {
        ALICEVISION_LOG_INFO(tile << "Export similarity volume (" << name << ").");

        const std::string volumePath = getFileNameFromIndex(_mp, tile.rc, mvsUtils::EFileType::volume, "_" + name, tileBeginX, tileBeginY);

        exportSimilarityVolume(volumeSim_hmh, tileDepthList.getDepths(), _mp, tile.rc, _sgmParams, volumePath, tile.roi);

        ALICEVISION_LOG_INFO(tile << "Export similarity volume (" << name << ") done.");
    }

    if (_sgmParams.exportIntermediateCrossVolumes)
    {
        ALICEVISION_LOG_INFO(tile << "Export similarity volume cross (" << name << ").");

        const std::string volumeCrossPath = getFileNameFromIndex(_mp, tile.rc, mvsUtils::EFileType::volumeCross, "_" + name, tileBeginX, tileBeginY);

        exportSimilarityVolumeCross(volumeSim_hmh, tileDepthList.getDepths(), _mp, tile.rc, _sgmParams, volumeCrossPath, tile.roi);

        ALICEVISION_LOG_INFO(tile << "Export similarity volume cross (" << name << ") done.");
    }

    if (_sgmParams.exportIntermediateTopographicCutVolumes)
    {
        ALICEVISION_LOG_INFO(tile << "Export similarity volume topographic cut (" << name << ").");

        const std::string volumeCutPath =
          getFileNameFromIndex(_mp, tile.rc, mvsUtils::EFileType::volumeTopographicCut, "_" + name, tileBeginX, tileBeginY);

        exportSimilarityVolumeTopographicCut(volumeSim_hmh, tileDepthList.getDepths(), _mp, tile.rc, _sgmParams, volumeCutPath, tile.roi);

        ALICEVISION_LOG_INFO(tile << "Export similarity volume topographic cut (" << name << ") done.");
    }

    if (_sgmParams.exportIntermediateVolume9pCsv)
    {
        ALICEVISION_LOG_INFO(tile << "Export similarity volume 9 points CSV (" << name << ").");

        const std::string stats9Path = getFileNameFromIndex(_mp, tile.rc, mvsUtils::EFileType::stats9p, "_sgm", tileBeginX, tileBeginY);

        exportSimilaritySamplesCSV(volumeSim_hmh, tileDepthList.getDepths(), name, _sgmParams, stats9Path, tile.roi);

        ALICEVISION_LOG_INFO(tile << "Export similarity volume 9 points CSV (" << name << ") done.");
    }
}

#elif ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_VULKAN)

Sgm::Sgm(const mvsUtils::MultiViewParams& mp,
         const mvsUtils::TileParams& tileParams,
         const SgmParams& sgmParams,
         bool computeDepthSimMap,
         bool computeNormalMap,
         uint32_t deviceID)
  : _mp(mp),
    _tileParams(tileParams),
    _sgmParams(sgmParams),
    _computeDepthSimMap(computeDepthSimMap || sgmParams.exportIntermediateDepthSimMaps),
    _computeNormalMap(computeNormalMap || sgmParams.exportIntermediateNormalMaps),
    _deviceID(deviceID)
{
    // get tile maximum dimensions
    const int downscale = _sgmParams.scale * _sgmParams.stepXY;
    const int maxTileWidth = divideRoundUp(tileParams.bufferWidth, downscale);
    const int maxTileHeight = divideRoundUp(tileParams.bufferHeight, downscale);

    // compute map maximum dimensions
    vk::Extent2D mapDim(maxTileWidth, maxTileHeight);

    _depths_dmp = std::make_shared<VulkanImage<float>>(VulkanMemoryBase<float>::create2DImage(deviceID, mapDim.width, mapDim.height, vk::Format::eR32Sfloat, 1, true, false));
    _depthThicknessMap_dmp = std::make_shared<VulkanImage<float2>>(VulkanMemoryBase<float2>::create2DImage(deviceID, mapDim.width, mapDim.height, vk::Format::eR32G32Sfloat, 1, true, false));
    if (_computeDepthSimMap)
        _depthSimMap_dmp = std::make_shared<VulkanImage<float2>>(VulkanMemoryBase<float2>::create2DImage(deviceID, mapDim.width, mapDim.height, vk::Format::eR32G32Sfloat, 1, true, false));

    // allocate normal map in device memory
    if (_computeNormalMap)
        _normalMap_dmp = std::make_shared<VulkanImage<float4>>(VulkanMemoryBase<float4>::create2DImage(deviceID, mapDim.width, mapDim.height, vk::Format::eR32G32B32A32Sfloat, 1, true, false));

    vk::Extent3D volDim(maxTileWidth, maxTileHeight, _sgmParams.maxDepths);
#if TSIM_USE_FLOAT
    _volumeBestSim_dmp = std::make_shared<VulkanImage<TSim>>(VulkanMemoryBase<TSim>::create3DImage(deviceID, volDim.width, volDim.height, volDim.depth, vk::Format::eR32Sfloat, 1, true, false));
    _volumeSecBestSim_dmp = std::make_shared<VulkanImage<TSim>>(VulkanMemoryBase<TSim>::create3DImage(deviceID, volDim.width, volDim.height, volDim.depth, vk::Format::eR32Sfloat, 1, true, false));
#else
    _volumeBestSim_dmp = std::make_shared<VulkanImage<TSim>>(VulkanMemoryBase<TSim>::create3DImage(deviceID, volDim.width, volDim.height, volDim.depth, vk::Format::eR8Uint, 1, true, false));
    _volumeSecBestSim_dmp = std::make_shared<VulkanImage<TSim>>(VulkanMemoryBase<TSim>::create3DImage(deviceID, volDim.width, volDim.height, volDim.depth, vk::Format::eR8Uint, 1, true, false));
#endif
    // allocate similarity volume optimization buffers
    if (sgmParams.doSgmOptimizeVolume)
    {
        const size_t maxTileSide = std::max(maxTileWidth, maxTileHeight);
#if TSIM_USE_FLOAT
        _volumeSliceAccA_dmp = std::make_shared<VulkanImage<TSimAcc>>(VulkanMemoryBase<TSimAcc>::create2DImage(deviceID, maxTileSide, _sgmParams.maxDepths, vk::Format::eR32Sfloat, 1, true, false));
        _volumeSliceAccB_dmp = std::make_shared<VulkanImage<TSimAcc>>(VulkanMemoryBase<TSimAcc>::create2DImage(deviceID, maxTileSide, _sgmParams.maxDepths, vk::Format::eR32Sfloat, 1, true, false));
        _volumeAxisAcc_dmp = std::make_shared<VulkanImage<TSimAcc>>(VulkanMemoryBase<TSimAcc>::create2DImage(deviceID, maxTileSide, 1, vk::Format::eR8Uint, 1, true, false));
#else
        _volumeSliceAccA_dmp = std::make_shared<VulkanImage<TSimAcc>>(VulkanMemoryBase<TSimAcc>::create2DImage(deviceID, maxTileSide, _sgmParams.maxDepths, vk::Format::eR32Uint, 1, true, false));
        _volumeSliceAccB_dmp = std::make_shared<VulkanImage<TSimAcc>>(VulkanMemoryBase<TSimAcc>::create2DImage(deviceID, maxTileSide, _sgmParams.maxDepths, vk::Format::eR32Uint, 1, true, false));
        _volumeAxisAcc_dmp = std::make_shared<VulkanImage<TSimAcc>>(VulkanMemoryBase<TSimAcc>::create2DImage(deviceID, maxTileSide, 1, vk::Format::eR8Uint, 1, true, false));
#endif
    }
}

double Sgm::getDeviceMemoryConsumption() const
{
    size_t bytes = 0;

    if(_depths_dmp)
        bytes += _depths_dmp->getByteSize();
    if(_depthThicknessMap_dmp)
        bytes += _depthThicknessMap_dmp->getByteSize();
    if(_depthSimMap_dmp)
        bytes += _depthSimMap_dmp->getByteSize();
    if(_normalMap_dmp)
        bytes += _normalMap_dmp->getByteSize();
    if(_depthThicknessMap_dmp)
        bytes += _depthThicknessMap_dmp->getByteSize();
    if(_volumeSecBestSim_dmp)
        bytes += _volumeSecBestSim_dmp->getByteSize();
    if(_volumeSliceAccA_dmp)
        bytes += _volumeSliceAccA_dmp->getByteSize();
    if(_volumeSliceAccB_dmp)
        bytes += _volumeSliceAccB_dmp->getByteSize();
    if(_volumeAxisAcc_dmp)
        bytes += _volumeAxisAcc_dmp->getByteSize();

    return bytes / (1024.0 * 1024.0);
}

void Sgm::sgmRc(const Tile& tile, const SgmDepthList& tileDepthList)
{
    const IndexT viewId = _mp.getViewId(tile.rc);

    ALICEVISION_LOG_INFO(tile << "SGM depth/thickness map of view id: " << viewId << ", rc: " << tile.rc << " (" << (tile.rc + 1) << " / "
                              << _mp.ncams << ").");

    // check SGM depth list and T cameras
    if (tile.sgmTCams.empty() || tileDepthList.getDepths().empty())
        ALICEVISION_THROW_ERROR(tile << "Cannot compute Semi-Global Matching, no depths or no T cameras (viewId: " << viewId << ").");

    // Create a host buffer
    std::vector<float> _depths_hmh = {};
    _depths_hmh.reserve(tileDepthList.getDepths().size());

    // copy rc depth data in page-locked host memory
    for (int i = 0; i < tileDepthList.getDepths().size(); ++i)
        _depths_hmh[i] = tileDepthList.getDepths()[i];

    // copy rc depth data in device memory
    _depths_dmp->copyFromHostToImage(_depths_hmh.data(), _depths_hmh.size(), 0, 0);

    // compute best sim and second best sim volumes
    computeSimilarityVolumes(tile, tileDepthList);

    // export intermediate volume information (if requested by user)
    exportVolumeInformation(tile, tileDepthList, *_volumeSecBestSim_dmp, "beforeFiltering");

    // this is here for experimental purposes
    // to show how SGGC work on non optimized depthmaps
    // it must equals to true in normal case
    if (_sgmParams.doSgmOptimizeVolume)
    {
        optimizeSimilarityVolume(tile, tileDepthList);
    }
    else
    {
        // best sim volume is normally reuse to put optimized similarity
        _volumeBestSim_dmp->copyFromAllocation(*_volumeSecBestSim_dmp, 0, 0);
    }

    // export intermediate volume information (if requested by user)
    exportVolumeInformation(tile, tileDepthList, *_volumeBestSim_dmp, "afterFiltering");

    // retrieve best depth
    retrieveBestDepth(tile, tileDepthList);

    // export intermediate depth/sim map (if requested by user)
    if (_sgmParams.exportIntermediateDepthSimMaps)
    {
        writeDepthSimMap(tile.rc, _mp, _tileParams, tile.roi, *_depthSimMap_dmp, _sgmParams.scale, _sgmParams.stepXY, "sgm");
    }

    // compute normal map from depth/sim map if needed
    if (_computeNormalMap)
    {
        // downscale the region of interest
        const ROI downscaledRoi = downscaleROI(tile.roi, _sgmParams.scale * _sgmParams.stepXY);

        // get R device camera parameters id from cache
        DeviceCache& deviceCache = DeviceCache::getInstance();
        const int rcDeviceCameraParamsId = deviceCache.requestCameraParamsId(_deviceID, tile.rc, _sgmParams.scale, _mp);

        ALICEVISION_LOG_INFO(tile << "SGM compute normal map of view id: " << viewId << ", rc: " << tile.rc << " (" << (tile.rc + 1) << " / "
                                  << _mp.ncams << ").");

        struct PushConstants
        {
            int32_t TWish;
            int32_t rcDeviceCameraParamsId;
            int32_t stepXY;
            ROI roi;
        };

        const auto pc = PushConstants{3, rcDeviceCameraParamsId, _sgmParams.stepXY, downscaledRoi};

        // DepthMap Similiarity Compute Normal
        VulkanCommandManager::getInstance(_deviceID)
            ->wait()
            ->reset()
            ->begin()
            ->shader(ALICEVISION_LOOKUP_SPIRV_MODULE(DepthSimMapComputeNormal))
            ->pushConstant(pc)
            ->bind(*_normalMap_dmp, vk::DescriptorType::eStorageImage)
            ->bind(*_depthSimMap_dmp, vk::DescriptorType::eStorageImage)
            ->bind(DeviceCameraParamsArray::getInstance(_deviceID)->getCameraParamsArray(), vk::DescriptorType::eUniformBuffer)
            ->transferImageLayout(*_normalMap_dmp, vk::ImageLayout::eGeneral, 0, 1)
            ->transferImageLayout(*_depthSimMap_dmp, vk::ImageLayout::eGeneral, 0, 1)
            ->workgroups(vk::Extent3D(divUp(downscaledRoi.width(), 8), divUp(downscaledRoi.height(), 8), 1))
            ->dispatch()
            ->end()
            ->submit()
            ->wait();

        // export intermediate normal map (if requested by user)
        if (_sgmParams.exportIntermediateNormalMaps)
        {
            writeNormalMap(tile.rc, _mp, _tileParams, tile.roi, *_normalMap_dmp, _sgmParams.scale, _sgmParams.stepXY, "sgm");
        }
    }

    ALICEVISION_LOG_INFO(tile << "SGM depth/thickness map done.");
}

void Sgm::smoothThicknessMap(const Tile& tile, const RefineParams& refineParams)
{
    ALICEVISION_LOG_INFO(tile << "SGM Smooth thickness map.");

    // downscale the region of interest
    const ROI downscaledRoi = downscaleROI(tile.roi, _sgmParams.scale * _sgmParams.stepXY);

    // in-place result thickness map smoothing with adjacent pixels
    const int sgmScaleStep = _sgmParams.scale * _sgmParams.stepXY;
    const int refineScaleStep = refineParams.scale * refineParams.stepXY;

    // min/max number of Refine samples in SGM thickness area
    const float minNbRefineSamples = 2.f;
    const float maxNbRefineSamples = std::max(sgmScaleStep / float(refineScaleStep), minNbRefineSamples);

    // min/max SGM thickness inflate factor
    const float minThicknessInflate = refineParams.halfNbDepths / maxNbRefineSamples;
    const float maxThicknessInflate = refineParams.halfNbDepths / minNbRefineSamples;

    struct PushConstants
    {
        float minThicknessInflate;
        float maxThicknessInflate;
        ROI roi;
    };

    const auto pc = PushConstants{minThicknessInflate, maxThicknessInflate, downscaledRoi};

    VulkanCommandManager::getInstance(_deviceID)
        ->wait()
        ->reset()
        ->begin()
        ->shader(ALICEVISION_LOOKUP_SPIRV_MODULE(DepthThicknessSmoothThickness))
        ->pushConstant(pc)
        ->bind(*_depthThicknessMap_dmp, vk::DescriptorType::eStorageImage)
        ->transferImageLayout(*_depthThicknessMap_dmp, vk::ImageLayout::eGeneral, 0, 1)
        ->workgroups(vk::Extent3D(divUp(downscaledRoi.width(), 8), divUp(downscaledRoi.height(), 8), 1))
        ->dispatch()
        ->end()
        ->submit()
        ->wait();

    ALICEVISION_LOG_INFO(tile << "SGM Smooth thickness map done.");
}

void Sgm::computeSimilarityVolumes(const Tile& tile, const SgmDepthList& tileDepthList)
{
    ALICEVISION_LOG_INFO(tile << "SGM Compute similarity volume.");

    // downscale the region of interest
    const ROI downscaledRoi = downscaleROI(tile.roi, _sgmParams.scale * _sgmParams.stepXY);

    // initialize the two similarity volumes at 255
    // This used to be CUDA code, for easyness we do this on the host
    // TODO: Figure out what performance penalty this may cause
    std::vector<TSim> _volumeBestSim_hm = {};
    std::vector<TSim> _volumeSecBestSim_hm = {};
    _volumeBestSim_hm.reserve(_volumeBestSim_dmp->getSize());
    _volumeSecBestSim_hm.reserve(_volumeSecBestSim_dmp->getSize());
    std::ranges::fill(_volumeBestSim_hm, 255.f);
    std::ranges::fill(_volumeSecBestSim_hm, 255.f);
    _volumeBestSim_dmp->copyFromHostToImage(_volumeBestSim_hm.data(), _volumeBestSim_hm.size(), 0, 0);
    _volumeSecBestSim_dmp->copyFromHostToImage(_volumeSecBestSim_hm.data(), _volumeSecBestSim_hm.size(), 0, 0);

    // get device cache instance
    DeviceCache& deviceCache = DeviceCache::getInstance();

    // get R device camera parameters id from cache
    const int rcDeviceCameraParamsId = deviceCache.requestCameraParamsId(_deviceID, tile.rc, _sgmParams.scale, _mp);

    // get R device mipmap image from cache
    const DeviceMipmapImage& rcDeviceMipmapImage = deviceCache.requestMipmapImage(_deviceID, tile.rc, _mp);

    // TODO: Parallelize with OpenMP?
    // compute similarity volume per Rc Tc
    for (std::size_t tci = 0; tci < tile.sgmTCams.size(); ++tci)
    {
        const int tc = tile.sgmTCams.at(tci);

        const int firstDepth = tileDepthList.getDepthsTcLimits()[tci].x;
        const int lastDepth = firstDepth + tileDepthList.getDepthsTcLimits()[tci].y;

        const Range tcDepthRange(firstDepth, lastDepth);

        // get T device camera parameters id from cache
        const int tcDeviceCameraParamsId = deviceCache.requestCameraParamsId(_deviceID, tc, _sgmParams.scale, _mp);

        // get T device mipmap image from cache
        const DeviceMipmapImage& tcDeviceMipmapImage = deviceCache.requestMipmapImage(_deviceID, tc, _mp);

        ALICEVISION_LOG_DEBUG(tile << "Compute similarity volume:" << std::endl
                                   << "\t- rc: " << tile.rc << std::endl
                                   << "\t- tc: " << tc << " (" << (tci + 1) << "/" << tile.sgmTCams.size() << ")" << std::endl
                                   << "\t- rc camera parameters id: " << rcDeviceCameraParamsId << std::endl
                                   << "\t- tc camera parameters id: " << tcDeviceCameraParamsId << std::endl
                                   << "\t- tc first depth: " << firstDepth << std::endl
                                   << "\t- tc last depth: " << lastDepth << std::endl
                                   << "\t- tile range x: [" << downscaledRoi.x.begin << " - " << downscaledRoi.x.end << "]" << std::endl
                                   << "\t- tile range y: [" << downscaledRoi.y.begin << " - " << downscaledRoi.y.end << "]" << std::endl);

        struct PushConstants
        {
            int32_t rcDeviceCameraParamsId;
            int32_t tcDeviceCameraParamsId;
            uint32_t rcSgmLevelWidth;
            uint32_t rcSgmLevelHeight;
            uint32_t tcSgmLevelWidth;
            uint32_t tcSgmLevelHeight;
            float rcMipmapLevel;
            int32_t stepXY;
            int32_t wsh;
            float invGammaC;
            float invGammaP;
            bool useConsistentScale;
            bool useCustomPatchPattern;
            Range depthRange;
            ROI roi;
        };

        const auto rcSgmLevelExtent = rcDeviceMipmapImage.getDimensions(_sgmParams.scale);
        const auto tcSgmLevelExtent = tcDeviceMipmapImage.getDimensions(_sgmParams.scale);
        const auto mipLevel = rcDeviceMipmapImage.getLevel(_sgmParams.scale);
        const auto pc = PushConstants{
            rcDeviceCameraParamsId,
            tcDeviceCameraParamsId,
            rcSgmLevelExtent.width,
            rcSgmLevelExtent.height,
            tcSgmLevelExtent.width,
            tcSgmLevelExtent.height,
            mipLevel,
            _sgmParams.stepXY,
            _sgmParams.wsh,
            1.f / static_cast<float>(_sgmParams.gammaC), // inverted gammaC
            1.f / static_cast<float>(_sgmParams.gammaP), // inverted gammaP
            _sgmParams.useConsistentScale,
            _sgmParams.useCustomPatchPattern,
            tcDepthRange,
            downscaledRoi
        };

        // TODO: Extend VulkanCommandManager so that push constants do not override each other! Currently, we can only execute sequentially!
        // TODO: Re above: This might not work, because it technically overwrites the Push Constants. So we must call submit() before adding new ones.
        // TODO: However, this is generally fine, because pc only has to live until dispatch(), where they are copied.
        // TODO: So maybe just omitting the wait() call is fine (top AND bottom). This is safe, if no memory races can occur!
        // TODO: Check if the shader is distinct enough to write to different depth regions in the parent for loop (likely, because CUDA assumes it apparently)
        // TODO: Check if vk::WholeSize is fitting or if we need to transform between the specific levels accessed!
        VulkanCommandManager::getInstance(_deviceID)
            ->wait()
            ->reset()
            ->begin()
#ifdef TSIM_USE_FLOAT
            ->shader(ALICEVISION_LOOKUP_SPIRV_MODULE(VolumeComputeSimiliarity_float))
#else
            ->shader(ALICEVISION_LOOKUP_SPIRV_MODULE(VolumeComputeSimiliarity_uchar))
#endif
            ->pushConstant(pc)
            ->bind(*_volumeBestSim_dmp, vk::DescriptorType::eStorageImage)
            ->bind(*_volumeSecBestSim_dmp, vk::DescriptorType::eStorageImage)
            ->bind(*_depths_dmp, vk::DescriptorType::eStorageImage)
            ->bind(rcDeviceMipmapImage.getImage(), vk::DescriptorType::eSampledImage)
            ->bind(tcDeviceMipmapImage.getImage(), vk::DescriptorType::eSampledImage)
            ->bind(DeviceCameraParamsArray::getInstance(_deviceID)->getCameraParamsArray(), vk::DescriptorType::eUniformBuffer)
            ->bind(DevicePatchPatternConstant::getInstance(_deviceID)->getPatchPatternConstant(), vk::DescriptorType::eUniformBuffer)
            ->transferImageLayout(*_volumeBestSim_dmp, vk::ImageLayout::eGeneral, 0, 1)
            ->transferImageLayout(*_volumeSecBestSim_dmp, vk::ImageLayout::eGeneral, 0, 1)
            ->transferImageLayout(*_depths_dmp, vk::ImageLayout::eGeneral, 0, 1)
            ->transferImageLayout(rcDeviceMipmapImage.getImage(), vk::ImageLayout::eShaderReadOnlyOptimal, 0, vk::WholeSize)
            ->transferImageLayout(tcDeviceMipmapImage.getImage(), vk::ImageLayout::eShaderReadOnlyOptimal, 0, vk::WholeSize)
            ->workgroups(vk::Extent3D(divUp(downscaledRoi.width(), 32), divUp(downscaledRoi.height(), 32), tcDepthRange.size()))
            ->dispatch()
            ->end()
            ->submit()
            ->wait();
    }

    // update second best uninitialized similarity volume values with first best similarity volume values
    // - allows to avoid the particular case with a single tc (second best volume has no valid similarity values)
    // - useful if a tc alone contributes to the calculation of a subpart of the similarity volume
    if (_sgmParams.updateUninitializedSim)  // should always be true, false for debug purposes
    {
        ALICEVISION_LOG_DEBUG(tile << "SGM Update uninitialized similarity volume values from best similarity volume.");

        const auto dimSizes = _volumeSecBestSim_dmp->getDimensionSizes().value();

        VulkanCommandManager::getInstance(_deviceID)
            ->wait()
            ->reset()
            ->begin()
#ifdef TSIM_USE_FLOAT
            ->shader(ALICEVISION_LOOKUP_SPIRV_MODULE(VolumeUpdateUninitialized_float))
#else
            ->shader(ALICEVISION_LOOKUP_SPIRV_MODULE(VolumeUpdateUninitialized_uchar))
#endif
            ->bind(*_volumeSecBestSim_dmp, vk::DescriptorType::eStorageImage)
            ->bind(*_volumeBestSim_dmp, vk::DescriptorType::eStorageImage)
            ->transferImageLayout(*_volumeBestSim_dmp, vk::ImageLayout::eGeneral, 0, 1)
            ->transferImageLayout(*_volumeSecBestSim_dmp, vk::ImageLayout::eGeneral, 0, 1)
            ->workgroups(vk::Extent3D(divUp(dimSizes.width, 32), divUp(dimSizes.height, 32), dimSizes.depth))
            ->dispatch()
            ->end()
            ->submit()
            ->wait();
    }

    ALICEVISION_LOG_INFO(tile << "SGM Compute similarity volume done.");
}

void Sgm::optimizeSimilarityVolume(const Tile& tile, const SgmDepthList& tileDepthList)
{
    ALICEVISION_LOG_INFO(tile << "SGM Optimizing volume (filtering axes: " << _sgmParams.filteringAxes << ").");

    // downscale the region of interest
    const ROI downscaledRoi = downscaleROI(tile.roi, _sgmParams.scale * _sgmParams.stepXY);

    // get R device mipmap image from cache
    DeviceCache& deviceCache = DeviceCache::getInstance();
    const DeviceMipmapImage& rcDeviceMipmapImage = deviceCache.requestMipmapImage(_deviceID, tile.rc, _mp);

    // get R mipmap image level and dimensions
    const float rcMipmapLevel = rcDeviceMipmapImage.getLevel(_sgmParams.scale);
    const auto rcLevelDim = rcDeviceMipmapImage.getDimensions(_sgmParams.scale);

    // update aggregation volume
    int npaths = 0;
    const auto updateAggrVolume = [&](const vk::Extent3D& axisT, bool invX)
    {
        // Get Extent of Volume
        auto volDimExtent = _volumeSecBestSim_dmp->getDimensionSizes().value();
        // override volume depth, use rc depth list last index
        volDimExtent.depth = tileDepthList.getDepths().size();

        const uint32_t volDim[3] = { volDimExtent.width, volDimExtent.height, volDimExtent.depth };

        const size_t volDimX = volDim[axisT.width];
        const size_t volDimY = volDim[axisT.height];
        const size_t volDimZ = volDim[axisT.depth];

        const int3 volDim_ = int3{volDim[0], volDim[1], volDim[2]};
        const int3 axisT_ = int3{axisT.width, axisT.height, axisT.depth};
        const int ySign = (invX ? -1 : 1);

        VulkanImage<TSimAcc>* xzSliceForY_dmpPtr   = &*_volumeSliceAccA_dmp; // Y slice
        VulkanImage<TSimAcc>* xzSliceForYm1_dmpPtr = &*_volumeSliceAccB_dmp; // Y-1 slice
        VulkanImage<TSimAcc>* bestSimInYm1_dmpPtr  = &*_volumeAxisAcc_dmp;   // best sim score along the Y axis for each Z value

        struct PushConstants
        {
            int3 axisT;
            int3 volDim;
            int32_t y;
        };

        const auto pc = PushConstants{volDim_, axisT_, 0};

        VulkanCommandManager::getInstance(_deviceID)
            ->wait()
            ->reset()
            ->begin()
#if TSIM_USE_FLOAT
            ->shader(ALICEVISION_LOOKUP_SPIRV_MODULE(GetVolumeXZSlice_float))
#else
            ->shader(ALICEVISION_LOOKUP_SPIRV_MODULE(GetVolumeXZSlice_uchar))
#endif
            ->pushConstant(pc)
            ->bind(*xzSliceForYm1_dmpPtr, vk::DescriptorType::eStorageImage)
            ->bind(*_volumeSecBestSim_dmp, vk::DescriptorType::eStorageImage)
            ->transferImageLayout(*xzSliceForYm1_dmpPtr, vk::ImageLayout::eGeneral, 0, 1)
            ->transferImageLayout(*_volumeSecBestSim_dmp, vk::ImageLayout::eGeneral, 0, 1)
            ->workgroups(vk::Extent3D(divUp(volDimX, 8), divUp(volDimZ, 8), 1))
            ->dispatch()
            ->end()
            ->submit()
            ->wait();

        struct PushConstants2
        {
            int3 axisT;
            int3 volDim;
            int32_t y;
            float cst;
        };

        const auto pc2 = PushConstants2{volDim_, axisT_, 0, 255};

        VulkanCommandManager::getInstance(_deviceID)
            ->wait()
            ->reset()
            ->begin()
#if TSIM_USE_FLOAT
            ->shader(ALICEVISION_LOOKUP_SPIRV_MODULE(InitVolumeYSlice_float))
#else
            ->shader(ALICEVISION_LOOKUP_SPIRV_MODULE(InitVolumeYSlice_uchar))
#endif
            ->pushConstant(pc2)
            ->bind(*_volumeBestSim_dmp, vk::DescriptorType::eStorageImage)
            ->transferImageLayout(*_volumeBestSim_dmp, vk::ImageLayout::eGeneral, 0, 1)
            ->workgroups(vk::Extent3D(divUp(volDimX, 8), divUp(volDimZ, 8), 1))
            ->dispatch()
            ->end()
            ->submit()
            ->wait();

        for(int iy = 1; iy < volDimY; ++iy) {
            const int y = invX ? volDimY - 1 - iy : iy;

            struct PushConstants3
            {
                int32_t volDimX;
                int32_t volDimZ;
            };

            const auto pc3 = PushConstants3{static_cast<int32_t>(volDimX), static_cast<int32_t>(volDimZ)};

            VulkanCommandManager::getInstance(_deviceID)
                ->wait()
                ->reset()
                ->begin()
#if TSIM_USE_FLOAT
                ->shader(ALICEVISION_LOOKUP_SPIRV_MODULE(ComputeBestZInSlice_float))
#else
                ->shader(ALICEVISION_LOOKUP_SPIRV_MODULE(ComputeBestZInSlice_uint))
#endif
                ->pushConstant(pc3)
                ->bind(*xzSliceForYm1_dmpPtr, vk::DescriptorType::eStorageImage)
                ->bind(*_volumeBestSim_dmp, vk::DescriptorType::eStorageImage)
                ->transferImageLayout(*xzSliceForYm1_dmpPtr, vk::ImageLayout::eGeneral, 0, 1)
                ->transferImageLayout(*_volumeBestSim_dmp, vk::ImageLayout::eGeneral, 0, 1)
                ->workgroups(vk::Extent3D(divUp(volDimX, 64), 1, 1))
                ->dispatch()
                ->end()
                ->submit()
                ->wait();

            const auto pc4 = PushConstants{volDim_, axisT_, y};

            VulkanCommandManager::getInstance(_deviceID)
                ->wait()
                ->reset()
                ->begin()
#if TSIM_USE_FLOAT
                ->shader(ALICEVISION_LOOKUP_SPIRV_MODULE(GetVolumeXZSlice_float))
#else
                ->shader(ALICEVISION_LOOKUP_SPIRV_MODULE(GetVolumeXZSlice_uchar))
#endif
                ->pushConstant(pc4)
                ->bind(*xzSliceForY_dmpPtr, vk::DescriptorType::eStorageImage)
                ->bind(*_volumeSecBestSim_dmp, vk::DescriptorType::eStorageImage)
                ->transferImageLayout(*xzSliceForY_dmpPtr, vk::ImageLayout::eGeneral, 0, 1)
                ->transferImageLayout(*_volumeSecBestSim_dmp, vk::ImageLayout::eGeneral, 0, 1)
                ->workgroups(vk::Extent3D(divUp(volDimX, 8), divUp(volDimZ, 8), 1))
                ->dispatch()
                ->end()
                ->submit()
                ->wait();

            struct PushConstants4
            {
                uint32_t rcSgmLevelWidth;
                uint32_t rcSgmLevelHeight;
                float rcMipmapLevel;
                int3 volDim;
                int3 axisT;
                float step;
                int y;
                float P1;
                float _P2;
                int ySign;
                int filteringIndex;
                ROI roi;
            };

            const auto pc5 = PushConstants4{
                rcLevelDim.width,
                rcLevelDim.height,
                rcMipmapLevel,
                volDim_,
                axisT_,
                static_cast<float>(_sgmParams.stepXY),
                y,
                static_cast<float>(_sgmParams.p1),
                static_cast<float>(_sgmParams.p2Weighting),
                ySign,
                npaths,
                downscaledRoi
            };

            VulkanCommandManager::getInstance(_deviceID)
                ->wait()
                ->reset()
                ->begin()
#if TSIM_USE_FLOAT
                ->shader(ALICEVISION_LOOKUP_SPIRV_MODULE(AggregateCostVolumeAtXInSlices_float))
#else
                ->shader(ALICEVISION_LOOKUP_SPIRV_MODULE(AggregateCostVolumeAtXInSlices_uchar))
#endif
                ->pushConstant(pc5)
                ->bind(rcDeviceMipmapImage.getImage(), vk::DescriptorType::eSampledImage)
                ->bind(*xzSliceForY_dmpPtr, vk::DescriptorType::eStorageImage)
                ->bind(*xzSliceForYm1_dmpPtr, vk::DescriptorType::eStorageImage)
                ->bind(*bestSimInYm1_dmpPtr, vk::DescriptorType::eStorageImage)
                ->bind(*_volumeBestSim_dmp, vk::DescriptorType::eStorageImage)
                ->transferImageLayout(rcDeviceMipmapImage.getImage(), vk::ImageLayout::eShaderReadOnlyOptimal, 0, vk::WholeSize)
                ->transferImageLayout(*xzSliceForY_dmpPtr, vk::ImageLayout::eGeneral, 0, 1)
                ->transferImageLayout(*xzSliceForYm1_dmpPtr, vk::ImageLayout::eGeneral, 0, 1)
                ->transferImageLayout(*bestSimInYm1_dmpPtr, vk::ImageLayout::eGeneral, 0, 1)
                ->transferImageLayout(*_volumeBestSim_dmp, vk::ImageLayout::eGeneral, 0, 1)
                ->workgroups(vk::Extent3D(divUp(volDimX, 64), volDimZ, 1))
                ->dispatch()
                ->end()
                ->submit()
                ->wait();

            std::swap(xzSliceForYm1_dmpPtr, xzSliceForY_dmpPtr);
        }

        // Update depth with
        npaths++;
    };

    // filtering is done on the last axis
    const std::map<char, vk::Extent3D> mapAxes = {
        {'X', {1, 0, 2}}, // XYZ -> YXZ
        {'Y', {0, 1, 2}}, // XYZ
    };

    for(char axis : _sgmParams.filteringAxes)
    {
        const vk::Extent3D& axisT = mapAxes.at(axis);
        updateAggrVolume(axisT, false); // without transpose
        updateAggrVolume(axisT, true);  // with transpose of the last axis
    }

    ALICEVISION_LOG_INFO(tile << "SGM Optimizing volume done.");
}

void Sgm::retrieveBestDepth(const Tile& tile, const SgmDepthList& tileDepthList)
{
    ALICEVISION_LOG_INFO(tile << "SGM Retrieve best depth in volume.");

    // downscale the region of interest
    const ROI downscaledRoi = downscaleROI(tile.roi, _sgmParams.scale * _sgmParams.stepXY);

    // get depth range
    const Range depthRange(0, tileDepthList.getDepths().size());

    // get R device camera parameters id from cache
    DeviceCache& deviceCache = DeviceCache::getInstance();
    const int rcDeviceCameraParamsId = deviceCache.requestCameraParamsId(_deviceID, tile.rc, 1, _mp);

    // constant kernel inputs
    const int scaleStep = _sgmParams.scale * _sgmParams.stepXY;
    const float thicknessMultFactor = 1.f + float(_sgmParams.depthThicknessInflate);
    const float maxSimilarity = float(_sgmParams.maxSimilarity) * 254.f; // convert from (0, 1) to (0, 254)

    struct PushConstants
    {
        int rcDeviceCameraParamsId;
        int volDimZ;
        int scaleStep;
        float thicknessMultFactor;
        float maxSimilarity;
        Range depthRange;
        ROI roi;
    };

    const auto pc = PushConstants{rcDeviceCameraParamsId, static_cast<int32_t>(_volumeBestSim_dmp->getDimensionSizes().value().depth), scaleStep, thicknessMultFactor, maxSimilarity, depthRange, downscaledRoi};

    VulkanCommandManager::getInstance(_deviceID)
        ->wait()
        ->reset()
        ->begin()
#if TSIM_USE_FLOAT
        ->shader(ALICEVISION_LOOKUP_SPIRV_MODULE(RetrieveBestDepth_float))
#else
        ->shader(ALICEVISION_LOOKUP_SPIRV_MODULE(RetrieveBestDepth_uchar))
#endif
        ->pushConstant(pc)
        ->bind(*_depthThicknessMap_dmp, vk::DescriptorType::eStorageImage)
        ->bind(*_depthSimMap_dmp, vk::DescriptorType::eStorageImage)
        ->bind(*_depths_dmp, vk::DescriptorType::eStorageImage)
        ->bind(*_volumeBestSim_dmp, vk::DescriptorType::eStorageImage)
        ->bind(DeviceCameraParamsArray::getInstance(_deviceID)->getCameraParamsArray(), vk::DescriptorType::eUniformBuffer)
        ->transferImageLayout(*_depthThicknessMap_dmp, vk::ImageLayout::eGeneral, 0, 1)
        ->transferImageLayout(*_depthSimMap_dmp, vk::ImageLayout::eGeneral, 0, 1)
        ->transferImageLayout(*_depths_dmp, vk::ImageLayout::eGeneral, 0, 1)
        ->transferImageLayout(*_volumeBestSim_dmp, vk::ImageLayout::eGeneral, 0, 1)
        ->workgroups(vk::Extent3D(divUp(downscaledRoi.width(), 32), divUp(downscaledRoi.height(), 32), 1))
        ->dispatch()
        ->end()
        ->submit()
        ->wait();

    ALICEVISION_LOG_INFO(tile << "SGM Retrieve best depth in volume done.");
}

void Sgm::exportVolumeInformation(const Tile& tile,
                                  const SgmDepthList& tileDepthList,
                                  const VulkanMemoryBase<TSim>& in_volume_dmp,
                                  const std::string& name) const
{
    if (!_sgmParams.exportIntermediateVolumes && !_sgmParams.exportIntermediateCrossVolumes && !_sgmParams.exportIntermediateVolume9pCsv)
    {
        // nothing to do
        return;
    }

    // get file tile begin indexes (default is single tile)
    int tileBeginX = -1;
    int tileBeginY = -1;

    if (tile.nbTiles > 1)
    {
        tileBeginX = tile.roi.x.begin;
        tileBeginY = tile.roi.y.begin;
    }

    if (_sgmParams.exportIntermediateVolumes)
    {
        ALICEVISION_LOG_INFO(tile << "Export similarity volume (" << name << ").");

        const std::string volumePath = getFileNameFromIndex(_mp, tile.rc, mvsUtils::EFileType::volume, "_" + name, tileBeginX, tileBeginY);

        exportSimilarityVolume(in_volume_dmp, tileDepthList.getDepths(), _mp, tile.rc, _sgmParams, volumePath, tile.roi);

        ALICEVISION_LOG_INFO(tile << "Export similarity volume (" << name << ") done.");
    }

    if (_sgmParams.exportIntermediateCrossVolumes)
    {
        ALICEVISION_LOG_INFO(tile << "Export similarity volume cross (" << name << ").");

        const std::string volumeCrossPath = getFileNameFromIndex(_mp, tile.rc, mvsUtils::EFileType::volumeCross, "_" + name, tileBeginX, tileBeginY);

        exportSimilarityVolumeCross(in_volume_dmp, tileDepthList.getDepths(), _mp, tile.rc, _sgmParams, volumeCrossPath, tile.roi);

        ALICEVISION_LOG_INFO(tile << "Export similarity volume cross (" << name << ") done.");
    }

    if (_sgmParams.exportIntermediateTopographicCutVolumes)
    {
        ALICEVISION_LOG_INFO(tile << "Export similarity volume topographic cut (" << name << ").");

        const std::string volumeCutPath =
          getFileNameFromIndex(_mp, tile.rc, mvsUtils::EFileType::volumeTopographicCut, "_" + name, tileBeginX, tileBeginY);

        exportSimilarityVolumeTopographicCut(in_volume_dmp, tileDepthList.getDepths(), _mp, tile.rc, _sgmParams, volumeCutPath, tile.roi);

        ALICEVISION_LOG_INFO(tile << "Export similarity volume topographic cut (" << name << ") done.");
    }

    if (_sgmParams.exportIntermediateVolume9pCsv)
    {
        ALICEVISION_LOG_INFO(tile << "Export similarity volume 9 points CSV (" << name << ").");

        const std::string stats9Path = getFileNameFromIndex(_mp, tile.rc, mvsUtils::EFileType::stats9p, "_sgm", tileBeginX, tileBeginY);

        exportSimilaritySamplesCSV(in_volume_dmp, tileDepthList.getDepths(), name, _sgmParams, stats9Path, tile.roi);

        ALICEVISION_LOG_INFO(tile << "Export similarity volume 9 points CSV (" << name << ") done.");
    }
}

#elif ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_METAL)

using namespace gpu;

Sgm::Sgm(const mvsUtils::MultiViewParams& mp,
         const mvsUtils::TileParams& tileParams,
         const SgmParams& sgmParams,
         const bool computeDepthSimMap,
         const bool computeNormalMap,
         const uint64_t deviceID)
  : _mp(mp),
    _tileParams(tileParams),
    _sgmParams(sgmParams),
    _computeDepthSimMap(computeDepthSimMap || sgmParams.exportIntermediateDepthSimMaps),
    _computeNormalMap(computeNormalMap || sgmParams.exportIntermediateNormalMaps),
    _deviceID(deviceID)
{
    // Get Resource Manager for deviceID
    MTLResourceManager* resMng = MTLDeviceManager::getInstance()->getResourceManager(deviceID);

    // get tile maximum dimensions
    const int downscale = _sgmParams.scale * _sgmParams.stepXY;
    const int maxTileWidth = divideRoundUp(tileParams.bufferWidth, downscale);
    const int maxTileHeight = divideRoundUp(tileParams.bufferHeight, downscale);

    // compute map maximum dimensions
    const MTL::Size mapDim(maxTileWidth, maxTileHeight, 1);

    // allocate depth list in device memory
    {
        const MTL::Size depthsDim(_sgmParams.maxDepths, 1, 1);
        _depths_dmp = resMng->createTexture2D<float>(MTL::PixelFormatR32Float, depthsDim.width, depthsDim.height, 1, false);
    }

    // allocate depth thickness map in device memory
    // TODO: Check if RGBA is fine, RG32SFloat is not rw!
    _depthThicknessMap_dmp = resMng->createTexture2D<float4>(MTL::PixelFormatRGBA32Float, mapDim.width, mapDim.height, 1, false);

    // allocate depth/sim map in device memory
    if (_computeDepthSimMap) // TODO: Check if RGBA is fine, RG32SFloat is not rw!
        _depthSimMap_dmp = resMng->createTexture2D<float4>(MTL::PixelFormatRGBA32Float, mapDim.width, mapDim.height, 1, false);
    else
        _depthSimMap_dmp_DUMMY = resMng->createTexture2D<float4>(MTL::PixelFormatRGBA32Float, 1, 1, 1, false);

    // allocate normal map in device memory
    if (_computeNormalMap)
        _normalMap_dmp = resMng->createTexture2D<float4>(MTL::PixelFormatRGBA32Float, mapDim.width, mapDim.height, 1, false);

    // allocate similarity volumes in device memory
    {
        const MTL::Size volDim(maxTileWidth, maxTileHeight, _sgmParams.maxDepths);

        _volumeBestSim_dmp = resMng->createTexture3D<TSim>(MTLTSimPixelFormat, volDim.width, volDim.height, volDim.depth, 1, false);
        _volumeSecBestSim_dmp = resMng->createTexture3D<TSim>(MTLTSimPixelFormat, volDim.width, volDim.height, volDim.depth, 1, false);
    }

    // allocate similarity volume optimization buffers
    if (sgmParams.doSgmOptimizeVolume)
    {
        const size_t maxTileSide = std::max(maxTileWidth, maxTileHeight);
        _volumeSliceAccA_dmp = resMng->createTexture2D<TSimAcc>(MTLTSimAccPixelFormat, maxTileSide, _sgmParams.maxDepths, 1, false);
        _volumeSliceAccB_dmp = resMng->createTexture2D<TSimAcc>(MTLTSimAccPixelFormat, maxTileSide, _sgmParams.maxDepths, 1, false);
        _volumeAxisAcc_dmp = resMng->createTexture2D<TSimAcc>(MTLTSimAccPixelFormat, maxTileSide, 1, 1, false);
    }
}

double Sgm::getDeviceMemoryConsumption() const
{
    size_t bytes = 0;

    bytes += _depths_dmp.getBytesPadded();
    bytes += _depthThicknessMap_dmp.getBytesPadded();
    if (_computeDepthSimMap)
        bytes += _depthSimMap_dmp.getBytesPadded();
    if (_computeNormalMap)
        bytes += _normalMap_dmp.getBytesPadded();
    bytes += _volumeBestSim_dmp.getBytesPadded();
    bytes += _volumeSecBestSim_dmp.getBytesPadded();
    if (_sgmParams.doSgmOptimizeVolume)
        bytes += _volumeSliceAccA_dmp.getBytesPadded();
    if (_sgmParams.doSgmOptimizeVolume)
        bytes += _volumeSliceAccB_dmp.getBytesPadded();
    if (_sgmParams.doSgmOptimizeVolume)
        bytes += _volumeAxisAcc_dmp.getBytesPadded();

    return (double(bytes) / (1024.0 * 1024.0));
}

double Sgm::getDeviceMemoryConsumptionUnpadded() const
{
    size_t bytes = 0;

    bytes += _depths_dmp.getBytesUnpadded();
    bytes += _depthThicknessMap_dmp.getBytesUnpadded();
    if (_computeDepthSimMap)
        bytes += _depthSimMap_dmp.getBytesUnpadded();
    if (_computeNormalMap)
        bytes += _normalMap_dmp.getBytesUnpadded();
    bytes += _volumeBestSim_dmp.getBytesUnpadded();
    bytes += _volumeSecBestSim_dmp.getBytesUnpadded();
    if (_sgmParams.doSgmOptimizeVolume)
        bytes += _volumeSliceAccA_dmp.getBytesUnpadded();
    if (_sgmParams.doSgmOptimizeVolume)
        bytes += _volumeSliceAccB_dmp.getBytesUnpadded();
    if (_sgmParams.doSgmOptimizeVolume)
        bytes += _volumeAxisAcc_dmp.getBytesUnpadded();

    return (double(bytes) / (1024.0 * 1024.0));
}

void Sgm::sgmRc(const Tile& tile, const SgmDepthList& tileDepthList)
{
    const IndexT viewId = _mp.getViewId(tile.rc);

    ALICEVISION_LOG_INFO(tile << "SGM depth/thickness map of view id: " << viewId << ", rc: " << tile.rc << " (" << (tile.rc + 1) << " / "
                              << _mp.ncams << ").");

    // check SGM depth list and T cameras
    if (tile.sgmTCams.empty() || tileDepthList.getDepths().empty())
        ALICEVISION_THROW_ERROR(tile << "Cannot compute Semi-Global Matching, no depths or no T cameras (viewId: " << viewId << ").");

    // copy rc depth data in device memory
    _depths_dmp.copyFromHost(tileDepthList.getDepths().data(), tileDepthList.getDepths().size(), 0, 0);

    // compute best sim and second best sim volumes
    computeSimilarityVolumes(tile, tileDepthList);

    // export intermediate volume information (if requested by user)
    exportVolumeInformation(tile, tileDepthList, _volumeSecBestSim_dmp, "beforeFiltering");

    // this is here for experimental purposes
    // to show how SGGC work on non optimized depthmaps
    // it must equals to true in normal case
    if (_sgmParams.doSgmOptimizeVolume)
    {
        optimizeSimilarityVolume(tile, tileDepthList);
    }
    else
    {
        // best sim volume is normally reuse to put optimized similarity
        _volumeBestSim_dmp.copyFromResource(_volumeSecBestSim_dmp, 0, 0);
    }

    // export intermediate volume information (if requested by user)
    exportVolumeInformation(tile, tileDepthList, _volumeBestSim_dmp, "afterFiltering");

    // retrieve best depth
    retrieveBestDepth(tile, tileDepthList);

    // export intermediate depth/sim map (if requested by user)
    if (_sgmParams.exportIntermediateDepthSimMaps)
    {
        writeDepthSimMap(tile.rc, _mp, _tileParams, tile.roi, _depthSimMap_dmp, _sgmParams.scale, _sgmParams.stepXY, "sgm");
    }

    // compute normal map from depth/sim map if needed
    if (_computeNormalMap)
    {
        // downscale the region of interest
        const ROI downscaledRoi = downscaleROI(tile.roi, _sgmParams.scale * _sgmParams.stepXY);

        // get R device camera parameters id from cache
        DeviceCache& deviceCache = DeviceCache::getInstance();
        const int rcDeviceCameraParamsId = deviceCache.requestCameraParamsId(_deviceID, tile.rc, _sgmParams.scale, _mp);

        ALICEVISION_LOG_INFO(tile << "SGM compute normal map of view id: " << viewId << ", rc: " << tile.rc << " (" << (tile.rc + 1) << " / "
                                  << _mp.ncams << ").");

        // Get command manager
        const auto cmdMng = MTLDeviceManager::getInstance()->getCommandManager(_deviceID);

        // Get device camera params
        const auto deviceCameraParamsWrapper = DeviceCache::getInstance().getDeviceCameraParamsArrayWrapper(_deviceID);
        const auto& deviceCameraParams = deviceCameraParamsWrapper->getDeviceCameraParamsBuffer();

        cmdMng
        ->loadLibrary("AVDepthMapMetalKernels", "org.aliceVision.AVDepthMap")
        ->reset()
        ->commandBuffer()
        ->pipeline("aliceVision::depthMap::DepthSimMapComputeNormal", "AVDepthMapMetalKernels")
        ->commandEncoder()
        ->bind(_normalMap_dmp, 0)
        ->bind(_depthSimMap_dmp, 1)
        ->bind(deviceCameraParams, 1)
        ->pushConstants(DepthSimMapComputeNormal_PushConstants{rcDeviceCameraParamsId, _sgmParams.stepXY, 3, downscaledRoi})
        ->dispatchDimensions(MTL::Size(downscaledRoi.width(), downscaledRoi.height(), 1), MTL::Size(8, 8, 1))
        ->endRecording()
        ->commitCommands()
        ->waitAll();

        // export intermediate normal map (if requested by user)
        if (_sgmParams.exportIntermediateNormalMaps)
        {
            writeNormalMap(tile.rc, _mp, _tileParams, tile.roi, _normalMap_dmp, _sgmParams.scale, _sgmParams.stepXY, "sgm");
        }
    }

    ALICEVISION_LOG_INFO(tile << "SGM depth/thickness map done.");
}

void Sgm::smoothThicknessMap(const Tile& tile, const RefineParams& refineParams)
{
    ALICEVISION_LOG_INFO(tile << "SGM Smooth thickness map.");

    // downscale the region of interest
    const ROI downscaledRoi = downscaleROI(tile.roi, _sgmParams.scale * _sgmParams.stepXY);

    const int sgmScaleStep = _sgmParams.scale * _sgmParams.stepXY;
    const int refineScaleStep = refineParams.scale * refineParams.stepXY;

    // min/max number of Refine samples in SGM thickness area
    const float minNbRefineSamples = 2.f;
    const float maxNbRefineSamples = max(sgmScaleStep / float(refineScaleStep), minNbRefineSamples);

    // min/max SGM thickness inflate factor
    const float minThicknessInflate = refineParams.halfNbDepths / maxNbRefineSamples;
    const float maxThicknessInflate = refineParams.halfNbDepths / minNbRefineSamples;

    // Get command manager
    const auto cmdMng = MTLDeviceManager::getInstance()->getCommandManager(_deviceID);

    cmdMng
    ->loadLibrary("AVDepthMapMetalKernels", "org.aliceVision.AVDepthMap")
    ->reset()
    ->commandBuffer()
    ->pipeline("aliceVision::depthMap::DepthThicknessSmoothThickness", "AVDepthMapMetalKernels")
    ->commandEncoder()
    ->bind(_depthThicknessMap_dmp, 0)
    ->pushConstants(DepthThicknessSmoothThickness_PushConstants{minThicknessInflate, maxThicknessInflate, downscaledRoi})
    ->dispatchDimensions(MTL::Size(downscaledRoi.width(), downscaledRoi.height(), 1), MTL::Size(8, 8, 1))
    ->endRecording()
    ->commitCommands()
    ->waitAll();

    ALICEVISION_LOG_INFO(tile << "SGM Smooth thickness map done.");
}

void Sgm::computeSimilarityVolumes(const Tile& tile, const SgmDepthList& tileDepthList)
{
    ALICEVISION_LOG_INFO(tile << "SGM Compute similarity volume.");

    // downscale the region of interest
    const ROI downscaledRoi = downscaleROI(tile.roi, _sgmParams.scale * _sgmParams.stepXY);

    // initialize the two similarity volumes at 255
    const MTL::Size volumeBestSim_dmp_size = _volumeBestSim_dmp.getTextureDimensions();
    const MTL::Size _volumeSecBestSim_dmp_size = _volumeBestSim_dmp.getTextureDimensions();
    std::vector<TSim> hostData;
    hostData.resize(volumeBestSim_dmp_size.width * volumeBestSim_dmp_size.height * volumeBestSim_dmp_size.depth);
    std::ranges::fill(hostData, 255.f);
    _volumeBestSim_dmp.copyFromHost(hostData.data(), hostData.size(), 0, 0);
    hostData.resize(_volumeSecBestSim_dmp_size.width * _volumeSecBestSim_dmp_size.height * _volumeSecBestSim_dmp_size.depth);
    std::ranges::fill(hostData, 255.f);
    _volumeSecBestSim_dmp.copyFromHost(hostData.data(), hostData.size(), 0, 0);

    // get device cache instance
    DeviceCache& deviceCache = DeviceCache::getInstance();

    // get R device camera parameters id from cache
    const int rcDeviceCameraParamsId = deviceCache.requestCameraParamsId(_deviceID, tile.rc, _sgmParams.scale, _mp);

    // get R device mipmap image from cache
    const DeviceMipmapImage& rcDeviceMipmapImage = deviceCache.requestMipmapImage(_deviceID, tile.rc, _mp);
    
    // Get command manager
    MTLCommandManager* cmdMng = MTLDeviceManager::getInstance()->getCommandManager(_deviceID);

    // Get DeviceCameraParams
    const auto deviceCameraParamsWrapper = deviceCache.getDeviceCameraParamsArrayWrapper(_deviceID);
    const auto& deviceCameraParams = deviceCameraParamsWrapper->getDeviceCameraParamsBuffer();

    // Get device patch pattern
    const auto devicePatchPatternWrapper = deviceCache.getDevicePatchPatternArrayWrapper(_deviceID);
    const auto& devicePatchPattern = devicePatchPatternWrapper->getDevicePatchPatternBuffer();

    // compute similarity volume per Rc Tc
    for (std::size_t tci = 0; tci < tile.sgmTCams.size(); ++tci)
    {
        const int tc = tile.sgmTCams.at(tci);

        const int firstDepth = tileDepthList.getDepthsTcLimits()[tci].x;
        const int lastDepth = firstDepth + tileDepthList.getDepthsTcLimits()[tci].y;

        const Range tcDepthRange(firstDepth, lastDepth);

        // get T device camera parameters id from cache
        const int tcDeviceCameraParamsId = deviceCache.requestCameraParamsId(_deviceID, tc, _sgmParams.scale, _mp);

        // get T device mipmap image from cache
        const DeviceMipmapImage& tcDeviceMipmapImage = deviceCache.requestMipmapImage(_deviceID, tc, _mp);

        ALICEVISION_LOG_DEBUG(tile << "Compute similarity volume:" << std::endl
                                   << "\t- rc: " << tile.rc << std::endl
                                   << "\t- tc: " << tc << " (" << (tci + 1) << "/" << tile.sgmTCams.size() << ")" << std::endl
                                   << "\t- rc camera parameters id: " << rcDeviceCameraParamsId << std::endl
                                   << "\t- tc camera parameters id: " << tcDeviceCameraParamsId << std::endl
                                   << "\t- tc first depth: " << firstDepth << std::endl
                                   << "\t- tc last depth: " << lastDepth << std::endl
                                   << "\t- tile range x: [" << downscaledRoi.x.begin << " - " << downscaledRoi.x.end << "]" << std::endl
                                   << "\t- tile range y: [" << downscaledRoi.y.begin << " - " << downscaledRoi.y.end << "]" << std::endl);

        const float rcMipmapLevel = rcDeviceMipmapImage.getLevel(_sgmParams.scale);
        const MTL::Size rcLevelDim = rcDeviceMipmapImage.getDimensions(_sgmParams.scale);
        const MTL::Size tcLevelDim = tcDeviceMipmapImage.getDimensions(_sgmParams.scale);

        cmdMng
        ->loadLibrary("AVDepthMapMetalKernels", "org.aliceVision.AVDepthMap")
        ->reset()
        ->commandBuffer()
        ->pipeline("aliceVision::depthMap::VolumeComputeSimiliarity", "AVDepthMapMetalKernels")
        ->commandEncoder()
        ->bind(_volumeBestSim_dmp, 0)
        ->bind(_volumeSecBestSim_dmp, 1)
        ->bind(_depths_dmp, 2)
        ->bind(rcDeviceMipmapImage.getTexture(), 3)
        ->bind(tcDeviceMipmapImage.getTexture(), 4)
        ->bind(deviceCameraParams, 1)
        ->bind(devicePatchPattern, 2)
        ->createSampler(0)
        ->createSampler(1)
        ->pushConstants(VolumeComputeSimiliarity_PushConstants{rcDeviceCameraParamsId, tcDeviceCameraParamsId, static_cast<unsigned int>(rcLevelDim.width), static_cast<unsigned int>(rcLevelDim.height), static_cast<unsigned int>(tcLevelDim.width), static_cast<unsigned int>(rcLevelDim.height), rcMipmapLevel, _sgmParams.stepXY, _sgmParams.wsh, (1.f / float(_sgmParams.gammaC)), (1.f / float(_sgmParams.gammaP)), _sgmParams.useConsistentScale, _sgmParams.useCustomPatchPattern, tcDepthRange, downscaledRoi})
        ->dispatchDimensions(MTL::Size(downscaledRoi.width(), downscaledRoi.height(), tcDepthRange.size()), MTL::Size(32, 1, 1))
        ->endRecording()
        ->commitCommands()
        ->waitAll();
    }

    // update second best uninitialized similarity volume values with first best similarity volume values
    // - allows to avoid the particular case with a single tc (second best volume has no valid similarity values)
    // - useful if a tc alone contributes to the calculation of a subpart of the similarity volume
    if (_sgmParams.updateUninitializedSim)  // should always be true, false for debug purposes
    {
        ALICEVISION_LOG_DEBUG(tile << "SGM Update uninitialized similarity volume values from best similarity volume.");

        assert(_volumeBestSim_dmp.getTextureDimensions().width == _volumeSecBestSim_dmp.getTextureDimensions().width);
        assert(_volumeBestSim_dmp.getTextureDimensions().height == _volumeSecBestSim_dmp.getTextureDimensions().height);
        assert(_volumeBestSim_dmp.getTextureDimensions().depth == _volumeSecBestSim_dmp.getTextureDimensions().depth);

        // get input/output volume dimensions
        const MTL::Size volDim = _volumeSecBestSim_dmp.getTextureDimensions();

        cmdMng
        ->loadLibrary("AVDepthMapMetalKernels", "org.aliceVision.AVDepthMap")
        ->reset()
        ->commandBuffer()
        ->pipeline("aliceVision::depthMap::VolumeUpdateUnitialized", "AVDepthMapMetalKernels")
        ->commandEncoder()
        ->bind(_volumeSecBestSim_dmp, 0)
        ->bind(_volumeBestSim_dmp, 1)
        ->dispatchDimensions(MTL::Size(volDim.width, volDim.height, volDim.depth), MTL::Size(32, 1, 1))
        ->endRecording()
        ->commitCommands()
        ->waitAll();
    }

    ALICEVISION_LOG_INFO(tile << "SGM Compute similarity volume done.");
}

void Sgm::optimizeSimilarityVolume(const Tile& tile, const SgmDepthList& tileDepthList)
{
    ALICEVISION_LOG_INFO(tile << "SGM Optimizing volume (filtering axes: " << _sgmParams.filteringAxes << ").");

    // downscale the region of interest
    const ROI downscaledRoi = downscaleROI(tile.roi, _sgmParams.scale * _sgmParams.stepXY);

    // get R device mipmap image from cache
    DeviceCache& deviceCache = DeviceCache::getInstance();
    const DeviceMipmapImage& rcDeviceMipmapImage = deviceCache.requestMipmapImage(_deviceID, tile.rc, _mp);

    // Get command manager
    MTLCommandManager* cmdMng = MTLDeviceManager::getInstance()->getCommandManager(_deviceID);

    // get R mipmap image level and dimensions
    const float rcMipmapLevel = rcDeviceMipmapImage.getLevel(_sgmParams.scale);
    const MTL::Size rcLevelDim = rcDeviceMipmapImage.getDimensions(_sgmParams.scale);

    // update aggregation volume
    int npaths = 0;
    const auto updateAggrVolume = [&](const int3& axisT, bool invX)
    {

        int3 volDim;
        const auto volDimMTL = _volumeSecBestSim_dmp.getTextureDimensions();
        volDim.x = volDimMTL.width;
        volDim.y = volDimMTL.height;
        volDim.z = tileDepthList.getDepths().size(); // override volume depth, use rc depth list last index

        const size_t volDimX = volDim[axisT.x];
        const size_t volDimY = volDim[axisT.y];
        const size_t volDimZ = volDim[axisT.z];

        const int3 volDim_ = int3{static_cast<int>(volDim[0]), static_cast<int>(volDim[1]), static_cast<int>(volDim[2])};
        const int3 axisT_ = int3{static_cast<int>(axisT[0]), static_cast<int>(axisT[1]), static_cast<int>(axisT[2])};
        const int ySign = (invX ? -1 : 1);

        // setup block and grid
        constexpr int blockSize = 8;
        const MTL::Size blockVolXZ(blockSize, blockSize, 1);
        const MTL::Size gridVolXZ(volDimX, volDimZ, 1);

        constexpr int blockSizeL = 64;
        const MTL::Size blockColZ(blockSizeL, 1, 1);
        const MTL::Size gridColZ(volDimX, 1, 1);

        const MTL::Size blockVolSlide(blockSizeL, 1, 1);
        const MTL::Size gridVolSlide(volDimX, volDimZ, 1);

        MTLSharedResource<MTLTexture, TSimAcc>* xzSliceForY_dmpPtr   = &_volumeSliceAccA_dmp; // Y slice
        MTLSharedResource<MTLTexture, TSimAcc>* xzSliceForYm1_dmpPtr = &_volumeSliceAccB_dmp; // Y-1 slice
        MTLSharedResource<MTLTexture, TSimAcc>* bestSimInYm1_dmpPtr  = &_volumeAxisAcc_dmp;   // best sim score along the Y axis for each Z value

        cmdMng
        ->loadLibrary("AVDepthMapMetalKernels", "org.aliceVision.AVDepthMap")
        ->reset()
        ->commandBuffer()
        ->pipeline("aliceVision::depthMap::GetVolumeXZSlice", "AVDepthMapMetalKernels")
        ->commandEncoder()
        ->bind(*xzSliceForYm1_dmpPtr, 0)
        ->bind(_volumeSecBestSim_dmp, 1)
        ->pushConstants(GetVolumeXZSlice_PushConstants{axisT, volDim, 0})
        ->dispatchDimensions(gridVolXZ, blockVolXZ)
        ->endRecording()
        ->commitCommands()
        ->waitAll();

        cmdMng
        ->loadLibrary("AVDepthMapMetalKernels", "org.aliceVision.AVDepthMap")
        ->reset()
        ->commandBuffer()
        ->pipeline("aliceVision::depthMap::InitVolumeYSlice", "AVDepthMapMetalKernels")
        ->commandEncoder()
        ->bind(_volumeBestSim_dmp, 0)
        ->pushConstants(InitVolumeYSlice_PushConstants{axisT, volDim, 0, 255})
        ->dispatchDimensions(gridVolXZ, blockVolXZ)
        ->endRecording()
        ->commitCommands()
        ->waitAll();

        for(int iy = 1; iy < volDimY; ++iy)
        {
            const int y = invX ? volDimY - 1 - iy : iy;

            // For each column: compute the best score
            // Foreach x:
            //   bestSimInYm1[x] = min(d_xzSliceForY[1:height])
            cmdMng
            ->loadLibrary("AVDepthMapMetalKernels", "org.aliceVision.AVDepthMap")
            ->reset()
            ->commandBuffer()
            ->pipeline("aliceVision::depthMap::ComputeBestZInSlice", "AVDepthMapMetalKernels")
            ->commandEncoder()
            ->bind(*xzSliceForYm1_dmpPtr, 0)
            ->bind(*bestSimInYm1_dmpPtr, 1)
            ->pushConstants(ComputeBestZInSlice_PushConstants{static_cast<int>(volDimX), static_cast<int>(volDimZ)})
            ->dispatchDimensions(gridColZ, blockColZ)
            ->endRecording()
            ->commitCommands()
            ->waitAll();

            // Copy the 'z' plane from 'in_volSim_dmp' into 'xzSliceForY'
            cmdMng
            ->loadLibrary("AVDepthMapMetalKernels", "org.aliceVision.AVDepthMap")
            ->reset()
            ->commandBuffer()
            ->pipeline("aliceVision::depthMap::GetVolumeXZSlice", "AVDepthMapMetalKernels")
            ->commandEncoder()
            ->bind(*xzSliceForY_dmpPtr, 0)
            ->bind(_volumeSecBestSim_dmp, 1)
            ->pushConstants(GetVolumeXZSlice_PushConstants{axisT_, volDim_, y})
            ->dispatchDimensions(gridColZ, blockColZ)
            ->endRecording()
            ->commitCommands()
            ->waitAll();

            cmdMng
            ->loadLibrary("AVDepthMapMetalKernels", "org.aliceVision.AVDepthMap")
            ->reset()
            ->commandBuffer()
            ->pipeline("aliceVision::depthMap::AggregateConstVolumeAtXInSlices", "AVDepthMapMetalKernels")
            ->commandEncoder()
            ->bind(rcDeviceMipmapImage.getTexture(), 0)
            ->createSampler(0)
            ->bind(*xzSliceForY_dmpPtr, 1)
            ->bind(*xzSliceForYm1_dmpPtr, 2)
            ->bind(*bestSimInYm1_dmpPtr, 3)
            ->bind(_volumeBestSim_dmp, 4)
            ->pushConstants(AggregateCostVolumeAtXInSlices_PushConstants{static_cast<unsigned int>(rcLevelDim.width), static_cast<unsigned int>(rcLevelDim.height), rcMipmapLevel, volDim_, axisT_, static_cast<float>(_sgmParams.stepXY), y, static_cast<float>(_sgmParams.p1), static_cast<float>(_sgmParams.p2Weighting), ySign, npaths, downscaledRoi})
            ->dispatchDimensions(gridVolSlide, blockVolSlide)
            ->endRecording()
            ->commitCommands()
            ->waitAll();

            std::swap(xzSliceForYm1_dmpPtr, xzSliceForY_dmpPtr);
        }

        npaths++;
    };

    // filtering is done on the last axis
    const std::map<char, int3> mapAxes = {
        std::make_pair<char, int3>('X', int3{1, 0, 2}), // XYZ -> YXZ
        std::make_pair<char, int3>('Y', int3{0, 1, 2}), // XYZ
    };

    for(char axis : _sgmParams.filteringAxes)
    {
        const int3& axisT = mapAxes.at(axis);
        updateAggrVolume(axisT, false); // without transpose
        updateAggrVolume(axisT, true);  // with transpose of the last axis
    }

    ALICEVISION_LOG_INFO(tile << "SGM Optimizing volume done.");
}

void Sgm::retrieveBestDepth(const Tile& tile, const SgmDepthList& tileDepthList)
{
    ALICEVISION_LOG_INFO(tile << "SGM Retrieve best depth in volume.");

    // downscale the region of interest
    const ROI downscaledRoi = downscaleROI(tile.roi, _sgmParams.scale * _sgmParams.stepXY);

    // get depth range
    const Range depthRange(0, tileDepthList.getDepths().size());

    // get R device camera parameters id from cache
    DeviceCache& deviceCache = DeviceCache::getInstance();
    const int rcDeviceCameraParamsId = deviceCache.requestCameraParamsId(_deviceID, tile.rc, 1, _mp);

    // Get command manager
    MTLCommandManager* cmdMng = MTLDeviceManager::getInstance()->getCommandManager(_deviceID);

    // Get Device Camera Params
    const auto deviceCameraParamsWrapper = deviceCache.getDeviceCameraParamsArrayWrapper(_deviceID);
    const auto& deviceCameraParams = deviceCameraParamsWrapper->getDeviceCameraParamsBuffer();

    // constant kernel inputs
    const int scaleStep = _sgmParams.scale * _sgmParams.stepXY;
    const float thicknessMultFactor = 1.f + float(_sgmParams.depthThicknessInflate);
    const float maxSimilarity = float(_sgmParams.maxSimilarity) * 254.f; // convert from (0, 1) to (0, 254)

    // kernel launch parameters
    const MTL::Size block(32, 1, 1);
    const MTL::Size grid(downscaledRoi.width(), downscaledRoi.height(), 1);

    // Depth Sim Map pointer
    MTLSharedResource<MTLTexture, float4>* depthSimMap_dmp_ptr;
    if (_computeDepthSimMap)
        depthSimMap_dmp_ptr = &this->_depthSimMap_dmp;
    else
        depthSimMap_dmp_ptr = &this->_depthSimMap_dmp_DUMMY;

    cmdMng
    ->loadLibrary("AVDepthMapMetalKernels", "org.aliceVision.AVDepthMap")
    ->reset()
    ->commandBuffer()
    ->pipeline("aliceVision::depthMap::RetrieveBestDepth", "AVDepthMapMetalKernels")
    ->commandEncoder()
    ->bind(_depthThicknessMap_dmp, 0)
    ->bind(*depthSimMap_dmp_ptr, 1)
    ->bind(_depths_dmp, 2)
    ->bind(_volumeBestSim_dmp, 3)
    ->bind(deviceCameraParams, 1)
    ->pushConstants(RetrieveBestDepth_PushConstants{_computeDepthSimMap, rcDeviceCameraParamsId, static_cast<int>(_volumeBestSim_dmp.getTextureDimensions().depth), scaleStep, thicknessMultFactor, maxSimilarity, depthRange, downscaledRoi})
    ->dispatchDimensions(grid, block)
    ->endRecording()
    ->commitCommands()
    ->waitAll();

    ALICEVISION_LOG_INFO(tile << "SGM Retrieve best depth in volume done.");
}

void Sgm::exportVolumeInformation(const Tile& tile,
                                  const SgmDepthList& tileDepthList,
                                  const MTLSharedResource<MTLTexture, TSim>& in_volume_dmp,
                                  const std::string& name) const
{
    if (!_sgmParams.exportIntermediateVolumes && !_sgmParams.exportIntermediateCrossVolumes && !_sgmParams.exportIntermediateVolume9pCsv)
    {
        // nothing to do
        return;
    }

    // get file tile begin indexes (default is single tile)
    int tileBeginX = -1;
    int tileBeginY = -1;

    if (tile.nbTiles > 1)
    {
        tileBeginX = tile.roi.x.begin;
        tileBeginY = tile.roi.y.begin;
    }

    // copy device similarity volume to host memory
    // CudaHostMemoryHeap<TSim, 3> volumeSim_hmh(in_volume_dmp.getSize());
    // volumeSim_hmh.copyFrom(in_volume_dmp);

    if (_sgmParams.exportIntermediateVolumes)
    {
        ALICEVISION_LOG_INFO(tile << "Export similarity volume (" << name << ").");

        const std::string volumePath = getFileNameFromIndex(_mp, tile.rc, mvsUtils::EFileType::volume, "_" + name, tileBeginX, tileBeginY);

        exportSimilarityVolume(in_volume_dmp, tileDepthList.getDepths(), _mp, tile.rc, _sgmParams, volumePath, tile.roi);

        ALICEVISION_LOG_INFO(tile << "Export similarity volume (" << name << ") done.");
    }

    if (_sgmParams.exportIntermediateCrossVolumes)
    {
        ALICEVISION_LOG_INFO(tile << "Export similarity volume cross (" << name << ").");

        const std::string volumeCrossPath = getFileNameFromIndex(_mp, tile.rc, mvsUtils::EFileType::volumeCross, "_" + name, tileBeginX, tileBeginY);

        exportSimilarityVolumeCross(in_volume_dmp, tileDepthList.getDepths(), _mp, tile.rc, _sgmParams, volumeCrossPath, tile.roi);

        ALICEVISION_LOG_INFO(tile << "Export similarity volume cross (" << name << ") done.");
    }

    if (_sgmParams.exportIntermediateTopographicCutVolumes)
    {
        ALICEVISION_LOG_INFO(tile << "Export similarity volume topographic cut (" << name << ").");

        const std::string volumeCutPath =
          getFileNameFromIndex(_mp, tile.rc, mvsUtils::EFileType::volumeTopographicCut, "_" + name, tileBeginX, tileBeginY);

        exportSimilarityVolumeTopographicCut(in_volume_dmp, tileDepthList.getDepths(), _mp, tile.rc, _sgmParams, volumeCutPath, tile.roi);

        ALICEVISION_LOG_INFO(tile << "Export similarity volume topographic cut (" << name << ") done.");
    }

    if (_sgmParams.exportIntermediateVolume9pCsv)
    {
        ALICEVISION_LOG_INFO(tile << "Export similarity volume 9 points CSV (" << name << ").");

        const std::string stats9Path = getFileNameFromIndex(_mp, tile.rc, mvsUtils::EFileType::stats9p, "_sgm", tileBeginX, tileBeginY);

        exportSimilaritySamplesCSV(in_volume_dmp, tileDepthList.getDepths(), name, _sgmParams, stats9Path, tile.roi);

        ALICEVISION_LOG_INFO(tile << "Export similarity volume 9 points CSV (" << name << ") done.");
    }
}

#else
    #error No backend for DepthMap computations selected!
#endif

}  // namespace depthMap
}  // namespace aliceVision

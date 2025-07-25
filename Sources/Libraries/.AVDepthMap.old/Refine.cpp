// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <AV/omp.hpp>
#include <AVDepthMap/DepthMapUtils.hpp>
#include <AVDepthMap/Refine.hpp>
#include <AVDepthMap/VolumeIO.hpp>
#include <AVMVSData/Point2d.hpp>
#include <AVMVSData/Point3d.hpp>
#include <AVMVSUtils/fileIO.hpp>
#include <AVSystem/Logger.hpp>

#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_CUDA)
    #include <AVDepthMap/cuda/host/DeviceCache.hpp>
    #include <AVDepthMap/cuda/planeSweeping/deviceDepthSimilarityMap.hpp>
    #include <AVDepthMap/cuda/planeSweeping/deviceSimilarityVolume.hpp>
#elif ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_VULKAN)
    #include <AVDepthMap/Vulkan/DeviceCache.hpp>
    #include <AVDepthMap/Vulkan/DevicePatchPattern.hpp>
    #include <AVDepthMap/Vulkan/DivUp.hpp>
    #include <AVDepthMapVulkanKernels/AVDepthMapVulkanKernels.hpp>
#elif ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_METAL)
    #include <AVDepthMap/Metal/DeviceCache.hpp>
    #include <AVDepthMap/Metal/DevicePatchPattern.hpp>
    #include <AVGPU/Metal/device.hpp>
    #include <AVGPU/Metal/command.hpp>
    #include <AVDepthMap/Metal/ComputeSgmUpscaledDepthPixSizeMap_Bilinear_PushConstants.hpp>
    #include <AVDepthMap/Metal/DepthSimMapComputeNormal_PushConstants.hpp>
    #include <AVDepthMap/Metal/DepthSimMapCopyDepthOnly_PushConstants.hpp>
    #include <AVDepthMap/Metal/MapUpscale_PushConstants.hpp>
    #include <AVDepthMap/Metal/OptimizeDepthSimMap_PushConstants.hpp>
    #include <AVDepthMap/Metal/OptimizeGetOptDepthMapFromOptDepthSimMap_PushConstants.hpp>
    #include <AVDepthMap/Metal/OptimizeVarLofLABtoW_PushConstants.hpp>
    #include <AVDepthMap/Metal/RefineBestDepth_PushConstants.hpp>
    #include <AVDepthMap/Metal/VolumeRefineSimiliarity_PushConstants.hpp>
    #include <AVDepthMap/Metal/ComputeSgmUpscaledDepthPixSizeMap_NearestNeighbor_PushConstants.hpp>
#else
    #error No backend for DepthMap computations selected!
#endif

namespace aliceVision {
namespace depthMap {

#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_CUDA)

Refine::Refine(const mvsUtils::MultiViewParams& mp, const mvsUtils::TileParams& tileParams, const RefineParams& refineParams, cudaStream_t stream)
  : _mp(mp),
    _tileParams(tileParams),
    _refineParams(refineParams),
    _stream(stream)
{
    // get tile maximum dimensions
    const int downscale = _refineParams.scale * _refineParams.stepXY;
    const int maxTileWidth = divideRoundUp(tileParams.bufferWidth, downscale);
    const int maxTileHeight = divideRoundUp(tileParams.bufferHeight, downscale);

    // compute depth/sim map maximum dimensions
    const CudaSize<2> depthSimMapDim(maxTileWidth, maxTileHeight);

    // allocate depth/sim maps in device memory
    _sgmDepthPixSizeMap_dmp.allocate(depthSimMapDim);
    _refinedDepthSimMap_dmp.allocate(depthSimMapDim);
    _optimizedDepthSimMap_dmp.allocate(depthSimMapDim);

    // allocate SGM upscaled normal map in device memory
    if (_refineParams.useSgmNormalMap)
        _sgmNormalMap_dmp.allocate(depthSimMapDim);

    // allocate normal map in device memory
    if (_refineParams.exportIntermediateNormalMaps)
        _normalMap_dmp.allocate(depthSimMapDim);

    // compute volume maximum dimensions
    const int nbDepthsToRefine = _refineParams.halfNbDepths * 2 + 1;
    const CudaSize<3> volDim(maxTileWidth, maxTileHeight, nbDepthsToRefine);

    // allocate refine volume in device memory
    _volumeRefineSim_dmp.allocate(volDim);

    // allocate depth/sim map optimization buffers
    if (_refineParams.useColorOptimization)
    {
        _optTmpDepthMap_dmp.allocate(depthSimMapDim);
        _optImgVariance_dmp.allocate(depthSimMapDim);
    }
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

void Refine::refineRc(const Tile& tile,
                      const CudaDeviceMemoryPitched<float2, 2>& in_sgmDepthThicknessMap_dmp,
                      const CudaDeviceMemoryPitched<float3, 2>& in_sgmNormalMap_dmp)
{
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
        const int rcDeviceCameraParamsId = deviceCache.requestCameraParamsId(tile.rc, _refineParams.scale, _mp);

        // get R device mipmap image from cache
        const DeviceMipmapImage& rcDeviceMipmapImage = deviceCache.requestMipmapImage(tile.rc, _mp);

        // compute upscaled SGM depth/pixSize map
        // - upscale SGM depth/thickness map
        // - filter masked pixels (alpha)
        // - compute pixSize from SGM thickness
        cuda_computeSgmUpscaledDepthPixSizeMap(
          _sgmDepthPixSizeMap_dmp, in_sgmDepthThicknessMap_dmp, rcDeviceCameraParamsId, rcDeviceMipmapImage, _refineParams, downscaledRoi, _stream);

        // export intermediate depth/pixSize map (if requested by user)
        if (_refineParams.exportIntermediateDepthSimMaps)
            writeDepthPixSizeMap(
              tile.rc, _mp, _tileParams, tile.roi, _sgmDepthPixSizeMap_dmp, _refineParams.scale, _refineParams.stepXY, "sgmUpscaled");

        // upscale SGM normal map (if needed)
        if (_refineParams.useSgmNormalMap && in_sgmNormalMap_dmp.getBuffer() != nullptr)
        {
            cuda_normalMapUpscale(_sgmNormalMap_dmp, in_sgmNormalMap_dmp, downscaledRoi, _stream);
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
        cuda_depthSimMapCopyDepthOnly(_refinedDepthSimMap_dmp, _sgmDepthPixSizeMap_dmp, 1.0f, _stream);
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
        _optimizedDepthSimMap_dmp.copyFrom(_refinedDepthSimMap_dmp, _stream);
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
    cuda_volumeInitialize(_volumeRefineSim_dmp, TSimRefine(0.f), _stream);

    // get device cache instance
    DeviceCache& deviceCache = DeviceCache::getInstance();

    // get R device camera parameters id from cache
    const int rcDeviceCameraParamsId = deviceCache.requestCameraParamsId(tile.rc, _refineParams.scale, _mp);

    // get R device mipmap image from cache
    const DeviceMipmapImage& rcDeviceMipmapImage = deviceCache.requestMipmapImage(tile.rc, _mp);

    // compute for each RcTc each similarity value for each depth to refine
    // sum the inverted / filtered similarity value, best value is the HIGHEST
    for (std::size_t tci = 0; tci < tile.refineTCams.size(); ++tci)
    {
        const int tc = tile.refineTCams.at(tci);

        // get T device camera parameters id from cache
        const int tcDeviceCameraParamsId = deviceCache.requestCameraParamsId(tc, _refineParams.scale, _mp);

        // get T device mipmap image from cache
        const DeviceMipmapImage& tcDeviceMipmapImage = deviceCache.requestMipmapImage(tc, _mp);

        ALICEVISION_LOG_DEBUG(tile << "Refine similarity volume:" << std::endl
                                   << "\t- rc: " << tile.rc << std::endl
                                   << "\t- tc: " << tc << " (" << (tci + 1) << "/" << tile.refineTCams.size() << ")" << std::endl
                                   << "\t- rc camera parameters id: " << rcDeviceCameraParamsId << std::endl
                                   << "\t- tc camera parameters id: " << tcDeviceCameraParamsId << std::endl
                                   << "\t- tile range x: [" << downscaledRoi.x.begin << " - " << downscaledRoi.x.end << "]" << std::endl
                                   << "\t- tile range y: [" << downscaledRoi.y.begin << " - " << downscaledRoi.y.end << "]" << std::endl);

        cuda_volumeRefineSimilarity(_volumeRefineSim_dmp,
                                    _sgmDepthPixSizeMap_dmp,
                                    (_refineParams.useSgmNormalMap) ? &_sgmNormalMap_dmp : nullptr,
                                    rcDeviceCameraParamsId,
                                    tcDeviceCameraParamsId,
                                    rcDeviceMipmapImage,
                                    tcDeviceMipmapImage,
                                    _refineParams,
                                    depthRange,
                                    downscaledRoi,
                                    _stream);
    }

    // export intermediate volume information (if requested by user)
    exportVolumeInformation(tile, "afterRefine");

    // retrieve the best depth/sim in the volume
    // compute sub-pixel sample using a sliding gaussian
    cuda_volumeRefineBestDepth(_refinedDepthSimMap_dmp, _sgmDepthPixSizeMap_dmp, _volumeRefineSim_dmp, _refineParams, downscaledRoi, _stream);

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
    const int rcDeviceCameraParamsId = deviceCache.requestCameraParamsId(tile.rc, _refineParams.scale, _mp);

    // get R device mipmap image from cache
    const DeviceMipmapImage& rcDeviceMipmapImage = deviceCache.requestMipmapImage(tile.rc, _mp);

    cuda_depthSimMapOptimizeGradientDescent(_optimizedDepthSimMap_dmp,  // output depth/sim map optimized
                                            _optImgVariance_dmp,        // image variance buffer pre-allocate
                                            _optTmpDepthMap_dmp,        // temporary depth map buffer pre-allocate
                                            _sgmDepthPixSizeMap_dmp,    // input SGM upscaled depth/pixSize map
                                            _refinedDepthSimMap_dmp,    // input refined and fused depth/sim map
                                            rcDeviceCameraParamsId,
                                            rcDeviceMipmapImage,
                                            _refineParams,
                                            downscaledRoi,
                                            _stream);

    ALICEVISION_LOG_INFO(tile << "Color optimize depth/sim map done.");
}

void Refine::computeAndWriteNormalMap(const Tile& tile, const CudaDeviceMemoryPitched<float2, 2>& in_depthSimMap_dmp, const std::string& name)
{
    // downscale the region of interest
    const ROI downscaledRoi = downscaleROI(tile.roi, _refineParams.scale * _refineParams.stepXY);

    // get R device camera parameters id from cache
    DeviceCache& deviceCache = DeviceCache::getInstance();
    const int rcDeviceCameraParamsId = deviceCache.requestCameraParamsId(tile.rc, _refineParams.scale, _mp);

    ALICEVISION_LOG_INFO(tile << "Refine compute normal map of view id: " << _mp.getViewId(tile.rc) << ", rc: " << tile.rc << " (" << (tile.rc + 1)
                              << " / " << _mp.ncams << ").");

    cuda_depthSimMapComputeNormal(_normalMap_dmp, in_depthSimMap_dmp, rcDeviceCameraParamsId, _refineParams.stepXY, downscaledRoi, _stream);

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
    CudaHostMemoryHeap<TSimRefine, 3> volumeSim_hmh(_volumeRefineSim_dmp.getSize());
    volumeSim_hmh.copyFrom(_volumeRefineSim_dmp);

    // copy device SGM upscale depth/sim map to host memory
    CudaHostMemoryHeap<float2, 2> depthPixSizeMapSgmUpscale_hmh(_sgmDepthPixSizeMap_dmp.getSize());
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

#elif ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_VULKAN)

Refine::Refine(const mvsUtils::MultiViewParams& mp, const mvsUtils::TileParams& tileParams, const RefineParams& refineParams, uint32_t deviceID)
  : _mp(mp),
    _tileParams(tileParams),
    _refineParams(refineParams),
    _deviceID(deviceID)
{
    // get tile maximum dimensions
    const int downscale = _refineParams.scale * _refineParams.stepXY;
    const int maxTileWidth = divideRoundUp(tileParams.bufferWidth, downscale);
    const int maxTileHeight = divideRoundUp(tileParams.bufferHeight, downscale);

    // compute depth/sim map maximum dimensions
    const vk::Extent2D depthSimMapDim(maxTileWidth, maxTileHeight);

    // allocate depth/sim maps in device memory
    _sgmDepthPixSizeMap_dmp = std::make_shared<VulkanImage<float2>>(VulkanMemoryBase<float2>::create2DImage(_deviceID, depthSimMapDim.width, depthSimMapDim.height, vk::Format::eR32G32Sfloat, 1, true, false));
    _refinedDepthSimMap_dmp = std::make_shared<VulkanImage<float2>>(VulkanMemoryBase<float2>::create2DImage(_deviceID, depthSimMapDim.width, depthSimMapDim.height, vk::Format::eR32G32Sfloat, 1, true, false));
    _optimizedDepthSimMap_dmp = std::make_shared<VulkanImage<float2>>(VulkanMemoryBase<float2>::create2DImage(_deviceID, depthSimMapDim.width, depthSimMapDim.height, vk::Format::eR32G32Sfloat, 1, true, false));

    // Always allocate SGM upscaled normal map in device memory
    _sgmNormalMap_dmp = std::make_shared<VulkanImage<float4>>(VulkanMemoryBase<float4>::create2DImage(_deviceID, depthSimMapDim.width, depthSimMapDim.height, vk::Format::eR32G32B32A32Sfloat, 1, true, false));

    // allocate normal map in device memory
    if (_refineParams.exportIntermediateNormalMaps)
        _normalMap_dmp = std::make_shared<VulkanImage<float4>>(VulkanMemoryBase<float4>::create2DImage(_deviceID, depthSimMapDim.width, depthSimMapDim.height, vk::Format::eR32G32B32A32Sfloat, 1, true, false));

    // compute volume maximum dimensions
    const int nbDepthsToRefine = _refineParams.halfNbDepths * 2 + 1;
    const vk::Extent3D volDim(maxTileWidth, maxTileHeight, nbDepthsToRefine);

    // allocate refine volume in device memory
#ifdef TSIM_REFINE_USE_HALF
    _volumeRefineSim_dmp = std::make_shared<VulkanImage<TSimRefine>>(VulkanMemoryBase<TSimRefine>::create3DImage(_deviceID, volDim.width, volDim.height, volDim.depth, vk::Format::eR16Sfloat, 1, true, false));
#else
    _volumeRefineSim_dmp = std::make_shared<VulkanImage<TSimRefine>>(VulkanMemoryBase<TSimRefine>::create3DImage(_deviceID, volDim.width, volDim.height, volDim.depth, vk::Format::eR32Sfloat, 1, true, false));
#endif
    // allocate depth/sim map optimization buffers
    if (_refineParams.useColorOptimization)
    {
        _optTmpDepthMap_dmp = std::make_shared<VulkanImage<float>>(VulkanMemoryBase<float>::create2DImage(_deviceID, depthSimMapDim.width, depthSimMapDim.height, vk::Format::eR32Sfloat, 1, true, false));
        _optImgVariance_dmp = std::make_shared<VulkanImage<float>>(VulkanMemoryBase<float>::create2DImage(_deviceID, depthSimMapDim.width, depthSimMapDim.height, vk::Format::eR32Sfloat, 1, true, false));
    }
}

double Refine::getDeviceMemoryConsumption() const
{
    size_t bytes = 0;

    if(_sgmDepthPixSizeMap_dmp)
        bytes += _sgmDepthPixSizeMap_dmp->getByteSize();
    if(_refinedDepthSimMap_dmp)
        bytes += _refinedDepthSimMap_dmp->getByteSize();
    if(_optimizedDepthSimMap_dmp)
        bytes += _optimizedDepthSimMap_dmp->getByteSize();
    if(_sgmNormalMap_dmp)
        bytes += _sgmNormalMap_dmp->getByteSize();
    if(_normalMap_dmp)
        bytes += _normalMap_dmp->getByteSize();
    if(_volumeRefineSim_dmp)
        bytes += _volumeRefineSim_dmp->getByteSize();
    if(_optTmpDepthMap_dmp)
        bytes += _optTmpDepthMap_dmp->getByteSize();
    if(_optImgVariance_dmp)
        bytes += _optImgVariance_dmp->getByteSize();

    return (double(bytes) / (1024.0 * 1024.0));
}

void Refine::refineRc(const Tile& tile,
                      const VulkanImage<float2>& in_sgmDepthThicknessMap_dmp,
                      const VulkanImage<float4>& in_sgmNormalMap_dmp)
{
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
        const int rcDeviceCameraParamsId = deviceCache.requestCameraParamsId(_deviceID, tile.rc, _refineParams.scale, _mp);

        // get R device mipmap image from cache
        const DeviceMipmapImage& rcDeviceMipmapImage = deviceCache.requestMipmapImage(_deviceID, tile.rc, _mp);

        // compute upscaled SGM depth/pixSize map
        // - upscale SGM depth/thickness map
        // - filter masked pixels (alpha)
        // - compute pixSize from SGM thickness

        // compute upscale ratio
        const vk::Extent3D out_mapDim = _sgmDepthPixSizeMap_dmp->getDimensionSizes().value();
        const vk::Extent3D in_mapDim = in_sgmDepthThicknessMap_dmp.getDimensionSizes().value();
        const float ratio = float(in_mapDim.width) / float(out_mapDim.width);

        // get R mipmap image level and dimensions
        const float rcMipmapLevel = rcDeviceMipmapImage.getLevel(_refineParams.scale);
        const vk::Extent2D rcLevelDim = rcDeviceMipmapImage.getDimensions(_refineParams.scale);

        struct PushConstants
        {
            int32_t rcDeviceCameraParamsId;
            uint32_t rcLevelWidth;
            uint32_t rcLevelHeight;
            float rcMipmapLevel;
            int32_t stepXY;
            int32_t halfNbDepths;
            float ratio;
            ROI roi;
        };

        const auto pc = PushConstants{rcDeviceCameraParamsId, rcLevelDim.width, rcLevelDim.height, rcMipmapLevel, _refineParams.stepXY, _refineParams.halfNbDepths, ratio, downscaledRoi};

        // kernel execution
        if(_refineParams.interpolateMiddleDepth)
            VulkanCommandManager::getInstance(_deviceID)
                ->wait()
                ->reset()
                ->begin()
                ->shader(ALICEVISION_LOOKUP_SPIRV_MODULE(ComputeSgmUpscaledDepthPixSizeMap_bilinear))
                ->pushConstant(pc)
                ->bind(*_sgmDepthPixSizeMap_dmp, vk::DescriptorType::eStorageImage)
                ->bind(in_sgmDepthThicknessMap_dmp, vk::DescriptorType::eStorageImage)
                ->bind(rcDeviceMipmapImage.getImage(), vk::DescriptorType::eSampledImage)
                ->bind(DeviceCameraParamsArray::getInstance(_deviceID)->getCameraParamsArray(), vk::DescriptorType::eUniformBuffer)
                ->transferImageLayout(*_sgmDepthPixSizeMap_dmp, vk::ImageLayout::eGeneral, 0, 1)
                ->transferImageLayout(in_sgmDepthThicknessMap_dmp, vk::ImageLayout::eGeneral, 0, 1)
                ->transferImageLayout(rcDeviceMipmapImage.getImage(), vk::ImageLayout::eShaderReadOnlyOptimal, 0, 1)
                ->workgroups(vk::Extent3D(divUp(downscaledRoi.width(), 16), divUp(downscaledRoi.height(), 16), 1))
                ->dispatch()
                ->end()
                ->submit()
                ->wait();
        else
            VulkanCommandManager::getInstance(_deviceID)
                ->wait()
                ->reset()
                ->begin()
                ->shader(ALICEVISION_LOOKUP_SPIRV_MODULE(ComputeSgmUpscaledDepthPixSizeMap_nearestNeighbor))
                ->pushConstant(pc)
                ->bind(*_sgmDepthPixSizeMap_dmp, vk::DescriptorType::eStorageImage)
                ->bind(in_sgmDepthThicknessMap_dmp, vk::DescriptorType::eStorageImage)
                ->bind(rcDeviceMipmapImage.getImage(), vk::DescriptorType::eSampledImage)
                ->bind(DeviceCameraParamsArray::getInstance(_deviceID)->getCameraParamsArray(), vk::DescriptorType::eUniformBuffer)
                ->transferImageLayout(*_sgmDepthPixSizeMap_dmp, vk::ImageLayout::eGeneral, 0, 1)
                ->transferImageLayout(in_sgmDepthThicknessMap_dmp, vk::ImageLayout::eGeneral, 0, 1)
                ->transferImageLayout(rcDeviceMipmapImage.getImage(), vk::ImageLayout::eShaderReadOnlyOptimal, 0, 1)
                ->workgroups(vk::Extent3D(divUp(downscaledRoi.width(), 16), divUp(downscaledRoi.height(), 16), 1))
                ->dispatch()
                ->end()
                ->submit()
                ->wait();

        // export intermediate depth/pixSize map (if requested by user)
        if (_refineParams.exportIntermediateDepthSimMaps)
            writeDepthPixSizeMap(
              tile.rc, _mp, _tileParams, tile.roi, *_sgmDepthPixSizeMap_dmp, _refineParams.scale, _refineParams.stepXY, "sgmUpscaled");

        // upscale SGM normal map (if needed)
        if (_refineParams.useSgmNormalMap)
        {
            // compute upscale ratio
            const vk::Extent3D out_mapDim2 = _sgmNormalMap_dmp->getDimensionSizes().value();
            const vk::Extent3D in_mapDim2 = in_sgmNormalMap_dmp.getDimensionSizes().value();
            const float ratio2 = float(in_mapDim2.width) / float(out_mapDim2.width);

            struct PushConstants2
            {
                float ratio;
                ROI roi;
            };

            const auto pc2 = PushConstants2{ratio2, downscaledRoi};
            VulkanCommandManager::getInstance(_deviceID)
                ->wait()
                ->reset()
                ->begin()
                ->shader(ALICEVISION_LOOKUP_SPIRV_MODULE(MapUpscale_float4))
                ->pushConstant(pc2)
                ->bind(*_sgmNormalMap_dmp, vk::DescriptorType::eStorageImage)
                ->bind(in_sgmNormalMap_dmp, vk::DescriptorType::eStorageImage)
                ->transferImageLayout(*_sgmNormalMap_dmp, vk::ImageLayout::eGeneral, 0, 1)
                ->transferImageLayout(in_sgmNormalMap_dmp, vk::ImageLayout::eGeneral, 0, 1)
                ->workgroups(vk::Extent3D(divUp(downscaledRoi.width(), 16), divUp(downscaledRoi.height(), 16), 1))
                ->dispatch()
                ->end()
                ->submit()
                ->wait();
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

        // get output map dimensions
        const vk::Extent3D depthSimMapDim = _refinedDepthSimMap_dmp->getDimensionSizes().value();

        struct PushConstants3
        {
            float defaultSim;
        };

        const auto pc3 = PushConstants3{1.0f};

        VulkanCommandManager::getInstance(_deviceID)
            ->wait()
            ->reset()
            ->begin()
            ->shader(ALICEVISION_LOOKUP_SPIRV_MODULE(DepthSimMapCopyDepthOnly))
            ->pushConstant(pc3)
            ->bind(*_refinedDepthSimMap_dmp, vk::DescriptorType::eStorageImage)
            ->bind(*_sgmDepthPixSizeMap_dmp, vk::DescriptorType::eStorageImage)
            ->transferImageLayout(*_refinedDepthSimMap_dmp, vk::ImageLayout::eGeneral, 0, 1)
            ->transferImageLayout(*_sgmDepthPixSizeMap_dmp, vk::ImageLayout::eGeneral, 0, 1)
            ->workgroups(vk::Extent3D(divUp(depthSimMapDim.width, 16), divUp(depthSimMapDim.height, 16), 1))
            ->dispatch()
            ->end()
            ->submit()
            ->wait();
    }

    // export intermediate depth/sim map (if requested by user)
    if (_refineParams.exportIntermediateDepthSimMaps)
        writeDepthSimMap(tile.rc, _mp, _tileParams, tile.roi, *_refinedDepthSimMap_dmp, _refineParams.scale, _refineParams.stepXY, "refinedFused");

    // export intermediate normal map (if requested by user)
    if (_refineParams.exportIntermediateNormalMaps)
        computeAndWriteNormalMap(tile, *_refinedDepthSimMap_dmp, "refinedFused");

    // optimize depth/sim map
    if (_refineParams.useColorOptimization && _refineParams.optimizationNbIterations > 0)
    {
        optimizeDepthSimMap(tile);
    }
    else
    {
        ALICEVISION_LOG_INFO(tile << "Color optimize depth/sim map disabled.");
        _optimizedDepthSimMap_dmp->copyFromAllocation(*_refinedDepthSimMap_dmp, 0, 0);
    }

    // export intermediate normal map (if requested by user)
    if (_refineParams.exportIntermediateNormalMaps)
        computeAndWriteNormalMap(tile, *_optimizedDepthSimMap_dmp);

    ALICEVISION_LOG_INFO(tile << "Refine depth/sim map done.");
}

void Refine::refineAndFuseDepthSimMap(const Tile& tile)
{
    ALICEVISION_LOG_INFO(tile << "Refine and fuse depth/sim map volume.");

    // downscale the region of interest
    const ROI downscaledRoi = downscaleROI(tile.roi, _refineParams.scale * _refineParams.stepXY);

    // get the depth range
    const Range depthRange(0, _volumeRefineSim_dmp->getDimensionSizes().value().depth);

    // initialize the similarity volume at 0
    // each tc filtered and inverted similarity value will be summed in this volume
    // TODO: Move initialization to GPU kernel?
    std::vector<TSimRefine> host = {};
    host.reserve(_volumeRefineSim_dmp->getSize());
    std::ranges::fill(host, TSimRefine(0.f));
    _volumeRefineSim_dmp->copyFromHostToImage(host.data(), host.size(), 0, 0);

    // get device cache instance
    DeviceCache& deviceCache = DeviceCache::getInstance();

    // get R device camera parameters id from cache
    const int rcDeviceCameraParamsId = deviceCache.requestCameraParamsId(_deviceID, tile.rc, _refineParams.scale, _mp);

    // get R device mipmap image from cache
    const DeviceMipmapImage& rcDeviceMipmapImage = deviceCache.requestMipmapImage(_deviceID, tile.rc, _mp);

    // compute for each RcTc each similarity value for each depth to refine
    // sum the inverted / filtered similarity value, best value is the HIGHEST
    for (std::size_t tci = 0; tci < tile.refineTCams.size(); ++tci)
    {
        const int tc = tile.refineTCams.at(tci);

        // get T device camera parameters id from cache
        const int tcDeviceCameraParamsId = deviceCache.requestCameraParamsId(_deviceID, tc, _refineParams.scale, _mp);

        // get T device mipmap image from cache
        const DeviceMipmapImage& tcDeviceMipmapImage = deviceCache.requestMipmapImage(_deviceID, tc, _mp);

        ALICEVISION_LOG_DEBUG(tile << "Refine similarity volume:" << std::endl
                                   << "\t- rc: " << tile.rc << std::endl
                                   << "\t- tc: " << tc << " (" << (tci + 1) << "/" << tile.refineTCams.size() << ")" << std::endl
                                   << "\t- rc camera parameters id: " << rcDeviceCameraParamsId << std::endl
                                   << "\t- tc camera parameters id: " << tcDeviceCameraParamsId << std::endl
                                   << "\t- tile range x: [" << downscaledRoi.x.begin << " - " << downscaledRoi.x.end << "]" << std::endl
                                   << "\t- tile range y: [" << downscaledRoi.y.begin << " - " << downscaledRoi.y.end << "]" << std::endl);

        // get mipmap images level and dimensions
        const float rcMipmapLevel = rcDeviceMipmapImage.getLevel(_refineParams.scale);
        const vk::Extent2D rcLevelDim = rcDeviceMipmapImage.getDimensions(_refineParams.scale);
        const vk::Extent2D tcLevelDim = tcDeviceMipmapImage.getDimensions(_refineParams.scale);

        struct PushConstants
        {
            int32_t rcDeviceCameraParamsId;
            int32_t tcDeviceCameraParamsId;
            uint32_t rcRefineLevelWidth;
            uint32_t rcRefineLevelHeight;
            uint32_t tcRefineLevelWidth;
            uint32_t tcRefineLevelHeight;
            float rcMipmapLevel;
            int32_t volDimZ;
            int32_t stepXY;
            int32_t wsh;
            float invGammaC;
            float invGammaP;
            bool useConsistentScale;
            bool useCustomPatchPattern;
            bool useNormalMap;
            Range depthRange;
            ROI roi;
        };

        const auto pc = PushConstants{
            rcDeviceCameraParamsId,
            tcDeviceCameraParamsId,
            rcLevelDim.width,
            rcLevelDim.height,
            tcLevelDim.width,
            tcLevelDim.height,
            rcMipmapLevel,
            static_cast<int32_t>(_volumeRefineSim_dmp->getDimensionSizes().value().depth),
            _refineParams.stepXY,
            _refineParams.wsh,
            1.f / float(_refineParams.gammaC), // inverted gammaC
            1.f / float(_refineParams.gammaP), // inverted gammaP_,
            _refineParams.useConsistentScale,
            _refineParams.useCustomPatchPattern,
            _sgmNormalMap_dmp != nullptr,
            depthRange,
            downscaledRoi
        };

        VulkanCommandManager::getInstance(_deviceID)
            ->wait()
            ->reset()
            ->begin()
#ifdef TSIM_REFINE_USE_HALF
            ->shader(ALICEVISION_LOOKUP_SPIRV_MODULE(VolumeRefineSimilarity_half))
#else
            ->shader(ALICEVISION_LOOKUP_SPIRV_MODULE(VolumeRefineSimilarity_float))
#endif
            ->pushConstant(pc)
            ->bind(*_volumeRefineSim_dmp, vk::DescriptorType::eStorageImage)
            ->bind(*_sgmDepthPixSizeMap_dmp, vk::DescriptorType::eStorageImage)
            ->bind(*_sgmNormalMap_dmp, vk::DescriptorType::eStorageImage)
            ->bind(rcDeviceMipmapImage.getImage(), vk::DescriptorType::eSampledImage)
            ->bind(tcDeviceMipmapImage.getImage(), vk::DescriptorType::eSampledImage)
            ->bind(DeviceCameraParamsArray::getInstance(_deviceID)->getCameraParamsArray(), vk::DescriptorType::eUniformBuffer)
            ->bind(DevicePatchPatternConstant::getInstance(_deviceID)->getPatchPatternConstant(), vk::DescriptorType::eUniformBuffer)
            ->transferImageLayout(*_volumeRefineSim_dmp, vk::ImageLayout::eGeneral, 0, 1)
            ->transferImageLayout(*_sgmDepthPixSizeMap_dmp, vk::ImageLayout::eGeneral, 0, 1)
            ->transferImageLayout(*_sgmNormalMap_dmp, vk::ImageLayout::eGeneral, 0, 1)
            ->transferImageLayout(rcDeviceMipmapImage.getImage(), vk::ImageLayout::eShaderReadOnlyOptimal, 0, vk::WholeSize)
            ->transferImageLayout(tcDeviceMipmapImage.getImage(), vk::ImageLayout::eShaderReadOnlyOptimal, 0, vk::WholeSize)
            ->workgroups(vk::Extent3D(divUp(downscaledRoi.width(), 32), divUp(downscaledRoi.height(), 32), depthRange.size()))
            ->dispatch()
            ->end()
            ->submit()
            ->wait();
    }

    // export intermediate volume information (if requested by user)
    exportVolumeInformation(tile, "afterRefine");

    // retrieve the best depth/sim in the volume
    // compute sub-pixel sample using a sliding gaussian

    // constant kernel inputs
    const int halfNbSamples = _refineParams.nbSubsamples * _refineParams.halfNbDepths;
    const float twoTimesSigmaPowerTwo = float(2.0 * _refineParams.sigma * _refineParams.sigma);

    struct PushConstants2
    {
        int volDimZ;
        int samplesPerPixSize;
        int halfNbSamples;
        int halfNbDepths;
        float twoTimesSigmaPowerTwo;
        ROI roi;
    };

    const auto pc2 = PushConstants2{
        static_cast<int32_t>(_volumeRefineSim_dmp->getDimensionSizes().value().depth),
        _refineParams.nbSubsamples,
        halfNbSamples,
        _refineParams.halfNbDepths,
        twoTimesSigmaPowerTwo,
        downscaledRoi
    };

    VulkanCommandManager::getInstance(_deviceID)
        ->wait()
        ->reset()
        ->begin()
#ifdef TSIM_REFINE_USE_HALF
        ->shader(ALICEVISION_LOOKUP_SPIRV_MODULE(RefineBestDepth_half))
#else
        ->shader(ALICEVISION_LOOKUP_SPIRV_MODULE(RefineBestDepth_float))
#endif
        ->pushConstant(pc2)
        ->bind(*_refinedDepthSimMap_dmp, vk::DescriptorType::eStorageImage)
        ->bind(*_sgmDepthPixSizeMap_dmp, vk::DescriptorType::eStorageImage)
        ->bind(*_volumeRefineSim_dmp, vk::DescriptorType::eStorageImage)
        ->transferImageLayout(*_refinedDepthSimMap_dmp, vk::ImageLayout::eGeneral, 0, 1)
        ->transferImageLayout(*_sgmDepthPixSizeMap_dmp, vk::ImageLayout::eGeneral, 0, 1)
        ->transferImageLayout(*_volumeRefineSim_dmp, vk::ImageLayout::eGeneral, 0, 1)
        ->workgroups(vk::Extent3D(divUp(downscaledRoi.width(), 32), divUp(downscaledRoi.height(), 32), depthRange.size()))
        ->dispatch()
        ->end()
        ->submit()
        ->wait();

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
    const int rcDeviceCameraParamsId = deviceCache.requestCameraParamsId(_deviceID, tile.rc, _refineParams.scale, _mp);

    // get R device mipmap image from cache
    const DeviceMipmapImage& rcDeviceMipmapImage = deviceCache.requestMipmapImage(_deviceID, tile.rc, _mp);

    // get R mipmap image level and dimensions
    const float rcMipmapLevel = rcDeviceMipmapImage.getLevel(_refineParams.scale);
    const vk::Extent2D rcLevelDim = rcDeviceMipmapImage.getDimensions(_refineParams.scale);

    // initialize depth/sim map optimized with SGM depth/pixSize map
    _optimizedDepthSimMap_dmp->copyFromAllocation(*_sgmDepthPixSizeMap_dmp, 0, 0);

    struct PushConstants
    {
        uint32_t rcLevelWidth;
        uint32_t rcLevelHeight;
        float rcMipmapLevel;
        int32_t stepXY;
        ROI roi;
    };

    const auto pc = PushConstants{rcLevelDim.width, rcLevelDim.height, rcMipmapLevel, _refineParams.stepXY, downscaledRoi};

    VulkanCommandManager::getInstance(_deviceID)
        ->wait()
        ->reset()
        ->begin()
        ->shader(ALICEVISION_LOOKUP_SPIRV_MODULE(OptimizeVarLofLABtoW))
        ->pushConstant(pc)
        ->bind(*_optImgVariance_dmp, vk::DescriptorType::eStorageImage)
        ->bind(rcDeviceMipmapImage.getImage(), vk::DescriptorType::eSampledImage)
        ->transferImageLayout(*_optImgVariance_dmp, vk::ImageLayout::eGeneral, 0, 1)
        ->transferImageLayout(rcDeviceMipmapImage.getImage(), vk::ImageLayout::eShaderReadOnlyOptimal, 0, vk::WholeSize)
        ->workgroups(vk::Extent3D(divUp(downscaledRoi.width(), 32), divUp(downscaledRoi.height(), 2), 1))
        ->dispatch()
        ->end()
        ->submit()
        ->wait();

    for(int iter = 0; iter < _refineParams.optimizationNbIterations; ++iter) // default nb iterations is 100
    {
        struct PushConstants2
        {
            ROI roi;
        };

        const auto pc2 = PushConstants2{downscaledRoi};

        VulkanCommandManager::getInstance(_deviceID)
            ->wait()
            ->reset()
            ->begin()
            ->shader(ALICEVISION_LOOKUP_SPIRV_MODULE(OptimizeGetOptDepthMapFromOptDepthSimMap))
            ->pushConstant(pc2)
            ->bind(*_optTmpDepthMap_dmp, vk::DescriptorType::eStorageImage)
            ->bind(*_optimizedDepthSimMap_dmp, vk::DescriptorType::eSampledImage)
            ->transferImageLayout(*_optTmpDepthMap_dmp, vk::ImageLayout::eGeneral, 0, 1)
            ->transferImageLayout(*_optimizedDepthSimMap_dmp, vk::ImageLayout::eGeneral, 0, 1)
            ->workgroups(vk::Extent3D(divUp(downscaledRoi.width(), 16), divUp(downscaledRoi.height(), 16), 1))
            ->dispatch()
            ->end()
            ->submit()
            ->wait();

        struct PushConstants3
        {
            int32_t rcDeviceCameraParamsId;
            int32_t iter;
            ROI roi;
        };

        const auto pc3 = PushConstants3{rcDeviceCameraParamsId, iter, downscaledRoi};

        VulkanCommandManager::getInstance(_deviceID)
            ->wait()
            ->reset()
            ->begin()
            ->shader(ALICEVISION_LOOKUP_SPIRV_MODULE(OptimizeDepthSimMap))
            ->pushConstant(pc3)
            ->bind(*_optimizedDepthSimMap_dmp, vk::DescriptorType::eStorageImage)
            ->bind(*_sgmDepthPixSizeMap_dmp, vk::DescriptorType::eStorageImage)
            ->bind(*_refinedDepthSimMap_dmp, vk::DescriptorType::eStorageImage)
            ->bind(*_optImgVariance_dmp, vk::DescriptorType::eSampledImage)
            ->bind(*_optTmpDepthMap_dmp, vk::DescriptorType::eSampledImage)
            ->bind(DeviceCameraParamsArray::getInstance(_deviceID)->getCameraParamsArray(), vk::DescriptorType::eUniformBuffer)
            ->transferImageLayout(*_optimizedDepthSimMap_dmp, vk::ImageLayout::eGeneral, 0, 1)
            ->transferImageLayout(*_sgmDepthPixSizeMap_dmp, vk::ImageLayout::eGeneral, 0, 1)
            ->transferImageLayout(*_refinedDepthSimMap_dmp, vk::ImageLayout::eGeneral, 0, 1)
            ->transferImageLayout(*_optImgVariance_dmp, vk::ImageLayout::eShaderReadOnlyOptimal, 0, vk::WholeSize)
            ->transferImageLayout(*_optTmpDepthMap_dmp, vk::ImageLayout::eShaderReadOnlyOptimal, 0, vk::WholeSize)
            ->workgroups(vk::Extent3D(divUp(downscaledRoi.width(), 16), divUp(downscaledRoi.height(), 16), 1))
            ->dispatch()
            ->end()
            ->submit()
            ->wait();
    }

    ALICEVISION_LOG_INFO(tile << "Color optimize depth/sim map done.");
}

void Refine::computeAndWriteNormalMap(const Tile& tile, const VulkanImage<float2>& in_depthSimMap_dmp, const std::string& name)
{
    // downscale the region of interest
    const ROI downscaledRoi = downscaleROI(tile.roi, _refineParams.scale * _refineParams.stepXY);

    // get R device camera parameters id from cache
    DeviceCache& deviceCache = DeviceCache::getInstance();
    const int rcDeviceCameraParamsId = deviceCache.requestCameraParamsId(_deviceID, tile.rc, _refineParams.scale, _mp);

    ALICEVISION_LOG_INFO(tile << "Refine compute normal map of view id: " << _mp.getViewId(tile.rc) << ", rc: " << tile.rc << " (" << (tile.rc + 1)
                              << " / " << _mp.ncams << ").");

    struct PushConstants
    {
        int32_t TWsh;
        int32_t rcDeviceCameraParamsId;
        int32_t stepXY;
        ROI roi;
    };

    const auto pc = PushConstants{3, rcDeviceCameraParamsId, _refineParams.stepXY, downscaledRoi};

    VulkanCommandManager::getInstance(_deviceID)
        ->wait()
        ->reset()
        ->begin()
        ->shader(ALICEVISION_LOOKUP_SPIRV_MODULE(DepthSimMapComputeNormal))
        ->pushConstant(pc)
        ->bind(*_normalMap_dmp, vk::DescriptorType::eStorageImage)
        ->bind(in_depthSimMap_dmp, vk::DescriptorType::eStorageImage)
        ->bind(DeviceCameraParamsArray::getInstance(_deviceID)->getCameraParamsArray(), vk::DescriptorType::eUniformBuffer)
        ->transferImageLayout(*_normalMap_dmp, vk::ImageLayout::eGeneral, 0, 1)
        ->transferImageLayout(in_depthSimMap_dmp, vk::ImageLayout::eGeneral, 0, 1)
        ->workgroups(vk::Extent3D(divUp(downscaledRoi.width(), 8), divUp(downscaledRoi.height(), 8), 1))
        ->dispatch()
        ->end()
        ->submit()
        ->wait();

    writeNormalMap(tile.rc, _mp, _tileParams, tile.roi, *_normalMap_dmp, _refineParams.scale, _refineParams.stepXY, name);
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

    if (_refineParams.exportIntermediateCrossVolumes)
    {
        ALICEVISION_LOG_INFO(tile << "Export similarity volume cross (" << name << ").");

        const std::string volumeCrossPath = getFileNameFromIndex(_mp, tile.rc, mvsUtils::EFileType::volumeCross, "_" + name, tileBeginX, tileBeginY);

        exportSimilarityVolumeCross(*_volumeRefineSim_dmp, *_sgmDepthPixSizeMap_dmp, _mp, tile.rc, _refineParams, volumeCrossPath, tile.roi);

        ALICEVISION_LOG_INFO(tile << "Export similarity volume cross (" << name << ") done.");
    }

    if (_refineParams.exportIntermediateTopographicCutVolumes)
    {
        ALICEVISION_LOG_INFO(tile << "Export similarity volume topographic cut (" << name << ").");

        const std::string volumeCutPath =
          getFileNameFromIndex(_mp, tile.rc, mvsUtils::EFileType::volumeTopographicCut, "_" + name, tileBeginX, tileBeginY);

        exportSimilarityVolumeTopographicCut(*_volumeRefineSim_dmp, *_sgmDepthPixSizeMap_dmp, _mp, tile.rc, _refineParams, volumeCutPath, tile.roi);

        ALICEVISION_LOG_INFO(tile << "Export similarity volume topographic cut (" << name << ") done.");
    }

    if (_refineParams.exportIntermediateVolume9pCsv)
    {
        ALICEVISION_LOG_INFO(tile << "Export similarity volume 9 points CSV (" << name << ").");

        const std::string stats9Path = getFileNameFromIndex(_mp, tile.rc, mvsUtils::EFileType::stats9p, "_refine", tileBeginX, tileBeginY);

        exportSimilaritySamplesCSV(*_volumeRefineSim_dmp, name, _refineParams, stats9Path, tile.roi);

        ALICEVISION_LOG_INFO(tile << "Export similarity volume 9 points CSV (" << name << ") done.");
    }
}

#elif ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_METAL)

using namespace gpu;

Refine::Refine(const mvsUtils::MultiViewParams& mp, const mvsUtils::TileParams& tileParams, const RefineParams& refineParams, const uint64_t deviceID)
  : _mp(mp),
    _tileParams(tileParams),
    _refineParams(refineParams),
    _deviceID(deviceID)
{
    // Get Resource Manager for deviceID
    MTLResourceManager* resMng = MTLDeviceManager::getInstance()->getResourceManager(deviceID);

    // get tile maximum dimensions
    const int downscale = _refineParams.scale * _refineParams.stepXY;
    const int maxTileWidth = divideRoundUp(tileParams.bufferWidth, downscale);
    const int maxTileHeight = divideRoundUp(tileParams.bufferHeight, downscale);

    // compute depth/sim map maximum dimensions
    const MTL::Size depthSimMapDim(maxTileWidth, maxTileHeight, 1);

    // allocate depth/sim maps in device memory
    _sgmDepthPixSizeMap_dmp = resMng->createTexture2D<float4>(MTL::PixelFormatRGBA32Float, depthSimMapDim.width, depthSimMapDim.height, 1, false);
    _refinedDepthSimMap_dmp = resMng->createTexture2D<float4>(MTL::PixelFormatRGBA32Float, depthSimMapDim.width, depthSimMapDim.height, 1, false);
    _optimizedDepthSimMap_dmp = resMng->createTexture2D<float4>(MTL::PixelFormatRGBA32Float, depthSimMapDim.width, depthSimMapDim.height, 1, false);

    // allocate SGM upscaled normal map in device memory
    if (_refineParams.useSgmNormalMap)
        _sgmNormalMap_dmp = resMng->createTexture2D<float4>(MTL::PixelFormatRGBA32Float, depthSimMapDim.width, depthSimMapDim.height, 1, false);
    else
        _sgmNormalMap_dmp_DUMMY = resMng->createTexture2D<float4>(MTL::PixelFormatRGBA32Float, 1, 1, 1, false);

    // allocate normal map in device memory
    if (_refineParams.exportIntermediateNormalMaps)
        _normalMap_dmp = resMng->createTexture2D<float4>(MTL::PixelFormatRGBA32Float, depthSimMapDim.width, depthSimMapDim.height, 1, false);

    // compute volume maximum dimensions
    const int nbDepthsToRefine = _refineParams.halfNbDepths * 2 + 1;
    const MTL::Size volDim(maxTileWidth, maxTileHeight, nbDepthsToRefine);

    // allocate refine volume in device memory
    _volumeRefineSim_dmp = resMng->createTexture3D<TSimRefine>(MTLTSimRefineFormat, volDim.width, volDim.height, volDim.depth, 1, false);

    // allocate depth/sim map optimization buffers
    if (_refineParams.useColorOptimization)
    {
        _optTmpDepthMap_dmp = resMng->createTexture2D<float>(MTL::PixelFormatR32Float, depthSimMapDim.width, depthSimMapDim.height, 1, false);
        _optImgVariance_dmp = resMng->createTexture2D<float>(MTL::PixelFormatR32Float, depthSimMapDim.width, depthSimMapDim.height, 1, false);
    }
}

double Refine::getDeviceMemoryConsumption() const
{
    size_t bytes = 0;

    bytes += _sgmDepthPixSizeMap_dmp.getBytesPadded();
    bytes += _refinedDepthSimMap_dmp.getBytesPadded();
    bytes += _optimizedDepthSimMap_dmp.getBytesPadded();
    if (_refineParams.useSgmNormalMap)
        bytes += _sgmNormalMap_dmp.getBytesPadded();
    if (_refineParams.exportIntermediateNormalMaps)
        bytes += _normalMap_dmp.getBytesPadded();
    bytes += _volumeRefineSim_dmp.getBytesPadded();
    if (_refineParams.useColorOptimization)
        bytes += _optTmpDepthMap_dmp.getBytesPadded();
    if (_refineParams.useColorOptimization)
        bytes += _optImgVariance_dmp.getBytesPadded();

    return (double(bytes) / (1024.0 * 1024.0));
}

double Refine::getDeviceMemoryConsumptionUnpadded() const
{
    size_t bytes = 0;

    bytes += _sgmDepthPixSizeMap_dmp.getBytesUnpadded();
    bytes += _refinedDepthSimMap_dmp.getBytesUnpadded();
    bytes += _optimizedDepthSimMap_dmp.getBytesUnpadded();
    if (_refineParams.useSgmNormalMap)
        bytes += _sgmNormalMap_dmp.getBytesUnpadded();
    if (_refineParams.exportIntermediateNormalMaps)
        bytes += _normalMap_dmp.getBytesUnpadded();
    bytes += _volumeRefineSim_dmp.getBytesUnpadded();
    if (_refineParams.useColorOptimization)
        bytes += _optTmpDepthMap_dmp.getBytesUnpadded();
    if (_refineParams.useColorOptimization)
        bytes += _optImgVariance_dmp.getBytesUnpadded();

    return (double(bytes) / (1024.0 * 1024.0));
}

void Refine::refineRc(const Tile& tile,
                      const MTLSharedResource<MTLTexture, float4>& in_sgmDepthThicknessMap_dmp,
                      const MTLSharedResource<MTLTexture, float4>& in_sgmNormalMap_dmp)
{
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
        const int rcDeviceCameraParamsId = deviceCache.requestCameraParamsId(_deviceID, tile.rc, _refineParams.scale, _mp);

        // get R device mipmap image from cache
        const DeviceMipmapImage& rcDeviceMipmapImage = deviceCache.requestMipmapImage(_deviceID, tile.rc, _mp);

        // compute upscaled SGM depth/pixSize map
        // - upscale SGM depth/thickness map
        // - filter masked pixels (alpha)
        // - compute pixSize from SGM thickness

        // compute upscale ratio
        const MTL::Size out_mapDim = _sgmDepthPixSizeMap_dmp.getTextureDimensions();
        const MTL::Size in_mapDim = in_sgmDepthThicknessMap_dmp.getTextureDimensions();
        const float ratio = float(in_mapDim.width) / float(out_mapDim.width);

        // get R mipmap image level and dimensions
        const float rcMipmapLevel = rcDeviceMipmapImage.getLevel(_refineParams.scale);
        const MTL::Size rcLevelDim = rcDeviceMipmapImage.getDimensions(_refineParams.scale);

        // Get Command Manager
        const auto cmdMng = MTLDeviceManager::getInstance()->getCommandManager(_deviceID);

        // Get Device Camera Params
        const auto deviceCameraParamsArrayWrapper = DeviceCache::getInstance().getDeviceCameraParamsArrayWrapper(_deviceID);
        const auto& deviceCameraParamsArray = deviceCameraParamsArrayWrapper->getDeviceCameraParamsBuffer();

        // kernel execution
        if(_refineParams.interpolateMiddleDepth)
        {
            cmdMng
            ->loadLibrary("AVDepthMapMetalKernels", "org.aliceVision.AVDepthMap")
            ->reset()
            ->commandBuffer()
            ->pipeline("aliceVision::depthMap::ComputeSgmUpscaledDepthPixSizeMap_Bilinear", "AVDepthMapMetalKernels")
            ->commandEncoder()
            ->bind(_sgmDepthPixSizeMap_dmp, 0)
            ->bind(in_sgmDepthThicknessMap_dmp, 1)
            ->bind(rcDeviceMipmapImage.getTexture(), 2)
            ->createSampler(0)
            ->bind(deviceCameraParamsArray, 1)
            ->pushConstants(ComputeSgmUpscaledDepthPixSizeMap_Bilinear_PushConstants{rcDeviceCameraParamsId, static_cast<unsigned int>(rcLevelDim.width), static_cast<unsigned int>(rcLevelDim.height), rcMipmapLevel, _refineParams.stepXY, _refineParams.halfNbDepths, ratio, downscaledRoi})
            ->dispatchDimensions(MTL::Size(downscaledRoi.width(), downscaledRoi.height(), 1), MTL::Size(16, 16, 1))
            ->endRecording()
            ->commitCommands()
            ->waitAll();
        }
        else
        {
            cmdMng
            ->loadLibrary("AVDepthMapMetalKernels", "org.aliceVision.AVDepthMap")
            ->reset()
            ->commandBuffer()
            ->pipeline("aliceVision::depthMap::ComputeSgmUpscaledDepthPixSizeMap_NearestNeighbor", "AVDepthMapMetalKernels")
            ->commandEncoder()
            ->bind(_sgmDepthPixSizeMap_dmp, 0)
            ->bind(in_sgmDepthThicknessMap_dmp, 1)
            ->bind(rcDeviceMipmapImage.getTexture(), 2)
            ->createSampler(0)
            ->bind(deviceCameraParamsArray, 1)
            ->pushConstants(ComputeSgmUpscaledDepthPixSizeMap_NearestNeighbor_PushConstants{rcDeviceCameraParamsId, static_cast<unsigned int>(rcLevelDim.width), static_cast<unsigned int>(rcLevelDim.height), rcMipmapLevel, _refineParams.stepXY, _refineParams.halfNbDepths, ratio, downscaledRoi})
            ->dispatchDimensions(MTL::Size(downscaledRoi.width(), downscaledRoi.height(), 1), MTL::Size(16, 16, 1))
            ->endRecording()
            ->commitCommands()
            ->waitAll();
        }

        // export intermediate depth/pixSize map (if requested by user)
        if (_refineParams.exportIntermediateDepthSimMaps)
            writeDepthPixSizeMap(
              tile.rc, _mp, _tileParams, tile.roi, _sgmDepthPixSizeMap_dmp, _refineParams.scale, _refineParams.stepXY, "sgmUpscaled");

        // upscale SGM normal map (if needed)
        if (_refineParams.useSgmNormalMap)
        {
            // compute upscale ratio
            const MTL::Size out_mapDim2 = _sgmNormalMap_dmp.getTextureDimensions();
            const MTL::Size in_mapDim2 = in_sgmNormalMap_dmp.getTextureDimensions();
            const float ratio2 = float(in_mapDim2.width) / float(out_mapDim2.width);

            cmdMng
            ->loadLibrary("AVDepthMapMetalKernels", "org.aliceVision.AVDepthMap")
            ->reset()
            ->commandBuffer()
            ->pipeline("aliceVision::depthMap::MapUpscale", "AVDepthMapMetalKernels")
            ->commandEncoder()
            ->bind(_sgmNormalMap_dmp, 0)
            ->bind(in_sgmNormalMap_dmp, 1)
            ->pushConstants(MapUpscale_PushConstants{ratio2, downscaledRoi})
            ->dispatchDimensions(MTL::Size(downscaledRoi.width(), downscaledRoi.height(), 1), MTL::Size(16, 16, 1))
            ->endRecording()
            ->commitCommands()
            ->waitAll();
        }
    }

    // Get Command Manager
    const auto cmdMng = MTLDeviceManager::getInstance()->getCommandManager(_deviceID);

    // refine and fuse depth/sim map
    if (_refineParams.useRefineFuse)
    {
        // refine and fuse with volume strategy
        refineAndFuseDepthSimMap(tile);
    }
    else
    {
        ALICEVISION_LOG_INFO(tile << "Refine and fuse depth/sim map volume disabled.");

        const MTL::Size depthSimMapDim = _refinedDepthSimMap_dmp.getTextureDimensions();

        cmdMng
        ->loadLibrary("AVDepthMapMetalKernels", "org.aliceVision.AVDepthMap")
        ->reset()
        ->commandBuffer()
        ->pipeline("aliceVision::depthMap::DepthSimMapCopyDepthOnly", "AVDepthMapMetalKernels")
        ->commandEncoder()
        ->bind(_refinedDepthSimMap_dmp, 0)
        ->bind(_sgmDepthPixSizeMap_dmp, 1)
        ->pushConstants(DepthSimMapCopyDepthOnly_PushConstants{1.0f, static_cast<unsigned int>(depthSimMapDim.width), static_cast<unsigned int>(depthSimMapDim.height)})
        ->dispatchDimensions(MTL::Size(depthSimMapDim.width, depthSimMapDim.height, 1), MTL::Size(16, 16, 1))
        ->endRecording()
        ->commitCommands()
        ->waitAll();
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
        _optimizedDepthSimMap_dmp.copyFromResource(_refinedDepthSimMap_dmp, 0, 0);
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
    const Range depthRange(0, _volumeRefineSim_dmp.getTextureDimensions().depth);

    // initialize the similarity volume at 0
    // each tc filtered and inverted similarity value will be summed in this volume
    std::vector<TSimRefine> hostData;
    const MTL::Size volDim = _volumeRefineSim_dmp.getTextureDimensions();
    hostData.resize(volDim.width * volDim.height * volDim.depth);
    std::ranges::fill(hostData, TSimRefine(0.f));
    _volumeRefineSim_dmp.copyFromHost(hostData.data(), hostData.size(), 0, 0);

    // get device cache instance
    DeviceCache& deviceCache = DeviceCache::getInstance();

    // get R device camera parameters id from cache
    const int rcDeviceCameraParamsId = deviceCache.requestCameraParamsId(_deviceID, tile.rc, _refineParams.scale, _mp);

    // get R device mipmap image from cache
    const DeviceMipmapImage& rcDeviceMipmapImage = deviceCache.requestMipmapImage(_deviceID, tile.rc, _mp);

    // compute for each RcTc each similarity value for each depth to refine
    // sum the inverted / filtered similarity value, best value is the HIGHEST
    for (std::size_t tci = 0; tci < tile.refineTCams.size(); ++tci)
    {
        const int tc = tile.refineTCams.at(tci);

        // get T device camera parameters id from cache
        const int tcDeviceCameraParamsId = deviceCache.requestCameraParamsId(_deviceID, tc, _refineParams.scale, _mp);

        // get T device mipmap image from cache
        const DeviceMipmapImage& tcDeviceMipmapImage = deviceCache.requestMipmapImage(_deviceID, tc, _mp);

        ALICEVISION_LOG_DEBUG(tile << "Refine similarity volume:" << std::endl
                                   << "\t- rc: " << tile.rc << std::endl
                                   << "\t- tc: " << tc << " (" << (tci + 1) << "/" << tile.refineTCams.size() << ")" << std::endl
                                   << "\t- rc camera parameters id: " << rcDeviceCameraParamsId << std::endl
                                   << "\t- tc camera parameters id: " << tcDeviceCameraParamsId << std::endl
                                   << "\t- tile range x: [" << downscaledRoi.x.begin << " - " << downscaledRoi.x.end << "]" << std::endl
                                   << "\t- tile range y: [" << downscaledRoi.y.begin << " - " << downscaledRoi.y.end << "]" << std::endl);

        // get mipmap images level and dimensions
        const float rcMipmapLevel = rcDeviceMipmapImage.getLevel(_refineParams.scale);
        const MTL::Size rcLevelDim = rcDeviceMipmapImage.getDimensions(_refineParams.scale);
        const MTL::Size tcLevelDim = tcDeviceMipmapImage.getDimensions(_refineParams.scale);

        // Get Command Manager
        const auto cmdMng = MTLDeviceManager::getInstance()->getCommandManager(_deviceID);

        // Create dummy pointer if necessary
        MTLSharedResource<MTLTexture, float4>* sgmNormalMapPtr;
        if (_refineParams.useSgmNormalMap)
            sgmNormalMapPtr = &_sgmNormalMap_dmp;
        else
            sgmNormalMapPtr = &_sgmNormalMap_dmp_DUMMY;

        // Get DeviceCameraParams Array
        const auto deviceCameraParamsArrayWrapper = DeviceCache::getInstance().getDeviceCameraParamsArrayWrapper(_deviceID);
        const auto& deviceCameraParamsArray = deviceCameraParamsArrayWrapper->getDeviceCameraParamsBuffer();

        // Get DevicePatchPattern Array
        const auto devicePatchPatternArrayWrapper = DeviceCache::getInstance().getDevicePatchPatternArrayWrapper(_deviceID);
        const auto& devicePatchPatternArray = devicePatchPatternArrayWrapper->getDevicePatchPatternBuffer();

        cmdMng
        ->loadLibrary("AVDepthMapMetalKernels", "org.aliceVision.AVDepthMap")
        ->reset()
        ->commandBuffer()
        ->pipeline("aliceVision::depthMap::VolumeRefineSimiliarity", "AVDepthMapMetalKernels")
        ->commandEncoder()
        ->bind(_volumeRefineSim_dmp, 0)
        ->bind(_sgmDepthPixSizeMap_dmp, 1)
        ->bind(*sgmNormalMapPtr, 2)
        ->bind(rcDeviceMipmapImage.getTexture(), 3)
        ->bind(tcDeviceMipmapImage.getTexture(), 4)
        ->createSampler(0)
        ->createSampler(1)
        ->bind(deviceCameraParamsArray, 1)
        ->bind(devicePatchPatternArray, 2)
        ->pushConstants(VolumeRefineSimiliarity_PushConstants{_refineParams.useSgmNormalMap, rcDeviceCameraParamsId, tcDeviceCameraParamsId, static_cast<unsigned int>(rcLevelDim.width), static_cast<unsigned int>(rcLevelDim.height), static_cast<unsigned int>(tcLevelDim.width), static_cast<unsigned int>(tcLevelDim.height), rcMipmapLevel, static_cast<int>(_volumeRefineSim_dmp.getTextureDimensions().depth), _refineParams.stepXY, _refineParams.wsh, (1.f / float(_refineParams.gammaC)), (1.f / float(_refineParams.gammaC)), _refineParams.useConsistentScale, _refineParams.useCustomPatchPattern, depthRange, downscaledRoi})
        ->dispatchDimensions(MTL::Size(downscaledRoi.width(), downscaledRoi.height(), depthRange.size()), MTL::Size(32, 1, 1))
        ->endRecording()
        ->commitCommands()
        ->waitAll();
    }

    // export intermediate volume information (if requested by user)
    exportVolumeInformation(tile, "afterRefine");

    // retrieve the best depth/sim in the volume
    // compute sub-pixel sample using a sliding gaussian

    // Get Command Manager
    const auto cmdMng = MTLDeviceManager::getInstance()->getCommandManager(_deviceID);

    // constant kernel inputs
    const int halfNbSamples = _refineParams.nbSubsamples * _refineParams.halfNbDepths;
    const float twoTimesSigmaPowerTwo = float(2.0 * _refineParams.sigma * _refineParams.sigma);

    cmdMng
    ->loadLibrary("AVDepthMapMetalKernels", "org.aliceVision.AVDepthMap")
    ->reset()
    ->commandBuffer()
    ->pipeline("aliceVision::depthMap::RefineBestDepth", "AVDepthMapMetalKernels")
    ->commandEncoder()
    ->bind(_refinedDepthSimMap_dmp, 0)
    ->bind(_sgmDepthPixSizeMap_dmp, 1)
    ->bind(_volumeRefineSim_dmp, 2)
    ->pushConstants(RefineBestDepth_PushConstants{static_cast<int>(_volumeRefineSim_dmp.getTextureDimensions().depth), _refineParams.nbSubsamples, halfNbSamples, _refineParams.halfNbDepths, twoTimesSigmaPowerTwo, downscaledRoi})
    ->dispatchDimensions(MTL::Size(downscaledRoi.width(), downscaledRoi.height(), 1), MTL::Size(32, 1, 1))
    ->endRecording()
    ->commitCommands()
    ->waitAll();

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
    const int rcDeviceCameraParamsId = deviceCache.requestCameraParamsId(_deviceID, tile.rc, _refineParams.scale, _mp);

    // get R device mipmap image from cache
    const DeviceMipmapImage& rcDeviceMipmapImage = deviceCache.requestMipmapImage(_deviceID, tile.rc, _mp);

    // Get Command Manager
    const auto cmdMng = MTLDeviceManager::getInstance()->getCommandManager(_deviceID);

    // get R mipmap image level and dimensions
    const float rcMipmapLevel = rcDeviceMipmapImage.getLevel(_refineParams.scale);
    const MTL::Size rcLevelDim = rcDeviceMipmapImage.getDimensions(_refineParams.scale);

    // initialize depth/sim map optimized with SGM depth/pixSize map
    _optimizedDepthSimMap_dmp.copyFromResource(_sgmDepthPixSizeMap_dmp, 0, 0);

    cmdMng
    ->loadLibrary("AVDepthMapMetalKernels", "org.aliceVision.AVDepthMap")
    ->reset()
    ->commandBuffer()
    ->pipeline("aliceVision::depthMap::OptimizeVarLofLABtoW", "AVDepthMapMetalKernels")
    ->commandEncoder()
    ->bind(_optImgVariance_dmp, 0)
    ->bind(rcDeviceMipmapImage.getTexture(), 1)
    ->createSampler(0)
    ->pushConstants(OptimizeVarLofLABtoW_PushConstants{static_cast<unsigned int>(rcLevelDim.width), static_cast<unsigned int>(rcLevelDim.height), rcMipmapLevel, _refineParams.stepXY, downscaledRoi})
    ->dispatchDimensions(MTL::Size(downscaledRoi.width(), downscaledRoi.height(), 1), MTL::Size(32, 1, 1))
    ->endRecording()
    ->commitCommands()
    ->waitAll();

    // Get DeviceCameraParams Array
    const auto deviceCameraParamsArrayWrapper = DeviceCache::getInstance().getDeviceCameraParamsArrayWrapper(_deviceID);
    const auto& deviceCameraParamsArray = deviceCameraParamsArrayWrapper->getDeviceCameraParamsBuffer();

    for(int iter = 0; iter < _refineParams.optimizationNbIterations; ++iter) // default nb iterations is 100
    {

        cmdMng
        ->loadLibrary("AVDepthMapMetalKernels", "org.aliceVision.AVDepthMap")
        ->reset()
        ->commandBuffer()
        ->pipeline("aliceVision::depthMap::OptimizeGetOptDepthMapFromOptDepthSimMap", "AVDepthMapMetalKernels")
        ->commandEncoder()
        ->bind(_optTmpDepthMap_dmp, 0)
        ->bind(_optimizedDepthSimMap_dmp, 1)
        ->pushConstants(OptimizeGetOptDepthMapFromOptDepthSimMap_PushConstants{downscaledRoi})
        ->dispatchDimensions(MTL::Size(downscaledRoi.width(), downscaledRoi.height(), 1), MTL::Size(16, 16, 1))
        ->endRecording()
        ->commitCommands()
        ->waitAll();

        cmdMng
        ->loadLibrary("AVDepthMapMetalKernels", "org.aliceVision.AVDepthMap")
        ->reset()
        ->commandBuffer()
        ->pipeline("aliceVision::depthMap::OptimizeDepthSimMap", "AVDepthMapMetalKernels")
        ->commandEncoder()
        ->bind(_optimizedDepthSimMap_dmp, 0)
        ->bind(_sgmDepthPixSizeMap_dmp, 1)
        ->bind(_refinedDepthSimMap_dmp, 2)
        ->bind(_optImgVariance_dmp, 3)
        ->bind(_optTmpDepthMap_dmp, 4)
        ->createSampler(false, false, 0)
        ->createSampler(false, false, 1)
        ->bind(deviceCameraParamsArray, 1)
        ->pushConstants(OptimizeDepthSimMap_PushConstants{rcDeviceCameraParamsId, iter, downscaledRoi})
        ->dispatchDimensions(MTL::Size(downscaledRoi.width(), downscaledRoi.height(), 1), MTL::Size(16, 16, 1))
        ->endRecording()
        ->commitCommands()
        ->waitAll();

    }

    ALICEVISION_LOG_INFO(tile << "Color optimize depth/sim map done.");
}

void Refine::computeAndWriteNormalMap(const Tile& tile, const MTLSharedResource<MTLTexture, float4>& in_depthSimMap_dmp, const std::string& name)
{
    // downscale the region of interest
    const ROI downscaledRoi = downscaleROI(tile.roi, _refineParams.scale * _refineParams.stepXY);

    // get R device camera parameters id from cache
    DeviceCache& deviceCache = DeviceCache::getInstance();
    const int rcDeviceCameraParamsId = deviceCache.requestCameraParamsId(_deviceID, tile.rc, _refineParams.scale, _mp);

    ALICEVISION_LOG_INFO(tile << "Refine compute normal map of view id: " << _mp.getViewId(tile.rc) << ", rc: " << tile.rc << " (" << (tile.rc + 1)
                              << " / " << _mp.ncams << ").");

    // Get Command Manager
    const auto cmdMng = MTLDeviceManager::getInstance()->getCommandManager(_deviceID);

    // Get DeviceCameraParams Array
    const auto deviceCameraParamsArrayWrapper = DeviceCache::getInstance().getDeviceCameraParamsArrayWrapper(_deviceID);
    const auto& deviceCameraParamsArray = deviceCameraParamsArrayWrapper->getDeviceCameraParamsBuffer();

    cmdMng
    ->loadLibrary("AVDepthMapMetalKernels", "org.aliceVision.AVDepthMap")
    ->reset()
    ->commandBuffer()
    ->pipeline("aliceVision::depthMap::DepthSimMapComputeNormal", "AVDepthMapMetalKernels")
    ->commandEncoder()
    ->bind(_normalMap_dmp, 0)
    ->bind(in_depthSimMap_dmp, 1)
    ->bind(deviceCameraParamsArray, 1)
    ->pushConstants(DepthSimMapComputeNormal_PushConstants{rcDeviceCameraParamsId, _refineParams.stepXY, 3, downscaledRoi})
    ->dispatchDimensions(MTL::Size(downscaledRoi.width(), downscaledRoi.height(), 1), MTL::Size(8, 8, 1))
    ->endRecording()
    ->commitCommands()
    ->waitAll();

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

    if (_refineParams.exportIntermediateCrossVolumes)
    {
        ALICEVISION_LOG_INFO(tile << "Export similarity volume cross (" << name << ").");

        const std::string volumeCrossPath = getFileNameFromIndex(_mp, tile.rc, mvsUtils::EFileType::volumeCross, "_" + name, tileBeginX, tileBeginY);

        exportSimilarityVolumeCross(_volumeRefineSim_dmp, _sgmDepthPixSizeMap_dmp, _mp, tile.rc, _refineParams, volumeCrossPath, tile.roi);

        ALICEVISION_LOG_INFO(tile << "Export similarity volume cross (" << name << ") done.");
    }

    if (_refineParams.exportIntermediateTopographicCutVolumes)
    {
        ALICEVISION_LOG_INFO(tile << "Export similarity volume topographic cut (" << name << ").");

        const std::string volumeCutPath =
          getFileNameFromIndex(_mp, tile.rc, mvsUtils::EFileType::volumeTopographicCut, "_" + name, tileBeginX, tileBeginY);

        exportSimilarityVolumeTopographicCut(_volumeRefineSim_dmp, _sgmDepthPixSizeMap_dmp, _mp, tile.rc, _refineParams, volumeCutPath, tile.roi);

        ALICEVISION_LOG_INFO(tile << "Export similarity volume topographic cut (" << name << ") done.");
    }

    if (_refineParams.exportIntermediateVolume9pCsv)
    {
        ALICEVISION_LOG_INFO(tile << "Export similarity volume 9 points CSV (" << name << ").");

        const std::string stats9Path = getFileNameFromIndex(_mp, tile.rc, mvsUtils::EFileType::stats9p, "_refine", tileBeginX, tileBeginY);

        exportSimilaritySamplesCSV(_volumeRefineSim_dmp, name, _refineParams, stats9Path, tile.roi);

        ALICEVISION_LOG_INFO(tile << "Export similarity volume 9 points CSV (" << name << ") done.");
    }
}

#else
    #error No backend for DepthMap computations selected!
#endif

}  // namespace depthMap
}  // namespace aliceVision

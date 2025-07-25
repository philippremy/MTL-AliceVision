// This file is part of the AliceVision project.
// Copyright (c) 2019 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <AV/config.hpp>

#include <AVMVSData/ROI.hpp>
#include <AVMVSUtils/MultiViewParams.hpp>
#include <AVDepthMap/SgmParams.hpp>
#include <AVDepthMap/RefineParams.hpp>

#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_CUDA)
    #include <AVDepthMap/cuda/host/memory.hpp>
    #include <AVDepthMap/cuda/planeSweeping/similarity.hpp>
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

#include <string>
#include <vector>

namespace aliceVision {
namespace depthMap {

#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_CUDA)

/**
 * @brief Export 9 similarity values over the entire depth in a CSV file.
 * @param[in] in_volumeSim_hmh the similarity in host memory
 * @param[in] in_depths the SGM depth list
 * @param[in] name the export name
 * @param[in] sgmParams the Semi Global Matching parameters
 * @param[in] filepath the export filepath
 * @param[in] roi the 2d region of interest
 */
void exportSimilaritySamplesCSV(const CudaHostMemoryHeap<TSim, 3>& in_volumeSim_hmh,
                                const std::vector<float>& in_depths,
                                const std::string& name,
                                const SgmParams& sgmParams,
                                const std::string& filepath,
                                const ROI& roi);

/**
 * @brief Export 9 similarity values over the entire depth in a CSV file.
 * @param[in] in_volumeSim_hmh the similarity in host memory
 * @param[in] name the export name
 * @param[in] refineParams the Refine parameters
 * @param[in] filepath the export filepath
 * @param[in] roi the 2d region of interest
 */
void exportSimilaritySamplesCSV(const CudaHostMemoryHeap<TSimRefine, 3>& in_volumeSim_hmh,
                                const std::string& name,
                                const RefineParams& refineParams,
                                const std::string& filepath,
                                const ROI& roi);

/**
 * @brief Export the given similarity volume to an Alembic file.
 * @param[in] in_volumeSim_hmh the similarity in host memory
 * @param[in] in_depths the SGM depth list
 * @param[in] mp the multi-view parameters
 * @param[in] camIndex the R cam global index
 * @param[in] sgmParams the Semi Global Matching parameters
 * @param[in] filepath the export filepath
 * @param[in] roi the 2d region of interest
 */
void exportSimilarityVolume(const CudaHostMemoryHeap<TSim, 3>& in_volumeSim_hmh,
                            const std::vector<float>& in_depths,
                            const mvsUtils::MultiViewParams& mp,
                            int camIndex,
                            const SgmParams& sgmParams,
                            const std::string& filepath,
                            const ROI& roi);

/**
 * @brief Export a cross of the given similarity volume to an Alembic file.
 * @param[in] in_volumeSim_hmh the similarity in host memory
 * @param[in] in_depths the SGM depth list
 * @param[in] mp the multi-view parameters
 * @param[in] camIndex the R cam global index
 * @param[in] sgmParams the Semi Global Matching parameters
 * @param[in] filepath the export filepath
 * @param[in] roi the 2d region of interest
 */
void exportSimilarityVolumeCross(const CudaHostMemoryHeap<TSim, 3>& in_volumeSim_hmh,
                                 const std::vector<float>& in_depths,
                                 const mvsUtils::MultiViewParams& mp,
                                 int camIndex,
                                 const SgmParams& sgmParams,
                                 const std::string& filepath,
                                 const ROI& roi);

/**
 * @brief Export a cross of the given similarity volume to an Alembic file.
 * @param[in] in_volumeSim_hmh the similarity in host memory
 * @param[in] in_depthSimMapSgmUpscale_hmh the upscaled SGM depth/sim map
 * @param[in] mp the multi-view parameters
 * @param[in] camIndex the R cam global index
 * @param[in] refineParams the Refine parameters
 * @param[in] filepath the export filepath
 * @param[in] roi the 2d region of interest
 */
void exportSimilarityVolumeCross(const CudaHostMemoryHeap<TSimRefine, 3>& in_volumeSim_hmh,
                                 const CudaHostMemoryHeap<float2, 2>& in_depthSimMapSgmUpscale_hmh,
                                 const mvsUtils::MultiViewParams& mp,
                                 int camIndex,
                                 const RefineParams& refineParams,
                                 const std::string& filepath,
                                 const ROI& roi);

/**
 * @brief Export a topographic cut of the given similarity volume to an Alembic file.
 * @param[in] in_volumeSim_hmh the similarity in host memory
 * @param[in] in_depths the SGM depth list
 * @param[in] mp the multi-view parameters
 * @param[in] camIndex the R cam global index
 * @param[in] sgmParams the Semi Global Matching parameters
 * @param[in] filepath the export filepath
 * @param[in] roi the 2d region of interest
 */
void exportSimilarityVolumeTopographicCut(const CudaHostMemoryHeap<TSim, 3>& in_volumeSim_hmh,
                                          const std::vector<float>& in_depths,
                                          const mvsUtils::MultiViewParams& mp,
                                          int camIndex,
                                          const SgmParams& sgmParams,
                                          const std::string& filepath,
                                          const ROI& roi);

/**
 * @brief Export a topographic cut of the given similarity volume to an Alembic file.
 * @param[in] in_volumeSim_hmh the similarity in host memory
 * @param[in] in_depthSimMapSgmUpscale_hmh the upscaled SGM depth/sim map
 * @param[in] mp the multi-view parameters
 * @param[in] camIndex the R cam global index
 * @param[in] refineParams the Refine parameters
 * @param[in] filepath the export filepath
 * @param[in] roi the 2d region of interest
 */
void exportSimilarityVolumeTopographicCut(const CudaHostMemoryHeap<TSimRefine, 3>& in_volumeSim_hmh,
                                          const CudaHostMemoryHeap<float2, 2>& in_depthSimMapSgmUpscale_hmh,
                                          const mvsUtils::MultiViewParams& mp,
                                          int camIndex,
                                          const RefineParams& refineParams,
                                          const std::string& filepath,
                                          const ROI& roi);

/**
 * @brief Export the given similarity volume to an Alembic file.
 */
void exportColorVolume(const CudaHostMemoryHeap<float4, 3>& in_volumeSim_hmh,
                       const std::vector<float>& in_depths,
                       int startDepth,
                       int nbDepths,
                       const mvsUtils::MultiViewParams& mp,
                       int camIndex,
                       int scale,
                       int step,
                       const std::string& filepath,
                       const ROI& roi);

#elif ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_VULKAN)

/**
 * @brief Export 9 similarity values over the entire depth in a CSV file.
 * @param[in] in_volumeSim_hmh the similarity in host memory
 * @param[in] in_depths the SGM depth list
 * @param[in] name the export name
 * @param[in] sgmParams the Semi Global Matching parameters
 * @param[in] filepath the export filepath
 * @param[in] roi the 2d region of interest
 */
void exportSimilaritySamplesCSV(const VulkanMemoryBase<TSim>& in_volumeSim_hmh,
                                const std::vector<float>& in_depths,
                                const std::string& name,
                                const SgmParams& sgmParams,
                                const std::string& filepath,
                                const ROI& roi);

/**
 * @brief Export 9 similarity values over the entire depth in a CSV file.
 * @param[in] in_volumeSim_hmh the similarity in host memory
 * @param[in] name the export name
 * @param[in] refineParams the Refine parameters
 * @param[in] filepath the export filepath
 * @param[in] roi the 2d region of interest
 */
void exportSimilaritySamplesCSV(const VulkanMemoryBase<TSimRefine>& in_volumeSim_hmh,
                                const std::string& name,
                                const RefineParams& refineParams,
                                const std::string& filepath,
                                const ROI& roi);

/**
 * @brief Export the given similarity volume to an Alembic file.
 * @param[in] in_volumeSim_hmh the similarity in host memory
 * @param[in] in_depths the SGM depth list
 * @param[in] mp the multi-view parameters
 * @param[in] camIndex the R cam global index
 * @param[in] sgmParams the Semi Global Matching parameters
 * @param[in] filepath the export filepath
 * @param[in] roi the 2d region of interest
 */
void exportSimilarityVolume(const VulkanMemoryBase<TSim>& in_volumeSim_hmh,
                            const std::vector<float>& in_depths,
                            const mvsUtils::MultiViewParams& mp,
                            int camIndex,
                            const SgmParams& sgmParams,
                            const std::string& filepath,
                            const ROI& roi);

/**
 * @brief Export a cross of the given similarity volume to an Alembic file.
 * @param[in] in_volumeSim_hmh the similarity in host memory
 * @param[in] in_depths the SGM depth list
 * @param[in] mp the multi-view parameters
 * @param[in] camIndex the R cam global index
 * @param[in] sgmParams the Semi Global Matching parameters
 * @param[in] filepath the export filepath
 * @param[in] roi the 2d region of interest
 */
void exportSimilarityVolumeCross(const VulkanMemoryBase<TSim>& in_volumeSim_hmh,
                                 const std::vector<float>& in_depths,
                                 const mvsUtils::MultiViewParams& mp,
                                 int camIndex,
                                 const SgmParams& sgmParams,
                                 const std::string& filepath,
                                 const ROI& roi);

/**
 * @brief Export a cross of the given similarity volume to an Alembic file.
 * @param[in] in_volumeSim_hmh the similarity in host memory
 * @param[in] in_depthSimMapSgmUpscale_hmh the upscaled SGM depth/sim map
 * @param[in] mp the multi-view parameters
 * @param[in] camIndex the R cam global index
 * @param[in] refineParams the Refine parameters
 * @param[in] filepath the export filepath
 * @param[in] roi the 2d region of interest
 */
void exportSimilarityVolumeCross(const VulkanMemoryBase<TSimRefine>& in_volumeSim_hmh,
                                 const VulkanMemoryBase<float2>& in_depthSimMapSgmUpscale_hmh,
                                 const mvsUtils::MultiViewParams& mp,
                                 int camIndex,
                                 const RefineParams& refineParams,
                                 const std::string& filepath,
                                 const ROI& roi);

/**
 * @brief Export a topographic cut of the given similarity volume to an Alembic file.
 * @param[in] in_volumeSim_hmh the similarity in host memory
 * @param[in] in_depths the SGM depth list
 * @param[in] mp the multi-view parameters
 * @param[in] camIndex the R cam global index
 * @param[in] sgmParams the Semi Global Matching parameters
 * @param[in] filepath the export filepath
 * @param[in] roi the 2d region of interest
 */
void exportSimilarityVolumeTopographicCut(const VulkanMemoryBase<TSim>& in_volumeSim_hmh,
                                          const std::vector<float>& in_depths,
                                          const mvsUtils::MultiViewParams& mp,
                                          int camIndex,
                                          const SgmParams& sgmParams,
                                          const std::string& filepath,
                                          const ROI& roi);

/**
 * @brief Export a topographic cut of the given similarity volume to an Alembic file.
 * @param[in] in_volumeSim_hmh the similarity in host memory
 * @param[in] in_depthSimMapSgmUpscale_hmh the upscaled SGM depth/sim map
 * @param[in] mp the multi-view parameters
 * @param[in] camIndex the R cam global index
 * @param[in] refineParams the Refine parameters
 * @param[in] filepath the export filepath
 * @param[in] roi the 2d region of interest
 */
void exportSimilarityVolumeTopographicCut(const VulkanMemoryBase<TSimRefine>& in_volumeSim_hmh,
                                          const VulkanMemoryBase<float2>& in_depthSimMapSgmUpscale_hmh,
                                          const mvsUtils::MultiViewParams& mp,
                                          int camIndex,
                                          const RefineParams& refineParams,
                                          const std::string& filepath,
                                          const ROI& roi);

/**
 * @brief Export the given similarity volume to an Alembic file.
 */
void exportColorVolume(const VulkanMemoryBase<float4>& in_volumeSim_hmh,
                       const std::vector<float>& in_depths,
                       int startDepth,
                       int nbDepths,
                       const mvsUtils::MultiViewParams& mp,
                       int camIndex,
                       int scale,
                       int step,
                       const std::string& filepath,
                       const ROI& roi);

#elif ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_METAL)

using namespace gpu;

/**
 * @brief Export 9 similarity values over the entire depth in a CSV file.
 * @param[in] in_volumeSim_hmh the similarity in host memory
 * @param[in] in_depths the SGM depth list
 * @param[in] name the export name
 * @param[in] sgmParams the Semi Global Matching parameters
 * @param[in] filepath the export filepath
 * @param[in] roi the 2d region of interest
 */
void exportSimilaritySamplesCSV(const MTLSharedResource<MTLTexture, TSim>& in_volumeSim_hmh,
                                const std::vector<float>& in_depths,
                                const std::string& name,
                                const SgmParams& sgmParams,
                                const std::string& filepath,
                                const ROI& roi);

/**
 * @brief Export 9 similarity values over the entire depth in a CSV file.
 * @param[in] in_volumeSim_hmh the similarity in host memory
 * @param[in] name the export name
 * @param[in] refineParams the Refine parameters
 * @param[in] filepath the export filepath
 * @param[in] roi the 2d region of interest
 */
void exportSimilaritySamplesCSV(const MTLSharedResource<MTLTexture, TSimRefine>& in_volumeSim_hmh,
                                const std::string& name,
                                const RefineParams& refineParams,
                                const std::string& filepath,
                                const ROI& roi);

/**
 * @brief Export the given similarity volume to an Alembic file.
 * @param[in] in_volumeSim_hmh the similarity in host memory
 * @param[in] in_depths the SGM depth list
 * @param[in] mp the multi-view parameters
 * @param[in] camIndex the R cam global index
 * @param[in] sgmParams the Semi Global Matching parameters
 * @param[in] filepath the export filepath
 * @param[in] roi the 2d region of interest
 */
void exportSimilarityVolume(const MTLSharedResource<MTLTexture, TSim>& in_volumeSim_hmh,
                            const std::vector<float>& in_depths,
                            const mvsUtils::MultiViewParams& mp,
                            int camIndex,
                            const SgmParams& sgmParams,
                            const std::string& filepath,
                            const ROI& roi);

/**
 * @brief Export a cross of the given similarity volume to an Alembic file.
 * @param[in] in_volumeSim_hmh the similarity in host memory
 * @param[in] in_depths the SGM depth list
 * @param[in] mp the multi-view parameters
 * @param[in] camIndex the R cam global index
 * @param[in] sgmParams the Semi Global Matching parameters
 * @param[in] filepath the export filepath
 * @param[in] roi the 2d region of interest
 */
void exportSimilarityVolumeCross(const MTLSharedResource<MTLTexture, TSim>& in_volumeSim_hmh,
                                 const std::vector<float>& in_depths,
                                 const mvsUtils::MultiViewParams& mp,
                                 int camIndex,
                                 const SgmParams& sgmParams,
                                 const std::string& filepath,
                                 const ROI& roi);

/**
 * @brief Export a cross of the given similarity volume to an Alembic file.
 * @param[in] in_volumeSim_hmh the similarity in host memory
 * @param[in] in_depthSimMapSgmUpscale_hmh the upscaled SGM depth/sim map
 * @param[in] mp the multi-view parameters
 * @param[in] camIndex the R cam global index
 * @param[in] refineParams the Refine parameters
 * @param[in] filepath the export filepath
 * @param[in] roi the 2d region of interest
 */
void exportSimilarityVolumeCross(const MTLSharedResource<MTLTexture, TSimRefine>& in_volumeSim_hmh,
                                 const MTLSharedResource<MTLTexture, float4>& in_depthSimMapSgmUpscale_hmh,
                                 const mvsUtils::MultiViewParams& mp,
                                 int camIndex,
                                 const RefineParams& refineParams,
                                 const std::string& filepath,
                                 const ROI& roi);

/**
 * @brief Export a topographic cut of the given similarity volume to an Alembic file.
 * @param[in] in_volumeSim_hmh the similarity in host memory
 * @param[in] in_depths the SGM depth list
 * @param[in] mp the multi-view parameters
 * @param[in] camIndex the R cam global index
 * @param[in] sgmParams the Semi Global Matching parameters
 * @param[in] filepath the export filepath
 * @param[in] roi the 2d region of interest
 */
void exportSimilarityVolumeTopographicCut(const MTLSharedResource<MTLTexture, TSim>& in_volumeSim_hmh,
                                          const std::vector<float>& in_depths,
                                          const mvsUtils::MultiViewParams& mp,
                                          int camIndex,
                                          const SgmParams& sgmParams,
                                          const std::string& filepath,
                                          const ROI& roi);

/**
 * @brief Export a topographic cut of the given similarity volume to an Alembic file.
 * @param[in] in_volumeSim_hmh the similarity in host memory
 * @param[in] in_depthSimMapSgmUpscale_hmh the upscaled SGM depth/sim map
 * @param[in] mp the multi-view parameters
 * @param[in] camIndex the R cam global index
 * @param[in] refineParams the Refine parameters
 * @param[in] filepath the export filepath
 * @param[in] roi the 2d region of interest
 */
void exportSimilarityVolumeTopographicCut(const MTLSharedResource<MTLTexture, TSimRefine>& in_volumeSim_hmh,
                                          const MTLSharedResource<MTLTexture, float4>& in_depthSimMapSgmUpscale_hmh,
                                          const mvsUtils::MultiViewParams& mp,
                                          int camIndex,
                                          const RefineParams& refineParams,
                                          const std::string& filepath,
                                          const ROI& roi);

/**
 * @brief Export the given similarity volume to an Alembic file.
 */
void exportColorVolume(const MTLSharedResource<MTLTexture, float4>& in_volumeSim_hmh,
                       const std::vector<float>& in_depths,
                       int startDepth,
                       int nbDepths,
                       const mvsUtils::MultiViewParams& mp,
                       int camIndex,
                       int scale,
                       int step,
                       const std::string& filepath,
                       const ROI& roi);

#else
    #error No backend for DepthMap computations selected!
#endif

}  // namespace depthMap
}  // namespace aliceVision

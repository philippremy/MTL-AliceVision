#pragma once

#include <cstdint>
#include <cstddef>

extern const uint32_t RGB2LAB_uchar[];
extern const size_t RGB2LAB_uchar_SIZE;

extern const uint32_t VolumeComputeSimiliarity_float[];
extern const size_t VolumeComputeSimiliarity_float_SIZE;

extern const uint32_t VolumeComputeSimiliarity_uchar[];
extern const size_t VolumeComputeSimiliarity_uchar_SIZE;

extern const uint32_t VolumeRefineSimilarity_float[];
extern const size_t VolumeRefineSimilarity_float_SIZE;

extern const uint32_t VolumeRefineSimilarity_half[];
extern const size_t VolumeRefineSimilarity_half_SIZE;

extern const uint32_t VolumeUpdateUninitialized_float[];
extern const size_t VolumeUpdateUninitialized_float_SIZE;

extern const uint32_t VolumeUpdateUninitialized_uchar[];
extern const size_t VolumeUpdateUninitialized_uchar_SIZE;

extern const uint32_t AggregateCostVolumeAtXInSlices_float[];
extern const size_t AggregateCostVolumeAtXInSlices_float_SIZE;

extern const uint32_t AggregateCostVolumeAtXInSlices_uchar[];
extern const size_t AggregateCostVolumeAtXInSlices_uchar_SIZE;

extern const uint32_t ComputeBestZInSlice_float[];
extern const size_t ComputeBestZInSlice_float_SIZE;

extern const uint32_t ComputeBestZInSlice_uint[];
extern const size_t ComputeBestZInSlice_uint_SIZE;

extern const uint32_t ComputeSgmUpscaledDepthPixSizeMap_bilinear[];
extern const size_t ComputeSgmUpscaledDepthPixSizeMap_bilinear_SIZE;

extern const uint32_t ComputeSgmUpscaledDepthPixSizeMap_nearestNeighbor[];
extern const size_t ComputeSgmUpscaledDepthPixSizeMap_nearestNeighbor_SIZE;

extern const uint32_t CreateMipmapLevel_float[];
extern const size_t CreateMipmapLevel_float_SIZE;

extern const uint32_t CreateMipmapLevel_half[];
extern const size_t CreateMipmapLevel_half_SIZE;

extern const uint32_t CreateMipmapLevel_uchar[];
extern const size_t CreateMipmapLevel_uchar_SIZE;

extern const uint32_t DepthSimMapComputeNormal[];
extern const size_t DepthSimMapComputeNormal_SIZE;

extern const uint32_t DepthSimMapCopyDepthOnly[];
extern const size_t DepthSimMapCopyDepthOnly_SIZE;

extern const uint32_t DepthThicknessSmoothThickness[];
extern const size_t DepthThicknessSmoothThickness_SIZE;

extern const uint32_t DownscaleWithGaussianBlur_float[];
extern const size_t DownscaleWithGaussianBlur_float_SIZE;

extern const uint32_t DownscaleWithGaussianBlur_half[];
extern const size_t DownscaleWithGaussianBlur_half_SIZE;

extern const uint32_t DownscaleWithGaussianBlur_uchar[];
extern const size_t DownscaleWithGaussianBlur_uchar_SIZE;

extern const uint32_t GetVolumeXZSlice_float[];
extern const size_t GetVolumeXZSlice_float_SIZE;

extern const uint32_t GetVolumeXZSlice_uchar[];
extern const size_t GetVolumeXZSlice_uchar_SIZE;

extern const uint32_t InitVolumeYSlice_float[];
extern const size_t InitVolumeYSlice_float_SIZE;

extern const uint32_t InitVolumeYSlice_uchar[];
extern const size_t InitVolumeYSlice_uchar_SIZE;

extern const uint32_t MapUpscale_float4[];
extern const size_t MapUpscale_float4_SIZE;

extern const uint32_t OptimizeDepthSimMap[];
extern const size_t OptimizeDepthSimMap_SIZE;

extern const uint32_t OptimizeGetOptDepthMapFromOptDepthSimMap[];
extern const size_t OptimizeGetOptDepthMapFromOptDepthSimMap_SIZE;

extern const uint32_t OptimizeVarLofLABtoW[];
extern const size_t OptimizeVarLofLABtoW_SIZE;

extern const uint32_t RefineBestDepth_float[];
extern const size_t RefineBestDepth_float_SIZE;

extern const uint32_t RefineBestDepth_half[];
extern const size_t RefineBestDepth_half_SIZE;

extern const uint32_t RetrieveBestDepth_float[];
extern const size_t RetrieveBestDepth_float_SIZE;

extern const uint32_t RetrieveBestDepth_uchar[];
extern const size_t RetrieveBestDepth_uchar_SIZE;

extern const uint32_t RGB2LAB_float[];
extern const size_t RGB2LAB_float_SIZE;

extern const uint32_t RGB2LAB_half[];
extern const size_t RGB2LAB_half_SIZE;


#define ALICEVISION_LOOKUP_SPIRV_MODULE(NAME) std::make_pair(NAME, NAME##_SIZE)


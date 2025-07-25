#pragma once

#include <cstdint>
#include <cstddef>

#include </Users/philippremy/CLionProjects/AV/Sources/Headers/AVDepthMapVulkanKernels/SPIRV/Generated/RGB2LAB_uchar.hpp>
#include </Users/philippremy/CLionProjects/AV/Sources/Headers/AVDepthMapVulkanKernels/SPIRV/Generated/VolumeComputeSimiliarity_float.hpp>
#include </Users/philippremy/CLionProjects/AV/Sources/Headers/AVDepthMapVulkanKernels/SPIRV/Generated/VolumeComputeSimiliarity_uchar.hpp>
#include </Users/philippremy/CLionProjects/AV/Sources/Headers/AVDepthMapVulkanKernels/SPIRV/Generated/VolumeRefineSimilarity_float.hpp>
#include </Users/philippremy/CLionProjects/AV/Sources/Headers/AVDepthMapVulkanKernels/SPIRV/Generated/VolumeRefineSimilarity_half.hpp>
#include </Users/philippremy/CLionProjects/AV/Sources/Headers/AVDepthMapVulkanKernels/SPIRV/Generated/VolumeUpdateUninitialized_float.hpp>
#include </Users/philippremy/CLionProjects/AV/Sources/Headers/AVDepthMapVulkanKernels/SPIRV/Generated/VolumeUpdateUninitialized_uchar.hpp>
#include </Users/philippremy/CLionProjects/AV/Sources/Headers/AVDepthMapVulkanKernels/SPIRV/Generated/AggregateCostVolumeAtXInSlices_float.hpp>
#include </Users/philippremy/CLionProjects/AV/Sources/Headers/AVDepthMapVulkanKernels/SPIRV/Generated/AggregateCostVolumeAtXInSlices_uchar.hpp>
#include </Users/philippremy/CLionProjects/AV/Sources/Headers/AVDepthMapVulkanKernels/SPIRV/Generated/ComputeBestZInSlice_float.hpp>
#include </Users/philippremy/CLionProjects/AV/Sources/Headers/AVDepthMapVulkanKernels/SPIRV/Generated/ComputeBestZInSlice_uint.hpp>
#include </Users/philippremy/CLionProjects/AV/Sources/Headers/AVDepthMapVulkanKernels/SPIRV/Generated/ComputeSgmUpscaledDepthPixSizeMap_bilinear.hpp>
#include </Users/philippremy/CLionProjects/AV/Sources/Headers/AVDepthMapVulkanKernels/SPIRV/Generated/ComputeSgmUpscaledDepthPixSizeMap_nearestNeighbor.hpp>
#include </Users/philippremy/CLionProjects/AV/Sources/Headers/AVDepthMapVulkanKernels/SPIRV/Generated/CreateMipmapLevel_float.hpp>
#include </Users/philippremy/CLionProjects/AV/Sources/Headers/AVDepthMapVulkanKernels/SPIRV/Generated/CreateMipmapLevel_half.hpp>
#include </Users/philippremy/CLionProjects/AV/Sources/Headers/AVDepthMapVulkanKernels/SPIRV/Generated/CreateMipmapLevel_uchar.hpp>
#include </Users/philippremy/CLionProjects/AV/Sources/Headers/AVDepthMapVulkanKernels/SPIRV/Generated/DepthSimMapComputeNormal.hpp>
#include </Users/philippremy/CLionProjects/AV/Sources/Headers/AVDepthMapVulkanKernels/SPIRV/Generated/DepthSimMapCopyDepthOnly.hpp>
#include </Users/philippremy/CLionProjects/AV/Sources/Headers/AVDepthMapVulkanKernels/SPIRV/Generated/DepthThicknessSmoothThickness.hpp>
#include </Users/philippremy/CLionProjects/AV/Sources/Headers/AVDepthMapVulkanKernels/SPIRV/Generated/DownscaleWithGaussianBlur_float.hpp>
#include </Users/philippremy/CLionProjects/AV/Sources/Headers/AVDepthMapVulkanKernels/SPIRV/Generated/DownscaleWithGaussianBlur_half.hpp>
#include </Users/philippremy/CLionProjects/AV/Sources/Headers/AVDepthMapVulkanKernels/SPIRV/Generated/DownscaleWithGaussianBlur_uchar.hpp>
#include </Users/philippremy/CLionProjects/AV/Sources/Headers/AVDepthMapVulkanKernels/SPIRV/Generated/GetVolumeXZSlice_float.hpp>
#include </Users/philippremy/CLionProjects/AV/Sources/Headers/AVDepthMapVulkanKernels/SPIRV/Generated/GetVolumeXZSlice_uchar.hpp>
#include </Users/philippremy/CLionProjects/AV/Sources/Headers/AVDepthMapVulkanKernels/SPIRV/Generated/InitVolumeYSlice_float.hpp>
#include </Users/philippremy/CLionProjects/AV/Sources/Headers/AVDepthMapVulkanKernels/SPIRV/Generated/InitVolumeYSlice_uchar.hpp>
#include </Users/philippremy/CLionProjects/AV/Sources/Headers/AVDepthMapVulkanKernels/SPIRV/Generated/MapUpscale_float4.hpp>
#include </Users/philippremy/CLionProjects/AV/Sources/Headers/AVDepthMapVulkanKernels/SPIRV/Generated/OptimizeDepthSimMap.hpp>
#include </Users/philippremy/CLionProjects/AV/Sources/Headers/AVDepthMapVulkanKernels/SPIRV/Generated/OptimizeGetOptDepthMapFromOptDepthSimMap.hpp>
#include </Users/philippremy/CLionProjects/AV/Sources/Headers/AVDepthMapVulkanKernels/SPIRV/Generated/OptimizeVarLofLABtoW.hpp>
#include </Users/philippremy/CLionProjects/AV/Sources/Headers/AVDepthMapVulkanKernels/SPIRV/Generated/RefineBestDepth_float.hpp>
#include </Users/philippremy/CLionProjects/AV/Sources/Headers/AVDepthMapVulkanKernels/SPIRV/Generated/RefineBestDepth_half.hpp>
#include </Users/philippremy/CLionProjects/AV/Sources/Headers/AVDepthMapVulkanKernels/SPIRV/Generated/RetrieveBestDepth_float.hpp>
#include </Users/philippremy/CLionProjects/AV/Sources/Headers/AVDepthMapVulkanKernels/SPIRV/Generated/RetrieveBestDepth_uchar.hpp>
#include </Users/philippremy/CLionProjects/AV/Sources/Headers/AVDepthMapVulkanKernels/SPIRV/Generated/RGB2LAB_float.hpp>
#include </Users/philippremy/CLionProjects/AV/Sources/Headers/AVDepthMapVulkanKernels/SPIRV/Generated/RGB2LAB_half.hpp>

#define ALICEVISION_LOOKUP_SPIRV_MODULE(NAME) std::make_pair(NAME, NAME##_SIZE)


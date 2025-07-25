# Headers
set(depthMap_files_headers
        ComputeOnMultiGPUs.hpp
        CustomPatchPatternParams.hpp
        DepthMapEstimator.hpp
        DepthMapParams.hpp
        DepthMapUtils.hpp
        NormalMapEstimator.hpp
        Refine.hpp
        RefineParams.hpp
        Sgm.hpp
        SgmDepthList.hpp
        SgmParams.hpp
        Tile.hpp
        VolumeIO.hpp
)

# Sources
set(depthMap_files_sources
        CustomPatchPatternParams.cpp
        DepthMapEstimator.cpp
        DepthMapUtils.cpp
        NormalMapEstimator.cpp
        Refine.cpp
        Sgm.cpp
        SgmDepthList.cpp
        VolumeIO.cpp
)

# Cuda Host Headers Only
set(depthMap_cuda_host_headers
        BufPtr.hpp
        CUDA/Host/LRUCameraCache.hpp
        CUDA/Host/LRUCache.hpp
        CUDA/Host/divUp.hpp
        CUDA/Host/memory.hpp
)

# Cuda Host Sources
set(depthMap_cuda_host_sources
        CUDA/ComputeOnMultiGPUs.cpp
        CUDA/Host/DeviceCache.hpp
        CUDA/Host/DeviceCache.cpp
        CUDA/Host/DeviceMipmapImage.hpp
        CUDA/Host/DeviceMipmapImage.cpp
        CUDA/Host/DeviceStreamManager.hpp
        CUDA/Host/DeviceStreamManager.cpp
        CUDA/Host/patchPattern.hpp
        CUDA/Host/patchPattern.cpp
        CUDA/Host/utils.hpp
        CUDA/Host/utils.cpp
)

# device CUDA Headers Only
set(depthMap_cuda_device_headers
        CUDA/Device/buffer.cuh
        CUDA/Device/color.cuh
        CUDA/Device/eig33.cuh
        CUDA/Device/matrix.cuh
        CUDA/Device/operators.cuh
        CUDA/Device/Patch.cuh
        CUDA/Device/SimStat.cuh
)

# device CUDA Sources
set(depthMap_cuda_device_sources
        CUDA/Device/DeviceCameraParams.hpp
        CUDA/Device/DeviceCameraParams.cu
        CUDA/Device/DevicePatchPattern.hpp
        CUDA/Device/DevicePatchPattern.cu
)

# imageProcessing CUDA Sources
set(depthMap_cuda_imageProcessing_sources
        CUDA/ImageProcessing/deviceGaussianFilter.hpp
        CUDA/ImageProcessing/deviceGaussianFilter.cu
        CUDA/ImageProcessing/deviceColorConversion.hpp
        CUDA/ImageProcessing/deviceColorConversion.cu
        CUDA/ImageProcessing/deviceMipmappedArray.hpp
        CUDA/ImageProcessing/deviceMipmappedArray.cu
)

# planeSweeping CUDA Headers Only
set(depthMap_cuda_planeSweeping_headers
        CUDA/PlaneSweeping/deviceDepthSimilarityMapKernels.cuh
        CUDA/PlaneSweeping/deviceSimilarityVolumeKernels.cuh
)

# planeSweeping CUDA Sources
set(depthMap_cuda_planeSweeping_sources
        CUDA/PlaneSweeping/similarity.hpp
        CUDA/PlaneSweeping/deviceDepthSimilarityMap.hpp
        CUDA/PlaneSweeping/deviceDepthSimilarityMap.cu
        CUDA/PlaneSweeping/deviceSimilarityVolume.hpp
        CUDA/PlaneSweeping/deviceSimilarityVolume.cu
)

set_source_files_properties(
        ${depthMap_cuda_host_headers}
        ${depthMap_cuda_device_headers}
        ${depthMap_cuda_planeSweeping_headers}
        PROPERTIES HEADER_FILE_ONLY true
)

source_group("AVDepthMap_CUDA_Host" FILES ${depthMap_cuda_host_headers} ${depthMap_cuda_host_sources})
source_group("AVDepthMap_CUDA_Device" FILES ${depthMap_cuda_device_headers} ${depthMap_cuda_device_sources})
source_group("AVDepthMap_CUDA_ImageProcessing" FILES ${depthMap_cuda_imageProcessing_sources})
source_group("AVDepthMap_CUDA_PlaneSweeping" FILES ${depthMap_cuda_planeSweeping_headers} ${depthMap_cuda_planeSweeping_sources})

# Cuda Sources
set(depthMap_cuda_files_sources
        ${depthMap_cuda_host_headers}
        ${depthMap_cuda_host_sources}
        ${depthMap_cuda_device_headers}
        ${depthMap_cuda_device_sources}
        ${depthMap_cuda_imageProcessing_sources}
        ${depthMap_cuda_planeSweeping_headers}
        ${depthMap_cuda_planeSweeping_sources}
)

# Vulkan Headers
set(depthMap_vulkan_headers
        Vulkan/DeviceCache.hpp
        Vulkan/DeviceCameraParams.hpp
        Vulkan/DeviceMipmapImage.hpp
        Vulkan/DivUp.hpp
        Vulkan/PatchPattern.hpp
        Vulkan/Types.hpp
)

# Vulkan Sources
set(depthMap_vulkan_sources
        Vulkan/ComputeOnMultiGPUs.cpp
        Vulkan/DeviceCache.cpp
        Vulkan/DeviceCameraParams.cpp
        Vulkan/DeviceGaussianArray.cpp
        Vulkan/DeviceMipmapImage.cpp
        Vulkan/DevicePatchPattern.cpp
        Vulkan/PatchPattern.cpp
)

# Metal Kernel Sources
set(depthMap_metal_kernel_sources
        Metal/AggregateCostVolumeAtXInSlices.metal
        Metal/ComputeBestZInSlice.metal
        Metal/ComputeSgmUpscaledDepthPixSizeMap_Bilinear.metal
        Metal/ComputeSgmUpscaledDepthPixSizeMap_NearestNeighbor.metal
        Metal/CreateMipmapLevel.metal
        Metal/DepthSimMapComputeNormal.metal
        Metal/DepthSimMapCopyDepthOnly.metal
        Metal/DepthThicknessSmoothThickness.metal
        Metal/DownscaleWithGaussianBlur.metal
        Metal/GetVolumeXZSlice.metal
        Metal/InitVolumeYSlice.metal
        Metal/MapUpscale.metal
        Metal/OptimizeDepthSimMap.metal
        Metal/OptimizeGetOptDepthMapFromOptDepthSimMap.metal
        Metal/OptimizeVarLofLABtoW.metal
        Metal/RefineBestDepth.metal
        Metal/RetrieveBestDepth.metal
        Metal/RGB2LAB.metal
        Metal/VolumeComputeSimiliarity.metal
        Metal/VolumeRefineSimiliarity.metal
        Metal/VolumeUpdateUnitialized.metal
)

# Metal Kernel Headers
set(depthMap_metal_kernel_headers
        Metal/Buffer.metal
        Metal/Color.metal
        Metal/Eig33.metal
        Metal/Math.metal
        Metal/Matrix.metal
        Metal/Patch.metal
        Metal/SimStat.metal
)

# Metal Sources
set(depthMap_metal_sources
        Metal/ComputeOnMultiGPUs.cpp
        Metal/DeviceCache.cpp
        Metal/DeviceCameraParams.cpp
        Metal/DeviceGaussianArray.cpp
        Metal/DeviceMipmapImage.cpp
        Metal/DevicePatchPattern.cpp
)

# Metal Headers
set(depthMap_metal_headers
        Metal/AggregateCostVolumeAtXInSlices_PushConstants.hpp
        Metal/ComputeBestZInSlice_PushConstants.hpp
        Metal/ComputeSgmUpscaledDepthPixSizeMap_Bilinear_PushConstants.hpp
        Metal/ComputeSgmUpscaledDepthPixSizeMap_NearestNeighbor_PushConstants.hpp
        Metal/CreateMipmapLevel_PushConstants.hpp
        Metal/DepthSimMapComputeNormal_PushConstants.hpp
        Metal/DepthSimMapCopyDepthOnly_PushConstants.hpp
        Metal/DepthThicknessSmoothThickness_PushConstants.hpp
        Metal/DeviceCache.hpp
        Metal/DeviceCameraParams.hpp
        Metal/DeviceGaussianArray.hpp
        Metal/DeviceMipmapImage.hpp
        Metal/DevicePatchPattern.hpp
        Metal/DownscaleWithGaussianBlur_PushConstants.hpp
        Metal/GetVolumeXZSlice_PushConstants.hpp
        Metal/InitVolumeYSlice_PushConstants.hpp
        Metal/LRUCache.hpp
        Metal/LRUCameraCache.hpp
        Metal/MapUpscale_PushConstants.hpp
        Metal/OptimizeDepthSimMap_PushConstants.hpp
        Metal/OptimizeGetOptDepthMapFromOptDepthSimMap_PushConstants.hpp
        Metal/OptimizeVarLofLABtoW_PushConstants.hpp
        Metal/RefineBestDepth_PushConstants.hpp
        Metal/RetrieveBestDepth_PushConstants.hpp
        Metal/Types.hpp
        Metal/VolumeComputeSimiliarity_PushConstants.hpp
        Metal/VolumeRefineSimiliarity_PushConstants.hpp
)


if(ALICEVISION_HAVE_CUDA)
    alicevision_add_library(AVDepthMap
            USE_CUDA
            SOURCES
                ${depthMap_files_sources}
                ${depthMap_cuda_files_sources}
            HEADERS
                ${depthMap_files_headers}
            PUBLIC_LINKS
                AVMVSData
                AVMVSUtils
                AVSystem
                assimp::assimp
            PRIVATE_LINKS
                AVGPU
                AVSfMData
                AVSfMDataIO
            PUBLIC_INCLUDE_DIRS
                ${CUDA_INCLUDE_DIRS}
    )

    # Add to Library List
    list(APPEND ALICEVISION_LIBRARIES AVDepthMap)

elseif(ALICEVISION_HAVE_VULKAN)
    # Add AVDepthMap Library
    alicevision_add_library(AVDepthMap
            SOURCES
                ${depthMap_files_sources}
                ${depthMap_vulkan_sources}
            HEADERS
                ${depthMap_files_headers}
                ${depthMap_vulkan_headers}
            PUBLIC_LINKS
                AVMVSData
                AVMVSUtils
                AVSystem
                assimp::assimp
            PRIVATE_LINKS
                AVDepthMapVulkanKernels
                AVGPU
                AVSfMData
                AVSfMDataIO
    )

elseif(ALICEVISION_HAVE_METAL)
    # Add Metal Kernel Library
    alicevision_add_kernel_library(AVDepthMapMetalKernels
            LANGUAGE Metal
            SOURCES
                "${depthMap_metal_kernel_sources}"
            INCLUDE_DIRS
                "${CMAKE_SOURCE_DIR}/Sources/Headers"
    )
    # Add AVDepthMap Library
    alicevision_add_library(AVDepthMap
            SOURCES
            ${depthMap_files_sources}
            ${depthMap_metal_sources}
            HEADERS
            ${depthMap_files_headers}
            ${depthMap_metal_headers}
            PUBLIC_LINKS
            AVMVSData
            AVMVSUtils
            AVSystem
            assimp::assimp
            PRIVATE_LINKS
            AVGPU
            AVSfMData
            AVSfMDataIO
    )
    add_dependencies(AVDepthMap AVDepthMapMetalKernels)
    target_embed_metal_shader_libraries(AVDepthMap AVDepthMapMetalKernels)
else()
    message(WARNING "[AliceVision] No DepthMap backend is enabled. AVDepthMap will not be build.")
endif()

# target_compile_definitions(aliceVision_depthMap PUBLIC TSIM_USE_FLOAT)
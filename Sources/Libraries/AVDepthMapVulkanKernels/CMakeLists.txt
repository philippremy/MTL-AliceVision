# GLSL Headers
set(depthMap_glsl_headers
        color.comp
        color_float.comp
        color_half.comp
        definitions.comp
        DeviceCameraParams.comp
        DevicePatchPattern.comp
        Eig33.comp
        Matrix.comp
        Patch.comp
        ROI.comp
        SimStat.comp
        Stat3D.comp
)

# GLSL Sources
set(depthMap_glsl_sources
        RGB2LAB_uchar.comp
        VolumeComputeSimiliarity_float.comp
        VolumeComputeSimiliarity_uchar.comp
        VolumeRefineSimilarity_float.comp
        VolumeRefineSimilarity_half.comp
        VolumeUpdateUninitialized_float.comp
        VolumeUpdateUninitialized_uchar.comp
        AggregateCostVolumeAtXInSlices_float.comp
        AggregateCostVolumeAtXInSlices_uchar.comp
        ComputeBestZInSlice_float.comp
        ComputeBestZInSlice_uint.comp
        ComputeSgmUpscaledDepthPixSizeMap_bilinear.comp
        ComputeSgmUpscaledDepthPixSizeMap_nearestNeighbor.comp
        CreateMipmapLevel_float.comp
        CreateMipmapLevel_half.comp
        CreateMipmapLevel_uchar.comp
        DepthSimMapComputeNormal.comp
        DepthSimMapCopyDepthOnly.comp
        DepthThicknessSmoothThickness.comp
        DownscaleWithGaussianBlur_float.comp
        DownscaleWithGaussianBlur_half.comp
        DownscaleWithGaussianBlur_uchar.comp
        GetVolumeXZSlice_float.comp
        GetVolumeXZSlice_uchar.comp
        InitVolumeYSlice_float.comp
        InitVolumeYSlice_uchar.comp
        MapUpscale_float4.comp
        OptimizeDepthSimMap.comp
        OptimizeGetOptDepthMapFromOptDepthSimMap.comp
        OptimizeVarLofLABtoW.comp
        RefineBestDepth_float.comp
        RefineBestDepth_half.comp
        RetrieveBestDepth_float.comp
        RetrieveBestDepth_uchar.comp
        RGB2LAB_float.comp
        RGB2LAB_half.comp
)

if(ALICEVISION_HAVE_VULKAN)
    # Add AVDepthMapVulkanKernels Library
    alicevision_add_kernel_library(AVDepthMapVulkanKernels
            LANGUAGE GLSL
            SOURCES
            ${depthMap_glsl_sources}
            HEADERS
            ${depthMap_glsl_headers}
            INCLUDE_DIRS
            "${ALICEVISION_INCLUDE_DIR}/AVDepthMapVulkanKernels"
            OUTPUT_HEADER
            "AVDepthMapVulkanKernels.hpp"
            OUTPUT_DIR
            "SPIRV"
    )

    if(APPLE)
        target_link_libraries(AVDepthMapVulkanKernels "-framework CoreFoundation")
    endif()
endif()
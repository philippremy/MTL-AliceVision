# Headers
set(depthMap_files_headers
    BufPtr.hpp
    computeOnMultiGPUs.hpp
    CustomPatchPatternParams.hpp
    DepthMapEstimator.hpp
    DepthMapParams.hpp
    depthMapUtils.hpp
    NormalMapEstimator.hpp
    Refine.hpp
    RefineParams.hpp
    Sgm.hpp
    SgmDepthList.hpp
    SgmParams.hpp
    Tile.hpp
    volumeIO.hpp
	Metal/util/KernelArgs.hpp
	Metal/util/MetalAddressSpace.hpp
	Metal/util/MetalTypes.hpp
	Metal/util/MTLMipmappedTexture.hpp
)

# Sources
set(depthMap_files_sources
    computeOnMultiGPUs.cpp
    CustomPatchPatternParams.cpp
    DepthMapEstimator.cpp
    depthMapUtils.cpp
    NormalMapEstimator.cpp
    Refine.cpp
    Sgm.cpp
    SgmDepthList.cpp
    volumeIO.cpp
)

# Metal Host Headers Only
set(depthMap_mtl_host_headers
	Metal/host/DeviceCache.hpp
	Metal/host/DeviceCommandManager.hpp
	Metal/host/DeviceMipmapImage.hpp
	Metal/host/divUp.hpp
	Metal/host/LRUCache.hpp
	Metal/host/LRUCameraCache.hpp
	Metal/host/memory.hpp
	Metal/host/patchPattern.hpp
	Metal/host/utils.hpp
	Metal/imageProcessing/deviceGaussianFilter_host.hpp
	Metal/imageProcessing/deviceColorConversion_host.hpp
	Metal/imageProcessing/deviceMipmappedArray_host.hpp
	Metal/planeSweeping/deviceDepthSimilarityMap_host.hpp
	Metal/planeSweeping/deviceSimilarityVolume_host.hpp
)

# Metal Host Sources
set(depthMap_mtl_host_sources
    Metal/host/DeviceCache.cpp
	Metal/host/DeviceCommandManager.cpp
	Metal/host/DeviceManager.cpp
    Metal/host/DeviceMipmapImage.cpp
    Metal/host/patchPattern.cpp
    Metal/host/utils.cpp
	Metal/imageProcessing/deviceGaussianFilter_host.cpp
	Metal/imageProcessing/deviceColorConversion_host.cpp
	Metal/imageProcessing/deviceMipmappedArray_host.cpp
	Metal/planeSweeping/deviceDepthSimilarityMap_host.cpp
	Metal/planeSweeping/deviceSimilarityVolume_host.cpp
)

# device Metal Headers Only
set(depthMap_mtl_device_headers
	Metal/device/buffer.hpp
	Metal/device/color.hpp
	Metal/device/DeviceCameraParams.hpp
	Metal/device/DeviceGaussianFilter.hpp
	Metal/device/DevicePatchPattern.hpp
	Metal/device/eig33.hpp
	Metal/device/math.hpp
	Metal/device/matrix.hpp
	Metal/device/operators.hpp
	Metal/device/Patch.hpp
	Metal/device/SimStat.hpp
	Metal/util/KernelArgs.hpp
	Metal/util/MetalAddressSpace.hpp
	Metal/util/MetalTypes.hpp
	Metal/util/MTLMipmappedTexture.hpp
)

# imageProcessing Metal Sources
set(depthMap_mtl_imageProcessing_sources
    Metal/imageProcessing/deviceGaussianFilter.metal
    Metal/imageProcessing/deviceColorConversion.metal
    Metal/imageProcessing/deviceMipmappedArray.metal
)

# planeSweeping Metal Sources
set(depthMap_mtl_planeSweeping_sources
	Metal/planeSweeping/deviceDepthSimilarityMapKernels.metal
	Metal/planeSweeping/deviceSimilarityVolumeKernels.metal
)

set_source_files_properties(
    ${depthMap_mtl_host_headers}
	${depthMap_mtl_device_headers}
    PROPERTIES HEADER_FILE_ONLY true
)

source_group("AVDepthMapMTLHost" FILES ${depthMap_mtl_host_headers} ${depthMap_mtl_host_sources})
source_group("AVDepthMapMTLDevice" FILES ${depthMap_mtl_device_headers} ${depthMap_mtl_device_sources})
source_group("AVDepthMapMTLImageProcessing" FILES ${depthMap_mtl_imageProcessing_sources})
source_group("AVDepthMapMTLPlaneSweeping" FILES ${depthMap_mtl_planeSweeping_sources})

# Cuda Sources
set(depthMap_mtl_files_sources
    ${depthMap_mtl_imageProcessing_sources}
    ${depthMap_mtl_planeSweeping_sources}
)

alicevision_add_kernel_library(AVDepthMapMTLKernels
		LANGUAGE Metal
		SOURCES
			${depthMap_mtl_files_sources}
		INCLUDE_DIRS
			"${CMAKE_SOURCE_DIR}/Sources/Headers"
)

alicevision_add_library(AVDepthMap
    SOURCES
        ${depthMap_files_sources}
		${depthMap_mtl_host_sources}
	HEADERS
		${depthMap_files_headers}
		${depthMap_mtl_host_headers}
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

add_dependencies(AVDepthMap AVDepthMapMTLKernels)
target_embed_metal_shader_libraries(AVDepthMap AVDepthMapMTLKernels)

# NOTE: Uncomment both!
# target_compile_definitions(AVDepthMap PUBLIC TSIM_USE_FLOAT)
# target_compile_definitions(AVDepthMapMTLKernels PUBLIC TSIM_USE_FLOAT)

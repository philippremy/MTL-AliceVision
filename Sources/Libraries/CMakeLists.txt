# Common modules

add_subdirectory(AVImage)
add_subdirectory(AVNumeric)
add_subdirectory(AVSystem)
add_subdirectory(AVCMDLine)
add_subdirectory(AVSTL)
add_subdirectory(AVUtils)
add_subdirectory(AVGPU)

# Third Party Modules
add_subdirectory(VLFeat)
add_subdirectory(MeshSDFilter)

# SfM modules

add_subdirectory(AVCamera)
add_subdirectory(AVColorHarmonization)
add_subdirectory(AVDataIO)
add_subdirectory(AVFeature)
add_subdirectory(AVFeatureEngine)
add_subdirectory(AVGeometry)
add_subdirectory(AVGraph)
add_subdirectory(AVImageMatching)
add_subdirectory(AVLinearProgramming)
add_subdirectory(AVLensCorrectionProfile)
add_subdirectory(AVLocalization)
add_subdirectory(AVMatching)
add_subdirectory(AVMatchingImageCollection)
add_subdirectory(AVMultiview)
add_subdirectory(AVMultiviewTestData)
add_subdirectory(AVRig)
add_subdirectory(AVRobustEstimation)
add_subdirectory(AVSensorDB)
add_subdirectory(AVLInfinityCV)
add_subdirectory(AVSfMData)
add_subdirectory(AVSfMBundle)
add_subdirectory(AVSfM)
add_subdirectory(AVSfMDataIO)
add_subdirectory(AVTrack)
add_subdirectory(AVVoctree)
add_subdirectory(AVCalibration)
if (ALICEVISION_HAVE_OPENCV)
    add_subdirectory(AVImageMasking)
    add_subdirectory(AVKeyframe)
endif()
add_subdirectory(AVPanorama)
add_subdirectory(AVHDR)

# MVS modules

add_subdirectory(AVMesh)
add_subdirectory(AVMVSData)
add_subdirectory(AVMVSUtils)
add_subdirectory(AVFuseCut)

# Helper modules
add_subdirectory(AVKVLD)
add_subdirectory(AVSfMMVSUtils)
add_subdirectory(AVSfMColorHarmonize)

if(ALICEVISION_HAVE_CUDA OR ALICEVISION_HAVE_VULKAN OR ALICEVISION_HAVE_METAL)
    if(ALICEVISION_HAVE_VULKAN)
        add_subdirectory(AVDepthMapVulkanKernels)
    endif()
    add_subdirectory(AVDepthMap)
endif()

if (ALICEVISION_HAVE_ONNX)
    add_subdirectory(AVSegmentation)
endif()

add_subdirectory(AVSfMUtils)

if (ALICEVISION_HAVE_OPENCV)
    add_subdirectory(AVPhotometricStereo)
    if (ALICEVISION_HAVE_ONNX)
        add_subdirectory(AVSphereDetection)
    endif()
    add_subdirectory(AVLightingEstimation)
endif()

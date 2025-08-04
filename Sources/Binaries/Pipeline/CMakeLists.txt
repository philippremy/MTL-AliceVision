## AliceVision
## Pipeline software

# Software PROPERTY FOLDER is 'Software/Pipeline'
set(FOLDER_SOFTWARE_PIPELINE "Software/Pipeline")

### SfM software
# Intrinsic image analysis and SfMData container initialization
alicevision_add_software(AVCameraInit
        SOURCE main_cameraInit.cpp
        FOLDER ${FOLDER_SOFTWARE_PIPELINE}
        LINKS
            AVSystem
            AVCMDLine
            AVSensorDB
            AVImage
            AVFeature
            AVSfMData
            AVSfMDataIO
            AVLensCorrectionProfile
            Boost::program_options
            Boost::boost
)

# Image processing
if (ALICEVISION_HAVE_OPENCV)
    alicevision_add_software(AVImageMasking
            SOURCE main_imageMasking.cpp
            FOLDER ${FOLDER_SOFTWARE_PIPELINE}
            LINKS AVImage
                  AVImageMasking
                  AVSystem
                  AVCMDLine
                  AVSfMData
                  AVSfMDataIO
                  ${OpenCV_LIBS}
                  ${Boost_LIBRARIES}
    )
endif()

# Feature extraction
alicevision_add_software(AVFeatureExtraction
        SOURCE main_featureExtraction.cpp
        FOLDER ${FOLDER_SOFTWARE_PIPELINE}
        LINKS AVSystem
              AVCMDLine
              AVGPU
              AVImage
              AVFeature
              AVFeatureEngine
              AVMultiview
              AVSfMData
              AVSfMDataIO
              VLFeat
              Boost::program_options
              Boost::boost
              Boost::timer
)

# Image matching
# - generate the image pair lists
alicevision_add_software(AVImageMatching
        SOURCE main_imageMatching.cpp
        FOLDER ${FOLDER_SOFTWARE_PIPELINE}
        LINKS AVSystem
              AVCMDLine
              AVImageMatching
              AVMatchingImageCollection
              AVSfM
              AVSfMData
              AVSfMDataIO
              AVVoctree
              Boost::program_options
)

# Feature matching
# - putative matches and geometric filtered matches
alicevision_add_software(AVFeatureMatching
        SOURCE main_featureMatching.cpp
        FOLDER ${FOLDER_SOFTWARE_PIPELINE}
        LINKS AVSystem
              AVCMDLine
              AVFeature
              AVMultiview
              AVMatchingImageCollection
              AVSfM
              AVSfMData
              AVSfMDataIO
              Boost::program_options
)

# Track builder
alicevision_add_software(AVTracksBuilding
        SOURCE main_tracksBuilding.cpp
        FOLDER ${FOLDER_SOFTWARE_PIPELINE}
        LINKS AVSystem
              AVCMDLine
              AVFeature
              AVSfM
              AVSfMData
              AVTrack
              Boost::program_options
              Boost::json
)

# Relative pose
alicevision_add_software(AVRelativePoseEstimating
        SOURCE main_relativePoseEstimating.cpp
        FOLDER ${FOLDER_SOFTWARE_PIPELINE}
        LINKS AVSystem
              AVCMDLine
              AVFeature
              AVSfM
              AVSfMData
              AVTrack
              Boost::program_options
              Boost::json
)

# Bootstrapping SfM
alicevision_add_software(AVSfMBootstraping
        SOURCE main_sfmBootstraping.cpp
        FOLDER ${FOLDER_SOFTWARE_PIPELINE}
        LINKS AVSystem
                AVCMDLine
                AVFeature
                AVSfM
                AVSfMData
                AVTrack
                AVDataIO
                AVMesh
                Boost::program_options
                Boost::json
)

# Expanding SfM
alicevision_add_software(AVSfMExpanding
        SOURCE main_sfmExpanding.cpp
        FOLDER ${FOLDER_SOFTWARE_PIPELINE}
        LINKS AVSystem
                AVCMDLine
                AVFeature
                AVSfM
                AVSfMData
                AVTrack
                AVDataIO
                AVMesh
                Boost::program_options
                Boost::json
)

# Global Rotation Estimating
alicevision_add_software(AVGlobalRotationEstimating
        SOURCE main_globalRotationEstimating.cpp
        FOLDER ${FOLDER_SOFTWARE_PIPELINE}
        LINKS AVSystem
              AVCMDLine
              AVFeature
              AVSfM
              AVSfMData
              AVTrack
              AVDataIO
              Boost::program_options
              Boost::json
)

# Incremental / Sequential SfM
alicevision_add_software(AVIncrementalSfM
        SOURCE main_incrementalSfM.cpp
        FOLDER ${FOLDER_SOFTWARE_PIPELINE}
        LINKS AVSystem
              AVCMDLine
              AVImage
              AVFeature
              AVSfM
              AVSfMData
              AVSfMDataIO
              Boost::program_options
)

# Incremental SFM for pure rotation
alicevision_add_software(AVNodalSfM
        SOURCE main_nodalSfM.cpp
        FOLDER ${FOLDER_SOFTWARE_PIPELINE}
        LINKS AVSystem
              AVCMDLine
              AVImage
              AVFeature
              AVSfM
              AVSfMData
              AVSfMDataIO
              Boost::program_options
)

# SfM Triangulation
alicevision_add_software(AVSfMTriangulation
        SOURCE main_sfmTriangulation.cpp
        FOLDER ${FOLDER_SOFTWARE_PIPELINE}
        LINKS AVSystem
              AVCMDLine
              AVImage
              AVFeature
              AVSfM
              AVSfMData
              AVSfMDataIO
              Boost::program_options
)

# Global SfM
alicevision_add_software(AVGlobalSfM
        SOURCE main_globalSfM.cpp
        FOLDER ${FOLDER_SOFTWARE_PIPELINE}
        LINKS AVSystem
              AVCMDLine
              AVImage
              AVFeature
              AVSfM
              AVSfMData
              AVSfMDataIO
              Boost::program_options
)

# Compute structure from known camera poses
alicevision_add_software(AVComputeStructureFromKnownPoses
        SOURCE main_computeStructureFromKnownPoses.cpp
        FOLDER ${FOLDER_SOFTWARE_PIPELINE}
        LINKS AVSystem
              AVCMDLine
              AVFeature
              AVSfM
              AVSfMData
              AVSfMDataIO
              Boost::program_options
)

# Calibrate a camera
if (ALICEVISION_HAVE_OPENCV)
    alicevision_add_software(AVCameraCalibration
            SOURCE main_cameraCalibration.cpp
            FOLDER ${FOLDER_SOFTWARE_PIPELINE}
            LINKS AVDataIO
                  AVImage
                  AVCalibration
                  AVSystem
                  AVCMDLine
                  Boost::program_options
                  Boost::boost
    )
endif()

alicevision_add_software(AVDistortionCalibration
        SOURCE main_distortionCalibration.cpp
        FOLDER ${FOLDER_SOFTWARE_PIPELINE}
        LINKS AVDataIO
              AVImage
              AVCalibration
              AVSystem
              AVCMDLine
              AVSfMDataIO
              Boost::program_options
              Boost::boost
              Boost::json
)

alicevision_add_software(AVCheckerboardCalibration
        SOURCE main_checkerboardCalibration.cpp
        FOLDER ${FOLDER_SOFTWARE_PIPELINE}
        LINKS AVDataIO
              AVImage
              AVCalibration
              AVSystem
              AVCMDLine
              AVSfMDataIO
              AVMultiview
              AVSfM
              Boost::program_options
              Boost::json
              Boost::boost
)

# Calibrate a rig
alicevision_add_software(AVRigCalibration
        SOURCE main_rigCalibration.cpp
        FOLDER ${FOLDER_SOFTWARE_PIPELINE}
        LINKS AVRig
              AVLocalization
              AVDataIO
              AVImage
              AVFeature
              AVCMDLine
              Boost::program_options
              Boost::boost
              Boost::timer
)

alicevision_add_software(AVCheckerboardDetection
        SOURCE main_checkerboardDetection.cpp
        FOLDER ${FOLDER_SOFTWARE_PIPELINE}
        LINKS AVDataIO
              AVImage
              AVCalibration
              AVSystem
              AVSfMDataIO
              AVCMDLine
              Boost::program_options
              Boost::boost
              Boost::json
)

if (ALICEVISION_HAVE_CCTAG)
    target_link_libraries(AVRigCalibration_exe PUBLIC CCTag::CCTag)
endif()

# Localize a camera
alicevision_add_software(AVCameraLocalization
        SOURCE main_cameraLocalization.cpp
        FOLDER ${FOLDER_SOFTWARE_PIPELINE}
        LINKS AVLocalization
              AVDataIO
              AVImage
              AVFeature
              AVCMDLine
              Boost::program_options
              Boost::boost
              Boost::timer
)

if (ALICEVISION_HAVE_CCTAG)
    target_link_libraries(AVCameraLocalization_exe PUBLIC CCTag::CCTag)
endif()

# Localize a rig
alicevision_add_software(AVRigLocalization
        SOURCE main_rigLocalization.cpp
        FOLDER ${FOLDER_SOFTWARE_PIPELINE}
        LINKS AVLocalization
              AVDataIO
              AVRig
              AVImage
              AVFeature
              AVCMDLine
              Boost::program_options
              Boost::boost
              Boost::timer
)

if (ALICEVISION_HAVE_CCTAG)
    target_link_libraries(AVRigLocalization_exe PUBLIC CCTag::CCTag)
endif()

# Prepare dense scene for MVS
alicevision_add_software(AVPrepareDenseScene
        SOURCE main_prepareDenseScene.cpp
        FOLDER ${FOLDER_SOFTWARE_PIPELINE}
        LINKS AVSystem
              AVCMDLine
              AVImage
              AVFeature
              AVSfMData
              AVSfMDataIO
              Boost::program_options
              Boost::boost
              Boost::timer
)

# Panorama
alicevision_add_software(AVPanoramaPrepareImages
        SOURCE main_panoramaPrepareImages.cpp
        FOLDER ${FOLDER_SOFTWARE_PIPELINE}
        LINKS AVSystem
              AVCMDLine
              AVSensorDB
              AVImage
              AVFeature
              AVSfMData
              AVSfMDataIO
              Boost::program_options
              Boost::boost
)

alicevision_add_software(AVPanoramaEstimation
        SOURCE main_panoramaEstimation.cpp
        FOLDER ${FOLDER_SOFTWARE_PIPELINE}
        LINKS AVSystem
              AVCMDLine
              AVImage
              AVFeature
              AVSfM
              AVSfMData
              AVSfMDataIO
              ${Boost_LIBRARIES}
)
alicevision_add_software(AVPanoramaWarping
        SOURCE main_panoramaWarping.cpp
        FOLDER ${FOLDER_SOFTWARE_PIPELINE}
        LINKS AVSystem
              AVCMDLine
              AVImage
              AVFeature
              AVSfM
              AVSfMData
              AVSfMDataIO
              AVPanorama
              ${Boost_LIBRARIES}
)
alicevision_add_software(AVPanoramaMerging
        SOURCE main_panoramaMerging.cpp
        FOLDER ${FOLDER_SOFTWARE_PIPELINE}
        LINKS AVSystem
              AVCMDLine
              AVImage
              AVSfMData
              AVSfMDataIO
              AVPanorama
              ${Boost_LIBRARIES}
)
alicevision_add_software(AVPanoramaPostProcessing
        SOURCE main_panoramaPostProcessing.cpp
        FOLDER ${FOLDER_SOFTWARE_PIPELINE}
        LINKS AVSystem
              AVCMDLine
              AVImage
              ${Boost_LIBRARIES}
              ${OpenCV_LIBS}
)
alicevision_add_software(AVPanoramaCompositing
        SOURCE main_panoramaCompositing.cpp
        FOLDER ${FOLDER_SOFTWARE_PIPELINE}
        LINKS AVSystem
              AVCMDLine
              AVImage
              AVSfMData
              AVSfMDataIO
              AVPanorama
              ${Boost_LIBRARIES}
)
alicevision_add_software(AVPanoramaSeams
        SOURCE main_panoramaSeams.cpp
        FOLDER ${FOLDER_SOFTWARE_PIPELINE}
        LINKS AVSystem
              AVCMDLine
              AVImage
              AVSfMData
              AVSfMDataIO
              AVPanorama
              ${Boost_LIBRARIES}
)
alicevision_add_software(AVPanoramaInit
        SOURCE main_panoramaInit.cpp
        FOLDER ${FOLDER_SOFTWARE_PIPELINE}
        LINKS AVSystem
              AVCMDLine
              AVImage
              AVFeature
              AVSfM
              AVSfMData
              AVSfMDataIO
              ${Boost_LIBRARIES}
)

if(ALICEVISION_HAVE_CUDA OR ALICEVISION_HAVE_VULKAN OR ALICEVISION_HAVE_METAL)
    # Depth Map Estimation
    alicevision_add_software(AVDepthMapEstimation
            SOURCE main_depthMapEstimation.cpp
            FOLDER ${FOLDER_SOFTWARE_PIPELINE}
            LINKS AVSystem
                  AVCMDLine
                  AVGPU
                  AVMVSData
                  AVMVSUtils
                  AVDepthMap
                  AVSfMData
                  AVSfMDataIO
                  Boost::program_options
    )

    # Depth Map Filtering
    alicevision_add_software(AVDepthMapFiltering
            SOURCE main_depthMapFiltering.cpp
            FOLDER ${FOLDER_SOFTWARE_PIPELINE}
            LINKS AVSystem
                  AVCMDLine
                  AVMVSData
                  AVMVSUtils
                  AVFuseCut
                  AVDepthMap
                  AVSfMData
                  AVSfMDataIO
                  Boost::program_options
    )
endif()

# Meshing
alicevision_add_software(AVMeshing
        SOURCE main_meshing.cpp
        FOLDER ${FOLDER_SOFTWARE_PIPELINE}
        LINKS AVSystem
              AVCMDLine
              AVMVSData
              AVMVSUtils
              AVMesh
              AVFuseCut
              AVSfMData
              AVSfMDataIO
              Boost::program_options
)

# Lidar meshing
alicevision_add_software(AVLidarMeshing
        SOURCE main_lidarMeshing.cpp
        FOLDER ${FOLDER_SOFTWARE_PIPELINE}
        LINKS AVSystem
              AVCMDLine
              AVMVSData
              AVMVSUtils
              AVMesh
              AVFuseCut
              AVSfMData
              AVSfMDataIO
              Boost::program_options
              Boost::json
)

# Lidar merging
alicevision_add_software(AVLidarMerging
        SOURCE main_lidarMerging.cpp
        FOLDER ${FOLDER_SOFTWARE_PIPELINE}
        LINKS AVSystem
              AVCMDLine
              AVMVSData
              AVMVSUtils
              AVMesh
              AVFuseCut
              AVSfMData
              AVSfMDataIO
              Boost::program_options
              Boost::json
)

# Lidar decimation
alicevision_add_software(AVLidarDecimating
        SOURCE main_lidarDecimating.cpp
        FOLDER ${FOLDER_SOFTWARE_PIPELINE}
        LINKS AVSystem
              AVCMDLine
              AVMVSData
              AVMVSUtils
              AVMesh
              AVFuseCut
              AVSfMData
              AVSfMDataIO
              Boost::program_options
              Boost::json
              OpenMeshCore
              OpenMeshTools
)

# Mesh Denoising
alicevision_add_software(AVMeshDenoising
        SOURCE main_meshDenoising.cpp
        FOLDER ${FOLDER_SOFTWARE_PIPELINE}
        LINKS AVSystem
              AVCMDLine
              AVMVSUtils
              MeshSDLibrary
              Eigen3::Eigen
              Boost::program_options
)

# Mesh Decimate
alicevision_add_software(AVMeshDecimate
        SOURCE main_meshDecimate.cpp
        FOLDER ${FOLDER_SOFTWARE_PIPELINE}
        LINKS AVSystem
              AVCMDLine
              AVMVSUtils
              OpenMeshCore
              OpenMeshTools
              Boost::program_options
)

# Mesh Filtering
alicevision_add_software(AVMeshFiltering
        SOURCE main_meshFiltering.cpp
        FOLDER ${FOLDER_SOFTWARE_PIPELINE}
        LINKS AVSystem
              AVCMDLine
              AVMVSUtils
              AVMesh
              Boost::program_options
)

# Mesh Masking
alicevision_add_software(AVMeshMasking
        SOURCE main_meshMasking.cpp
        FOLDER ${FOLDER_SOFTWARE_PIPELINE}
        LINKS AVSystem
              AVCMDLine
              AVMVSUtils
              AVMesh
              AVSfMData
              AVSfMDataIO
              AVSfMMVSUtils
              Boost::program_options
)

# Mesh Resampling
alicevision_add_software(AVMeshResampling
        SOURCE main_meshResampling.cpp
        FOLDER ${FOLDER_SOFTWARE_PIPELINE}
        LINKS AVSystem
              AVCMDLine
              AVMVSUtils
              Geogram::geogram
              Boost::program_options
)

# Texturing
alicevision_add_software(AVTexturing
        SOURCE main_texturing.cpp
        FOLDER ${FOLDER_SOFTWARE_PIPELINE}
        LINKS AVSystem
              AVCMDLine
              AVMVSData
              AVSfMMVSUtils
              AVMVSUtils
              AVMesh
              AVSfMData
              AVSfMDataIO
              Boost::program_options
)

if (ALICEVISION_HAVE_OPENCV)
    # Photometric Stereo

        alicevision_add_software(AVPhotometricStereo
                SOURCE main_photometricStereo.cpp
                FOLDER ${FOLDER_SOFTWARE_PIPELINE}
                LINKS AVPhotometricStereo
                      AVCMDLine
                      AVSystem
                      AVMVSData
                      AVMVSUtils
                      AVSfMData
                      AVSfMDataIO
                      Boost::program_options
        )

        alicevision_add_software(AVNormalIntegration
                SOURCE main_normalIntegration.cpp
                FOLDER ${FOLDER_SOFTWARE_PIPELINE}
                LINKS AVPhotometricStereo
                      AVCMDLine
                      AVSystem
                      AVMVSData
                      AVMVSUtils
                      AVSfMData
                      AVSfMDataIO
                      Boost::program_options
        )

    if (ALICEVISION_HAVE_ONNX)
        # SphereDetection
        alicevision_add_software(AVSphereDetection
                SOURCE main_sphereDetection.cpp
                FOLDER ${FOLDER_SOFTWARE_PIPELINE}
                LINKS AVImage
                      AVCMDLine
                      AVSystem
                      AVSphereDetection
                      AVSfMData
                      AVSfMDataIO
                      ${OpenCV_LIBRARIES}
        )
    endif()

    alicevision_add_software(AVLightingCalibration
            SOURCE main_lightingCalibration.cpp
            FOLDER ${FOLDER_SOFTWARE_PIPELINE}
            LINKS AVLightingEstimation
                  AVCMDLine
                  AVSystem
                  AVSfMData
                  AVSfMDataIO
                  Boost::program_options
    )

endif()

alicevision_add_software(AVLDRToHDRSampling
        SOURCE main_LdrToHdrSampling.cpp
        FOLDER ${FOLDER_SOFTWARE_PIPELINE}
        LINKS AVSystem
              AVCMDLine
              AVImage
              AVHDR
              AVSfMData
              AVSfMDataIO
              Boost::program_options
)

alicevision_add_software(AVLDRToHDRCalibration
        SOURCE main_LdrToHdrCalibration.cpp
        FOLDER ${FOLDER_SOFTWARE_PIPELINE}
        LINKS AVSystem
              AVCMDLine
              AVImage
              AVHDR
              AVSfMData
              AVSfMDataIO
              Boost::program_options
)

alicevision_add_software(AVLDRToHDRMerge
        SOURCE main_LdrToHdrMerge.cpp
        FOLDER ${FOLDER_SOFTWARE_PIPELINE}
        LINKS AVSystem
              AVCMDLine
              AVImage
              AVHDR
              AVSfMData
              AVSfMDataIO
              Boost::program_options
)

if (ALICEVISION_HAVE_ONNX)
    # Image Segmentation
    alicevision_add_software(AVImageSegmentation
            SOURCE main_imageSegmentation.cpp
            FOLDER ${FOLDER_SOFTWARE_PIPELINE}
            LINKS AVImage
                  AVCMDLine
                  AVSystem
                  AVSfMData
                  AVSfMDataIO
                  AVSegmentation
    )
endif()

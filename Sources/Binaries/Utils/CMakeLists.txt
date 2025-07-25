## AliceVision
## Utilities software

# Software PROPERTY FOLDER is 'Software/Utils'
set(FOLDER_SOFTWARE_UTILS "Software/Utils")


if (ALICEVISION_HAVE_CUDA)
    alicevision_add_software(AVHardwareResources
        SOURCE main_hardwareResources.cpp
        FOLDER ${FOLDER_SOFTWARE_UTILS}
        LINKS AVSystem
              AVCMDLine
              AVGPU
              Boost::program_options
    )
endif()

# Uncertainty
if (ALICEVISION_HAVE_UNCERTAINTYTE)
    alicevision_add_software(AVComputeUncertainty
        SOURCE main_computeUncertainty.cpp
        FOLDER ${FOLDER_SOFTWARE_UTILS}
        LINKS AVSfM
              AVSystem
              AVCMDLine
              ${CUDA_LIBRARIES}
              ${CUDA_CUBLAS_LIBRARIES}
              ${CUDA_cusparse_LIBRARY}
              ${UNCERTAINTYTE_LIBRARY}
              Boost::program_options
        INCLUDE_DIRS ${UNCERTAINTYTE_INCLUDE_DIR}
    )
    # message(warning "UNCERTAINTYTE_LIBRARY: ${UNCERTAINTYTE_LIBRARY}")
    # message(warning "CUDA_LIBRARIES: ${CUDA_LIBRARIES}")
    # message(warning "CUDA_CUBLAS_LIBRARIES: ${CUDA_CUBLAS_LIBRARIES}")
    # message(warning "CUDA_cusparse_LIBRARY: ${CUDA_cusparse_LIBRARY}")
endif()

alicevision_add_software(AVImageProcessing
    SOURCE main_imageProcessing.cpp
    FOLDER ${FOLDER_SOFTWARE_UTILS}
    LINKS AVSystem
          AVCMDLine
          AVSensorDB
          AVImage
          AVFeature
          AVSfM
          AVSfMData
          AVSfMDataIO
          AVLensCorrectionProfile
          ${Boost_LIBRARIES}
)

alicevision_add_software(AVMaskProcessing
    SOURCE main_maskProcessing.cpp
    FOLDER ${FOLDER_SOFTWARE_UTILS}
    LINKS AVSystem
          AVCMDLine
          AVImage
          ${Boost_LIBRARIES}
)

if (ALICEVISION_HAVE_OPENCV)
    target_link_libraries(AVImageProcessing_exe PRIVATE ${OpenCV_LIBS})
endif()

alicevision_add_software(AVImportMiddlebury
    SOURCE main_importMiddlebury.cpp
    FOLDER ${FOLDER_SOFTWARE_UTILS}
    LINKS AVNumeric
          AVGeometry
          AVSfMData
          AVSfMDataIO
          AVSystem
          AVCMDLine
          ${Boost_LIBRARIES}
)

# Voctree creation
alicevision_add_software(AVVoctreeCreation
    SOURCE main_voctreeCreation.cpp
    FOLDER ${FOLDER_SOFTWARE_UTILS}
    LINKS AVVoctree
          AVFeature
          AVSfMData
          AVSfMDataIO
          AVSystem
          AVCMDLine
          Boost::program_options
)

# Voctree query utility
alicevision_add_software(AVVoctreeQueryUtility
    SOURCE main_voctreeQueryUtility.cpp
    FOLDER ${FOLDER_SOFTWARE_UTILS}
    LINKS AVVoctree
          AVSfM
          AVSfMData
          AVSfMDataIO
          AVSystem
          AVCMDLine
          Boost::program_options
          Boost::boost
          Boost::timer
)

# Voctree statistics
alicevision_add_software(AVVoctreeStatistics
    SOURCE main_voctreeStatistics.cpp
    FOLDER ${FOLDER_SOFTWARE_UTILS}
    LINKS AVVoctree
          AVSfMData
          AVSystem
          AVCMDLine
          AVSfMDataIO
          Boost::program_options
          Boost::boost
)

# Frustum filtering
alicevision_add_software(AVFrustumFiltering
    SOURCE main_frustumFiltering.cpp
    FOLDER ${FOLDER_SOFTWARE_UTILS}
    LINKS AVSystem
          AVCMDLine
          AVFeature
          AVMatchingImageCollection
          AVSfM
          AVSfMData
          AVSfMDataIO
          Boost::program_options
)

if(ALICEVISION_HAVE_ALEMBIC)

    # Import external Alembic files
    alicevision_add_software(AVImportAlembic
        SOURCE main_importAlembic.cpp
        FOLDER ${FOLDER_SOFTWARE_UTILS}
        LINKS AVSfMData
              AVSfMDataIO
              AVSystem
              AVCMDLine
              AVImage
              Boost::program_options
              Alembic::Alembic
    )

    # Transform rig
    alicevision_add_software(AVRigTransform
        SOURCE main_rigTransform.cpp
        FOLDER ${FOLDER_SOFTWARE_UTILS}
        LINKS AVSfMData
              AVSfMDataIO
              AVLocalization
              AVDataIO
              AVRig
              AVSystem
              AVCMDLine
              Boost::program_options
    )

    # Output intrinsics transformed images
    alicevision_add_software(AVExportImages
    SOURCE main_exportImages.cpp
    FOLDER ${FOLDER_SOFTWARE_UTILS}
    LINKS AVSystem
        AVCMDLine
        AVImage
        AVSfMData
        AVSfMDataIO
        Boost::program_options
        Boost::boost
    )

    # Output intrinsics transformed sfmdata
    alicevision_add_software(AVIntrinsicsTransforming
    SOURCE main_intrinsicsTransforming.cpp
    FOLDER ${FOLDER_SOFTWARE_UTILS}
    LINKS AVSystem
        AVCMDLine
        AVSfMData
        AVSfMDataIO
        AVTrack
        Boost::program_options
        Boost::boost
    )
endif()

# SfM quality evaluation
# - quality comparison against a GT camera path (MultiView Evaluation dataset)
alicevision_add_software(AVQualityEvaluation
    SOURCE main_qualityEvaluation.cpp
    FOLDER ${FOLDER_SOFTWARE_UTILS}
    LINKS AVSystem
          AVCMDLine
          AVFeature
          AVSfMData
          AVSfMDataIO
          Boost::program_options
)

# SfM alignment
alicevision_add_software(AVSfMAlignment
    SOURCE main_sfmAlignment.cpp
    FOLDER ${FOLDER_SOFTWARE_UTILS}
    LINKS AVSystem
          AVCMDLine
          AVFeature
          AVSfM
          AVSfMData
          AVSfMDataIO
          Boost::program_options
)

# SfM transfer
alicevision_add_software(AVSfMTransfer
    SOURCE main_sfmTransfer.cpp
    FOLDER ${FOLDER_SOFTWARE_UTILS}
    LINKS AVSystem
          AVCMDLine
          AVFeature
          AVSfM
          AVSfMData
          AVSfMDataIO
          Boost::program_options
)

# SfM transform
alicevision_add_software(AVSfMTransform
    SOURCE main_sfmTransform.cpp
    FOLDER ${FOLDER_SOFTWARE_UTILS}
    LINKS AVSystem
        AVCMDLine
        AVFeature
        AVMesh
        AVSfM
        AVSfMData
        AVSfMDataIO
        Boost::program_options
        Boost::json
)

# SfM filter
alicevision_add_software(AVSfMFilter
    SOURCE main_sfmFilter.cpp
    FOLDER ${FOLDER_SOFTWARE_UTILS}
    LINKS AVSystem
        AVCMDLine
        AVFeature
        AVSfM
        AVSfMData
        AVSfMDataIO
        Boost::program_options
)

# SfM to rig
alicevision_add_software(AVSfMToRig
    SOURCE main_sfmToRig.cpp
    FOLDER ${FOLDER_SOFTWARE_UTILS}
    LINKS AVSystem
          AVCMDLine
          AVSfMData
          AVSfMDataIO
          Boost::program_options
)

# SfM Merge
alicevision_add_software(AVSfMMerge
    SOURCE main_sfmMerge.cpp
    FOLDER ${FOLDER_SOFTWARE_UTILS}
    LINKS AVSystem
          AVCMDLine
          AVSfMData
          AVSfMDataIO
          AVSfM
          AVMatching
          Boost::program_options
)

# Tracks Merge
alicevision_add_software(AVTracksMerging
    SOURCE main_tracksMerging.cpp
    FOLDER ${FOLDER_SOFTWARE_UTILS}
    LINKS AVSystem
          AVCMDLine
          AVTrack
          Boost::program_options
)

# SfM Pose Injecting
alicevision_add_software(AVSfMPoseInjecting
    SOURCE main_sfmPoseInjecting.cpp
    FOLDER ${FOLDER_SOFTWARE_UTILS}
    LINKS AVSystem
          AVCMDLine
          AVSfMData
          AVSfMDataIO
          AVSfM
          Boost::program_options
          Boost::json
)

# SfM Survey Injecting
alicevision_add_software(AVSfMSurveyInjecting
    SOURCE main_sfmSurveyInjecting.cpp
    FOLDER ${FOLDER_SOFTWARE_UTILS}
    LINKS AVSystem
          AVCMDLine
          AVSfMData
          AVSfMDataIO
          AVSfM
          Boost::program_options
          Boost::json
)

# SfM Colorizing
alicevision_add_software(AVSfMColorizing
    SOURCE main_sfmColorizing.cpp
    FOLDER ${FOLDER_SOFTWARE_UTILS}
    LINKS AVSystem
          AVCMDLine
          AVSfMData
          AVSfMDataIO
          AVSfM
          Boost::program_options
          Boost::json
)

# SfM split reconstructed
alicevision_add_software(AVSfMSplitReconstructed
    SOURCE main_sfmSplitReconstructed.cpp
    FOLDER ${FOLDER_SOFTWARE_UTILS}
    LINKS AVSystem
          AVCMDLine
          AVSfMData
          AVSfMDataIO
          Boost::program_options
)

# SfM color harmonize
alicevision_add_software(AVSfMColorHarmonize
    SOURCE main_sfmColorHarmonize.cpp
    FOLDER ${FOLDER_SOFTWARE_UTILS}
    LINKS AVSystem
          AVCMDLine
          AVImage
          AVFeature
          AVKVLD
          AVSfM
          AVSfMColorHarmonize
          AVSfMData
          AVSfMDataIO
          AVColorHarmonization
          Coin::Clp
          Coin::CoinUtils
          Coin::Osi
          Boost::program_options
          Boost::boost
          Boost::timer
          ${LEMON_LIBRARY}
)

# Keyframe selection
# - export keyframes from video files / image sequence directories
if (ALICEVISION_HAVE_OPENCV)
    alicevision_add_software(AVKeyframeSelection
        SOURCE main_keyframeSelection.cpp
        FOLDER ${FOLDER_SOFTWARE_UTILS}
        LINKS AVSystem
              AVCMDLine
              AVImage
              AVKeyframe
              ${OPENIMAGEIO_LIBRARIES}
              Boost::program_options
    )
endif()


# Print distances between 3D objects
alicevision_add_software(AVSfMDistances
    SOURCE main_sfmDistances.cpp
    FOLDER ${FOLDER_SOFTWARE_UTILS}
    LINKS AVSfM
          AVSfMData
          AVSfMDataIO
          AVSystem
          AVCMDLine
          ${Boost_LIBRARIES}
)

# Perform color chart detection
if (ALICEVISION_HAVE_OPENCV)
    alicevision_add_software(AVColorCheckerDetection
        SOURCE main_colorCheckerDetection.cpp
        FOLDER ${FOLDER_SOFTWARE_UTILS}
        LINKS AVSystem
              AVCMDLine
              AVSfMData
              AVSfMDataIO
              Boost::program_options
              ${OpenCV_LIBS} # opencv_core opencv_imgproc opencv_video opencv_imgcodecs opencv_videoio opencv_features2d opencv_xfeatures2d
              opencv_mcc
    )

    # Apply calibration
    alicevision_add_software(AVApplyCalibration
        SOURCE main_applyCalibration.cpp
        FOLDER ${FOLDER_SOFTWARE_UTILS}
        LINKS AVSystem
              AVCMDLine
              AVSfMData
              AVSfMDataIO
              AVDataIO
              AVCamera
              Boost::program_options
              Boost::json
    )
endif()


# Perform color correction
if (ALICEVISION_HAVE_OPENCV)
    alicevision_add_software(AVColorCheckerCorrection
        SOURCE main_colorCheckerCorrection.cpp
        FOLDER ${FOLDER_SOFTWARE_UTILS}
        LINKS AVSystem
              AVCMDLine
              AVSfMData
              AVSfMDataIO
              Boost::program_options
              ${OpenCV_LIBS} # opencv_core opencv_imgproc opencv_video opencv_imgcodecs opencv_videoio opencv_features2d opencv_xfeatures2d
              opencv_mcc
    )
endif()

# Split 360 images in input in order to export square images
alicevision_add_software(AVSplit360Images
    SOURCE main_split360Images.cpp
    FOLDER ${FOLDER_SOFTWARE_UTILS}
    LINKS AVSystem
          AVCMDLine
          AVNumeric
          AVImage
          AVPanorama
          AVSfMData
          AVSfMDataIO
          AVCamera
          ${OPENIMAGEIO_LIBRARIES}
          Boost::program_options
)

if (ALICEVISION_HAVE_OPENCV)
    alicevision_add_software(AVLightingEstimation
        SOURCE main_lightingEstimation.cpp
        FOLDER ${FOLDER_SOFTWARE_UTILS}
        LINKS AVSystem
            AVCMDLine
            AVNumeric
            AVSfMData
            AVSfMDataIO
            AVMVSUtils
            AVImage
            AVLightingEstimation
            ${Boost_LIBRARIES}
    )
endif()

# Create SfMData with some arbitrary data
alicevision_add_software(AVGenerateSampleScene
    SOURCE main_generateSampleScene.cpp
    FOLDER ${FOLDER_SOFTWARE_UTILS}
    LINKS AVSystem
            AVCMDLine
            AVNumeric
            AVSfMData
            AVSfMDataIO
            AVMVSUtils
            ${Boost_LIBRARIES}
)

# Merge two meshes
alicevision_add_software(AVMergeMeshes
    SOURCE main_mergeMeshes.cpp
    FOLDER ${FOLDER_SOFTWARE_UTILS}
    LINKS AVSystem
            AVCMDLine
            AVNumeric
            Geogram::geogram
            ${Boost_LIBRARIES}
)

# Rendering of depthmaps
alicevision_add_software(AVDepthMapRendering
    SOURCE main_depthMapRendering.cpp
    FOLDER ${FOLDER_SOFTWARE_UTILS}
    LINKS AVSystem
        AVCMDLine
        AVFeature
        AVMesh
        AVSfM
        AVSfMData
        AVSfMDataIO
)

# Rendering of normalmaps
alicevision_add_software(AVNormalMapRendering
    SOURCE main_normalMapRendering.cpp
    FOLDER ${FOLDER_SOFTWARE_UTILS}
    LINKS AVSystem
        AVCMDLine
        AVFeature
        AVMesh
        AVSfM
        AVSfMData
        AVSfMDataIO
)

# Mesh Remove Unseen Faces
alicevision_add_software(AVMeshRemoveUnseenFaces
    SOURCE main_meshRemoveUnseenFaces.cpp
    FOLDER ${FOLDER_SOFTWARE_UTILS}
    LINKS AVSystem
            AVCMDLine
            AVMVSUtils
            AVMesh
            AVSfMData
            AVSfMDataIO
            Boost::program_options
)

# Select Connected Views
alicevision_add_software(AVSelectConnectedViews
    SOURCE main_selectConnectedViews.cpp
    FOLDER ${FOLDER_SOFTWARE_UTILS}
    LINKS AVSystem
            AVCMDLine
            AVMVSData
            AVMVSUtils
            AVDepthMap
            AVSfMData
            AVSfMDataIO
            Boost::program_options
)

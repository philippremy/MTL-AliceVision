## AliceVision
## Exportation software

# Software PROPERTY FOLDER is 'Software/Export'
set(FOLDER_SOFTWARE_EXPORT "Software/Export")

# Export an alembic animated camera from a SfMData file
if (ALICEVISION_HAVE_ALEMBIC)
    alicevision_add_software(AVExportAnimatedCamera
        SOURCE main_exportAnimatedCamera.cpp
        FOLDER ${FOLDER_SOFTWARE_EXPORT}
        LINKS AVSystem
              AVCMDLine
              AVSfMData
              AVSfMDataIO
              Boost::program_options
              Boost::boost
              Boost::timer
    )
endif()

# Export view extracted keypoints
alicevision_add_software(AVExportKeypoints
    SOURCE main_exportKeypoints.cpp
    FOLDER ${FOLDER_SOFTWARE_EXPORT}
    LINKS AVSystem
          AVCMDLine
          AVFeature
          AVSfM
          AVSfMData
          AVSfMDataIO
          Boost::program_options
          Boost::boost
          Boost::timer
)

# Export view computed matches (putatives, geometrics) per image pair
alicevision_add_software(AVExportMatches
    SOURCE main_exportMatches.cpp
    FOLDER ${FOLDER_SOFTWARE_EXPORT}
    LINKS AVSystem
          AVCMDLine
          AVFeature
          AVSfM
          AVSfMData
          AVSfMDataIO
          Boost::program_options
          Boost::boost
          Boost::timer
)

# Export view tracks per image pair
alicevision_add_software(AVExportTracks
    SOURCE main_exportTracks.cpp
    FOLDER ${FOLDER_SOFTWARE_EXPORT}
    LINKS AVSystem
          AVCMDLine
          AVFeature
          AVSfM
          AVSfMData
          AVSfMDataIO
          Boost::program_options
          Boost::boost
          Boost::timer
)

# Export a SfM aliceVision scene to PMVS format
alicevision_add_software(AVExportPMVS
    SOURCE main_exportPMVS.cpp
    FOLDER ${FOLDER_SOFTWARE_EXPORT}
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

# Export a SfM aliceVision scene to PMVS format
alicevision_add_software(AVExportColmap
    SOURCE main_exportColmap.cpp
    FOLDER ${FOLDER_SOFTWARE_EXPORT}
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


# Export point cloud
alicevision_add_software(AVExportColoredPointCloud
    SOURCE main_exportColoredPointCloud.cpp
    FOLDER ${FOLDER_SOFTWARE_EXPORT}
    LINKS AVSystem
          AVCMDLine
          AVImage
          AVFeature
          AVSfMData
          AVSfMDataIO
          Boost::program_options
)

# Export a SfM aliceVision scene to MayaMVG
if (ALICEVISION_HAVE_ALEMBIC) # maya can read alembic file
    alicevision_add_software(AVExportMeshroomMaya
        SOURCE main_exportMeshroomMaya.cpp
        FOLDER ${FOLDER_SOFTWARE_EXPORT}
        LINKS AVSystem
              AVCMDLine
              AVImage
              AVFeature
              AVSfMData
              AVSfMDataIO
              Boost::program_options
              Boost::boost
              Boost::timer
              ${OPENIMAGEIO_LIBRARIES}
    )
endif()

# Export a SfM aliceVision scene to MVE(v2) format
alicevision_add_software(AVExportMVE2
    SOURCE main_exportMVE2.cpp
    FOLDER ${FOLDER_SOFTWARE_EXPORT}
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

# Export a SfM aliceVision scene to meshlab scene with rasters
alicevision_add_software(AVExportMeshlab
    SOURCE main_exportMeshlab.cpp
    FOLDER ${FOLDER_SOFTWARE_EXPORT}
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

# Export a SfM aliceVision scene to mvs-texturing scene folder
alicevision_add_software(AVExportMVSTexturing
    SOURCE main_exportMVSTexturing.cpp
    FOLDER ${FOLDER_SOFTWARE_EXPORT}
    LINKS AVSystem
          AVCMDLine
          AVImage
          AVFeature
          AVSfMData
          AVSfMDataIO
          Boost::program_options
)

# Export a SfM aliceVision scene to Matlab
alicevision_add_software(AVExportMatlab
    SOURCE main_exportMatlab.cpp
    FOLDER ${FOLDER_SOFTWARE_EXPORT}
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

# Export geometry and textures as USD
if (ALICEVISION_HAVE_USD)
    alicevision_add_software(AVExportUSD
        SOURCE main_exportUSD.cpp
        FOLDER ${FOLDER_SOFTWARE_EXPORT}
        LINKS AVSystem
              AVCMDLine
              AVMesh
              Boost::program_options
              usd
              usdGeom
              usdImaging
              usdShade
    )
endif()

# Export distortion to be used in external tools
alicevision_add_software(AVExportDistortion
    SOURCE main_exportDistortion.cpp
    FOLDER ${FOLDER_SOFTWARE_EXPORT}
    LINKS AVSystem
          AVCMDLine
          AVCamera
          AVSfMData
          AVSfMDataIO
          AVImage
          Boost::program_options
)

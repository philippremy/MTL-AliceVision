## AliceVision
## Conversion software

# Software PROPERTY FOLDER is 'Software/Convert'
set(FOLDER_SOFTWARE_CONVERT "Software/Convert")

# Convert SfMData format (from one to another)
alicevision_add_software(AVConvertSfMFormat
    SOURCE main_convertSfMFormat.cpp
    FOLDER ${FOLDER_SOFTWARE_CONVERT}
    LINKS AVSystem
          AVCMDLine
          AVFeature
          AVSfMData
          AVSfMDataIO
          Boost::program_options
          Boost::system
)

alicevision_add_software(AVImportKnownPoses
    SOURCE main_importKnownPoses.cpp
    FOLDER ${FOLDER_SOFTWARE_CONVERT}
    LINKS AVLocalization
          AVFeature
          AVSfMData
          AVSfMDataIO
          AVCMDLine
          Boost::program_options
          Boost::boost
          Boost::timer
)

if (TARGET E57Format)
    alicevision_add_software(AVImportE57
        SOURCE main_importE57.cpp
        FOLDER ${FOLDER_SOFTWARE_CONVERT}
        LINKS AVSystem
              AVCMDLine
              AVFeature
              AVSfMData
              AVSfMDataIO
              AVMesh
              AVFuseCut
              Boost::program_options
              Boost::json
              E57Format
              nanoflann::nanoflann
    )
endif()

# Convert Distortion format (from one to another)
alicevision_add_software(AVConvertDistortion
    SOURCE main_convertDistortion.cpp
    FOLDER ${FOLDER_SOFTWARE_CONVERT}
    LINKS AVSystem
          AVCMDLine
          AVFeature
          AVSfMData
          AVSfMDataIO
          AVCalibration
          Boost::program_options
          Boost::system
)

# Merge two meshes
alicevision_add_software(AVConvertMesh
    SOURCE main_convertMesh.cpp
    FOLDER ${FOLDER_SOFTWARE_CONVERT}
    LINKS AVSystem
          AVNumeric
          AVCMDLine
          AVMesh
          ${Boost_LIBRARIES}
)

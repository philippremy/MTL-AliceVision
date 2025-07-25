#pragma once

/// Function used to safely check defines
#define ALICEVISION_IS_DEFINED(F) F() == 1
#define ALICEVISION_IS_NOT_DEFINED(F) F() != 1

/// Whether SSE instructions can be used
#define ALICEVISION_HAVE_SSE() 0

/// Whether the OpenMP runtime can be used
#define ALICEVISION_HAVE_OPENMP() 1

/// Whether OpenCV functions can be used
#define ALICEVISION_HAVE_OPENCV() 1

/// Whether to use Eigen3 Memory Alignment
#define ALICEVISION_EIGEN_REQUIRE_ALIGNMENT() 1

/// Whether CUDA is available
#define ALICEVISION_HAVE_CUDA() 0

/// Whether Vulkan is available
#define ALICEVISION_HAVE_VULKAN() 0

/// Whether Metal is available
#define ALICEVISION_HAVE_METAL() 1

/// Whether ONNXRuntime is GPU accelerated (CUDA)
#define ALICEVISION_HAVE_ONNX_GPU() 0

/// Whether MOSEK linear programming functions can be used
#define ALICEVISION_HAVE_MOSEK() 0

/// Whether OpenCV is available
#define ALICEVISION_HAVE_OPENCV() 1

/// Whether OpenCV SIFT functions should be used
#define ALICEVISION_HAVE_OCVSIFT() 

/// Whether CCTag functions should be used
#define ALICEVISION_HAVE_CCTAG() 1

/// Whether AprilTag functions should be used
#define ALICEVISION_HAVE_APRILTAG() 1

/// Whether Popsift functions should be used
#define ALICEVISION_HAVE_POPSIFT() 0

/// Whether Alembic functions can be used
#define ALICEVISION_HAVE_ALEMBIC() 1

/// Whether LiDAR (E57Format) functions can be used
#define ALICEVISION_HAVE_LIDAR() 1

/// Whether OpenGV functions can be used
#define ALICEVISION_HAVE_OPENGV() 1

/// Whether Vulkan Validation Layers should be enabled
#define ALICEVISION_USE_VULKAN_VALIDATION() 0

/// If ALICEVISION_EIGEN_REQUIRE_ALIGNMENT is not defined, set some standard variables
#if ALICEVISION_IS_NOT_DEFINED(ALICEVISION_EIGEN_REQUIRE_ALIGNMENT)
    #define EIGEN_MAX_ALIGN_BYTES 
    #define EIGEN_MAX_STATIC_ALIGN_BYTES 
#endif

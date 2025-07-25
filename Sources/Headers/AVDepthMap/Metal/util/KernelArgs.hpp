// This file is part of the AliceVision project.
// Copyright (c) 2025 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <AVMVSData/ROI.hpp>

namespace aliceVision {
namespace depthMap {

#pragma mark DeviceDepthSimilarityMapKernels

struct depthSimMapCopyDepthOnly_kernel_PC
{
    int out_deptSimMap_p;
    const int in_depthSimMap_p;
    const unsigned int width;
    const unsigned int height;
    const float defaultSim;
};

struct mapUpscale_kernel_PC
{
    int out_upscaledMap_p;
    int in_map_p;
    const float ratio;
    const ROI roi;
};

struct depthThicknessMapSmoothThickness_kernel_PC
{
    int inout_depthThicknessMap_p;
    const float minThicknessInflate;
    const float maxThicknessInflate;
    const ROI roi;
};

struct computeSgmUpscaledDepthPixSizeMap_nearestNeighbor_kernel_PC
{
    int out_upscaledDepthPixSizeMap_p;
    const int in_sgmDepthThicknessMap_p;
    const int rcDeviceCameraParamsId; // useful for direct pixSize computation
    const unsigned int rcLevel0Width;
    const unsigned int rcLevel0Height;
    const unsigned int rcLevelCount;
    const unsigned int rcLevelWidth;
    const unsigned int rcLevelHeight;
    const float rcMipmapLevel;
    const int stepXY;
    const int halfNbDepths;
    const float ratio;
    const ROI roi;
};

struct computeSgmUpscaledDepthPixSizeMap_bilinear_kernel_PC
{
    int out_upscaledDepthPixSizeMap_p;
    const int in_sgmDepthThicknessMap_p;
    const int rcDeviceCameraParamsId; // useful for direct pixSize computation
    const unsigned int rcLevel0Width;
    const unsigned int rcLevel0Height;
    const unsigned int rcLevelCount;
    const unsigned int rcLevelWidth;
    const unsigned int rcLevelHeight;
    const float rcMipmapLevel;
    const int stepXY;
    const int halfNbDepths;
    const float ratio;
    const ROI roi;
};

struct depthSimMapComputeNormal_kernel_PC
{
    const int TWsh;
    int out_normalMap_p;
    int in_depthSimMap_p;
    const int rcDeviceCameraParamsId;
    const int stepXY;
    const ROI roi;
};

struct optimize_varLofLABtoW_kernel_PC
{
    int out_varianceMap_p;
    const unsigned int rcLevel0Width;
    const unsigned int rcLevel0Height;
    const unsigned int rcLevelCount;
    const unsigned int rcLevelWidth;
    const unsigned int rcLevelHeight;
    const float rcMipmapLevel;
    const int stepXY;
    const ROI roi;
};

struct optimize_getOptDeptMapFromOptDepthSimMap_kernel_PC
{
    int out_tmpOptDepthMap_p;
    const int in_optDepthSimMap_p;
    const ROI roi;
};

struct optimize_depthSimMap_kernel_PC
{
    int out_optimizeDepthSimMap_p;
    const int in_sgmDepthPixSizeMap_p;
    const int in_refineDepthSimMap_p;
    const int rcDeviceCameraParamsId;
    const unsigned int imgVariance_texLevel0Width;
    const unsigned int imgVariance_texLevel0Height;
    const unsigned int imgVariance_texLevelCount;
    const unsigned int depth_tex_texLevel0Width;
    const unsigned int depth_tex_texLevel0Height;
    const unsigned int depth_tex_texLevelCount;
    const int iter;
    const ROI roi;
};

#pragma mark DeviceSimilarityVolumeKernels

struct volume_add_kernel_PC
{
    int inout_volume_s;
    int inout_volume_p;
    const int in_volume_s;
    const int in_volume_p;
    const unsigned int volDimX;
    const unsigned int volDimY;
};

struct volume_updateUninitialized_kernel_PC
{
    int inout_volume2nd_s;
    int inout_volume2nd_p;
    const int in_volume1st_s;
    const int in_volume1st_p;
    const unsigned int volDimX;
    const unsigned int volDimY;
};

struct volume_computeSimilarity_kernel_PC
{
    int out_volume1st_s;
    int out_volume1st_p;
    int out_volume2nd_s;
    int out_volume2nd_p;
    const int in_depths_p;
    const int rcDeviceCameraParamsId;
    const int tcDeviceCameraParamsId;
    const unsigned int rcLevel0Width;
    const unsigned int rcLevel0Height;
    const unsigned int rcLevelCount;
    const unsigned int tcLevel0Width;
    const unsigned int tcLevel0Height;
    const unsigned int tcLevelCount;
    const unsigned int rcSgmLevelWidth;
    const unsigned int rcSgmLevelHeight;
    const unsigned int tcSgmLevelWidth;
    const unsigned int tcSgmLevelHeight;
    const float rcMipmapLevel;
    const int stepXY;
    const int wsh;
    const float invGammaC;
    const float invGammaP;
    const bool useConsistentScale;
    const bool useCustomPatchPattern;
    const Range depthRange;
    const ROI roi;
};

struct volume_refineSimilarity_kernel_PC
{
    bool useNormalMap;
    int inout_volSim_s;
    int inout_volSim_p;
    const int in_sgmDepthPixSizeMap_p;
    const int in_sgmNormalMap_p;
    const int rcDeviceCameraParamsId;
    const int tcDeviceCameraParamsId;
    const unsigned int rcLevel0Width;
    const unsigned int rcLevel0Height;
    const unsigned int rcLevelCount;
    const unsigned int tcLevel0Width;
    const unsigned int tcLevel0Height;
    const unsigned int tcLevelCount;
    const unsigned int rcRefineLevelWidth;
    const unsigned int rcRefineLevelHeight;
    const unsigned int tcRefineLevelWidth;
    const unsigned int tcRefineLevelHeight;
    const float rcMipmapLevel;
    const int volDimZ;
    const int stepXY;
    const int wsh;
    const float invGammaC;
    const float invGammaP;
    const bool useConsistentScale;
    const bool useCustomPatchPattern;
    const Range depthRange;
    const ROI roi;
};

struct volume_retrieveBestDepth_kernel_PC
{
    bool useDepthSimMap;
    int out_sgmDepthThicknessMap_p;
    int out_sgmDepthSimMap_p; // output depth/sim map is optional (nullptr)
    const int in_depths_p;
    const int in_volSim_s;
    const int in_volSim_p;
    const int rcDeviceCameraParamsId;
    const int volDimZ; // useful for depth/sim interpolation
    const int scaleStep;
    const float thicknessMultFactor; // default 1
    const float maxSimilarity;
    const Range depthRange;
    const ROI roi;
};

struct volume_refineBestDepth_kernel_PC
{
    int out_refineDepthSimMap_p;
    int in_sgmDepthPixSizeMap_p;
    int in_volSim_s;
    int in_volSim_p;
    int volDimZ;
    int samplesPerPixSize; // number of subsamples (samples between two depths)
    int halfNbSamples;     // number of samples (in front and behind mid depth)
    int halfNbDepths;      // number of depths  (in front and behind mid depth) should be equal to (volDimZ - 1) / 2
    float twoTimesSigmaPowerTwo;
    const ROI roi;
};

struct volume_initVolumeYSlice_kernel_PC
{
    int volume_s;
    int volume_p;
    const int3 volDim;
    const int3 axisT;
    int y;
    TSim cst;
};

struct volume_getVolumeXZSlice_kernel_PC
{
    int slice_p;
    int volume_s;
    int volume_p;
    const int3 volDim;
    const int3 axisT;
    int y;
};

struct volume_computeBestZInSlice_kernel_PC
{
    int xzSlice_p;
    int volDimX;
    int volDimZ;
};

struct volume_agregateCostVolumeAtXinSlices_kernel_PC
{
    const unsigned int rcLevel0Width;
    const unsigned int rcLevel0Height;
    const unsigned int rcLevelCount;
    const unsigned int rcSgmLevelWidth;
    const unsigned int rcSgmLevelHeight;
    const float rcMipmapLevel;
    int xzSliceForY_p;
    const int xzSliceForYm1_p;
    const int volAgr_s;
    const int volAgr_p;
    const int3 volDim;
    const int3 axisT;
    const float step;
    const int y;
    const float P1;
    const float _P2;
    const int ySign;
    const int filteringIndex;
    const ROI roi;
};

#pragma mark DeviceColorConversion

struct rgb2lab_kernel_PC
{
    unsigned int inout_img_p;
    unsigned int width;
    unsigned int height;
};

#pragma mark DeviceGaussianFilter

struct downscaleWithGaussianBlur_kernel_PC
{
    const unsigned int in_img_texLevel0Width;
    const unsigned int in_img_texLevel0Height;
    const unsigned int in_img_texLevelCount;
    int out_downscaledImg_p;
    unsigned int downscaledImgWidth;
    unsigned int downscaledImgHeight;
    int downscale;
    int gaussRadius;
};

struct gaussianBlurVolumeZ_kernel_PC
{
    int out_volume_s;
    int out_volume_p;
    int in_volume_s;
    int in_volume_p;
    int volDimX;
    int volDimY;
    int volDimZ;
    int gaussRadius;
};

struct gaussianBlurVolumeXYZ_kernel_PC
{
    int out_volume_s;
    int out_volume_p;
    int in_volume_s;
    int in_volume_p;
    int volDimX;
    int volDimY;
    int volDimZ;
    int gaussRadius;
};

struct medianFilter3_kernel_PC
{
    const unsigned int texLevel0Width;
    const unsigned int texLevel0Height;
    const unsigned int texLevelCount;
    int texLab_p;
    int width;
    int height;
    int scale;
};

#pragma mark DeviceMipmappedArray

struct createMipmappedArrayLevel_kernel_PC
{
    int TRadius;
    unsigned int width;
    unsigned int height;
    const unsigned int previousMipLevel;
    const unsigned int currentMipLevel;
    const unsigned int inout_mipmapped_array_dLevel0Width;
    const unsigned int inout_mipmapped_array_dLevel0Height;
    const unsigned int inout_mipmapped_array_dLevelCount;
};

struct createMipmappedArrayDebugFlatImage_kernel_PC
{
    int out_flatImage_p;
    const unsigned int in_mipmappedArray_texLevel0Width;
    const unsigned int in_mipmappedArray_texLevel0Height;
    const unsigned int in_mipmappedArray_texLevelCount;
    unsigned int levels;
    unsigned int firstLevelWidth;
    unsigned int firstLevelHeight;
};

}
}

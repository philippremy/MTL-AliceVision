// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <AVImage/Image.hpp>
#include <AVMVSUtils/MultiViewParams.hpp>
#include <AVMVSData/Point3d.hpp>
#include <AVMVSData/StaticVector.hpp>
#include <AVMVSData/Universe.hpp>
#include <AVMVSData/Voxel.hpp>
#include <AVSfMData/SfMData.hpp>

namespace aliceVision {

namespace fuseCut {

class Fuser
{
  public:
    const mvsUtils::MultiViewParams& _mp;

    Fuser(const mvsUtils::MultiViewParams& mp);
    ~Fuser();

    // minNumOfModals number of other cams including this cam ... minNumOfModals /in 2,3,... default 3
    // pixSizeBall = default 2
    void filterGroups(const std::vector<int>& cams, float pixToleranceFactor, int pixSizeBall, int pixSizeBallWSP, int nNearestCams);
    bool filterGroupsRC(int rc, float pixToleranceFactor, int pixSizeBall, int pixSizeBallWSP, int nNearestCams);
    void filterDepthMaps(const std::vector<int>& cams, int minNumOfModals, int minNumOfModalsWSP2SSP);
    bool filterDepthMapsRC(int rc, int minNumOfModals, int minNumOfModalsWSP2SSP);

    void divideSpaceFromDepthMaps(Point3d* hexah, float& minPixSize);
    void divideSpaceFromSfM(const sfmData::SfMData& sfmData, Point3d* hexah, std::size_t minObservations = 0, float minObservationAngle = 0.0f) const;

    /// @brief Compute average pixel size in the given hexahedron
    float computeAveragePixelSizeInHexahedron(Point3d* hexah, int step, int scale);
    float computeAveragePixelSizeInHexahedron(Point3d* hexah, const sfmData::SfMData& sfmData);

  private:
    bool updateInSurr(float pixToleranceFactor,
                      int pixSizeBall,
                      int pixSizeBallWSP,
                      Point3d& p,
                      int rc,
                      int tc,
                      StaticVector<int>* numOfPtsMap,
                      const image::Image<float>& depthMap,
                      const image::Image<float>& simMap,
                      int scale);
};

unsigned long computeNumberOfAllPoints(const mvsUtils::MultiViewParams& mp, int scale);



}  // namespace fuseCut
}  // namespace aliceVision

// This file is part of the AliceVision project.
// Copyright (c) 2025 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <AVSfMData/Landmark.hpp>

namespace aliceVision {
namespace sfmData {

/**
 * @brief ConstraintPoint defines a 3D constraint on a landmark
 */
struct ConstraintPoint
{
    ConstraintPoint() = default;

    ConstraintPoint(const IndexT paramLandmarkId, const Vec3 & paramNormal, const Vec3 & paramPoint)
      : landmarkId(paramLandmarkId),
        normal(paramNormal),
        point(paramPoint)
    {}

    IndexT landmarkId;
    Vec3 normal;
    Vec3 point;

    bool operator==(const ConstraintPoint& other) const
    {
        return (normal == other.normal) && (point == other.point) && (landmarkId == other.landmarkId);
    }

    inline bool operator!=(const ConstraintPoint& other) const { return !(*this == other); }
};

}  // namespace sfmData
}  // namespace aliceVision

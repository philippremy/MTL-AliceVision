// This file is part of the AliceVision project.
// Copyright (c) 2020 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <AVNumeric/numeric.hpp>
#include <AVGeometry/Pose3.hpp>
#include <AVCamera/camera.hpp>

#include <AVPanorama/boundingBox.hpp>

namespace aliceVision {

bool computeCoarseBB(BoundingBox& coarse_bbox,
                     const std::pair<int, int>& panoramaSize,
                     const geometry::Pose3& pose,
                     const aliceVision::camera::IntrinsicBase& intrinsics);

}

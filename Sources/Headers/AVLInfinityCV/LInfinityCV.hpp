// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#ifndef ALICEVISION_L_INFINITY_COMPUTER_VISION_H_
#define ALICEVISION_L_INFINITY_COMPUTER_VISION_H_

// Structure and motion problem solver
#include <AVLInfinityCV/tijsAndXis_From_xi_Ri.hpp>
#include <AVLInfinityCV/tijsAndXis_From_xi_Ri_noise.hpp>
#include <AVLInfinityCV/triplet_tijsAndXis_kernel.hpp>

// Pose estimation solver
#include <AVLInfinityCV/resection.hpp>
#include <AVLInfinityCV/resection_kernel.hpp>
// N-View Triangulation solver
#include <AVLInfinityCV/triangulation.hpp>

//-------------
//-- Global SfM
//-------------
// Compute from global translation by using 2-views relative translations guess
#include <AVLInfinityCV/global_translations_fromTij.hpp>
// Compute from global translation by using 3-views relative translations guess
#include <AVLInfinityCV/global_translations_fromTriplets.hpp>

#endif  // ALICEVISION_L_INFINITY_COMPUTER_VISION_H_

// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

// SfM

#include <AVSfM/filters.hpp>
#include <AVSfM/FrustumFilter.hpp>
#include <AVSfMBundle/BundleAdjustment.hpp>
#include <AVSfMBundle/BundleAdjustmentCeres.hpp>
#include <AVSfMBundle/LocalBundleAdjustmentGraph.hpp>
#include <AVSfM/generateReport.hpp>
#include <AVSfM/sfmFilters.hpp>
#include <AVSfM/sfmTriangulation.hpp>

// SfM pipeline

#include <AVSfM/pipeline/ReconstructionEngine.hpp>
#include <AVSfM/pipeline/pairwiseMatchesIO.hpp>
#include <AVSfM/pipeline/RelativePoseInfo.hpp>
#include <AVSfM/pipeline/global/reindexGlobalSfM.hpp>
#include <AVSfM/pipeline/global/ReconstructionEngine_globalSfM.hpp>
#include <AVSfM/pipeline/panorama/ReconstructionEngine_panorama.hpp>
#include <AVSfM/pipeline/sequential/ReconstructionEngine_sequentialSfM.hpp>
#include <AVSfM/pipeline/structureFromKnownPoses/StructureEstimationFromKnownPoses.hpp>
#include <AVSfM/pipeline/localization/SfMLocalizer.hpp>

// This file is part of the AliceVision project.
// Copyright (c) 2024 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <AVSfM/pipeline/expanding/SfmBundle.hpp>
#include <AVSfM/sfmFilters.hpp>
#include <AVSfMBundle/BundleAdjustmentCeres.hpp>

namespace aliceVision {
namespace sfm {

bool SfmBundle::process(sfmData::SfMData & sfmData, const track::TracksHandler & tracksHandler, const std::set<IndexT> & viewIds)
{
    ALICEVISION_LOG_INFO("SfmBundle::process start");

    BundleAdjustmentCeres::CeresOptions options;
    BundleAdjustment::ERefineOptions refineOptions;

    refineOptions |= BundleAdjustment::REFINE_ROTATION;
    refineOptions |= BundleAdjustment::REFINE_TRANSLATION;
    refineOptions |= BundleAdjustment::REFINE_STRUCTURE;
    refineOptions |= BundleAdjustment::REFINE_INTRINSICS_ALL;

    if (!initialize(sfmData, tracksHandler, viewIds))
    {
        return false;
    }

    options.setSparseBA();
    BundleAdjustmentCeres bundleObject(options, _minNbCamerasToRefinePrincipalPoint);

    //Repeat until nothing change
    do
    {
        const bool success = bundleObject.adjust(sfmData, refineOptions);
        if (!success)
        {
            return false;
        }
    }
    while (cleanup(sfmData));

    ALICEVISION_LOG_INFO("SfmBundle::process end");
    return true;
}

bool SfmBundle::cleanup(sfmData::SfMData & sfmData)
{
    // Remove observations with too large residuals
    const std::size_t nbOutliersResidualErr = removeOutliersWithPixelResidualError(sfmData, _featureConstraint, _maxReprojectionError, _minTrackLength);

    // Remove landmarks without enough observed parallax
    const std::size_t nbOutliersAngleErr = removeOutliersWithAngleError(sfmData, _minAngleForLandmark);

    // Remove constraints which are too far away from landmark
    const std::size_t nbOutliersConstraints =  removeConstraints(sfmData, _maxConstraintDistance);

    // Remove poses without enough observations in an interactive fashion
    const std::size_t nbOutliers = nbOutliersResidualErr + nbOutliersAngleErr;
    std::set<IndexT> removedViewsIdIteration;
    bool somethingErased = eraseUnstablePosesAndObservations(sfmData, _minPointsPerPose, _minTrackLength, &removedViewsIdIteration);

    bool somethingChanged = /*somethingErased || */(nbOutliers > _bundleAdjustmentMaxOutlier) || (nbOutliersConstraints > 0);

    ALICEVISION_LOG_INFO("SfmBundle::cleanup : ");
    ALICEVISION_LOG_INFO(" - nbOutliersResidualErr : " << nbOutliersResidualErr);
    ALICEVISION_LOG_INFO(" - nbOutliersAngleErr : " << nbOutliersAngleErr);
    ALICEVISION_LOG_INFO(" - nbOutliersConstraints : " << nbOutliersConstraints);
    ALICEVISION_LOG_INFO(" - somethingErased : " << somethingErased);
    ALICEVISION_LOG_INFO(" - somethingChanged : " << somethingChanged);

    return somethingChanged;
}

bool SfmBundle::initialize(sfmData::SfMData & sfmData, const track::TracksHandler & tracksHandler, const std::set<IndexT> & viewIds)
{
    bool enableLocalStrategy = _useLBA;

    if (sfmData.getPoses().size() < _minNbCamerasLBA)
    {
        enableLocalStrategy = false;
    }

    if (enableLocalStrategy)
    {
        if (_lbaPolicy)
        {
            _lbaPolicy->build(sfmData, tracksHandler, viewIds);
        }
    }

    return true;
}

} // namespace sfm
} // namespace aliceVision

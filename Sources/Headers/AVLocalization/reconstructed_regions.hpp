// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <AV/config.hpp>
#include <AV/types.hpp>
#include <AVFeature/Regions.hpp>
#include <AVFeature/Descriptor.hpp>
#include <AVFeature/imageDescriberCommon.hpp>

#include <vector>
#include <map>
#include <memory>

namespace aliceVision {
namespace localization {

struct ReconstructedRegionsMapping
{
    std::vector<IndexT> _associated3dPoint;
    std::map<IndexT, IndexT> _mapFullToLocal;
};

inline std::unique_ptr<feature::Regions> createFilteredRegions(const feature::Regions& regions,
                                                               const std::vector<feature::FeatureInImage>& featuresInImage,
                                                               ReconstructedRegionsMapping& out_mapping)
{
    return regions.createFilteredRegions(featuresInImage, out_mapping._associated3dPoint, out_mapping._mapFullToLocal);
}

using ReconstructedRegionsMappingPerDesc = std::map<feature::EImageDescriberType, ReconstructedRegionsMapping>;

using ReconstructedRegionsMappingPerView = std::map<IndexT, ReconstructedRegionsMappingPerDesc>;

}  // namespace localization
}  // namespace aliceVision

// This file is part of the AliceVision project.
// Copyright (c) 2024 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <AVTrack/TracksHandler.hpp>

#include <AVTrack/trackIO.hpp>
#include <AVTrack/tracksUtils.hpp>

#include <fstream>

namespace aliceVision {
namespace track {

bool TracksHandler::load(const std::string& pathJson, const std::set<IndexT>& viewIds)
{
    _mapTracks.clear();
    if (!loadTracks(_mapTracks, pathJson))
    {
        return false;
    }

    // Compute tracks per view
    _mapTracksPerView.clear();
    for (const auto& viewId : viewIds)
    {
        // create an entry in the map
        _mapTracksPerView[viewId];
    }
    track::computeTracksPerView(_mapTracks, _mapTracksPerView);

    return true;
}

}  // namespace track
}  // namespace aliceVision

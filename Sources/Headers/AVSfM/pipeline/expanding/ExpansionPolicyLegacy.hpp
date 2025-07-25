// This file is part of the AliceVision project.
// Copyright (c) 2024 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.


#pragma once

#include <AVSfM/pipeline/expanding/ExpansionPolicy.hpp>

namespace aliceVision {
namespace sfm {

class ExpansionPolicyLegacy : public ExpansionPolicy
{
public:
    using uptr = std::unique_ptr<ExpansionPolicyLegacy>;

public:

    /**
     * @brief Initialize policy for an iteration
     * @param sfmData the scene to process
     * @return true if the init succeeded
    */
    virtual bool initialize(const sfmData::SfMData & sfmData);

    /**
     * @brief compute policy for an iteration
     * @param sfmData the scene to process
     * @param tracksHandler the tracks for this scene
     * @return true if the policy succeeded
    */
    virtual bool process(const sfmData::SfMData & sfmData, const track::TracksHandler & tracksHandler);

    /**
     * @brief Retrieve the selected next views
     * @return a set of views to process
    */
    virtual std::set<IndexT> getNextViews();

    /**
     * @brief Compute score of a view
     * @param tracksMap the scene set of tracks
     * @param usedTracks the list of tracks to consider
     * @param viewId the view of interest
     * @param maxSize the largest dimension of the view's image
     * @param countLevels the number of levels we want to use (starting from a single pixel level)
     * @return a score
    */
    static double computeScore(const track::TracksMap & tracksMap,
                                const std::vector<std::size_t> & usedTracks,
                                const IndexT viewId,
                                const size_t maxSize,
                                const size_t countLevels);

    /**
     * @brief set the number of views for which we consider the sfm to be unstable
     * @param count a number of views
    */
    void setNbFirstUnstableViews(size_t count)
    {
        _nbFirstUnstableViews = count;
    }

    /**
     * @brief set the number of views we want for a given chunk
     * @param count a number of views
    */
    void setMaxViewsPerGroup(size_t count)
    {
        _maxViewsPerGroup = count;
    }

    /**
     * @brief rollback some processed views inside the available views
     * @param viewsSet the set of views that we want to be able to select again.
    */
    virtual void rollback(const std::set<IndexT> & viewsSet);

private:

    // vector of  selected views for this iteration
    std::set<IndexT> _selectedViews;

    // List of available view indices
    std::set<IndexT> _availableViewsIds;

private:
    // Minimal number of points viewed in a view
    // AND reconstructed in the sfm
    std::size_t _minPointsThreshold = 30;

    // Number of levels in the pyramid starting
    // from the level with a grid size of 2x2
    // Level 0 = 2x2, Level 1 = 4x4, etc.
    std::size_t _countPyramidLevels = 5;

    // Number of cameras in scene under which the set is considered as unstable
    size_t _nbFirstUnstableViews = 30;

    // Maximal number of images in a chunk
    size_t _maxViewsPerGroup = 30;
};

}
}

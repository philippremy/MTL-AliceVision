// This file is part of the AliceVision project.
// Copyright (c) 2024 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <AV/types.hpp>
#include <AVTrack/TracksHandler.hpp>
#include <AVSfMData/SfMData.hpp>
#include <AVSfM/pipeline/expanding/ExpansionHistory.hpp>
#include <AVSfM/pipeline/expanding/SfmBundle.hpp>
#include <AVSfM/pipeline/expanding/PointFetcher.hpp>

namespace aliceVision {
namespace sfm {

class ExpansionChunk
{
public:
    using uptr = std::unique_ptr<ExpansionChunk>;

public:

    /**
     * @brief Compute a chunk of views assuming the sfmData already has an initial set of
     * reconstructed cameras and 3D points to connect to.
     * @param sfmData the sfmData which describes the current sfm state
     * @param tracksHandler the scene tracks handler
     * @param viewsChunks a list of view ids to process in this chunk
    */
    bool process(sfmData::SfMData & sfmData,
                const track::TracksHandler & tracksHandler,
                const std::set<IndexT> & viewsChunk);

    /**
     * @brief setup the bundle handler
     * @param bundleHandler a unique ptr. the Ownership will be taken
    */
    void setBundleHandler(SfmBundle::uptr & bundleHandler)
    {
        _bundleHandler = std::move(bundleHandler);
    }

    /**
     * brief setup the expansion history handler
     * @param expansionHistory a shared ptr
     */
    void setExpansionHistoryHandler(ExpansionHistory::sptr & expansionHistory)
    {
        _historyHandler = expansionHistory;
    }

    /**
     * brief setup the point fetcher handler
     * @param pointFetcher a unique ptr. the Ownership will be taken
    */
    void setPointFetcherHandler(PointFetcher::uptr & pointFetcherHandler)
    {
        _pointFetcherHandler = std::move(pointFetcherHandler);
    }

    void setResectionMaxIterations(size_t maxIterations)
    {
        _resectionIterations = maxIterations;
    }

    /**
     * @brief set the maximal error allowed for ransac resection module
     * @param error the error value or <= 0 for automatic decision
     * @param count the number of points
    */
    void setResectionMaxError(double error)
    {
        _resectionMaxError = error;
        if (_resectionMaxError <= 0.0)
        {
            _resectionMaxError = std::numeric_limits<double>::infinity();
        }
    }

    /**
     * @brief set the minimal number of points to enable triangulation of a track
     * @param count the number of points
    */
    void setTriangulationMinPoints(size_t count)
    {
        _triangulationMinPoints = count;
    }

    /**
     * @brief set the minimal allowed parallax degree for triangulation
     * @param angle the angle in DEGREES
    */
    void setMinAngleTriangulation(double angle)
    {
        _minTriangulationAngleDegrees = angle;
    }

    const std::set<IndexT> & getIgnoredViews()
    {
        return _ignoredViews;
    }

private:

    /**
     * @Brief assign the computed pose to the view
     * @param sfmData the sfmData to update
     * @param viewId the viewId of interest
     * @param pose the homogeneous matrix computed from the resection
    */
    void addPose(sfmData::SfMData & sfmData, IndexT viewId, const Eigen::Matrix4d & pose);

    /**
     * @brief Try to upgrade sfm with new landmarks
     * @param sfmData the object to update
     * @param tracks all tracks of the scene as a map {trackId, track}
     * @param viewIds the set of views to triangulate
     */
    bool triangulate(sfmData::SfMData & sfmData, const track::TracksHandler & tracksHandler, const std::set<IndexT> & viewIds);

    /**
     * @brief Add constraints on points
     * @param sfmData the object to update
     * @param tracks all tracks of the scene as a map {trackId, track}
     * @param viewIds the set of views to process
    */
    void setConstraints(sfmData::SfMData & sfmData, const track::TracksHandler & tracksHandler, const std::set<IndexT> & viewIds);

private:
    SfmBundle::uptr _bundleHandler;
    ExpansionHistory::sptr _historyHandler;
    PointFetcher::uptr _pointFetcherHandler;
    std::set<IndexT> _ignoredViews;

private:
    size_t _resectionIterations = 1024;
    size_t _triangulationMinPoints = 2;
    double _minTriangulationAngleDegrees = 3.0;
    double _maxTriangulationError = 8.0;
    double _resectionMaxError = std::numeric_limits<double>::infinity();
};

} // namespace sfm
} // namespace aliceVision

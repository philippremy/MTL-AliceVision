// This file is part of the AliceVision project.
// Copyright (c) 2021 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <AVCalibration/distortionEstimationLine.hpp>

#include <AVSystem/Logger.hpp>
#include <ceres/ceres.h>
#include <cmath>

namespace aliceVision {
namespace calibration {

class CostLine : public ceres::CostFunction
{
  public:
    CostLine(const std::shared_ptr<camera::Undistortion>& undistortion, const Vec2& pt, double sigma)
      : _undistortion(undistortion),
        _pt(pt),
        _sigma(sigma)
    {
        set_num_residuals(1);

        mutable_parameter_block_sizes()->push_back(1);
        mutable_parameter_block_sizes()->push_back(1);
        mutable_parameter_block_sizes()->push_back(2);
        mutable_parameter_block_sizes()->push_back(undistortion->getParameters().size());
    }

    bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const override
    {
        const double* parameter_angle_line = parameters[0];
        const double* parameter_dist_line = parameters[1];
        const double* parameter_offset = parameters[2];
        const double* parameter_disto = parameters[3];

        const double angle = parameter_angle_line[0];
        const double distanceToLine = parameter_dist_line[0];

        const double cangle = std::cos(angle);
        const double sangle = std::sin(angle);

        std::vector<double> cameraDistortionParams = _undistortion->getParameters();
        const std::size_t distortionSize = cameraDistortionParams.size();
        for (std::size_t idParam = 0; idParam < distortionSize; ++idParam)
        {
            cameraDistortionParams[idParam] = parameter_disto[idParam];
        }
        _undistortion->setParameters(cameraDistortionParams);

        Vec2 undistortionOffset;
        undistortionOffset.x() = parameter_offset[0];
        undistortionOffset.y() = parameter_offset[1];
        _undistortion->setOffset(undistortionOffset);

        const Vec2 ipt = _undistortion->undistort(_pt);
        const double pa = _undistortion->getPixelAspectRatio();
        const double ny = ipt.y() / pa;
        const double w = 1.0 / _sigma;

        residuals[0] = w * (cangle * ipt.x() + sangle * ny - distanceToLine);

        if (jacobians == nullptr)
        {
            return true;
        }

        if (jacobians[0] != nullptr)
        {
            Eigen::Map<Eigen::Matrix<double, 1, 1, Eigen::RowMajor>> J(jacobians[0]);
            J(0, 0) = w * (ipt.x() * -sangle + ny * cangle);
        }

        if (jacobians[1] != nullptr)
        {
            Eigen::Map<Eigen::Matrix<double, 1, 1, Eigen::RowMajor>> J(jacobians[1]);
            J(0, 0) = -w;
        }

        if (jacobians[2] != nullptr)
        {
            Eigen::Map<Eigen::Matrix<double, 1, 2, Eigen::RowMajor>> J(jacobians[2]);

            Eigen::Matrix<double, 1, 2> Jline;
            Jline(0, 0) = cangle;
            Jline(0, 1) = sangle / pa;

            J = w * Jline * _undistortion->getDerivativeUndistortWrtOffset(_pt);
        }

        if (jacobians[3] != nullptr)
        {
            Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> J(jacobians[3], 1, distortionSize);

            Eigen::Matrix<double, 1, 2> Jline;
            Jline(0, 0) = cangle;
            Jline(0, 1) = sangle / pa;

            J = w * Jline * _undistortion->getDerivativeUndistortWrtParameters(_pt);
        }

        return true;
    }

  private:
    std::shared_ptr<camera::Undistortion> _undistortion;
    Vec2 _pt;
    double _sigma;
};

bool estimate(std::shared_ptr<camera::Undistortion> undistortionToEstimate,
              Statistics& statistics,
              std::vector<LineWithPoints>& lines,
              const bool lockCenter,
              const bool lockAngles,
              const std::vector<bool>& lockDistortions)
{
    if (!undistortionToEstimate)
    {
        return false;
    }

    if (lines.empty())
    {
        return false;
    }

    ceres::Problem problem;
    ceres::LossFunction* lossFunction = nullptr;

    std::vector<double> undistortionParameters = undistortionToEstimate->getParameters();
    Vec2 undistortionOffset = undistortionToEstimate->getOffset();
    const std::size_t countUndistortionParams = undistortionParameters.size();

    if (lockDistortions.size() != countUndistortionParams)
    {
        ALICEVISION_LOG_ERROR("Invalid number of distortion parameters (lockDistortions=" << lockDistortions.size() << ", countDistortionParams="
                                                                                          << countUndistortionParams << ").");
        return false;
    }

    double* ptrUndistortionParameters = &undistortionParameters[0];
    double* center = &undistortionOffset.x();

    // Add off center parameter
    problem.AddParameterBlock(center, 2);
    if (lockCenter)
    {
        problem.SetParameterBlockConstant(center);
    }

    // Add distortion parameter
    problem.AddParameterBlock(ptrUndistortionParameters, countUndistortionParams);

    // Check if all distortions are locked
    bool allLocked = true;
    for (bool lock : lockDistortions)
    {
        if (!lock)
        {
            allLocked = false;
        }
    }

    if (allLocked)
    {
        problem.SetParameterBlockConstant(ptrUndistortionParameters);
    }
    else
    {
        // At least one parameter is not locked

        std::vector<int> constantDistortions;
        for (int idParamDistortion = 0; idParamDistortion < static_cast<int>(lockDistortions.size()); ++idParamDistortion)
        {
            if (lockDistortions[idParamDistortion])
            {
                constantDistortions.push_back(idParamDistortion);
            }
        }

        if (!constantDistortions.empty())
        {
            ceres::SubsetManifold* subsetManifold = new ceres::SubsetManifold(countUndistortionParams, constantDistortions);
            problem.SetManifold(ptrUndistortionParameters, subsetManifold);
        }
    }

    for (auto& l : lines)
    {
        problem.AddParameterBlock(&l.angle, 1);
        problem.AddParameterBlock(&l.dist, 1);

        if (lockAngles)
        {
            problem.SetParameterBlockConstant(&l.angle);
        }

        for (const auto& pt : l.points)
        {
            ceres::CostFunction* costFunction = new CostLine(undistortionToEstimate, pt.center, pow(2.0, pt.scale));
            problem.AddResidualBlock(costFunction, lossFunction, &l.angle, &l.dist, center, ptrUndistortionParameters);
        }
    }

    ceres::Solver::Options options;
    options.use_inner_iterations = true;
    options.max_num_iterations = 1000;
    options.logging_type = ceres::SILENT;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    ALICEVISION_LOG_TRACE(summary.FullReport());

    if (!summary.IsSolutionUsable())
    {
        ALICEVISION_LOG_ERROR("Lens calibration estimation failed.");
        return false;
    }

    undistortionToEstimate->setOffset(undistortionOffset);
    undistortionToEstimate->setParameters(undistortionParameters);

    std::vector<double> errors;
    for (auto& l : lines)
    {
        const double sangle = std::sin(l.angle);
        const double cangle = std::cos(l.angle);

        for (const auto& pt : l.points)
        {
            const Vec2 ipt = undistortionToEstimate->undistort(pt.center);
            const double pa = undistortionToEstimate->getPixelAspectRatio();
            const double ny = ipt.y() / pa;
            double divider = pow(2.0, pt.scale);
            const double res = (cangle * ipt.x() + sangle * ny - l.dist) / divider;
            errors.push_back(std::abs(res));
        }
    }

    const double mean = std::accumulate(errors.begin(), errors.end(), 0.0) / static_cast<double>(errors.size());
    const double sqSum = std::inner_product(errors.begin(), errors.end(), errors.begin(), 0.0);
    const double stddev = std::sqrt(sqSum / errors.size() - mean * mean);
    std::sort(errors.begin(), errors.end());
    const double median = errors[errors.size() / 2];
    const double max = errors[errors.size() - 1];
    const double lastDecile = errors[errors.size() * 0.9];

    statistics.mean = mean;
    statistics.stddev = stddev;
    statistics.median = median;
    statistics.max = max;
    statistics.lastDecile = lastDecile;

    return true;
}

}  // namespace calibration
}  // namespace aliceVision

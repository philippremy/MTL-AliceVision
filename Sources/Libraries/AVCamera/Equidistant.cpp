// This file is part of the AliceVision project.
// Copyright (c) 2023 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <AVCamera/Equidistant.hpp>

#include <algorithm>
#include <cmath>

namespace aliceVision {
namespace camera {

std::shared_ptr<Equidistant> Equidistant::cast(std::shared_ptr<IntrinsicBase> sptr) { return std::dynamic_pointer_cast<Equidistant>(sptr); }

Vec2 Equidistant::transformProject(const Eigen::Matrix4d& pose, const Vec4& pt, bool applyDistortion) const
{
    const double rsensor = std::min(sensorWidth(), sensorHeight());
    const double rscale = sensorWidth() / std::max(w(), h());
    const double fmm = _scale(0) * rscale;
    const double fov = rsensor / fmm;

    const Vec4 X = pose * pt;

    // Compute angle with optical center
    const double angle_Z = std::atan2(sqrt(X(0) * X(0) + X(1) * X(1)), X(2));

    // Ignore depth component and compute radial angle
    const double angle_radial = std::atan2(X(1), X(0));

    const double radius = angle_Z / (0.5 * fov);

    // radius = focal * angle_Z
    const Vec2 P{cos(angle_radial) * radius, sin(angle_radial) * radius};

    const Vec2 pt_disto = applyDistortion ? this->addDistortion(P) : P;
    const Vec2 pt_ima = this->cam2ima(pt_disto);

    return pt_ima;
}

Vec2 Equidistant::project(const Vec4& pt, bool applyDistortion) const
{
    const double rsensor = std::min(sensorWidth(), sensorHeight());
    const double rscale = sensorWidth() / std::max(w(), h());
    const double fmm = _scale(0) * rscale;
    const double fov = rsensor / fmm;

    // Compute angle with optical center
    const double angle_Z = std::atan2(sqrt(pt(0) * pt(0) + pt(1) * pt(1)), pt(2));

    // Ignore depth component and compute radial angle
    const double angle_radial = std::atan2(pt(1), pt(0));

    const double radius = angle_Z / (0.5 * fov);

    // radius = focal * angle_Z
    const Vec2 P{cos(angle_radial) * radius, sin(angle_radial) * radius};

    const Vec2 pt_disto = applyDistortion ? this->addDistortion(P) : P;
    const Vec2 pt_ima = this->cam2ima(pt_disto);

    return pt_ima;
}

Eigen::Matrix<double, 2, 3> Equidistant::getDerivativeTransformProjectWrtPoint3(const Eigen::Matrix4d& T, const Vec4& pt) const
{
    const Vec4 X = T * pt;  // apply pose

    const Eigen::Matrix<double, 4, 3>& d_X_d_pt = T.block<4, 3>(0, 0);

    /* Compute angle with optical center */
    const double len2d = sqrt(X(0) * X(0) + X(1) * X(1));
    Eigen::Matrix<double, 2, 2> d_len2d_d_X;
    d_len2d_d_X(0) = X(0) / len2d;
    d_len2d_d_X(1) = X(1) / len2d;

    const double angle_Z = std::atan2(len2d, X(2));
    const double d_angle_Z_d_len2d = X(2) / (len2d * len2d + X(2) * X(2));

    /* Ignore depth component and compute radial angle */
    const double angle_radial = std::atan2(X(1), X(0));

    Eigen::Matrix<double, 2, 4> d_angles_d_X;
    d_angles_d_X(0, 0) = -X(1) / (X(0) * X(0) + X(1) * X(1));
    d_angles_d_X(0, 1) = X(0) / (X(0) * X(0) + X(1) * X(1));
    d_angles_d_X(0, 2) = 0.0;
    d_angles_d_X(0, 3) = 0.0;

    d_angles_d_X(1, 0) = d_angle_Z_d_len2d * d_len2d_d_X(0);
    d_angles_d_X(1, 1) = d_angle_Z_d_len2d * d_len2d_d_X(1);
    d_angles_d_X(1, 2) = -len2d / (len2d * len2d + X(2) * X(2));
    d_angles_d_X(1, 3) = 0.0;

    const double rsensor = std::min(sensorWidth(), sensorHeight());
    const double rscale = sensorWidth() / std::max(w(), h());
    const double fmm = _scale(0) * rscale;
    const double fov = rsensor / fmm;
    const double radius = angle_Z / (0.5 * fov);

    double d_radius_d_angle_Z = 1.0 / (0.5 * fov);

    /* radius = focal * angle_Z */
    const Vec2 P{cos(angle_radial) * radius, sin(angle_radial) * radius};

    Eigen::Matrix<double, 2, 2> d_P_d_angles;
    d_P_d_angles(0, 0) = -sin(angle_radial) * radius;
    d_P_d_angles(0, 1) = cos(angle_radial) * d_radius_d_angle_Z;
    d_P_d_angles(1, 0) = cos(angle_radial) * radius;
    d_P_d_angles(1, 1) = sin(angle_radial) * d_radius_d_angle_Z;

    return getDerivativeCam2ImaWrtPoint() * getDerivativeAddDistoWrtPt(P) * d_P_d_angles * d_angles_d_X * d_X_d_pt;
}

Eigen::Matrix<double, 2, Eigen::Dynamic> Equidistant::getDerivativeTransformProjectWrtDistortion(const Eigen::Matrix4d& pose, const Vec4& pt) const
{
    Eigen::Matrix4d T = pose;
    const Vec4 X = T * pt;  // apply pose

    /* Compute angle with optical center */
    const double len2d = sqrt(X(0) * X(0) + X(1) * X(1));
    const double angle_Z = std::atan2(len2d, X(2));

    /* Ignore depth component and compute radial angle */
    const double angle_radial = std::atan2(X(1), X(0));

    const double rsensor = std::min(sensorWidth(), sensorHeight());
    const double rscale = sensorWidth() / std::max(w(), h());
    const double fmm = _scale(0) * rscale;
    const double fov = rsensor / fmm;
    const double radius = angle_Z / (0.5 * fov);

    /* radius = focal * angle_Z */
    const Vec2 P{cos(angle_radial) * radius, sin(angle_radial) * radius};

    return getDerivativeCam2ImaWrtPoint() * getDerivativeAddDistoWrtDisto(P);
}

Eigen::Matrix<double, 2, 2> Equidistant::getDerivativeTransformProjectWrtScale(const Eigen::Matrix4d& pose, const Vec4& pt) const
{
    Eigen::Matrix4d T = pose;
    const Vec4 X = T * pt;  // apply pose

    /* Compute angle with optical center */
    const double len2d = sqrt(X(0) * X(0) + X(1) * X(1));
    const double angle_Z = std::atan2(len2d, X(2));

    /* Ignore depth component and compute radial angle */
    const double angle_radial = std::atan2(X(1), X(0));

    const double rsensor = std::min(sensorWidth(), sensorHeight());
    const double rscale = sensorWidth() / std::max(w(), h());
    const double fmm = _scale(0) * rscale;
    const double fov = rsensor / fmm;
    const double radius = angle_Z / (0.5 * fov);

    /* radius = focal * angle_Z */
    const Vec2 P{cos(angle_radial) * radius, sin(angle_radial) * radius};

    Eigen::Matrix<double, 2, 1> d_P_d_radius;
    d_P_d_radius(0, 0) = cos(angle_radial);
    d_P_d_radius(1, 0) = sin(angle_radial);

    Eigen::Matrix<double, 1, 1> d_radius_d_fov;
    d_radius_d_fov(0, 0) = (-2.0 * angle_Z / (fov * fov));

    Eigen::Matrix<double, 1, 2> d_fov_d_scale;
    d_fov_d_scale(0, 0) = -rsensor / (_scale(0) * _scale(0) * rscale);
    d_fov_d_scale(0, 1) = 0.0;

    return getDerivativeCam2ImaWrtPoint() * getDerivativeAddDistoWrtPt(P) * d_P_d_radius * d_radius_d_fov * d_fov_d_scale;
}

Eigen::Matrix<double, 2, 2> Equidistant::getDerivativeTransformProjectWrtPrincipalPoint([[maybe_unused]] const Eigen::Matrix4d& pose,
                                                                                        [[maybe_unused]] const Vec4& pt) const
{
    return getDerivativeCam2ImaWrtPrincipalPoint();
}

Eigen::Matrix<double, 2, Eigen::Dynamic> Equidistant::getDerivativeTransformProjectWrtParams(const Eigen::Matrix4d& pose, const Vec4& pt3D) const
{
    Eigen::Matrix<double, 2, Eigen::Dynamic> ret(2, getParameters().size());

    ret.block<2, 2>(0, 0) = getDerivativeTransformProjectWrtScale(pose, pt3D);
    ret.block<2, 2>(0, 2) = getDerivativeTransformProjectWrtPrincipalPoint(pose, pt3D);

    return ret;
}

Vec3 Equidistant::toUnitSphere(const Vec2& pt) const
{
    const double rsensor = std::min(sensorWidth(), sensorHeight());
    const double rscale = sensorWidth() / std::max(w(), h());
    const double fmm = _scale(0) * rscale;
    const double fov = rsensor / fmm;

    const double angle_radial = atan2(pt(1), pt(0));
    const double angle_Z = pt.norm() * 0.5 * fov;

    const Vec3 ret{cos(angle_radial) /** / 1.0 / **/ * sin(angle_Z), sin(angle_radial) /** / 1.0 / **/ * sin(angle_Z), cos(angle_Z)};

    return ret;
}

Eigen::Matrix<double, 3, 2> Equidistant::getDerivativetoUnitSphereWrtPoint(const Vec2& pt) const
{
    const double rsensor = std::min(sensorWidth(), sensorHeight());
    const double rscale = sensorWidth() / std::max(w(), h());
    const double fmm = _scale(0) * rscale;
    const double fov = rsensor / fmm;

    const double angle_radial = atan2(pt(1), pt(0));
    const double angle_Z = pt.norm() * 0.5 * fov;

    Eigen::Matrix<double, 3, 2> d_ret_d_angles;
    d_ret_d_angles(0, 0) = -sin(angle_radial) * sin(angle_Z);
    d_ret_d_angles(0, 1) = cos(angle_radial) * cos(angle_Z);
    d_ret_d_angles(1, 0) = cos(angle_radial) * sin(angle_Z);
    d_ret_d_angles(1, 1) = sin(angle_radial) * cos(angle_Z);
    d_ret_d_angles(2, 0) = 0;
    d_ret_d_angles(2, 1) = -sin(angle_Z);

    Eigen::Matrix<double, 2, 2> d_angles_d_pt;
    d_angles_d_pt(0, 0) = -pt(1) / (pt(0) * pt(0) + pt(1) * pt(1));
    d_angles_d_pt(0, 1) = pt(0) / (pt(0) * pt(0) + pt(1) * pt(1));
    d_angles_d_pt(1, 0) = 0.5 * fov * pt(0) / pt.norm();
    d_angles_d_pt(1, 1) = 0.5 * fov * pt(1) / pt.norm();

    return d_ret_d_angles * d_angles_d_pt;
}

Eigen::Matrix<double, 3, 2> Equidistant::getDerivativetoUnitSphereWrtScale(const Vec2& pt) const
{
    const double rsensor = std::min(sensorWidth(), sensorHeight());
    const double rscale = sensorWidth() / std::max(w(), h());
    const double fmm = _scale(0) * rscale;
    const double fov = rsensor / fmm;

    const double angle_radial = atan2(pt(1), pt(0));
    const double angle_Z = pt.norm() * 0.5 * fov;

    Eigen::Matrix<double, 3, 2> d_ret_d_angles;
    d_ret_d_angles(0, 0) = -sin(angle_radial) * sin(angle_Z);
    d_ret_d_angles(0, 1) = cos(angle_radial) * cos(angle_Z);
    d_ret_d_angles(1, 0) = cos(angle_radial) * sin(angle_Z);
    d_ret_d_angles(1, 1) = sin(angle_radial) * cos(angle_Z);
    d_ret_d_angles(2, 0) = 0;
    d_ret_d_angles(2, 1) = -sin(angle_Z);

    Eigen::Matrix<double, 2, 1> d_angles_d_fov;

    d_angles_d_fov(0, 0) = 0;
    d_angles_d_fov(1, 0) = pt.norm() * 0.5;

    Eigen::Matrix<double, 1, 2> d_fov_d_scale;
    d_fov_d_scale(0, 0) = -rsensor / (_scale(0) * _scale(0) * rscale);
    d_fov_d_scale(0, 1) = 0.0;

    return d_ret_d_angles * d_angles_d_fov * d_fov_d_scale;
}

Eigen::Matrix<double, 3, Eigen::Dynamic> Equidistant::getDerivativeBackProjectUnitWrtParams(const Vec2& pt2D) const
{
    const Vec2 ptMeters = ima2cam(pt2D);
    const Vec2 ptUndist = removeDistortion(ptMeters);

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> J(3, getParameters().size());

    J.block<3, 2>(0, 0) = getDerivativetoUnitSphereWrtScale(ptUndist);
    J.block<3, 2>(0, 2) =
      getDerivativetoUnitSphereWrtPoint(ptUndist) * getDerivativeRemoveDistoWrtPt(ptMeters) * getDerivativeIma2CamWrtPrincipalPoint();

    return J;
}

Eigen::Matrix<double, 3, Eigen::Dynamic> Equidistant::getDerivativeBackProjectUnitWrtDistortion(const Vec2& pt2D) const
{
    size_t disto_size = getDistortionParamsSize();

    const Vec2 ptMeters = ima2cam(pt2D);
    const Vec2 ptUndist = removeDistortion(ptMeters);

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> J(3, disto_size);
    J = getDerivativetoUnitSphereWrtPoint(ptUndist) * getDerivativeRemoveDistoWrtDisto(ptMeters);

    return J;
}

double Equidistant::imagePlaneToCameraPlaneError(double value) const { return value / _scale(0); }

Vec2 Equidistant::cam2ima(const Vec2& p) const { return _circleRadius * p + getPrincipalPoint(); }

Eigen::Matrix2d Equidistant::getDerivativeCam2ImaWrtPoint() const { return Eigen::Matrix2d::Identity() * _circleRadius; }

Vec2 Equidistant::ima2cam(const Vec2& p) const { return (p - getPrincipalPoint()) / _circleRadius; }

Eigen::Matrix2d Equidistant::getDerivativeIma2CamWrtPoint() const { return Eigen::Matrix2d::Identity() * (1.0 / _circleRadius); }

Eigen::Matrix2d Equidistant::getDerivativeIma2CamWrtPrincipalPoint() const { return Eigen::Matrix2d::Identity() * (-1.0 / _circleRadius); }

bool Equidistant::isVisibleRay(const Vec3& ray) const
{
    const double rsensor = std::min(sensorWidth(), sensorHeight());
    const double rscale = sensorWidth() / std::max(w(), h());
    const double fmm = _scale(0) * rscale;
    const double fov = rsensor / fmm;

    double angle = std::acos(ray.normalized().dot(Eigen::Vector3d::UnitZ()));
    if (std::abs(angle) > 1.2 * (0.5 * fov))
        return false;

    const Vec2 proj = transformProject(Eigen::Matrix4d::Identity(), ray.homogeneous(), true);
    const Vec2 centered = proj - Vec2(_circleCenter(0), _circleCenter(1));
    return centered.norm() <= _circleRadius;
}

EINTRINSIC Equidistant::getType() const { return EINTRINSIC::EQUIDISTANT_CAMERA; }

double Equidistant::getHorizontalFov() const
{
    const double rsensor = std::min(sensorWidth(), sensorHeight());
    const double rscale = sensorWidth() / std::max(w(), h());
    const double fmm = _scale(0) * rscale;
    const double fov = rsensor / fmm;

    return fov;
}

double Equidistant::getVerticalFov() const { return getHorizontalFov(); }

double Equidistant::pixelProbability() const { return 1.0 / double(w()); }

}  // namespace camera
}  // namespace aliceVision

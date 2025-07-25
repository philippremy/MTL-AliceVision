// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <AV/types.hpp>
#include <AVNumeric/numeric.hpp>
#include <AVCamera/cameraCommon.hpp>
#include <AVCamera/IntrinsicInitMode.hpp>
#include <AVGeometry/Pose3.hpp>
#include <AV/version.hpp>

#include <vector>

namespace aliceVision {
namespace camera {

/**
 * @brief Basis class for all intrinsic parameters of a camera.
 *
 * Store the image size & define all basis optical modelization of a camera
 */
class IntrinsicBase
{
  public:
    using sptr = std::shared_ptr<IntrinsicBase>;
    using ptr = IntrinsicBase*;

  public:
    explicit IntrinsicBase(unsigned int width, unsigned int height, const std::string& serialNumber = "")
      : _w(width),
        _h(height),
        _serialNumber(serialNumber)
    {}

    virtual ~IntrinsicBase() = default;

    /**
     * @brief Get the lock state of the intrinsic
     * @return True if the intrinsic is locked, false otherwise
     */
    inline bool isLocked() const { return _locked; }

    /**
     * @brief Get the intrinsic image width
     * @return The intrinsic image width
     */
    inline unsigned int w() const { return _w; }

    /**
     * @brief Get the intrinsic image height
     * @return The intrinsic image height
     */
    inline unsigned int h() const { return _h; }

    /**
     * @brief Get the intrinsic sensor width
     * @return The intrinsic sensor width
     */
    inline double sensorWidth() const { return _sensorWidth; }

    /**
     * @brief Get the intrinsic sensor height
     * @return The intrinsic sensor height
     */
    inline double sensorHeight() const { return _sensorHeight; }

    /**
     * @brief Get the intrinsic serial number
     * @return The intrinsic serial number
     */
    inline const std::string& serialNumber() const { return _serialNumber; }

    /**
     * @brief Get the intrinsic initialization mode
     * @return The intrinsic initialization mode
     */
    inline EInitMode getInitializationMode() const { return _initializationMode; }

    /**
     * @brief operator ==
     * @param[in] other
     * @return True if equals
     */
    virtual bool operator==(const IntrinsicBase& other) const;

    inline bool operator!=(const IntrinsicBase& other) const { return !(*this == other); }

    /**
     * @brief Projection of a 3D point into the camera plane (Apply pose, disto (if any) and Intrinsics)
     * @param[in] pose The pose
     * @param[in] pt3D The 3D point
     * @param[in] applyDistortion If true, apply the distortion if there is any
     * @return The 2D projection in the camera plane
     */
    Vec2 transformProject(const geometry::Pose3& pose, const Vec4& pt3D, bool applyDistortion = true) const
    {
        return transformProject(pose.getHomogeneous(), pt3D, applyDistortion);
    }

    /**
     * @brief Projection of a 3D point into the camera plane (Apply pose, disto (if any) and Intrinsics)
     * @param[in] pose The pose
     * @param[in] pt3D The 3D point
     * @param[in] applyDistortion If true, apply the distortion if there is any
     * @return The 2D projection in the camera plane
     */
    virtual Vec2 transformProject(const Eigen::Matrix4d& pose, const Vec4& pt3D, bool applyDistortion = true) const = 0;

    /**
     * @brief Projection of a 3D point into the camera plane (Apply disto (if any) and Intrinsics)
     * @param[in] pt3D The 3D point
     * @param[in] applyDistortion If true, apply the distortion if there is any
     * @return The 2D projection in the camera plane
     */
    virtual Vec2 project(const Vec4& pt3D, bool applyDistortion = true) const = 0;

    /**
     * @brief Back-projection of a 2D point at a specific depth into a 3D point
     * @param[in] pt2D The 2D point
     * @param[in] applyDistortion If true, apply the distortion if there is any
     * @param[in] pose The camera pose
     * @param[in] depth The depth
     * @return The 3D point
     */
    Vec3 backprojectTransform(const Vec2& pt2D,
                              bool applyUndistortion = true,
                              const geometry::Pose3& pose = geometry::Pose3(),
                              double depth = 1.0) const;

    /**
     * @brief Back-projection of a 2D point on a unitsphere
     * @param[in] pt2D The 2D point
     * @return The 3D point
     */
    Vec3 backProjectUnit(const Vec2& pt2D) const;

    Vec4 getCartesianfromSphericalCoordinates(const Vec3& pt);

    Eigen::Matrix<double, 4, 3> getDerivativeCartesianfromSphericalCoordinates(const Vec3& pt);

    /**
     * @brief Get the derivative of a projection of a 3D point into the camera plane
     * @param[in] pose The pose
     * @param[in] pt3D The 3D point
     * @return The projection jacobian with respect to the point
     */
    virtual Eigen::Matrix<double, 2, 3> getDerivativeTransformProjectWrtPoint3(const Eigen::Matrix4d& pose, const Vec4& pt3D) const = 0;

    /**
     * @brief Get the derivative of a projection of a 3D point into the camera plane
     * @param[in] pose The pose
     * @param[in] pt3D The 3D point
     * @return The projection jacobian with respect to the params
     */
    virtual Eigen::Matrix<double, 2, Eigen::Dynamic> getDerivativeTransformProjectWrtParams(const Eigen::Matrix4d& pos, const Vec4& pt3D) const = 0;

    /**
     * @brief Get the derivative of the unit sphere backprojection
     * @param[in] pt2D The 2D point
     * @return The backproject jacobian with respect to the pose
     */
    virtual Eigen::Matrix<double, 3, Eigen::Dynamic> getDerivativeBackProjectUnitWrtParams(const Vec2& pt2D) const = 0;

    /**
     * @brief Compute the residual between the 3D projected point X and an image observation x
     * @param[in] pose The pose
     * @param[in] X The 3D projected point
     * @param[in] x The image observation
     * @param[in] applyDistortion If true, apply the distortion if there is any
     * @return residual between the 3D projected point and the image observation
     */
    inline Vec2 residual(const geometry::Pose3& pose, const Vec4& X, const Vec2& x, bool applyDistortion = true) const
    {
        // We will compare to an undistorted point, so always ignore the distortion when computing coordinates
        const Vec2 proj = this->transformProject(pose, X, false);

        return ((applyDistortion) ? this->getUndistortedPixel(x) : x) - proj;
    }

    /**
     * @brief Compute the residuals between the 3D projected point X and an image observation x
     * @param[in] pose The pose
     * @param[in] X The 3D projection
     * @param[in] x The image observation
     * @return residuals between the 3D projected point and the image observation
     */
    inline Mat2X residuals(const geometry::Pose3& pose, const Mat3X& X, const Mat2X& x) const
    {
        assert(X.cols() == x.cols());
        const std::size_t numPts = x.cols();
        Mat2X residuals = Mat2X::Zero(2, numPts);
        for (std::size_t i = 0; i < numPts; ++i)
        {
            residuals.col(i) = residual(pose, ((const Vec3&)X.col(i)).homogeneous(), x.col(i));
        }
        return residuals;
    }

    /**
     * @brief Lock the intrinsic
     */
    inline void lock() { _locked = true; }

    /**
     * @brief Unlock the intrinsic
     */
    inline void unlock() { _locked = false; }

    /**
     * @brief Set the intrinsic image width
     * @param[in] width The image width
     */
    inline void setWidth(unsigned int width) { _w = width; }

    /**
     * @brief Set the intrinsic image height
     * @param[in] height The image height
     */
    inline void setHeight(unsigned int height) { _h = height; }

    /**
     * @brief Set the intrinsic sensor width
     * @param[in] width The sensor width
     */
    inline void setSensorWidth(double width) { _sensorWidth = width; }

    /**
     * @brief Set the intrinsic sensor height
     * @param[in] height The sensor height
     */
    inline void setSensorHeight(double height) { _sensorHeight = height; }

    /**
     * @brief Set the serial number
     * @param[in] serialNumber The serial number
     */
    inline void setSerialNumber(const std::string& serialNumber) { _serialNumber = serialNumber; }

    /**
     * @brief Set the intrinsic initialization mode
     * @param[in] initializationMode The intrintrinsic initialization mode enum value
     */
    inline void setInitializationMode(EInitMode initializationMode) { _initializationMode = initializationMode; }

    // Virtual members

    /**
     * @brief Polymorphic clone
     */
    virtual IntrinsicBase* clone() const = 0;

    /**
     * @brief Assign object
     * @param[in] other
     */
    virtual void assign(const IntrinsicBase& other) = 0;

    /**
     * @brief Get embed camera type
     * @return EINTRINSIC enum value
     */
    virtual EINTRINSIC getType() const = 0;

    /**
     * @brief Get the string describing the intrinsic type
     * @return The string describing the intrinsic type
     */
    std::string getTypeStr() const { return EINTRINSIC_enumToString(getType()); }

    /**
     * @brief Get the intrinsic parameters
     * @return Intrinsic parameters as a vector
     */
    virtual std::vector<double> getParameters() const = 0;

    /**
     * @brief Get the count of intrinsic parameters
     * @return The number of intrinsic parameters
     */
    virtual std::size_t getParametersSize() const = 0;

    /**
     * @brief Update intrinsic parameters
     * @param[in] intrinsic parameters
     * @return True if the parameters were successfully updated, false otherwise
     */
    virtual bool updateFromParams(const std::vector<double>& params) = 0;

    /**
     * @brief Import intrinsic parameters from external array
     * @param[in] intrinsic parameters
     * @param[in] inputVersion input source version (for optional transformation)
     * @return True if the parameters were successfully imported, false otherwise
     */
    virtual bool importFromParams(const std::vector<double>& params, const Version& inputVersion) = 0;

    /**
     * @brief Transform a point from the camera plane to the image plane
     * @param[in] p A point from the camera plane
     * @return Image plane point
     */
    virtual Vec2 cam2ima(const Vec2& p) const = 0;

    /**
     * @brief Transform a point from the image plane to the camera plane
     * @param[in] p A point from the image plane
     * @return Camera plane point
     */
    virtual Vec2 ima2cam(const Vec2& p) const = 0;

    /**
     * @brief Camera model handles a distortion field
     * @return True if the camera model handles a distortion field
     */
    virtual bool hasDistortion() const { return false; }

    /**
     * @brief Add the distortion field to a point (that is in normalized camera frame)
     * @param[in] p The point
     * @return The point with added distortion field
     */
    virtual Vec2 addDistortion(const Vec2& p) const = 0;

    /**
     * @brief Remove the distortion to a camera point (that is in normalized camera frame)
     * @param[in] p The point
     * @return The point with removed distortion field
     */
    virtual Vec2 removeDistortion(const Vec2& p) const = 0;

    /**
     * @brief Return the undistorted pixel (with removed distortion)
     * @param[in] p The point
     * @return The undistorted pixel
     */
    virtual Vec2 getUndistortedPixel(const Vec2& p) const = 0;

    /**
     * @brief Return the distorted pixel (with added distortion)
     * @param[in] p The undistorted point
     * @return The distorted pixel
     */
    virtual Vec2 getDistortedPixel(const Vec2& p) const = 0;

    /**
     * @brief Set The intrinsic disto initialization mode
     * @param[in] distortionInitializationMode The intrintrinsic distortion initialization mode enum
     */
    virtual void setDistortionInitializationMode(EInitMode distortionInitializationMode) = 0;

    /**
     * @brief Get the intrinsic disto initialization mode
     * @return The intrinsic disto initialization mode
     */
    virtual EInitMode getDistortionInitializationMode() const = 0;

    /**
     * @brief Normalize a given unit pixel error to the camera plane
     * @param[in] value Given unit pixel error
     * @return Normalized unit pixel error to the camera plane
     */
    virtual double imagePlaneToCameraPlaneError(double value) const = 0;

    /**
     * @brief What is the probability of a pixel wrt the whole fov
     * @return a value in radians
     */
    virtual double pixelProbability() const = 0;

    /**
     * @brief Return true if the intrinsic is valid
     * @return True if the intrinsic is valid
     */
    virtual bool isValid() const { return _w != 0 && _h != 0; }

    /**
     * @brief Return true if this ray should be visible in the image
     * @param ray input ray to check for visibility
     * @return True if this ray is visible theoretically
     */
    virtual bool isVisibleRay(const Vec3& ray) const = 0;

    /**
     * @brief Return true if these pixel coordinates should be visible in the image
     * @param pix input pixel coordinates to check for visibility
     * @return True if visible
     */
    virtual bool isVisible(const Vec2& pix) const;

    /**
     * @brief Return true if these pixel coordinates should be visible in the image
     * @param pix input pixel coordinates to check for visibility
     * @return True if visible
     */
    virtual bool isVisible(const Vec2f& pix) const;

    /**
     * @brief Assuming the distortion is a function of radius, estimate the
     * maximal undistorted radius for a range of distorted radius.
     * @param minRadius the minimal radius to consider
     * @param maxRadius the maximal radius to consider
     * @return The maximal undistorted radius
     */
    virtual float getMaximalDistortion(double minRadius, double maxRadius) const;

    /**
     * @brief Generate an unique Hash from the camera parameters (used for grouping)
     * @return Unique Hash from the camera parameters
     */
    virtual std::size_t hashValue() const;

    /**
     * @brief Rescale intrinsics to reflect a rescale of the camera image
     * @param factorW a scale factor for Width
     * @param factorH a scale factor for Height
     */
    virtual void rescale(float factorW, float factorH);

    /**
     * @brief Transform a given point (in pixels) to unit sphere in meters
     * @param pt the input point
     * @return A point on the unit sphere
     */
    virtual Vec3 toUnitSphere(const Vec2& pt) const = 0;

    /**
     * @brief Get the horizontal FOV in radians
     * @return Horizontal FOV in radians
     */
    virtual double getHorizontalFov() const = 0;

    /**
     * @brief Get the vertical FOV in radians
     * @return Vertical FOV in radians
     */
    virtual double getVerticalFov() const = 0;

    /**
     * @brief Initialize state with default values
     * The estimator state is used in the bundle adjustment to decide if we update it.
     * It is set to constant if the intrinsic is locked
     * It is set to refined if unlocked
     */
    virtual void initializeState()
    {
        if (_locked)
        {
            _state = EEstimatorParameterState::CONSTANT;
        }
        else
        {
            _state = EEstimatorParameterState::REFINED;
        }
    }

    /**
     * @brief accessor to estimator state
     * @return a state
     */
    EEstimatorParameterState getState() const { return _state; }

    /**
     * @brief mutator for the estimator state
     * @param state the new state of the intrinsic
     */
    void setState(EEstimatorParameterState state) { _state = state; }

  protected:
    /// initialization mode
    EInitMode _initializationMode = EInitMode::NONE;
    /// intrinsic lock
    bool _locked = false;
    EEstimatorParameterState _state = EEstimatorParameterState::REFINED;

    unsigned int _w = 0;
    unsigned int _h = 0;
    double _sensorWidth = 36.0;
    double _sensorHeight = 24.0;
    std::string _serialNumber;
};

/**
 * @brief Apply intrinsic and extrinsic parameters to unit vector
 * from the cameras focus to a point on the camera plane
 * @param[in] pose Extrinsic pose
 * @param[in] intrinsic Intrinsic camera parameters
 * @param[in] x Point in image
 * @return The unit vector in 3D space pointing out from the camera to the point
 */
inline Vec3 applyIntrinsicExtrinsic(const geometry::Pose3& pose, const IntrinsicBase* intrinsic, const Vec2& x)
{
    // x = (u, v, 1.0)  // image coordinates
    // X = R.t() * K.inv() * x + C // Camera world point
    // getting the ray:
    // ray = X - C = R.t() * K.inv() * x
    return (pose.rotation().transpose() * intrinsic->toUnitSphere(intrinsic->removeDistortion(intrinsic->ima2cam(x)))).normalized();
}

/**
 * @brief Return the angle (degree) between two bearing vector rays
 * @param[in] ray1 First bearing vector ray
 * @param[in] ray2 Second bearing vector ray
 * @return The angle (degree) between two bearing vector rays
 */
inline double angleBetweenRays(const Vec3& ray1, const Vec3& ray2)
{
    const double mag = ray1.norm() * ray2.norm();
    const double dotAngle = ray1.dot(ray2);
    return radianToDegree(acos(clamp(dotAngle / mag, -1.0 + 1.e-8, 1.0 - 1.e-8)));
}

/**
 * @brief Return the angle (degree) between two bearing vector rays
 * @param[in] pose1 First pose
 * @param[in] intrinsic1 First intrinsic
 * @param[in] pose2 Second pose
 * @param[in] intrinsic2 Second intrinsic
 * @param[in] x1 First image point
 * @param[in] x2 Second image point
 * @return The angle (degree) between two bearing vector rays
 */
inline double angleBetweenRays(const geometry::Pose3& pose1,
                               const IntrinsicBase* intrinsic1,
                               const geometry::Pose3& pose2,
                               const IntrinsicBase* intrinsic2,
                               const Vec2& x1,
                               const Vec2& x2)
{
    const Vec3 ray1 = applyIntrinsicExtrinsic(pose1, intrinsic1, x1);
    const Vec3 ray2 = applyIntrinsicExtrinsic(pose2, intrinsic2, x2);
    return angleBetweenRays(ray1, ray2);
}

/**
 * @brief Return the angle (degree) between two poses and a 3D point.
 * @param[in] pose1 First pose
 * @param[in] pose2 Second Pose
 * @param[in] pt3D The 3d point
 * @return The angle (degree) between two poses and a 3D point.
 */
inline double angleBetweenRays(const geometry::Pose3& pose1, const geometry::Pose3& pose2, const Vec3& pt3D)
{
    const Vec3 ray1 = pt3D - pose1.center();
    const Vec3 ray2 = pt3D - pose2.center();
    return angleBetweenRays(ray1, ray2);
}

}  // namespace camera

}  // namespace aliceVision

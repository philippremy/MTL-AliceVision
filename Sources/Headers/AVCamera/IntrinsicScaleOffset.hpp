// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <AVCamera/IntrinsicBase.hpp>

namespace aliceVision {
namespace camera {

/**
 * @brief Class with "focal" (scale) and center offset
 */
class IntrinsicScaleOffset : public IntrinsicBase
{
  public:
    IntrinsicScaleOffset(unsigned int w, unsigned int h, double scaleX, double scaleY, double offsetX, double offsetY)
      : IntrinsicBase(w, h),
        _scale(scaleX, scaleY),
        _offset(offsetX, offsetY)
    {}

    ~IntrinsicScaleOffset() override = default;

    static std::shared_ptr<IntrinsicScaleOffset> cast(std::shared_ptr<IntrinsicBase> sptr);

    void copyFrom(const IntrinsicScaleOffset& other) { *this = other; }

    bool operator==(const IntrinsicBase& otherBase) const override;

    void setScale(const Vec2& scale) { _scale = scale; }

    inline Vec2 getScale() const { return _scale; }

    void setOffset(const Vec2& offset) { _offset = offset; }

    inline Vec2 getOffset() const { return _offset; }

    /**
     * @brief Principal point in image coordinate ((0,0) is image top-left).
     */
    inline const Vec2 getPrincipalPoint() const
    {
        Vec2 ret = _offset;
        ret(0) += static_cast<double>(_w) * 0.5;
        ret(1) += static_cast<double>(_h) * 0.5;

        return ret;
    }

    // Transform a point from the camera plane to the image plane
    Vec2 cam2ima(const Vec2& p) const override;

    virtual Eigen::Matrix2d getDerivativeCam2ImaWrtScale(const Vec2& p) const;

    virtual Eigen::Matrix2d getDerivativeCam2ImaWrtPoint() const;

    virtual Eigen::Matrix2d getDerivativeCam2ImaWrtPrincipalPoint() const;

    // Transform a point from the image plane to the camera plane
    Vec2 ima2cam(const Vec2& p) const override;

    virtual Eigen::Matrix<double, 2, 2> getDerivativeIma2CamWrtScale(const Vec2& p) const;

    virtual Eigen::Matrix2d getDerivativeIma2CamWrtPoint() const;

    virtual Eigen::Matrix2d getDerivativeIma2CamWrtPrincipalPoint() const;

    /**
     * @brief Rescale intrinsics to reflect a rescale of the camera image
     * @param factorW a scale factor for Width
     * @param factorH a scale factor for Height
     */
    void rescale(float factorW, float factorH) override;

    // Data wrapper for non linear optimization (update from data)
    bool updateFromParams(const std::vector<double>& params) override;

    /**
     * @brief Import a vector of params loaded from a file. It is similar to updateFromParams but it deals with file compatibility.
     * @param[in] params a vector containing the intrinsic's parameters
     * @param[in] inputVersion
     * @return True if the parameters were correctly updated, false otherwise
     */
    bool importFromParams(const std::vector<double>& params, const Version& inputVersion) override;

    /**
     * @brief Set the initial scale for the intrinsic (used to constrain minimization)
     */
    inline void setInitialScale(const Vec2& initialScale) { _initialScale = initialScale; }

    /**
     * @brief Get the intrinsic initial scale
     * @return The intrinsic initial scale
     */
    inline Vec2 getInitialScale() const { return _initialScale; }

    /**
     * @brief Lock the ratio between fx and fy
     * @param lock True if the ratio is locked, false otherwise
     */
    inline void setRatioLocked(bool lock) { _ratioLocked = lock; }

    inline bool isRatioLocked() const { return _ratioLocked; }

    /**
     * @brief Lock the offset
     * @param lock True if the offset is locked, false otherwise
     */
    inline void setOffsetLocked(bool lock) { _offsetLocked = lock; }

    inline bool isOffsetLocked() const { return _offsetLocked; }

    /**
     * @brief Lock the scale
     * @param lock True if the scale is locked, false otherwise
     */
    inline void setScaleLocked(bool lock) { _scaleLocked = lock; }

    inline bool isScaleLocked() const { return _scaleLocked; }

    /**
     * @brief get focal length in mm
     * @return focal length in mm
     */
    double getFocalLength() const;

    /**
     * @brief get initial focal length in mm
     * @return initial focal length in mm
     */
    double getInitialFocalLength() const;

    /**
     * @brief estimate the pixel aspect ratio
     * @return the pixel aspect ratio
     */
    double getPixelAspectRatio() const;

    /**
     * @Brief set scale given focal length and pixelaspectratio
     * @param focalLengthMM focal length in mm
     * @param pixelAspectRatio pixel aspect ratio (image with = pixelAspectRatio * realwidth)
     * @param usePixelRatioForX use pixel ratio
     */
    void setFocalLength(double focalLengthMM, double pixelAspectRatio, bool useCompatibility = false);

    /**
     * @Brief set initial scale given initial focal length and pixelaspectratio
     * @param initialFocalLengthMM initial focal length in mm
     * @param pixelAspectRatio pixel aspect ratio (image with = pixelAspectRatio * realwidth)
     * @param useCompability make sure compatibility mode is used
     */
    void setInitialFocalLength(double initialFocalLengthMM, double pixelAspectRatio, bool useCompatibility = false);

  protected:
    Vec2 _scale{1.0, 1.0};
    Vec2 _offset{0.0, 0.0};
    Vec2 _initialScale{-1.0, -1.0};
    bool _ratioLocked{true};
    bool _offsetLocked{false};
    bool _scaleLocked{false};
};

}  // namespace camera
}  // namespace aliceVision

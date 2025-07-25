// This file is part of the AliceVision project.
// Copyright (c) 2025 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <AVCamera/camera.hpp>

namespace aliceVision {
namespace sfm {

struct ConstraintPointErrorFunctor
{
    explicit ConstraintPointErrorFunctor(const double weight, const Vec3 & normal, const Vec3 & point)
    : _weight(weight), _normal(normal)
    {
        _constraintDistance = _normal.dot(point);
    }

    template<typename T>
    bool operator()(T const* const* parameters, T* residuals) const
    {
        const T* parameter_point = parameters[0];

        T distance = parameter_point[0] * _normal[0] + parameter_point[1] * _normal[1] + parameter_point[2] * _normal[2];
        residuals[0] = _weight * (distance - _constraintDistance);

        return true;
    }

    double _weight;
    Vec3 _normal;
    double _constraintDistance;
};

}  // namespace sfm
}  // namespace aliceVision

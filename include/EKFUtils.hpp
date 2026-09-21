#pragma once

#include <Eigen/Dense>
#include "StateVector.hpp"

namespace tracking {
    // Type alias for 3x9 EKF-Matrix
    using Matrix3x9 = Eigen::Matrix<double, 3, 9>;

    [[nodiscard]] auto h(const StateVector& x) noexcept -> Eigen::Vector3d;

    [[nodiscard]] auto calculateRadarJacobian(const StateVector& x) noexcept -> Matrix3x9;
}
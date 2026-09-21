#pragma once

#include <Eigen/Dense>
#include <cmath>
#include "Measurement.hpp"

namespace tracking {

using StateVector = Eigen::Matrix<double, 9, 1>;

using StateCovariance = Eigen::Matrix<double, 9, 9>;

using SystemMatrix = Eigen::Matrix<double, 9, 9>;

using Matrix6d = Eigen::Matrix<double, 6, 6>;

using Vector6d = Eigen::Matrix<double, 6, 1>;

[[nodiscard]] inline auto radarToCartesian(const RadarMeasurement& radar) noexcept -> Eigen::Vector3d {
    const double cos_el = std::cos(radar.elevation);
    const double sin_el = std::sin(radar.elevation);
    const double cos_az = std::cos(radar.azimuth);
    const double sin_az = std::sin(radar.azimuth);

    return Eigen::Vector3d{
        radar.range * cos_el * sin_az,
        radar.range * cos_el * cos_az,
        radar.range * sin_el
    };
}

[[nodiscard]] inline auto getSystemMatrix(double dt) noexcept -> SystemMatrix {
    SystemMatrix F = SystemMatrix::Identity();
    const double dt2 = 0.5 * dt * dt;

    F(0, 3) = dt; F(0, 6) = dt2;
    F(1, 4) = dt; F(1, 7) = dt2;
    F(2, 5) = dt; F(2, 8) = dt2;

    F(3, 6) = dt;
    F(4, 7) = dt;
    F(5, 8) = dt;

    return F;
}

}
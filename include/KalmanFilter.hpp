#pragma once

#include "StateVector.hpp"
#include "Measurement.hpp"
#include "EKFUtils.hpp"
#include <utility>

namespace tracking {

class KalmanFilter {
private:
    StateVector x_{StateVector::Zero()};
    StateCovariance P_{StateCovariance::Identity() * 100.0};
    StateCovariance Q_{StateCovariance::Identity() * 0.1};

public:
    KalmanFilter() = default;

    explicit KalmanFilter(StateVector initial_state) noexcept;

    void predict(double dt) noexcept;

    [[nodiscard]] auto getState() const noexcept -> const StateVector& { return x_; }
    [[nodiscard]] auto getCovariance() const noexcept -> const StateCovariance& { return P_; }

    void update(const AtsbMeasurement& atsb, const Matrix6d& R) noexcept;

    void update(const RadarMeasurement& radar, const Eigen::Matrix3d& R) noexcept;

private:
    [[nodiscard]] static auto normalizeAngle(double angle) noexcept -> double;
};

}
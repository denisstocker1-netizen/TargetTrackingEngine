#include "KalmanFilter.hpp"

#include <cmath>
#include <utility> //

#include <cmath>

namespace tracking {

    KalmanFilter::KalmanFilter(StateVector initial_state) noexcept
    : x_(std::move(initial_state)) {}

    void KalmanFilter::predict(double dt) noexcept {
    const SystemMatrix F = getSystemMatrix(dt);
    x_ = F * x_;
    P_ = F * P_ * F.transpose() + Q_;
}

    void KalmanFilter::update(const AtsbMeasurement& atsb, const Matrix6d& R) noexcept {
        Eigen::Matrix<double, 6, 9> H = Eigen::Matrix<double, 6, 9>::Zero();
        H(0, 0) = 1.0;
        H(1, 1) = 1.0;
        H(2, 2) = 1.0;
        H(3, 3) = 1.0;
        H(4, 4) = 1.0;
        H(5, 5) = 1.0;

        const auto I = StateCovariance::Identity();

        const Vector6d z{atsb.x, atsb.y, atsb.z, atsb.vx, atsb.vy, atsb.vz};
        const Vector6d y = z - (H * x_);

        const Matrix6d S = H * P_ * H.transpose() + R;
        const Eigen::Matrix<double, 9, 6> K = P_ * H.transpose() * S.inverse();

        x_ = x_ + K * y;
        P_ = (I - K * H) * P_;
    }

    auto KalmanFilter::normalizeAngle(double angle) noexcept -> double {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }

    void KalmanFilter::update(const RadarMeasurement& radar, const Eigen::Matrix3d& R) noexcept {
        const Eigen::Vector3d z_meas{radar.range, radar.azimuth, radar.elevation};
        const Eigen::Vector3d z_pred = h(x_);
        const Matrix3x9 Hj = calculateRadarJacobian(x_);

        Eigen::Vector3d y = z_meas - z_pred;
        y(1) = normalizeAngle(y(1));

        const Eigen::Matrix3d S = Hj * P_ * Hj.transpose() + R;
        const Eigen::Matrix<double, 9, 3> K = P_ * Hj.transpose() * S.inverse();

        const auto I = StateCovariance::Identity();
        x_ = x_ + K * y;
        P_ = (I - K * Hj) * P_;
    }

}
#include "EKFUtils.hpp"

#include <cmath>

namespace tracking {
    auto h(const StateVector& x) noexcept -> Eigen::Vector3d {
        const double x_pos = x(0);
        const double y_pos = x(1);
        const double z_pos = x(2);

        const double r = std::sqrt(x_pos*x_pos + y_pos*y_pos);
        const double slant_range = std::sqrt(x_pos*x_pos + y_pos*y_pos + z_pos*z_pos);

        const double azi = std::atan2(y_pos, x_pos);
        const double el = std::atan2(z_pos, r);

        return Eigen::Vector3d{slant_range, azi, el};
    }

    auto calculateRadarJacobian(const StateVector& x) noexcept -> Matrix3x9 {
        Matrix3x9 Hj = Matrix3x9::Zero();

        const double x_pos = x(0);
        const double y_pos = x(1);
        const double z_pos = x(2);

        const double r_sq = x_pos * x_pos + y_pos * y_pos;
        const double slant_range_sq = r_sq + z_pos * z_pos;

        if (r_sq < 1e-6 || slant_range_sq < 1e-6) {
            return Hj;
        }

        const double r = std::sqrt(r_sq);
        const double slant_range = std::sqrt(slant_range_sq);

        Hj(0, 0) = x_pos / slant_range;
        Hj(0, 1) = y_pos / slant_range;
        Hj(0, 2) = z_pos / slant_range;

        Hj(1, 0) = -y_pos / r_sq;
        Hj(1, 1) = x_pos / r_sq;
        Hj(1, 2) = 0.0;

        Hj(2, 0) = -(x_pos * z_pos) / (slant_range_sq * r);
        Hj(2, 1) = -(y_pos * z_pos) / (slant_range_sq * r);
        Hj(2, 2) = r / slant_range_sq;

        return Hj;
    }
}
#include <cassert>
#include <iostream>
#include "Measurement.hpp"
#include "SPSCQueue.hpp"
#include "StateVector.hpp"
#include "KalmanFilter.hpp"
#include <cmath>
#include "EKFUtils.hpp"

// Test 1: Check initial state
void test_initial_state() {
    SPSCQueue<int, 4> queue;

    assert(queue.isEmpty() == true);
    assert(queue.isFull() == false);
    assert(queue.pop().has_value() == false);

    std::cout << "[Passed] Test 1: Initial State\n";
}


// Test 2: Full state and overflow behavior
void test_overflow() {
    SPSCQueue<int, 4> queue;

    assert(queue.push(10) == true);
    assert(queue.push(20) == true);
    assert(queue.push(30) == true);

    // Queue must be full now
    assert(queue.isFull() == true);
    assert(queue.isEmpty() == false);

    assert(queue.push(40) == false);

    std::cout << "[Passed] Test 2: Full State\n";
}

// Test 3: Wrap around
void test_wrap_around() {
    SPSCQueue<int, 4> queue;

    for (int i = 0; i < 10; i++) {
        assert(queue.push(i) == true);
        auto item = queue.pop();
        assert(item.has_value() == true);
        assert(*item == i);
        assert(queue.isEmpty() == true);
    }

    std::cout << "[Passed] Test 3: Wrap around\n";
}

// Test 4: Measurement variant
void test_measurement_variant() {
    SPSCQueue<tracking::Measurement, 4> queue;

    tracking::RadarMeasurement radar{.timestamp = 100, .range = 500.0, .azimuth = 0.1, .elevation = 0.2};
    assert(queue.push(radar) == true);

    auto item = queue.pop();
    assert(item.has_value() == true);
    assert(std::holds_alternative<tracking::RadarMeasurement>(*item) == true);

    std::cout << "[Passed] Test 4: Measurement variant\n";
}

// Test 5: Kinematics and Transformation
void test_radar_to_cartesian() {
    tracking::RadarMeasurement radar{.timestamp = 1, .range = 1000.0, .azimuth = 0.0, .elevation = 0.0};
    Eigen::Vector3d pos = tracking::radarToCartesian(radar);

    assert(std::abs(pos.x() - 0.0) < 1e-6);
    assert(std::abs(pos.y() - 1000) < 1e-6);
    assert(std::abs(pos.z() - 0.0) < 1e-6);

    tracking::StateVector x = tracking::StateVector::Zero();
    x(3) = 100.0;

    tracking::SystemMatrix F = tracking::getSystemMatrix(1.0);
    tracking::StateVector x_next = F * x;

    assert(std::abs(x_next(0) - 100.0) < 1e-6);
    assert(std::abs(x_next(3) - 100.0) < 1e-6);

    std::cout << "[PASSED] Test 5: State Vector Kinematics & Transformation\n";
}

// Test 6: ADS-B Update
void test_adsb_update() {
    tracking::StateVector x_init = tracking::StateVector::Zero();
    tracking::KalmanFilter filter{x_init};

    filter.predict(1.0);

    tracking::AtsbMeasurement atsb{
        .timestamp = 1000,
        .x = 10.0,
        .y = 5.0,
        .z = 0.0,
        .vx = 200.0,
        .vy = 10.0,
        .vz = 0.0,
    };

    tracking::Matrix6d R = tracking::Matrix6d::Identity() * 1.0;

    filter.update(atsb, R);

    const auto& state = filter.getState();

    assert(std::abs(state(0) - 10.0) < 2.0);
    assert(std::abs(state(1) - 5.0) < 1.0);
    assert(state(3) > 150.0);
    assert(state(4) > 5.0);

    std::cout << "[PASSED] Test 6: ADS-B Update\n";
}

// Test 7: EKF Utilities
void test_ekf_utils() {
    tracking::StateVector x = tracking::StateVector::Zero();
    x(0) = 100.0;
    x(1) = 100.0;
    x(2) = 0.0;

    Eigen::Vector3d h_x = tracking::h(x);

    const double expected_range = std::sqrt(20000.0);
    const double expected_azi = M_PI / 4.0;
    const double expected_el = 0.0;

    assert(std::abs(h_x(0) - expected_range) < 1e-5);
    assert(std::abs(h_x(1) - expected_azi) < 1e-5);
    assert(std::abs(h_x(2) - expected_el) < 1e-5);

    tracking::Matrix3x9 Hj = tracking::calculateRadarJacobian(x);

    assert(std::abs(Hj(0, 0) - (100.0 / expected_range)) < 1e-5);
    assert(std::abs(Hj(0, 1) - (100.0 / expected_range)) < 1e-5);
    assert(std::abs(Hj(0, 2) - 0.0) < 1e-5);

    assert(std::abs(Hj(1, 0) - (-100.0 / 20000.0)) < 1e-5);
    assert(std::abs(Hj(1, 1) - (100.0 / 20000.0)) < 1e-5);

    assert(std::abs(Hj(2, 2) - (expected_range / 20000.0)) < 1e-5);

    assert(Hj(0, 3) == 0.0 && Hj(1, 4) == 0.0 && Hj(2, 5) == 0.0);

    std::cout << "[PASSED] Test 7: EKF Utilities (h(x) & Jacobian Hj)\n";
}

// Test 8: Extended Kalman Filter Radar Update
void test_radar_ekf_update() {
    tracking::StateVector x_init = tracking::StateVector::Zero();
    x_init(0) = 100.0;

    tracking::KalmanFilter filter{x_init};

    tracking::RadarMeasurement radar {
        .timestamp = 1000,
        .range = 105.0,
        .azimuth = 0.05,
        .elevation = 0.0
    };

    Eigen::Matrix3d R = Eigen::Matrix3d::Zero();
    R(0, 0) = 1.0;
    R(1, 1) = 0.001;
    R(2, 2) = 0.001;

    const auto p_prior = filter.getCovariance();

    filter.update(radar, R);

    const auto& state = filter.getState();
    const auto& p_post = filter.getCovariance();

    assert(state(0) > 100.0 && state(0) < 105.0);
    
    assert(state(1) > 0.0);

    assert(p_post(0, 0) < p_prior(0, 0));
    assert(p_post(1, 1) < p_prior(1, 1));

    std::cout << "[PASSED] Test 8: Extended Kalman Filter Radar Update\n";
}

auto main() -> int {
    std::cout << "=== Running Unit Tests for SPSCQueue ===\n";

    test_initial_state();
    test_overflow();
    test_wrap_around();
    test_measurement_variant();
    test_radar_to_cartesian();
    test_adsb_update();
    test_ekf_utils();
    test_radar_ekf_update();

    std::cout << "=== All Unit Tests Passed Successfully! ===\n";
    return 0;
}
#include <chrono>
#include <iostream>
#include <thread>
#include <variant>

#include "include/KalmanFilter.hpp"
#include "include/Measurement.hpp"
#include "include/SPSCQueue.hpp"

auto main() -> int {
    std::cout << "=== Starting Target Tracking Engine Demonstration ===\n\n";

    SPSCQueue<tracking::Measurement, 10> queue;
    tracking::KalmanFilter filter;

    
    Eigen::Matrix3d R_radar = Eigen::Matrix3d::Identity() * 0.1;
    tracking::Matrix6d R_atsb = tracking::Matrix6d::Identity() * 0.5;

    // 1. Producer Thread
    std::jthread producer([&queue](const std::stop_token& stop_tok) {
        uint64_t ts = 0;

        while (!stop_tok.stop_requested()) {
            const auto ts_d = static_cast<double>(++ts);

            if (ts % 2 == 0) {
                tracking::RadarMeasurement radar{
                    .timestamp = ts,
                    .range = 1500.0 + (ts_d * 10.0),
                    .azimuth = 0.5,
                    .elevation = 0.1
                };

                if (queue.push(radar)) {
                    std::cout << "[Producer] Pushed Radar Measurement (TS: " << ts << ")\n";
                }
            } else {
                tracking::AtsbMeasurement adsb{
                    .timestamp = ts,
                    .x = 1000.0 + (ts_d * 15.0),
                    .y = 500.0,
                    .z = 200.0,
                    .vx = 150.0,
                    .vy = 0.0,
                    .vz = 0.0
                };

                if (queue.push(adsb)) {
                    std::cout << "[Producer] Pushed ADS-B Measurement  (TS: " << ts << ")\n";
                }
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    });

    // 2. Consumer Thread
    std::jthread consumer([&queue, &filter, &R_radar, &R_atsb](const std::stop_token& stop_tok) {
        constexpr double dt = 0.1;

        while (!stop_tok.stop_requested()) {
            auto item = queue.pop();

            if (item.has_value()) {
                filter.predict(dt);

                std::visit([&filter, &R_radar, &R_atsb](const auto& meas) {
                    using T = std::decay_t<decltype(meas)>;
                    if constexpr (std::is_same_v<T, tracking::RadarMeasurement>) {
                        filter.update(meas, R_radar);
                    } else if constexpr (std::is_same_v<T, tracking::AtsbMeasurement>) {
                        filter.update(meas, R_atsb);
                    }
                }, item.value());

                const auto& state = filter.getState();
                std::cout << "  [Consumer] Filter Updated -> State Pos (x,y,z): ["
                          << state(0) << ", " << state(1) << ", " << state(2) << "]\n";
            } else {
                std::this_thread::sleep_for(std::chrono::microseconds(50));
            }
        }
    });

    std::this_thread::sleep_for(std::chrono::seconds(2));

    std::cout << "\n=== Demonstration Finished Successfully ===\n";
    return 0;
}
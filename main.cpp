#include <iostream>
#include "include/Measurement.hpp"
#include "include/SPSCQueue.hpp"
#include <thread>
#include <chrono>

auto main() -> int {
    SPSCQueue<tracking::Measurement, 10> queue;

    std::jthread producer([&queue](std::stop_token stop_tok) {
        uint64_t ts = 0;

        while (!stop_tok.stop_requested()){
            tracking::RadarMeasurement radar{.timestamp = ++ts, .range = 1500.0, .azimuth = 0.5, .elevation = 0.1};

            if (queue.push(radar)) {
                std::cout << "[Producer] Radar-Packet pushed! TS: " << ts << "\n";
            }else {
                std::cout << "[Producer] Queue is full! Packet dismissed.\n";
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    });

    std::jthread consumer([&queue](std::stop_token stop_tok) {
        while(!stop_tok.stop_requested()){
            auto item = queue.pop();

            if(item.has_value()) {
                std::cout << "[Consumer] Packet received!\n";
            }else {
                std::this_thread::sleep_for(std::chrono::microseconds(10));
            }
        }
    });

    std::this_thread::sleep_for(std::chrono::seconds(2));

    return 0;
}
#pragma once

#include <cstdint>
#include <iostream>
#include <variant>

namespace tracking {
    struct RadarMeasurement {
    uint64_t timestamp;
    double range;
    double azimuth;
    double elevation;
};

struct AtsbMeasurement {
    uint64_t timestamp;
    double x, y, z;
    double vx, vy, vz;
};

using Measurement = std::variant<RadarMeasurement, AtsbMeasurement>;

static_assert(std::is_trivially_copyable_v<RadarMeasurement>, "RadarMeasurement must be trivially copyable!");
static_assert(std::is_trivially_copyable_v<AtsbMeasurement>, "AtsbMeasurement must be trivially copyable!");
static_assert(std::is_trivially_copyable_v<Measurement>, "Measurement must be trivially copyable!");
}



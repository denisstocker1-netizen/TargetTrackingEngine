# High-Performance Multi-Sensor Target Tracking Engine (C++20)

A modular, real-time target tracking engine written in modern C++20. The engine processes heterogeneous sensor data (Cartesian ADS-B and spherical Primary Radar measurements) through a lock-free Single-Producer Single-Consumer (SPSC) queue, applying an Extended Kalman Filter (EKF) with continuous kinematic prediction and measurement updating.

![C++20](https://img.shields.io/badge/C%2B%2B-20-blue.svg)
![Build](https://img.shields.io/badge/build-passing-brightgreen.svg)
![License](https://img.shields.io/badge/license-MIT-blue.svg)

---

## ── System Architecture

```text
                               ┌────────────────────────────────────────────────────────┐
                               │                    Producer Thread                     │
                               │        (e.g., Radar / ADS-B Measurement Stream)     │
                               └───────────────────────────┬────────────────────────────┘
                                                           │
                                                           ▼
                               ┌────────────────────────────────────────────────────────┐
                               │                 SPSCQueue<Measurement>                 │
                               │        (Lock-free, Thread-safe Ring Buffer)            │
                               └───────────────────────────┬────────────────────────────┘
                                                           │
                                                           ▼
                               ┌────────────────────────────────────────────────────────┐
                               │                    Consumer Thread                     │
                               │            (Measurement Dispatch via std::visit)       │
                               └───────────────────────────┬────────────────────────────┘
                                                           │
                                                           ▼
                               ┌────────────────────────────────────────────────────────┐
                               │                      KalmanFilter                      │
                               │         Predict:  x_k = F*x,  P_k = F*P*F^T + Q        │
                               ├───────────────────────────┬────────────────────────────┤
                               │                           │                            │
                     AtsbMeasurement (Linear)              RadarMeasurement (EKF)
                     - Linear H matrix (6x9)               - Non-linear observation h(x)
                     - Kalman Gain K = P*H^T*S^-1          - 3x9 Jacobian matrix Hj
                                                           - Azimuth angle unwrapping
                                                           │                            │
                               └───────────────────────────┴────────────────────────────┘
```
## ── Key Features

* **Lock-Free Concurrency:** High-throughput `SPSCQueue` using atomic operations (`std::atomic`) with explicit acquire/release memory orders and cache-line alignment (`alignas(64)`) to prevent false sharing.
* **Extended Kalman Filtering (EKF):**
  * **9D State Vector:** Position $(x, y, z)$, Velocity $(v_x, v_y, v_z)$, and Acceleration $(a_x, a_y, a_z)$.
  * **Non-Linear Observation Model:** $h(x)$ transforms cartesian state estimates to spherical radar coordinates $(\text{Range } \rho, \text{Azimuth } \theta, \text{Elevation } \phi)$.
  * **Analytical Jacobian Computation:** Dynamic $3 \times 9$ Jacobian matrix $H_j$ evaluated at each step with singularity safeguards.
  * **Angle Unwrapping:** Normalizes azimuth innovations to $[-\pi, \pi]$ to prevent filter divergence around boundary discontinuities.
* **Modern C++20 Practices:** Utilizes `std::jthread`, `std::stop_token`, `std::variant`, `std::visit`, compile-time concepts, `noexcept` guarantees, and explicit move semantics.
* **Rigorous Build System & Tooling:** CMake architecture with a dedicated static library target (`TargetTrackingEngine_lib`), strict compiler flags (`-Wall -Wextra -Wpedantic -Werror`), Clang-Tidy static analysis integration, and comprehensive unit test coverage via CTest.

---

## ── Mathematical Model

### State Vector ($9\text{D}$)
$$\mathbf{x} = \begin{bmatrix} x & y & z & v_x & v_y & v_z & a_x & a_y & a_z \end{bmatrix}^T$$

### Radar Observation Function $h(\mathbf{x})$
$$h(\mathbf{x}) = \begin{bmatrix}  \sqrt{x^2 + y^2 + z^2} \\ \operatorname{atan2}(y, x) \\ \operatorname{atan2}(z, \sqrt{x^2 + y^2}) \end{bmatrix}$$

---

## ── Project Structure

```text
TargetTrackingEngine/
├── CMakeLists.txt             # Modern CMake build configuration
├── main.cpp                   # Multi-threaded pipeline demonstration
├── include/                   # Public Header Files
│   ├── StateVector.hpp        # Eigen aliases for 9D state & system matrices
│   ├── Measurement.hpp        # ATSB & Radar structs + std::variant
│   ├── SPSCQueue.hpp          # Lock-free SPSC Queue implementation
│   ├── EKFUtils.hpp           # Declarations for h(x) and Jacobian Hj
│   └── KalmanFilter.hpp       # KalmanFilter class interface
├── src/                       # Library Implementation
│   ├── EKFUtils.cpp           # Non-linear observation and Jacobian math
│   └── KalmanFilter.cpp       # Predict and Overloaded Update implementations
└── tests/                     # Unit Testing Suite
    └── test_main.cpp          # 8 Comprehensive CTest cases
```
## ── Building and Testing

### Prerequisites
* **Compiler:** GCC 11+ or Clang 13+ (C++20 support required)
* **Build System:** CMake $\ge$ 3.20
* **Dependencies:** [Eigen3](https://eigen.tuxfamily.org/) (Linear algebra library)

### Build Instructions

```bash
# Clone repository
git clone [https://github.com/denisstocker1-netizen/TargetTrackingEngine.git](https://github.com/denisstocker1-netizen/TargetTrackingEngine.git)
cd TargetTrackingEngine

# Configure project
cmake -B build -DCMAKE_BUILD_TYPE=Release

# Build library, demo executable, and test suite
cmake --build build

# Run unit tests
cd build && ctest --output-on-failure

## ── License
This project is licensed under the MIT License - see the LICENSE file for details.
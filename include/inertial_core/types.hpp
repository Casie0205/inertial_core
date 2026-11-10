#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace inertial_core {

// The IMU increment is defined over the interval ending at the state time.
// Angular increments are in radians and velocity increments are in m/s.
struct ImuIncrement {
    double dt_s{0.0};
    Eigen::Vector3d delta_angle_rad{Eigen::Vector3d::Zero()};
    Eigen::Vector3d delta_velocity_mps{Eigen::Vector3d::Zero()};
};

// Position is geodetic [latitude, longitude, height].
// Latitude and longitude are radians, height is metres.
// Velocity is [north, east, down] in m/s.
// q_bn rotates a vector from body FRD coordinates to navigation NED coordinates.
struct NavState {
    double time_s{0.0};
    Eigen::Vector3d position_rad_m{Eigen::Vector3d::Zero()};
    Eigen::Vector3d velocity_ned_mps{Eigen::Vector3d::Zero()};
    Eigen::Quaterniond q_bn{Eigen::Quaterniond::Identity()};
};

inline Eigen::Quaterniond normalizedAttitude(const Eigen::Quaterniond& q_bn) {
    if (!q_bn.coeffs().allFinite() || q_bn.norm() <= 0.0) {
        return Eigen::Quaterniond::Identity();
    }
    return q_bn.normalized();
}

}  // namespace inertial_core

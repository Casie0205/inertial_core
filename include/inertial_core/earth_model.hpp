#pragma once

#include <Eigen/Core>

#include "inertial_core/runtime_config.hpp"
#include "inertial_core/types.hpp"

namespace inertial_core {

using EarthModelOptions = RuntimeConfig;

struct EarthParameters {
    Eigen::Vector2d meridian_prime_vertical_radii_m{Eigen::Vector2d::Zero()};
    Eigen::Vector3d omega_ie_ned_radps{Eigen::Vector3d::Zero()};
    Eigen::Vector3d omega_en_ned_radps{Eigen::Vector3d::Zero()};
    Eigen::Vector3d omega_in_ned_radps{Eigen::Vector3d::Zero()};
    double gravity_mps2{0.0};
};

class EarthModel {
public:
    static EarthParameters evaluate(
        const Eigen::Vector3d& position_rad_m,
        const Eigen::Vector3d& velocity_ned_mps,
        const EarthModelOptions& options = {});

    static Eigen::Vector2d meridianPrimeVerticalRadii(
        double latitude_rad,
        const EarthModelOptions& options = {});

    static double normalGravity(
        double latitude_rad,
        double height_m,
        const EarthModelOptions& options = {});

    static Eigen::Vector3d earthRateNed(
        double latitude_rad,
        const EarthModelOptions& options = {});

    static Eigen::Vector3d transportRateNed(
        const Eigen::Vector2d& radii_m,
        const Eigen::Vector3d& position_rad_m,
        const Eigen::Vector3d& velocity_ned_mps,
        const EarthModelOptions& options = {});
};

}  // namespace inertial_core

#pragma once

#include <Eigen/Core>

#include "inertial_core/types.hpp"

namespace inertial_core {

enum class GravityModel {
    kConstant,
    kNormalGravity,
};

// Options are deliberately independent so an experiment can change one
// physical term at a time.
struct EarthModelOptions {
    bool enable_earth_rotation{true};
    bool enable_transport_rate{true};
    bool enable_coriolis{true};
    bool enable_ellipsoid_radii{true};
    GravityModel gravity_model{GravityModel::kNormalGravity};

    double earth_rotation_rate_radps{7.2921151467e-5};
    double constant_gravity_mps2{9.80665};
    double semi_major_axis_m{6378137.0};
    double eccentricity_squared{0.0066943799901413156};
    double fallback_sphere_radius_m{6371000.0};
};

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

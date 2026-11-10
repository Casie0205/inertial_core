#include "inertial_core/earth_model.hpp"

#include <cmath>
#include <stdexcept>

namespace inertial_core {
namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr double kHalfPi = kPi * 0.5;

void validateOptions(const EarthModelOptions& options) {
    if (!std::isfinite(options.earth_rotation_rate_radps) ||
        options.earth_rotation_rate_radps < 0.0) {
        throw std::invalid_argument("earth rotation rate must be finite and nonnegative");
    }
    if (!std::isfinite(options.constant_gravity_mps2) ||
        options.constant_gravity_mps2 <= 0.0) {
        throw std::invalid_argument("constant gravity must be finite and positive");
    }
    if (!std::isfinite(options.semi_major_axis_m) ||
        options.semi_major_axis_m <= 0.0 ||
        !std::isfinite(options.eccentricity_squared) ||
        options.eccentricity_squared < 0.0 ||
        options.eccentricity_squared >= 1.0 ||
        !std::isfinite(options.fallback_sphere_radius_m) ||
        options.fallback_sphere_radius_m <= 0.0) {
        throw std::invalid_argument("earth geometry parameters are invalid");
    }
    if (!std::isfinite(options.equal_interval_tolerance_s) ||
        options.equal_interval_tolerance_s < 0.0) {
        throw std::invalid_argument(
            "equal-interval tolerance must be finite and nonnegative");
    }
}

void validatePositionVelocity(
    const Eigen::Vector3d& position_rad_m,
    const Eigen::Vector3d& velocity_ned_mps) {
    if (!position_rad_m.allFinite() || !velocity_ned_mps.allFinite()) {
        throw std::invalid_argument("position and velocity must be finite");
    }
    if (position_rad_m.x() < -kHalfPi || position_rad_m.x() > kHalfPi) {
        throw std::invalid_argument("latitude must be in [-pi/2, pi/2]");
    }
}

}  // namespace

Eigen::Vector2d EarthModel::meridianPrimeVerticalRadii(
    const double latitude_rad,
    const EarthModelOptions& options) {
    validateOptions(options);
    if (!std::isfinite(latitude_rad) ||
        latitude_rad < -kHalfPi ||
        latitude_rad > kHalfPi) {
        throw std::invalid_argument("latitude must be finite and in [-pi/2, pi/2]");
    }
    if (!options.enable_ellipsoid_radii) {
        return Eigen::Vector2d::Constant(options.fallback_sphere_radius_m);
    }

    const double sin_lat = std::sin(latitude_rad);
    const double denominator =
        1.0 - options.eccentricity_squared * sin_lat * sin_lat;
    const double sqrt_denominator = std::sqrt(denominator);
    const double prime_vertical =
        options.semi_major_axis_m / sqrt_denominator;
    const double meridian =
        options.semi_major_axis_m * (1.0 - options.eccentricity_squared) /
        (denominator * sqrt_denominator);
    return Eigen::Vector2d(meridian, prime_vertical);
}

double EarthModel::normalGravity(
    const double latitude_rad,
    const double height_m,
    const EarthModelOptions& options) {
    validateOptions(options);
    if (!std::isfinite(latitude_rad) ||
        !std::isfinite(height_m) ||
        latitude_rad < -kHalfPi ||
        latitude_rad > kHalfPi) {
        throw std::invalid_argument("gravity arguments are invalid");
    }
    if (options.gravity_model == GravityModel::kConstant) {
        return options.constant_gravity_mps2;
    }

    const double sin_lat = std::sin(latitude_rad);
    const double sin2 = sin_lat * sin_lat;
    const double sin4 = sin2 * sin2;
    const double sin6 = sin4 * sin2;
    const double sin8 = sin4 * sin4;

    const double gravity_at_height =
        9.7803267715 *
            (1.0 + 0.0052790414 * sin2 + 0.0000232718 * sin4 +
             0.0000001262 * sin6 + 0.0000000007 * sin8) -
        (3.0877e-6 - 4.3e-9 * sin2) * height_m +
        0.72e-12 * height_m * height_m;
    return gravity_at_height;
}

Eigen::Vector3d EarthModel::earthRateNed(
    const double latitude_rad,
    const EarthModelOptions& options) {
    validateOptions(options);
    if (!std::isfinite(latitude_rad) ||
        latitude_rad < -kHalfPi ||
        latitude_rad > kHalfPi) {
        throw std::invalid_argument("latitude must be finite and in [-pi/2, pi/2]");
    }
    if (!options.enable_earth_rotation) {
        return Eigen::Vector3d::Zero();
    }
    return Eigen::Vector3d(
        options.earth_rotation_rate_radps * std::cos(latitude_rad),
        0.0,
        -options.earth_rotation_rate_radps * std::sin(latitude_rad));
}

Eigen::Vector3d EarthModel::transportRateNed(
    const Eigen::Vector2d& radii_m,
    const Eigen::Vector3d& position_rad_m,
    const Eigen::Vector3d& velocity_ned_mps,
    const EarthModelOptions& options) {
    if (!options.enable_transport_rate) {
        return Eigen::Vector3d::Zero();
    }

    const double latitude_rad = position_rad_m.x();
    const double height_m = position_rad_m.z();
    const double rm_h = radii_m.x() + height_m;
    const double rn_h = radii_m.y() + height_m;
    const double cos_lat = std::cos(latitude_rad);
    if (rm_h <= 0.0 || rn_h <= 0.0 || std::abs(cos_lat) <= 1e-15) {
        throw std::invalid_argument("earth radii and latitude are invalid");
    }

    return Eigen::Vector3d(
        velocity_ned_mps.y() / rn_h,
        -velocity_ned_mps.x() / rm_h,
        -velocity_ned_mps.y() * std::tan(latitude_rad) / rn_h);
}

EarthParameters EarthModel::evaluate(
    const Eigen::Vector3d& position_rad_m,
    const Eigen::Vector3d& velocity_ned_mps,
    const EarthModelOptions& options) {
    validateOptions(options);
    validatePositionVelocity(position_rad_m, velocity_ned_mps);

    EarthParameters parameters;
    parameters.meridian_prime_vertical_radii_m =
        meridianPrimeVerticalRadii(position_rad_m.x(), options);
    parameters.omega_ie_ned_radps =
        earthRateNed(position_rad_m.x(), options);
    parameters.omega_en_ned_radps =
        transportRateNed(
            parameters.meridian_prime_vertical_radii_m,
            position_rad_m,
            velocity_ned_mps,
            options);
    parameters.omega_in_ned_radps =
        parameters.omega_ie_ned_radps + parameters.omega_en_ned_radps;
    parameters.gravity_mps2 =
        normalGravity(position_rad_m.x(), position_rad_m.z(), options);
    return parameters;
}

}  // namespace inertial_core

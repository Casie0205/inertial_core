#include "inertial_core/earth_model.hpp"
#include "inertial_core/mechanization.hpp"

#include <Eigen/Geometry>

#include <cmath>
#include <cstdio>
#include <stdexcept>

namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr double kDegToRad = kPi / 180.0;

int g_failures = 0;

void check(const bool condition, const char* name, const double value = 0.0) {
    if (condition) {
        std::printf("[ok]   %s\n", name);
    } else {
        std::printf("[FAIL] %s (%g)\n", name, value);
        ++g_failures;
    }
}

bool near(const double value, const double expected, const double tolerance) {
    return std::abs(value - expected) <= tolerance;
}

inertial_core::EarthModelOptions noEarthOptions() {
    inertial_core::EarthModelOptions options;
    options.enable_earth_rotation = false;
    options.enable_transport_rate = false;
    options.enable_coriolis = false;
    options.enable_ellipsoid_radii = false;
    options.gravity_model = inertial_core::GravityModel::kConstant;
    options.constant_gravity_mps2 = 9.80665;
    return options;
}

}  // namespace

int main() {
    const double latitude = 30.5 * kDegToRad;
    const double longitude = 114.0 * kDegToRad;
    const double height = 20.0;
    const Eigen::Vector3d position(latitude, longitude, height);
    const Eigen::Vector3d zero_velocity = Eigen::Vector3d::Zero();
    const inertial_core::EarthModelOptions earth_options;

    const Eigen::Vector2d radii =
        inertial_core::EarthModel::meridianPrimeVerticalRadii(
            latitude, earth_options);
    check(radii.x() > 6.3e6 && radii.x() < 6.4e6, "meridian radius is plausible", radii.x());
    check(radii.y() > radii.x(), "prime vertical radius exceeds meridian radius");
    check(
        near(
            inertial_core::EarthModel::normalGravity(latitude, height, earth_options),
            9.7935,
            2e-3),
        "normal gravity at the test latitude is plausible");

    const Eigen::Vector3d omega_ie =
        inertial_core::EarthModel::earthRateNed(latitude, earth_options);
    constexpr double earth_rate_radps = 7.2921151467e-5;
    const Eigen::Vector3d expected_omega(
        earth_rate_radps * std::cos(latitude),
        0.0,
        -earth_rate_radps * std::sin(latitude));
    check(
        (omega_ie - expected_omega).norm() < 1e-15,
        "Earth rate matches the independent NED formula",
        (omega_ie - expected_omega).norm());
    check(omega_ie.x() > 0.0 && std::abs(omega_ie.y()) < 1e-18 && omega_ie.z() < 0.0,
          "Earth rate uses the NED convention");

    inertial_core::EarthModelOptions no_earth = noEarthOptions();
    check(
        inertial_core::EarthModel::earthRateNed(latitude, no_earth).isZero(0.0),
        "Earth rate can be disabled");

    inertial_core::NavState earth_static;
    earth_static.position_rad_m = position;
    const inertial_core::EarthParameters earth_static_parameters =
        inertial_core::EarthModel::evaluate(position, zero_velocity, earth_options);
    inertial_core::ImuIncrement earth_static_increment;
    earth_static_increment.dt_s = 0.005;
    earth_static_increment.delta_angle_rad =
        expected_omega * earth_static_increment.dt_s;
    earth_static_increment.delta_velocity_mps =
        Eigen::Vector3d(
            0.0, 0.0,
            -earth_static_parameters.gravity_mps2 *
                earth_static_increment.dt_s);
    inertial_core::MechanizationRunner earth_static_runner(
        earth_static, earth_options);
    for (int index = 0; index < 200; ++index) {
        earth_static_runner.step(earth_static_increment);
    }
    const inertial_core::NavState earth_static_end =
        earth_static_runner.state();
    check(
        earth_static_end.velocity_ned_mps.norm() < 2e-6,
        "Earth-on ideal static velocity remains bounded",
        earth_static_end.velocity_ned_mps.norm());
    check(
        near(earth_static_end.position_rad_m.z(), height, 2e-6),
        "Earth-on ideal static height remains bounded",
        earth_static_end.position_rad_m.z() - height);
    const Eigen::Quaterniond attitude_error =
        earth_static_end.q_bn.conjugate();
    check(
        std::abs(attitude_error.w()) > 1.0 - 1e-9,
        "Earth-on ideal static attitude remains fixed",
        1.0 - std::abs(attitude_error.w()));

    inertial_core::NavState free_fall;
    free_fall.position_rad_m = position;
    inertial_core::ImuIncrement zero_increment;
    zero_increment.dt_s = 0.005;
    inertial_core::MechanizationRunner free_fall_runner(free_fall, no_earth);
    for (int index = 0; index < 200; ++index) {
        free_fall_runner.step(zero_increment);
    }
    const inertial_core::NavState free_fall_end = free_fall_runner.state();
    check(
        near(free_fall_end.velocity_ned_mps.z(), no_earth.constant_gravity_mps2, 2e-2),
        "free fall accumulates constant gravity",
        free_fall_end.velocity_ned_mps.z());
    check(
        near(
            free_fall_end.position_rad_m.z(),
            height - 0.5 * no_earth.constant_gravity_mps2,
            1e-2),
        "free fall position uses midpoint velocity",
        free_fall_end.position_rad_m.z());

    inertial_core::NavState supported;
    supported.position_rad_m = position;
    inertial_core::MechanizationRunner supported_runner(supported, no_earth);
    inertial_core::ImuIncrement supported_increment;
    supported_increment.dt_s = 0.005;
    supported_increment.delta_velocity_mps =
        Eigen::Vector3d(0.0, 0.0, -no_earth.constant_gravity_mps2 * 0.005);
    for (int index = 0; index < 200; ++index) {
        supported_runner.step(supported_increment);
    }
    const inertial_core::NavState supported_end = supported_runner.state();
    check(
        supported_end.velocity_ned_mps.norm() < 1e-10,
        "specific force cancels gravity in the no-Earth baseline",
        supported_end.velocity_ned_mps.norm());
    check(
        near(supported_end.position_rad_m.z(), height, 1e-9),
        "supported static point keeps its height",
        supported_end.position_rad_m.z() - height);

    inertial_core::NavState moving;
    moving.position_rad_m = position;
    moving.velocity_ned_mps = Eigen::Vector3d(5.0, 2.0, 0.0);
    inertial_core::ImuIncrement balanced_increment;
    balanced_increment.dt_s = 0.01;
    balanced_increment.delta_velocity_mps =
        Eigen::Vector3d(0.0, 0.0, -no_earth.constant_gravity_mps2 * 0.01);
    const inertial_core::NavState moving_end =
        inertial_core::Mechanization::propagate(
            moving, balanced_increment, no_earth);
    check(
        near(moving_end.velocity_ned_mps.x(), 5.0, 1e-10) &&
            near(moving_end.velocity_ned_mps.y(), 2.0, 1e-10),
        "constant-velocity baseline preserves horizontal velocity");
    check(
        moving_end.position_rad_m.x() > latitude &&
            moving_end.position_rad_m.y() > longitude &&
            near(moving_end.position_rad_m.z(), height, 1e-8),
        "north/east motion increases latitude and longitude without height drift");

    bool invalid_dt_thrown = false;
    try {
        inertial_core::ImuIncrement invalid;
        inertial_core::Mechanization::propagate(moving, invalid, no_earth);
    } catch (const std::invalid_argument&) {
        invalid_dt_thrown = true;
    }
    check(invalid_dt_thrown, "non-positive dt is rejected");

    bool unequal_dt_thrown = false;
    try {
        inertial_core::ImuIncrement previous;
        previous.dt_s = 0.005;
        inertial_core::ImuIncrement current;
        current.dt_s = 0.006;
        inertial_core::Mechanization::propagate(
            moving, previous, current, no_earth);
    } catch (const std::invalid_argument&) {
        unequal_dt_thrown = true;
    }
    check(unequal_dt_thrown, "unequal adjacent dt is rejected");

    if (g_failures == 0) {
        std::printf("inertial_core_tests: all checks passed\n");
        return 0;
    }
    std::printf("inertial_core_tests: %d checks failed\n", g_failures);
    return 1;
}

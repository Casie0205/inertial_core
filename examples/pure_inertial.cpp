#include "inertial_core/mechanization.hpp"

#include <cstdio>

int main() {
    inertial_core::NavState initial;
    initial.position_rad_m =
        Eigen::Vector3d(30.5 * 3.14159265358979323846 / 180.0,
                        114.0 * 3.14159265358979323846 / 180.0,
                        20.0);

    inertial_core::EarthModelOptions options;
    inertial_core::MechanizationRunner runner(initial, options);

    const inertial_core::EarthParameters earth =
        inertial_core::EarthModel::evaluate(
            initial.position_rad_m,
            initial.velocity_ned_mps,
            options);
    inertial_core::ImuIncrement increment;
    increment.dt_s = 0.005;
    increment.delta_velocity_mps =
        Eigen::Vector3d(0.0, 0.0, -earth.gravity_mps2 * increment.dt_s);
    increment.delta_angle_rad =
        earth.omega_ie_ned_radps * increment.dt_s;

    for (int index = 0; index < 200; ++index) {
        runner.step(increment);
    }

    const auto& state = runner.state();
    std::printf(
        "t=%.1f s pos=[%.12f, %.12f, %.9f] vel=[%.9f, %.9f, %.9f]\n",
        state.time_s,
        state.position_rad_m.x(),
        state.position_rad_m.y(),
        state.position_rad_m.z(),
        state.velocity_ned_mps.x(),
        state.velocity_ned_mps.y(),
        state.velocity_ned_mps.z());
    return 0;
}

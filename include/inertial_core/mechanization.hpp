#pragma once

#include "inertial_core/earth_model.hpp"
#include "inertial_core/types.hpp"

namespace inertial_core {

struct MechanizationDiagnostics {
    EarthParameters earth_at_start;
    EarthParameters earth_at_midpoint;
    Eigen::Vector3d specific_force_increment_nav_mps{Eigen::Vector3d::Zero()};
    Eigen::Vector3d gravity_coriolis_increment_mps{Eigen::Vector3d::Zero()};
};

class Mechanization {
public:
    // Propagates one IMU interval. The previous increment is used only for the
    // second-order coning and sculling corrections. Pass a zero previous
    // increment for the first sample.
    static NavState propagate(
        const NavState& previous,
        const ImuIncrement& previous_increment,
        const ImuIncrement& increment,
        const EarthModelOptions& options = {},
        MechanizationDiagnostics* diagnostics = nullptr);

    // Convenience overload for callers that do not retain a previous sample.
    // It omits only the adjacent-sample cross terms.
    static NavState propagate(
        const NavState& previous,
        const ImuIncrement& increment,
        const EarthModelOptions& options = {},
        MechanizationDiagnostics* diagnostics = nullptr);
};

class MechanizationRunner {
public:
    explicit MechanizationRunner(
        NavState initial_state,
        EarthModelOptions options = {});

    NavState step(
        const ImuIncrement& increment,
        MechanizationDiagnostics* diagnostics = nullptr);

    const NavState& state() const { return state_; }
    const EarthModelOptions& options() const { return options_; }

    void reset(NavState initial_state);

private:
    NavState state_;
    EarthModelOptions options_;
    ImuIncrement previous_increment_{};
    bool has_previous_increment_{false};
};

}  // namespace inertial_core

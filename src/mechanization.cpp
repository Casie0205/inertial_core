#include "inertial_core/mechanization.hpp"

#include <Eigen/Geometry>

#include <cmath>
#include <stdexcept>
#include <utility>

namespace inertial_core {
namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr double kEqualIntervalToleranceS = 1e-9;

Eigen::Matrix3d skew(const Eigen::Vector3d& vector) {
    Eigen::Matrix3d matrix;
    matrix << 0.0, -vector.z(), vector.y(),
        vector.z(), 0.0, -vector.x(),
        -vector.y(), vector.x(), 0.0;
    return matrix;
}

Eigen::Quaterniond expMap(const Eigen::Vector3d& rotation_vector_rad) {
    const double angle = rotation_vector_rad.norm();
    if (angle < 1e-15) {
        return Eigen::Quaterniond::Identity();
    }
    return Eigen::Quaterniond(
        Eigen::AngleAxisd(angle, rotation_vector_rad / angle));
}

Eigen::Vector3d quaternionVector(const Eigen::Quaterniond& quaternion) {
    const Eigen::Quaterniond normalized = quaternion.normalized();
    const double sign = normalized.w() < 0.0 ? -1.0 : 1.0;
    const double vector_norm =
        Eigen::Vector3d(normalized.x(), normalized.y(), normalized.z()).norm();
    if (vector_norm < 1e-15) {
        return Eigen::Vector3d::Zero();
    }
    const double angle = 2.0 * std::atan2(vector_norm, sign * normalized.w());
    return sign * normalized.vec() * (angle / vector_norm);
}

Eigen::Vector3d positionRate(
    const Eigen::Vector3d& position_rad_m,
    const Eigen::Vector2d& radii_m,
    const Eigen::Vector3d& velocity_ned_mps) {
    const double latitude = position_rad_m.x();
    const double height = position_rad_m.z();
    const double rm_h = radii_m.x() + height;
    const double rn_h = radii_m.y() + height;
    const double cos_latitude = std::cos(latitude);
    if (rm_h <= 0.0 || rn_h <= 0.0 || std::abs(cos_latitude) <= 1e-15) {
        throw std::invalid_argument("position rate is singular");
    }
    return Eigen::Vector3d(
        velocity_ned_mps.x() / rm_h,
        velocity_ned_mps.y() / (rn_h * cos_latitude),
        -velocity_ned_mps.z());
}

Eigen::Quaterniond navToEcefQuaternion(const Eigen::Vector3d& position_rad_m) {
    const double latitude = position_rad_m.x();
    const double longitude = position_rad_m.y();
    const double cos_lon = std::cos(longitude * 0.5);
    const double sin_lon = std::sin(longitude * 0.5);
    const double cos_lat = std::cos(-kPi * 0.25 - latitude * 0.5);
    const double sin_lat = std::sin(-kPi * 0.25 - latitude * 0.5);

    Eigen::Quaterniond q_ne;
    q_ne.w() = cos_lat * cos_lon;
    q_ne.x() = -sin_lat * sin_lon;
    q_ne.y() = sin_lat * cos_lon;
    q_ne.z() = cos_lat * sin_lon;
    return q_ne.normalized();
}

Eigen::Vector3d quaternionToGeodetic(
    const Eigen::Quaterniond& q_ne,
    const double height_m) {
    const Eigen::Quaterniond q = q_ne.normalized();
    if (std::abs(q.w()) < 1e-15) {
        throw std::invalid_argument("attitude geometry is singular near a pole");
    }
    return Eigen::Vector3d(
        -2.0 * std::atan(q.y() / q.w()) - kPi * 0.5,
        2.0 * std::atan2(q.z(), q.w()),
        height_m);
}

Eigen::Vector3d gravityCoriolisIncrement(
    const EarthParameters& earth,
    const Eigen::Vector3d& velocity_ned_mps,
    const EarthModelOptions& options,
    const double dt_s) {
    const Eigen::Vector3d gravity_ned(0.0, 0.0, earth.gravity_mps2);
    Eigen::Vector3d rate = earth.omega_en_ned_radps;
    if (options.enable_coriolis) {
        rate += 2.0 * earth.omega_ie_ned_radps;
    }
    return (gravity_ned - rate.cross(velocity_ned_mps)) * dt_s;
}

void validateIncrement(const ImuIncrement& increment) {
    if (!std::isfinite(increment.dt_s) || increment.dt_s <= 0.0) {
        throw std::invalid_argument("IMU increment dt must be finite and positive");
    }
    if (!increment.delta_angle_rad.allFinite() ||
        !increment.delta_velocity_mps.allFinite()) {
        throw std::invalid_argument("IMU increment values must be finite");
    }
}

void validateState(const NavState& state) {
    if (!std::isfinite(state.time_s) ||
        !state.position_rad_m.allFinite() ||
        !state.velocity_ned_mps.allFinite() ||
        !state.q_bn.coeffs().allFinite()) {
        throw std::invalid_argument("navigation state values must be finite");
    }
}

}  // namespace

NavState Mechanization::propagate(
    const NavState& previous,
    const ImuIncrement& previous_increment,
    const ImuIncrement& increment,
    const EarthModelOptions& options,
    MechanizationDiagnostics* diagnostics) {
    validateState(previous);
    validateIncrement(previous_increment);
    validateIncrement(increment);
    if (std::abs(previous_increment.dt_s - increment.dt_s) >
        kEqualIntervalToleranceS) {
        throw std::invalid_argument(
            "adjacent-sample coning/sculling requires equal IMU intervals");
    }

    const double previous_attitude_norm = previous.q_bn.norm();
    if (previous_attitude_norm < 1e-12) {
        throw std::invalid_argument("previous attitude is invalid");
    }
    const Eigen::Quaterniond q_bn_previous =
        previous.q_bn.normalized();

    const EarthParameters earth_start =
        EarthModel::evaluate(previous.position_rad_m, previous.velocity_ned_mps, options);

    const Eigen::Vector3d d_theta_body =
        increment.delta_angle_rad;
    const Eigen::Vector3d d_v_body =
        increment.delta_velocity_mps;
    const Eigen::Vector3d d_v_fb =
        d_v_body +
        0.5 * d_theta_body.cross(d_v_body) +
        (1.0 / 12.0) *
            (previous_increment.delta_angle_rad.cross(d_v_body) +
             previous_increment.delta_velocity_mps.cross(d_theta_body));

    const Eigen::Vector3d nav_rate_start =
        earth_start.omega_ie_ned_radps + earth_start.omega_en_ned_radps;
    const Eigen::Matrix3d c_nn_start =
        Eigen::Matrix3d::Identity() - skew(nav_rate_start * increment.dt_s * 0.5);
    const Eigen::Vector3d d_v_fn_start =
        c_nn_start * q_bn_previous.toRotationMatrix() * d_v_fb;
    const Eigen::Vector3d d_v_gn_start =
        gravityCoriolisIncrement(
            earth_start, previous.velocity_ned_mps, options, increment.dt_s);
    const Eigen::Vector3d mid_velocity =
        previous.velocity_ned_mps + 0.5 * (d_v_fn_start + d_v_gn_start);
    const Eigen::Vector3d mid_position =
        previous.position_rad_m +
        positionRate(
            previous.position_rad_m,
            earth_start.meridian_prime_vertical_radii_m,
            mid_velocity) *
            (increment.dt_s * 0.5);

    const EarthParameters earth_mid =
        EarthModel::evaluate(mid_position, mid_velocity, options);
    const Eigen::Vector3d nav_rate_mid =
        earth_mid.omega_ie_ned_radps + earth_mid.omega_en_ned_radps;
    const Eigen::Matrix3d c_nn_mid =
        Eigen::Matrix3d::Identity() - skew(nav_rate_mid * increment.dt_s * 0.5);
    const Eigen::Vector3d d_v_fn =
        c_nn_mid * q_bn_previous.toRotationMatrix() * d_v_fb;
    const Eigen::Vector3d d_v_gn =
        gravityCoriolisIncrement(
            earth_mid, mid_velocity, options, increment.dt_s);

    NavState current = previous;
    current.time_s += increment.dt_s;
    current.velocity_ned_mps =
        previous.velocity_ned_mps + d_v_fn + d_v_gn;

    const Eigen::Vector3d position_velocity =
        0.5 * (previous.velocity_ned_mps + current.velocity_ned_mps);
    const Eigen::Vector3d position_midpoint =
        previous.position_rad_m +
        positionRate(
            previous.position_rad_m,
            earth_start.meridian_prime_vertical_radii_m,
            position_velocity) *
            (increment.dt_s * 0.5);
    const EarthParameters earth_position =
        EarthModel::evaluate(position_midpoint, position_velocity, options);
    current.position_rad_m =
        previous.position_rad_m +
        positionRate(
            position_midpoint,
            earth_position.meridian_prime_vertical_radii_m,
            position_velocity) *
            increment.dt_s;

    // The attitude update follows the same body/navigation frame sequence as
    // the full mechanization: body coning, navigation transport, Earth rate,
    // and the previous body-to-navigation attitude.
    const Eigen::Quaterniond q_ne_previous =
        navToEcefQuaternion(previous.position_rad_m);
    const Eigen::Quaterniond q_ne_current =
        navToEcefQuaternion(current.position_rad_m);
    const double earth_rate_magnitude =
        options.enable_earth_rotation ? options.earth_rotation_rate_radps : 0.0;
    const Eigen::Quaterniond q_ee_full = expMap(
        Eigen::Vector3d(0.0, 0.0, -earth_rate_magnitude * increment.dt_s));
    const Eigen::Quaterniond q_ne_midpoint = [&]() {
        if (!options.enable_earth_rotation) {
            return navToEcefQuaternion(
                0.5 * (previous.position_rad_m + current.position_rad_m));
        }
        const Eigen::Vector3d q_ne_delta = quaternionVector(
            q_ne_previous.inverse() * q_ee_full.inverse() * q_ne_current);
        return (expMap(Eigen::Vector3d(
                    0.0, 0.0,
                    -earth_rate_magnitude * increment.dt_s * 0.5)) *
                    q_ne_previous *
                    expMap(q_ne_delta * 0.5))
            .normalized();
    }();

    const Eigen::Vector3d attitude_midpoint =
        quaternionToGeodetic(
            q_ne_midpoint,
            0.5 * (previous.position_rad_m.z() + current.position_rad_m.z()));
    const Eigen::Vector3d attitude_velocity =
        0.5 * (previous.velocity_ned_mps + current.velocity_ned_mps);
    const EarthParameters earth_attitude =
        EarthModel::evaluate(attitude_midpoint, attitude_velocity, options);
    const Eigen::Vector3d q_nn_vector =
        -(earth_attitude.omega_ie_ned_radps +
          earth_attitude.omega_en_ned_radps) *
        increment.dt_s;
    const Eigen::Vector3d q_bb_vector =
        d_theta_body +
        (1.0 / 12.0) *
            previous_increment.delta_angle_rad.cross(d_theta_body);
    const Eigen::Quaterniond q_nn = expMap(q_nn_vector);
    const Eigen::Quaterniond q_bb = expMap(q_bb_vector);
    current.q_bn =
        (q_nn * q_bn_previous * q_bb).normalized();

    if (!current.position_rad_m.allFinite() ||
        !current.velocity_ned_mps.allFinite() ||
        !current.q_bn.coeffs().allFinite()) {
        throw std::runtime_error("mechanization produced a non-finite state");
    }

    if (diagnostics != nullptr) {
        diagnostics->earth_at_start = earth_start;
        diagnostics->earth_at_midpoint = earth_mid;
        diagnostics->specific_force_increment_nav_mps = d_v_fn;
        diagnostics->gravity_coriolis_increment_mps = d_v_gn;
    }
    return current;
}

NavState Mechanization::propagate(
    const NavState& previous,
    const ImuIncrement& increment,
    const EarthModelOptions& options,
    MechanizationDiagnostics* diagnostics) {
    ImuIncrement zero_previous;
    zero_previous.dt_s = increment.dt_s;
    return propagate(previous, zero_previous, increment, options, diagnostics);
}

MechanizationRunner::MechanizationRunner(
    NavState initial_state,
    EarthModelOptions options)
    : state_(std::move(initial_state)),
      options_(options) {
    validateState(state_);
}

NavState MechanizationRunner::step(
    const ImuIncrement& increment,
    MechanizationDiagnostics* diagnostics) {
    if (!has_previous_increment_) {
        ImuIncrement zero_previous;
        zero_previous.dt_s = increment.dt_s;
        state_ = Mechanization::propagate(
            state_, zero_previous, increment, options_, diagnostics);
        previous_increment_ = increment;
        has_previous_increment_ = true;
        return state_;
    }

    state_ = Mechanization::propagate(
        state_, previous_increment_, increment, options_, diagnostics);
    previous_increment_ = increment;
    return state_;
}

void MechanizationRunner::reset(NavState initial_state) {
    validateState(initial_state);
    state_ = std::move(initial_state);
    previous_increment_ = ImuIncrement{};
    has_previous_increment_ = false;
}

}  // namespace inertial_core

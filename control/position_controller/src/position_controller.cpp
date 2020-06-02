#include "position_controller/position_controller.h"

#include <quadrotor_common/math_common.h>

namespace position_controller {

PositionController::PositionController() {}

PositionController::~PositionController() {}

quadrotor_common::ControlCommand PositionController::off() {
  quadrotor_common::ControlCommand command;

  command.zero();

  return command;
}

quadrotor_common::ControlCommand PositionController::run(
    const quadrotor_common::QuadStateEstimate& state_estimate,
    const quadrotor_common::Trajectory& reference_trajectory,
    const PositionControllerParams& config) {
  quadrotor_common::ControlCommand command;
  command.armed = true;

  quadrotor_common::TrajectoryPoint reference_state(
      reference_trajectory.points.front());

  // Compute reference inputs
  Eigen::Vector3d drag_accelerations = Eigen::Vector3d::Zero();
  quadrotor_common::ControlCommand reference_inputs;
  if (config.perform_aerodynamics_compensation) {
    // Compute reference inputs that compensate for aerodynamic drag
    computeAeroCompensatedReferenceInputs(reference_state, state_estimate,
                                          config, &reference_inputs,
                                          &drag_accelerations);
  } else {
    // In this case we are not considering aerodynamic accelerations
    drag_accelerations = Eigen::Vector3d::Zero();

    // Compute reference inputs as feed forward terms
    reference_inputs = computeNominalReferenceInputs(
        reference_state, state_estimate.orientation);
  }

  // Compute desired control commands
  const Eigen::Vector3d pid_error_accelerations =
      computePIDErrorAcc(state_estimate, reference_state, config);
  const Eigen::Vector3d desired_acceleration = pid_error_accelerations +
                                               reference_state.acceleration -
                                               kGravity_ - drag_accelerations;

  command.collective_thrust = computeDesiredCollectiveMassNormalizedThrust(
      state_estimate.orientation, desired_acceleration, config);
  if (config.perform_aerodynamics_compensation) {
    // This compensates for an acceleration component in thrust direction due
    // to the square of the body-horizontal velocity.
    command.collective_thrust -=
        config.k_thrust_horz * (pow(state_estimate.velocity.x(), 2.0) +
                                pow(state_estimate.velocity.y(), 2.0));
  }

  const Eigen::Quaterniond desired_attitude =
      computeDesiredAttitude(desired_acceleration, reference_state.heading,
                             state_estimate.orientation);
  const Eigen::Vector3d feedback_bodyrates = computeFeedBackControlBodyrates(
      desired_attitude, state_estimate.orientation, config);

  if (config.use_rate_mode) {
    command.control_mode = quadrotor_common::ControlMode::BODY_RATES;
    command.bodyrates = reference_inputs.bodyrates + feedback_bodyrates;
  } else {
    command.control_mode = quadrotor_common::ControlMode::ATTITUDE;
    command.orientation = desired_attitude;

    // In ATTITUDE control mode the x-y-bodyrates contain just the feed forward
    // terms. The z-bodyrate has to be from feedback control
    command.bodyrates = reference_inputs.bodyrates;
    command.bodyrates.z() += feedback_bodyrates.z();
  }

  command.angular_accelerations = reference_inputs.angular_accelerations;

  return command;
}

quadrotor_common::ControlCommand
PositionController::computeNominalReferenceInputs(
    const quadrotor_common::TrajectoryPoint& reference_state,
    const Eigen::Quaterniond& attitude_estimate) const {
  quadrotor_common::ControlCommand reference_command;

  const Eigen::Quaterniond q_heading = Eigen::Quaterniond(
      Eigen::AngleAxisd(reference_state.heading, Eigen::Vector3d::UnitZ()));

  // Rotate world frame by the reference heading --> C-coordinate system
  const Eigen::Vector3d x_C = q_heading * Eigen::Vector3d::UnitX();
  const Eigen::Vector3d y_C = q_heading * Eigen::Vector3d::UnitY();

  const Eigen::Vector3d des_acc = reference_state.acceleration - kGravity_;

  // Reference attitude
  const Eigen::Quaterniond q_W_B = computeDesiredAttitude(
      des_acc, reference_state.heading, attitude_estimate);

  const Eigen::Vector3d x_B = q_W_B * Eigen::Vector3d::UnitX();
  const Eigen::Vector3d y_B = q_W_B * Eigen::Vector3d::UnitY();
  const Eigen::Vector3d z_B = q_W_B * Eigen::Vector3d::UnitZ();

  reference_command.orientation = q_W_B;

  // Reference thrust
  reference_command.collective_thrust = des_acc.norm();

  // Reference body rates
  if (almostZeroThrust(reference_command.collective_thrust)) {
    reference_command.bodyrates.x() = 0.0;
    reference_command.bodyrates.y() = 0.0;
  } else {
    reference_command.bodyrates.x() = -1.0 /
                                      reference_command.collective_thrust *
                                      y_B.dot(reference_state.jerk);
    reference_command.bodyrates.y() = 1.0 /
                                      reference_command.collective_thrust *
                                      x_B.dot(reference_state.jerk);
  }

  if (almostZero((y_C.cross(z_B)).norm())) {
    reference_command.bodyrates.z() = 0.0;
  } else {
    reference_command.bodyrates.z() =
        1.0 / (y_C.cross(z_B)).norm() *
        (reference_state.heading_rate * x_C.dot(x_B) +
         reference_command.bodyrates.y() * y_C.dot(z_B));
  }

  // Reference angular accelerations
  if (almostZeroThrust(reference_command.collective_thrust)) {
    reference_command.angular_accelerations.x() = 0.0;
    reference_command.angular_accelerations.y() = 0.0;
  } else {
    const double thrust_dot = z_B.dot(reference_state.jerk);
    reference_command.angular_accelerations.x() =
        -1.0 / reference_command.collective_thrust *
        (y_B.dot(reference_state.snap) +
         2.0 * thrust_dot * reference_command.bodyrates.x() -
         reference_command.collective_thrust * reference_command.bodyrates.y() *
             reference_command.bodyrates.z());
    reference_command.angular_accelerations.y() =
        1.0 / reference_command.collective_thrust *
        (x_B.dot(reference_state.snap) -
         2.0 * thrust_dot * reference_command.bodyrates.y() -
         reference_command.collective_thrust * reference_command.bodyrates.x() *
             reference_command.bodyrates.z());
  }

  if (almostZero((y_C.cross(z_B)).norm())) {
    reference_command.angular_accelerations.z() = 0.0;
  } else {
    reference_command.angular_accelerations.z() =
        1.0 / (y_C.cross(z_B)).norm() *
        (reference_state.heading_acceleration * x_C.dot(x_B) +
         2.0 * reference_state.heading_rate * reference_command.bodyrates.z() *
             x_C.dot(y_B) -
         2.0 * reference_state.heading_rate * reference_command.bodyrates.y() *
             x_C.dot(z_B) -
         reference_command.bodyrates.x() * reference_command.bodyrates.y() *
             y_C.dot(y_B) -
         reference_command.bodyrates.x() * reference_command.bodyrates.z() *
             y_C.dot(z_B) +
         reference_command.angular_accelerations.y() * y_C.dot(z_B));
  }

  return reference_command;
}

void PositionController::computeAeroCompensatedReferenceInputs(
    const quadrotor_common::TrajectoryPoint& reference_state,
    const quadrotor_common::QuadStateEstimate& state_estimate,
    const PositionControllerParams& config,
    quadrotor_common::ControlCommand* reference_inputs,
    Eigen::Vector3d* drag_accelerations) const {
  quadrotor_common::ControlCommand reference_command;

  const Eigen::Vector3d velocity_in_body =
      state_estimate.orientation.inverse() * reference_state.velocity;

  const double dx = config.k_drag_x;
  const double dy = config.k_drag_y;
  const double dz = config.k_drag_z;

  const Eigen::Quaterniond q_heading = Eigen::Quaterniond(
      Eigen::AngleAxisd(reference_state.heading, Eigen::Vector3d::UnitZ()));

  const Eigen::Vector3d x_C = q_heading * Eigen::Vector3d::UnitX();
  const Eigen::Vector3d y_C = q_heading * Eigen::Vector3d::UnitY();

  const Eigen::Vector3d alpha =
      reference_state.acceleration - kGravity_ + dx * reference_state.velocity;
  const Eigen::Vector3d beta =
      reference_state.acceleration - kGravity_ + dy * reference_state.velocity;
  const Eigen::Vector3d gamma =
      reference_state.acceleration - kGravity_ + dz * reference_state.velocity;

  // Reference attitude
  const Eigen::Vector3d x_B_prototype = y_C.cross(alpha);
  const Eigen::Vector3d x_B = computeRobustBodyXAxis(
      x_B_prototype, x_C, y_C, state_estimate.orientation);

  Eigen::Vector3d y_B = beta.cross(x_B);
  if (almostZero(y_B.norm())) {
    const Eigen::Vector3d z_B_estimated =
        state_estimate.orientation * Eigen::Vector3d::UnitZ();
    y_B = z_B_estimated.cross(x_B);
    if (almostZero(y_B.norm())) {
      y_B = y_C;
    } else {
      y_B.normalize();
    }
  } else {
    y_B.normalize();
  }

  const Eigen::Vector3d z_B = x_B.cross(y_B);

  const Eigen::Matrix3d R_W_B_ref(
      (Eigen::Matrix3d() << x_B, y_B, z_B).finished());

  reference_command.orientation = Eigen::Quaterniond(R_W_B_ref);

  // Reference thrust
  reference_command.collective_thrust = z_B.dot(gamma);

  // Rotor drag matrix
  const Eigen::Matrix3d D = Eigen::Vector3d(dx, dy, dz).asDiagonal();

  // Reference body rates and angular accelerations
  const double B1 = reference_command.collective_thrust -
                    (dz - dx) * z_B.dot(reference_state.velocity);
  const double C1 = -(dx - dy) * y_B.dot(reference_state.velocity);
  const double D1 = x_B.dot(reference_state.jerk) +
                    dx * x_B.dot(reference_state.acceleration);
  const double A2 = reference_command.collective_thrust +
                    (dy - dz) * z_B.dot(reference_state.velocity);
  const double C2 = (dx - dy) * x_B.dot(reference_state.velocity);
  const double D2 = -y_B.dot(reference_state.jerk) -
                    dy * y_B.dot(reference_state.acceleration);
  const double B3 = -y_C.dot(z_B);
  const double C3 = (y_C.cross(z_B)).norm();
  const double D3 = reference_state.heading_rate * x_C.dot(x_B);

  const double denominator = B1 * C3 - B3 * C1;

  if (almostZero(denominator)) {
    reference_command.bodyrates = Eigen::Vector3d::Zero();
    reference_command.angular_accelerations = Eigen::Vector3d::Zero();
  } else {
    // Compute body rates
    if (almostZero(A2)) {
      reference_command.bodyrates.x() = 0.0;
    } else {
      reference_command.bodyrates.x() =
          (-B1 * C2 * D3 + B1 * C3 * D2 - B3 * C1 * D2 + B3 * C2 * D1) /
          (A2 * denominator);
    }
    reference_command.bodyrates.y() = (-C1 * D3 + C3 * D1) / denominator;
    reference_command.bodyrates.z() = (B1 * D3 - B3 * D1) / denominator;

    // Compute angular accelerations
    const double thrust_dot = z_B.dot(reference_state.jerk) +
                              reference_command.bodyrates.x() * (dy - dz) *
                                  y_B.dot(reference_state.velocity) +
                              reference_command.bodyrates.y() * (dz - dx) *
                                  x_B.dot(reference_state.velocity) +
                              dz * z_B.dot(reference_state.acceleration);

    const Eigen::Matrix3d omega_hat =
        quadrotor_common::skew(reference_command.bodyrates);
    const Eigen::Vector3d xi =
        R_W_B_ref *
            (omega_hat * omega_hat * D + D * omega_hat * omega_hat +
             2.0 * omega_hat * D * omega_hat.transpose()) *
            R_W_B_ref.transpose() * reference_state.velocity +
        2.0 * R_W_B_ref * (omega_hat * D + D * omega_hat.transpose()) *
            R_W_B_ref.transpose() * reference_state.acceleration +
        R_W_B_ref * D * R_W_B_ref.transpose() * reference_state.jerk;

    const double E1 = x_B.dot(reference_state.snap) -
                      2.0 * thrust_dot * reference_command.bodyrates.y() -
                      reference_command.collective_thrust *
                          reference_command.bodyrates.x() *
                          reference_command.bodyrates.z() +
                      x_B.dot(xi);
    const double E2 = -y_B.dot(reference_state.snap) -
                      2.0 * thrust_dot * reference_command.bodyrates.x() +
                      reference_command.collective_thrust *
                          reference_command.bodyrates.y() *
                          reference_command.bodyrates.z() -
                      y_B.dot(xi);
    const double E3 = reference_state.heading_acceleration * x_C.dot(x_B) +
                      2.0 * reference_state.heading_rate *
                          (reference_command.bodyrates.z() * x_C.dot(y_B) -
                           reference_command.bodyrates.y() * x_C.dot(z_B)) -
                      reference_command.bodyrates.x() *
                          (reference_command.bodyrates.y() * y_C.dot(y_B) +
                           reference_command.bodyrates.z() * y_C.dot(z_B));

    if (almostZero(A2)) {
      reference_command.angular_accelerations.x() = 0.0;
    } else {
      reference_command.angular_accelerations.x() =
          (-B1 * C2 * E3 + B1 * C3 * E2 - B3 * C1 * E2 + B3 * C2 * E1) /
          (A2 * denominator);
    }
    reference_command.angular_accelerations.y() =
        (-C1 * E3 + C3 * E1) / denominator;
    reference_command.angular_accelerations.z() =
        (B1 * E3 - B3 * E1) / denominator;
  }

  // Transform reference rates and derivatives into estimated body frame
  const Eigen::Matrix3d R_trans =
      state_estimate.orientation.toRotationMatrix().transpose() * R_W_B_ref;
  const Eigen::Vector3d bodyrates_ref = reference_command.bodyrates;
  const Eigen::Vector3d angular_accelerations_ref =
      reference_command.angular_accelerations;
  const Eigen::Matrix3d bodyrates_est_hat =
      quadrotor_common::skew(state_estimate.bodyrates);

  reference_command.bodyrates = R_trans * bodyrates_ref;
  reference_command.angular_accelerations =
      R_trans * angular_accelerations_ref -
      bodyrates_est_hat * R_trans * bodyrates_ref;

  *reference_inputs = reference_command;

  // Drag accelerations
  *drag_accelerations =
      -1.0 *
      (R_W_B_ref * (D * (R_W_B_ref.transpose() * reference_state.velocity)));
}

Eigen::Vector3d PositionController::computePIDErrorAcc(
    const quadrotor_common::QuadStateEstimate& state_estimate,
    const quadrotor_common::TrajectoryPoint& reference_state,
    const PositionControllerParams& config) const {
  // Compute the desired accelerations due to control errors in world frame
  // with a PID controller
  Eigen::Vector3d acc_error;

  // x acceleration
  double x_pos_error =
      reference_state.position.x() - state_estimate.position.x();
  quadrotor_common::limit(&x_pos_error, -config.pxy_error_max,
                          config.pxy_error_max);

  double x_vel_error =
      reference_state.velocity.x() - state_estimate.velocity.x();
  quadrotor_common::limit(&x_vel_error, -config.vxy_error_max,
                          config.vxy_error_max);

  acc_error.x() = config.kpxy * x_pos_error + config.kdxy * x_vel_error;

  // y acceleration
  double y_pos_error =
      reference_state.position.y() - state_estimate.position.y();
  quadrotor_common::limit(&y_pos_error, -config.pxy_error_max,
                          config.pxy_error_max);

  double y_vel_error =
      reference_state.velocity.y() - state_estimate.velocity.y();
  quadrotor_common::limit(&y_vel_error, -config.vxy_error_max,
                          config.vxy_error_max);

  acc_error.y() = config.kpxy * y_pos_error + config.kdxy * y_vel_error;

  // z acceleration
  double z_pos_error =
      reference_state.position.z() - state_estimate.position.z();
  quadrotor_common::limit(&z_pos_error, -config.pz_error_max,
                          config.pz_error_max);

  double z_vel_error =
      reference_state.velocity.z() - state_estimate.velocity.z();
  quadrotor_common::limit(&z_vel_error, -config.vz_error_max,
                          config.vz_error_max);

  acc_error.z() = config.kpz * z_pos_error + config.kdz * z_vel_error;

  return acc_error;
}

double PositionController::computeDesiredCollectiveMassNormalizedThrust(
    const Eigen::Quaterniond& attitude_estimate,
    const Eigen::Vector3d& desired_acc,
    const PositionControllerParams& config) const {
  const Eigen::Vector3d body_z_axis =
      attitude_estimate * Eigen::Vector3d::UnitZ();

  double normalized_thrust = desired_acc.dot(body_z_axis);
  if (normalized_thrust < kMinNormalizedCollectiveThrust_) {
    normalized_thrust = kMinNormalizedCollectiveThrust_;
  }
  return normalized_thrust;
}

Eigen::Quaterniond PositionController::computeDesiredAttitude(
    const Eigen::Vector3d& desired_acceleration, const double reference_heading,
    const Eigen::Quaterniond& attitude_estimate) const {
  const Eigen::Quaterniond q_heading = Eigen::Quaterniond(
      Eigen::AngleAxisd(reference_heading, Eigen::Vector3d::UnitZ()));

  // Compute desired orientation
  const Eigen::Vector3d x_C = q_heading * Eigen::Vector3d::UnitX();
  const Eigen::Vector3d y_C = q_heading * Eigen::Vector3d::UnitY();

  Eigen::Vector3d z_B;
  if (almostZero(desired_acceleration.norm())) {
    // In case of free fall we keep the thrust direction to be the estimated one
    // This only works assuming that we are in this condition for a very short
    // time (otherwise attitude drifts)
    z_B = attitude_estimate * Eigen::Vector3d::UnitZ();
  } else {
    z_B = desired_acceleration.normalized();
  }

  const Eigen::Vector3d x_B_prototype = y_C.cross(z_B);
  const Eigen::Vector3d x_B =
      computeRobustBodyXAxis(x_B_prototype, x_C, y_C, attitude_estimate);

  const Eigen::Vector3d y_B = (z_B.cross(x_B)).normalized();

  // From the computed desired body axes we can now compose a desired attitude
  const Eigen::Matrix3d R_W_B((Eigen::Matrix3d() << x_B, y_B, z_B).finished());

  const Eigen::Quaterniond desired_attitude(R_W_B);

  return desired_attitude;
}

Eigen::Vector3d PositionController::computeRobustBodyXAxis(
    const Eigen::Vector3d& x_B_prototype, const Eigen::Vector3d& x_C,
    const Eigen::Vector3d& y_C,
    const Eigen::Quaterniond& attitude_estimate) const {
  Eigen::Vector3d x_B = x_B_prototype;

  if (almostZero(x_B.norm())) {
    // if cross(y_C, z_B) == 0, they are collinear =>
    // every x_B lies automatically in the x_C - z_C plane

    // Project estimated body x-axis into the x_C - z_C plane
    const Eigen::Vector3d x_B_estimated =
        attitude_estimate * Eigen::Vector3d::UnitX();
    const Eigen::Vector3d x_B_projected =
        x_B_estimated - (x_B_estimated.dot(y_C)) * y_C;
    if (almostZero(x_B_projected.norm())) {
      // Not too much intelligent stuff we can do in this case but it should
      // basically never occur
      x_B = x_C;
    } else {
      x_B = x_B_projected.normalized();
    }
  } else {
    x_B.normalize();
  }

  // if the quad is upside down, x_B will point in the "opposite" direction
  // of x_C => flip x_B (unfortunately also not the solution for our problems)
  //  if (x_B.dot(x_C) < 0.0)
  //  {
  //    x_B = -x_B;
  //  }

  return x_B;
}

Eigen::Vector3d PositionController::computeFeedBackControlBodyrates(
    const Eigen::Quaterniond& desired_attitude,
    const Eigen::Quaterniond& attitude_estimate,
    const PositionControllerParams& config) const {
  // Compute the error quaternion
  const Eigen::Quaterniond q_e = attitude_estimate.inverse() * desired_attitude;

  // Compute desired body rates from control error
  Eigen::Vector3d bodyrates;

  if (q_e.w() >= 0) {
    bodyrates.x() = 2.0 * config.krp * q_e.x();
    bodyrates.y() = 2.0 * config.krp * q_e.y();
    bodyrates.z() = 2.0 * config.kyaw * q_e.z();
  } else {
    bodyrates.x() = -2.0 * config.krp * q_e.x();
    bodyrates.y() = -2.0 * config.krp * q_e.y();
    bodyrates.z() = -2.0 * config.kyaw * q_e.z();
  }

  return bodyrates;
}

bool PositionController::almostZero(const double value) const {
  return fabs(value) < kAlmostZeroValueThreshold_;
}

bool PositionController::almostZeroThrust(const double thrust_value) const {
  return fabs(thrust_value) < kAlmostZeroThrustThreshold_;
}

}  // namespace position_controller

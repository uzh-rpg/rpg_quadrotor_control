#pragma once

#include <quadrotor_common/parameter_helper.h>

namespace position_controller {

class PositionControllerParams {
 public:
  PositionControllerParams()
      : use_rate_mode(true),
        kpxy(0.0),
        kdxy(0.0),
        kpz(0.0),
        kdz(0.0),
        krp(0.0),
        kyaw(0.0),
        pxy_error_max(0.0),
        vxy_error_max(0.0),
        pz_error_max(0.0),
        vz_error_max(0.0),
        yaw_error_max(0.0),
        perform_aerodynamics_compensation(false),
        k_drag_x(0.0),
        k_drag_y(0.0),
        k_drag_z(0.0),
        k_thrust_horz(0.0) {}

  ~PositionControllerParams() {}

  bool loadParameters(const ros::NodeHandle& pnh) {
    const std::string path_rel_to_node = "position_controller";

    if (!quadrotor_common::getParam(path_rel_to_node + "/use_rate_mode",
                                    use_rate_mode, pnh)) {
      return false;
    }

    if (!quadrotor_common::getParam(path_rel_to_node + "/kpxy", kpxy, pnh)) {
      return false;
    }
    if (!quadrotor_common::getParam(path_rel_to_node + "/kdxy", kdxy, pnh)) {
      return false;
    }

    if (!quadrotor_common::getParam(path_rel_to_node + "/kpz", kpz, pnh)) {
      return false;
    }
    if (!quadrotor_common::getParam(path_rel_to_node + "/kdz", kdz, pnh)) {
      return false;
    }

    if (!quadrotor_common::getParam(path_rel_to_node + "/krp", krp, pnh)) {
      return false;
    }
    if (!quadrotor_common::getParam(path_rel_to_node + "/kyaw", kyaw, pnh)) {
      return false;
    }

    if (!quadrotor_common::getParam(path_rel_to_node + "/pxy_error_max",
                                    pxy_error_max, pnh)) {
      return false;
    }
    if (!quadrotor_common::getParam(path_rel_to_node + "/vxy_error_max",
                                    vxy_error_max, pnh)) {
      return false;
    }
    if (!quadrotor_common::getParam(path_rel_to_node + "/pz_error_max",
                                    pz_error_max, pnh)) {
      return false;
    }
    if (!quadrotor_common::getParam(path_rel_to_node + "/vz_error_max",
                                    vz_error_max, pnh)) {
      return false;
    }
    if (!quadrotor_common::getParam(path_rel_to_node + "/yaw_error_max",
                                    yaw_error_max, pnh)) {
      return false;
    }

    if (!quadrotor_common::getParam(
            path_rel_to_node + "/perform_aerodynamics_compensation",
            perform_aerodynamics_compensation, pnh)) {
      return false;
    }
    if (!quadrotor_common::getParam(path_rel_to_node + "/k_drag_x", k_drag_x,
                                    pnh)) {
      return false;
    }
    if (!quadrotor_common::getParam(path_rel_to_node + "/k_drag_y", k_drag_y,
                                    pnh)) {
      return false;
    }
    if (!quadrotor_common::getParam(path_rel_to_node + "/k_drag_z", k_drag_z,
                                    pnh)) {
      return false;
    }
    if (!quadrotor_common::getParam(path_rel_to_node + "/k_thrust_horz",
                                    k_thrust_horz, pnh)) {
      return false;
    }

    return true;
  }

  // Send bodyrate commands if true, attitude commands otherwise
  bool use_rate_mode;

  double kpxy;  // [1/s^2]
  double kdxy;  // [1/s]

  double kpz;  // [1/s^2]
  double kdz;  // [1/s]

  double krp;   // [1/s]
  double kyaw;  // [1/s]

  double pxy_error_max;  // [m]
  double vxy_error_max;  // [m/s]
  double pz_error_max;   // [m]
  double vz_error_max;   // [m/s]
  double yaw_error_max;  // [rad]

  // Whether or not to compensate for aerodynamic effects
  bool perform_aerodynamics_compensation;
  double k_drag_x;  // x-direction rotor drag coefficient
  double k_drag_y;  // y-direction rotor drag coefficient
  double k_drag_z;  // z-direction rotor drag coefficient
  // thrust correction coefficient due to body horizontal velocity
  double k_thrust_horz;
};

}  // namespace position_controller

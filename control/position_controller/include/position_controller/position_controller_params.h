#pragma once

#include <quadrotor_common/parameter_helper.h>

namespace position_controller
{

class PositionControllerParams
{
public:
  PositionControllerParams() :
      use_rate_mode(true), kpxy(0.0), kdxy(0.0), kpz(0.0), kdz(0.0), krp(0.0),
          kyaw(0.0), pxy_error_max(0.0), vxy_error_max(0.0), pz_error_max(0.0),
          vz_error_max(0.0), yaw_error_max(0.0),
          perform_aerodynamics_compensation(false), k_drag_x(0.0),
          k_drag_y(0.0), k_drag_z(0.0), k_thrust_horz(0.0), perform_compensation(false),
          compensation_coeff_xx(0.0), compensation_coeff_xy(0.0), compensation_coeff_xz(0.0),
          compensation_coeff_yx(0.0), compensation_coeff_yy(0.0), compensation_coeff_yz(0.0),
          compensation_coeff_zx(0.0), compensation_coeff_zy(0.0), compensation_coeff_zz(0.0) 
  {
  }

  ~PositionControllerParams()
  {
  }

  bool loadParameters(const ros::NodeHandle & pnh)
  {
    const std::string path_rel_to_node = "position_controller";

    if (!quadrotor_common::getParam(path_rel_to_node + "/use_rate_mode",
                                    use_rate_mode, pnh))
    {
      return false;
    }

    if (!quadrotor_common::getParam(path_rel_to_node + "/kpxy", kpxy, pnh))
    {
      return false;
    }
    if (!quadrotor_common::getParam(path_rel_to_node + "/kdxy", kdxy, pnh))
    {
      return false;
    }

    if (!quadrotor_common::getParam(path_rel_to_node + "/kpz", kpz, pnh))
    {
      return false;
    }
    if (!quadrotor_common::getParam(path_rel_to_node + "/kdz", kdz, pnh))
    {
      return false;
    }

    if (!quadrotor_common::getParam(path_rel_to_node + "/krp", krp, pnh))
    {
      return false;
    }
    if (!quadrotor_common::getParam(path_rel_to_node + "/kyaw", kyaw, pnh))
    {
      return false;
    }

    if (!quadrotor_common::getParam(path_rel_to_node + "/pxy_error_max",
                                    pxy_error_max, pnh))
    {
      return false;
    }
    if (!quadrotor_common::getParam(path_rel_to_node + "/vxy_error_max",
                                    vxy_error_max, pnh))
    {
      return false;
    }
    if (!quadrotor_common::getParam(path_rel_to_node + "/pz_error_max",
                                    pz_error_max, pnh))
    {
      return false;
    }
    if (!quadrotor_common::getParam(path_rel_to_node + "/vz_error_max",
                                    vz_error_max, pnh))
    {
      return false;
    }
    if (!quadrotor_common::getParam(path_rel_to_node + "/yaw_error_max",
                                    yaw_error_max, pnh))
    {
      return false;
    }

    if (!quadrotor_common::getParam(
        path_rel_to_node + "/perform_aerodynamics_compensation",
        perform_aerodynamics_compensation, pnh))
    {
      return false;
    }
    if (!quadrotor_common::getParam(path_rel_to_node + "/k_drag_x", k_drag_x,
                                    pnh))
    {
      return false;
    }
    if (!quadrotor_common::getParam(path_rel_to_node + "/k_drag_y", k_drag_y,
                                    pnh))
    {
      return false;
    }
    if (!quadrotor_common::getParam(path_rel_to_node + "/k_drag_z", k_drag_z,
                                    pnh))
    {
      return false;
    }
    if (!quadrotor_common::getParam(path_rel_to_node + "/k_thrust_horz",
                                    k_thrust_horz, pnh))
    {
      return false;
    }

    if (!quadrotor_common::getParam(path_rel_to_node + "/perform_compensation",
                                    perform_compensation, pnh))
    {
      return false;
    }

    if (!quadrotor_common::getParam(path_rel_to_node + "/compensation_coeff_xx",
                                    compensation_coeff_xx, pnh))
    {
      return false;
    }

    if (!quadrotor_common::getParam(path_rel_to_node + "/compensation_coeff_xy",
                                    compensation_coeff_xy, pnh))
    {
      return false;
    }

    if (!quadrotor_common::getParam(path_rel_to_node + "/compensation_coeff_xz",
                                    compensation_coeff_xz, pnh))
    {
      return false;
    }
    if (!quadrotor_common::getParam(path_rel_to_node + "/compensation_coeff_yx",
                                    compensation_coeff_yx, pnh))
    {
      return false;
    }
    if (!quadrotor_common::getParam(path_rel_to_node + "/compensation_coeff_yy",
                                    compensation_coeff_yy, pnh))
    {
      return false;
    }
    if (!quadrotor_common::getParam(path_rel_to_node + "/compensation_coeff_yz",
                                    compensation_coeff_yz, pnh))
    {
      return false;
    }
    if (!quadrotor_common::getParam(path_rel_to_node + "/compensation_coeff_zx",
                                    compensation_coeff_zx, pnh))
    {
      return false;
    }
    if (!quadrotor_common::getParam(path_rel_to_node + "/compensation_coeff_zy",
                                    compensation_coeff_zy, pnh))
    {
      return false;
    }
    if (!quadrotor_common::getParam(path_rel_to_node + "/compensation_coeff_zz",
                                    compensation_coeff_zz, pnh))
    {
      return false;
    }

    return true;
  }

  // Send bodyrate commands if true, attitude commands otherwise
  bool use_rate_mode;

  double kpxy; // [1/s^2]
  double kdxy; // [1/s]

  double kpz; // [1/s^2]
  double kdz; // [1/s]

  double krp; // [1/s]
  double kyaw; // [1/s]

  double pxy_error_max; // [m]
  double vxy_error_max; // [m/s]
  double pz_error_max; // [m]
  double vz_error_max; // [m/s]
  double yaw_error_max; // [rad]

  // Whether or not to compensate for aerodynamic effects
  bool perform_aerodynamics_compensation;
  double k_drag_x; // x-direction rotor drag coefficient
  double k_drag_y; // y-direction rotor drag coefficient
  double k_drag_z; // z-direction rotor drag coefficient
  // thrust correction coefficient due to body horizontal velocity
  double k_thrust_horz;

  bool perform_compensation;
  double compensation_coeff_xx;
  double compensation_coeff_xy;
  double compensation_coeff_xz;
  double compensation_coeff_yx;
  double compensation_coeff_yy;
  double compensation_coeff_yz;
  double compensation_coeff_zx; 
  double compensation_coeff_zy; 
  double compensation_coeff_zz; 

  double mlp_compensation_x_{0.0};
  double mlp_compensation_y_{0.0};
  double mlp_compensation_z_{0.0};
};

} // namespace position_controller

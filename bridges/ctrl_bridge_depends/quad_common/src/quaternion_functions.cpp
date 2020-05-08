#include "quad_common/quaternion_functions.h"

namespace quad_common
{

Eigen::Vector3d quaternionRatesToBodyRates(Eigen::Quaterniond q2, Eigen::Quaterniond q1, double dt, int coord_frame)
{
  // Computes the angular velocity in body coordinate system for a rotation from q1 to q2
  // within the time dt

  Eigen::Quaterniond q_dot;

  // Note, that the space of unit quaternion S(3) double covers the space
  // of physical attitudes SO(3) therefore q = -q.
  // I want the minimal q_dot, therefore I need to check which is
  // better q or -q (they both represent the same attitude in space)

  if ((-q2.coeffs() - q1.coeffs()).norm() < (q2.coeffs() - q1.coeffs()).norm())
  {
    q2.coeffs() = -q2.coeffs();
  }
  q_dot.coeffs() = (q2.coeffs() - q1.coeffs()) / dt;

  // [ o W_omega ]' = 2q_dot * q_bar in world frame
  // [ o B_omega ]' = 2q_bar * q_dot in body frame
  if (coord_frame == WORLD_FRAME)
  {
    return Eigen::Vector3d(2.0 * (q_dot * q2.conjugate()).vec());
  }
  else
  {
    return Eigen::Vector3d(2.0 * (q2.conjugate() * q_dot).vec());
  }
}

Eigen::Vector3d quaternionDeltaToBodyRates(Eigen::Quaterniond q2, Eigen::Quaterniond q1, double dt, int coord_frame)
{
  // This is basically the inverse of the integrateQuaternion function
  Eigen::Quaterniond q_e = q1.inverse() * q2;

  if (q_e.w() < 0)
  {
    // Make sure real part of quaternion has positive sign such that we take the minimum distance from q1 to q2 to compute the body rates
    q_e = Eigen::Quaterniond(-q_e.w(), -q_e.x(), -q_e.y(), -q_e.z());
  }

  double a = sinc(acos(q_e.w()) / M_PI);

  Eigen::Vector3d alpha(q_e.x() / a, q_e.y() / a, q_e.z() / a);

  Eigen::Vector3d omega = 2.0 * alpha / dt;

  if (coord_frame == WORLD_FRAME)
  {
    return q2 * omega;
  }
  else
  {
    return omega;
  }
}

double sinc(double a)
{
  if (fabs(a) >= 1e-5)
  {
    return sin(M_PI * a) / (M_PI * a);
  }
  else
  {
    return 1.0;
  }
}

Eigen::Quaterniond integrateQuaternion(Eigen::Quaterniond q_start, Eigen::Vector3d bodyrates, double dt)
{
  // pushes the orientation forward in time assuming constant bodyrates B_omega_WB
  // make sure that the bodyrates are in the body frame not the world frame
  Eigen::Quaterniond q_end;

  double p = bodyrates.x();
  double q = bodyrates.y();
  double r = bodyrates.z();

  double omega_norm = bodyrates.norm();
  // only do something if the bodyrates are not euqal to zero
  if (omega_norm > 1e-40)
  {
    Eigen::Matrix4d Lambda;
    Lambda << 0, -p, -q, -r, p, 0, r, -q, q, -r, 0, p, r, q, -p, 0;
    Lambda = Lambda * 0.5;

    // here i use a hack because coeffs returns [x y z w ] but we always work with [ w x y z ]
    Eigen::Vector4d q_start_as_vector(q_start.w(), q_start.x(), q_start.y(), q_start.z()); // [w x y z ]
    Eigen::Vector4d q_end_as_vector; // [w x y z ]

    q_end_as_vector = (Eigen::Matrix4d::Identity() * cos(omega_norm * dt / 2.0)
        + 2.0 / omega_norm * Lambda * sin(omega_norm * dt / 2.0)) * q_start_as_vector;
    q_end = Eigen::Quaterniond(q_end_as_vector(0), q_end_as_vector(1), q_end_as_vector(2), q_end_as_vector(3));
  }
  else
  {
    q_end = q_start;
  }
  return q_end;
}

} //quad_common

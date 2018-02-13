#include "quadrotor_common/math_common.h"

namespace quadrotor_common
{

double fcToAlpha(const double fc, const double dt)
{
  if (fc < 0.0001)
  {
    return 0;
  }
  else
  {
    double tau = 1.0 / (2.0 * M_PI * fc); // [s/rad]
    return dt / (tau + dt);
  }
}

float lowpass(const float filterin, const float input, const double alpha)
{
  float filterout;
  filterout = (1.0 - alpha) * filterin + alpha * input;
  return filterout;
}

float lowpass(const float filterin, const float input, const double fc,
              const double dt)
{
  double alpha;
  alpha = fcToAlpha(fc, dt);
  return lowpass(filterin, input, alpha);
}

Eigen::Vector3d lowpass(const Eigen::Vector3d& filterin,
                        const Eigen::Vector3d& input, const double alpha)
{
  Eigen::Vector3d filterout;
  filterout = (1.0 - alpha) * filterin + alpha * input;
  return filterout;
}

Eigen::Vector3d lowpass(const Eigen::Vector3d& filterin,
                        const Eigen::Vector3d& input, const double fc,
                        const double dt)
{
  double alpha;
  alpha = fcToAlpha(fc, dt);
  return lowpass(filterin, input, alpha);
}

Eigen::Quaterniond lowpass(const Eigen::Quaterniond& filterin,
                           const Eigen::Quaterniond& input, const double alpha)
{
  return filterin.slerp(alpha, input);
}

Eigen::Quaterniond lowpass(const Eigen::Quaterniond& filterin,
                           const Eigen::Quaterniond& input, const double fc,
                           const double dt)
{
  double alpha;
  alpha = fcToAlpha(fc, dt);
  return filterin.slerp(alpha, input);
}

Eigen::Affine3d lowpass(const Eigen::Affine3d& filterin,
                        const Eigen::Affine3d& input, const double alpha)
{
  Eigen::Vector3d filter_position = filterin.translation();
  Eigen::Quaterniond filer_orientation(filterin.rotation());

  Eigen::Vector3d input_position = input.translation();
  Eigen::Quaterniond input_orientation(input.rotation());

  Eigen::Vector3d filter_out_position;
  Eigen::Quaterniond filter_out_orientation;

  filter_out_position = lowpass(filter_position, input_position, alpha);
  filter_out_orientation = lowpass(filer_orientation, input_orientation, alpha);

  Eigen::Affine3d filter_out = Eigen::Translation3d(filter_out_position)
      * filter_out_orientation;
  return filter_out;
}

Eigen::Affine3d lowpass(const Eigen::Affine3d& filterin,
                        const Eigen::Affine3d& input, const double fc,
                        const double dt)
{
  double alpha;
  alpha = fcToAlpha(fc, dt);
  return lowpass(filterin, input, alpha);
}

double wrapZeroToTwoPi(const double angle)
{
  if (angle >= 0.0 && angle <= 2.0 * M_PIl)
  {
    return angle;
  }
  double wrapped_angle = fmod(angle, 2.0 * M_PIl);
  if (wrapped_angle < 0.0)
  {
    wrapped_angle += 2.0 * M_PIl;
  }
  return wrapped_angle;
}

double wrapMinusPiToPi(const double angle)
{
  if (angle >= -M_PIl && angle <= M_PIl)
  {
    return angle;
  }
  double wrapped_angle = angle + M_PIl;
  wrapped_angle = wrapZeroToTwoPi(wrapped_angle);
  wrapped_angle -= M_PIl;
  return wrapped_angle;
}

double wrapAngleDifference(const double current_angle,
                           const double desired_angle)
{
  double angle_diff = wrapZeroToTwoPi(desired_angle)
      - wrapZeroToTwoPi(current_angle);
  if (angle_diff > M_PIl)
  {
    angle_diff = (-2.0 * M_PIl + angle_diff);
  }
  if (angle_diff < -M_PIl)
  {
    angle_diff = (2.0 * M_PIl + angle_diff);
  }
  return angle_diff;
}

void limit(double *val, const double min, const double max)
{
  if (*val > max)
  {
    *val = max;
  }
  if (*val < min)
  {
    *val = min;
  }
}

Eigen::Matrix3d skew(const Eigen::Vector3d& v)
{
  Eigen::Matrix3d v_hat;
  v_hat << 0.0, -v.z(), v.y(), v.z(), 0.0, -v.x(), -v.y(), v.x(), 0.0;
  return v_hat;
}

double degToRad(const double deg)
{
  return 0.017453292519943 * deg;
}

double radToDeg(const double rad)
{
  return 57.295779513082323 * rad;
}

Eigen::Vector3d quaternionToEulerAnglesZYX(const Eigen::Quaterniond& q)
{
  Eigen::Vector3d euler_angles;
  euler_angles(0) = atan2(
      2.0 * q.w() * q.x() + 2.0 * q.y() * q.z(),
      q.w() * q.w() - q.x() * q.x() - q.y() * q.y() + q.z() * q.z());
  euler_angles(1) = -asin(2.0 * q.x() * q.z() - 2.0 * q.w() * q.y());
  euler_angles(2) = atan2(
      2.0 * q.w() * q.z() + 2.0 * q.x() * q.y(),
      q.w() * q.w() + q.x() * q.x() - q.y() * q.y() - q.z() * q.z());
  return euler_angles;
}

Eigen::Quaterniond eulerAnglesZYXToQuaternion(
    const Eigen::Vector3d& euler_angles)
{
  Eigen::Quaterniond q;
  double r = euler_angles(0) / 2.0;
  double p = euler_angles(1) / 2.0;
  double y = euler_angles(2) / 2.0;
  q.w() = cos(r) * cos(p) * cos(y) + sin(r) * sin(p) * sin(y);
  q.x() = sin(r) * cos(p) * cos(y) - cos(r) * sin(p) * sin(y);
  q.y() = cos(r) * sin(p) * cos(y) + sin(r) * cos(p) * sin(y);
  q.z() = cos(r) * cos(p) * sin(y) - sin(r) * sin(p) * cos(y);
  return q;
}

Eigen::Vector3d rotationMatrixToEulerAnglesZYX(const Eigen::Matrix3d& R)
{
  Eigen::Vector3d euler_angles;
  euler_angles(0) = atan2(R(2, 1), R(2, 2));
  euler_angles(1) = -atan2(R(2, 0),
                           sqrt(pow(R(2, 1), 2.0) + pow(R(2, 2), 2.0)));
  euler_angles(2) = atan2(R(1, 0), R(0, 0));
  return euler_angles;
}

Eigen::Matrix3d eulerAnglesZYXToRotationMatrix(
    const Eigen::Vector3d& euler_angles)
{
  double r = euler_angles(0);
  double p = euler_angles(1);
  double y = euler_angles(2);

  Eigen::Matrix3d R;
  R(0, 0) = cos(y) * cos(p);
  R(1, 0) = sin(y) * cos(p);
  R(2, 0) = -sin(p);

  R(0, 1) = cos(y) * sin(p) * sin(r) - sin(y) * cos(r);
  R(1, 1) = sin(y) * sin(p) * sin(r) + cos(y) * cos(r);
  R(2, 1) = cos(p) * sin(r);

  R(0, 2) = cos(y) * sin(p) * cos(r) + sin(y) * sin(r);
  R(1, 2) = sin(y) * sin(p) * cos(r) - cos(y) * sin(r);
  R(2, 2) = cos(p) * cos(r);

  return R;
}

Eigen::Matrix3d quaternionToRotationMatrix(const Eigen::Quaterniond& q)
{
  Eigen::Matrix3d R;

  R(0, 0) = q.w() * q.w() + q.x() * q.x() - q.y() * q.y() - q.z() * q.z();
  R(1, 0) = 2.0 * q.w() * q.z() + 2.0 * q.x() * q.y();
  R(2, 0) = 2.0 * q.x() * q.z() - 2.0 * q.w() * q.y();

  R(0, 1) = 2.0 * q.x() * q.y() - 2.0 * q.w() * q.z();
  R(1, 1) = q.w() * q.w() - q.x() * q.x() + q.y() * q.y() - q.z() * q.z();
  R(2, 1) = 2.0 * q.w() * q.x() + 2.0 * q.y() * q.z();

  R(0, 2) = 2.0 * q.w() * q.y() + 2.0 * q.x() * q.z();
  R(1, 2) = 2.0 * q.y() * q.z() - 2.0 * q.w() * q.x();
  R(2, 2) = q.w() * q.w() - q.x() * q.x() - q.y() * q.y() + q.z() * q.z();

  return R;
}

Eigen::Quaterniond RotationMatrixToQuaternion(const Eigen::Matrix3d& R)
{
  return Eigen::Quaterniond(R);
}

} // quadrotor_common

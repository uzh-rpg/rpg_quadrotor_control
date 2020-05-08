/*
 * torques_and_thrust.h
 *
 *  Created on: Sep 22, 2015
 *      Author: davide
 */

#pragma once

#include <ros/ros.h>
#include "Eigen/Dense"

namespace quad_common
{

class TorquesAndThrust
{

public:
  TorquesAndThrust();
  ~TorquesAndThrust();

  void zero();

  ros::Time timestamp;

  double thrust; // m/s^2
  Eigen::Vector3d torques;

};

}
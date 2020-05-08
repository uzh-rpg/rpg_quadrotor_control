/*
 * torques_and_thrust.cpp
 *
 *  Created on: Sep 22, 2015
 *      Author: davide
 */

#include "quad_common/torques_and_thrust.h"

namespace quad_common
{

TorquesAndThrust::TorquesAndThrust() :
timestamp(ros::Time::now()), thrust(0.0), torques(Eigen::Vector3d::Zero())
{
}

TorquesAndThrust::~TorquesAndThrust()
{
}

void TorquesAndThrust::zero()
{
  timestamp = ros::Time::now();
  torques = Eigen::Vector3d::Zero();
  thrust = 0.0;
}

}
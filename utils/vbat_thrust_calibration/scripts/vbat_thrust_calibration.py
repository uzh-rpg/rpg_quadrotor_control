#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import AutopilotFeedback
from quadrotor_msgs.msg import ControlCommand
from quadrotor_msgs.msg import LowLevelFeedback
import rospy


class VbatThrustCalibration:

    def __init__(self):

        # Load parameters
        self.time_average_interval = float(
            rospy.get_param('~time_average_interval', 1.0))
        self.vbat_min = float(
            rospy.get_param('~min_battery_voltage', 10.7))
        self.vbat_max = float(
            rospy.get_param('~max_battery_voltage', 12.2))
        self.hover_tolerance_radius = float(
            rospy.get_param('~hover_tolerance_radius', 0.3))

        self.min_n_samples = 10

        # store measurements of vbat and commanded thrusts
        self.vbat_meas = np.array([])
        self.thrust_cmds = np.array([])

        # We solve A x = B such that || B - A x ||^2 is minimized
        self.A = np.empty([0, 2])
        self.B = np.empty([0, 1])

        # Store min and max battery voltages used for identification
        self.id_max_voltage = self.vbat_max
        self.id_min_voltage = self.vbat_min

        # Some flags
        self.received_first_low_level_feedback = False
        self.received_first_state_estimate = False
        self.received_first_autopilot_feedback = False
        self.collecting_data = False
        self.vbat_in_range = False

        rospy.Subscriber(
            "low_level_feedback", LowLevelFeedback, self.low_level_feedback_cb)
        rospy.Subscriber(
            "control_command", ControlCommand, self.control_command_cb)
        rospy.Subscriber(
            "state_estimate", Odometry, self.state_estimate_cb)
        rospy.Subscriber(
            "autopilot/feedback", AutopilotFeedback, self.autopilot_fb_cb)

    def low_level_feedback_cb(self, msg):

        if not self.received_first_low_level_feedback:
            if msg.battery_voltage < self.vbat_max:
                rospy.logwarn(
                    "[{0}] Battery voltage was below maximum of estimation "
                    "interval ({1} < {2}) when starting the "
                    "calibration, it will proceed anyway".format(
                        rospy.get_name(), msg.battery_voltage, self.vbat_max))
                self.id_max_voltage = msg.battery_voltage
                self.vbat_in_range = True
            rospy.loginfo(
                "[{0}] Received first low level feedback message".format(
                    rospy.get_name()))
            self.received_first_low_level_feedback = True

        if msg.battery_voltage <= self.vbat_max and not self.vbat_in_range:
            rospy.loginfo(
                "[{0}] Battery voltage dropped below max capturing "
                "threshold, starting calibration".format(rospy.get_name()))
            self.id_max_voltage = self.vbat_max
            self.vbat_in_range = True

        # Check if battery is low
        if msg.battery_voltage < self.vbat_min:
            rospy.loginfo(
                "[{0}] Stopping because of low battery voltage ({1}V)".format(
                    rospy.get_name(), msg.battery_voltage))

            self.id_min_voltage = self.vbat_min
            self.compute_coefficients()

            rospy.signal_shutdown("Stopped because of low battery")

        # Store vbat measurements
        if self.collecting_data:
            self.vbat_meas = np.append(
                self.vbat_meas, np.array([msg.battery_voltage]), axis=0)

    def control_command_cb(self, msg):

        if (self.received_first_low_level_feedback and
                self.received_first_state_estimate and
                self.received_first_autopilot_feedback and
                self.vbat_in_range and not self.collecting_data):
            rospy.loginfo(
                "[{0}] Starting to collect data now".format(
                    rospy.get_name()))
            self.collecting_data = True
            self.t_started_average_interval = rospy.get_time()

        if self.collecting_data:
            self.thrust_cmds = np.append(
                self.thrust_cmds, np.array([msg.collective_thrust]), axis=0)

        # Check if a time interval has passed
        if (self.collecting_data and
                (rospy.get_time() - self.t_started_average_interval) >=
                self.time_average_interval):
            # Store averaged measurements
            if self.vbat_meas.shape[0] > 0 and self.thrust_cmds.shape[0] > 0:
                # Avoid considering intervals where no messages were received
                self.A = np.append(
                    self.A, [[np.mean(self.vbat_meas), 1]], axis=0)
                self.B = np.append(
                    self.B, [[np.mean(self.thrust_cmds) / 9.81]], axis=0)
            # Reset values
            self.vbat_meas = np.array([])
            self.thrust_cmds = np.array([])
            self.t_started_average_interval = rospy.get_time()

    def state_estimate_cb(self, msg):

        if not self.received_first_state_estimate:
            self.hover_position = np.array(
                [msg.pose.pose.position.x, msg.pose.pose.position.y,
                 msg.pose.pose.position.z])
            rospy.loginfo(
                "[{0}] Received first state estimate message".format(
                    rospy.get_name()))
            self.received_first_state_estimate = True

        # Check if we are hovering well
        hover_deviation = np.linalg.norm(
            self.hover_position - np.array(
                [msg.pose.pose.position.x, msg.pose.pose.position.y,
                 msg.pose.pose.position.z]))
        if hover_deviation > self.hover_tolerance_radius:
            rospy.logerr(
                "[{0}] Too large deviation from hover position, aborting "
                "calibration, consider redoing this calibration instead of "
                "using the computed coefficients".format(rospy.get_name()))
            self.id_min_voltage = self.A[-1][0]
            self.compute_coefficients()
            rospy.signal_shutdown("Too large hover deviation")

    def autopilot_fb_cb(self, msg):

        if not self.received_first_autopilot_feedback:
            rospy.loginfo(
                "[{0}] Received first autopilot feedback message".format(
                    rospy.get_name()))
            self.received_first_autopilot_feedback = True

        if msg.autopilot_state != msg.HOVER:
            rospy.logerr(
                "[{0}] Autopilot is not in hover, stopping vbat thrust "
                "calibration.".format(rospy.get_name()))
            rospy.signal_shutdown("Autopilot not in hover")

    def compute_coefficients(self):

        if self.A.shape[0] < self.min_n_samples:
            rospy.logerr(
                "[{0}] Too few samples ({1} < {2}) to compute vbat thrust "
                "calibration".format(
                    rospy.get_name(), self.A.shape[0], self.min_n_samples))
            return

        sol = np.linalg.lstsq(self.A, self.B)
        if sol[2] < 2:  # Check rank of A
            rospy.logerr(
                "[{0}] Could not compute coefficients because A matrix "
                "does not have full rank".format(rospy.get_name()))
        else:
            rospy.loginfo(
                "[{0}] Calibration was performed for battery voltages between "
                "{1}V and {2}V:".format(rospy.get_name(), self.id_max_voltage,
                                        self.id_min_voltage))
            rospy.loginfo(
                "[{0}] Vbat Thrust calibration coefficients:".format(
                    rospy.get_name()))
            rospy.loginfo(
                "[{0}] thrust_ratio_voltage_map_a: {1}".format(
                    rospy.get_name(), sol[0][0]))
            rospy.loginfo(
                "[{0}] thrust_ratio_voltage_map_b: {1}".format(
                    rospy.get_name(), sol[0][1]))


if __name__ == '__main__':

    try:
        rospy.init_node("vbat_thrust_calibration", anonymous=True)

        vbat_thrust_calibration = VbatThrustCalibration()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass

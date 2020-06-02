#!/usr/bin/env python
import os
import rospy
import rospkg

from python_qt_binding import loadUi
try:
    # Starting from Qt 5 QWidget is defined in QtWidgets and not QtGui anymore
    from python_qt_binding.QtWidgets import QWidget
except:
    from python_qt_binding.QtGui import QWidget
from python_qt_binding.QtCore import QTimer, Slot
from python_qt_binding.QtCore import pyqtSlot

import geometry_msgs.msg as geometry_msgs
import quadrotor_msgs.msg as quadrotor_msgs
import std_msgs.msg as std_msgs

import math
import numpy as np


class AutopilotWidget(QWidget):

    def __init__(self, parent):
        # Init QWidget
        super(AutopilotWidget, self).__init__(parent)
        self.setObjectName('Autopilot Widget')

        # set variables
        self._quad_namespace = None
        self._connected = False

        self._arm_bridge_pub = None
        self._start_pub = None
        self._land_pub = None
        self._off_pub = None
        self._force_hover_pub = None
        self._go_to_pose_pub = None

        self._autopilot_feedback_sub = None
        self._autopilot_feedback = quadrotor_msgs.AutopilotFeedback()
        self._autopilot_feedback_stamp = rospy.Time.now()

        self._previous_autopilot_state = self._autopilot_feedback.OFF

        # load UI
        ui_file = os.path.join(
            os.path.dirname(os.path.realpath(__file__)),
            '../../resource/autopilot_widget.ui')
        loadUi(ui_file, self)

        # Timer
        self._update_info_timer = QTimer(self)
        self._update_info_timer.timeout.connect(self.update_gui)
        self._update_info_timer.start(100)

        self.disconnect()

    def connect(self, quad_namespace):
        self._quad_namespace = quad_namespace

        self._arm_bridge_pub = rospy.Publisher(
            quad_namespace+'/bridge/arm', std_msgs.Bool, queue_size=1)
        self._start_pub = rospy.Publisher(
            quad_namespace+'/autopilot/start', std_msgs.Empty, queue_size=1)
        self._land_pub = rospy.Publisher(
            quad_namespace+'/autopilot/land', std_msgs.Empty, queue_size=1)
        self._off_pub = rospy.Publisher(
            quad_namespace+'/autopilot/off', std_msgs.Empty, queue_size=1)
        self._force_hover_pub = rospy.Publisher(
            quad_namespace+'/autopilot/force_hover', std_msgs.Empty,
            queue_size=1)
        self._go_to_pose_pub = rospy.Publisher(
            quad_namespace+'/autopilot/pose_command', geometry_msgs.PoseStamped,
            queue_size=1)

        self._autopilot_feedback_sub = rospy.Subscriber(
            quad_namespace+'/autopilot/feedback',
            quadrotor_msgs.AutopilotFeedback, self.autopilot_feedback_cb)

        self.button_arm_bridge.setEnabled(True)
        self.button_start.setEnabled(True)
        self.button_land.setEnabled(True)
        self.button_off.setEnabled(True)
        self.button_force_hover.setEnabled(True)
        self.button_go_to_pose.setEnabled(False)



        self._connected = True

    def disconnect_pub_sub(self, pub):
        if pub is not None:
            pub.unregister()
            pub = None

    def disconnect(self):
        self.disconnect_pub_sub(self._autopilot_feedback_sub)

        self.disconnect_pub_sub(self._arm_bridge_pub)
        self.disconnect_pub_sub(self._start_pub)
        self.disconnect_pub_sub(self._land_pub)
        self.disconnect_pub_sub(self._off_pub)
        self.disconnect_pub_sub(self._force_hover_pub)
        self.disconnect_pub_sub(self._go_to_pose_pub)

        self.button_arm_bridge.setEnabled(False)
        self.button_start.setEnabled(False)
        self.button_land.setEnabled(False)
        self.button_off.setEnabled(False)
        self.button_force_hover.setEnabled(False)
        self.button_go_to_pose.setEnabled(False)

        self._connected = False

    def autopilot_feedback_cb(self, msg):
        self._autopilot_feedback = msg
        self._autopilot_feedback_stamp = rospy.Time.now()

    def update_gui(self):
        if (self._connected and self.autopilot_feedback_available()):
            # Autopilot status
            self.autopilot_state.setText(
                self.get_autopilot_state_name(
                    self._autopilot_feedback.autopilot_state))
            self.control_command_delay.setText('%.1f'
                % (self._autopilot_feedback.control_command_delay.to_sec() * 1000.0))
            self.control_computation_time.setText('%.1f'
                % (self._autopilot_feedback.control_computation_time.to_sec() * 1000.0))
            self.trajectory_execution_left_duration.setText('%.1f'
                % self._autopilot_feedback.trajectory_execution_left_duration.to_sec())
            self.trajectories_left_in_queue.setText(
                str(self._autopilot_feedback.trajectories_left_in_queue))

            # Low-Level status
            if (self._autopilot_feedback.low_level_feedback.battery_state ==
                    self._autopilot_feedback.low_level_feedback.BAT_GOOD):
                self.status_battery_voltage.setStyleSheet('QLabel { color : green; }')
                self.status_battery_state.setStyleSheet('QLabel { color : green; }')
            elif (self._autopilot_feedback.low_level_feedback.battery_state ==
                    self._autopilot_feedback.low_level_feedback.BAT_LOW):
                self.status_battery_voltage.setStyleSheet('QLabel { color : orange; }')
                self.status_battery_state.setStyleSheet('QLabel { color : orange; }')
            else:
                self.status_battery_voltage.setStyleSheet('QLabel { color : red; }')
                self.status_battery_state.setStyleSheet('QLabel { color : red; }')

            self.status_battery_state.setText(
                self.get_battery_state_name(
                    self._autopilot_feedback.low_level_feedback.battery_state))
            self.status_battery_voltage.setText(
                '%.2f' % self._autopilot_feedback.low_level_feedback.battery_voltage)
            self.status_control_mode.setText(
                self.get_control_mode_name(
                    self._autopilot_feedback.low_level_feedback.control_mode))

            # State Estimate
            self.state_est_frame_id.setText(
                'Frame ID: %s' % self._autopilot_feedback.state_estimate.header.frame_id)
            euler_angles = np.rad2deg(
                self.quat_to_euler_angles(
                    self._autopilot_feedback.state_estimate.pose.pose.orientation))
            self.state_est_position.setText('x:%+.2f  y:%+.2f  z:%+.2f' %
                (self._autopilot_feedback.state_estimate.pose.pose.position.x,
                 self._autopilot_feedback.state_estimate.pose.pose.position.y,
                 self._autopilot_feedback.state_estimate.pose.pose.position.z))
            self.state_est_velocity.setText('x:%+.2f  y:%+.2f  z:%+.2f' %
                (self._autopilot_feedback.state_estimate.twist.twist.linear.x,
                 self._autopilot_feedback.state_estimate.twist.twist.linear.y,
                 self._autopilot_feedback.state_estimate.twist.twist.linear.z))
            self.state_est_orientation.setText('r:%+.1f  p:%+.1f  h:%+.1f' %
                (euler_angles[0], euler_angles[1], euler_angles[2]))
            self.state_est_body_rates.setText('x:%+.2f  y:%+.2f  z:%+.2f' %
                (self._autopilot_feedback.state_estimate.twist.twist.angular.x / math.pi * 180.0,
                 self._autopilot_feedback.state_estimate.twist.twist.angular.y / math.pi * 180.0,
                 self._autopilot_feedback.state_estimate.twist.twist.angular.z / math.pi * 180.0))

            # Reference State
            self.ref_position.setText('x:%(x)+.2f  y:%(y)+.2f  z:%(z)+.2f' %
                {'x': self._autopilot_feedback.reference_state.pose.position.x,
                 'y': self._autopilot_feedback.reference_state.pose.position.y,
                 'z': self._autopilot_feedback.reference_state.pose.position.z})
            self.ref_velocity.setText('x:%(x)+.2f  y:%(y)+.2f  z:%(z)+.2f' %
                {'x': self._autopilot_feedback.reference_state.velocity.linear.x,
                 'y': self._autopilot_feedback.reference_state.velocity.linear.y,
                 'z': self._autopilot_feedback.reference_state.velocity.linear.z})
            self.ref_acc.setText('x:%(x)+.2f  y:%(y)+.2f  z:%(z)+.2f' %
                {'x': self._autopilot_feedback.reference_state.acceleration.linear.x,
                 'y': self._autopilot_feedback.reference_state.acceleration.linear.y,
                 'z': self._autopilot_feedback.reference_state.acceleration.linear.z})
            self.ref_heading.setText('%+.1f' % (self._autopilot_feedback.reference_state.heading / math.pi * 180.0))

            # Update go_to fields and enable go to pose button on if in HOVER
            if (self._autopilot_feedback.autopilot_state == self._autopilot_feedback.HOVER and
                    self._previous_autopilot_state == self._autopilot_feedback.HOVER):

                self.button_go_to_pose.setEnabled(True)
                self.go_to_pose_x.setEnabled(True)
                self.go_to_pose_y.setEnabled(True)
                self.go_to_pose_z.setEnabled(True)
                self.go_to_pose_heading.setEnabled(True)

            else:

                self.button_go_to_pose.setDisabled(True)
                self.go_to_pose_x.setDisabled(True)
                self.go_to_pose_y.setDisabled(True)
                self.go_to_pose_z.setDisabled(True)
                self.go_to_pose_heading.setDisabled(True)

                self.go_to_pose_x.setText('%.2f' % (self._autopilot_feedback.reference_state.pose.position.x))
                self.go_to_pose_y.setText('%.2f' % (self._autopilot_feedback.reference_state.pose.position.y))
                self.go_to_pose_z.setText('%.2f' % (self._autopilot_feedback.reference_state.pose.position.z))
                self.go_to_pose_heading.setText('%.0f' % (self._autopilot_feedback.reference_state.heading / math.pi * 180.0))

            self._previous_autopilot_state = self._autopilot_feedback.autopilot_state

        else:
            # Autopilot status
            self.autopilot_state.setText('Not Available')
            self.control_command_delay.setText('0.0')
            self.control_computation_time.setText('0.0')
            self.trajectory_execution_left_duration.setText('0.0')
            self.trajectories_left_in_queue.setText('0')

            # Low-Level status
            self.status_battery_voltage.setStyleSheet('QLabel { color : gray; }')
            self.status_battery_state.setStyleSheet('QLabel { color : gray; }')

            self.status_battery_state.setText('Not Available')
            self.status_battery_voltage.setText('Not Available')
            self.status_control_mode.setText('Not Available')

            # State Estimate
            self.state_est_frame_id.setText('Frame ID:')
            self.state_est_position.setText('Not Available')
            self.state_est_velocity.setText('Not Available')
            self.state_est_orientation.setText('Not Available')
            self.state_est_body_rates.setText('Not Available')

            # Reference State
            self.ref_position.setText('Not Available')
            self.ref_velocity.setText('Not Available')
            self.ref_acc.setText('Not Available')
            self.ref_heading.setText('Not Available')


    @Slot(bool)
    def on_button_connect_clicked(self):
        if(self._connected):
            self.disconnect()
            self.button_connect.setText('Connect')
        else:
            quad_namespace = self.namespace_text.text()
            self.connect(quad_namespace)
            self.button_connect.setText('Disconnect')

    @Slot(bool)
    def on_button_arm_bridge_clicked(self):
        arm_message = std_msgs.Bool(True)
        self._arm_bridge_pub.publish(arm_message)

    @Slot(bool)
    def on_button_start_clicked(self):
        start_message = std_msgs.Empty()
        self._start_pub.publish(start_message)

    @Slot(bool)
    def on_button_land_clicked(self):
        land_message = std_msgs.Empty()
        self._land_pub.publish(land_message)

    @Slot(bool)
    def on_button_off_clicked(self):
        off_message = std_msgs.Empty()
        self._off_pub.publish(off_message)
        arm_message = std_msgs.Bool(False)
        self._arm_bridge_pub.publish(arm_message)

    @Slot(bool)
    def on_button_force_hover_clicked(self):
        force_hover_msg = std_msgs.Empty()
        self._force_hover_pub.publish(force_hover_msg)

    @Slot(bool)
    def on_button_go_to_pose_clicked(self):
        try:
            go_to_pose_msg = geometry_msgs.PoseStamped()
            go_to_pose_msg.pose.position.x = float(self.go_to_pose_x.text())
            go_to_pose_msg.pose.position.y = float(self.go_to_pose_y.text())
            go_to_pose_msg.pose.position.z = float(self.go_to_pose_z.text())

            heading = float(self.go_to_pose_heading.text()) / 180.0 * math.pi

            go_to_pose_msg.pose.orientation.w = math.cos(heading / 2.0)
            go_to_pose_msg.pose.orientation.z = math.sin(heading / 2.0)

            self._go_to_pose_pub.publish(go_to_pose_msg)
        except:
            rospy.logwarn("Could not read and send go to pose message!")

    def autopilot_feedback_available(self):
        if (rospy.Time.now() - self._autopilot_feedback_stamp).to_sec() <= 1.0:
            return True
        return False

    def get_autopilot_state_name(self, autopilot_state):
        if (autopilot_state == self._autopilot_feedback.START):
            return "START"
        if (autopilot_state == self._autopilot_feedback.HOVER):
            return "HOVER"
        if (autopilot_state == self._autopilot_feedback.LAND):
            return "LAND"
        if (autopilot_state == self._autopilot_feedback.EMERGENCY_LAND):
            return "EMERGENCY_LAND"
        if (autopilot_state == self._autopilot_feedback.BREAKING):
            return "BREAKING"
        if (autopilot_state == self._autopilot_feedback.GO_TO_POSE):
            return "GO_TO_POSE"
        if (autopilot_state == self._autopilot_feedback.VELOCITY_CONTROL):
            return "VELOCITY_CONTROL"
        if (autopilot_state == self._autopilot_feedback.REFERENCE_CONTROL):
            return "REFERENCE_CONTROL"
        if (autopilot_state == self._autopilot_feedback.TRAJECTORY_CONTROL):
            return "TRAJECTORY_CONTROL"
        if (autopilot_state == self._autopilot_feedback.COMMAND_FEEDTHROUGH):
            return "COMMAND_FEEDTHROUGH"
        if (autopilot_state == self._autopilot_feedback.RC_MANUAL):
            return "RC_MANUAL"
        return "OFF"

    def get_battery_state_name(self, battery_state):
        if (battery_state == self._autopilot_feedback.low_level_feedback.BAT_GOOD):
            return "Good"
        if (battery_state == self._autopilot_feedback.low_level_feedback.BAT_LOW):
            return "Low"
        if (battery_state == self._autopilot_feedback.low_level_feedback.BAT_CRITICAL):
            return "Critical"
        return "Invalid"

    def get_control_mode_name(self, control_mode):
        if (control_mode == self._autopilot_feedback.low_level_feedback.ATTITUDE):
            return "Attitude"
        if (control_mode == self._autopilot_feedback.low_level_feedback.BODY_RATES):
            return "Body Rates"
        if (control_mode == self._autopilot_feedback.low_level_feedback.ANGULAR_ACCELERATION):
            return "Angular Accelerations"
        if (control_mode == self._autopilot_feedback.low_level_feedback.ROTOR_THRUSTS):
            return "Rotor Thrusts"
        if (control_mode == self._autopilot_feedback.low_level_feedback.RC_MANUAL):
            return "RC_Manual"
        return "None"

    def quat_to_euler_angles(self, q):
        #  Computes the euler angles from a unit quaternion using the
        #  z-y-x convention.
        #  euler_angles = [roll pitch yaw]'
        #  A quaternion is defined as q = [qw qx qy qz]'
        #  where qw is the real part.

        euler_angles = np.zeros((3, 1))

        euler_angles[0] = np.arctan2(
            2*q.w*q.x + 2*q.y*q.z, q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z)
        euler_angles[1] = -np.arcsin(2*q.x*q.z - 2*q.w*q.y)
        euler_angles[2] = np.arctan2(
            2*q.w*q.z + 2*q.x*q.y, q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z)
        return euler_angles

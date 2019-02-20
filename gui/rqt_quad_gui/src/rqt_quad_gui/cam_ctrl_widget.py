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

import std_msgs.msg as std_msgs
import quad_msgs.msg as quad_msgs
import std_srvs.srv as std_srvs

class CameraControlWidget(QWidget):

    def __init__(self, parent):
        super(CameraControlWidget, self).__init__(parent)
        self.setObjectName('CameraControlWidget')

        self._set_fixed_pub = None
        self._set_cam_ctrl_pub = None
        self._svo_remote_pub = None

        # load UI
        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), '../../resource/cam_ctrl_widget.ui')
        loadUi(ui_file, self)

        self.disconnect()

    def update_gui(self):
        pass

    def connect(self, quad_namespace):
        self._set_fixed_pub = rospy.Publisher(quad_namespace+'/set_camera_exposure', std_msgs.String, queue_size=1)
        self._set_cam_ctrl_pub = rospy.Publisher(quad_namespace+'/set_camera_control', std_msgs.String, queue_size=1)
        self._svo_remote_pub = rospy.Publisher(quad_namespace+'/svo/remote_key', std_msgs.String, queue_size=1)

        self.button_set_from_builtin.setEnabled( True )
        self.button_set_from_preset.setEnabled( True )
        self.button_set_gradient.setEnabled( True )
        self.button_set_intensity.setEnabled( True )

    def disconnect_pub_sub(self, pub):
        if pub is not None:
            pub.unregister()
            pub = None

    def disconnect(self):
        self.disconnect_pub_sub(self._set_fixed_pub)

        self.button_set_from_builtin.setEnabled( False )
        self.button_set_from_preset.setEnabled( False )
        self.button_set_gradient.setEnabled( False )
        self.button_set_intensity.setEnabled( False )

    @Slot(bool)
    def on_button_set_from_builtin_clicked(self):
        str_msg = std_msgs.String("Built-in")
        self._set_fixed_pub.publish(str_msg)
        self.ctrl_type.setText("Built-in")
        self._svo_remote_pub.publish(std_msgs.String("C"))

    @Slot(bool)
    def on_button_set_from_preset_clicked(self):
        str_msg = std_msgs.String("Preset")
        self._set_fixed_pub.publish(str_msg)
        self.ctrl_type.setText("Preset")
        self._svo_remote_pub.publish(std_msgs.String("C"))

    @Slot(bool)
    def on_button_set_intensity_clicked(self):
        str_msg = std_msgs.String("AutoExposureGainIntensity")
        self._set_cam_ctrl_pub.publish( str_msg )
        self.ctrl_type.setText("Intensity")
        self._svo_remote_pub.publish(std_msgs.String("c"))

    @Slot(bool)
    def on_button_set_gradient_clicked(self):
        str_msg = std_msgs.String("AutoExposureGainGradient")
        self._set_cam_ctrl_pub.publish( str_msg )
        self.ctrl_type.setText("Gradient")
        self._svo_remote_pub.publish(std_msgs.String("c"))

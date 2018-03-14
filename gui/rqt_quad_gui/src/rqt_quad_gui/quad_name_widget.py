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


class QuadNameWidget(QWidget):

    def __init__(self, parent):
        super(QuadNameWidget, self).__init__(parent)
        self._parent = parent
        self._connected = False

        # load UI
        ui_file = os.path.join(
            os.path.dirname(os.path.realpath(__file__)),
            '../../resource/quad_name_widget.ui')
        loadUi(ui_file, self)

    def getQuadName(self):
        return self.namespace_text.text()

    def setQuadName(self, quadname):
        self.namespace_text.setText(quadname)

    @Slot(bool)
    def on_button_connect_clicked(self):
        if(self._connected):
            self.button_connect.setText('Connect')
            self._parent.disconnect()
            self._connected = False
        else:
            self.button_connect.setText('Disconnect')
            self._parent.connect()
            self._connected = True

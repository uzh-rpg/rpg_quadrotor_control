#!/usr/bin/env python
from python_qt_binding import loadUi
qt_version_below_5 = False
try:
    # Starting from Qt 5 QWidget is defined in QtWidgets and not QtGui anymore
    from python_qt_binding import QtWidgets
    from python_qt_binding.QtWidgets import QWidget
except:
    from python_qt_binding import QtGui
    from python_qt_binding.QtGui import QWidget
    qt_version_below_5 = True
from python_qt_binding import QtCore

from .quad_name_widget import QuadNameWidget


class QuadWidgetCommon(QWidget):
    def __init__(self):
        super(QuadWidgetCommon, self).__init__()

        # the name widget is separate since we need to access it directly
        self._name_widget = QuadNameWidget(self)
        if qt_version_below_5:
            self._column_1 = QtGui.QVBoxLayout()
            self._column_2 = QtGui.QVBoxLayout()
        else:
            self._column_1 = QtWidgets.QVBoxLayout()
            self._column_2 = QtWidgets.QVBoxLayout()

    def setup_gui(self, two_columns=True):
        if qt_version_below_5:
            widget_layout = QtGui.QHBoxLayout()
        else:
            widget_layout = QtWidgets.QHBoxLayout()
        widget_layout.addLayout(self._column_1)
        if two_columns:
            widget_layout.addLayout(self._column_2)

        if qt_version_below_5:
            main_layout = QtGui.QHBoxLayout()
        else:
            main_layout = QtWidgets.QHBoxLayout()
        main_layout = QtWidgets.QVBoxLayout()
        main_layout.addLayout(widget_layout)

        self._column_1.setAlignment(QtCore.Qt.AlignTop)
        if two_columns:
            self._column_2.setAlignment(QtCore.Qt.AlignTop)
        widget_layout.setAlignment(QtCore.Qt.AlignTop)
        main_layout.setAlignment(QtCore.Qt.AlignTop)

        self.setLayout(main_layout)

        self._update_info_timer = QtCore.QTimer(self)
        self._update_info_timer.timeout.connect(self.update_gui)
        self._update_info_timer.start(100)

    def get_list_of_plugins(self):
        quad_plugins = []

        for i in range(1, self._column_1.count()):
            quad_plugins.append(self._column_1.itemAt(i).widget())

        for i in range(0, self._column_2.count()):
            quad_plugins.append(self._column_2.itemAt(i).widget())
        return quad_plugins

    def connect(self):
        quad_name = self._name_widget.getQuadName()
        self.setWindowTitle(quad_name)
        for plugin in self.get_list_of_plugins():
            plugin.connect(quad_name)

    def disconnect(self):
        self.setWindowTitle("RPG Quad Gui")
        for plugin in self.get_list_of_plugins():
            plugin.disconnect()

    def update_gui(self):
        for plugin in self.get_list_of_plugins():
            plugin.update_gui()

    def getQuadName(self):
        return self._name_widget.getQuadName()

    def setQuadName(self, quadname):
        self._name_widget.setQuadName(quadname)

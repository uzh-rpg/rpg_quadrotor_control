#!/usr/bin/env python
import os
import rospy
import argparse
from qt_gui.plugin import Plugin


class GuiBase(Plugin):
    """
    Subclass of Plugin to display Quad status
    """
    def __init__(self, context, main_qt_widget):
        # Init Plugin
        super(GuiBase, self).__init__(context)  # Init Plugin superclass
        self.setObjectName('RPG Quad Gui Plugin')

        # Load arguments
        args = self._parse_args(context.argv())

        # Create QWidget
        self._widget = main_qt_widget

        # Set quad_name
        self._widget.setQuadName("/" + args.quad_name)

        # Show _widget.windowTitle on left-top of each plugin (when
        # it's set in _widget). This is useful when you open multiple
        # plugins at once. Also if you open multiple instances of your
        # plugin at once, these lines add number to make it easy to
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(
                'RPG Quad GUI' + (' (%d)' % context.serial_number()))
        else:
            self._widget.setWindowTitle('RPG Quad GUI')
        # Add widget to the user interface
        context.add_widget(self._widget)

    def _parse_args(self, argv):
        parser = argparse.ArgumentParser()
        parser.add_argument(
            '--quad_name', help='quadrotor name', default='quad_name')
        args, unknown = parser.parse_known_args()
        return args

    def shutdown_plugin(self):
        self._widget.disconnect()

    def save_settings(self, plugin_settings, instance_settings):
        # Save instance settings here
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # Restore instance settings here after gui has been newly launched
        pass

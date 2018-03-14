#!/usr/bin/env python
from .quad_widget_common import QuadWidgetCommon

from .autopilot_widget import AutopilotWidget


class BasicFlightWidget(QuadWidgetCommon):
    def __init__(self):
        super(BasicFlightWidget, self).__init__()

        self._column_1.addWidget(self._name_widget)
        self._column_1.addWidget(AutopilotWidget(self))

        self.setup_gui(two_columns=False)

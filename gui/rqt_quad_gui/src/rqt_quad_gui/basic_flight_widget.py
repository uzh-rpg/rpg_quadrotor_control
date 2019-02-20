#!/usr/bin/env python
from .quad_widget_common import QuadWidgetCommon

from .autopilot_widget import AutopilotWidget

from .cam_ctrl_widget import CameraControlWidget

class BasicFlightWidget(QuadWidgetCommon):
    def __init__(self):
        super(BasicFlightWidget, self).__init__()

        self._column_1.addWidget(self._name_widget)
        self._column_1.addWidget( AutopilotWidget(self) )
        
        self._column_2.addWidget( CameraControlWidget(self) )

        self.setup_gui(two_columns=False)

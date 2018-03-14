#!/usr/bin/env python
from .gui_base import GuiBase
from .basic_flight_widget import BasicFlightWidget


class BasicFlight(GuiBase):
    """
    Subclass of GuiBase to display Quad status
    """
    def __init__(self, context):
        # Create QWidget
        widget = BasicFlightWidget()

        # Init GuiBase
        super(BasicFlight, self).__init__(context, widget)

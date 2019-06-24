##############################################################################
# Copyright (C) 2019, UW REACT
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#   * Redistributions of source code must retain the above copyright notice,
#     this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above copyright
#     notice, this list of conditions and the following disclaimer in the
#     documentation and/or other materials provided with the distribution.
#   * Neither the name of UW REACT, nor the names of its
#     contributors may be used to endorse or promote products derived from
#     this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
##############################################################################

"""The StatusStringWidget class."""

# Standard imports
from enum import Enum

# ROS imports
from python_qt_binding import QtCore

# frc_control imports
from driver_station import structs


class StatusStrings(Enum):
    """The set of possible status strings to display."""
    NO_COMMS = 'No Robot\nCommunication'
    NO_CODE = 'No Robot\nCode'
    ESTOP = 'Emergency\nStopped'
    BROWNOUT = 'Voltage\nBrownout'
    TELEOP_DIS = 'Teleoperated\nDisabled'
    TELEOP_EN = 'Teleoperated\nEnabled'
    AUTO_DIS = 'Autonomous\nDisabled'
    AUTO_EN = 'Autonomous\nEnabled'
    TEST_DIS = 'Test\nDisabled'
    TEST_EN = 'Test\nEnabled'


class StatusStringWidget(object):
    """A widget to control the main status string."""

    def __init__(self, window, data):
        self.window = window
        self.data = data

        self.data.has_robot_comms.add_observer(self._update_string)
        self.data.has_robot_code.add_observer(self._update_string)
        self.data.brownout.add_observer(self._update_string)
        self.data.robot_mode.add_observer(self._update_string)
        self.data.enable_disable.add_observer(self._update_string)

        self._update_string()

    def blink(self):
        """Trigger the status string to flash three times."""

        # Dict hack since nonlocal doesn't exist in py2.7
        blinks = {'': 3}
        period = 150

        def _red_callback():
            self.window.statusStringDisplay.setStyleSheet('color: red')
            QtCore.QTimer.singleShot(period, _white_callback)

        def _white_callback():
            self.window.statusStringDisplay.setStyleSheet('color: white')
            blinks[''] -= 1
            if blinks[''] > 0:
                QtCore.QTimer.singleShot(period, _red_callback)

        _red_callback()

    def _update_string(self, _1=None, _2=None):
        """Update the displayed string."""

        if not self.data.has_robot_comms.get():
            self.window.statusStringDisplay.setText(StatusStrings.NO_COMMS.value)
        elif not self.data.has_robot_code.get():
            self.window.statusStringDisplay.setText(StatusStrings.NO_CODE.value)
        elif self.data.enable_disable.get() == structs.EnableDisableState.ESTOP:
            self.window.statusStringDisplay.setText(StatusStrings.ESTOP.value)
        elif self.data.brownout.get():
            self.window.statusStringDisplay.setText(StatusStrings.BROWNOUT.value)
        else:

            key = ''
            if self.data.robot_mode.get() == structs.RobotModeState.TELEOP:
                key += 'TELEOP'
            elif self.data.robot_mode.get() == structs.RobotModeState.AUTO:
                key += 'AUTO'
            elif self.data.robot_mode.get() == structs.RobotModeState.TEST:
                key += 'TEST'

            key += '_'

            if self.data.enable_disable.get() == structs.EnableDisableState.ENABLE:
                key += 'EN'
            else:
                key += 'DIS'

            self.window.statusStringDisplay.setText(getattr(StatusStrings, key).value)

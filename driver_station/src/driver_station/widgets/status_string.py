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

    def __init__(self, window):
        self.window = window

        self.has_robot_comms = False
        self.has_robot_code = False
        self.brownout = False
        self.robot_mode = structs.RobotModeState.TELEOP
        self.enable_disable = structs.EnableDisableState.DISABLE

        self._update_string()

    def set_robot_comms(self, has_robot_comms):
        """Set the robot communications state."""
        self.has_robot_comms = has_robot_comms
        self._update_string()

    def set_robot_code(self, has_robot_code):
        """Set the robot code state."""
        self.has_robot_code = has_robot_code
        self._update_string()

    def set_brownout(self, brownout):
        """Set the voltage brownout state."""
        self.brownout = brownout
        self._update_string()

    def set_robot_mode(self, mode):
        """Set the robot mode.

        `mode` should be a structs.RobotModeState enum value."""
        self.robot_mode = mode
        self._update_string()

    def set_enable_disable(self, mode):
        """Set whether the robot is enabled, disabled, or estopped.

        `mode` should be a structs.EnableDisableState enum value."""
        self.enable_disable = mode
        self._update_string()

    def _update_string(self):
        """Update the displayed string."""

        if not self.has_robot_comms:
            self.window.statusStringDisplay.setText(StatusStrings.NO_COMMS.value)
        elif not self.has_robot_code:
            self.window.statusStringDisplay.setText(StatusStrings.NO_CODE.value)
        if self.enable_disable == structs.EnableDisableState.ESTOP:
            self.window.statusStringDisplay.setText(StatusStrings.ESTOP.value)
        elif self.brownout:
            self.window.statusStringDisplay.setText(StatusStrings.BROWNOUT.value)
        else:

            key = ''
            if self.robot_mode == structs.RobotModeState.TELEOP:
                key += 'TELEOP'
            elif self.robot_mode == structs.RobotModeState.AUTO:
                key += 'AUTO'
            elif self.robot_mode == structs.RobotModeState.TEST:
                key += 'TEST'

            key += '_'

            if self.enable_disable == structs.EnableDisableState.ENABLE:
                key += 'EN'
            else:
                key += 'DIS'

            self.window.statusStringDisplay.setText(getattr(StatusStrings, key).value)

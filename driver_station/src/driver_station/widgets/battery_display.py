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

"""The BatteryDisplayWidget class."""


class BatteryDisplayWidget(object):
    """A widget to control the robot battery voltage."""

    def __init__(self, window, data):
        self.window = window
        self.data = data

        # Register callbacks
        self.data.robot_state.add_observer(self._update_voltage)
        self.data.has_robot_comms.add_observer(self._update_robot_comms)

        # Apply default value
        self.window.batteryVoltageDisplay.setText('--.--')

    def _update_voltage(self, _, robot_state):
        """Update the battery voltage."""
        self.window.batteryVoltageDisplay.setText('{:0.2f}'.format(robot_state.battery_voltage))

    def _update_robot_comms(self, _, has_comms):
        """Clear the battery voltage when comms are lost."""
        if not has_comms:
            self.window.batteryVoltageDisplay.setText('--.--')

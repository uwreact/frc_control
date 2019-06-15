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

"""The MajorStatusWidget class."""

# Standard imports
from functools import partial

# frc_control imports
from driver_station.utils import async_pinger
from driver_station.utils import gui_utils
from driver_station.utils import utils
from driver_station.utils import watchdog

HEARTBEAT_TIMEOUT = 0.5


class MajorStatusWidget(object):
    """A widget for displaying the major status indicators.

    This includes:
     - Communications: Whether the DS is currently communicating with the roboRIO
     - Robot Code: Whether the team Robot Code is currently running on the roboRIO
     - Joysticks: Whether at least one joystick is plugged in and detected by the DS

    Temporary: Also handles clearing the battery voltage once comms are lost.
    """

    def __init__(self, window):
        self.window = window
        self.init_ui()

        self.team_number = 0

        # Setup an async pinger to ping the roboRIO using mDNS (with a fallback of a static 10.TE.AM.2 address)
        self.robot_pinger = async_pinger.AsyncPinger()
        self.robot_pinger.set_hostnames(['roborio-0-frc.local', '10.0.0.2'])
        self.robot_pinger.signalConnectionStatus.connect(self._update_robot_comms)
        self.robot_pinger.start()

        # Setup a watchdog to indicate robot code status by listening to updates from the robot
        self.watchdog = watchdog.Watchdog()
        self.watchdog.set_timeout(HEARTBEAT_TIMEOUT)
        self.watchdog.timeoutExpired.connect(partial(self._has_robot_code, False))
        self.watchdog.watchdogFed.connect(partial(self._has_robot_code, True))
        self.watchdog.start()

    def init_ui(self):
        """Setup the UI elements."""

        gui_utils.bool_style(self.window.communicationsStatusDisplay, False, True)
        gui_utils.bool_style(self.window.robotCodeStatusDisplay, False, True)
        gui_utils.bool_style(self.window.joystickStatusDisplay, False, True)  # TODO: Check if num joysticks > 0

    def set_team_number(self, team_number):
        """Set the team number."""
        self.team_number = team_number
        upper = team_number / 100
        lower = team_number % 100
        self.robot_pinger.set_hostnames(['roborio-{}-frc.local'.format(team_number), '10.{}.{}.2'.format(upper, lower)])

    def _update_robot_comms(self, connected):
        """Update the connection indicator with the new state."""

        # Update the indicator
        gui_utils.bool_style(self.window.communicationsStatusDisplay, connected, True)

        # If there is a connection, fetch the image version from the roboRIO
        if connected:

            def _on_success(output):
                self.window.versions['RIO'] = output.split('IMAGEVERSION = ')[1].replace('"', '')

            def _on_failure(_error):
                self.window.versions['RIO'] = 'Unknown'

            utils.async_check_output([[
                'ssh', '-q', 'admin@roborio-{}-frc.local'.format(self.team_number), 'cat',
                '/etc/natinst/share/scs_imagemetadata.ini'
            ]], _on_success, _on_failure)

        else:
            if 'RIO' in self.window.versions:
                del self.window.versions['RIO']

        self.window.update_versions()
        self.window.batteryVoltageDisplay.setText('--.--')

    def _has_robot_code(self, has_code):
        """Update the robot code indicator with the new state."""
        gui_utils.bool_style(self.window.robotCodeStatusDisplay, has_code, True)

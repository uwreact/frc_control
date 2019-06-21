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

"""The RioUtilsWidget class."""

# frc_control imports
from driver_station.utils import utils


class RioUtilsWidget(object):
    """A widget for controlling the RIO."""

    def __init__(self, window):
        self.window = window

        # Connect reboot roboRIO and restart robot code buttons
        self.window.rebootRioButton.clicked.connect(self._reboot_rio)
        self.window.restartCodeButton.clicked.connect(self._restart_robot_code)

        # TODO: Flash status string when buttons are pressed and there are no comms or no robot code to indicate failure

    def _reboot_rio(self):
        utils.async_popen([['ssh', 'admin@roborio-{}-frc.local'.format(self.window.team_number), 'reboot']])

    def _restart_robot_code(self):
        utils.async_popen([[
            'ssh', 'admin@roborio-{}-frc.local'.format(self.window.team_number),
            '. /etc/profile.d/natinst-path.sh; /usr/local/frc/bin/frcKillRobot.sh -t -r'
        ]])

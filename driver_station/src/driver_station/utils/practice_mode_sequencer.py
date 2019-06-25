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

"""The PracticeModeSequencer class."""

# Standard imports
import time

# ROS imports
from python_qt_binding import QtCore

# frc_control imports
from driver_station import structs
from driver_station.utils import sound_player
from frc_msgs.msg import DriverStationMode


class PracticeModeSequencer(object):
    """Sequence DS Mode during Practice mode."""

    def __init__(self, window, data):
        self.window = window
        self.data = data

        self.timer = QtCore.QTimer(window)
        self.timer.timeout.connect(self._update)

        self.start_time = 0  # The time the current mode/stage began
        self.mode = -1  # The current mode/stage
        self.timings = structs.PracticeTiming()

    def start(self, period=0.1):
        """Start the sequencer with the specified update period."""
        self._set_mode(0)

        self.timer.start(period * 1000)
        self._update()
        if self.data.sound_enabled.get():
            sound_player.play_countdown_blip()

    def stop(self):
        """Stop the sequencer."""
        if self.timer.isActive() and self.data.sound_enabled.get():
            sound_player.play_match_pause()
        self.timer.stop()

    def set_timings(self, timings):
        """Set the duration of each stage of Practice mode."""
        self.timings = timings

    def _set_mode(self, new_mode):
        self.start_time = time.time()
        self.mode = new_mode

        # Play sound effect
        if self.data.sound_enabled.get():

            if self.mode == -1:  # End of match
                sound_player.play_match_end()
            elif self.mode == 1 and self.timings.autonomous != 0:  # Autonomous
                sound_player.play_start_auto()
            elif self.mode == 3 and self.timings.teleop != 0:  # Teleoperated
                sound_player.play_start_teleop()
            elif self.mode == 4 and self.timings.endgame != 0:  # Endgame
                sound_player.play_start_endgame()

    def _update(self):
        # pylint: disable=too-many-branches

        prev_time = self.data.match_time.get().remaining_time
        remaining_time = None
        ds_mode = None

        # Countdown
        if self.mode == 0:
            self.data.robot_mode.set(structs.RobotModeState.AUTO)
            self.data.enable_disable.set(structs.EnableDisableState.DISABLE)
            ds_mode = DriverStationMode.MODE_DISABLED
            remaining_time = self.timings.countdown - (time.time() - self.start_time)
            if remaining_time < 0:
                self._set_mode(1)

            # Play countdown sounds
            if int(prev_time) > int(remaining_time) and self.data.sound_enabled.get():
                sound_player.play_countdown_blip()

        # Autonomous
        if self.mode == 1:
            self.data.robot_mode.set(structs.RobotModeState.AUTO)
            self.data.enable_disable.set(structs.EnableDisableState.ENABLE)
            ds_mode = DriverStationMode.MODE_AUTONOMOUS
            remaining_time = self.timings.autonomous - (time.time() - self.start_time)
            if remaining_time < 0:
                self._set_mode(2)

        # Delay
        if self.mode == 2:
            self.data.robot_mode.set(structs.RobotModeState.TELEOP)
            self.data.enable_disable.set(structs.EnableDisableState.DISABLE)
            ds_mode = DriverStationMode.MODE_DISABLED
            remaining_time = self.timings.delay - (time.time() - self.start_time)
            if remaining_time < 0:
                self._set_mode(3)

        # Teleop
        if self.mode == 3:
            self.data.robot_mode.set(structs.RobotModeState.TELEOP)
            self.data.enable_disable.set(structs.EnableDisableState.ENABLE)
            ds_mode = DriverStationMode.MODE_OPERATOR
            remaining_time = self.timings.teleop - (time.time() - self.start_time)
            if remaining_time < 0:
                self._set_mode(4)

        # Endgame
        if self.mode == 4:
            self.data.robot_mode.set(structs.RobotModeState.TELEOP)
            self.data.enable_disable.set(structs.EnableDisableState.ENABLE)
            ds_mode = DriverStationMode.MODE_OPERATOR
            remaining_time = self.timings.endgame - (time.time() - self.start_time)
            if remaining_time < 0:
                self._set_mode(-1)
                self.timer.stop()

                self.data.robot_mode.set(structs.RobotModeState.TELEOP)
                self.data.enable_disable.set(structs.EnableDisableState.DISABLE)
                ds_mode = DriverStationMode.MODE_DISABLED

        self.data.ds_mode.set_attr('mode', ds_mode)
        if self.mode == 3:
            self.data.match_time.set_attr('remaining_time', remaining_time + self.timings.endgame)
        else:
            self.data.match_time.set_attr('remaining_time', remaining_time)

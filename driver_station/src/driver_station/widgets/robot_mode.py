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

"""The RobotModeWidget class."""

# Standard imports
import time

# ROS imports
from python_qt_binding import QtCore

# frc_control imports
from driver_station import structs
from driver_station.utils import practice_mode_sequencer
from frc_msgs.msg import DriverStationMode


class RobotModeWidget(object):
    """A widget to control the robot mode."""

    def __init__(self, window, data):
        self.window = window
        self.data = data
        self.init_ui()

        self.start_time = 0
        self.robot_mode = structs.RobotModeState.TELEOP
        self.enable_disable = structs.EnableDisableState.DISABLE

        self.practice_sequencer = practice_mode_sequencer.PracticeModeSequencer(self.window, self.data)
        self.elapsed_timer = QtCore.QTimer(self.window)
        self.elapsed_timer.timeout.connect(self._update_elapsed_time)

    def init_ui(self):
        """Setup the UI elements."""

        # Assign an ID to every button
        self.window.enableDisableGroup.setId(self.window.enableRobotButton, structs.EnableDisableState.ENABLE)
        self.window.enableDisableGroup.setId(self.window.disableRobotButton, structs.EnableDisableState.DISABLE)
        self.window.enableDisableGroup.setId(self.window.estopRobotButton, structs.EnableDisableState.ESTOP)
        self.window.robotModeGroup.setId(self.window.teleopModeButton, structs.RobotModeState.TELEOP)
        self.window.robotModeGroup.setId(self.window.autoModeButton, structs.RobotModeState.AUTO)
        self.window.robotModeGroup.setId(self.window.practiceModeButton, structs.RobotModeState.PRACTICE)
        self.window.robotModeGroup.setId(self.window.testModeButton, structs.RobotModeState.TEST)

        # Connect button click signals to update functions
        self.window.enableDisableGroup.buttonClicked.connect(self._update_ds_mode)
        self.window.robotModeGroup.buttonClicked.connect(self._update_robot_mode)

    def _update_robot_mode(self):
        new_mode = self.window.robotModeGroup.checkedId()

        # Mode unchanged, return
        if new_mode == self.robot_mode:
            return

        # Set the mode, and press the disable button
        self.robot_mode = new_mode
        self.window.disableRobotButton.click()
        if self.robot_mode == structs.RobotModeState.PRACTICE:
            self.data.robot_mode.set(structs.RobotModeState.AUTO)
        else:
            self.data.robot_mode.set(self.robot_mode)

    def _update_ds_mode(self):
        enable_disable = self.window.enableDisableGroup.checkedId()

        # If mode unchanged, return
        if enable_disable == self.enable_disable:
            return

        # Set the mode
        self.enable_disable = enable_disable
        self.data.enable_disable.set(self.enable_disable)

        # Estop
        if enable_disable == structs.EnableDisableState.ESTOP:
            self.data.ds_mode.set_attr('mode', DriverStationMode.MODE_ESTOP)
            self.elapsed_timer.stop()
            self.practice_sequencer.stop()
            self.data.match_time.set_attr('remaining_time', 0)

        # Disabled
        elif enable_disable == structs.EnableDisableState.DISABLE:
            self.data.ds_mode.set_attr('mode', DriverStationMode.MODE_DISABLED)
            self.elapsed_timer.stop()
            self.practice_sequencer.stop()
            self.data.match_time.set_attr('remaining_time', 0)

        # Teleop
        elif self.robot_mode == structs.RobotModeState.TELEOP:
            self.data.ds_mode.set_attr('mode', DriverStationMode.MODE_OPERATOR)
            self.start_time = time.time()
            self.elapsed_timer.start()

        # Autonomous
        elif self.robot_mode == structs.RobotModeState.AUTO:
            self.data.ds_mode.set_attr('mode', DriverStationMode.MODE_AUTONOMOUS)
            self.start_time = time.time()
            self.elapsed_timer.start()

        # Test
        elif self.robot_mode == structs.RobotModeState.TEST:
            self.data.ds_mode.set_attr('mode', DriverStationMode.MODE_TEST)
            self.start_time = time.time()
            self.elapsed_timer.start()

        # Practice
        elif self.robot_mode == structs.RobotModeState.PRACTICE:
            self.practice_sequencer.set_timings(self.data.practice_timing.get())
            self.practice_sequencer.start()

    def _update_elapsed_time(self):
        self.data.match_time.set_attr('remaining_time', time.time() - self.start_time)

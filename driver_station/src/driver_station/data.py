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

"""The MainData class."""

# frc_control imports
from driver_station import structs
from driver_station.utils.observable import ObservableData, ObservableDict, ObservableObj
from frc_msgs.msg import DriverStationMode
from frc_msgs.msg import JoyArray
from frc_msgs.msg import JoyFeedback
from frc_msgs.msg import MatchData
from frc_msgs.msg import MatchTime
from frc_msgs.msg import RobotState


class MainData(object):
    """The main application data."""

    # pylint: disable=too-many-instance-attributes

    def __init__(self):

        # User-inputted data
        self.sound_enabled = ObservableData(False)
        self.team_number = ObservableData(0)
        self.practice_timing = ObservableObj(structs.PracticeTiming())

        # Diagnostic information
        self.versions = ObservableDict()

        # ROS messages
        self.ds_mode = ObservableObj(DriverStationMode())
        self.joys = ObservableObj(JoyArray())
        self.joy_feedback = ObservableObj(JoyFeedback())
        self.match_data = ObservableObj(MatchData())
        self.match_time = ObservableObj(MatchTime())
        self.robot_state = ObservableObj(RobotState())

        # Internal state variables
        self.joystick_mappings = ObservableDict()
        self.selected_joystick = ObservableData(-1)
        self.has_robot_comms = ObservableData(False)
        self.has_robot_code = ObservableData(False)
        self.robot_mode = ObservableData(structs.RobotModeState.TELEOP)
        self.enable_disable = ObservableData(structs.EnableDisableState.DISABLE)

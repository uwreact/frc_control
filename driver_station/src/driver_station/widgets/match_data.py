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

"""The MatchDataWidget class."""

# Standard imports
from locale import atoi

# ROS imports
from python_qt_binding import QtGui

# frc_control imports
from driver_station.utils import gui_utils
from frc_msgs.msg import MatchData


class MatchDataWidget(object):
    """A widget for controlling the match data.

    This includes data such as match number, event name, game-specific message,
    alliance, and player station/driver position.
    """

    def __init__(self, window, match_data):
        self.window = window
        self.init_ui()

        self.match_data = match_data

        # Load default values
        self._update_game_data_string()
        self._update_team_station()
        self._update_match_type_box()

    def init_ui(self):
        """Setup the UI elements."""

        # Setup the replay and match number input
        gui_utils.setup_int_validator(self.window.replayNumberInput, lambda text: str(self.match_data.replay_number))
        gui_utils.setup_int_validator(self.window.matchNumberInput, lambda text: str(self.match_data.match_number))

        self.window.gameDataInput.editingFinished.connect(self._update_game_data_string)
        self.window.eventNameInput.editingFinished.connect(self._update_event_name)
        self.window.replayNumberInput.editingFinished.connect(self._update_replay_number)
        self.window.matchNumberInput.editingFinished.connect(self._update_match_number)
        self.window.teamStationInput.activated.connect(self._update_team_station)
        self.window.matchTypeInput.activated.connect(self._update_match_type_box)

    def _update_game_data_string(self):
        self.match_data.game_specific_message = self.window.gameDataInput.text()

    def _update_event_name(self):
        self.match_data.event_name = self.window.eventNameInput.text()

    def _update_match_number(self):
        self.match_data.match_number = atoi(self.window.matchNumberInput.text())

    def _update_replay_number(self):
        self.match_data.replay_number = atoi(self.window.replayNumberInput.text())

    def _update_match_type_box(self):
        index = self.window.matchTypeInput.currentIndex()
        self.match_data.match_type = index

    def _update_team_station(self):
        # Combo Box Indices:
        # 0: Red 1
        # 1: Red 2
        # 2: Red 3
        # 3: Blue 1
        # 4: Blue 2
        # 5: Blue 3

        index = self.window.teamStationInput.currentIndex()
        if index < 3:
            self.match_data.location = (index % 3) + 1
            self.match_data.alliance = MatchData.ALLIANCE_RED
        elif index < 6:
            self.match_data.location = (index % 3) + 1
            self.match_data.alliance = MatchData.ALLIANCE_BLUE
        else:
            self.match_data.location = MatchData.LOCATION_INVALID
            self.match_data.alliance = MatchData.ALLIANCE_INVALID

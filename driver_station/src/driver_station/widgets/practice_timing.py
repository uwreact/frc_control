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

"""The PracticeTimingWidget class."""

# Standard imports
from locale import atoi

# ROS imports
from driver_station import structs
from driver_station.utils import gui_utils


class PracticeTimingWidget(object):
    """A widget to control the duration of each stage of practice mode."""

    def __init__(self, window):
        self.window = window
        self.init_ui()

        self.timing = structs.PracticeTiming()
        self._update_timing()

    def init_ui(self):
        """Setup the UI elements."""

        # Setup the timing input validators
        gui_utils.setup_int_validator(self.window.countdownTimingInput, lambda text: str(self.timing.countdown))
        gui_utils.setup_int_validator(self.window.autonomousTimingInput, lambda text: str(self.timing.autonomous))
        gui_utils.setup_int_validator(self.window.delayTimingInput, lambda text: str(self.timing.delay))
        gui_utils.setup_int_validator(self.window.teleoperatedTimingInput, lambda text: str(self.timing.teleop))
        gui_utils.setup_int_validator(self.window.endgameTimingInput, lambda text: str(self.timing.endgame))

        # Setup the callbacks
        self.window.countdownTimingInput.editingFinished.connect(self._update_timing)
        self.window.autonomousTimingInput.editingFinished.connect(self._update_timing)
        self.window.delayTimingInput.editingFinished.connect(self._update_timing)
        self.window.teleoperatedTimingInput.editingFinished.connect(self._update_timing)
        self.window.endgameTimingInput.editingFinished.connect(self._update_timing)

        self.window.enableSoundButton.clicked.connect(self._enable_sound)

    def _update_timing(self):
        self.timing.countdown = atoi(self.window.countdownTimingInput.text())
        self.timing.autonomous = atoi(self.window.autonomousTimingInput.text())
        self.timing.delay = atoi(self.window.delayTimingInput.text())
        self.timing.teleop = atoi(self.window.teleoperatedTimingInput.text())
        self.timing.endgame = atoi(self.window.endgameTimingInput.text())

    def _enable_sound(self, enabled):
        #TODO: Move elsewhere?
        self.window.sound_enabled = enabled

    def get_timing(self):
        """Get the practice mode timing struct."""
        return self.timing

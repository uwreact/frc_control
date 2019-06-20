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

"""Audio player for Driver Station sound effects."""

from __future__ import print_function

# ROS imports
try:
    from python_qt_binding import QtMultimedia
    SOUND_SUPPORTED = True
except ImportError:
    SOUND_SUPPORTED = False

# frc_control imports
from driver_station.utils import utils


def play_countdown_blip():
    """Play a brief blip sound effect."""
    if not SOUND_SUPPORTED:
        return
    # TODO: Find/create sound
    print('Blip')


def play_disable():
    """Play the Robot Disabled sound effect."""
    if not SOUND_SUPPORTED:
        return
    # TODO: Find/create sound
    print('Beep boop')


def play_match_end():
    """Play the Match End sound effect."""
    if not SOUND_SUPPORTED:
        return
    QtMultimedia.QSound.play(utils.load_resource('sounds/Match End_normalized.wav'))


def play_match_pause():
    """Play the Match Pause (aka Field Fault) sound effect."""
    if not SOUND_SUPPORTED:
        return
    QtMultimedia.QSound.play(utils.load_resource('sounds/Match Pause_normalized.wav'))


def play_start_auto():
    """Play the Start Auto sound effect."""
    if not SOUND_SUPPORTED:
        return
    QtMultimedia.QSound.play(utils.load_resource('sounds/Start Auto_normalized.wav'))


def play_start_endgame():
    """Play the Start Endgame sound effect."""
    if not SOUND_SUPPORTED:
        return
    QtMultimedia.QSound.play(utils.load_resource('sounds/Start of End Game_normalized.wav'))


def play_start_teleop():
    """Play the Start Teleop sound effect."""
    if not SOUND_SUPPORTED:
        return
    QtMultimedia.QSound.play(utils.load_resource('sounds/Start Teleop_normalized.wav'))

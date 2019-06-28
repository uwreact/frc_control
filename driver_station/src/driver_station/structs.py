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

"""Various POD data structures."""

from enum import IntEnum


class EnableDisableState(IntEnum):
    """The enabled/disabled state of the robot."""
    ESTOP = 0
    ENABLE = 1
    DISABLE = 2


class RobotModeState(IntEnum):
    """The mode of the robot."""
    TELEOP = 0
    AUTO = 1
    PRACTICE = 2
    TEST = 3


class PracticeTiming(object):
    """Data structure containing the durations of each stage of practice mode."""

    # TODO: Change to a dict to be more pythonic? Or wait for py3.7 @dataclass?

    def __init__(self):
        self.countdown = 3
        self.autonomous = 15
        self.delay = 1
        self.teleop = 100
        self.endgame = 20


class JoystickInfo(object):
    """Data structure containing the durations of each stage of practice mode."""

    # TODO: Change to a dict to be more pythonic? Or wait for py3.7 @dataclass?

    def __init__(self):
        self.index = 0
        self.name = 1
        self.locked = False
        self.axis_names = []
        self.btn_names = []

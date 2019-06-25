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

"""The JoystickIndicatorWidget class."""

# ROS imports
from sensor_msgs.msg import Joy

# frc_control imports
from driver_station.utils import gui_utils
from frc_msgs.msg import JoyArray


class JoystickIndicatorWidget(object):
    """A widget to display the joystick state"""

    def __init__(self, window):
        self.window = window

        empty_joy = Joy()
        self._update_state(empty_joy)

        # TODO: Delete me
        test = Joy()
        test.axes.append(0.1)
        test.axes.append(-0.5)
        test.axes.append(-0.9)
        test.axes.append(0.8)
        test.buttons.append(1)
        test.buttons.append(1)
        test.buttons.append(0)
        test.buttons.append(0)
        test.buttons.append(1)
        test.buttons.append(0)
        test.buttons.append(0)
        test.buttons.append(1)
        test.buttons.append(1)
        test.buttons.append(0)
        self._update_state(test)

    def _update_state(self, joystick):

        # TODO: Show/hide buttons and axis based on length

        for i, axis in enumerate(joystick.axes):
            if i > JoyArray.MAX_JOYSTICK_AXES:
                break

            try:
                axis_display = getattr(self.window, 'axis{}Display'.format(i))
            except AttributeError:
                break

            axis_display.setValue(axis * 100)

        for i, btn in enumerate(joystick.buttons):
            if i > JoyArray.MAX_JOYSTICK_BUTTONS:
                break

            try:
                btn_display = getattr(self.window, 'button{}Display'.format(i))
            except AttributeError:
                break

            gui_utils.bool_style(btn_display, btn > 0)

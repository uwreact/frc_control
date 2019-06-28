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
from python_qt_binding import QtCore
from sensor_msgs.msg import Joy

# frc_control imports
from driver_station.utils import gui_utils
# from frc_msgs.msg import JoyArray


class JoystickIndicatorWidget(object):
    """A widget to display the joystick state."""

    def __init__(self, window, data):
        self.window = window
        self.data = data

        self.joy_state = None

        self.timer = QtCore.QTimer(window)
        self.timer.timeout.connect(self._update)
        self.timer.start(10)  # 100Hz
        self._clear()

    def _clear(self):
        empty_joy = Joy()
        self._update_state(empty_joy)

    def _update(self):
        joy_array = self.data.joys.get()
        idx = self.data.selected_joystick.get()

        if idx < 0 or len(joy_array.sticks) <= idx:
            self._clear()
            return

        joy_state = joy_array.sticks[idx]

        if self.joy_state == joy_state:
            return

        self._update_state(joy_state)

    def _update_state(self, joy_state):
        self.joy_state = joy_state

        # TODO: Support up to JoyArray.MAX_JOYSTICK_XXX indicators?
        max_axes = 6  #JoyArray.MAX_JOYSTICK_AXES
        max_btns = 16  #JoyArray.MAX_JOYSTICK_BUTTONS

        joy_state.axes = joy_state.axes[:max_axes]
        joy_state.buttons = joy_state.buttons[:max_btns]

        stick_mappings = self.data.joystick_mappings.get_all()
        info = None
        for mapping in stick_mappings.values():
            if mapping.index == self.data.selected_joystick.get():
                info = mapping

        for i, axis in enumerate(joy_state.axes):
            axis_display = getattr(self.window, 'axis{}Display'.format(i))
            if info is not None:
                axis_display.setFormat('{}: {}'.format(i, info.axis_names[i]))
            axis_display.setVisible(True)
            axis_display.setValue(axis * 100)

        for i in range(len(joy_state.axes), max_axes):
            axis_display = getattr(self.window, 'axis{}Display'.format(i))
            axis_display.setVisible(False)

        for i, btn in enumerate(joy_state.buttons):
            btn_display = getattr(self.window, 'button{}Display'.format(i))
            if info is not None:
                btn_display.setToolTip('{}: {}'.format(i, info.btn_names[i]))
            btn_display.setVisible(True)
            gui_utils.bool_style(btn_display, btn > 0)

        for i in range(len(joy_state.buttons), max_btns):
            btn_display = getattr(self.window, 'button{}Display'.format(i))
            btn_display.setVisible(False)

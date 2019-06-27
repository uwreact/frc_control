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

"""The JoystickSelectorWidget class."""

# ROS imports
from python_qt_binding import QtCore
from python_qt_binding import QtGui
from python_qt_binding import QtWidgets

# frc_control imports
from driver_station.utils import gui_utils


class JoystickSelectorWidget(object):
    """A widget to reorder joysticks."""

    def __init__(self, window, data):
        self.window = window
        self.data = data

        self.init_ui()

        self.stick_names = []
        self.data.joystick_mappings.add_observer(self._update_sticks)
        self.data.joystick_mappings.force_notify()

    def init_ui(self):
        """Setup the UI elements."""

        self.window.usbSetupList.currentRowChanged.connect(self._update_selection)
        self.window.usbSetupList.itemDoubleClicked.connect(self._toggle_locked)
        self.window.usbSetupList.model().rowsMoved.connect(self._update_rows)
        self.window.usbSetupList.setDragDropMode(QtWidgets.QAbstractItemView.InternalMove)

        self.window.rescanUsbButton.clicked.connect(self.window.joys.rescan)

    def _update_rows(self, _1, source, _2, _3, dest):

        # Update joystick mappings
        if dest < source:
            for i in range(dest, source + 1):
                uuid = self.window.usbSetupList.item(i).data(QtCore.Qt.UserRole)
                if uuid is None:
                    self.window.usbSetupList.item(i).setText('{} ----'.format(i))
                else:
                    data = list(self.data.joystick_mappings.get(uuid))
                    data[0] = i
                    if i == dest:
                        data[2] = True

                    self.data.joystick_mappings.set(uuid, data)
                    gui_utils.set_underlined(self.window.usbSetupList.item(i), data[2])
                    self.window.usbSetupList.item(i).setText('{} {}'.format(i, data[1]))

        if dest > source:
            for i in range(source, dest):
                uuid = self.window.usbSetupList.item(i).data(QtCore.Qt.UserRole)

                if uuid is None:
                    self.window.usbSetupList.item(i).setText('{} ----'.format(i))
                else:
                    data = list(self.data.joystick_mappings.get(uuid))
                    data[0] = i
                    if i == dest - 1:
                        data[2] = True

                    self.data.joystick_mappings.set(uuid, data)
                    gui_utils.set_underlined(self.window.usbSetupList.item(i), data[2])
                    self.window.usbSetupList.item(i).setText('{} {}'.format(i, data[1]))

        self.data.selected_joystick.set(self.window.usbSetupList.currentRow())

    def _toggle_locked(self, item):
        uuid = item.data(QtCore.Qt.UserRole)
        if uuid is None:
            return

        data = list(self.data.joystick_mappings.get(uuid))
        data[2] = not data[2]
        self.data.joystick_mappings.set(uuid, data)
        gui_utils.set_underlined(item, data[2])

    def _update_sticks(self, _, sticks):
        empty_sticks = range(0, 6)

        # Display the active and locked sticks
        for uuid, data in sticks.items():
            idx = data[0]
            empty_sticks.remove(idx)
            name = data[1]
            locked = data[2]
            gui_utils.set_underlined(self.window.usbSetupList.item(idx), locked)
            self.window.usbSetupList.item(idx).setText('{} {}'.format(idx, name))
            self.window.usbSetupList.item(idx).setData(QtCore.Qt.UserRole, uuid)

        # Clear out the empty sticks
        for i in empty_sticks:
            gui_utils.set_underlined(self.window.usbSetupList.item(i), False)
            self.window.usbSetupList.item(i).setText('{} ----'.format(i))
            self.window.usbSetupList.item(i).setData(QtCore.Qt.UserRole, None)

        self.data.selected_joystick.set(self.window.usbSetupList.currentRow())

    def _update_selection(self, row):
        self.data.selected_joystick.set(row)

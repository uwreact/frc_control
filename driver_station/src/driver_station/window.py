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

"""TODO: Module docstring."""

# ROS imports
from python_qt_binding import loadUi as loadLayout
from python_qt_binding import QtGui
from python_qt_binding import QtCore
from python_qt_binding import QtWidgets

# frc_control imports
from driver_station.utils import utils


class MainWindow(QtWidgets.QMainWindow):
    """Main window for the visualizer."""

    def __init__(self, *args, **kwargs):
        super(MainWindow, self).__init__(*args, **kwargs)
        self.setWindowFlags(QtCore.Qt.WindowStaysOnTopHint)

        # Setup the UI
        self.init_ui()

    def init_ui(self):
        """Setup the UI elements."""

        # Load the layout
        ui_file = utils.load_resource('layout/driver-station.ui')
        loadLayout(ui_file, self)

        # Load the stylesheet
        stylesheet = '\n'.join([
            open(utils.load_resource('stylesheet/default.qss')).read(),
            open(utils.load_resource('stylesheet/operations.qss')).read(),
            open(utils.load_resource('stylesheet/setup.qss')).read(),
            open(utils.load_resource('stylesheet/tabs.qss')).read()
        ])
        self.setStyleSheet(stylesheet)

        # Set the icon and title
        icon = utils.load_resource('icon.svg')
        self.setWindowIcon(QtGui.QIcon(icon))
        self.setWindowTitle('Driver Station')

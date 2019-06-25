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

"""The MainWindow class."""

# Standard imports
from locale import atoi

# ROS imports
import rospy
import rospkg
from python_qt_binding import loadUi as loadLayout
from python_qt_binding import QtGui
from python_qt_binding import QtCore
from python_qt_binding import QtWidgets

# frc_control imports
from driver_station import data
from driver_station import joysticks
from driver_station import publisher
from driver_station import subscriber
from driver_station.utils import gui_utils
from driver_station.utils import utils
from driver_station.widgets import battery_display
from driver_station.widgets import communications
from driver_station.widgets import joystick_indicator
from driver_station.widgets import major_status
from driver_station.widgets import match_data
from driver_station.widgets import metrics
from driver_station.widgets import pc_stats
from driver_station.widgets import practice_timing
from driver_station.widgets import rio_utils
from driver_station.widgets import robot_mode
from driver_station.widgets import status_string
from driver_station.widgets import time_display


class MainWindow(QtWidgets.QMainWindow):
    """Main window for the visualizer."""

    # pylint: disable=too-many-instance-attributes

    def __init__(self, *args, **kwargs):
        super(MainWindow, self).__init__(*args, **kwargs)
        self.setWindowFlags(QtCore.Qt.WindowStaysOnTopHint)

        self.data = data.MainData()
        self.pub = publisher.Publisher(self.data)
        self.sub = subscriber.Subscriber(self.data)
        self.joys = joysticks.JoystickUpdater(self.data)

        # Start the publisher and joystick threads
        self.pub.start()
        self.joys.start()

        # Setup the UI
        self.init_ui()

        # Setup all the inner widgets
        self.battery_display = battery_display.BatteryDisplayWidget(self, self.data)
        self.communications = communications.CommunicationsWidget(self, self.data)
        self.joystick_indicator = joystick_indicator.JoystickIndicatorWidget(self)
        self.major_status = major_status.MajorStatusWidget(self, self.data)
        self.match_data = match_data.MatchDataWidget(self, self.data)
        self.metrics = metrics.MetricsWidget(self, self.data)
        self.pc_stats = pc_stats.PcStatsWidget(self)
        self.practice_timing = practice_timing.PracticeTimingWidget(self, self.data)
        self.rio_utils = rio_utils.RioUtilsWidget(self, self.data)
        self.robot_mode = robot_mode.RobotModeWidget(self, self.data)
        self.status_string = status_string.StatusStringWidget(self, self.data)
        self.time_display = time_display.TimeDisplayWidget(self, self.data)

        # Feed the Robot Code indicator's watchdog whenever the subscriber receives data from the robot
        self.sub.add_robot_state_callback(self.major_status.watchdog.feed)

        # Register callbacks
        self.data.versions.add_observer(self.update_versions)
        self.data.versions.force_notify(self.data.versions.get_all())

        # Set initial values
        # TODO: Load previous values from save file
        self.data.sound_enabled.set(False)
        self.data.team_number.set(0)
        self.data.versions.set('DS', rospkg.RosPack().get_manifest('driver_station').version)
        self.data.versions.set(
            'ROS',
            '{} {}'.format(rospy.get_param('rosdistro').strip().capitalize(),
                           rospy.get_param('rosversion').strip()))

        # Display initial values
        self._setup_team_number()

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

        # Load tab icons
        operations_icon = QtGui.QIcon(utils.load_resource('icons/baseline-gamepad-24px.svg'))
        self.leftTabWidget.setTabIcon(0, operations_icon)
        diagnostics_icon = QtGui.QIcon(utils.load_resource('icons/baseline-timeline-24px.svg'))
        self.leftTabWidget.setTabIcon(1, diagnostics_icon)
        settings_icon = QtGui.QIcon(utils.load_resource('icons/baseline-settings-20px.svg'))
        self.leftTabWidget.setTabIcon(2, settings_icon)
        usb_icon = QtGui.QIcon(utils.load_resource('icons/baseline-usb-24px.svg'))
        self.leftTabWidget.setTabIcon(3, usb_icon)
        power_icon = QtGui.QIcon(utils.load_resource('icons/baseline-flash_on-24px.svg'))
        self.leftTabWidget.setTabIcon(4, power_icon)

        log_icon = QtGui.QIcon(utils.load_resource('icons/baseline-mail-24px.svg'))
        self.rightTabWidget.setTabIcon(0, log_icon)
        fms_icon = QtGui.QIcon(utils.load_resource('icons/baseline-router-24px.svg'))
        self.rightTabWidget.setTabIcon(1, fms_icon)

        # Set the icon and title
        icon = utils.load_resource('icon.svg')
        self.setWindowIcon(QtGui.QIcon(icon))
        self.setWindowTitle('Driver Station')

    def _setup_team_number(self):
        """Setup the team number input."""

        # Setup the team number input box
        gui_utils.setup_int_validator(self.teamNumberInput, lambda text: str(self.data.team_number.get()))
        self.teamNumberInput.setMaxLength(4)
        self.teamNumberInput.editingFinished.connect(self._update_team_number)

        # If a default team number is set, apply it
        if self.data.team_number.get() > 0:
            self.teamNumberInput.setText(str(self.data.team_number.get()))
            self._update_team_number(True)

    def _update_team_number(self, force_update=False):
        """Parse the value of the team numer input."""

        if not force_update and atoi(self.teamNumberInput.text()) == self.data.team_number.get():
            return

        new_val = self.teamNumberInput.text()
        self.teamNumberDisplay.setText(new_val)
        self.data.team_number.set(atoi(new_val))

    def update_versions(self, _, versions):
        """Update the Version Information panel."""
        text = '\n'.join(['{}: {}'.format(k, versions[k]) for k in sorted(versions)])
        self.versionsTextDisplay.setText(text)

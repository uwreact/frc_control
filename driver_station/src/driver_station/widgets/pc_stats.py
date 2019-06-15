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

"""The PcStatsWidget class."""

# Standard imports
from collections import deque
from locale import atoi

# ROS imports
from python_qt_binding.QtCore import QTimer

# frc_control imports
from driver_station.utils import gui_utils


class PcStatsWidget(object):
    """Widget for displaying the PC's battery and CPU usage."""

    def __init__(self, window):
        self.window = window
        self.init_ui()

        self.cpu_last_idle = 0
        self.cpu_last_total = 0
        self.cpu_buffer = deque(maxlen=100)

        self.pc_timer = QTimer(self.window)
        self.pc_timer.timeout.connect(self._update)

        self.start_periodic()

    def init_ui(self):
        """Setup the UI elements."""

        # Display default values until first update
        self.window.pcBatteryDisplay.setValue(100)
        self.window.pcCpuDisplay.setValue(0)

    def start_periodic(self, period=100):
        """Start timer to read PC battery & CPU usage."""

        self.pc_timer.setInterval(period)
        self.pc_timer.start()
        self._update()

    def _update(self):
        """Update the PC battery & CPU usage."""

        # Read the power supply status
        # TODO: Display power icon while charging
        plugged_in = open('/sys/class/power_supply/AC0/online').readline().strip()  # pylint: disable=unused-variable
        power_percent = atoi(open('/sys/class/power_supply/BAT0/capacity').readline().strip())

        self.window.pcBatteryDisplay.setValue(power_percent)

        # Set color based on power_level
        # TODO: Check thresholds against real driver station
        if power_percent < 20:
            self.window.pcBatteryDisplay.setStyleSheet('QProgressBar::chunk {{background-color: #{:06x}}}'.format(
                gui_utils.Color.RED))
        elif power_percent < 80:
            self.window.pcBatteryDisplay.setStyleSheet('QProgressBar::chunk {{background-color: #{:06x}}}'.format(
                gui_utils.Color.ORANGE))
        else:
            self.window.pcBatteryDisplay.setStyleSheet('QProgressBar::chunk {{background-color: #{:06x}}}'.format(
                gui_utils.Color.BAR_GREEN))

        # Compute the CPU usage
        with open('/proc/stat') as f:

            # Parse the data from the file
            fields = [float(column) for column in f.readline().strip().split()[1:]]
            idle, total = fields[3], sum(fields)
            idle_delta = idle - self.cpu_last_idle
            total_delta = total - self.cpu_last_total
            self.cpu_last_idle = idle
            self.cpu_last_total = total

            # Calulate the utilisation
            utilisation = 100.0 * (1.0 - idle_delta / total_delta)
            self.cpu_buffer.append(utilisation)

        self.window.pcCpuDisplay.setValue(sum(self.cpu_buffer) / len(self.cpu_buffer))

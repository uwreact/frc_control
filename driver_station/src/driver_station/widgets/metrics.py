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

"""The MetricsWidget class."""


class MetricsWidget(object):
    """A widget to control the power, faults, and CAN metrics."""

    def __init__(self, window, data):
        self.window = window
        self.data = data

        # Register callback
        self.data.robot_state.add_observer(self._update)

    def _update(self, _, robot_state):
        """Update the metrics."""

        # Faults
        # self.window.faultsCommsDisplay.setText('?')
        # self.window.faults12vDisplay.setText('?')
        self.window.faults6vDisplay.setText(str(robot_state.fault_count_6v))
        self.window.faults5vDisplay.setText(str(robot_state.fault_count_5v))
        self.window.faults3v3Display.setText(str(robot_state.fault_count_3v3))

        # CAN Metrics
        percent_bus_utilization = int(round(robot_state.can_status.percent_bus_utilization * 100))
        self.window.metricsUtilizationDisplay.setText(str(percent_bus_utilization))
        self.window.metricsBusOffDisplay.setText(str(robot_state.can_status.bus_off_count))
        self.window.metricsTxFullDisplay.setText(str(robot_state.can_status.tx_full_count))
        self.window.metricsReceiveDisplay.setText(str(robot_state.can_status.receive_error_count))
        self.window.metricsTransmitDisplay.setText(str(robot_state.can_status.transmit_error_count))

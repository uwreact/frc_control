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

"""The AsyncPinger class."""

# Standard imports
import subprocess
import time

# ROS imports
from python_qt_binding import QtCore


class AsyncPinger(QtCore.QObject):
    """A QObject that asynchronously pings the specified hostnames, in order."""

    signalConnectionStatus = QtCore.pyqtSignal(bool)

    def __init__(self, parent=None, period=0.5, hostnames=None):
        super(AsyncPinger, self).__init__(parent)

        self.thread = QtCore.QThread()
        self.moveToThread(self.thread)
        self.thread.started.connect(self._run)

        self.running = True
        self.set_period(period)
        self.set_hostnames(hostnames)
        self.last_successful_hostname = ''

    def set_period(self, period):
        """Set the period between pings."""
        self.period = period

    def set_hostnames(self, hostnames):
        """Set the list of hostnames to ping, in order."""
        if not isinstance(hostnames, list):
            hostnames = [hostnames]
        self.hostnames = hostnames
        self.last_successful_hostname = ''

    def get_hostnames(self):
        """Return the list of hostnames to ping, in order."""
        return self.hostnames

    def get_last_success(self):
        """Return the hostname of the last successful ping."""
        return self.last_successful_hostname

    def start(self):
        """Start the pinger."""
        self.thread.start()

    def stop(self):
        """Stop the pinger."""
        self.running = False

    def _run(self):
        while self.running:

            # Ping each hostname for up to 0.2sec or 1 successful packet
            successful_hostname = None
            for hostname in self.hostnames:
                ret = subprocess.call(['timeout', '0.2', 'ping', '-c', '1', hostname],
                                      stdout=open('/dev/null'),
                                      stderr=open('/dev/null'))
                if ret == 0:
                    successful_hostname = hostname
                    break

            if successful_hostname is not None:
                self.last_successful_hostname = successful_hostname
                self.signalConnectionStatus.emit(True)
            else:
                self.signalConnectionStatus.emit(False)
            time.sleep(self.period)

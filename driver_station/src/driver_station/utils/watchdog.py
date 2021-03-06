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

"""The Watchdog class."""

# ROS imports
from python_qt_binding import QtCore
from rospy import Duration
from rospy import Timer


class Watchdog(QtCore.QObject):
    """A QObject that implements a watchdog, with `fed` and `expired` signals."""

    timeoutExpired = QtCore.pyqtSignal()
    watchdogFed = QtCore.pyqtSignal()

    def __init__(self, parent=None, timeout=1):
        super(Watchdog, self).__init__(parent)

        self.timeout = timeout
        self.timer = None

    def set_timeout(self, timeout):
        """Set the watchdog timeout."""
        self.timeout = timeout

    def start(self):
        """Start the watchdog."""
        self.timer = Timer(Duration(self.timeout), self._timeout, oneshot=True)

    def stop(self):
        """Stop the watchdog."""
        if self.timer is None:
            return

        self.timer.shutdown()
        self.timer = None

    def feed(self):
        """Feed the watchdog."""
        if self.timer is None:
            return

        self.watchdogFed.emit()
        self.timer.shutdown()
        self.start()

    def _timeout(self, _timer):
        self.timeoutExpired.emit()

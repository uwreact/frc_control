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
##############################################################################c

"""The Joystick and JoystickUpdater classes."""

# Standard imports
import array
import os
import struct
import threading
from fcntl import ioctl

# ROS imports
import rospy
from sensor_msgs.msg import Joy

# frc_control imports
from frc_msgs.msg import JoyArray

# Extracted from <linux/joystick.h>
LINUX_INPUT_MACROS = {
    'JSIOCGAXES': 0x80016A11,
    'JSIOCGBUTTONS': 0x80016A12,
    'JSIOCGNAME': lambda len: 0x80016A13 + (0x10000 * len)
}


class Joystick(object):
    """A Linux joystick wrapper."""

    def __init__(self, dev):
        self.devfile = dev
        self.eventfile = None

        devnum = int(self.devfile.split('js')[1])
        for dev_dirs in os.listdir('/sys/class/input/js{}/device'.format(devnum)):
            if dev_dirs.startswith('event'):
                self.eventfile = '/dev/input/' + dev_dirs

        self.jsdev = open(self.devfile, 'rb')
        if self.eventfile is not None:
            self.eventdev = open(self.eventfile, 'rb')

    @staticmethod
    def detect_sticks():
        """Detect up to 6 available joysticks."""
        joystick_devs = [fn for fn in os.listdir('/dev/input') if fn.startswith('js')]
        return ['/dev/input/' + fn for fn in joystick_devs[:6]]

    def get_num_axes(self):
        """Get the number of axes."""
        buf = array.array('B', [0])
        ioctl(self.jsdev, LINUX_INPUT_MACROS['JSIOCGAXES'], buf)
        return buf[0]

    def get_num_buttons(self):
        """Get the number of buttons."""
        buf = array.array('B', [0])
        ioctl(self.jsdev, LINUX_INPUT_MACROS['JSIOCGBUTTONS'], buf)
        return buf[0]

    def get_name(self, maxlen=64):
        """Get the identifier string."""
        buf = array.array('c', ['\0'] * maxlen)
        ioctl(self.jsdev, LINUX_INPUT_MACROS['JSIOCGNAME'](len(buf)), buf)
        return buf.tostring()

    def get_state(self):
        """Get the current state of the joystick, as a sensor_msgs/Joy."""
        joy = Joy()
        joy.axes = [0] * self.get_num_axes()
        joy.buttons = [0] * self.get_num_buttons()

        # Close and re-open the file to load the initial values.
        # TODO: This is slow. Instead, keep the file open and listen for events
        self.jsdev.close()
        self.jsdev = open(self.devfile, 'rb')

        for _ in range(0, self.get_num_axes() + self.get_num_buttons()):
            buf = self.jsdev.read(8)
            _time, value, val_type, number = struct.unpack('IhBB', buf)

            if val_type & 0x01:
                joy.buttons[number] = value

            if val_type & 0x02:
                joy.axes[number] = value / 32767.0

        return joy


class JoystickUpdater(threading.Thread):
    """Thread to update joystick states."""

    def __init__(self, data, joys=None):
        super(JoystickUpdater, self).__init__()
        self.data = data
        self.joys = joys or [Joystick(f) for f in Joystick.detect_sticks()]

    def run(self):
        # Loop until shutdown
        # TODO: Limit rate?
        while not rospy.is_shutdown():
            msg = JoyArray()
            for stick in self.joys:
                msg.sticks.append(stick.get_state())
                msg.names.append(stick.get_name())

            self.data.joys.set(msg)

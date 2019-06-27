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
import select
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
    'JSIOCGNAME': lambda length: 0x80016A13 + (0x10000 * length),
}


class Joystick(object):
    """A Linux joystick wrapper."""

    def __init__(self, devfile):
        self.devfile = devfile
        self.devname = self.devfile.split('/dev/')[1]
        self.eventfile = None

        # Load the feedback file, if it exists
        for dev_dirs in os.listdir('/sys/class/{}/device'.format(self.devname)):
            if dev_dirs.startswith('event'):
                self.eventfile = '/dev/input/' + dev_dirs

        # Open the devfiles
        self.jsdev = open(self.devfile, 'rb')
        if self.eventfile is not None:
            self.eventdev = open(self.eventfile, 'rb')

        # Load initial state
        self.uuid = self.get_uuid()
        self.state = self.get_state()

    @staticmethod
    def detect_sticks():
        """Detect up to 6 available joysticks."""
        joystick_devs = [fn for fn in sorted(os.listdir('/dev/input')) if fn.startswith('js')]
        return ['/dev/input/' + fn for fn in joystick_devs[:6]]

    def close(self):
        """Close the device files."""
        self.jsdev.close()
        if self.eventfile is not None:
            self.eventdev.close()

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

    def get_long_product(self):
        """Get the concatenated bustype, vendor, product, and version descriptors."""
        long_product = ''
        with open('/sys/class/{}/device/id/bustype'.format(self.devname), 'r') as f:
            long_product += '{}/'.format(f.read().strip())
        with open('/sys/class/{}/device/id/vendor'.format(self.devname), 'r') as f:
            long_product += '{}/'.format(f.read().strip())
        with open('/sys/class/{}/device/id/product'.format(self.devname), 'r') as f:
            long_product += '{}/'.format(f.read().strip())
        with open('/sys/class/{}/device/id/version'.format(self.devname), 'r') as f:
            long_product += '{}'.format(f.read().strip())
        return long_product

    def get_phys(self):
        """Get the phys descriptor."""
        with open('/sys/class/{}/device/phys'.format(self.devname), 'r') as f:
            return f.read().strip()

    def get_uuid(self):
        """Get a unique ID for this joystick.

        Since uniq is not reliable with JSIO devices, create an ID by appending the
        bustype, vendor, product, version, and phys descriptors."""
        return '{}/{}'.format(self.get_long_product(), self.get_phys())

    def get_state(self):
        """Get the current state of the joystick, as a sensor_msgs/Joy."""
        joy = Joy()
        joy.axes = [0] * self.get_num_axes()
        joy.buttons = [0] * self.get_num_buttons()

        if self.jsdev.closed:
            return joy

        # Close and re-open the file to load the initial values.
        self.jsdev.close()
        self.jsdev = open(self.devfile, 'rb')

        while len(select.select([self.jsdev], [], [], 0)[0]) > 0:
            try:
                buf = self.jsdev.read(8)
            except IOError:
                self.close()
                break

            _time, value, val_type, number = struct.unpack('IhBB', buf)

            if val_type & 0x01:
                joy.buttons[number] = value

            if val_type & 0x02:
                joy.axes[number] = value / 32767.0

        return joy

    def update(self):
        """Update the internal state."""
        if self.jsdev.closed:
            return self.state

        while len(select.select([self.jsdev], [], [], 0)[0]) > 0:

            try:
                buf = self.jsdev.read(8)
            except IOError:
                self.close()
                break

            if buf:
                _time, value, val_type, number = struct.unpack('IhBB', buf)
                if val_type & 0x01:
                    self.state.buttons[number] = value
                elif val_type & 0x02:
                    self.state.axes[number] = value / 32767.0
        return self.state


class JoystickManager(threading.Thread):
    """Thread to update joystick states."""

    def __init__(self, data, joys=None):
        super(JoystickManager, self).__init__()
        self.data = data
        self.rescan()

    def rescan(self):
        """Reload the available joysticks."""

        self.joys = [Joystick(f) for f in Joystick.detect_sticks()]

        mappings = self.data.joystick_mappings.get_all()
        uuids = mappings.keys()
        for uuid in uuids:

            # Remove all unlocked sticks
            if not mappings[uuid][2]:
                self.data.joystick_mappings.delete(uuid)
                del mappings[uuid]

            # Update all locked sticks
            else:
                for joy in self.joys:
                    if uuid == joy.uuid:
                        data = list(self.data.joystick_mappings.get(uuid))
                        data[1] = joy.get_name()
                        self.data.joystick_mappings.set(uuid, data)

        occupied_indexes = []
        for uuid in mappings.keys():
            occupied_indexes.append(self.data.joystick_mappings.get(uuid)[0])

        # Add in new sticks
        next_free_idx = 0
        for joy in self.joys:
            if joy.uuid in mappings:
                continue

            while next_free_idx in occupied_indexes:
                next_free_idx += 1

            self.data.joystick_mappings.set(joy.uuid, (next_free_idx, joy.get_name(), False))
            next_free_idx += 1

        self.data.joystick_mappings.force_notify()

    def run(self):

        # Update data at 75Hz. Must be >= publish frequency, which is 50Hz
        frequency = 75
        rate = rospy.Rate(frequency)

        # Loop until shutdown
        while not rospy.is_shutdown():
            msg = JoyArray()
            msg.sticks = [Joy()] * JoyArray.MAX_JOYSTICKS
            msg.names = [Joy()] * JoyArray.MAX_JOYSTICKS

            for stick in self.joys:
                idx = self.data.joystick_mappings.get(stick.uuid)[0]
                msg.sticks[idx] = stick.update()
                msg.names[idx] = stick.get_name()
            self.data.joys.set(msg)
            rate.sleep()

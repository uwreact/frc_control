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

"""The Joystick and JoystickManager classes."""

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
from driver_station.structs import JoystickInfo
from frc_msgs.msg import JoyArray

# Extracted from <linux/joystick.h>
LINUX_INPUT_MACROS = {
    'JSIOCGAXES': 0x80016A11,
    'JSIOCGBUTTONS': 0x80016A12,
    'JSIOCGNAME': lambda length: 0x80016A13 + (0x10000 * length),
    'JSIOCGAXMAP': 0x80406A32,
    'JSIOCGBTNMAP': 0x80406A34,
}

# Extracted from <linux/input-event-codes.h>
LINUX_ABS_AXIS_NAMES = {
    0x00: 'X Axis',
    0x01: 'Y Axis',
    0x02: 'Z Axis',
    0x03: 'RX Axis',
    0x04: 'RY Axis',
    0x05: 'RZ Axis',
    0x06: 'Throttle',
    0x07: 'Rudder',
    0x08: 'Wheel',
    0x09: 'Gas',
    0x0a: 'Brake',
    0x10: 'HAT0X',
    0x11: 'HAT0Y',
    0x12: 'HAT1X',
    0x13: 'HAT1Y',
    0x14: 'HAT2X',
    0x15: 'HAT2Y',
    0x16: 'HAT3X',
    0x17: 'HAT3Y',
    0x18: 'Pressure',
    0x19: 'Distance',
    0x1a: 'Tilt X',
    0x1b: 'Tilt Y',
    0x1c: 'Tool Width',
    0x20: 'Volume',
    0x28: 'Misc',
}

# Extracted from <linux/input-event-codes.h>
LINUX_BTN_NAMES = {
    0x120: 'Trigger',
    0x121: 'Thumb',
    0x122: 'Thumb 2',
    0x123: 'Top',
    0x124: 'Top 2',
    0x125: 'Pinkie',
    0x126: 'Base',
    0x127: 'Base 2',
    0x128: 'Base 3',
    0x129: 'Base 4',
    0x12a: 'Base 5',
    0x12b: 'Base 6',
    # 0x12f: 'Dead', # NOTE: Overriden by Mayflash GameCube controller
    0x130: 'A',
    0x131: 'B',
    0x132: 'C',
    0x133: 'X',
    0x134: 'Y',
    0x135: 'Z',
    0x136: 'Left Trigger',
    0x137: 'Right Trigger',
    0x138: 'Left Trigger 2',
    0x139: 'Right Trigger 2',
    0x13a: 'Select',
    0x13b: 'Start',
    0x13c: 'Mode',
    0x13d: 'Thumb L',
    0x13e: 'Thumb R',
    0x220: 'DPad Up',
    0x221: 'DPad Down',
    0x222: 'DPad Left',
    0x223: 'DPad Right',

    # XBox 360 Controller:
    0x2c0: 'DPad Left',
    0x2c1: 'DPad Right',
    0x2c2: 'DPad Up',
    0x2c3: 'DPad Down',

    # Mayflash GameCube Controller:
    0x12c: 'DPad Up',
    0x12d: 'DPad Right',
    0x12e: 'DPad Down',
    0x12f: 'DPad Left',
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

    def close(self):
        """Close the device files."""
        self.jsdev.close()
        if self.eventfile is not None:
            self.eventdev.close()

    def get_info(self):
        """Get the info for the joystick."""
        info = JoystickInfo()
        info.name = self.get_name()
        info.axis_names = self.get_axis_names()
        info.btn_names = self.get_button_names()
        return info

    def get_name(self, maxlen=64):
        """Get the identifier string."""
        buf = array.array('c', ['\0'] * maxlen)
        ioctl(self.jsdev, LINUX_INPUT_MACROS['JSIOCGNAME'](len(buf)), buf)
        return buf.tostring()

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

    def get_axis_names(self):
        """Get the name of each axis."""
        buf = array.array('B', [0] * 0x40)
        ioctl(self.jsdev, LINUX_INPUT_MACROS['JSIOCGAXMAP'], buf)
        return [LINUX_ABS_AXIS_NAMES.get(axis, 'Unknown ({})'.format(axis)) for axis in buf[:self.get_num_axes()]]

    def get_button_names(self):
        """Get the name of each button."""
        buf = array.array('H', [0] * 0x200)
        ioctl(self.jsdev, LINUX_INPUT_MACROS['JSIOCGBTNMAP'], buf)
        return [LINUX_BTN_NAMES.get(btn, 'Unknown ({})'.format(btn)) for btn in buf[:self.get_num_buttons()]]

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

        # Detect and load up to 6 available joysticks
        # TODO: Ensure that all locked joysticks are loaded, even if more than 6 joysticks are detected
        devfiles = [fn for fn in sorted(os.listdir('/dev/input')) if fn.startswith('js')]
        self.joys = [Joystick('/dev/input/' + fn) for fn in devfiles[:6]]

        mappings = self.data.joystick_mappings.get_all()
        uuids = mappings.keys()
        for uuid in uuids:

            # Remove all unlocked sticks
            if not mappings[uuid].locked:
                self.data.joystick_mappings.delete(uuid)
                del mappings[uuid]

            # Update all locked sticks
            else:
                for joy in self.joys:
                    if uuid == joy.uuid:
                        info = self.data.joystick_mappings.get(uuid)
                        info.name = joy.get_name()
                        info.axis_names = joy.get_axis_names()
                        info.btn_names = joy.get_button_names()
                        self.data.joystick_mappings.set(uuid, info)

        occupied_indexes = []
        for uuid in mappings.keys():
            occupied_indexes.append(self.data.joystick_mappings.get(uuid).index)

        # Add in new sticks
        next_free_idx = 0
        for joy in self.joys:
            if joy.uuid in mappings:
                continue

            while next_free_idx in occupied_indexes:
                next_free_idx += 1

            info = joy.get_info()
            info.index = next_free_idx
            info.locked = False
            self.data.joystick_mappings.set(joy.uuid, info)
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
                idx = self.data.joystick_mappings.get(stick.uuid).index
                msg.sticks[idx] = stick.update()
                msg.names[idx] = stick.get_name()
            self.data.joys.set(msg)
            rate.sleep()

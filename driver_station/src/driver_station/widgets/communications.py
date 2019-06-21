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

"""The CommunicationsWidget class."""

# TODO: Probably shouldn't call subprocesses from the main thread, should use the utils.async_popen instead

# Standard imports
import os
import re
import subprocess
from functools import partial

# ROS imports
from python_qt_binding.QtCore import QTimer

# frc_control imports
from driver_station.utils import gui_utils
from driver_station.utils import async_pinger

SYSFS_NETWORK_DEVICE_PATH = '/sys/class/net'


class CommunicationsWidget(object):
    """A widget to display the various communications statuses."""

    def __init__(self, window, match_data):
        self.window = window
        self.match_data = match_data
        self.init_ui()

        # Setup timer to periodically update the network indicators
        self.timer = QTimer(self.window)
        self.timer.timeout.connect(self._update)
        self.start_periodic()

        # Setup an async pinger to ping the radio
        #
        # From the manual:
        # 'Robot Radio' indicates the ping status to the robot wireless bridge at 10.TE.AM.1.
        self.robot_radio_pinger = async_pinger.AsyncPinger()
        self.robot_radio_pinger.set_hostnames('10.0.0.1')
        self.robot_radio_pinger.signalConnectionStatus.connect(
            partial(gui_utils.bool_style, self.window.robotRadioCommsDisplay))
        self.robot_radio_pinger.start()

        # Setup an async pinger to ping the roboRIO
        #
        # From the manual:
        # 'Robot' indicates the ping status to the roboRIO using mDNS (with a fallback of a static 10.TE.AM.2 address).
        self.robot_pinger = async_pinger.AsyncPinger()
        self.robot_pinger.set_hostnames(['roborio-0-frc.local', '10.0.0.2'])
        self.robot_pinger.signalConnectionStatus.connect(self._update_robot_comms)
        self.robot_pinger.start()

    def init_ui(self):
        """Setup the UI elements."""

        # Connection indicators
        gui_utils.bool_style(self.window.enetLinkCommsDisplay, False)
        gui_utils.bool_style(self.window.robotRadioCommsDisplay, False)
        gui_utils.bool_style(self.window.robotCommsDisplay, False)
        gui_utils.bool_style(self.window.fmsCommsDisplay, False)

        # Network indicators
        gui_utils.bool_style(self.window.enetCommsDisplay, False)
        gui_utils.bool_style(self.window.wifiCommsDisplay, False)
        gui_utils.bool_style(self.window.usbCommsDisplay, False)
        gui_utils.bool_style(self.window.firewallCommsDisplay, False)

        # TODO: Implement USB indicator logic
        # Since USB indicator is not yet implemented, grey it out
        self.window.usbCommsDisplay.setStyleSheet('background-color: rgb(100, 100, 100);')
        self.window.usbCommsTitle.setStyleSheet('color: rgb(100, 100, 100);')

    def set_team_number(self, team_number):
        """Set the team number."""
        upper = team_number / 100
        lower = team_number % 100

        self.robot_radio_pinger.set_hostnames('10.{}.{}.1'.format(upper, lower))
        self.robot_pinger.set_hostnames(['roborio-{}-frc.local'.format(team_number), '10.{}.{}.2'.format(upper, lower)])

    def start_periodic(self, period=100):
        """Start timer to update comms status."""
        self.timer.setInterval(period)
        self.timer.start()
        self._update()

    def _update(self):
        self._update_enet()
        self._update_wifi()
        self._update_firewall()
        gui_utils.bool_style(self.window.fmsCommsDisplay, self.match_data.event_name != '')

    def _update_enet(self):
        """Update the ethernet-related comms status.

        'Enet Link' indicates the computer has something connected to the ethernet port.
        'Enet' indicates the IP address of the detected Ethernet adapter.
        """

        enet_adapters = os.listdir(SYSFS_NETWORK_DEVICE_PATH)
        enet_adapters = [net for net in enet_adapters if 'en' in net]

        enet_link = False
        enet_addr = ''

        if len(enet_adapters) > 0:
            enet_link = False
            for adapter in enet_adapters:
                path = os.path.join(SYSFS_NETWORK_DEVICE_PATH, adapter)
                state = open(os.path.join(path, 'operstate')).readline()

                if 'up' in state or 'unknown' in state:
                    enet_link = True
                    enet_addr = get_ip_address(enet_adapters[0])
                    break

        gui_utils.bool_style(self.window.enetLinkCommsDisplay, enet_link)
        gui_utils.bool_style(self.window.enetCommsDisplay, enet_addr != '')

        text = 'Enet'
        if enet_addr != '':
            text += ' <span style="color:#{:06x};">{}</span>'.format(gui_utils.Color.BTN_GREEN, enet_addr)
        self.window.enetCommsTitle.setText(text)

    def _update_wifi(self):
        """Update the wifi-related comms status.

        'WiFi' indicates if a wireless adapter has been detected as enabled.
        """

        wifi_adapters = os.listdir(SYSFS_NETWORK_DEVICE_PATH)
        wifi_adapters = [net for net in wifi_adapters if 'wl' in net]

        wifi_link = False

        if len(wifi_adapters) > 0:
            for adapter in wifi_adapters:
                path = os.path.join(SYSFS_NETWORK_DEVICE_PATH, adapter)
                state = open(os.path.join(path, 'operstate')).readline()

                if 'up' in state:
                    wifi_link = True
                    break

        gui_utils.bool_style(self.window.wifiCommsDisplay, wifi_link)

    def _update_firewall(self):
        """Update the firewall status.

        'Firewall' indicates if any firewalls are detected as enabled.
        Enabled firewalls will show in orange (Dom = Domain, Pub = Public, Prv = Private).
        """

        # TODO: Display 'Dom' 'Pub' 'Prv'? Not sure how these map from Windows to UFW
        # TODO: Should this be green when the firewall is on or off? Pretty sure green when off.

        try:
            status = subprocess.check_output(['ufw', 'status'], stderr=open('/dev/null')).split(": ")[1].strip()
            gui_utils.bool_style(self.window.firewallCommsDisplay, status == 'inactive')
        except subprocess.CalledProcessError:
            # User is not sudo or ufw not installed. In either case, just grey out the firewall indicator
            self.window.firewallCommsDisplay.setStyleSheet('background-color: rgb(100, 100, 100);')
            self.window.firewallCommsTitle.setStyleSheet('color: rgb(100, 100, 100);')

    def _update_robot_comms(self, has_comms):
        """Update the robot comms status."""

        hostname = self.robot_pinger.get_last_success()
        if re.compile('^[0-9\\.]*$').match(hostname):
            robot_ip = hostname
        else:
            robot_ip = resolve_hostname(hostname)
        text = 'Robot'
        if robot_ip != '':
            text += ' <span style="color:#{:06x};">{}</span>'.format(gui_utils.Color.BTN_GREEN, robot_ip)

        gui_utils.bool_style(self.window.robotCommsDisplay, has_comms)
        self.window.robotCommsTitle.setText(text)


def get_ip_address(ifname):
    """Get the IP address associated with the specified interface."""

    try:
        return subprocess.check_output(['ip', 'addr', 'show', ifname]).split("inet ")[1].split("/")[0]
    except (subprocess.CalledProcessError, IndexError):
        return ''


def resolve_hostname(hostname):
    """Get the IP address associated with the specified hostname."""
    if hostname == '':
        return ''

    try:
        return subprocess.check_output(['timeout', '0.1', 'getent', 'hosts', hostname]).split(" ")[0]
    except (subprocess.CalledProcessError, IndexError):
        return ''

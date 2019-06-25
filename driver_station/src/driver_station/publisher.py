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

"""The Publisher class."""

# Standard imports
import threading

# ROS imports
import rospy

# frc_control imports
from frc_msgs.msg import DriverStationMode
from frc_msgs.msg import JoyArray
from frc_msgs.msg import MatchData
from frc_msgs.msg import MatchTime


class Publisher(threading.Thread):
    """ROS Topic publishers."""

    def __init__(self, data):
        super(Publisher, self).__init__()

        self.data = data

        self.ds_mode_pub = rospy.Publisher('/frc/ds_mode', DriverStationMode, queue_size=10)
        self.match_time_pub = rospy.Publisher('/frc/match_time', MatchTime, queue_size=10)
        self.match_data_pub = rospy.Publisher('/frc/match_data', MatchData, queue_size=10)
        self.joys_pub = rospy.Publisher('/frc/joys', JoyArray, queue_size=10)

        # Since this app IS the driver station, is_ds_attached is always True
        self.data.ds_mode.set_attr('is_ds_attached', True)

    def run(self):

        # Publish data at 50Hz to match the rate of the real driver station.
        frequency = 50
        rate = rospy.Rate(frequency)

        # Loop until shutdown
        while not rospy.is_shutdown():
            self.ds_mode_pub.publish(self.data.ds_mode.get())
            self.match_time_pub.publish(self.data.match_time.get())
            self.match_data_pub.publish(self.data.match_data.get())
            self.joys_pub.publish(self.data.joys.get())
            rate.sleep()

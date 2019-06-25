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

"""The Subscriber class."""

# ROS imports
import rospy

# frc_control imports
from frc_msgs.msg import JoyFeedback
from frc_msgs.msg import RobotState


class Subscriber(object):
    """ROS Topic Subscribers."""

    def __init__(self, data):
        self.data = data

        rospy.Subscriber('/frc/joy_feedback', JoyFeedback, self._joy_feedback_callback)
        rospy.Subscriber('/frc/robot_state', RobotState, self._robot_state_callback)

        self.robot_state_callbacks = []

    def add_robot_state_callback(self, callback):
        """Add a callback to notify when any RobotState msg is received."""
        self.robot_state_callbacks.append(callback)

    def _joy_feedback_callback(self, feedback_data):

        # Remove the header so that observers are only notified when the actual contents of the msg change
        feedback_data.header = None
        self.data.joy_feedback.set(feedback_data)

    def _robot_state_callback(self, robot_state):

        # Remove the header so that observers are only notified when the actual contents of the msg change
        robot_state.header = None
        self.data.robot_state.set(robot_state)

        # Use explicit callbacks here rather than relying on the robot_state observer since we want to notify
        # some observers whenever ANY msg is received, regardless of whether the data is changed or not.
        for callback in self.robot_state_callbacks:
            callback()

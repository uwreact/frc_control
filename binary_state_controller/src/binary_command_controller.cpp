///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2019, UW REACT
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of UW REACT, nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//////////////////////////////////////////////////////////////////////////////

#include <binary_state_controller/binary_command_controller.h>
#include <pluginlib/class_list_macros.hpp>

namespace binary_state_controller {

bool BinaryCommandController::init(hardware_interface::BinaryCommandInterface* hw, ros::NodeHandle& n) {
  std::string joint_name;
  if (!n.getParam("joint", joint_name)) {
    ROS_ERROR("No joint given (namespace: %s)", n.getNamespace().c_str());
    return false;
  }
  joint_       = hw->getHandle(joint_name);

  if (!n.getParam("default", default_val_)) {
    default_val_ = false;
  }

  sub_command_ = n.subscribe<std_msgs::Bool>("command", 1, &BinaryCommandController::commandCB, this);
  return true;
}

void BinaryCommandController::starting(const ros::Time& /*time*/) {
  command_buffer_.writeFromNonRT(default_val_);
}

void BinaryCommandController::update(const ros::Time& /*time*/, const ros::Duration& /*period*/) {
  joint_.setCommand(*command_buffer_.readFromRT());
}

void BinaryCommandController::commandCB(const std_msgs::BoolConstPtr& msg) {
  command_buffer_.writeFromNonRT(msg->data);
}

}  // namespace binary_state_controller

PLUGINLIB_EXPORT_CLASS(binary_state_controller::BinaryCommandController, controller_interface::ControllerBase)

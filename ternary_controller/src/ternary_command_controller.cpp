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

#include <pluginlib/class_list_macros.hpp>
#include <ternary_controller/ternary_command_controller.h>

namespace ternary_controller {
bool TernaryCommandController::init(hardware_interface::TernaryCommandInterface* hw, ros::NodeHandle& nh) {
  std::string joint_name;
  if (!nh.getParam("joint", joint_name)) {
    ROS_ERROR("No joint given (namespace: %s)", nh.getNamespace().c_str());
    return false;
  }
  joint_ = hw->getHandle(joint_name);

  int def;
  if (!nh.getParam("default", def)) {
    def = 0;
  }

  if (def < 0) {
    default_val_ = TernaryState::kReverse;
  } else if (def > 0) {
    default_val_ = TernaryState::kForward;
  } else {
    default_val_ = TernaryState::kOff;
  }

  sub_command_ = nh.subscribe<std_msgs::Int8>("command", 1, &TernaryCommandController::commandCallback, this);
  return true;
}

void TernaryCommandController::starting(const ros::Time& /*time*/) {
  command_buffer_.writeFromNonRT(default_val_);
}

void TernaryCommandController::update(const ros::Time& /*time*/, const ros::Duration& /*period*/) {
  joint_.setCommand(*command_buffer_.readFromRT());
}

void TernaryCommandController::commandCallback(const std_msgs::Int8ConstPtr& msg) {
  TernaryState state;
  if (msg->data < 0) {
    state = TernaryState::kReverse;
  } else if (msg->data > 0) {
    state = TernaryState::kForward;
  } else {
    state = TernaryState::kOff;
  }
  command_buffer_.writeFromNonRT(state);
}

}  // namespace ternary_controller

PLUGINLIB_EXPORT_CLASS(ternary_controller::TernaryCommandController, controller_interface::ControllerBase)

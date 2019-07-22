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

#include <compressor_controller/compressor_command_controller.h>
#include <pluginlib/class_list_macros.hpp>

namespace compressor_controller {

bool CompressorCommandController::init(hardware_interface::CompressorCommandInterface* hw, ros::NodeHandle& nh) {
  std::string joint_name;
  if (!nh.getParam("joint", joint_name)) {
    ROS_ERROR("No joint given (namespace: %s)", nh.getNamespace().c_str());
    return false;
  }
  compressor_ = hw->getHandle(joint_name);

  sub_command_ = nh.subscribe<std_msgs::Bool>("command", 1, &CompressorCommandController::commandCallback, this);
  return true;
}

void CompressorCommandController::starting(const ros::Time& /*time*/) {
  command_buffer_.writeFromNonRT(true);
}

void CompressorCommandController::update(const ros::Time& /*time*/, const ros::Duration& /*period*/) {
  compressor_.setCommand(*command_buffer_.readFromRT());
}

void CompressorCommandController::commandCallback(const std_msgs::BoolConstPtr& msg) {
  command_buffer_.writeFromNonRT((bool) msg->data);
}

}  // namespace compressor_controller

PLUGINLIB_EXPORT_CLASS(compressor_controller::CompressorCommandController, controller_interface::ControllerBase)

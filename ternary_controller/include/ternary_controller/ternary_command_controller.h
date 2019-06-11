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

#pragma once

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <realtime_tools/realtime_buffer.h>
#include <ros/node_handle.h>
#include <std_msgs/Int8.h>
#include <ternary_controller/ternary_command_interface.h>


namespace ternary_controller {

/**
 * @brief Ternary joint controller.
 *
 * This class passes the ternary (tri-state) command signal down to the joint.
 *
 * Joint commands are converted from Int8 to hardware_interface::TernaryStateHandle::TernaryState according the Int8's
 * sign. Negative values correspond to kReverse. Positive values correspond to kForward. Zero corresponds to kOff.
 *
 * Subscribes to:
 * - @bold command (std_msgs::Int8) : The joint command to apply.
 */
class TernaryCommandController : public controller_interface::Controller<hardware_interface::TernaryCommandInterface> {
private:
  using TernaryState = hardware_interface::TernaryStateHandle::TernaryState;

public:
  TernaryCommandController() = default;
  ~TernaryCommandController() { sub_command_.shutdown(); }

  bool init(hardware_interface::TernaryCommandInterface* hw, ros::NodeHandle& n) override;
  void starting(const ros::Time& /*time*/) override;
  void update(const ros::Time& /*time*/, const ros::Duration& /*period*/) override;

  hardware_interface::TernaryCommandHandle     joint_;
  realtime_tools::RealtimeBuffer<TernaryState> command_buffer_;
  TernaryState                                 default_val_;

private:
  ros::Subscriber sub_command_;
  void            commandCB(const std_msgs::Int8ConstPtr& msg);
};

}  // namespace ternary_controller

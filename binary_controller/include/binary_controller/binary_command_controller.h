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

#include <binary_controller/binary_command_interface.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <realtime_tools/realtime_buffer.h>
#include <ros/node_handle.h>
#include <std_msgs/Bool.h>


namespace binary_controller {

/**
 * @brief Binary joint controller.
 *
 * This class passes the binary (dual-state) command signal down to the joint.
 *
 * Subscribes to:
 * - @bold command (std_msgs::Bool) : The joint command to apply.
 */
class BinaryCommandController : public controller_interface::Controller<hardware_interface::BinaryCommandInterface> {
public:
  BinaryCommandController() = default;
  ~BinaryCommandController() { sub_command_.shutdown(); }

  bool init(hardware_interface::BinaryCommandInterface* hw, ros::NodeHandle& n) override;
  void starting(const ros::Time& /*time*/) override;
  void update(const ros::Time& /*time*/, const ros::Duration& /*period*/) override;

  hardware_interface::BinaryCommandHandle joint_;
  realtime_tools::RealtimeBuffer<bool>    command_buffer_;
  bool                                    default_val_;

private:
  ros::Subscriber sub_command_;
  void            commandCB(const std_msgs::BoolConstPtr& msg);
};

}  // namespace binary_controller

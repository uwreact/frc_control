///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2018, UW REACT
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

#include <frc_robot_hw/robot_control_loop.h>

namespace frc_robot_hw {

bool RobotControlLoop::init() {

  // Load control loop parameters
  ros::NodeHandle loop_nh(nh_, name_);
  loop_nh.param("control_frequency", control_freq_, 50.0);
  loop_nh.param("controller_watchdog_timeout", controller_watchdog_timeout_, 0.5);

  // Ensure that robot_hw_ exists
  if (!robot_hw_) {
    ROS_WARN_NAMED(name_, "No RobotHW initialized, falling back on default base class");
    robot_hw_ = std::make_unique<FRCRobotHW>();
  }

  // Initialize the RobotHW
  ros::NodeHandle robot_nh(nh_, robot_hw_->getName());
  if (!robot_hw_->init(nh_, robot_nh)) {
    ROS_FATAL_STREAM_NAMED(name_, "Failed to initialize " << robot_hw_->getName());
    return false;
  }

  // Initialize the controller manager
  controller_manager_ = std::make_unique<controller_manager::ControllerManager>(robot_hw_.get(), nh_);
  return true;
}

void RobotControlLoop::update(const ros::Time& time_now, bool update_controllers) {
  ros::Duration rw_period = time_now - last_rw_time_;
  last_rw_time_           = time_now;

  // Read from hardware
  robot_hw_->read(time_now, rw_period);
  robot_hw_->updateRobotState();

  if (update_controllers) {
    ros::Duration update_period = time_now - last_update_time_;
    last_update_time_           = time_now;

    // If we've gone too long without updating controllers, reset them
    bool timeout = (update_period.toSec() > controller_watchdog_timeout_);

    // Update the controllers
    controller_manager_->update(time_now, update_period, timeout);
  }

  // TODO: Enforce limits

  // Write to hardware
  robot_hw_->write(time_now, rw_period);
}

}  // namespace frc_robot_hw

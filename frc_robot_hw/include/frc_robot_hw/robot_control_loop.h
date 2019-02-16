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

#pragma once

#include <controller_manager/controller_manager.h>
#include <frc_robot_hw/frc_robot_hw.h>
#include <memory>
#include <ros/ros.h>
#include <thread>

namespace frc_robot_hw {

/// Generic template for robot control loop
class RobotControlLoop {
public:
  RobotControlLoop(std::string name, const ros::NodeHandle& nh = ros::NodeHandle()) : name_(std::move(name)), nh_(nh) {}

  virtual ~RobotControlLoop() = default;

  /// Initialize the control loop
  virtual bool init();

  /**
   * @brief Read and write to the hardware, and optionally update the controllers
   *
   * @param time_now The current timestamp
   * @param update_controllers Whether to update the controllers
   */
  void update(const ros::Time& time_now, bool update_controllers = true);

protected:
  // Short name of this class
  const std::string name_;

  // Handle at root of robot's namespace
  ros::NodeHandle nh_;

  // FRCRobotHW instance
  std::unique_ptr<FRCRobotHW> robot_hw_;

  // Controller manager
  std::unique_ptr<controller_manager::ControllerManager> controller_manager_;

  // Timing
  ros::Time last_update_time_;             ///< The last time the controllers were updated
  ros::Time last_rw_time_;                 ///< The last time the hardware was read/written
  double    controller_watchdog_timeout_;  ///< The watchdog time interval, in seconds
  double    control_freq_;                 ///< The frequency of the control loop, in hz
};

}  // namespace frc_robot_hw

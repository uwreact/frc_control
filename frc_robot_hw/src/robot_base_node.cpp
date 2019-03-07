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

#include <frc_robot_hw/frc_robot_hw.h>
#include <ros/callback_queue.h>

namespace frc_robot_hw {

class RobotHWControlLoop : public RobotControlLoop {
public:
  explicit RobotHWControlLoop(const ros::NodeHandle& nh) : RobotControlLoop("robot_hw_control_loop", nh) {

    // Load RobotHW and controller manager
    robot_hw_ = std::make_unique<FRCRobotHW>();
    if (!init())
      throw std::runtime_error("Failed to initialize RobotHW");

    // Get current time for use with first update
    ros::ros_steadytime(last_update_time_.sec, last_update_time_.nsec);
  }

  /// Run the control loop (blocking)
  void run() {
    ros::Time time_now;
    ros::Rate rate(control_freq_);

    while (ros::ok()) {
      ros::ros_steadytime(time_now.sec, time_now.nsec);
      update(time_now);
      rate.sleep();
    }
  }
};

}  // namespace frc_robot_hw

int main(int argc, char** argv) {
  ros::init(argc, argv, "robot_base");
  ros::NodeHandle nh;

  // Queue for RobotHW node callbacks (seperate from global queue)
  ros::CallbackQueue cb_queue;
  nh.setCallbackQueue(&cb_queue);

  // Start AsyncSpinner for custom queue, run on all available threads
  ros::AsyncSpinner spinner(0, &cb_queue);
  spinner.start();

  // Create the controller manager interface
  try {
    frc_robot_hw::RobotHWControlLoop robot_control_loop(nh);
    robot_control_loop.run();  // Blocks until shutdown signal recieved
  }
  catch (const std::runtime_error& e) {
    ROS_FATAL(e.what());
  }

  ROS_INFO("Shutting down");
  return 0;
}

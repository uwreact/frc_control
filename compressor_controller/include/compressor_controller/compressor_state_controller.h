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

// ROS
#include <controller_interface/controller.h>
#include <realtime_tools/realtime_publisher.h>

// Custom
#include <compressor_controller/CompressorData.h>
#include <compressor_controller/compressor_state_interface.h>

namespace compressor_controller {

/** @brief Controller that publishes the state of every compressor in a robot.
 *
 * This controller publishes the state of all resources registered to a `hardware_interface::CompressorStateHandle` to a
 * topic of type `compressor_controller/CompressorData`. The following is a basic configuration of the controller.
 *
 * @code
 * compressor_state_controller:
 *   type: compressor_controller/CompressorStateController
 *   publish_rate: 50
 * @endcode
 */
class CompressorStateController
    : public controller_interface::Controller<hardware_interface::CompressorStateInterface> {
public:
  CompressorStateController() : publish_rate_(15.0) {}

  virtual bool init(hardware_interface::CompressorStateInterface* hw,
                    ros::NodeHandle&                              root_nh,
                    ros::NodeHandle&                              controller_nh) override;
  virtual void starting(const ros::Time& time) override;
  virtual void update(const ros::Time& time, const ros::Duration& period) override;

private:
  using RtPublisher         = realtime_tools::RealtimePublisher<compressor_controller::CompressorData>;
  using RtPublisherPtr      = std::shared_ptr<RtPublisher>;
  using RtPublisherConstPtr = std::shared_ptr<const RtPublisher>;

  std::vector<hardware_interface::CompressorStateHandle> compressor_states_;
  std::vector<RtPublisherPtr>                            realtime_pubs_;
  std::vector<ros::Time>                                 last_publish_times_;
  double                                                 publish_rate_;
};

}  // namespace compressor_controller

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

#include <pdp_state_controller/pdp_state_controller.h>

#include <algorithm>
#include <pluginlib/class_list_macros.h>

namespace pdp_state_controller {

bool PDPStateController::init(hardware_interface::PDPStateInterface* hw,
                              ros::NodeHandle&                       root_nh,
                              ros::NodeHandle&                       controller_nh) {

  // Get all PDP names from the hardware interface
  const std::vector<std::string>& pdp_names = hw->getNames();

  // Get publishing period
  if (!controller_nh.getParam("publish_rate", publish_rate_)) {
    ROS_ERROR("PDPStateController parameter 'publish_rate' not set");
    return false;
  }

  // Setup publishers
  for (const auto& pdp_name : pdp_names) {
    ROS_DEBUG("Got pdp %s", pdp_name.c_str());
    pdp_states_.push_back(hw->getHandle(pdp_name));

    realtime_pubs_.push_back(std::make_shared<RtPublisher>(root_nh, pdp_name, 4));
  }

  // Last published times
  last_publish_times_.resize(pdp_names.size());
  return true;
}

void PDPStateController::starting(const ros::Time& time) {
  std::fill(last_publish_times_.begin(), last_publish_times_.end(), time);
}

void PDPStateController::update(const ros::Time& time, const ros::Duration& /*period*/) {
  for (unsigned i = 0; i < realtime_pubs_.size(); i++) {

    // Limit rate of publishing
    if ((publish_rate_ > 0.0) && (last_publish_times_[i] + ros::Duration(1.0 / publish_rate_) < time)) {

      // Try to publish
      if (realtime_pubs_[i]->trylock()) {
        last_publish_times_[i] = last_publish_times_[i] + ros::Duration(1.0 / publish_rate_);

        // Populate message
        realtime_pubs_[i]->msg_.header.stamp = time;
        realtime_pubs_[i]->msg_.voltage      = pdp_states_[i].getVoltage();
        realtime_pubs_[i]->msg_.temperature  = pdp_states_[i].getTemperature();
        realtime_pubs_[i]->msg_.totalCurrent = pdp_states_[i].getTotalCurrent();
        realtime_pubs_[i]->msg_.totalPower   = pdp_states_[i].getTotalPower();
        realtime_pubs_[i]->msg_.totalEnergy  = pdp_states_[i].getTotalEnergy();

        for (unsigned channel = 0; channel < 16; channel++)
          realtime_pubs_[i]->msg_.current[channel] = pdp_states_[i].getCurrent(channel);

        // Publish data
        realtime_pubs_[i]->unlockAndPublish();
      }
    }
  }
}

}  // namespace pdp_state_controller

PLUGINLIB_EXPORT_CLASS(pdp_state_controller::PDPStateController, controller_interface::ControllerBase)

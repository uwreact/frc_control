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

#include <frc_robot_sim/gazebo_hw_plugin.h>

#include <frc_robot_sim/frc_robot_hw_sim.h>
#include <functional>

namespace frc_robot_sim {

void GazeboHWPlugin::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) {
  parent_model_ = model;
  sdf_          = sdf;

  // Ensure we have a parent model
  if (!parent_model_) {
    ROS_ERROR_NAMED(name_, "No model");
    return;
  }

  // Ensure the ROS plugin is loaded
  if (!ros::isInitialized()) {
    ROS_FATAL_NAMED(name_, "ROS API plugin not loaded");
    return;
  }

  // Setup the node handle
  if (sdf_->HasElement("robotNamespace"))
    robot_namespace_ = sdf_->Get<std::string>("robotNamespace");
  else
    robot_namespace_ = parent_model_->GetName();  // Default namespace
  nh_ = ros::NodeHandle(robot_namespace_);

  // Setup the RobotHW
  robot_hw_ = std::make_unique<FRCRobotHWSim>(parent_model_);

  // Setup the control loop
  try {
    init();
  } catch (const std::exception& e) {
    ROS_FATAL_STREAM_NAMED(name_, e.what());
    // TODO: Shutdown
  }

  control_period_ = ros::Duration(1.0 / control_freq_);

  // Listen to the world update event. This event is broadcast every simulation step.
  update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&GazeboHWPlugin::WorldUpdate, this, std::placeholders::_1));

  ROS_INFO_STREAM_NAMED(name_, "Loaded " << name_);
}

void GazeboHWPlugin::Reset() {
  // Reset timing variables on world reset
  last_rw_time_     = ros::Time();
  last_update_time_ = ros::Time();
}

void GazeboHWPlugin::WorldUpdate(const gazebo::common::UpdateInfo& info) {
  const ros::Time now(info.simTime.sec, info.simTime.nsec);
  const bool&     update_controllers = (now - last_update_time_) > control_period_;

  update(now, update_controllers);
}

// Register plugin with Gazebo
GZ_REGISTER_MODEL_PLUGIN(GazeboHWPlugin);

}  // namespace frc_robot_sim

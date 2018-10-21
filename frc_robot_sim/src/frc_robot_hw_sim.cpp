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

#include <frc_robot_sim/frc_robot_hw_sim.h>

namespace frc_robot_sim {

bool FRCRobotHWSim::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) {
  if (!FRCRobotHW::init(root_nh, robot_hw_nh))
    return false;

  // TODO: Any other require initialization

  return true;
}

void FRCRobotHWSim::read(const ros::Time& time, const ros::Duration& period) {

  // Read all simple speed controllers
  for (const auto& pair : simple_speed_controller_templates_) {
    const auto& joint = model_->GetJoint(pair.first);
    joint_states_[pair.first].pos = joint->GetAngle(0).Radian();
    joint_states_[pair.first].vel = joint->GetVelocity(0);
    joint_states_[pair.first].eff = joint->GetForce(0);
  }

  // Read all servos
  for (const auto& pair : servo_templates_) {
    const auto& joint = model_->GetJoint(pair.first);
    joint_states_[pair.first].pos = joint->GetAngle(0).Radian();
    joint_states_[pair.first].vel = joint->GetVelocity(0);
    joint_states_[pair.first].eff = joint->GetForce(0);
  }

  // Read all solenoids
  for (const auto& pair : solenoid_templates_) {
    const auto& joint = model_->GetJoint(pair.first);
    const double min = joint->GetLowerLimit(0).Radian(); // Note: Radian is actually meters in prismatic joints
    const double max = joint->GetUpperLimit(0).Radian(); // Note: Radian is actually meters in prismatic joints
    const double mid = (max - min) / 2.0 + min;
    const double pos = joint->GetAngle(0).Radian(); // Note: Radian is actually meters in prismatic joints

    binary_states_[pair.first] = pos > mid;
  }

  // Read all double solenoids
  for (const auto& pair : double_solenoid_templates_) {
    const auto& joint = model_->GetJoint(pair.first);
    const double min = joint->GetLowerLimit(0).Radian(); // Note: Radian is actually meters in prismatic joints
    const double max = joint->GetUpperLimit(0).Radian(); // Note: Radian is actually meters in prismatic joints
    const double mid = (max - min) / 2.0 + min;
    const double pos = joint->GetAngle(0).Radian(); // Note: Radian is actually meters in prismatic joints

    TernaryState state;
    if (pos > mid)
      state = TernaryState::kForward;
    else
      state = TernaryState::kReverse;
    // TODO: How to implement TernaryState::kOff? Not sure it's needed but would be nice for completeness

    ternary_states_[pair.first] = state;
  }

  // TODO: Support DigitalInput, AnalogInput, IMUs
}

void FRCRobotHWSim::write(const ros::Time& time, const ros::Duration& period) {

  // Command all simple speed controllers
  for (const auto& pair : simple_speed_controller_templates_) {
    const auto& joint = model_->GetJoint(pair.first);

    // TODO: Validate these
    switch (joint_commands_[pair.first].type) {
      case JointCmd::Type::kPos:
        joint->SetParam("fmax", 0, 0.0); // TODO: Is this needed?
        joint->SetPosition(0, joint_commands_[pair.first].data); // TODO: SetPosition is bad
        break;
      case JointCmd::Type::kVel:
        // TODO: Support non-ODE engines? Why is SetVelocity so broken?

        // Note: We don't use SetVelocity since we want to set velocity using Joint Motors
        // Therefore, we set fmax whenever in velocity control, otherwise we unset fmax and use the standard
        // SetPosition() and SetForce() functions. TODO: Do we need to unset?
        // See http://gazebosim.org/tutorials?tut=set_velocity
        joint->SetParam("fmax", 0, 1000.0); // TODO: Appropriate limit
        joint->SetParam("vel", 0, joint_commands_[pair.first].data);
        break;
      case JointCmd::Type::kEff:
        joint->SetParam("fmax", 0, 0.0); // TODO: Is this needed?
        joint->SetForce(0, joint_commands_[pair.first].data);
        break;
      case JointCmd::Type::kVolt:
        // TODO: Zoom zoom
        break;
      case JointCmd::Type::kNone:
        // TODO: Brake vs coast

        joint->SetForce(0, 0.0);
        joint->SetParam("vel", 0, 0.0);
        break;
    }
  }

  // Command all servos
  for (const auto& pair : servo_templates_) {
    const auto& joint = model_->GetJoint(pair.first);
    joint->SetPosition(0, joint_commands_[pair.first].data); // TODO: SetPosition is bad
  }

  // Command all single Solenoids
  for (const auto& pair : solenoid_templates_) {
    const auto& joint = model_->GetJoint(pair.first);
    double position;
    if (binary_commands_[pair.first])
      position = joint->GetUpperLimit(0).Radian();
    else
      position = joint->GetLowerLimit(0).Radian();
    joint->SetPosition(0, position);
  }

  // Command all double Solenoids
  for (const auto& pair : double_solenoid_templates_) {
    const auto& joint = model_->GetJoint(pair.first);
    double position;
    if (ternary_commands_[pair.first] == TernaryState::kForward)
      position = joint->GetUpperLimit(0).Radian();
    else if (ternary_commands_[pair.first] == TernaryState::kReverse)
      position = joint->GetLowerLimit(0).Radian();
    else
      position = joint->GetAngle(0).Radian();
    joint->SetPosition(0, position);
  }

  // TODO: Support DigitalOutput, AnalogOutput, (Relay?)
  }

} // namespace frc_robot_sim
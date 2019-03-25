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

#include <frc_robot_hw/pid_controller.h>
#include <ros/time.h>

namespace frc_robot_hw {

bool MultiPIDController::setMode(Mode mode) {
  mode_ = mode;

  switch (mode_) {
    case Mode::disabled:
      cur_gains_ = nullptr;
    case Mode::position:
      cur_gains_ = &position_gains_;
      break;
    case Mode::velocity:
      cur_gains_ = &velocity_gains_;
      break;
    case Mode::effort:
      cur_gains_ = &effort_gains_;
      break;
      // No default case
  }

  reset();
  return true;
}

bool MultiPIDController::setMode(const std::string& mode) {
  if (mode == "disabled" || mode == "Disabled")
    return setMode(Mode::disabled);
  if (mode == "position" || mode == "Position")
    return setMode(Mode::position);
  if (mode == "velocity" || mode == "Velocity")
    return setMode(Mode::velocity);
  if (mode == "effort" || mode == "Effort")
    return setMode(Mode::effort);
  return false;
}

void MultiPIDController::reset() {
  integral_   = 0.0;
  last_error_ = 0.0;
  is_first_   = true;
  output_     = 0.0;
}

void MultiPIDController::setSetpoint(float setpoint) {
  setpoint_ = setpoint;
}

void MultiPIDController::update(float input) {
  if (mode_ == Mode::disabled)
    return;

  float error = setpoint_ - input;

  // Calculate time step
  // TODO: Pass in time step?
  static ros::Time last_time;
  ros::Time        cur_time;
  ros::ros_steadytime(cur_time.sec, cur_time.nsec);
  const ros::Duration step = last_time - cur_time;

  // When the controller is first enabled (or after a mode switch), set error = 0
  if (is_first_) {
    error     = 0.0;
    is_first_ = false;
  }

  // Calculate integral term
  integral_ += cur_gains_->k_i * error * step.toSec();
  if (cur_gains_->has_i_clamp && integral_ > cur_gains_->i_clamp)
    integral_ = cur_gains_->i_clamp;
  else if (cur_gains_->has_i_clamp && integral_ < -cur_gains_->i_clamp)
    integral_ = -cur_gains_->i_clamp;

  // Calculate output
  float ff_term = cur_gains_->k_f * setpoint_;  // TODO: Plus intercept
  float p_term  = cur_gains_->k_p * error;
  float i_term  = integral_;
  float d_term  = cur_gains_->k_d * (error - last_error_) / step.toSec();
  output_       = ff_term + p_term + i_term + d_term;

  last_error_ = error;
  last_time   = cur_time;
}

float MultiPIDController::getOutput() const {
  if (mode_ == Mode::disabled)
    return 0.0;
  return output_;
}

float MultiPIDController::getOutput(float input) {
  update(input);
  return getOutput();
}

}  // namespace frc_robot_hw

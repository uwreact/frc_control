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

#include <frc_robot_hw/wpilib_hardware_templates.h>

namespace frc_robot_hw {

class MultiPIDController {
public:
  using PIDGains = hardware_template::PIDGains;

  enum class Mode { disabled, position, velocity, effort };

  MultiPIDController(const PIDGains& pos, const PIDGains& vel, const PIDGains& eff)
      : position_gains_(pos), velocity_gains_(vel), effort_gains_(eff), cur_gains_(nullptr) {
    reset();
  }

  bool setMode(Mode mode);
  bool setMode(const std::string& mode);

  void reset();

  void  setSetpoint(float sp);
  void  update(float input);
  float getOutput(float input);
  float getOutput() const;

private:
  float setpoint_;
  float integral_;
  float last_error_;
  bool  is_first_;
  float output_;

  Mode      mode_      = Mode::position;
  PIDGains* cur_gains_ = nullptr;  // TODO: Smart ptr
  PIDGains  position_gains_;
  PIDGains  velocity_gains_;
  PIDGains  effort_gains_;
};

}  // namespace frc_robot_hw

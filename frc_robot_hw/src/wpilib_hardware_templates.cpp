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

#include <boost/assign.hpp>
#include <frc_robot_hw/wpilib_hardware_templates.h>

namespace frc_robot_hw {
namespace hardware_template {


SimpleSpeedController::Type SimpleSpeedController::stringToType(const std::string& string) {

  // clang-format off
  if (string == "dmc60")                return Type::DMC60;
  else if (string == "jaguar")          return Type::Jaguar;
  else if (string == "pwm_talon_srx")   return Type::PWMTalonSRX;
  else if (string == "pwm_victor_spx")  return Type::PWMVictorSPX;
  else if (string == "sd540")           return Type::SD540;
  else if (string == "spark")           return Type::Spark;
  else if (string == "talon")           return Type::Talon;
  else if (string == "victor")          return Type::Victor;
  else if (string == "victor_sp")       return Type::VictorSP;
  else if (string == "nidec")           return Type::Nidec;
#if USE_CTRE
  else if (string == "can_victor_spx")  return Type::CANVictorSPX;
#endif
  // clang-format on
  else
    throw std::runtime_error("Invalid simple SpeedController type '" + string + "'.");
}

std::string SimpleSpeedController::typeToString(const SimpleSpeedController::Type& type) {
  switch (type) {
    // clang-format off
    case Type::DMC60:         return "dmc60";
    case Type::Jaguar:        return "jaguar";
    case Type::PWMTalonSRX:   return "pwm_talon_srx";
    case Type::PWMVictorSPX:  return "pwm_victor_spx";
    case Type::SD540:         return "sd540";
    case Type::Spark:         return "spark";
    case Type::Talon:         return "talon";
    case Type::Victor:        return "victor";
    case Type::VictorSP:      return "victor_sp";
    case Type::Nidec:         return "nidex";
#if USE_CTRE
    case Type::CANVictorSPX:  return "can_victor_spx";
#endif
      // clang-format on
      // Note: No default case - Generates compile-time error if a case is missed
  }
}

Relay::Direction Relay::stringToDirection(const std::string& string) {

  // clang-format off
  if (string == "both")         return Direction::kBoth;
  else if (string == "forward") return Direction::kForward;
  else if (string == "reverse") return Direction::kReverse;
  // clang-format on
  else
    throw std::runtime_error("Invalid relay direction '" + string + "', must be one of 'both', 'forward', 'reverse'.");
}

std::string Relay::directionToString(const Relay::Direction& direction) {
  switch (direction) {
    // clang-format off
    case Direction::kBoth:    return "both";
    case Direction::kForward: return "forward";
    case Direction::kReverse: return "reverse";
      // clang-format on
      // Note: No default case - Generates compile-time error if a case is missed
  }
}

#if USE_CTRE

const CANTalonSrx::FeedbackTypeBimap
    CANTalonSrx::FEEDBACK_TYPE_BIMAP = boost::assign::list_of<CANTalonSrx::FeedbackTypeBimap::relation>
    // clang-format off
    (FeedbackType::kNone,         "none")
    (FeedbackType::kQuadEncoder,  "quad_encoder")
    (FeedbackType::kAnalog,       "analog")
    (FeedbackType::kTachometer,   "tachometer")
    (FeedbackType::kPulseWidth,   "pulse_width");
// clang-format on

const CANTalonSrx::LimitSwitchModeBimap
    CANTalonSrx::LIMIT_SWITCH_MODE_BIMAP = boost::assign::list_of<CANTalonSrx::LimitSwitchModeBimap::relation>
    // clang-format off
    (LimitSwitchMode::kNone,            "none")
    (LimitSwitchMode::kNormallyOpen,    "normally_open")
    (LimitSwitchMode::kNormallyClosed,  "kNormallyClosed");
// clang-format on
#endif

}  // namespace hardware_template
}  // namespace frc_robot_hw

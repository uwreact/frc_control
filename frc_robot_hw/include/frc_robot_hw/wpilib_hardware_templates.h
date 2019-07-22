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

#include <boost/variant.hpp>
#include <ostream>
#include <string>

// TODO: This file is a mess

namespace frc_robot_hw {
namespace hardware_template {

struct PIDGains {
  double k_p;
  double k_i;
  double k_d;
  double k_f;
  double i_clamp;
  bool   has_i_clamp;
};

// "Smart" indicates that these controllers have built-in feedback
// TODO: Might need custom templates for these since they're smart and therefore have more config data
// TODO: Add field for feedback type. Internal, External, or None
struct SmartSpeedController {
  enum class Type {
  // CANJaguar,   ///< Luminary Micro / Vex Robotics Jaguar with CAN control. Deprecated.

#if USE_MINDSENSORS
  // CANSD540,    ///< Mindsensors SD540 with CAN control
#endif

#if USE_CTRE
    CANTalonSRX,  ///< CTRE Talon SRX with CAN control
#endif
  };

  /**
   * @brief Convert the string representation of the Type to its enum equivalent.
   * @throws std::runtime_error if the input string is not a valid Type
   */
  static Type stringToType(const std::string& string) {
    if (false) {
      // TODO: This is ugly :(
    }
#if USE_MINDSENSORS
    // else if (string == "can_sd540")
    //   return Type::CANSD540;
#endif
#if USE_CTRE
    else if (string == "can_talon_spx")
      return Type::CANTalonSRX;
#endif
    else
      throw std::runtime_error("Invalid smart SpeedController type '" + string + "'.");
  }

  /// Convert a Type enum to its string representation
  static std::string typeToString(const Type& type) {
    switch (type) {
#if USE_MINDSENSORS
      // case Type::CANSD540:      return "can_sd540";
#endif
#if USE_CTRE
      case Type::CANTalonSRX:
        return "can_talon_spx";
#endif
        // Note: No default case - Generates compile-time error if a case is missed
    }
  }

  Type type;      ///< The type of the controller, corresponding with WPILib & vendor SpeedControllers
  int  id;        ///< The ID/channel of the controller. Typically CAN id, based on controller type
  bool inverted;  ///< Whether to invert the direction of the motor
};

// "Simple" indicates that these controllers do not have feedback.
// TODO: Add field for feedback type. External, or None
struct SimpleSpeedController {
  enum class Type {
    // WPILib PWMSpeedControllers
    DMC60,         ///< Digilent DMC 60 (VictorSP clone)
    Jaguar,        ///< Luminary Micro / Vex Robotics Jaguar with PWM control
    PWMTalonSRX,   ///< CTRE Talon SRX with PWM control
    PWMVictorSPX,  ///< CTRE Victor SPX with PWM control
    SD540,         ///< Mindsensors SD540 with PWM control
    Spark,         ///< REV Robotics Spark
    Talon,         ///< CTRE Original Talon and TalonSR
    Victor,        ///< Vex Robotics Victor 888, or 884 if calibrated
    VictorSP,      ///< Vex Robotics Victor SP

    // WPILib Other SpeedControllers
    Nidec,  ///< Nidec Dynamo BLDC

  // Vendor SpeedControllers
#if USE_CTRE
    CANVictorSPX,  ///< CTRE Victor SPX with CAN control
#endif
  };

  /**
   * @brief Convert the string representation of the Type to its enum equivalent.
   * @throws std::runtime_error if the input string is not a valid Type
   */
  static Type stringToType(const std::string& string) {

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

  /// Convert a Type enum to its string representation
  static std::string typeToString(const Type& type) {
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

  Type        type;       ///< The type of the controller, corresponding with WPILib & vendor SpeedControllers
  int         id;         ///< The ID/channel of the controller. Can be PWM channel or CAN id, based on controller type
  int         dio_id;     ///< Only used by Nidec. The DIO channel
  bool        inverted;   ///< Whether to invert the direction of the motor
  std::string pdp;        ///< Name of PDP. Must be "none" or the name of one of the PDPs
  int         pdp_ch;     ///< Which PDP channel the motor controller is connected to. Used for current draw measurement
  double      k_eff;      ///< Scale of current to effort/torque, in N or Nm. Based on motor type and gearing.
  PIDGains    pos_gains;  ///< The set of PID gains for position control
  PIDGains    vel_gains;  ///< The set of PID gains for velocity control
  PIDGains    eff_gains;  ///< The set of PID gains for effort control
  bool        has_pos_gains;  ///< Whether the controller specified gains for position control
  bool        has_vel_gains;  ///< Whether the controller specified gains for velocity control
  bool        has_eff_gains;  ///< Whether the controller specified gains for effort control
};

struct Relay {
  enum class Direction { kBoth, kForward, kReverse };

  static Direction stringToDirection(const std::string& string) {
    if (string == "both")
      return Direction::kBoth;
    else if (string == "forward")
      return Direction::kForward;
    else if (string == "reverse")
      return Direction::kReverse;
    else
      throw std::runtime_error("Invalid relay direction '" + string
                               + "', must be one of 'both', 'forward', 'reverse'.");
  }

  static std::string directionToString(const Direction& direction) {
    switch (direction) {
      case Direction::kBoth:
        return "both";
      case Direction::kForward:
        return "forward";
      case Direction::kReverse:
        return "reverse";
        // Note: No default case - Generates compile-time error if a case is missed
    }
  }

  int       id;
  Direction direction;
};

struct Solenoid {
  int id;      ///< The ID of the channel on the PCM
  int pcm_id;  ///< The CAN ID of the PCM this Solenoid is connected to
};

struct DoubleSolenoid {
  int forward_id;  ///< The ID of the 'forward' channel on the PCM
  int reverse_id;  ///< The ID of the 'reverse' channel on the PCM
  int pcm_id;      ///< The CAN ID of the PCM this DoubleSolenoid is connected to
};

struct DigitalIO {
  std::string joint;     ///< The name of the joint this sensor describes
  int         id;        ///< The DIO channel
  bool        inverted;  ///< Whether `false` represents LOW (0v) or HIGH (5v)
};

struct AnalogIO {
  std::string joint;   ///< The name of the joint this sensor describes
  int         id;      ///< The AnalogIO channel
  double      scale;   ///< The full-scale value of the sensor
  double      offset;  ///< The offset value of the sensor
};

struct Encoder {
  std::string joint;               ///< The name of the joint this sensor describes
  int         ch_a;                ///< The DIO channel of the encoder's A input
  int         ch_b;                ///< The DIO channel of the encoder's B input
  double      distance_per_pulse;  ///< The distance travelled per pulse
  bool        inverted;            ///< Whether to invert the direction
  int         encoding;            ///< The encoding format. Must be 1, 2, or 4
};

struct BuiltInAccelerometer {
  std::string frame_id;  ///< For TF2 TODO: Doc better
  // TODO: Range? tbh not sure why this is a feature of the wpilib and if it's worth implementing
};

#if USE_KAUAI
struct NavX {
  std::string interface;  ///< The interface on which the NavX is connected. Must be 'i2c', 'serial', or 'spi'
  int         id;         ///< The port number on whichever interface is specified above
  std::string frame_id;   ///< For TF2 TODO: Doc better
};
#endif

#if USE_CTRE
struct PigeonIMU {
  boost::variant<int, std::string> interface;  ///< The interface on which the PigeonIMU is connected.
                                               ///< If an int, the CAN ID of the IMU.
                                               ///< If a string, the can_talon_srx name the IMU is riding on.
  std::string frame_id;                        ///< For TF2 TODO: Doc better
};
#endif

}  // namespace hardware_template
}  // namespace frc_robot_hw

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

#include <frc_robot_hw/frc_robot_hw.h>
#include <frc_robot_hw/pid_controller.h>

#include <thread>

#include <realtime_tools/realtime_publisher.h>

// Custom messages
#include <frc_msgs/DriverStationMode.h>
#include <frc_msgs/JoyArray.h>
#include <frc_msgs/JoyFeedback.h>
#include <frc_msgs/MatchData.h>
#include <frc_msgs/MatchTime.h>
#include <frc_msgs/RobotState.h>

// WPILib headers
#include <frc/DriverStation.h>
#include <frc/Joystick.h>

// WPILib & vendor sensors/actuators
#include <frc/AnalogAccelerometer.h>
#include <frc/AnalogInput.h>
#include <frc/AnalogOutput.h>
#include <frc/BuiltInAccelerometer.h>
#include <frc/Compressor.h>
#include <frc/DMC60.h>
#include <frc/DigitalInput.h>
#include <frc/DigitalOutput.h>
#include <frc/DoubleSolenoid.h>
#include <frc/Encoder.h>
#include <frc/Jaguar.h>
#include <frc/NidecBrushless.h>
#include <frc/PWMTalonSRX.h>
#include <frc/PWMVictorSPX.h>
#include <frc/PowerDistributionPanel.h>
#include <frc/Relay.h>
#include <frc/SD540.h>
#include <frc/Servo.h>
#include <frc/Solenoid.h>
#include <frc/Spark.h>
#include <frc/Talon.h>
#include <frc/Victor.h>
#include <frc/VictorSP.h>

// Note: We will use WPI_ prefixed versions of the CTRE controllers. They are almost identical but add the standard
// speed controller watchdog all WPILib speed controllers use, as well as allowing for the controller to be put on the
// SmartDashboard. It also makes them play nicely with other WPILib stuff such as its PIDs
#if USE_CTRE
#include <ctre/phoenix/motorcontrol/can/WPI_TalonSRX.h>
#include <ctre/phoenix/motorcontrol/can/WPI_VictorSPX.h>
#include <ctre/phoenix/sensors/PigeonIMU.h>
#endif

#if USE_KAUAI
#include <AHRS.h>
#endif

#if USE_MINDSENSORS
// #include <mindsensors/CANLIGHT.h>
// #include <mindsensors/CANSD540.h>
#endif

namespace frc_robot_hw {

/// FRC Robot HW for real robots running on the RoboRIO
class FRCRobotHWReal : public FRCRobotHW {
public:
  FRCRobotHWReal() : FRCRobotHW("frc_robot_hw_real") {}

  /**
   * @brief Initialize the HAL
   *
   * Note: This MUST occur before creating WPILib objects! See
   * https://www.chiefdelphi.com/forums/showpost.php?p=1640943&postcount=3
   */
  bool initHAL();

  // Overrides
  bool init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) override;
  void read(const ros::Time& time, const ros::Duration& period) override;
  void write(const ros::Time& time, const ros::Duration& period) override;
  void doSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
                const std::list<hardware_interface::ControllerInfo>& stop_list) override;

  inline void setPublishRate(double rate) { publish_period_ = ros::Duration(1.0 / rate); }

private:
  /**
   * @brief Maintain a loop to get data from the driver station.
   *
   * ** Blocking! **
   *
   * Used get data from driver station and appear as a ready robot to the HAL/DS/FMS.
   * Reads joystick data, game-specific-data, and network tables.
   */
  void runHAL();

  void joyFeedbackCallback(const frc_msgs::JoyFeedbackConstPtr& msg);

  // Maps of the WPILib objects used to interact with the HAL
  // TODO: Once AnalogAccelerometer extends Accelerometer, merge arrays
  std::map<std::string, std::unique_ptr<frc::SpeedController>>        simple_speed_controllers_;
  std::map<std::string, std::unique_ptr<MultiPIDController>>          simple_speed_controller_pids_;
  std::map<std::string, std::unique_ptr<frc::PowerDistributionPanel>> pdps_;
  std::map<std::string, std::unique_ptr<frc::Servo>>                  servos_;
  std::map<std::string, std::unique_ptr<frc::Relay>>                  relays_;
  std::map<std::string, std::unique_ptr<frc::Solenoid>>               solenoids_;
  std::map<std::string, std::unique_ptr<frc::DoubleSolenoid>>         double_solenoids_;
  std::map<std::string, std::unique_ptr<frc::Compressor>>             compressors_;
  std::map<std::string, std::unique_ptr<frc::DigitalInput>>           digital_inputs_;
  std::map<std::string, std::unique_ptr<frc::DigitalOutput>>          digital_outputs_;
  std::map<std::string, std::unique_ptr<frc::AnalogInput>>            analog_inputs_;
  std::map<std::string, std::unique_ptr<frc::AnalogOutput>>           analog_outputs_;
  std::map<std::string, std::unique_ptr<frc::Encoder>>                encoders_;
  std::map<std::string, std::unique_ptr<frc::Accelerometer>>          generic_accelerometers_;
  std::map<std::string, std::unique_ptr<frc::AnalogAccelerometer>>    analog_accelerometers_;
#if USE_KAUAI
  std::map<std::string, std::unique_ptr<AHRS>> navxs_;
#endif
#if USE_CTRE
  std::map<std::string, std::unique_ptr<ctre::phoenix::motorcontrol::can::WPI_TalonSRX>> can_talon_srxs_;
  std::map<std::string, std::unique_ptr<ctre::phoenix::sensors::PigeonIMU>>              pigeons_;
#endif

  std::thread hal_thread_;
  bool        robot_code_ready_ = false;

  frc::Joystick sticks_[frc::DriverStation::kJoystickPorts] {frc::Joystick(0),
                                                             frc::Joystick(1),
                                                             frc::Joystick(2),
                                                             frc::Joystick(3),
                                                             frc::Joystick(4),
                                                             frc::Joystick(5)};

  ros::Duration publish_period_;

  realtime_tools::RealtimePublisher<frc_msgs::DriverStationMode> ds_mode_pub_;
  realtime_tools::RealtimePublisher<frc_msgs::JoyArray>          joy_pub_;
  realtime_tools::RealtimePublisher<frc_msgs::MatchData>         match_data_pub_;
  realtime_tools::RealtimePublisher<frc_msgs::MatchTime>         match_time_pub_;
  realtime_tools::RealtimePublisher<frc_msgs::RobotState>        robot_state_pub_;
  ros::Subscriber                                                joy_feedback_sub_;
};

}  // namespace frc_robot_hw

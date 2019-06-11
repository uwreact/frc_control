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

#pragma once

// ROS
#include <ros/ros.h>
#include <urdf/model.h>

// ROS Controls
#include <hardware_interface/imu_sensor_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include <transmission_interface/transmission.h>
#include <transmission_interface/transmission_info.h>
#include <transmission_interface/transmission_interface.h>

// ROS Messages
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <std_srvs/Trigger.h>

// TinyXML
#include <tinyxml2.h>

// Custom
#include <analog_controller/analog_command_interface.h>
#include <binary_state_controller/binary_command_interface.h>
#include <pdp_state_controller/pdp_state_interface.h>
#include <ternary_state_controller/ternary_command_interface.h>

#include <frc_robot_hw/custom_hardware_interfaces.h>
#include <frc_robot_hw/wpilib_hardware_templates.h>

namespace frc_robot_hw {

/// A generic template for FRC robot HW interfaces, both real and simulated
class FRCRobotHW : public hardware_interface::RobotHW {
protected:
  explicit FRCRobotHW(const std::string& name) : name_(std::move(name)) {}

public:
  FRCRobotHW() : FRCRobotHW("frc_robot_hw") {}

  using TernaryState = hardware_interface::TernaryStateHandle::TernaryState;
  using PDPState     = hardware_interface::PDPStateHandle::PDPState;

  /// Joint feedback data
  struct JointState {
    double pos;
    double vel;
    double eff;
  };

  /// Joint command data
  struct JointCmd {
    enum class Type {
      kNone,  ///< No command
      kPos,   ///< Command is position, in rad or m
      kVel,   ///< Command is velocity, in rad/s or m/s
      kEff,   ///< Command is effort, in N or N*m
      kVolt   ///< Command is voltage, from about -12.0 to 12.0
    };

    double data;                ///< The commanded data
    Type   type = Type::kNone;  ///< The command type
  };

  /// A state (position) and its first derivative
  struct RateState {
    double state;  ///< Current state, eg. position or angle
    double rate;   ///< Current rate, eg. velocity or angular velocity (dstate/dt)
  };

  /// IMU sensor data
  struct ImuData {
    double orientation[4];                     ///< Quaternion (x,y,z,w)
    double orientation_covariance[9];          ///< Row major 3x3 matrix about (x,y,z)
    double angular_velocity[3];                ///< Vector (x,y,z), in deg/s
    double angular_velocity_covariance[9];     ///< Row major 3x3 matrix about (x,y,z)
    double linear_acceleration[3];             ///< Vector (x,y,z), in m/s^2
    double linear_acceleration_covariance[9];  ///< Row major 3x3 matrix about (x,y,z)
  };


  /**
   * @brief Read the robot model from the URDF on the parameter server
   *
   * This URDF model will probably be useful for TF2 stuff but idk.
   * We currently don't use it. We could also use it for joint limits.
   */
  void loadURDF(const ros::NodeHandle& nh, const std::string& param_name);

  /**
   * @brief Read the joint descriptions from the parameter server
   *
   * The joint descriptions will define what type of controller each joint in the URDF uses.
   * For example, it could specify a CANTalon with builtin encoder feedback, a VictorSP with
   * an encoder on the specified port, etc
   */
  void loadJoints(const ros::NodeHandle& nh, const std::string& param_name);

  /**
   * @brief Update all the Joint States of the robot
   *
   * A complete joint state contains joint position, velocity, and effort. None of the FRC sensors
   * offer this complete set of data, therefore we must combine the data here so that a transform tree
   * of the robot can be generated with tf2.
   */
  void updateRobotState();

  void registerTransmissions();

  // Overrides
  virtual bool init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) override;
  virtual void read(const ros::Time& time, const ros::Duration& period) override;
  virtual void write(const ros::Time& time, const ros::Duration& period) override;
  void         doSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
                        const std::list<hardware_interface::ControllerInfo>& stop_list) override;

  /// Enforce limits for all values before writing.
  virtual void enforceLimits(ros::Duration& period) {};

  /// Get the short name of the class.
  inline std::string getName() const { return name_; }

protected:
  // Model of the robot used for TF2 transformations and for converting sensor data into joint data
  // For example, parsing DigitalInputs into prismatic joint positions (like Solenoids)
  tinyxml2::XMLDocument urdf_xml_;
  urdf::Model           urdf_model_;

  // Short name of this class
  const std::string name_;

  // Hardware state interfaces
  hardware_interface::JointStateInterface   joint_state_interface_;
  hardware_interface::PDPStateInterface     pdp_state_interface_;
  hardware_interface::AnalogStateInterface  analog_state_interface_;
  hardware_interface::BinaryStateInterface  binary_state_interface_;
  hardware_interface::TernaryStateInterface ternary_state_interface_;
  hardware_interface::ImuSensorInterface    imu_sensor_interface_;

  // Transmission interfaces
  // Convert actuator state to joint state
  // transmission_interface::ActuatorToJointStateInterface act_to_jnt_state_interface_;
  // Convert joint state to actuator state
  // transmission_interface::JointToActuatorStateInterface jnt_to_act_state_interface_;

  // Hardware command iterfaces
  hardware_interface::PositionJointInterface  joint_position_command_interface_;  // Position
  hardware_interface::VelocityJointInterface  joint_velocity_command_interface_;  // Veloicty
  hardware_interface::EffortJointInterface    joint_effort_command_interface_;    // Effort
  hardware_interface::VoltageJointInterface   joint_voltage_command_interface_;   // Voltage (-12ish to 12ish)
  hardware_interface::AnalogCommandInterface  analog_command_interface_;
  hardware_interface::BinaryCommandInterface  binary_command_interface_;
  hardware_interface::TernaryCommandInterface ternary_command_interface_;

  // Sensors and actuator templates
  // TODO: Change servo_templates_ to struct containing scale?
  std::map<std::string, hardware_template::SmartSpeedController>  smart_speed_controller_templates_;
  std::map<std::string, hardware_template::SimpleSpeedController> simple_speed_controller_templates_;
  std::map<std::string, int>                                      pdp_templates_;
  std::map<std::string, int>                                      servo_templates_;
  std::map<std::string, hardware_template::Relay>                 relay_templates_;
  std::map<std::string, hardware_template::Solenoid>              solenoid_templates_;
  std::map<std::string, hardware_template::DoubleSolenoid>        double_solenoid_templates_;
  std::map<std::string, int>                                      compressor_templates_;
  std::map<std::string, hardware_template::DigitalIO>             digital_input_templates_;
  std::map<std::string, hardware_template::DigitalIO>             digital_output_templates_;
  std::map<std::string, hardware_template::AnalogIO>              analog_input_templates_;
  std::map<std::string, hardware_template::AnalogIO>              analog_output_templates_;
  std::map<std::string, hardware_template::Encoder>               encoder_templates_;
// TODO: More IMUs! AnalogGyro, AnalogAccelerometer, ADXL/ADXR series, and BuiltInAccelerometer
#if USE_KAUAI
  std::map<std::string, hardware_template::NavX> navx_templates_;
#endif
#if USE_CTRE
  std::map<std::string, hardware_template::PigeonIMU> pigeon_templates_;
#endif

  // Transmissions
  // std::map<std::string, transmission_interface::TransmissionInfo> transmission_info_;
  // std::map<std::string, std::unique_ptr<transmission_interface::Transmission>> transmissions_;

  // Transmission data
  // std::map<std::string, transmission_interface::ActuatorData> actuator_state_data_;
  // std::map<std::string, transmission_interface::ActuatorData> actuator_cmd_data_;
  // std::map<std::string, transmission_interface::JointData> joint_state_data_;
  // std::map<std::string, transmission_interface::JointData> joint_cmd_data_;

  // States
  // std::map<std::string, JointState> actuator_states_;///< Transmission stuff
  std::map<std::string, JointState>   joint_states_;    ///< State of SpeedControllers, (Servos???)
  std::map<std::string, PDPState>     pdp_states_;      ///< State of PowerDistributionPanels
  std::map<std::string, RateState>    rate_states_;     ///< State of AnalogInput/Outputs, Encoders, etc
  std::map<std::string, bool>         binary_states_;   ///< State of DigitalInputs/Outputs, Solenoids, etc
  std::map<std::string, TernaryState> ternary_states_;  ///< State of Relays, DoubleSolenoids, etc
  std::map<std::string, ImuData>      imu_states_;      ///< State of IMUs such as PigeonIMU, NavX

  // Commands
  // TODO: Servos as Joint or Analog commands?
  // std::map<std::string, double> actuator_commands_;    ///< Transmission stuff
  std::map<std::string, JointCmd>     joint_commands_;    ///< Commands for SpeedControllers, Servos
  std::map<std::string, double>       analog_commands_;   ///< Commands for AnalogOutputs
  std::map<std::string, bool>         binary_commands_;   ///< Commands for DigitalOutputs, Solenoids
  std::map<std::string, TernaryState> ternary_commands_;  ///< Commands for Relays, DoubleSolenoids

private:
  bool validateJointParamMember(XmlRpc::XmlRpcValue&             value,
                                const std::string&               member,
                                const XmlRpc::XmlRpcValue::Type& type,
                                const bool                       warn_exist = true,
                                const bool                       warn_type  = true);
};

}  // namespace frc_robot_hw

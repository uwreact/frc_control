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

#include <frc_robot_hw/frc_robot_hw_real.h>

// WPILib headers
#include <frc/RobotController.h>
#include <hal/HAL.h>

#include <tf2/LinearMath/Quaternion.h>

namespace frc_robot_hw {

bool FRCRobotHWReal::initHAL() {
  if (!HAL_Initialize(500, 0)) {
    ROS_FATAL_NAMED(name_, "HAL could not be initialized!");
    return false;
  }

  HAL_Report(HALUsageReporting::kResourceType_Language, HALUsageReporting::kLanguage_CPlusPlus);
  HAL_Report(HALUsageReporting::kResourceType_Framework, HALUsageReporting::kFramework_ROS);

  ROS_INFO_NAMED(name_, "HAL successfully initialized");
  return true;
}

void FRCRobotHWReal::runHAL() {

  // Wait until all controllers have been loaded by the controller manager
  // before starting the HAL thread and displaying 'Robot code ready'
  while (!robot_code_ready_) {
    ros::Rate(10).sleep();
  }

  // Tell the DS that the robot is ready to be enabled ('Robot code ready')
  ROS_INFO("Robot code ready");
  HAL_ObserveUserProgramStarting();

  frc::DriverStation& ds = frc::DriverStation::GetInstance();
  ros::Time           time;

  // We use realtime publishers here because we must publish ds_mode and joysticks deterministically since they are
  // safety-related. We populate the messages here in the HAL thread, but punt the actual writing to the socket to
  // another thread so that we don't delay this thread.
  while (ros::ok()) {

    // Publish joysticks
    time = ros::Time::now();
    static ros::Time last_joy_pub_time;
    if (!publish_period_.isZero() && last_joy_pub_time + publish_period_ < time && joy_pub_.trylock()) {
      last_joy_pub_time          = time;
      joy_pub_.msg_.header.stamp = time;

      joy_pub_.msg_.sticks.resize(frc::DriverStation::kJoystickPorts);
      joy_pub_.msg_.types.resize(frc::DriverStation::kJoystickPorts);
      joy_pub_.msg_.names.resize(frc::DriverStation::kJoystickPorts);

      for (unsigned i = 0; i < frc::DriverStation::kJoystickPorts; i++) {
        // NOTE(matt.reynolds): Driver station axes are shortened from double (float64)
        // to float (float32) to be packed in sensor_msgs::Joy.

        sensor_msgs::Joy stick;
        stick.header.stamp = time;

        stick.axes.resize(ds.GetStickAxisCount(i));
        for (unsigned axis = 0; axis < ds.GetStickAxisCount(i); axis++)
          stick.axes[axis] = ds.GetStickAxis(i, axis);

        // Note: Buttons, unlike axes, are indexed from 1 rather than 0
        stick.buttons.resize(ds.GetStickButtonCount(i));
        for (unsigned button = 0; button < ds.GetStickButtonCount(i); button++)
          stick.buttons[button] = ds.GetStickButton(i, button + 1);

        // TODO(matt.reynolds): Ensure POV hat is covered. If not, append it to buttons[] and axes[] or add new array
        // See https://github.com/uwreact/frc_control/issues/51

        joy_pub_.msg_.sticks[i] = stick;
        joy_pub_.msg_.types[i]  = ds.GetJoystickType(i);
        joy_pub_.msg_.names[i]  = ds.GetJoystickName(i);
      }

      joy_pub_.unlockAndPublish();
    }


    // Publish match data
    time = ros::Time::now();
    static ros::Time last_match_data_pub_time;
    if (!publish_period_.isZero() && last_match_data_pub_time + publish_period_ < time && match_data_pub_.trylock()) {
      last_match_data_pub_time          = time;
      match_data_pub_.msg_.header.stamp = time;

      match_data_pub_.msg_.game_specific_message = ds.GetGameSpecificMessage();
      match_data_pub_.msg_.event_name            = ds.GetEventName();
      match_data_pub_.msg_.match_type            = ds.GetMatchType();  // kNone, kPractice, kQualification, kElimination
      match_data_pub_.msg_.match_number          = ds.GetMatchNumber();
      match_data_pub_.msg_.replay_number         = ds.GetReplayNumber();
      match_data_pub_.msg_.alliance              = ds.GetAlliance();  // kRed, kBlue, or kInvalid
      match_data_pub_.msg_.location              = ds.GetLocation();  // 1, 2, or 3. 0 if invalid

      match_data_pub_.unlockAndPublish();
    }


    // Publish match time
    // APPROXIMATE remaining time in current period (auto or teleop), in seconds.
    // Note: The FMS does not report official match time to robots, thus this may be imprecise.
    time = ros::Time::now();
    static ros::Time last_match_time_pub_time;
    if (!publish_period_.isZero() && last_match_time_pub_time + publish_period_ < time && match_time_pub_.trylock()) {
      last_match_time_pub_time          = time;
      match_time_pub_.msg_.header.stamp = time;

      match_time_pub_.msg_.remaining_time = ds.GetMatchTime();

      match_time_pub_.unlockAndPublish();
    }


    // Publish driver station mode
    time = ros::Time::now();
    static ros::Time last_ds_mode_pub_time;
    if (!publish_period_.isZero() && last_ds_mode_pub_time + publish_period_ < time && ds_mode_pub_.trylock()) {
      last_ds_mode_pub_time          = time;
      ds_mode_pub_.msg_.header.stamp = time;

      // TODO(matt.reynolds): Estop
      if (ds.IsDisabled())
        ds_mode_pub_.msg_.mode = frc_msgs::DriverStationMode::MODE_DISABLED;
      else if (ds.IsOperatorControl())
        ds_mode_pub_.msg_.mode = frc_msgs::DriverStationMode::MODE_OPERATOR;
      else if (ds.IsAutonomous())
        ds_mode_pub_.msg_.mode = frc_msgs::DriverStationMode::MODE_AUTONOMOUS;
      else if (ds.IsTest())
        ds_mode_pub_.msg_.mode = frc_msgs::DriverStationMode::MODE_TEST;
      else
        ds_mode_pub_.msg_.mode = frc_msgs::DriverStationMode::MODE_DISABLED;

      ds_mode_pub_.msg_.is_ds_attached  = ds.IsDSAttached();
      ds_mode_pub_.msg_.is_fms_attached = ds.IsFMSAttached();

      ds_mode_pub_.unlockAndPublish();
    }


    // Publish robot state
    time = ros::Time::now();
    static ros::Time last_robot_state_pub_time;
    if (!publish_period_.isZero() && last_robot_state_pub_time + publish_period_ < time && robot_state_pub_.trylock()) {
      last_robot_state_pub_time          = time;
      robot_state_pub_.msg_.header.stamp = time;

      robot_state_pub_.msg_.fpga_version    = frc::RobotController::GetFPGAVersion();
      robot_state_pub_.msg_.fpga_revision   = frc::RobotController::GetFPGARevision();
      robot_state_pub_.msg_.fpga_time       = frc::RobotController::GetFPGATime();
      robot_state_pub_.msg_.user_button     = frc::RobotController::GetUserButton();
      robot_state_pub_.msg_.sys_active      = frc::RobotController::IsSysActive();
      robot_state_pub_.msg_.browned_out     = frc::RobotController::IsBrownedOut();
      robot_state_pub_.msg_.input_voltage   = frc::RobotController::GetInputVoltage();
      robot_state_pub_.msg_.input_current   = frc::RobotController::GetInputCurrent();
      robot_state_pub_.msg_.voltage_3v3     = frc::RobotController::GetVoltage3V3();
      robot_state_pub_.msg_.current_3v3     = frc::RobotController::GetCurrent3V3();
      robot_state_pub_.msg_.enabled_3v3     = frc::RobotController::GetEnabled3V3();
      robot_state_pub_.msg_.fault_count_3v3 = frc::RobotController::GetFaultCount3V3();
      robot_state_pub_.msg_.voltage_5v      = frc::RobotController::GetVoltage5V();
      robot_state_pub_.msg_.current_5v      = frc::RobotController::GetCurrent5V();
      robot_state_pub_.msg_.enabled_5v      = frc::RobotController::GetEnabled5V();
      robot_state_pub_.msg_.fault_count_5v  = frc::RobotController::GetFaultCount5V();
      robot_state_pub_.msg_.voltage_6v      = frc::RobotController::GetVoltage6V();
      robot_state_pub_.msg_.current_6v      = frc::RobotController::GetCurrent6V();
      robot_state_pub_.msg_.enabled_6v      = frc::RobotController::GetEnabled6V();
      robot_state_pub_.msg_.fault_count_6v  = frc::RobotController::GetFaultCount6V();

      const frc::CANStatus& can_status                         = frc::RobotController::GetCANStatus();
      robot_state_pub_.msg_.can_status.percent_bus_utilization = can_status.percentBusUtilization;
      robot_state_pub_.msg_.can_status.bus_off_count           = can_status.busOffCount;
      robot_state_pub_.msg_.can_status.tx_full_count           = can_status.txFullCount;
      robot_state_pub_.msg_.can_status.receive_error_count     = can_status.receiveErrorCount;
      robot_state_pub_.msg_.can_status.transmit_error_count    = can_status.transmitErrorCount;

      robot_state_pub_.msg_.battery_voltage = ds.GetBatteryVoltage();

      robot_state_pub_.unlockAndPublish();
    }

    // TODO: Publish NetworkTables.

    // Wait for driver station data so the loop doesn't hog the CPU
    // Timeout after 100ms so that data is published even when DS is not connected
    ds.WaitForData(0.1);
  }
}

void FRCRobotHWReal::joyFeedbackCallback(const frc_msgs::JoyFeedbackConstPtr& msg) {
  using RumbleType = frc::GenericHID::RumbleType;

  if (msg->left_rumbles.size() != msg->right_rumbles.size() || msg->left_rumbles.size() != msg->outputs.size()) {
    ROS_WARN_NAMED(name_,
                   "Invalid joystick feeedback! Both rumble vectors and the output vector must be the same size.");
  } else if (msg->left_rumbles.size() > 6) {
    ROS_WARN_NAMED(name_, "Invalid joystick feedback! More than 6 joysticks specified.");
  } else {
    for (int i = 0; i < msg->left_rumbles.size(); i++) {
      sticks_[i].SetRumble(RumbleType::kLeftRumble, msg->left_rumbles[i]);
      sticks_[i].SetRumble(RumbleType::kRightRumble, msg->right_rumbles[i]);
      sticks_[i].SetOutputs(msg->outputs[i]);
    }
  }
}

// Setup HAL interaction. Create WPILib objects for each specified sensor and actuator.
bool FRCRobotHWReal::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) {
  if (!FRCRobotHW::init(root_nh, robot_hw_nh))
    return false;

  if (!initHAL())
    return false;

  double publish_freq;
  robot_hw_nh.param("frc_msg_publish_frequency", publish_freq, 10.0);
  setPublishRate(publish_freq);

  // Setup the realtime publishers
  ds_mode_pub_.init(root_nh, "frc/ds_mode", 10);
  joy_pub_.init(root_nh, "frc/joys", 10);
  match_data_pub_.init(root_nh, "frc/match_data", 10);
  match_time_pub_.init(root_nh, "frc/match_time", 10);
  robot_state_pub_.init(root_nh, "frc/robot_state", 10);

  // Setup the subscribers
  joy_feedback_sub_ = root_nh.subscribe("frc/joy_feedback", 1, &FRCRobotHWReal::joyFeedbackCallback, this);


  // =*=*=*=*=*=*=*= Sensors =*=*=*=*=*=*=*=

  // Create AnalogInputs
  // TODO: Allow configuration of bits, accumulator
  for (const auto& pair : analog_input_templates_) {
    // clang-format off
    ROS_DEBUG_STREAM_NAMED(name_, "Creating WPILib AnalogInput " << pair.first
                                  << " on channel " << pair.second.id
                                  << " w/ scale " << pair.second.scale     // Non-wpilib feature
                                  << " w/ offset " << pair.second.offset); // Non-wpilib feature
    // clang-format on
    analog_inputs_[pair.first] = std::make_unique<frc::AnalogInput>(pair.second.id);
  }

  // Create DigitalInputs
  for (const auto& pair : digital_input_templates_) {
    // clang-format off
    ROS_DEBUG_STREAM_NAMED(name_, "Creating WPILib DigitalInput " << pair.first
                                  << " on channel " << pair.second.id
                                  << " w/ inverted=" << pair.second.inverted); // Non-wpilib feature
    // clang-format on
    digital_inputs_[pair.first] = std::make_unique<frc::DigitalInput>(pair.second.id);
  }

  // Create Encoders
  for (const auto& pair : encoder_templates_) {
    // clang-format off
    ROS_DEBUG_STREAM_NAMED(name_, "Creating WPILib Encoder " << pair.first
                                  << " on channel A " << pair.second.ch_a
                                  << " and channel B " << pair.second.ch_b
                                  << " w/ distance per pulse " << pair.second.distance_per_pulse
                                  << " and encoding x" << pair.second.encoding
                                  << " and inverted " << pair.second.inverted);
    // clang-format on
    frc::CounterBase::EncodingType encoding;
    if (pair.second.encoding == 1)
      encoding = frc::CounterBase::EncodingType::k1X;
    else if (pair.second.encoding == 2)
      encoding = frc::CounterBase::EncodingType::k2X;
    else
      encoding = frc::CounterBase::EncodingType::k4X;

    encoders_[pair.first] = std::make_unique<frc::Encoder>(pair.second.ch_a,
                                                           pair.second.ch_b,
                                                           pair.second.inverted,
                                                           encoding);
    encoders_[pair.first]->SetDistancePerPulse(pair.second.distance_per_pulse);
  }

  // Create built-in accelerometers
  for (const auto& pair : built_in_accelerometer_templates_) {
    // clang-format off
    ROS_DEBUG_STREAM_NAMED(name_, "Creating WPILib built-in accelerometer " << pair.first
                                  << " with tf frame " << pair.second.frame_id); // Non-wpilib feature
    // clang-format on

    generic_accelerometers_[pair.first] = std::make_unique<frc::BuiltInAccelerometer>();
  }

  // Create navXs
#if USE_KAUAI
  for (const auto& pair : navx_templates_) {
    // clang-format off
    ROS_DEBUG_STREAM_NAMED(name_, "Creating WPILib NavX-MXP " << pair.first
                                  << " on interface " << pair.second.interface
                                  << " at id " << pair.second.id
                                  << " with tf frame " << pair.second.frame_id); // Non-wpilib feature
    // clang-format on

    if (pair.second.interface == "i2c")
      navxs_[pair.first] = std::make_unique<AHRS>((frc::I2C::Port) pair.second.id);
    else if (pair.second.interface == "serial")
      navxs_[pair.first] = std::make_unique<AHRS>((frc::SerialPort::Port) pair.second.id);
    else
      navxs_[pair.first] = std::make_unique<AHRS>((frc::SPI::Port) pair.second.id);
  }
#endif

  // Create PigeonIMUs
#if USE_CTRE
  for (const auto& pair : pigeon_templates_) {
    // clang-format off
    ROS_DEBUG_STREAM_NAMED(name_, "Creating WPILib PigeonIMU " << pair.first
                                  << " on CAN ID or TalonSRX name " << pair.second.interface
                                  << " with tf frame " << pair.second.frame_id); // Non-wpilib feature
    // clang-format on

    using PigeonIMU    = ctre::phoenix::sensors::PigeonIMU;
    using PigeonIMUPtr = std::unique_ptr<PigeonIMU>;

    struct Visitor : public boost::static_visitor<PigeonIMUPtr> {
      PigeonIMUPtr operator()(const int& i) const { return std::make_unique<PigeonIMU>(i); }
      PigeonIMUPtr operator()(const std::string& s) const { return std::make_unique<PigeonIMU>(0); }
      // TODO: Get associated TalonSRX by name
    };

    pigeons_[pair.first] = boost::apply_visitor(Visitor(), pair.second.interface);
  }
#endif


  // =*=*=*=*=*=*=*= Actuators =*=*=*=*=*=*=*=

  // Create AnalogOutputs
  for (const auto& pair : analog_output_templates_) {
    // clang-format off
    ROS_DEBUG_STREAM_NAMED(name_, "Creating WPILib AnalogOutput " << pair.first
                                  << " on channel " << pair.second.id
                                  << " w/ scale " << pair.second.scale     // Non-wpilib feature
                                  << " w/ offset " << pair.second.offset); // Non-wpilib feature
    // clang-format on
    analog_outputs_[pair.first] = std::make_unique<frc::AnalogOutput>(pair.second.id);
  }

  // Create DigitalOutputs
  for (const auto& pair : digital_output_templates_) {
    // clang-format off
    ROS_DEBUG_STREAM_NAMED(name_, "Creating WPILib DigitalOutput " << pair.first
                                  << " on channel " << pair.second.id
                                  << " w/ inverted=" << pair.second.inverted); // Non-wpilib feature
    // clang-format on
    digital_outputs_[pair.first] = std::make_unique<frc::DigitalOutput>(pair.second.id);
  }

  // Create DoubleSolenoids
  for (const auto& pair : double_solenoid_templates_) {
    // clang-format off
    ROS_DEBUG_STREAM_NAMED(name_, "Creating WPILib DoubleSolenoid " << pair.first
                                  << " on channels fwd:" << pair.second.forward_id
                                  << " rev:" << pair.second.reverse_id
                                  << " on PCM  " << pair.second.pcm_id);
    // clang-format on
    double_solenoids_[pair.first] = std::make_unique<frc::DoubleSolenoid>(pair.second.pcm_id,
                                                                          pair.second.forward_id,
                                                                          pair.second.reverse_id);
  }

  // Create Relays
  for (const auto& pair : relay_templates_) {
    using Direction                     = hardware_template::Relay::Direction;
    const std::string& direction_string = hardware_template::Relay::directionToString(pair.second.direction);
    // clang-format off
    ROS_DEBUG_STREAM_NAMED(name_, "Creating WPILib Relay " << pair.first
                                  << " on channel " << pair.second.id
                                  << " w/ direction " << direction_string);
    // clang-format on
    frc::Relay::Direction dir;
    switch (pair.second.direction) {
      case Direction::kBoth:
        dir = frc::Relay::Direction::kBothDirections;
        break;
      case Direction::kForward:
        dir = frc::Relay::Direction::kForwardOnly;
        break;
      case Direction::kReverse:
        dir = frc::Relay::Direction::kReverseOnly;
        break;
    }
    relays_[pair.first] = std::make_unique<frc::Relay>(pair.second.id, dir);
  }

  // Create Servos
  for (const auto& pair : servo_templates_) {
    // clang-format off
    ROS_DEBUG_STREAM_NAMED(name_, "Creating WPILib Servo " << pair.first
                                  << " on channel " << pair.second);
    // clang-format on
    servos_[pair.first] = std::make_unique<frc::Servo>(pair.second);
  }

  // Create Solenoids
  for (const auto& pair : solenoid_templates_) {
    // clang-format off
    ROS_DEBUG_STREAM_NAMED(name_, "Creating WPILib Solenoid " << pair.first
                                  << " on channel " << pair.second.id
                                  << " on PCM  " << pair.second.pcm_id);
    // clang-format on
    solenoids_[pair.first] = std::make_unique<frc::Solenoid>(pair.second.pcm_id, pair.second.id);
  }

  // Create simple SpeedControllers
  for (const auto& pair : simple_speed_controller_templates_) {
    const std::string& type_string = hardware_template::SimpleSpeedController::typeToString(pair.second.type);
    // clang-format off
    ROS_DEBUG_STREAM_NAMED(name_, "Creating " << type_string
                                  << " SpeedController " << pair.first
                                  << " on channel " << pair.second.id
                                  << " w/ inverted=" << pair.second.inverted);
    // clang-format on
    using Type = hardware_template::SimpleSpeedController::Type;

    const int id     = pair.second.id;
    const int dio_id = pair.second.dio_id;

    std::unique_ptr<frc::SpeedController> controller;
    switch (pair.second.type) {
        // clang-format off
      case Type::DMC60:         controller = std::make_unique<frc::DMC60>(id);        break;
      case Type::Jaguar:        controller = std::make_unique<frc::Jaguar>(id);       break;
      case Type::PWMTalonSRX:   controller = std::make_unique<frc::PWMTalonSRX>(id);  break;
      case Type::PWMVictorSPX:  controller = std::make_unique<frc::PWMVictorSPX>(id); break;
      case Type::SD540:         controller = std::make_unique<frc::SD540>(id);        break;
      case Type::Spark:         controller = std::make_unique<frc::Spark>(id);        break;
      case Type::Talon:         controller = std::make_unique<frc::Talon>(id);        break;
      case Type::Victor:        controller = std::make_unique<frc::Victor>(id);       break;
      case Type::VictorSP:      controller = std::make_unique<frc::VictorSP>(id);     break;
      case Type::Nidec:         controller = std::make_unique<frc::NidecBrushless>(id, dio_id); break;
#if USE_CTRE
      case Type::CANVictorSPX:  controller = std::make_unique<ctre::phoenix::motorcontrol::can::WPI_VictorSPX>(id); break;
#endif
        // clang-format on
    }

    simple_speed_controllers_[pair.first]     = std::move(controller);
    simple_speed_controller_pids_[pair.first] = std::make_unique<MultiPIDController>(pair.second.pos_gains,
                                                                                     pair.second.vel_gains,
                                                                                     pair.second.eff_gains);
  }


#if USE_CTRE

  // Create CANTalonSRXs
  for (const auto& pair : can_talon_srx_templates_) {
    // clang-format off
    ROS_DEBUG_STREAM_NAMED(name_, "Creating CANTalonSrx " << pair.first
                                  << " on channel " << pair.second.id
                                  << " w/ inverted=" << pair.second.inverted);
    // clang-format on

    using ctre::phoenix::motorcontrol::FeedbackDevice;
    using ctre::phoenix::motorcontrol::can::WPI_TalonSRX;

    // Create the object
    can_talon_srxs_[pair.first] = std::make_unique<WPI_TalonSRX>(pair.second.id);
    can_talon_srxs_[pair.first]->SetInverted(pair.second.inverted);
    can_talon_srxs_[pair.first]->SetSensorPhase(pair.second.feedback_inverted);

    // Load position PID gains
    can_talon_srxs_[pair.first]->Config_kP(0, pair.second.pos_gains.k_p);
    can_talon_srxs_[pair.first]->Config_kI(0, pair.second.pos_gains.k_i);
    can_talon_srxs_[pair.first]->Config_kD(0, pair.second.pos_gains.k_d);
    can_talon_srxs_[pair.first]->Config_kF(0, pair.second.pos_gains.k_f);
    can_talon_srxs_[pair.first]->ConfigMaxIntegralAccumulator(0, pair.second.pos_gains.i_clamp);

    // Load velocity PID gains
    can_talon_srxs_[pair.first]->Config_kP(1, pair.second.vel_gains.k_p);
    can_talon_srxs_[pair.first]->Config_kI(1, pair.second.vel_gains.k_i);
    can_talon_srxs_[pair.first]->Config_kD(1, pair.second.vel_gains.k_d);
    can_talon_srxs_[pair.first]->Config_kF(1, pair.second.vel_gains.k_f);
    can_talon_srxs_[pair.first]->ConfigMaxIntegralAccumulator(1, pair.second.vel_gains.i_clamp);

    // Load effort PID gains
    can_talon_srxs_[pair.first]->Config_kP(1, pair.second.eff_gains.k_p);
    can_talon_srxs_[pair.first]->Config_kI(1, pair.second.eff_gains.k_i);
    can_talon_srxs_[pair.first]->Config_kD(1, pair.second.eff_gains.k_d);
    can_talon_srxs_[pair.first]->Config_kF(1, pair.second.eff_gains.k_f);
    can_talon_srxs_[pair.first]->ConfigMaxIntegralAccumulator(1, pair.second.eff_gains.i_clamp);

    // Select the sensor
    if (pair.second.feedback == "quad_encoder") {
      can_talon_srxs_[pair.first]->ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder);
    } else if (pair.second.feedback == "analog") {
      can_talon_srxs_[pair.first]->ConfigSelectedFeedbackSensor(FeedbackDevice::Analog);
    } else if (pair.second.feedback == "tachometer") {
      can_talon_srxs_[pair.first]->ConfigSelectedFeedbackSensor(FeedbackDevice::Tachometer);
    } else if (pair.second.feedback == "pule_width") {
      can_talon_srxs_[pair.first]->ConfigSelectedFeedbackSensor(FeedbackDevice::PulseWidthEncodedPosition);
    } else {
      // No built-in feedback
      // TODO: Support this case
    }
  }

#endif

  // =*=*=*=*=*=*=*= Misc =*=*=*=*=*=*=*=

  // Create Compressors
  for (const auto& pair : compressor_templates_) {
    // clang-format off
    ROS_DEBUG_STREAM_NAMED(name_, "Creating WPILib Compressor " << pair.first
                                  << " on PCM  " << pair.second);
    // clang-format on
    compressors_[pair.first] = std::make_unique<frc::Compressor>(pair.second);
  }

  // Create PDPs
  for (const auto& pair : pdp_templates_) {
    // clang-format off
    ROS_DEBUG_STREAM_NAMED(name_, "Creating PDP " << pair.first
                                  << " with ID " << pair.second);
    // clang-format on
    pdps_[pair.first] = std::make_unique<frc::PowerDistributionPanel>(pair.second);
  }

  // Start up the HAL thread to setup the HAL and report robot code ready to the FMS
  hal_thread_ = std::thread(&FRCRobotHWReal::runHAL, this);

  // TODO: Set robot_code_ready once all controllers are loaded
  robot_code_ready_ = true;
  return true;
}

void FRCRobotHWReal::doSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
                              const std::list<hardware_interface::ControllerInfo>& stop_list) {

  FRCRobotHW::doSwitch(start_list, stop_list);

  using Mode = MultiPIDController::Mode;

  // Disable PID for speed controllers claimed by stopping controllers
  for (const auto& controller : stop_list) {
    for (const auto& claimed : controller.claimed_resources) {
      for (const auto& resource : claimed.resources) {
        if (simple_speed_controller_pids_.find(resource) != simple_speed_controller_pids_.end()) {
          simple_speed_controller_pids_[resource]->setMode(Mode::disabled);
        }

#if USE_CTRE
        else if (can_talon_srxs_.find(resource) != can_talon_srxs_.end()) {
          // Do nothing
        }
#endif
      }
    }
  }

  // Set PID mode for speed controllers claimed by starting controllers
  for (const auto& controller : start_list) {
    for (const auto& claimed : controller.claimed_resources) {
      for (const auto& resource : claimed.resources) {
        if (simple_speed_controller_pids_.find(resource) != simple_speed_controller_pids_.end()) {
          if (claimed.hardware_interface == "hardware_interface::PositionJointInterface") {
            simple_speed_controller_pids_[resource]->setMode(Mode::position);
          } else if (claimed.hardware_interface == "hardware_interface::VelocityJointInterface") {
            simple_speed_controller_pids_[resource]->setMode(Mode::velocity);
          } else if (claimed.hardware_interface == "hardware_interface::EffortJointInterface") {
            simple_speed_controller_pids_[resource]->setMode(Mode::effort);
          } else if (claimed.hardware_interface == "hardware_interface::VoltageJointInterface") {
            simple_speed_controller_pids_[resource]->setMode(Mode::disabled);
          }
        }


#if USE_CTRE
        else if (can_talon_srxs_.find(resource) != can_talon_srxs_.end()) {
          if (claimed.hardware_interface == "hardware_interface::PositionJointInterface") {
            can_talon_srxs_[resource]->SelectProfileSlot(0, 0);
          } else if (claimed.hardware_interface == "hardware_interface::VelocityJointInterface") {
            can_talon_srxs_[resource]->SelectProfileSlot(1, 0);
          } else if (claimed.hardware_interface == "hardware_interface::EffortJointInterface") {
            can_talon_srxs_[resource]->SelectProfileSlot(2, 0);
          } else if (claimed.hardware_interface == "hardware_interface::VoltageJointInterface") {
            // Do nothing
          }
        }
#endif
      }
    }
  }
}

void FRCRobotHWReal::read(const ros::Time& time, const ros::Duration& period) {

  // =*=*=*=*=*=*=*= Sensors =*=*=*=*=*=*=*=

  // Read current AnalogInput states
  for (const auto& pair : analog_inputs_) {
    const auto& config             = analog_input_templates_[pair.first];
    double      last_state         = rate_states_[pair.first].state;
    rate_states_[pair.first].state = pair.second->GetVoltage() / 5.0 * config.scale + config.offset;

    // Sloppy rate calculation
    // TODO: Use hardware rate when available (AnalogGyro). Actually, we should probably treat AnalogGyros as IMUs,
    // and reserve AnalogInputs for pots, linear transducers, analog ultrasonics, etc.
    static double last_time       = ros::Time::now().toSec();
    double        cur_time        = ros::Time::now().toSec();
    rate_states_[pair.first].rate = (rate_states_[pair.first].state - last_state) / (cur_time - last_time);
    last_time                     = cur_time;
  }

  // Read current DigitalInput states
  for (const auto& pair : digital_inputs_) {
    const auto& config         = digital_input_templates_[pair.first];
    binary_states_[pair.first] = pair.second->Get() ^ config.inverted;
  }

  // Read current Encoder states
  for (const auto& pair : encoders_) {
    rate_states_[pair.first].state = pair.second->GetDistance();  // Distance, scaled by distancePerPulse
    rate_states_[pair.first].rate  = pair.second->GetRate();      // Distance / second, scaled by distancePerPulse
  }

  // Read current accelerometer states
  for (const auto& pair : generic_accelerometers_) {

    // Linear acceleration (X, Y, Z in m/s^2)
    imu_states_[pair.first].linear_acceleration[0] = pair.second->GetX() * 9.81;
    imu_states_[pair.first].linear_acceleration[1] = pair.second->GetY() * 9.81;
    imu_states_[pair.first].linear_acceleration[2] = pair.second->GetZ() * 9.81;

    // Angular velocity, orientation are not reported
    imu_states_[pair.first].angular_velocity_covariance[0] = -1.0;
    imu_states_[pair.first].orientation_covariance[0]      = -1.0;
  }

  // Read current navX IMU states
#if USE_KAUAI
  for (const auto& pair : navxs_) {
    const auto& navx = pair.second;

    // TODO: Orientation/heading zeroing if needed?
    // const double last_zero_val;
    // const double offset = last_zero_val - navx->GetFusedHeading() / 180.0 * M_PI;
    // const double cur_heading = navx->GetFusedHeading() / 180.0 * M_PI + offset;

    // Linear acceleration (X, Y, Z in m/s^2)
    // NOTE: Use RawAccel rather than WorldLinearAccel in order to be REP0145 compliant.
    //       RawAccel does not compensate for gravity or orientation.
    imu_states_[pair.first].linear_acceleration[0] = navx->GetRawAccelX() * 9.81;
    imu_states_[pair.first].linear_acceleration[1] = navx->GetRawAccelY() * 9.81;
    imu_states_[pair.first].linear_acceleration[2] = navx->GetRawAccelZ() * 9.81;

    // Angular velocity (X,Y,Z in degrees per second)
    imu_states_[pair.first].angular_velocity[0] = navx->GetRawGyroX() / 180.0 * M_PI;
    imu_states_[pair.first].angular_velocity[1] = navx->GetRawGyroY() / 180.0 * M_PI;
    imu_states_[pair.first].angular_velocity[2] = navx->GetRawGyroZ() / 180.0 * M_PI;

    // Orientation (Quaternion X, Y, Z, W)
    tf2::Quaternion tempQ;
    tempQ.setRPY(navx->GetRoll() / -180.0 * M_PI,
                 navx->GetPitch() / -180.0 * M_PI,
                 navx->GetFusedHeading() / 180.0 * M_PI /* + offset*/);
    imu_states_[pair.first].orientation[0] = tempQ.x();
    imu_states_[pair.first].orientation[1] = tempQ.y();
    imu_states_[pair.first].orientation[2] = tempQ.z();
    imu_states_[pair.first].orientation[3] = tempQ.w();
  }
#endif

  // Read current Pigeon IMU states
#if USE_CTRE
  for (const auto& pair : pigeons_) {
    const auto& pigeon = pair.second;

    // TODO: Orientation/heading zeroing if needed?

    // Linear acceleration (X, Y, Z in m/s^2)
    int16_t accl_xyz[3];
    pigeon->GetBiasedAccelerometer(accl_xyz);
    // Convert Q2.14 to a double, in Gs, and then to m/s^2
    imu_states_[pair.first].linear_acceleration[0] = (accl_xyz[0] >> 14) + (accl_xyz[0] & 0x3FFF) * pow(2, -14) * 9.81;
    imu_states_[pair.first].linear_acceleration[1] = (accl_xyz[1] >> 14) + (accl_xyz[1] & 0x3FFF) * pow(2, -14) * 9.81;
    imu_states_[pair.first].linear_acceleration[2] = (accl_xyz[2] >> 14) + (accl_xyz[2] & 0x3FFF) * pow(2, -14) * 9.81;

    // Angular velocity (X,Y,Z in degrees per second)
    pigeon->GetRawGyro(imu_states_[pair.first].angular_velocity);

    // Orientation (Quaternion X, Y, Z, W)
    double q_wxyz[4];
    pigeon->Get6dQuaternion(q_wxyz);
    imu_states_[pair.first].orientation[0] = q_wxyz[1];
    imu_states_[pair.first].orientation[1] = q_wxyz[2];
    imu_states_[pair.first].orientation[2] = q_wxyz[3];
    imu_states_[pair.first].orientation[3] = q_wxyz[0];
  }
#endif


  // =*=*=*=*=*=*=*= Actuators =*=*=*=*=*=*=*=

  // Read current AnalogOutput states
  for (const auto& pair : analog_outputs_) {
    const auto& config             = analog_output_templates_[pair.first];
    rate_states_[pair.first].state = pair.second->GetVoltage() / 5.0 * config.scale + config.offset;
    rate_states_[pair.first].rate  = 0.0;
  }

  // Read current DigitalOutput states
  for (const auto& pair : digital_outputs_) {
    const auto& config         = digital_output_templates_[pair.first];
    binary_states_[pair.first] = pair.second->Get() ^ config.inverted;
  }

  // Read current DoubleSolenoid states
  for (const auto& pair : double_solenoids_) {
    using Value = frc::DoubleSolenoid::Value;
    TernaryState state;
    switch (pair.second->Get()) {
      case Value::kOff:
        state = TernaryState::kOff;
        break;
      case Value::kForward:
        state = TernaryState::kForward;
        break;
      case Value::kReverse:
        state = TernaryState::kReverse;
        break;
    }
    ternary_states_[pair.first] = state;
  }

  // Read current Relay states
  for (const auto& pair : relays_) {
    using Value     = frc::Relay::Value;
    using Direction = hardware_template::Relay::Direction;

    Value value = pair.second->Get();
    if (value == Value::kOff)
      ternary_states_[pair.first] = TernaryState::kOff;
    else if (value == Value::kForward)
      ternary_states_[pair.first] = TernaryState::kForward;
    else if (value == Value::kReverse)
      ternary_states_[pair.first] = TernaryState::kReverse;

    // We cannot get direction of relay from WPILib, so instead fetch from template
    else if (value == Value::kOn && relay_templates_[pair.first].direction == Direction::kForward)
      ternary_states_[pair.first] = TernaryState::kForward;
    else if (value == Value::kOn && relay_templates_[pair.first].direction == Direction::kReverse)
      ternary_states_[pair.first] = TernaryState::kReverse;
    else {
      ROS_ERROR_STREAM_NAMED(name_, "Error: Unexpected state kOn on bidirectional relay! Setting state to kOff");
      ternary_states_[pair.first]   = TernaryState::kOff;
      ternary_commands_[pair.first] = TernaryState::kOff;
    }
  }

  // Read current Servo states
  for (const auto& pair : servos_) {
    joint_states_[pair.first].pos = pair.second->Get();  // TODO: Scale to degrees, etc
    joint_states_[pair.first].vel = 0.0;
    joint_states_[pair.first].eff = 0.0;
  }

  // Read current Solenoid states
  for (const auto& pair : solenoids_) {
    binary_states_[pair.first] = pair.second->Get();
  }

#if USE_CTRE

  // Read current CANTalonSRX states
  for (const auto& pair : can_talon_srxs_) {
    if (can_talon_srx_templates_[pair.first].feedback != "none") {
      joint_states_[pair.first].pos = pair.second->GetSelectedSensorPosition();
      joint_states_[pair.first].vel = pair.second->GetSelectedSensorVelocity() / 10.0;  // Units/100ms to units/sec
    }
    joint_states_[pair.first].eff = pair.second->GetOutputCurrent() * can_talon_srx_templates_[pair.first].k_eff;
  }
#endif


  // Notice we don't read from the simple_speed_controllers, since they have no feedback.
  // We rely on merging other sensors together to form the JointState

  // =*=*=*=*=*=*=*= Misc =*=*=*=*=*=*=*=

  // Read current Compressor states
  for (const auto& pair : compressors_) {
    compressor_states_[pair.first].closed_loop            = pair.second->GetClosedLoopControl();
    compressor_states_[pair.first].enabled                = pair.second->Enabled();
    compressor_states_[pair.first].pressure_switch        = pair.second->GetPressureSwitchValue();
    compressor_states_[pair.first].current                = pair.second->GetCompressorCurrent();
    compressor_states_[pair.first].fault_current_too_high = pair.second->GetCompressorCurrentTooHighFault();
    compressor_states_[pair.first].fault_shorted          = pair.second->GetCompressorShortedFault();
    compressor_states_[pair.first].fault_not_connected    = pair.second->GetCompressorNotConnectedFault();
  }

  // Read current PDP current draw states
  for (const auto& pair : pdps_) {
    pdp_states_[pair.first].voltage       = pair.second->GetVoltage();
    pdp_states_[pair.first].temperature   = pair.second->GetTemperature();
    pdp_states_[pair.first].total_current = pair.second->GetTotalCurrent();
    pdp_states_[pair.first].total_power   = pair.second->GetTotalPower();
    pdp_states_[pair.first].total_energy  = pair.second->GetTotalEnergy();
    for (unsigned i = 0; i < 16; i++)
      pdp_states_[pair.first].current[i] = pair.second->GetCurrent(i);
  }
};

void FRCRobotHWReal::write(const ros::Time& time, const ros::Duration& period) {

  // Write AnalogOutput commands
  for (const auto& pair : analog_outputs_) {
    const auto& config = analog_output_templates_[pair.first];
    pair.second->SetVoltage((analog_commands_[pair.first] - config.offset) / config.scale * 5.0);
  }

  // Write DigitalOutput commands
  for (const auto& pair : digital_outputs_) {
    const auto& config = digital_output_templates_[pair.first];
    pair.second->Set(binary_commands_[pair.first] ^ config.inverted);
  }

  // Write DoubleSolenoid commands
  for (const auto& pair : double_solenoids_) {
    using Value = frc::DoubleSolenoid::Value;
    Value value;
    switch (ternary_commands_[pair.first]) {
      case TernaryState::kOff:
        value = Value::kOff;
        break;
      case TernaryState::kForward:
        value = Value::kForward;
        break;
      case TernaryState::kReverse:
        value = Value::kReverse;
        break;
    }
    pair.second->Set(value);
  }

  // Write Relay commands
  for (const auto& pair : relays_) {
    using Value     = frc::Relay::Value;
    using Direction = hardware_template::Relay::Direction;

    Value command;
    switch (ternary_commands_[pair.first]) {
      case TernaryState::kOff:
        command = Value::kOff;
        break;
      case TernaryState::kForward:
        command = Value::kForward;
        break;
      case TernaryState::kReverse:
        command = Value::kReverse;
        break;
    }
    pair.second->Set(command);
  }

  // Write Servo commands
  for (const auto& pair : servos_) {
    pair.second->Set(joint_commands_[pair.first].data);  // TODO: Scale from angle to 0-1
  }

  // Write Solenoid commands
  for (const auto& pair : solenoids_) {
    pair.second->Set(binary_commands_[pair.first]);
  }

  // Write SpeedController commands
  for (const auto& pair : simple_speed_controllers_) {

    const auto&  config = simple_speed_controller_templates_[pair.first];
    const auto&  pid    = simple_speed_controller_pids_[pair.first];
    const double vbus   = pdp_states_[config.pdp].voltage;

    // Calculate the raw output value
    double output = 0.0;
    switch (joint_commands_[pair.first].type) {
      case JointCmd::Type::kNone:
        output = 0.0;
        break;
      case JointCmd::Type::kPos:
        pid->setSetpoint(joint_commands_[pair.first].data);
        output = pid->getOutput(joint_states_[pair.first].pos);
        break;
      case JointCmd::Type::kVel:
        pid->setSetpoint(joint_commands_[pair.first].data);
        output = pid->getOutput(joint_states_[pair.first].vel);
        break;
      case JointCmd::Type::kEff:
        pid->setSetpoint(joint_commands_[pair.first].data);
        output = pid->getOutput(joint_states_[pair.first].eff);
        break;
      case JointCmd::Type::kVolt:
        output = joint_commands_[pair.first].data / vbus;
        break;
    }

    pair.second->Set(output);
  }

  // Write Compressor commands
  for (const auto& pair : compressors_) {
    pair.second->SetClosedLoopControl(binary_commands_[pair.first]);
  }

#if USE_CTRE

  // Write CANTalonSrx commands
  for (const auto& pair : can_talon_srxs_) {
    using ctre::phoenix::motorcontrol::ControlMode;

    switch (joint_commands_[pair.first].type) {
      case JointCmd::Type::kNone:
        pair.second->Set(ControlMode::Disabled, 0.0);
        break;
      case JointCmd::Type::kPos:
        pair.second->Set(ControlMode::Position, joint_commands_[pair.first].data);
        break;
      case JointCmd::Type::kVel:
        pair.second->Set(ControlMode::Velocity, joint_commands_[pair.first].data * 10.0);  // Units/sec to units/100ms
        break;
      case JointCmd::Type::kEff:
        pair.second->Set(ControlMode::Current,
                         joint_commands_[pair.first].data / can_talon_srx_templates_[pair.first].k_eff);
        break;
      case JointCmd::Type::kVolt:
        pair.second->Set(ControlMode::PercentOutput, joint_commands_[pair.first].data / pair.second->GetBusVoltage());
        break;
    }
  }
#endif
};

}  // namespace frc_robot_hw

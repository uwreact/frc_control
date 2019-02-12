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

#include <frc_robot_hw/frc_robot_hw_real.h>

#include <boost/make_unique.hpp>
#include <hal/HAL.h>
#include <sensor_msgs/Joy.h>
#include <tf2/LinearMath/Quaternion.h>

namespace frc_robot_hw {

bool FRCRobotHWReal::initHAL() {
  if (!HAL_Initialize(500, 0)) {
    ROS_FATAL_NAMED(name_, "HAL could not be initialized!");
    return false;
  }

  HAL_Report(HALUsageReporting::kResourceType_Language, HALUsageReporting::kLanguage_CPlusPlus);

  // TODO: Waiting for 2019
  // HAL_Report(HALUsageReporting::kResourceType_Framework, HALUsageReporting::kFramework_ROS);

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
  HAL_ObserveUserProgramStarting();

  sensor_msgs::Joy joysticks[frc::DriverStation::kJoystickPorts];

  frc::DriverStation& ds = frc::DriverStation::GetInstance();

  while (ros::ok()) {
    // TODO: Limit update & publish rate of match data?

    // Read all joysticks
    for (unsigned stick = 0; stick < frc::DriverStation::kJoystickPorts; stick++) {

      joysticks[stick].header.stamp = ros::Time::now();

      joysticks[stick].axes.resize(ds.GetStickAxisCount(stick));
      for (unsigned axis = 0; axis < ds.GetStickAxisCount(stick); axis++)
        joysticks[stick].axes[axis] = ds.GetStickAxis(stick, axis);

      // Note: Buttons, unlike axes, are indexed from 1 rather than 0
      joysticks[stick].buttons.resize(ds.GetStickButtonCount(stick));
      for (unsigned button = 0; button < ds.GetStickButtonCount(stick); button++)
        joysticks[stick].buttons[button] = ds.GetStickButton(stick, button+1);

      // TODO: Ensure POV hat is covered. If not, append it to axes and/or buttons
    }

    // Get match data
    ds.GetGameSpecificMessage();
    ds.GetEventName();
    ds.GetMatchType(); // kNone, kPractice, kQualification, kElimination
    ds.GetMatchNumber();
    ds.GetReplayNumber(); // Presumably will increment on each replay of a match, since match number is no longer unique
                          // TODO: Validate this understanding
    ds.GetAlliance(); // kRed, kBlue, or kInvalid
    ds.GetLocation(); // 1, 2, or 3. 0 if invalid

    // Get current match data
    ds.GetMatchTime();  // APPROXIMATE remaining time in current period (auto or teleop), in seconds.
                        // Note: The FMS does not report official match time to robots, thus this may be imprecise.

    // Get robot state
    ds.IsEnabled();
    ds.IsAutonomous();
    ds.IsOperatorControl();
    ds.IsTest();
    ds.IsDSAttached();
    ds.IsFMSAttached();

    // TODO: Publish NetworkTables.

    // TODO: Publish data. Use RT pub?

    // Wait for driver station data so the loop doesn't hog the CPU
    ds.WaitForData();
  }
}

// Setup HAL interaction. Create WPILib objects for each specified sensor and actuator.
bool FRCRobotHWReal::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) {
  if(!FRCRobotHW::init(root_nh, robot_hw_nh))
    return false;

  if(!initHAL())
    return false;

  // =*=*=*=*=*=*=*= Sensors =*=*=*=*=*=*=*=

  // Create AnalogInputs
  // TODO: Allow configuration of bits, accumulator
  for (const auto& pair : analog_input_templates_) {
    ROS_DEBUG_STREAM_NAMED(name_, "Creating WPILib AnalogInput " << pair.first
                                  << " on channel " << pair.second.id
                                  << " w/ scale " << pair.second.scale     // Non-wpilib feature
                                  << " w/ offset " << pair.second.offset); // Non-wpilib feature
    analog_inputs_[pair.first] = boost::make_unique<frc::AnalogInput>(pair.second.id);
  }

  // Create DigitalInputs
  for (const auto& pair : digital_input_templates_) {
    ROS_DEBUG_STREAM_NAMED(name_, "Creating WPILib DigitalInput " << pair.first
                                  << " on channel " << pair.second.id
                                  << " w/ inverted=" << pair.second.inverted); // Non-wpilib feature
    digital_inputs_[pair.first] = boost::make_unique<frc::DigitalInput>(pair.second.id);
  }

  // Create Encoders
  for (const auto& pair : encoder_templates_) {
    ROS_DEBUG_STREAM_NAMED(name_, "Creating WPILib Encoder " << pair.first
                                  << " on channel A " << pair.second.ch_a
                                  << " and channel B " << pair.second.ch_b
                                  << " w/ distance per pulse " << pair.second.distance_per_pulse
                                  << " and encoding x" << pair.second.encoding
                                  << " and inverted " << pair.second.inverted);
    frc::CounterBase::EncodingType encoding;
    if (pair.second.encoding == 1)
      encoding = frc::CounterBase::EncodingType::k1X;
    else if (pair.second.encoding == 2)
      encoding = frc::CounterBase::EncodingType::k2X;
    else
      encoding = frc::CounterBase::EncodingType::k4X;

    encoders_[pair.first] =
      boost::make_unique<frc::Encoder>(pair.second.ch_a, pair.second.ch_b, pair.second.inverted, encoding);
  }

  // Create navXs
#if USE_KAUAI
  for (const auto& pair : navx_templates_) {
    ROS_DEBUG_STREAM_NAMED(name_, "Creating WPILib NavX-MXP " << pair.first
                                  << " on interface " << pair.second.interface
                                  << " at id " << pair.second.id
                                  << " with tf frame " << pair.second.frame_id); // Non-wpilib feature

    if (pair.second.interface == "i2c")
      navxs_[pair.first] = boost::make_unique<AHRS>((frc::I2C::Port) pair.second.id);
    else if (pair.second.interface == "serial")
      navxs_[pair.first] = boost::make_unique<AHRS>((frc::SerialPort::Port) pair.second.id);
    else
      navxs_[pair.first] = boost::make_unique<AHRS>((frc::SPI::Port) pair.second.id);
  }
#endif

  // Create PigeonIMUs
#if USE_CTRE
  for (const auto& pair : pigeon_templates_) {
    ROS_DEBUG_STREAM_NAMED(name_, "Creating WPILib PigeonIMU " << pair.first
                                  << " on CAN ID or TalonSRX name " << pair.second.interface
                                  << " with tf frame " << pair.second.frame_id); // Non-wpilib feature

    using PigeonIMU = ctre::phoenix::sensors::PigeonIMU;
    using PigeonIMUPtr = std::unique_ptr<PigeonIMU>;

    struct Visitor : public boost::static_visitor<PigeonIMUPtr> {
      PigeonIMUPtr operator()(const int& i) const { return boost::make_unique<PigeonIMU>(i); }
      PigeonIMUPtr operator()(const std::string& s) const { return boost::make_unique<PigeonIMU>( 0 ); } // TODO: Get associated TalonSRX
    };

    pigeons_[pair.first] = boost::apply_visitor(Visitor{}, pair.second.interface);
  }
#endif


  // =*=*=*=*=*=*=*= Actuators =*=*=*=*=*=*=*=

  // Create AnalogOutputs
  for (const auto& pair : analog_output_templates_) {
    ROS_DEBUG_STREAM_NAMED(name_, "Creating WPILib AnalogOutput " << pair.first
                                  << " on channel " << pair.second.id
                                  << " w/ scale " << pair.second.scale     // Non-wpilib feature
                                  << " w/ offset " << pair.second.offset); // Non-wpilib feature
    analog_outputs_[pair.first] = boost::make_unique<frc::AnalogOutput>(pair.second.id);
  }

  // Create DigitalOutputs
  for (const auto& pair : digital_output_templates_) {
    ROS_DEBUG_STREAM_NAMED(name_, "Creating WPILib DigitalOutput " << pair.first
                                  << " on channel " << pair.second.id
                                  << " w/ inverted=" << pair.second.inverted); // Non-wpilib feature
    digital_outputs_[pair.first] = boost::make_unique<frc::DigitalOutput>(pair.second.id);
  }

  // Create DoubleSolenoids
  for (const auto& pair : double_solenoid_templates_) {
    ROS_DEBUG_STREAM_NAMED(name_, "Creating WPILib DoubleSolenoid " << pair.first
                                  << " on channels fwd:" << pair.second.forward_id
                                  << " rev:" << pair.second.reverse_id
                                  << " on PCM  " << pair.second.pcm_id);
    double_solenoids_[pair.first] =
      boost::make_unique<frc::DoubleSolenoid>(pair.second.pcm_id, pair.second.forward_id, pair.second.reverse_id);
  }

  // Create Relays
  for (const auto& pair : relay_templates_) {
    using Direction = hardware_template::Relay::Direction;
    const std::string& direction_string = hardware_template::Relay::directionToString(pair.second.direction);
    ROS_DEBUG_STREAM_NAMED(name_, "Creating WPILib Relay " << pair.first
                                  << " on channel " << pair.second.id
                                  << " w/ direction " << direction_string);
    frc::Relay::Direction dir;
    switch (pair.second.direction) {
      case Direction::kBoth:    dir = frc::Relay::Direction::kBothDirections; break;
      case Direction::kForward: dir = frc::Relay::Direction::kForwardOnly;    break;
      case Direction::kReverse: dir = frc::Relay::Direction::kReverseOnly;    break;
    }
    relays_[pair.first] = boost::make_unique<frc::Relay>(pair.second.id, dir);
  }

  // Create Servos
  for (const auto& pair : servo_templates_) {
    ROS_DEBUG_STREAM_NAMED(name_, "Creating WPILib Servo " << pair.first
                                  << " on channel " << pair.second);
    servos_[pair.first] = boost::make_unique<frc::Servo>(pair.second);
  }

  // Create Solenoids
  for (const auto& pair : solenoid_templates_) {
    ROS_DEBUG_STREAM_NAMED(name_, "Creating WPILib Solenoid " << pair.first
                                  << " on channel " << pair.second.id
                                  << " on PCM  " << pair.second.pcm_id);
    solenoids_[pair.first] = boost::make_unique<frc::Solenoid>(pair.second.pcm_id, pair.second.id);
  }

  // Create simple SpeedControllers
  for (const auto& pair : simple_speed_controller_templates_) {
    const std::string& type_string = hardware_template::SimpleSpeedController::typeToString(pair.second.type);
    ROS_DEBUG_STREAM_NAMED(name_, "Creating " << type_string
                                  << " SpeedController " << pair.first
                                  << " on channel " << pair.second.id
                                  << " w/ inverted=" << pair.second.inverted);
    using Type = hardware_template::SimpleSpeedController::Type;

    const int id = pair.second.id;
    const int dio_id = pair.second.dio_id;
    std::unique_ptr<frc::SpeedController> controller;
    switch (pair.second.type) {
      case Type::DMC60:         controller = boost::make_unique<frc::DMC60>(id);        break;
      case Type::Jaguar:        controller = boost::make_unique<frc::Jaguar>(id);       break;
      case Type::PWMTalonSRX:   controller = boost::make_unique<frc::PWMTalonSRX>(id);  break;
      case Type::PWMVictorSPX:  controller = boost::make_unique<frc::PWMVictorSPX>(id); break;
      case Type::SD540:         controller = boost::make_unique<frc::SD540>(id);        break;
      case Type::Spark:         controller = boost::make_unique<frc::Spark>(id);        break;
      case Type::Talon:         controller = boost::make_unique<frc::Talon>(id);        break;
      case Type::Victor:        controller = boost::make_unique<frc::Victor>(id);       break;
      case Type::VictorSP:      controller = boost::make_unique<frc::VictorSP>(id);     break;
      case Type::Nidec:         controller = boost::make_unique<frc::NidecBrushless>(id, dio_id); break;
#if USE_CTRE
      case Type::CANVictorSPX:  controller = boost::make_unique<ctre::phoenix::motorcontrol::can::WPI_VictorSPX>(id); break;
#endif
    }
    simple_speed_controllers_[pair.first] = std::move(controller);
  }

  // Create smart SpeedControllers
  // TODO


  // =*=*=*=*=*=*=*= Misc =*=*=*=*=*=*=*=

  // Create Compressors
  for (const auto& pair : compressor_templates_) {
    ROS_DEBUG_STREAM_NAMED(name_, "Creating WPILib Compressor " << pair.first
                                  << " on PCM  " << pair.second);
    compressors_[pair.first] = boost::make_unique<frc::Compressor>(pair.second);
  }

  // Create PDPs
  for (const auto& pair : pdp_templates_) {
    ROS_DEBUG_STREAM_NAMED(name_, "Creating PDP " << pair.first
                                  << " with ID " << pair.second);
    pdps_[pair.first] = boost::make_unique<frc::PowerDistributionPanel>(pair.second);
  }

  // Start up the HAL thread to setup the HAL and report robot code ready to the FMS
  hal_thread_ = std::thread(&FRCRobotHWReal::runHAL, this);

  return true;
}

void FRCRobotHWReal::read(const ros::Time& time, const ros::Duration& period) {

  // =*=*=*=*=*=*=*= Sensors =*=*=*=*=*=*=*=

  // Read current AnalogInput states
  for (const auto& pair : analog_inputs_) {
    const auto& config = analog_input_templates_[pair.first];
    double last_state = rate_states_[pair.first].state;
    rate_states_[pair.first].state = pair.second->GetVoltage() / 5.0 * config.scale + config.offset;

    // Sloppy rate calculation
    // TODO: Use hardware rate when available (AnalogGyro). Actually, we should probably treat AnalogGyros as IMUs, and
    // reserve AnalogInputs for pots, linear transducers, analog ultrasonics, etc.
    static double last_time = ros::Time::now().toSec();
    double cur_time = ros::Time::now().toSec();
    rate_states_[pair.first].rate = (rate_states_[pair.first].state - last_state) / (cur_time - last_time);
    last_time = cur_time;
  }

  // Read current DigitalInput states
  for (const auto& pair : digital_inputs_) {
    const auto& config = digital_input_templates_[pair.first];
    binary_states_[pair.first] = pair.second->Get() ^ config.inverted;
  }

  // Read current Encoder states
  for (const auto& pair : encoders_) {
    rate_states_[pair.first].state = pair.second->GetDistance();  // Distance, scaled by distancePerPulse
    rate_states_[pair.first].rate = pair.second->GetRate();       // Distance / second, scaled by distancePerPulse
  }

  // Read current navX IMU states
#if USE_KAUAI
  for (const auto& pair : navxs_) {
    const auto& navx = pair.second;

    // TODO: Orientation/heading zeroing if needed?
    //const double last_zero_val;
    //const double offset = last_zero_val - navx->GetFusedHeading() / 180.0 * M_PI;
    //const double cur_heading = navx->GetFusedHeading() / 180.0 * M_PI + offset;

    // Linear acceleration (X, Y, Z in m/s^2)
    // TODO: Determine if we should use this or getRawAccel. Might factor into gravity.
    imu_states_[pair.first].linear_acceleration[0] = navx->GetWorldLinearAccelX() * 9.81;
    imu_states_[pair.first].linear_acceleration[1] = navx->GetWorldLinearAccelY() * 9.81;
    imu_states_[pair.first].linear_acceleration[2] = navx->GetWorldLinearAccelZ() * 9.81;

    // Angular velocity (X,Y,Z in degrees per second)
    imu_states_[pair.first].angular_velocity[0] = navx->GetRawGyroX() / 180.0 * M_PI;
    imu_states_[pair.first].angular_velocity[1] = navx->GetRawGyroY() / 180.0 * M_PI;
    imu_states_[pair.first].angular_velocity[2] = navx->GetRawGyroZ() / 180.0 * M_PI;

    // Orientation (Quaternion X, Y, Z, W)
    tf2::Quaternion tempQ;
    tempQ.setRPY(navx->GetRoll() / -180.0 * M_PI,
                  navx->GetPitch() / -180.0 * M_PI,
                  navx->GetFusedHeading() / 180.0 * M_PI/* + offset*/);
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
    const auto& config = analog_output_templates_[pair.first];
    rate_states_[pair.first].state = pair.second->GetVoltage() * config.scale + config.offset;
    rate_states_[pair.first].rate = 0;
  }

  // Read current DigitalOutput states
  for (const auto& pair : digital_outputs_) {
    const auto& config = digital_output_templates_[pair.first];
    binary_states_[pair.first] = pair.second->Get() ^ config.inverted;
  }

  // Read current DoubleSolenoid states
  for (const auto& pair : double_solenoids_) {
    using Value = frc::DoubleSolenoid::Value;
    TernaryState state;
    switch (pair.second->Get()) {
      case Value::kOff:     state = TernaryState::kOff;     break;
      case Value::kForward: state = TernaryState::kForward; break;
      case Value::kReverse: state = TernaryState::kReverse; break;
    }
     ternary_states_[pair.first] = state;
  }

  // Read current Relay states
  for (const auto& pair : relays_) {
    using Value = frc::Relay::Value;
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
      ternary_states_[pair.first] = TernaryState::kOff;
      ternary_commands_[pair.first] = TernaryState::kOff;
    }
  }

  // Read current Servo states
  for (const auto& pair : servos_) {
    joint_states_[pair.first].pos = pair.second->Get(); // TODO: Scale to degrees, etc
    joint_states_[pair.first].vel = 0;
    joint_states_[pair.first].eff = 0;
  }

  // Read current Solenoid states
  for (const auto& pair : solenoids_) {
    binary_states_[pair.first] = pair.second->Get();
  }

  // Read current smart SpeedController states
  // for (const auto& pair : smart_speed_controllers_) {
  //   // TODO
  // }

  // Notice we don't read from the simple_speed_controllers, since they have no feedback.
  // We rely on merging other sensors together to form the JointState

  // =*=*=*=*=*=*=*= Misc =*=*=*=*=*=*=*=

  // Read current Compressor states
  // TODO: Implement current, pressure switch feedback
  for (const auto& pair : compressors_) {
    binary_states_[pair.first] = pair.second->GetClosedLoopControl();
  }

  // Read current PDP current draw states
  for (const auto& pair : pdps_) {
    pdp_states_[pair.first].voltage = pair.second->GetVoltage();
    pdp_states_[pair.first].temperature = pair.second->GetTemperature();
    pdp_states_[pair.first].total_current = pair.second->GetTotalCurrent();
    pdp_states_[pair.first].total_power = pair.second->GetTotalPower();
    pdp_states_[pair.first].total_energy = pair.second->GetTotalEnergy();
    for (unsigned i = 0; i < 16; i++)
      pdp_states_[pair.first].current[i] = pair.second->GetCurrent(i);
  }
};

void FRCRobotHWReal::write(const ros::Time& time, const ros::Duration& period) {

  // Write AnalogOutput commands
  for (const auto& pair : analog_outputs_) {
    const auto& config = analog_output_templates_[pair.first];
    pair.second->SetVoltage((analog_commands_[pair.first] - config.offset) /config.scale);
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
    switch (ternary_states_[pair.first]) {
      case TernaryState::kOff:     value = Value::kOff;     break;
      case TernaryState::kForward: value = Value::kForward; break;
      case TernaryState::kReverse: value = Value::kReverse; break;
    }
    pair.second->Set(value);
  }

  // Write Relay commands
  for (const auto& pair : relays_) {
    using Value = frc::Relay::Value;
    using Direction = hardware_template::Relay::Direction;

    Value command;
    switch (ternary_commands_[pair.first]) {
      case TernaryState::kOff:      command = Value::kOff;      break;
      case TernaryState::kForward:  command = Value::kForward;  break;
      case TernaryState::kReverse:  command = Value::kReverse;  break;
    }
    pair.second->Set(command);
  }

  // Write Servo commands
  for (const auto& pair : servos_) {
    pair.second->Set(joint_commands_[pair.first].data); // TODO: Scale from angle to 0-1
  }

  // Write Solenoid commands
  for (const auto& pair : solenoids_) {
    pair.second->Set(binary_commands_[pair.first]);
  }

  // Write SpeedController commands
  for (const auto& pair : simple_speed_controllers_) {

    // Get the bus voltage
    const auto& config = simple_speed_controller_templates_[pair.first];
    const double vbus = pdp_states_[config.pdp].voltage;

    // TODO: Need PIDs to convert cmd from pos/vel/eff to percent Vbus, unless in voltage mode
    switch(joint_commands_[pair.first].type) {
      case JointCmd::Type::kNone:
        pair.second->Set(0);
        break;
      case JointCmd::Type::kPos:
        ROS_WARN_ONCE_NAMED(name_, "Position control is not yet implemented");
        break;
      case JointCmd::Type::kVel:
        ROS_WARN_ONCE_NAMED(name_, "Velocity control is not yet implemented");
        break;
      case JointCmd::Type::kEff:
        ROS_WARN_ONCE_NAMED(name_, "Effort control is not yet implemented");
        break;
      case JointCmd::Type::kVolt:
        pair.second->Set(joint_commands_[pair.first].data / vbus);
        break;
    }
  }

  // Write smart SpeedController commands
  // for (const auto& pair : smart_speed_controllers_) {
  //   // TODO
  // }

  // Write Compressor commands
  for (const auto& pair : compressors_) {
    pair.second->SetClosedLoopControl(binary_commands_[pair.first]);
  }
};

} // namespace frc_robot_hw

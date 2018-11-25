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

#include <frc_robot_hw/frc_robot_hw.h>

// #include <transmission_interface/simple_transmission.h>
// #include <transmission_interface/differential_transmission.h>

namespace frc_robot_hw {

void FRCRobotHW::loadURDF(const ros::NodeHandle& nh, const std::string& param_name) {

  // Search through the node_handle's namespace for the specified parameter
  std::string search_param_name;
  if (!nh.searchParam(param_name, search_param_name)) {
    search_param_name = param_name;
  }

  // Pull the URDF off of the parameter server
  std::string urdf_string;
  if (!nh.getParam(search_param_name, urdf_string)) {
    ROS_ERROR_STREAM_NAMED(name_, "Unable to load URDF model, parameter " << param_name << " not found");
    return;
  }

  // Parse the string into an XML document
  if (urdf_xml_.Parse(urdf_string.c_str())) {
    ROS_ERROR_STREAM_NAMED(name_, "Unable to parse URDF model, invalid XML");
    return;
  }

  // Create the URDF model
  // Note: We could pass the XML doc in here, but it is more efficient to just pass in the raw string
  if (!urdf_model_.initString(urdf_string)) {
    ROS_ERROR_NAMED(name_, "Unable to load URDF model");
  } else {
    ROS_INFO_NAMED(name_, "Loaded URDF from parameter server");
  }
}

void FRCRobotHW::loadJoints(const ros::NodeHandle& nh, const std::string& param_name) {
  using XmlValue = XmlRpc::XmlRpcValue;
  using namespace hardware_template;

  ROS_INFO_NAMED(name_, "Loading joints");

  // Read a list of joint information from the parameter server. Each entry in the list
  // specifies a name for the joint and a hardware ID corresponding
  // to that value. Joint types (eg. revolute, prismatic) and locations are specified (by name)
  // in a URDF file loaded along with the controller.

  // Search through the node_handle's namespace for the specified parameter
  std::string search_param_name;
  if (!nh.searchParam(param_name, search_param_name)) {
    search_param_name = param_name;
  }

  // Pull the joint parameters off of the parameter server
  XmlValue joint_param_list;
  if (!nh.getParam(search_param_name, joint_param_list)) {
    ROS_ERROR_STREAM_NAMED(name_, "Unable to load joints, parameter " << param_name << " not found");
    return;
  }

  // Parse each joint and store it in its respective map
  for (const auto& pair : joint_param_list) {
    const std::string joint_name = pair.first;
    XmlValue          cur_joint  = pair.second;
    ROS_INFO_STREAM(joint_name << cur_joint);

    // Ensure the current joint parameter specifies a type
    // TODO: Consider failing instead of skipping (throw std::runtime_error) in some cases
    if (!validateJointParamMember(cur_joint, "type", XmlValue::TypeString)) {
      continue;
    }

    // Get the type of the joint
    const std::string joint_type = cur_joint["type"];

    bool is_smart_speed_controller;
    try {
      SmartSpeedController::stringToType(joint_type);
      is_smart_speed_controller = true;
    } catch (const std::runtime_error& e) {
      is_smart_speed_controller = false;
    }

    bool is_simple_speed_controller;
    try {
      SimpleSpeedController::stringToType(joint_type);
      is_simple_speed_controller = true;
    } catch (const std::runtime_error& e) {
      is_simple_speed_controller = false;
    }

    // Depending on the type of joint, parse the data further and store the config in a template
    if (is_smart_speed_controller) {
      if (!validateJointParamMember(cur_joint, "id", XmlValue::TypeInt))
        continue;

      bool inverted = validateJointParamMember(cur_joint, "inverted", XmlValue::TypeBoolean, false, true)
                      && cur_joint["inverted"];


      // Can be "none", "internal", or another feedback device (eg. analog, encoder)
      std::string feedback;
      if (validateJointParamMember(cur_joint, "feedback", XmlValue::TypeString, false, true))
        feedback = (std::string) cur_joint["feedback"];
      else
        feedback = "none";

      smart_speed_controller_templates_[joint_name] = {
          .type     = SmartSpeedController::stringToType(joint_type),
          .id       = cur_joint["id"],
          .inverted = inverted,
      };
    } else if (is_simple_speed_controller) {
      if (!validateJointParamMember(cur_joint, "id", XmlValue::TypeInt))
        continue;

      SimpleSpeedController::Type type = SimpleSpeedController::stringToType(joint_type);

      // Nidec additionally requires a dio channel
      int dio_ch = -1;
      if (type == SimpleSpeedController::Type::Nidec) {
        if (!validateJointParamMember(cur_joint, "dio_channel", XmlValue::TypeInt))
          continue;
        dio_ch = cur_joint["dio_channel"];
      }

      bool inverted = validateJointParamMember(cur_joint, "inverted", XmlValue::TypeBoolean, false, true)
                      && cur_joint["inverted"];

      // Can be "none" or the name of a PDP
      std::string pdp;
      if (validateJointParamMember(cur_joint, "pdp", XmlValue::TypeString, false, true))
        pdp = (std::string) cur_joint["pdp"];
      else
        pdp = "none";

      int pdp_ch;
      if (validateJointParamMember(cur_joint, "pdp_ch", XmlValue::TypeInt, false, true))
        pdp_ch = cur_joint["pdp_ch"];
      else
        pdp_ch = -1;

      if (pdp_ch < -1 || pdp_ch > 15) {
        ROS_WARN_STREAM_NAMED(name_,
                              "Invalid PDP channel '"
                                  << pdp_ch << "', must be within range [0,15]. "
                                  << " Skipping PDP current feedback (Joint will have no effort state)");
        pdp_ch = -1;
      }

      double k_eff;
      if (validateJointParamMember(cur_joint, "k_eff", XmlValue::TypeDouble, false, true))
        k_eff = getXmlRpcDouble(cur_joint["k_eff"]);
      else
        k_eff = 1.0;

      if (pdp == "none" && pdp_ch != -1)
        ROS_WARN_STREAM_NAMED(name_,
                              "PDP channel " << pdp_ch << " specified, but no PDP specified. "
                                             << "Please verify your configuration.");

      simple_speed_controller_templates_[joint_name] = {
          .type     = type,
          .id       = cur_joint["id"],
          .dio_id   = dio_ch,
          .inverted = inverted,
          .pdp      = pdp,
          .pdp_ch   = pdp_ch,
          .k_eff    = k_eff,
      };
    } else if (joint_type == "pdp") {
      if (validateJointParamMember(cur_joint, "id", XmlValue::TypeInt, false, true)) {
        pdp_templates_[joint_name] = cur_joint["id"];
      } else {
        ROS_WARN_STREAM_NAMED(name_,
                              "No PowerDistributionPanel ID specified, using default ID 0 for PDP '" << joint_name
                                                                                                     << "'");
        pdp_templates_[joint_name] = 0;
      }
    } else if (joint_type == "servo") {
      if (!validateJointParamMember(cur_joint, "id", XmlValue::TypeInt))
        continue;
      servo_templates_[joint_name] = cur_joint["id"];
    } else if (joint_type == "relay") {
      if (!validateJointParamMember(cur_joint, "relay_id", XmlValue::TypeInt)
          || !validateJointParamMember(cur_joint, "direction", XmlValue::TypeString))
        continue;

      using Direction = Relay::Direction;
      Direction direction;
      try {
        direction = Relay::stringToDirection(cur_joint["direction"]);
      } catch (const std::runtime_error& e) {
        ROS_WARN_STREAM_NAMED(name_, e.what() << ". Using default 'both'");
        direction = Direction::kBoth;
        // TODO: Should probably throw a bigger error here, since running the relay with incorrect direction could fry
        // components
      }

      relay_templates_[joint_name] = {
          .id        = cur_joint["relay_id"],
          .direction = direction,
      };
    } else if (joint_type == "solenoid") {
      if (!validateJointParamMember(cur_joint, "id", XmlValue::TypeInt))
        continue;

      int pcm_id;
      if (validateJointParamMember(cur_joint, "pcm_id", XmlValue::TypeInt, false, true))
        pcm_id = cur_joint["pcm_id"];
      else
        pcm_id = 0;

      solenoid_templates_[joint_name] = {
          .id     = cur_joint["id"],
          .pcm_id = pcm_id,
      };
    } else if (joint_type == "double_solenoid") {
      if (!validateJointParamMember(cur_joint, "forward_id", XmlValue::TypeInt)
          || !validateJointParamMember(cur_joint, "reverse_id", XmlValue::TypeInt))
        continue;

      int pcm_id;
      if (validateJointParamMember(cur_joint, "pcm_id", XmlValue::TypeInt, false, true))
        pcm_id = cur_joint["pcm_id"];
      else
        pcm_id = 0;

      double_solenoid_templates_[joint_name] = {
          .forward_id = cur_joint["forward_id"],
          .reverse_id = cur_joint["reverse_id"],
          .pcm_id     = pcm_id,
      };
    } else if (joint_type == "compressor") {
      int pcm_id;
      if (validateJointParamMember(cur_joint, "pcm_id", XmlValue::TypeInt, false, true))
        pcm_id = cur_joint["pcm_id"];
      else
        pcm_id = 0;

      compressor_templates_[joint_name] = pcm_id;
    } else if (joint_type == "digital_input") {
      if (!validateJointParamMember(cur_joint, "dio_channel", XmlValue::TypeInt))
        continue;

      bool inverted = validateJointParamMember(cur_joint, "inverted", XmlValue::TypeBoolean, false, true)
                      && cur_joint["inverted"];

      digital_input_templates_[joint_name] = {
          .joint    = "",  // TODO: Lookup in URDF
          .id       = cur_joint["dio_channel"],
          .inverted = inverted,
      };
    } else if (joint_type == "digital_output") {
      if (!validateJointParamMember(cur_joint, "dio_channel", XmlValue::TypeInt))
        continue;

      bool inverted = validateJointParamMember(cur_joint, "inverted", XmlValue::TypeBoolean, false, true)
                      && cur_joint["inverted"];

      digital_output_templates_[joint_name] = {
          .joint    = "",
          .id       = cur_joint["digital_output"],
          .inverted = inverted,
      };
    } else if (joint_type == "analog_input") {
      if (!validateJointParamMember(cur_joint, "ain_channel", XmlValue::TypeInt)
          || !validateJointParamMember(cur_joint, "scale", XmlValue::TypeDouble)
          || !validateJointParamMember(cur_joint, "offset", XmlValue::TypeDouble))
        continue;

      analog_input_templates_[joint_name] = {
          .joint  = "",  // TODO: Lookup in URDF
          .id     = cur_joint["ain_channel"],
          .scale  = getXmlRpcDouble(cur_joint["scale"]),
          .offset = getXmlRpcDouble(cur_joint["offset"]),
      };
    } else if (joint_type == "analog_output") {
      if (!validateJointParamMember(cur_joint, "aout_channel", XmlValue::TypeInt)
          || !validateJointParamMember(cur_joint, "scale", XmlValue::TypeDouble)
          || !validateJointParamMember(cur_joint, "offset", XmlValue::TypeDouble))
        continue;

      std::string joint;
      if (validateJointParamMember(cur_joint, "joint", XmlValue::TypeString, false, true))
        joint = (std::string) cur_joint["joint"];
      else
        joint = "none";

      analog_output_templates_[joint_name] = {
          .joint  = joint,
          .id     = cur_joint["analog_channel"],
          .scale  = getXmlRpcDouble(cur_joint["scale"]),
          .offset = getXmlRpcDouble(cur_joint["offset"]),
      };
    } else if (joint_type == "encoder") {
      if (!validateJointParamMember(cur_joint, "ch_a", XmlValue::TypeInt)
          || !validateJointParamMember(cur_joint, "ch_b", XmlValue::TypeInt)
          || !validateJointParamMember(cur_joint, "dist_per_pulse", XmlValue::TypeDouble))
        continue;

      const bool inverted = validateJointParamMember(cur_joint, "inverted", XmlValue::TypeBoolean, false, true)
                            && cur_joint["inverted"];
      int encoding;
      if (validateJointParamMember(cur_joint, "encoding", XmlValue::TypeInt, false, true))
        encoding = cur_joint["encoding"];
      else
        encoding = 4;

      if (encoding != 1 && encoding != 2 && encoding != 4) {
        ROS_WARN_STREAM_NAMED(name_,
                              "Invalid encoder encoding '" << encoding << "', must be 1, 2, or 4. Using default 4.");
        encoding = 4;
      }

      std::string joint;
      if (validateJointParamMember(cur_joint, "joint", XmlValue::TypeString, false, true))
        joint = (std::string) cur_joint["joint"];
      else
        joint = "none";

      encoder_templates_[joint_name] = {
          .joint              = joint,
          .ch_a               = cur_joint["ch_a"],
          .ch_b               = cur_joint["ch_b"],
          .distance_per_pulse = getXmlRpcDouble(cur_joint["dist_per_pulse"]),
          .inverted           = inverted,
          .encoding           = encoding,
      };
    }
#if USE_KAUAI
    else if (joint_type == "navx") {
      if (!validateJointParamMember(cur_joint, "frame_id", XmlValue::TypeString)
          || !validateJointParamMember(cur_joint, "interface", XmlValue::TypeString)
          || !validateJointParamMember(cur_joint, "id", XmlValue::TypeInt))
        continue;

      std::string interface = cur_joint["interface"];
      if (interface != "spi" && interface != "i2c" && interface != "serial") {
        ROS_WARN_STREAM_NAMED(name_,
                              "Invalid NavX-MXP interface '"
                                  << interface << "', must be 'spi', 'i2c', or 'serial. Using default 'serial'.");
        interface = "serial";
      }

      navx_templates_[joint_name] = {
          .interface = interface,
          .id        = cur_joint["id"],        // TODO: Validate
          .frame_id  = cur_joint["frame_id"],  // TODO: If not specified, use joint name
      };
    }
#endif
#if USE_CTRE
    else if (joint_type == "pigeon_imu") {
      if (!validateJointParamMember(cur_joint, "frame_id", XmlValue::TypeString))
        continue;

      bool has_can_id     = validateJointParamMember(cur_joint, "id", XmlValue::TypeInt, false, true);
      bool has_talon_name = validateJointParamMember(cur_joint, "talon", XmlValue::TypeString, false, true);

      boost::variant<int, std::string> interface;
      if (has_can_id && !has_talon_name) {
        interface = (int) cur_joint["id"];
      } else if (!has_can_id && has_talon_name) {
        interface = (std::string) cur_joint["talon"];
      } else if (has_can_id && has_talon_name) {
        ROS_WARN_STREAM_NAMED(name_,
                              "Skipping pigeon_imu '" << joint_name << "', two interfaces specified! "
                                                      << "Please specify either 'id' or 'talon' but not both");
        continue;
      } else {
        ROS_WARN_STREAM_NAMED(name_,
                              "Skipping pigeon_imu '"
                                  << joint_name
                                  << "', Pigeon must be connected directly to the CAN bus or to the bus through a "
                                  << "CANTalonSRX. Please specify either 'id' (int) or 'talon' (string)");
        continue;
      }
      pigeon_templates_[joint_name] = {
          .interface = interface,
          .frame_id  = cur_joint["frame_id"],
      };
    }
#endif
    else {
      ROS_WARN_STREAM_NAMED(name_,
                            "Skipping malformed joint, invalid type '" << joint_type << "': '" << cur_joint << "'");
      continue;
    }
  }

  ROS_INFO_NAMED(name_, "Done loading joints");
}

bool FRCRobotHW::validateJointParamMember(XmlRpc::XmlRpcValue&             value,
                                          const std::string&               member,
                                          const XmlRpc::XmlRpcValue::Type& type,
                                          const bool                       warn_exist,
                                          const bool                       warn_type) {

  // If the value is an int, allow implicit casting to double
  using Type = XmlRpc::XmlRpcValue::Type;
  if (type == Type::TypeDouble) {
    if (validateJointParamMember(value, member, Type::TypeInt, false, false))
      return true;
  }

  // Check that the member exists
  if (!value.hasMember(member)) {
    // clang-format off
    ROS_WARN_STREAM_COND_NAMED(warn_exist, name_, "Skipping malformed joint, missing mandatory member '"
                                                  << member << "': '" << value << "'");
    // clang-format on
    return false;
  }

  // Check that the member is of the correct type
  if (value[member].getType() != type) {
    // TODO: Map of Type to string
    // clang-format off
    ROS_WARN_STREAM_COND_NAMED(warn_type, name_, "Skipping malformed joint, '"
                                                  << member << "' must be a '" << type << "': '" << value << "'");
    //  clang-format on
    return false;
  }

  return true;
}

double FRCRobotHW::getXmlRpcDouble(XmlRpc::XmlRpcValue& value) {
  if (value.getType() == XmlRpc::XmlRpcValue::Type::TypeDouble)
    return value;
  if (value.getType() == XmlRpc::XmlRpcValue::Type::TypeInt)
    return (int) value;

   throw std::runtime_error("XmlRpcValue is not of type Double or Int!");
}

void FRCRobotHW::registerTransmissions() {
  // using namespace transmission_interface;

  //   if(pair.second.type_ == "SimpleTransmission")
  //     transmissions_[pair.first] = std::make_unique<SimpleTransmission>(0, 0); // TODO
  //   else if (pair.second.type_ == "DifferentialTransmission")
  //     transmissions_[pair.first] = std::make_unique<DifferentialTransmission>(
  //                                   std::vector<double>(pair.second.actuators_.size(), 0.0),
  //                                   std::vector<double>(pair.second.joints_.size(), 0.0)); // TODO

  //   actuator_state_data_[pair.first].position.push_back(&actuator_states_[pair.first].pos);

  //   act_to_jnt_state_interface_.registerHandle(ActuatorToJointStateHandle(
  //                                               pair.first,
  //                                               transmissions_[pair.first].get(),
  //                                               actuator_state_data_[pair.first],
  //                                               joint_state_data_[pair.first]));
  //   jnt_to_act_state_interface_.registerHandle(JointToActuatorStateHandle(
  //                                               pair.first,
  //                                               transmissions_[pair.first].get(),
  //                                               actuator_cmd_data_[pair.first],
  //                                               joint_cmd_data_[pair.first]));
  // }
}

bool FRCRobotHW::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) {
  loadURDF(root_nh, "robot_description");
  loadJoints(robot_hw_nh, "joints");

  // NOTE: Debug statements here are to be kept minimal. Describe only the name of the interface being registered,
  // and any KEY additional information, eg. cross-references interfaces. We do not need to print out additional joint
  // configuration information such as ID, inverted, etc. This data is irrelevant to the state interfaces.

  // TODO: Register handles for smart_motor_controllers

  // Register a command handle for each simple motor controller
  for (const auto& pair : simple_speed_controller_templates_) {
    // clang-format off
    ROS_DEBUG_STREAM_NAMED(name_, "Registering interface for SpeedController " << pair.first
                                  << " on channel " << pair.second.pdp_ch
                                  << " of PDP " << pair.second.pdp);
    // clang-format on
    hardware_interface::JointStateHandle state_handle(pair.first,
                                                      &joint_states_[pair.first].pos,
                                                      &joint_states_[pair.first].vel,
                                                      &joint_states_[pair.first].eff);
    joint_state_interface_.registerHandle(state_handle);

    // TODO: Only register pos, vel, effort cmds if the controller a) has a feedback device and b) has PID tunings
    joint_position_command_interface_.registerHandle(
        hardware_interface::JointHandle(state_handle, &(joint_commands_[pair.first].data)));
    joint_velocity_command_interface_.registerHandle(
        hardware_interface::JointHandle(state_handle, &(joint_commands_[pair.first].data)));
    joint_effort_command_interface_.registerHandle(
        hardware_interface::JointHandle(state_handle, &(joint_commands_[pair.first].data)));
    joint_voltage_command_interface_.registerHandle(
        hardware_interface::JointHandle(state_handle, &(joint_commands_[pair.first].data)));
  }

  // Register a state handle for each pdp
  for (const auto& pair : pdp_templates_) {
    ROS_DEBUG_STREAM_NAMED(name_, "Registering interface for PDP " << pair.first);
    pdp_state_interface_.registerHandle(hardware_interface::PDPStateHandle(pair.first,
                                                                           &pdp_states_[pair.first].voltage,
                                                                           &pdp_states_[pair.first].temperature,
                                                                           &pdp_states_[pair.first].total_current,
                                                                           &pdp_states_[pair.first].total_power,
                                                                           &pdp_states_[pair.first].total_energy,
                                                                           pdp_states_[pair.first].current));
  }

  // Register a command handle for each servo
  for (const auto& pair : servo_templates_) {
    ROS_DEBUG_STREAM_NAMED(name_, "Registering interface for servo " << pair.first);
    joint_commands_[pair.first].type = JointCmd::Type::kPos;  // TODO: Should this be here?
    hardware_interface::JointStateHandle state_handle(pair.first,
                                                      &joint_states_[pair.first].pos,
                                                      &joint_states_[pair.first].vel,
                                                      &joint_states_[pair.first].eff);
    joint_state_interface_.registerHandle(state_handle);
    joint_position_command_interface_.registerHandle(
        hardware_interface::JointHandle(state_handle, &(joint_commands_[pair.first].data)));
  }

  // Register a command handle for each relay
  // NOTE: The WPILib will handle invalid state commands itself, so we don't need to decide whether each relay is a
  // ternary or binary actuator, we can simply treat all as ternary
  // TODO: Design choice: Allow 4th state? (12v-12v). WPILib allows for this but it's a bit of an abuse of the hardware
  // since it's typically only used for controlling two actuators with a single relay. For now, we will continue to
  // treat relays as ternary devices.
  for (const auto& pair : relay_templates_) {
    ROS_DEBUG_STREAM_NAMED(name_, "Registering interface for relay " << pair.first);
    hardware_interface::TernaryStateHandle state_handle(pair.first, &ternary_states_[pair.first]);
    ternary_state_interface_.registerHandle(state_handle);
    ternary_command_interface_.registerHandle(
        hardware_interface::TernaryCommandHandle(state_handle, &ternary_commands_[pair.first]));
  }

  // Register a command handle for each single-acting solenoid
  for (const auto& pair : solenoid_templates_) {
    ROS_DEBUG_STREAM_NAMED(name_, "Registering interface for solenoid " << pair.first);
    hardware_interface::BinaryStateHandle state_handle(pair.first, &binary_states_[pair.first]);
    binary_state_interface_.registerHandle(state_handle);
    binary_command_interface_.registerHandle(
        hardware_interface::BinaryCommandHandle(state_handle, &binary_commands_[pair.first]));
    // Register a corresponding JointStateHandle for the solenoid
    joint_state_interface_.registerHandle(hardware_interface::JointStateHandle(pair.first,
                                                                               &joint_states_[pair.first].pos,
                                                                               &joint_states_[pair.first].vel,
                                                                               &joint_states_[pair.first].eff));
  }

  // Register a command handle for each double-acting solenoid
  for (const auto& pair : double_solenoid_templates_) {
    ROS_DEBUG_STREAM_NAMED(name_, "Registering interface for double solenoid " << pair.first);
    hardware_interface::TernaryStateHandle state_handle(pair.first, &ternary_states_[pair.first]);
    ternary_state_interface_.registerHandle(state_handle);
    ternary_command_interface_.registerHandle(
        hardware_interface::TernaryCommandHandle(state_handle, &ternary_commands_[pair.first]));
    // Register a corresponding JointStateHandle for the solenoid
    joint_state_interface_.registerHandle(hardware_interface::JointStateHandle(pair.first,
                                                                               &joint_states_[pair.first].pos,
                                                                               &joint_states_[pair.first].vel,
                                                                               &joint_states_[pair.first].eff));
  }

  // Register a command handle for each compressor
  for (const auto& pair : compressor_templates_) {
    ROS_DEBUG_STREAM_NAMED(name_, "Registering interface for compressor " << pair.first);
    hardware_interface::BinaryStateHandle state_handle(pair.first, &binary_states_[pair.first]);
    binary_state_interface_.registerHandle(state_handle);
    binary_command_interface_.registerHandle(
        hardware_interface::BinaryCommandHandle(state_handle, &binary_commands_[pair.first]));
  }

  // Register a state handle for each digital input
  for (const auto& pair : digital_input_templates_) {
    ROS_DEBUG_STREAM_NAMED(name_, "Registering interface for digital input " << pair.first);
    binary_state_interface_.registerHandle(
        hardware_interface::BinaryStateHandle(pair.first, &binary_states_[pair.first]));

    // If the digital input's parent joint hasn't been registered, then this must be a passive (non-driven) joint.
    // Therefore, register a corresponding JointStateHandle
    const auto& handles = joint_state_interface_.getNames();
    if (std::find(handles.begin(), handles.end(), pair.second.joint) == handles.end()) {
      joint_state_interface_.registerHandle(
          hardware_interface::JointStateHandle(pair.second.joint,
                                               &joint_states_[pair.second.joint].pos,
                                               &joint_states_[pair.second.joint].vel,
                                               &joint_states_[pair.second.joint].eff));
    }
  }

  // Register a command handle for each digital output
  for (const auto& pair : digital_output_templates_) {
    ROS_DEBUG_STREAM_NAMED(name_, "Registering interface for digital output " << pair.first);
    hardware_interface::BinaryStateHandle state_handle(pair.first, &binary_states_[pair.first]);
    binary_state_interface_.registerHandle(state_handle);
    binary_command_interface_.registerHandle(
        hardware_interface::BinaryCommandHandle(state_handle, &binary_commands_[pair.first]));
    // Register a corresponding JointStateHandle for the digital output
    joint_state_interface_.registerHandle(hardware_interface::JointStateHandle(pair.first,
                                                                               &joint_states_[pair.first].pos,
                                                                               &joint_states_[pair.first].vel,
                                                                               &joint_states_[pair.first].eff));
  }

  // Register a state handle for each analog input
  for (const auto& pair : analog_input_templates_) {
    ROS_DEBUG_STREAM_NAMED(name_, "Registering interface for analog input " << pair.first);
    analog_state_interface_.registerHandle(
        hardware_interface::AnalogStateHandle(pair.first, &rate_states_[pair.first].state));

    // If the analog input's parent joint hasn't been registered, then this must be a passive (non-driven) joint.
    // Therefore, register a corresponding JointStateHandle
    const auto& handles = joint_state_interface_.getNames();
    if (std::find(handles.begin(), handles.end(), pair.second.joint) == handles.end()) {
      joint_state_interface_.registerHandle(
          hardware_interface::JointStateHandle(pair.second.joint,
                                               &joint_states_[pair.second.joint].pos,
                                               &joint_states_[pair.second.joint].vel,
                                               &joint_states_[pair.second.joint].eff));
    }
  }

  // Register a command handle for each analog output
  for (const auto& pair : analog_output_templates_) {
    ROS_DEBUG_STREAM_NAMED(name_, "Registering interface for analog output " << pair.first);
    hardware_interface::AnalogStateHandle state_handle(pair.first, &rate_states_[pair.first].state);
    analog_state_interface_.registerHandle(state_handle);
    analog_command_interface_.registerHandle(
        hardware_interface::AnalogCommandHandle(state_handle, &analog_commands_[pair.first]));
    // Register a corresponding JointStateHandle for the analog output
    joint_state_interface_.registerHandle(hardware_interface::JointStateHandle(pair.first,
                                                                               &joint_states_[pair.first].pos,
                                                                               &joint_states_[pair.first].vel,
                                                                               &joint_states_[pair.first].eff));
  }

  // Register a state handle for each encoder
  for (const auto& pair : encoder_templates_) {
    ROS_DEBUG_STREAM_NAMED(name_, "Registering interface for encoder " << pair.first);
    analog_state_interface_.registerHandle(
        hardware_interface::AnalogStateHandle(pair.first, &rate_states_[pair.first].state));

    // If the encoder's parent joint hasn't been registered, then this must be a passive (non-driven) joint.
    // Therefore, register a corresponding JointStateHandle
    const auto& handles = joint_state_interface_.getNames();
    if (std::find(handles.begin(), handles.end(), pair.second.joint) == handles.end()) {
      joint_state_interface_.registerHandle(
          hardware_interface::JointStateHandle(pair.second.joint,
                                               &joint_states_[pair.second.joint].pos,
                                               &joint_states_[pair.second.joint].vel,
                                               &joint_states_[pair.second.joint].eff));
    }
  }

  // Register a state handle for each navX IMU
#if USE_KAUAI
  for (const auto& pair : navx_templates_) {
    // clang-format off
    ROS_DEBUG_STREAM_NAMED(name_, "Registering interface for navX-MXP IMU " << pair.first
                                  << " with tf frame " << pair.second.frame_id);
    // clang-format on

    // TODO: Init default values?

    imu_sensor_interface_.registerHandle(
        hardware_interface::ImuSensorHandle(pair.first,
                                            pair.second.frame_id,
                                            imu_states_[pair.first].orientation,
                                            imu_states_[pair.first].orientation_covariance,
                                            imu_states_[pair.first].angular_velocity,
                                            imu_states_[pair.first].angular_velocity_covariance,
                                            imu_states_[pair.first].linear_acceleration,
                                            imu_states_[pair.first].linear_acceleration_covariance));
  }
#endif

  // Register a state handle for each Pigeon IMU
#if USE_CTRE
  for (const auto& pair : pigeon_templates_) {
    // clang-format off
    ROS_DEBUG_STREAM_NAMED(name_, "Registering interface for Pigeon IMU " << pair.first
                                  << " with tf frame " << pair.second.frame_id);
    // clang-format on

    // TODO: Init default values?

    imu_sensor_interface_.registerHandle(
        hardware_interface::ImuSensorHandle(pair.first,
                                            pair.second.frame_id,
                                            imu_states_[pair.first].orientation,
                                            imu_states_[pair.first].orientation_covariance,
                                            imu_states_[pair.first].angular_velocity,
                                            imu_states_[pair.first].angular_velocity_covariance,
                                            imu_states_[pair.first].linear_acceleration,
                                            imu_states_[pair.first].linear_acceleration_covariance));
  }
#endif


  // TODO: Register all interfaces

  registerInterface(&joint_state_interface_);
  registerInterface(&joint_position_command_interface_);
  registerInterface(&joint_velocity_command_interface_);
  registerInterface(&joint_effort_command_interface_);
  registerInterface(&joint_voltage_command_interface_);

  return true;
}

void FRCRobotHW::updateRobotState() {

  // ** Actuators to JointStates **
  // [X] Analog Outputs
  // [X] Digial Outputs
  // [X] Double Solenoids
  // [ ] Relays?? - Not sure how to handle these
  // [X] Single Solenoids

  // ** Sensors to JointStates **
  // [X] Analog Input
  // [X] Digital Input
  // [X] Encoder
  // [X] PDP - Could be improved

  // =*=*=*= Convert non-motor actuator states into JointStates  =*=*=*=

  // Note that for solenoids and DIO, we set the velocity to 0, since there exists no concept of speed for these joints.
  // This is physically incorrect, but is the current model this entire project is designed around.

  // Convert AnalogOutput states
  for (const auto& pair : analog_output_templates_) {
    joint_states_[pair.first].pos = rate_states_[pair.first].state;
    joint_states_[pair.first].vel = rate_states_[pair.first].rate;
    joint_states_[pair.first].eff = 0;  // TODO: ???
  }

  // Convert DigitalOutput states
  for (const auto& pair : digital_output_templates_) {
    if (binary_states_[pair.first])
      joint_states_[pair.first].pos = urdf_model_.getJoint(pair.first)->limits->upper;
    else
      joint_states_[pair.first].pos = urdf_model_.getJoint(pair.first)->limits->lower;
    joint_states_[pair.first].vel = 0;  // Set vel to 0, since binary joints can have no speed
    joint_states_[pair.first].eff = 0;  // TODO: ???
  }

  // Convert DoubleSolenoid states
  for (const auto& pair : double_solenoid_templates_) {
    if (ternary_states_[pair.first] == TernaryState::kForward)
      joint_states_[pair.first].pos = urdf_model_.getJoint(pair.first)->limits->upper;
    else if (ternary_states_[pair.first] == TernaryState::kReverse)
      joint_states_[pair.first].pos = urdf_model_.getJoint(pair.first)->limits->lower;
    else {
    }
    joint_states_[pair.first].vel = 0;  // Set vel to 0, since binary joints can have no speed
    joint_states_[pair.first].eff = 0;  // TODO: Pressure?
  }

  // Convert Solenoid states
  for (const auto& pair : solenoid_templates_) {
    if (binary_states_[pair.first])
      joint_states_[pair.first].pos = urdf_model_.getJoint(pair.first)->limits->upper;
    else
      joint_states_[pair.first].pos = urdf_model_.getJoint(pair.first)->limits->lower;
    joint_states_[pair.first].vel = 0;  // Set vel to 0, since binary joints can have no speed
    joint_states_[pair.first].eff = 0;  // TODO: Pressure?
  }

  // =*=*=*= Convert pure sensors to JointStates =*=*=*=
  // Note: These sensors don't set unknown values to 0, as multiple sensors might be fused to complete the JointState.
  // For example, it is quite common to have an Encoder provide pos and vel, and the PDP provide effort

  // Convert AnalogInput states
  for (const auto& pair : analog_input_templates_) {
    if (pair.second.joint == "none")
      continue;
    joint_states_[pair.first].pos = rate_states_[pair.first].state;
    joint_states_[pair.first].vel = rate_states_[pair.first].rate;
    // Note: Don't set effort, since another sensor might
  }

  // Convert DigitalInput states
  for (const auto& pair : digital_input_templates_) {
    if (pair.second.joint == "none")
      continue;
    if (binary_states_[pair.first])
      joint_states_[pair.second.joint].pos = urdf_model_.getJoint(pair.second.joint)->limits->upper;
    else
      joint_states_[pair.second.joint].pos = urdf_model_.getJoint(pair.second.joint)->limits->lower;
    joint_states_[pair.second.joint].vel = 0;  // Set vel to 0, since binary joints can have no speed
    // Note: Don't set effort, since another sensor might
  }

  // Convert Encoder states
  for (const auto& pair : encoder_templates_) {
    if (pair.second.joint == "none")
      continue;
    joint_states_[pair.first].pos = rate_states_[pair.first].state;
    joint_states_[pair.first].vel = rate_states_[pair.first].rate;
    // Note: Don't set effort, since another sensor might
  }

  // Get the effort/current from the PDP
  // TODO: Move into PDP loop in case we have non-SpeedControllers drawing current?
  for (const auto& pair : simple_speed_controller_templates_) {
    const auto& config = pair.second;

    if (config.pdp != "none" && config.pdp_ch != -1)
      joint_states_[pair.first].eff = pdp_states_[config.pdp].current[config.pdp_ch] * config.k_eff;
  }
}

void FRCRobotHW::doSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
                          const std::list<hardware_interface::ControllerInfo>& stop_list) {

  // Reset command type for joints claimed by stopping controllers
  for (const auto& controller : stop_list) {
    for (const auto& claimed : controller.claimed_resources) {
      for (const auto& resource : claimed.resources) {
        joint_commands_[resource].type = JointCmd::Type::kNone;
        joint_commands_[resource].data = 0;
      }
    }
  }

  // Set command type for joints claimed by starting controllers
  for (const auto& controller : start_list) {
    for (const auto& claimed : controller.claimed_resources) {
      for (const auto& resource : claimed.resources) {
        if (claimed.hardware_interface == "hardware_interface::PositionJointInterface") {
          joint_commands_[resource].type = JointCmd::Type::kPos;
          joint_commands_[resource].data = joint_states_[resource].pos;
        } else if (claimed.hardware_interface == "hardware_interface::VelocityJointInterface") {
          joint_commands_[resource].type = JointCmd::Type::kVel;
          joint_commands_[resource].data = 0.0;
        } else if (claimed.hardware_interface == "hardware_interface::EffortJointInterface") {
          joint_commands_[resource].type = JointCmd::Type::kEff;
          joint_commands_[resource].data = 0.0;
        } else if (claimed.hardware_interface == "hardware_interface::VoltageJointInterface") {
          joint_commands_[resource].type = JointCmd::Type::kVolt;
          joint_commands_[resource].data = 0.0;
        }
      }
    }
  }
}

void FRCRobotHW::read(const ros::Time& /*time*/, const ros::Duration& /*period*/) {
  ROS_INFO_THROTTLE_NAMED(1, name_, "Reading Data - Overload me!");
}

void FRCRobotHW::write(const ros::Time& /*time*/, const ros::Duration& /*period*/) {
  ROS_INFO_THROTTLE_NAMED(1, name_, "Writing Data - Overload me!");
}

}  // namespace frc_robot_hw

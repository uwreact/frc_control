# General Methodology of ROS-WPI Integration

We wish to provide full coverage of the tools in the WPILib within ROS. As
such, we have a parser designed to read all possible sensors and actuators,
create appropriate `State` and (when applicable) `Command` interfaces, and
publish all data in standard ROS formats. That is, use a `JointStateController`
to publish all data in a manner that can easily be parsed into a TF tree.

# Custom State and Commands

`ros_control` and `ros_controllers` provide powerful tools for working with
powerful, smart actuators, typically with closed-loop control and built-in
feedback. In FRC, the majority of controllers are much simpler (with the most
prominent exception being CTRE's TalonSRX). As such, we have created a set of
custom `hardware_interfaces` (handles and interfaces) for these simpler sensors
and actuators:

 - `AnalogState` and `AnalogCommand`:
    These interfaces contain a single `double` field
 - `BinaryState` and `BinaryCommand`:
    These interfaces contain a single `boolean` field
 - `TernaryState` and `TernaryCommand`:
    These interfaces contain a tri-state enum
 - `PDPState`:
    This interface contains the state of a PDP (voltage, currents, etc)

# URDF Format

The URDF is used to model and describe a robot by specifying with its joints,
links (members), and sensors. However, the sensor format is poorly defined and
not widely used. As such, we have described here how `frc_control` expects
sensors and joints in URDFs to be organized:

Joints must be of type 'revolute', 'continuous', or 'prismatic'.
'floating' and 'planar' joints are not supported.
'fixed' joints are treated as solid bodies, and are therefore allowed but
should not have an actuator or sensor associated with them.

Sensors must be defined within a joint, as shown below. The sensor measures
the joint that it inhabits. In the case below, the `shoulder_joint_pot` is
a potentiometer that measures the position of the `shoulder_joint`. Note that
this joint may or may not be driven by an actuator; both cases are valid. The
type of the sensor is not currently used, and can be anything (or even ommitted).
**TODO: Discuss Gazebo sensors.**

    <joint name="shoulder_joint" type="revolute">
      <sensor name="shoulder_joint_pot" type="analog"/>
      <parent link="torso"/>
      <child link="arm"/>
      <limit lower="0" upper="1.57" effort="60" velocity="100"/>
    </joint>

Some sensors may be used to measure things that are not joints on the robot.
For example, ultrasonic range finders and beam break sensors are used to
sense the environment rather than the robot. For these sensors, it is
appropriate to instead place them within a link, as shown below.

    <link name="imu">
      <sensor name="main_imu" type="imu"/>
      <!-- etc -->
    </link>

**Note:** This format is NOT URDF compliant. Perhaps we should change this.
Typically, when placing a sensor, the `<sensor>` tag would not be within
the `<link>` tag. Instead, the sensor would contain a `<parent>` tag, as
shown below.

    <sensor name="main_imu" type="imu">
      <parent link="chassis"/>
    </sensor>

However, since URDF does not define a method for applying sensors to joints,
we created our own format for defining joints. We could have been more URDF
compliant by specifying either `<parent link=""/>` or `<parent joint="">`,
but we found this to be confusing and lead to mix ups. That said, this model
is very much open to discussion.

# YAML Format

Once a URDF model of the robot has been created, the joints and sensors are
defined in a YAML file. The exact syntax of each possible actuator and sensor
are defined below, however they all take the following general form:

    joints:
      - {name: shoulder_joint,      type: talon,        id: 1}
      - {name: shoulder_joint_pot,  type: analog_input, ain_channel: 1}

Note a few key attributes:
 - Confusingly, both sensors and actuators are refered to here as 'joints'.
    If you have a better name, please let me know because I really dislike this
 - Every entry **MUST** have a 'name' field, matching its associated URDF
    joint or sensor
 - Every entry **MUST** have a 'type' field, matching its standard WPILib
    object name (in `snake_case`, eg. talon, double_solenoid, navx)
 - Additional fields are defined for each object type, as described below

Currently, the standard joint sensors (Encoder, AnalogInput, & DigitalInput)
take an additional `joint` parameter, containing the name of the URDF joint
that encompasses them. However, this will be changed to instead be loaded
from the URDF directly.

# Driven vs Passive Joints

Each joint must have a maximum of one single `JointStateHandle`. If a joint
is driven (Eg. Speed controllers, solenoids), this is easy; When we create
the actuator, we also create the `JointStateHandle`. Any sensors that
describe this joint store their data in a different location, for example a
`RateState` (Pos and Vel) for an encoder and the `PDPState` for current. Then,
after reading from the sensors, the sensor data is used to populate the
`JointStateHandle`. From the previous example, the encoder would be used to
populate the `JointState`'s position and velocity, while the PDP's current
measurement would be scaled to N*m to populate the `JointState`'s effort. Note
that it is not required for all of the position, velocity, and effort to be
populated, though it is ideal.

If, however, a joint is *not* driven, we still have to create a
`JointStateHandle` but we cannot rely on the actuator to create it, since there
is no actuator. Instead, when we create the sensor, we check to see if a
`JointStateHandle` has been registered for the sensor's parent yet. If not, we
then create the `JointStateHandle`, and populate it in the same way as before;
Data is stored in independant handles, and merged into the `JointStateHandle`
at once to create a fully-defined `JointState`.

**TODO: RateStates != AnalogStates. We should either add a rate field to
AnalogState, or create an additional interface**

# Supported WPILib & Vendor sensors (alphabetical)

## 1. Analog Input
 - The state of the sensor is stored as a `RateState` (Pos and Vel), with
    an `AnalogStateHandle` handle pointing to the position.
 - If the sensor is linked to a joint, the `RateState` is transformed
    into a `JointState` each update by setting
    `JointState::pos = RateState::state` and
    `JointState::vel = RateState::rate`. Note that the effort is not set.
#### YAML Syntax:
Key|Type|Required?|Value/Description
---|----|---------|-----------------
`type`|string|required|Must be 'analog_input'
`ain_channel`|int|required|The analog input channel of the sensor
`scale`|float|required|The full-scale value of the sensor (Usually rad or m)
`offset`|float|required|The offset value of the sensor (Usually rad or m)
||||**TODO** Allow configuration of bits and accumulator

## 2. Digital Input
 - The state of the sensor is stored with a `BinaryStateHandle`
 - If the sensor is linked to a joint, the `BinaryState`
    is transformed into a `JointState` each update by setting the
    joint's position to either the max or min limit of the joint.
    Note that the velocity and effort will both be 0.
#### YAML Syntax:
Key|Type|Required?|Value/Description
---|----|---------|-----------------
`type`|string|required|Must be 'digital_input'
`dio_channel`|int|required|The DIO channel of the sensor
`inverted`|bool|optional, default=`false`|Whether the input signal should be inverted

## 3. Encoder
 - The state of the sensor is stored with a `RateState` (Pos and Vel), with
    an `AnalogStateHandle` pointing to the position.
 - If the sensor is linked to a joint, the `RateState`
    is transformed into a `JointState` each update by setting
    `JointState::pos = RateState::state` and
    `JointState::vel = RateState::rate`. Note that the effort is not set.
#### YAML Syntax:
Key|Type|Required?|Value/Description
---|----|---------|-----------------
`type`|string|required|Must be 'encoder'
`ch_a`|int|required|The DIO channel of signal A
`ch_b`|int|required|The DIO channel of signal B
`dist_per_pulse`|float|required|The distance travelled on each tick of the encoder, (Usually rad or m)
`inverted`|bool|optional, default=`false`|Whether the direction is inverted
`encoding`|int|optional, default=`4`|The sampling mode. Must be `1`, `2`, or `4`. See [ScreenStepsLive](https://wpilib.screenstepslive.com/s/currentCS/m/cpp/l/241875-encoders-measuring-rotation-of-a-wheel-or-other-shaft).

## 4. CTRE - PigeonIMU
 - The state of the IMU is stored with an `IMUSensorHandle`
 - **Note:** Either `id` or `talon` MUST be specified! You may NOT specify both!
#### YAML Syntax:
Key|Type|Required?|Value/Description
---|----|---------|-----------------
`type`|string|required|Must be 'pigeon_imu'
`frame_id`|string|required|The tf2 frame of the sensor. **TODO**
`id`|int|optional|Used if the `PigeonIMU` is connected to the CAN bus. The CAN ID of the sensor.
`talon`|string|optional|Used if the `PigeonIMU` is piggybacking on a TalonSRX. The name of the attached TalonSRX.


## 4. Kauai Labs - navX-MXP IMU
 - The state of the IMU is stored with an `IMUSensorHandle`
#### YAML Syntax:
Key|Type|Required?|Value/Description
---|----|---------|-----------------
`type`|string|required|Must be 'navx'
`frame_id`|string|required|The tf2 frame of the sensor. **TODO**
`interface`|string|required|The interface used to connect to the navX. Must be one of 'spi', 'serial', or 'i2c'
`id`|int|required|The ID of the device on the specified interface

# Supported WPILib & Vendor actuators (alphabetical)

## 1. Analog Output
 - The state of the output (last commanded value) is stored as a `RateState`
    (Pos and Vel), with an `AnalogStateHandle` handle pointing to the position.
 - The next command is stored as an `AnalogCommandHandle`
 - The `RateState` is transformed into a `JointState` each update by setting
    `JointState::pos = RateState::state` and
    `JointState::vel = RateState::rate`. Note that the effort is not set.
#### YAML Syntax:
Key|Type|Required?|Value/Description
---|----|---------|-----------------
`type`|string|required|Must be 'analog_output'
`aout_channel`|int|required|The analog output channel of the output
`scale`|float|required|The full-scale value of the output (Usually rad or m)
`offset`|float|required|The offset value of the output (Usually rad or m)

## 2. Digital Output
 - The state of the output (last commanded value) is stored with a
     `BinaryStateHandle`
 - The next command is stored with a `BinaryCommandHandle`
 - The `BinaryState` is transformed into a `JointState` each update by setting
    the joint's position to either the max or min limit of the joint.
    Note that the velocity and effort will both be 0.
#### YAML Syntax:
Key|Type|Required?|Value/Description
---|----|---------|-----------------
`type`|string|required|Must be 'digital_output'
`dio_channel`|int|required|The DIO channel of the output
`inverted`|bool|optional, default=`false`|Whether the output should be inverted

## 3. Double Solenoid
 - The state of the output (last commanded value) is stored with a
     `TernaryStateHandle`
 - The next command is stored with a `TernaryCommandHandle`
 - The `TernaryState` is transformed into a `JointState` each update by setting
    the joint's position to either the max or min limit of the joint.
#### YAML Syntax:
Key|Type|Required?|Value/Description
---|----|---------|-----------------
`type`|string|required|Must be 'double_solenoid'
`forward_id`|int|required|The PCM channel of the forward direction
`reverse_id`|int|required|The PCM channel of the reverse direction
`pcm_id`|int|optional, default=`0`|The CAN ID of the PCM that the solenoid is connected to. Currently, the forward and reverse IDs must both be on the same PCM.

## 4. Relay
 - **Note: Relays are currently poorly supported!**
 - **Note: Relays used to control two separate actuators are NOT supported,
    as this is an abuse of hardware**
 - The state of the output (last commanded value) is stored with a
     `TernaryStateHandle`
 - The next command is stored with a `TernaryCommandHandle`
 - The `TernaryState` is NOT transformed into a `JointState`! This can
    lead to incomplete tf trees.
 - **Potential future solutions**: Use the URDF to try to determine what
    the `Relay` is controlling. If a prismatic or revolute joint, set
    position/velocity?
#### YAML Syntax:
Key|Type|Required?|Value/Description
---|----|---------|-----------------
`type`|string|required|Must be 'relay'
`relay_id`|int|required|The relay channel
`direction`|string|required|'both', 'forward', or 'reverse'

## 5. Servo
 - The state of the output (last commanded value) is stored with a
    `JointStateHandle`. Note that only the position is populated.
 - The next command is stored with a `JointHandle` registered on a
   `PositionJointInterface`
#### YAML Syntax:
Key|Type|Required?|Value/Description
---|----|---------|-----------------
`type`|string|required|Must be 'servo'
`id`|int|required|The PWM channel
||||**TODO** Get servo parameters like range, max/min, etc

## 6. Solenoid
 - The state of the output (last commanded value) is stored with a
     `BinaryStateHandle`
 - The next command is stored with a `BinaryCommandHandle`
 - The `BinaryState` is transformed into a `JointState` each update by setting
    the joint's position to either the max or min limit of the joint.
#### YAML Syntax:
Key|Type|Required?|Value/Description
---|----|---------|-----------------
`type`|string|required|Must be 'solenoid'
`id`|int|required|The PCM channel of the solenoid
`pcm_id`|int|optional, default=`0`|The CAN ID of the PCM that the solenoid is connected to

## 7. Speed Controllers

### 7.1 Simple Speed Controller (PWM & no-feedback CAN)
 - This includes:
   - `DMC60` (`type` = 'dmc60')
   - `Jaguar` (`type` = 'jaguar')
   - `PWMTalonSRX` (`type` = 'pwm_talon_srx')
   - `PWMVictorSPX` (`type` = 'pwm_victor_spx')
   - `SD540` (`type` = 'sd540')
   - `Spark` (`type` = 'spark')
   - `Talon` (`type` = 'talon')
   - `Victor` (`type` = 'victor')
   - `VictorSP` (`type` = 'victor_sp')
   - `NidecBrushless` (`type` = 'nidec') ** See 4.2 for details
   - `CANVictorSPX` (`type` = 'can_victor_spx') ** Only if CTRE is enabled
 - The state is stored in a `JointState`. However, since these speed
    controllers offer no feedback, the controller itself does not populate
    the data in this handle. Instead, the data from other sensors is
    aggregated as required to fill out the `JointState`. For example, it
    is common for an `Encoder` and a `PDP channel` to both be used in tandem
    to populate the position, velocity, and effort of the joint.
 - The command is stored in a series of `JointHandle`s, registered to different
     interfaces for different control methods.
    - For position-based control, the handle is registered to a
    `PositionJointInterface`
    - For velocity-based control, the handle is registered to a
    `VelocityJointInterface`
    - For effort-based control, the handle is registered to a
    `EffortJointInterface`
    - For voltage-based control, the handle is registered to a
    `VoltageJointInterface`
 - In many robotic systems, the closed-loop feedback is performed by the
    motor controller itself, allowing commands of position, velocity, or
    effort to be passed directly to the hardware controller. Since these
    controllers are more primitive, we must move this low-level feedback loop
    up to main controller itself, namely the roboRIO.
 - For all control modes other than voltage, we require an additional PID
    controller be specified to drive the motor with the specified command.
    **TODO:** This isn't yet implemented, meaning only voltage control is
    currently available.
 - **Current->Effort scaling:**
    - Current is proportional to effort (either force or torque), allowing us
      to use the PDP along with the robot geometry/gearing to get effort
      feedback for each motor.
    - **TODO:** Link to doc explaining how to get current-torque relationship.
      VEX motor data is very useful, unlike stall torque. Simply divide torque by
      current. Then, multiply by the gear reduction (eg. A reduction of 100:1
      would result in a multiplication by 100. A gearing of 1:100 would result in
      a multiplication of 1/100). The result is `effort = current * k_eff`, where
    `k_eff` accounts for both motor type and gear reduction.
    - `k_eff` should have units of N/A or N*m/A

#### YAML Syntax:
Key|Type|Required?|Value/Description
---|----|---------|-----------------
`type`|string|required|See above
`id`|int|required|The PWM or CAN channel of the motor controller
`inverted`|bool|optional, default=`false`|Whether to invert the direction of the motor
`pdp`|string|optional, default='none'|The name of the PDP powering this motor controller **TODO: Default to PDP ID0 rather than none**
`pdp_ch`|int|optional|The channel of the PDP powering this motor controller
`k_eff`|float|optional, default=`1.0`|The current to effort scaling


### 7.1.1 Nidec Brushless
 - The `NidecBrushless` is identical to the simple speed controllers described
   above, but takes and additional YAML parameter:
   - `dio_channel` (int, required): The digital input channel of the Nidec

### 7.2 Smart Speed Controller (CAN)
 - **TODO**

# Supported special WPILib objects (alphabetical)
These objects do not specify a URDF sensor; they are YAML only.

## 1. Compressor
 - The current state is stored in a `BinaryStateHandle`, tracking whether the
    compressor is in closed-loop control or not. **TODO: Implement custom
    state, as with the PDP, to also track current pressure switch and
    current draw**
 - The next command (enable/disable closed-loop control) is stored in a
    `BinaryCommandHandle`
#### YAML Syntax:
Key|Type|Required?|Value/Description
---|----|---------|-----------------
`type`|string|required|Must be 'compressor'
`pcm_id`|int|optional, default=`0`|The CAN ID of the PCM that the solenoid is connected to.

## 2. Power Distribution Panel
 - The current state is stored in a `PDPStateHandle`, tracking:
    - Battery/bus voltage
    - Temperature
    - Total current draw
    - Total power draw
    - Total energy use
    - Current draw on each of the 16 channels
 - NOTE: `frc_control` is set up to handle multiple PDPs even though
    FRC-compliant robots may only use a single PDP. Although the FRC manual
    requires only one PDP be used, the WPI HAL theoretically allows multiple
    PDPs so we will support this use case for non-FRC-compliant robots using
    the FRC control system.
#### YAML Syntax:
Key|Type|Required?|Value/Description
---|----|---------|-----------------
`type`|string|required|Must be 'pdp'
`id`|int|required|The CAN ID of the PDP (Usually 0)

# A note on vendor libraries
**TODO**

# Other notes
 - Currently, all AnalogOutput and DigitalOutputs will create a `JointState`,
    even if they aren't used to affect the robot's pose (eg. LEDs). This
    should be fixed.
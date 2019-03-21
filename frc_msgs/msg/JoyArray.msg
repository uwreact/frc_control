# Corresponds to definitions in DriverStationTypes.h
uint8 MAX_JOYSTICK_AXES = 12
uint8 MAX_JOYSTICK_POVS = 12
uint8 MAX_JOYSTICK_BUTTONS = 32
uint8 MAX_JOYSTICKS = 6

# Corresponds to frc::GenericHID::HIDType
int8 HID_UNKNOWN = -1
int8 HID_XINPUT_UNKNOWN = 0
int8 HID_XINPUT_GAMEPAD = 1
int8 HID_XINPUT_WHEEL = 2
int8 HID_XINPUT_ARCADE_STICK = 3
int8 HID_XINPUT_FLIGHT_STICK = 4
int8 HID_XINPUT_DANCE_PAD = 5
int8 HID_XINPUT_GUITAR = 6
int8 HID_XINPUT_GUITAR2 = 7
int8 HID_XINPUT_DRUM_KIT = 8
int8 HID_XINPUT_GUITAR3 = 11
int8 HID_XINPUT_ARCADE_PAD = 19
int8 HID_JOYSTICK = 20
int8 HIT_GAMEPAD = 21
int8 HID_DRIVING = 22
int8 HID_FLIGHT = 23
int8 HID_1ST_PERSON = 24

# Note: We will treat POVs as both buttons and axes
# They do not count towards the axis limits
Header header
sensor_msgs/Joy[] sticks
int8[] types
string[] names

# Corresponds to frc::RobotController
Header header
int32 fpga_version
int64 fpga_revision
uint64 fpga_time
bool user_button
bool sys_active
bool browned_out
float64 input_voltage
float64 input_current
float64 voltage_3v3
float64 current_3v3
bool enabled_3v3
int32 fault_count_3v3
float64 voltage_5v
float64 current_5v
bool enabled_5v
int32 fault_count_5v
float64 voltage_6v
float64 current_6v
bool enabled_6v
int32 fault_count_6v
CANStatus can_status

# Corresponds to frc::DriverStation::GetBatteryVoltage()
float64 battery_voltage

Header header

bool    closed_loop     # If closed loop control of the compressor is enabled
bool    enabled         # If the compressor output is active
bool    pressure_switch # If the pressure switch is triggered (pressure is low)
float64 current         # How much current the compressor is drawing

bool fault_current_too_high # If the compressor output has been disabled due to high current draw
bool fault_shorted          # If the compressor output has been disabled due to a short circuit
bool fault_not_connected    # If the compressor output does not appear to be wired

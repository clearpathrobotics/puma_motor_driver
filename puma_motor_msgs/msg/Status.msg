Header header

# Duty cycle of the h-bridge driver (-1..1)
float32 duty_cycle

# Input terminal voltage (volts).
float32 bus_voltage

# Current consumed (amps).
float32 current

# Internal driver temperature (degC).
float32 temperature

# Travel as measured by the encoder (rad).
float32 travel

# Angular velocity as measured by the encoder (rad/s).
float32 speed

# Voltage as output to the motor (volts).
float32 output_voltage

# Value of the auxiliary ADC (volts).
float32 analog_input

# Fault states which could cause the driver to be immobilized.
uint8 FAULT_CURRENT=1
uint8 FAULT_TEMPERATURE=2
uint8 FAULT_BUS_VOLTAGE=4
uint8 FAULT_BRIDGE_DRIVER=8
uint8 fault

# Available control modes, not all of which are broken out to
# this ROS driver.
uint8 MODE_VOLTAGE=1
uint8 MODE_CURRENT=2
uint8 MODE_SPEED=4
uint8 MODE_POSITION=8
uint8 MODE_VCOMP=16
uint8 mode

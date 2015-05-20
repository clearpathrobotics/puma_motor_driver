# Number on the bus (CAN ID)
uint8 device_number

# Name of joint controlled, or other identifier.
string device_name

# Duty cycle of the h-bridge driver (-1..1)
float32 duty_cycle

# Current consumed (amps).
float32 current

# Travel as measured by the encoder (rad).
float64 travel

# Angular velocity as measured by the encoder (rad/s).
float64 speed

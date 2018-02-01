# Quadrotor status message

# battery state enums
uint8 INVALID=0
uint8 GOOD=1
uint8 LOW=2
uint8 CRITICAL=3

# control mode enums
uint8 RATE_MODE=0
uint8 ATTITUDE_MODE=1

Header header

float32 battery_voltage
uint8 battery_state
uint8 control_mode

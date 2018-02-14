# battery state enums
uint8 INVALID=0
uint8 GOOD=1
uint8 LOW=2
uint8 CRITICAL=3

# control mode enums as defined in ControlCommand.msg
uint8 NONE=0
uint8 ATTITUDE=1
uint8 BODY_RATES=2
uint8 ANGULAR_ACCELERATION=3
uint8 ROTOR_THRUSTS=4
# Additionally to the control command we want to know whether an RC has taken
# over from the low level feedback
uint8 RC_MANUAL=10

Header header

# Battery information
float32 battery_voltage
uint8 battery_state

# Control mode as defined above
uint8 control_mode

# Motor speed feedback [rpm]
int16[] motor_speeds

# Thrust mapping coefficients
# thrust = thrust_mapping_coeffs[2] * u^2 + thrust_mapping_coeffs[1] * u +
#     thrust_mapping_coeffs[0]
float64[] thrust_mapping_coeffs

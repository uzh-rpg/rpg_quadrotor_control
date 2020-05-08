# Status message from the PX4

# commander state enums
uint8 LANDED=0
uint8 AUTONOMOUS_FLYING=1
uint8 MANUAL_FLYING=2
uint8 EMERGENCY_LANDING=3

# battery state enums
uint8 INVALID=0
uint8 GOOD=1
uint8 LOW=2
uint8 CRITICAL=3

        
# control mode enums
uint8 RATE_MODE=0
uint8 ATTITUDE_MODE=1

Header header

uint8 commander_state
uint8 battery_state
uint8 control_mode
float64 battery_voltage
float32 pcb_temperature

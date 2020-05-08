#Message type handling onboard parameters of the PX4

Header header

#Onboard parameter id (char[16])
string param_id

#Value of the parameter
float64 param_value

#Onboard parameter type according to the mavlink message: MAV_PARAM_TYPE
uint8 param_type

#Total number of onboard parameters
uint16 param_count

#Index of this onboard parameter
uint16 param_index
